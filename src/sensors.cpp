#include "sensors.h"
#include <Wire.h>
#include <math.h>
#include <pico/critical_section.h>
#include <string.h>

namespace Sensors {

namespace {

SensorData currentSensorData{};
SensorData publishedSensorData{};
critical_section_t stateLock;
bool stateLockInitialized = false;

uint8_t imuAddress = Config::MPU6050_ADDR_PRIMARY;
uint8_t pressureAddress = 0;
uint16_t pressureCoefficients[8]{};

uint32_t rawPressure = 0;
uint32_t rawTemperature = 0;
uint32_t pressureConversionStartedAt = 0;
uint32_t lastPressureCycleAt = 0;
bool pressureConversionPending = false;
bool pressureConversionIsPressure = true;

bool attitudeInitialized = false;
uint32_t previousImuReadAt = 0;
uint32_t lastImuRetryAt = 0;
uint32_t lastPressureRetryAt = 0;

constexpr uint32_t SENSOR_RETRY_INTERVAL_MS = 1500;
constexpr uint32_t MS5803_CONVERSION_TIME_MS = 10;
constexpr uint8_t MS5803_CMD_RESET = 0x1E;
constexpr uint8_t MS5803_CMD_D1_4096 = 0x48;
constexpr uint8_t MS5803_CMD_D2_4096 = 0x58;

bool probe(uint8_t address) {
  Wire1.beginTransmission(address);
  return Wire1.endTransmission() == 0;
}

bool writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire1.beginTransmission(address);
  Wire1.write(reg);
  Wire1.write(value);
  return Wire1.endTransmission() == 0;
}

bool readRegisters(uint8_t address, uint8_t reg, uint8_t* values, size_t length) {
  Wire1.beginTransmission(address);
  Wire1.write(reg);
  if (Wire1.endTransmission(false) != 0 || Wire1.requestFrom(address, length) != length) {
    return false;
  }
  for (size_t i = 0; i < length; ++i) {
    if (!Wire1.available()) return false;
    values[i] = static_cast<uint8_t>(Wire1.read());
  }
  return true;
}

int16_t readSignedBigEndian(const uint8_t* bytes) {
  return static_cast<int16_t>((static_cast<uint16_t>(bytes[0]) << 8) | bytes[1]);
}

bool sendCommand(uint8_t address, uint8_t command) {
  Wire1.beginTransmission(address);
  Wire1.write(command);
  return Wire1.endTransmission() == 0;
}

bool readAdc(uint32_t& result) {
  uint8_t bytes[3]{};
  if (!readRegisters(pressureAddress, 0x00, bytes, sizeof(bytes))) {
    return false;
  }
  result = (static_cast<uint32_t>(bytes[0]) << 16) |
           (static_cast<uint32_t>(bytes[1]) << 8) | bytes[2];
  return true;
}

uint8_t calculatePromCrc4(const uint16_t* coefficients) {
  uint16_t prom[8];
  memcpy(prom, coefficients, sizeof(prom));
  prom[7] &= 0xFF00;

  uint16_t remainder = 0;
  for (uint8_t byteIndex = 0; byteIndex < 16; ++byteIndex) {
    remainder ^= (byteIndex & 1)
        ? static_cast<uint16_t>(prom[byteIndex >> 1] & 0x00FF)
        : static_cast<uint16_t>(prom[byteIndex >> 1] >> 8);
    for (uint8_t bit = 0; bit < 8; ++bit) {
      remainder = (remainder & 0x8000)
          ? static_cast<uint16_t>((remainder << 1) ^ 0x3000)
          : static_cast<uint16_t>(remainder << 1);
    }
  }
  return static_cast<uint8_t>((remainder >> 12) & 0x0F);
}

bool initializePressureAt(uint8_t address) {
  if (!probe(address) || !sendCommand(address, MS5803_CMD_RESET)) {
    return false;
  }
  delay(3);

  uint16_t coefficients[8]{};
  for (uint8_t index = 0; index < 8; ++index) {
    uint8_t bytes[2]{};
    if (!readRegisters(address, static_cast<uint8_t>(0xA0 + index * 2), bytes, sizeof(bytes))) {
      return false;
    }
    coefficients[index] = (static_cast<uint16_t>(bytes[0]) << 8) | bytes[1];
  }
  if (calculatePromCrc4(coefficients) != (coefficients[7] & 0x0F)) {
    return false;
  }
  memcpy(pressureCoefficients, coefficients, sizeof(pressureCoefficients));
  pressureAddress = address;
  return true;
}

bool initializePressureSensor() {
  if (initializePressureAt(Config::MS5803_ADDR_HIGH) ||
      initializePressureAt(Config::MS5803_ADDR_LOW)) {
    currentSensorData.pressureReady = true;
    return true;
  }
  currentSensorData.pressureReady = false;
  return false;
}

void calculatePressureAndTemperature() {
  const int64_t deltaTemperature = static_cast<int64_t>(rawTemperature) -
      (static_cast<int64_t>(pressureCoefficients[5]) << 8);
  int64_t temperature = 2000 +
      ((deltaTemperature * pressureCoefficients[6]) >> 23);
  int64_t offset = (static_cast<int64_t>(pressureCoefficients[2]) << 16) +
      ((static_cast<int64_t>(pressureCoefficients[4]) * deltaTemperature) >> 7);
  int64_t sensitivity = (static_cast<int64_t>(pressureCoefficients[1]) << 15) +
      ((static_cast<int64_t>(pressureCoefficients[3]) * deltaTemperature) >> 8);

  int64_t temperatureCorrection = 0;
  int64_t offsetCorrection = 0;
  int64_t sensitivityCorrection = 0;
  if (temperature < 2000) {
    temperatureCorrection = 3 * ((deltaTemperature * deltaTemperature) >> 33);
    const int64_t temperatureDelta = temperature - 2000;
    offsetCorrection = 3 * temperatureDelta * temperatureDelta / 2;
    sensitivityCorrection = 5 * temperatureDelta * temperatureDelta / 8;
    if (temperature < -1500) {
      const int64_t coldDelta = temperature + 1500;
      offsetCorrection += 7 * coldDelta * coldDelta;
      sensitivityCorrection += 4 * coldDelta * coldDelta;
    }
  } else {
    temperatureCorrection = 7 * (deltaTemperature * deltaTemperature) / (1LL << 37);
    const int64_t temperatureDelta = temperature - 2000;
    offsetCorrection = temperatureDelta * temperatureDelta / 16;
  }

  temperature -= temperatureCorrection;
  offset -= offsetCorrection;
  sensitivity -= sensitivityCorrection;
  const int64_t pressure =
      (((sensitivity * rawPressure) / (1LL << 21)) - offset) / (1LL << 15);

  currentSensorData.temperatureC = static_cast<float>(temperature) / 100.0f;
  currentSensorData.pressureMbar = static_cast<float>(pressure) / 10.0f;
}

bool updatePressureSensor(uint32_t now) {
  if (!currentSensorData.pressureReady) {
    return false;
  }

  if (pressureConversionPending) {
    if (now - pressureConversionStartedAt < MS5803_CONVERSION_TIME_MS) {
      return false;
    }
    uint32_t conversionResult = 0;
    if (!readAdc(conversionResult)) {
      currentSensorData.pressureReady = false;
      pressureConversionPending = false;
      return true;
    }
    if (pressureConversionIsPressure) {
      rawPressure = conversionResult;
      if (!sendCommand(pressureAddress, MS5803_CMD_D2_4096)) {
        currentSensorData.pressureReady = false;
        pressureConversionPending = false;
        return true;
      }
      pressureConversionIsPressure = false;
      pressureConversionStartedAt = now;
      return false;
    }

    rawTemperature = conversionResult;
    calculatePressureAndTemperature();
    pressureConversionPending = false;
    pressureConversionIsPressure = true;
    lastPressureCycleAt = now;
    return true;
  }

  if (now - lastPressureCycleAt < 50) {
    return false;
  }
  if (!sendCommand(pressureAddress, MS5803_CMD_D1_4096)) {
    currentSensorData.pressureReady = false;
    return true;
  }
  pressureConversionIsPressure = true;
  pressureConversionPending = true;
  pressureConversionStartedAt = now;
  return false;
}

void initializeImu() {
  if (probe(Config::MPU6050_ADDR_PRIMARY)) {
    imuAddress = Config::MPU6050_ADDR_PRIMARY;
  } else if (probe(Config::MPU6050_ADDR_SECONDARY)) {
    imuAddress = Config::MPU6050_ADDR_SECONDARY;
  } else {
    currentSensorData.imuReady = false;
    return;
  }

  uint8_t identity = 0;
  if (readRegisters(imuAddress, 0x75, &identity, 1) && (identity == 0x68 || identity == 0x72 || identity == 0x70) &&
      writeRegister(imuAddress, 0x6B, 0x00) &&  // Wake up
      writeRegister(imuAddress, 0x1A, 0x03) &&  // DLPF 44Hz
      writeRegister(imuAddress, 0x1B, 0x00) &&  // Gyro ±250 dps
      writeRegister(imuAddress, 0x1C, 0x00)) {  // Accel ±2g
    currentSensorData.imuReady = true;
    attitudeInitialized = false;
  } else {
    currentSensorData.imuReady = false;
  }
}

void updateImu(uint32_t now) {
  if (!currentSensorData.imuReady) {
    return;
  }

  uint8_t bytes[14]{};
  if (!readRegisters(imuAddress, 0x3B, bytes, sizeof(bytes))) {
    currentSensorData.imuReady = false;
    return;
  }

  currentSensorData.accelG[0] = readSignedBigEndian(&bytes[0]) / 16384.0f;
  currentSensorData.accelG[1] = readSignedBigEndian(&bytes[2]) / 16384.0f;
  currentSensorData.accelG[2] = readSignedBigEndian(&bytes[4]) / 16384.0f;

  currentSensorData.gyroDps[0] = readSignedBigEndian(&bytes[8]) / 131.0f;
  currentSensorData.gyroDps[1] = readSignedBigEndian(&bytes[10]) / 131.0f;
  currentSensorData.gyroDps[2] = readSignedBigEndian(&bytes[12]) / 131.0f;

  const float accelPitch = atan2f(-currentSensorData.accelG[0],
      sqrtf(currentSensorData.accelG[1] * currentSensorData.accelG[1] +
            currentSensorData.accelG[2] * currentSensorData.accelG[2])) * 180.0f / PI;
  const float accelRoll = atan2f(currentSensorData.accelG[1],
                                currentSensorData.accelG[2]) * 180.0f / PI;

  const float elapsedSeconds = previousImuReadAt == 0
      ? 0.02f
      : min(static_cast<float>(now - previousImuReadAt) / 1000.0f, 0.1f);
  previousImuReadAt = now;

  if (!attitudeInitialized) {
    currentSensorData.rawPitch = accelPitch;
    currentSensorData.rawRoll = accelRoll;
    attitudeInitialized = true;
  } else {
    currentSensorData.rawPitch =
        0.98f * (currentSensorData.rawPitch + currentSensorData.gyroDps[1] * elapsedSeconds) +
        0.02f * accelPitch;
    currentSensorData.rawRoll =
        0.98f * (currentSensorData.rawRoll + currentSensorData.gyroDps[0] * elapsedSeconds) +
        0.02f * accelRoll;
  }
}

void publishState() {
  if (!stateLockInitialized) return;
  critical_section_enter_blocking(&stateLock);
  publishedSensorData = currentSensorData;
  critical_section_exit(&stateLock);
}

}  // namespace

void begin() {
  critical_section_init(&stateLock);
  stateLockInitialized = true;

  if (!Wire1.setSDA(Config::I2C_SDA_PIN) || !Wire1.setSCL(Config::I2C_SCL_PIN)) {
    currentSensorData.i2cReady = false;
    publishState();
    return;
  }
  Wire1.begin();
  Wire1.setClock(Config::I2C_CLOCK_HZ);
  currentSensorData.i2cReady = true;

  initializeImu();
  initializePressureSensor();
  publishState();
}

void update(const Config::Settings& settings) {
  const uint32_t now = millis();
  updatePressureSensor(now);
  updateImu(now);

  // Apply calibration offsets
  currentSensorData.pitch = currentSensorData.rawPitch - settings.pitchOffset;
  currentSensorData.roll = currentSensorData.rawRoll - settings.rollOffset;

  // Calculate depth if zero calibrated
  if (currentSensorData.pressureReady && settings.depthZeroCalibrated && settings.depthZeroPressureMbar > 0.0f) {
    const float deltaP = currentSensorData.pressureMbar - settings.depthZeroPressureMbar;
    const float depth = (deltaP * 10000.0f) / (settings.waterDensity * 9.80665f);
    currentSensorData.depthCm = (depth < 0.0f) ? 0.0f : depth;
  } else {
    currentSensorData.depthCm = 0.0f;
  }

  currentSensorData.lastUpdateMs = now;
  currentSensorData.sampleCount++;

  // Auto-retry offline sensors periodically
  if (!currentSensorData.imuReady && now - lastImuRetryAt >= SENSOR_RETRY_INTERVAL_MS) {
    lastImuRetryAt = now;
    initializeImu();
  }
  if (!currentSensorData.pressureReady && now - lastPressureRetryAt >= SENSOR_RETRY_INTERVAL_MS) {
    lastPressureRetryAt = now;
    initializePressureSensor();
  }

  publishState();
}

void snapshot(SensorData& dest) {
  if (!stateLockInitialized) {
    dest = currentSensorData;
    return;
  }
  critical_section_enter_blocking(&stateLock);
  dest = publishedSensorData;
  critical_section_exit(&stateLock);
}

bool calibrateDepthZero(Config::Settings& settings) {
  SensorData s{};
  snapshot(s);
  if (!s.pressureReady || s.pressureMbar <= 0.0f) {
    return false;
  }
  settings.depthZeroPressureMbar = s.pressureMbar;
  settings.depthZeroCalibrated = 1;
  return true;
}

bool calibrateImuLevel(Config::Settings& settings) {
  SensorData s{};
  snapshot(s);
  if (!s.imuReady) {
    return false;
  }
  settings.pitchOffset = s.rawPitch;
  settings.rollOffset = s.rawRoll;
  return true;
}

}  // namespace Sensors
