#pragma once

#include <Arduino.h>
#include <EEPROM.h>
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <string.h>

namespace Config {

constexpr uint32_t CONFIG_MAGIC = 0x41555631;  // 'AUV1'
constexpr uint8_t CONFIG_VERSION = 8;
constexpr size_t THRUSTER_COUNT = 6;
constexpr size_t AXIS_COUNT = 6;  // Surge, Sway, Heave, Yaw, Pitch, Roll

// Hardware Pins
constexpr uint8_t DEFAULT_THRUSTER_PINS[THRUSTER_COUNT] = {2, 3, 4, 5, 6, 7};
constexpr uint8_t I2C_SDA_PIN = 26;
constexpr uint8_t I2C_SCL_PIN = 27;
constexpr uint32_t I2C_CLOCK_HZ = 100000;

// Pulse Spectrum & Polarity (Standard Marine ESC: 1000-1500-2000 us)
constexpr uint16_t DEFAULT_PULSE_MIN_US = 1000;
constexpr uint16_t DEFAULT_PULSE_NEUTRAL_US = 1500;
constexpr uint16_t DEFAULT_PULSE_MAX_US = 2000;

// Direction Polarity: T1, T2 (CW) = 1; T3, T4 (CCW) = 0; T5 (CW dive) = 0; T6 (CCW dive) = 1
constexpr uint8_t DEFAULT_THRUSTER_REVERSE[THRUSTER_COUNT] = {1, 1, 0, 0, 0, 1};
constexpr float DEFAULT_THRUSTER_TRIM[THRUSTER_COUNT] = {1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f};

// Safety Limits & ESC Deadband
constexpr uint8_t DEFAULT_POWER_LIMIT_PERCENT = 25;  // Default 25% safety cap for bench testing
constexpr uint16_t DEFAULT_ESC_DEADBAND_US = 25;    // ESC deadband in microseconds (0-80)
constexpr float DEFAULT_DIFF_HORIZ_TRIM = 0.0f;     // Differential trim Port (T1, T3) vs Starboard (T2, T4) (-20 to +20%)
constexpr float DEFAULT_DIFF_VERT_TRIM = 0.0f;      // Differential trim T5 vs T6 (-20 to +20%)

// Auto-Run Timing Defaults
constexpr uint8_t DEFAULT_AUTORUN_TARE_SEC = 3;     // Surface depth tare (seconds before launch)
constexpr uint8_t DEFAULT_AUTORUN_LEVEL_SEC = 2;    // IMU level tare (seconds before launch)

// Sensor Addresses
constexpr uint8_t MPU6050_ADDR_PRIMARY = 0x68;
constexpr uint8_t MPU6050_ADDR_SECONDARY = 0x69;
constexpr uint8_t MS5803_ADDR_HIGH = 0x76;
constexpr uint8_t MS5803_ADDR_LOW = 0x77;

// Default PID Parameters (Responsive gains to break ESC deadband)
constexpr float DEFAULT_PID_DEPTH_KP = 0.60f;
constexpr float DEFAULT_PID_DEPTH_KI = 0.000f;
constexpr float DEFAULT_PID_DEPTH_KD = 0.015f;

constexpr float DEFAULT_PID_ROLL_KP = 0.80f;
constexpr float DEFAULT_PID_ROLL_KI = 0.000f;
constexpr float DEFAULT_PID_ROLL_KD = 0.008f;

constexpr float DEFAULT_PID_PITCH_KP = 0.80f;
constexpr float DEFAULT_PID_PITCH_KI = 0.000f;
constexpr float DEFAULT_PID_PITCH_KD = 0.008f;

// Mixer Matrix for 6-Thruster Vector Layout
// Row: T1 (Rear-Left), T2 (Front-Right), T3 (Front-Left), T4 (Rear-Right), T5 (Mid-Right Vert), T6 (Mid-Left Vert)
// Col: Surge, Sway, Heave, Yaw, Pitch, Roll
constexpr float DEFAULT_MIXER[THRUSTER_COUNT][AXIS_COUNT] = {
    { 1.0f, -1.0f,  0.0f, -1.0f,  0.0f,  0.0f}, // T1
    { 1.0f, -1.0f,  0.0f,  1.0f,  0.0f,  0.0f}, // T2
    { 1.0f,  1.0f,  0.0f, -1.0f,  0.0f,  0.0f}, // T3
    { 1.0f,  1.0f,  0.0f,  1.0f,  0.0f,  0.0f}, // T4
    { 0.0f,  0.0f,  1.0f,  0.0f,  0.0f,  1.0f}, // T5 (Right vertical)
    { 0.0f,  0.0f,  1.0f,  0.0f,  0.0f, -1.0f}  // T6 (Left vertical)
};

struct ThrusterConfig {
  uint8_t pin;
  uint16_t pulseMin;
  uint16_t pulseNeutral;
  uint16_t pulseMax;
  uint8_t reverse;
  float trim;
};

struct PidGains {
  float kp;
  float ki;
  float kd;
};

struct Settings {
  uint32_t magic;
  uint8_t version;
  uint8_t powerLimitPercent;

  ThrusterConfig thrusters[THRUSTER_COUNT];

  // PID Parameters
  PidGains pidDepth;
  PidGains pidRoll;
  PidGains pidPitch;

  // Calibration Offsets
  float pitchOffset;
  float rollOffset;
  float depthZeroPressureMbar;
  uint8_t depthZeroCalibrated;
  float waterDensity;  // kg/m3

  // Auto-Run Configuration
  uint8_t autoRunEnabled;   // 0 = disabled, 1 = enabled on boot
  uint8_t autoRunDelaySec;  // Countdown delay (3 to 60 seconds, default 10)
  uint8_t autoRunTareSec;   // Surface depth tare (default: 3)
  uint8_t autoRunLevelSec;  // IMU level tare (default: 2)

  // Actuator Trims & Deadband
  uint16_t escDeadbandUs;   // ESC deadband in microseconds (0-80, default 25)
  float diffHorizTrim;      // Differential trim Port (T1, T3) vs Starboard (T2, T4) (-20.0 to +20.0, default 0.0)
  float diffVertTrim;       // Differential trim T5 vs T6 (-20.0 to +20.0, default 0.0)

  uint8_t autoDepthTareOnBoot;  // Capture surface pressure at startup
  uint8_t autoImuLevelOnBoot;   // Capture level IMU offsets at startup

  uint32_t checksum;
};

inline bool validPidGains(const PidGains& gains) {
  constexpr float MAX_PID_GAIN = 100.0f;
  return isfinite(gains.kp) && isfinite(gains.ki) && isfinite(gains.kd) &&
      gains.kp >= 0.0f && gains.kp <= MAX_PID_GAIN &&
      gains.ki >= 0.0f && gains.ki <= MAX_PID_GAIN &&
      gains.kd >= 0.0f && gains.kd <= MAX_PID_GAIN;
}

inline bool validSettings(const Settings& s) {
  if (s.powerLimitPercent < 1 || s.powerLimitPercent > 100 ||
      !isfinite(s.waterDensity) || s.waterDensity < 800.0f || s.waterDensity > 1200.0f ||
      !isfinite(s.pitchOffset) || fabsf(s.pitchOffset) > 90.0f ||
      !isfinite(s.rollOffset) || fabsf(s.rollOffset) > 90.0f ||
      !isfinite(s.depthZeroPressureMbar) || s.depthZeroPressureMbar < 500.0f ||
      s.depthZeroPressureMbar > 2000.0f || s.depthZeroCalibrated > 1 ||
      s.autoRunEnabled > 1 || s.autoRunDelaySec < 3 || s.autoRunDelaySec > 60 ||
      s.autoRunTareSec < 1 || s.autoRunTareSec > s.autoRunDelaySec ||
      s.autoRunLevelSec < 1 || s.autoRunLevelSec > s.autoRunDelaySec ||
      s.autoDepthTareOnBoot > 1 || s.autoImuLevelOnBoot > 1 ||
      s.escDeadbandUs > 80 || !isfinite(s.diffHorizTrim) ||
      fabsf(s.diffHorizTrim) > 20.0f || !isfinite(s.diffVertTrim) ||
      fabsf(s.diffVertTrim) > 20.0f ||
      !validPidGains(s.pidDepth) || !validPidGains(s.pidRoll) ||
      !validPidGains(s.pidPitch)) {
    return false;
  }

  for (size_t i = 0; i < THRUSTER_COUNT; ++i) {
    const ThrusterConfig& t = s.thrusters[i];
    if (t.pin > 29 || t.pin == I2C_SDA_PIN || t.pin == I2C_SCL_PIN ||
        t.pulseMin < 800 || t.pulseMin > 1400 ||
        t.pulseNeutral < 1400 || t.pulseNeutral > 1600 ||
        t.pulseMax < 1600 || t.pulseMax > 2200 ||
        t.pulseMin >= t.pulseNeutral || t.pulseNeutral >= t.pulseMax ||
        t.reverse > 1 || !isfinite(t.trim) || t.trim < 0.1f || t.trim > 2.5f) {
      return false;
    }
    for (size_t j = 0; j < i; ++j) {
      if (s.thrusters[j].pin == t.pin) {
        return false;
      }
    }
  }
  return true;
}

inline uint32_t calculateChecksum(const Settings& s) {
  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&s);
  uint32_t hash = 2166136261u;
  for (size_t i = 0; i < offsetof(Settings, checksum); ++i) {
    hash = (hash ^ bytes[i]) * 16777619u;
  }
  return hash;
}

inline void loadDefaults(Settings& s) {
  memset(&s, 0, sizeof(s));
  s.magic = CONFIG_MAGIC;
  s.version = CONFIG_VERSION;
  s.powerLimitPercent = DEFAULT_POWER_LIMIT_PERCENT;

  for (size_t i = 0; i < THRUSTER_COUNT; ++i) {
    s.thrusters[i].pin = DEFAULT_THRUSTER_PINS[i];
    s.thrusters[i].pulseMin = DEFAULT_PULSE_MIN_US;
    s.thrusters[i].pulseNeutral = DEFAULT_PULSE_NEUTRAL_US;
    s.thrusters[i].pulseMax = DEFAULT_PULSE_MAX_US;
    s.thrusters[i].reverse = DEFAULT_THRUSTER_REVERSE[i];
    s.thrusters[i].trim = DEFAULT_THRUSTER_TRIM[i];
  }

  s.pidDepth.kp = DEFAULT_PID_DEPTH_KP;
  s.pidDepth.ki = DEFAULT_PID_DEPTH_KI;
  s.pidDepth.kd = DEFAULT_PID_DEPTH_KD;

  s.pidRoll.kp = DEFAULT_PID_ROLL_KP;
  s.pidRoll.ki = DEFAULT_PID_ROLL_KI;
  s.pidRoll.kd = DEFAULT_PID_ROLL_KD;

  s.pidPitch.kp = DEFAULT_PID_PITCH_KP;
  s.pidPitch.ki = DEFAULT_PID_PITCH_KI;
  s.pidPitch.kd = DEFAULT_PID_PITCH_KD;

  s.pitchOffset = 0.0f;
  s.rollOffset = 0.0f;
  s.depthZeroPressureMbar = 1013.25f;
  s.depthZeroCalibrated = 0;
  s.waterDensity = 1000.0f;

  s.autoRunEnabled = 0;
  s.autoRunDelaySec = 10;
  s.autoRunTareSec = DEFAULT_AUTORUN_TARE_SEC;
  s.autoRunLevelSec = DEFAULT_AUTORUN_LEVEL_SEC;
  s.autoDepthTareOnBoot = 0;
  s.autoImuLevelOnBoot = 0;

  s.escDeadbandUs = DEFAULT_ESC_DEADBAND_US;
  s.diffHorizTrim = DEFAULT_DIFF_HORIZ_TRIM;
  s.diffVertTrim = DEFAULT_DIFF_VERT_TRIM;

  s.checksum = calculateChecksum(s);
}

inline bool saveSettings(Settings& s) {
  if (!validSettings(s)) {
    return false;
  }
  s.magic = CONFIG_MAGIC;
  s.version = CONFIG_VERSION;
  s.checksum = calculateChecksum(s);
  EEPROM.begin(sizeof(Settings));
  EEPROM.put(0, s);
  return EEPROM.commit();
}

inline bool loadSettings(Settings& s) {
  EEPROM.begin(sizeof(Settings));
  EEPROM.get(0, s);
  if (s.magic == CONFIG_MAGIC && s.version == CONFIG_VERSION &&
      s.checksum == calculateChecksum(s)) {
    if (!validSettings(s)) {
      loadDefaults(s);
      return false;
    }
    return true;
  }

  // Automatic EEPROM Schema Migration (v7 -> v8)
  if (s.magic == CONFIG_MAGIC && s.version == 7) {
    const uint8_t calibrateOnBoot = s.autoDepthTareOnBoot;
    s.version = CONFIG_VERSION;
    s.autoDepthTareOnBoot = calibrateOnBoot;
    s.autoImuLevelOnBoot = calibrateOnBoot;
    saveSettings(s);
    return true;
  }

  // Automatic EEPROM Schema Migration (v6 -> v8)
  if (s.magic == CONFIG_MAGIC && s.version == 6) {
    s.version = CONFIG_VERSION;
    s.autoDepthTareOnBoot = 0;
    s.autoImuLevelOnBoot = 0;
    saveSettings(s);
    return true;
  }

  // Automatic EEPROM Schema Migration (v5 -> v8)
  if (s.magic == CONFIG_MAGIC && s.version == 5) {
    s.version = CONFIG_VERSION;
    s.autoRunTareSec = DEFAULT_AUTORUN_TARE_SEC;
    s.autoRunLevelSec = DEFAULT_AUTORUN_LEVEL_SEC;
    s.autoDepthTareOnBoot = 0;
    s.autoImuLevelOnBoot = 0;
    s.escDeadbandUs = DEFAULT_ESC_DEADBAND_US;
    s.diffHorizTrim = DEFAULT_DIFF_HORIZ_TRIM;
    s.diffVertTrim = DEFAULT_DIFF_VERT_TRIM;
    saveSettings(s);
    return true;
  }

  // Automatic EEPROM Schema Migration (v4 -> v8)
  if (s.magic == CONFIG_MAGIC && s.version == 4) {
    s.version = CONFIG_VERSION;
    for (size_t i = 0; i < THRUSTER_COUNT; ++i) {
      s.thrusters[i].reverse = DEFAULT_THRUSTER_REVERSE[i];
    }
    s.autoRunTareSec = DEFAULT_AUTORUN_TARE_SEC;
    s.autoRunLevelSec = DEFAULT_AUTORUN_LEVEL_SEC;
    s.autoDepthTareOnBoot = 0;
    s.autoImuLevelOnBoot = 0;
    s.escDeadbandUs = DEFAULT_ESC_DEADBAND_US;
    s.diffHorizTrim = DEFAULT_DIFF_HORIZ_TRIM;
    s.diffVertTrim = DEFAULT_DIFF_VERT_TRIM;
    saveSettings(s);
    return true;
  }

  loadDefaults(s);
  return false;
}

}  // namespace Config
