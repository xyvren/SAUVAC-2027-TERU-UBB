#include <Arduino.h>
#include <Servo.h>
#include <math.h>

#include "config.h"
#include "sensors.h"
#include "cli.h"

namespace {

Config::Settings settings{};
Servo thrusters[Config::THRUSTER_COUNT];
uint16_t currentPulse[Config::THRUSTER_COUNT];

bool armed = false;
bool pidEnabled = false;
bool outputsReady = false;
bool thrustersNeutral = true;

// Motor Test Engine State
uint32_t testUntilMs = 0;
uint8_t testMotorId = 0;  // 0 = all, 1..6 = specific motor
uint16_t testMotorPulse[Config::THRUSTER_COUNT];

// Control Loop Timing (50 Hz / 20 ms fixed tick)
constexpr uint32_t OUTPUT_INTERVAL_MS = 20;
uint32_t lastOutputAtMs = 0;

// PID Controller Internal Integrators
float rollIntegral = 0.0f;
float pitchIntegral = 0.0f;
float depthIntegral = 0.0f;
float previousDepthCm = 0.0f;
float targetDepthCm = 0.0f;

void stopAllThrusters() {
  for (size_t i = 0; i < Config::THRUSTER_COUNT; ++i) {
    if (thrusters[i].attached()) {
      thrusters[i].writeMicroseconds(settings.thrusters[i].pulseNeutral);
    }
    currentPulse[i] = settings.thrusters[i].pulseNeutral;
  }
  thrustersNeutral = true;
}

bool attachThrusters() {
  for (size_t i = 0; i < Config::THRUSTER_COUNT; ++i) {
    const auto& t = settings.thrusters[i];
    // Attach with wide envelope [750, 2250] so fine-tuned pulses in CLI are never clamped by driver
    const int channel = thrusters[i].attach(t.pin, 750, 2250, t.pulseNeutral);
    if (channel < 0) {
      CLI::logError("Failed to attach thruster T%u on GP%u", static_cast<unsigned>(i + 1), t.pin);
      return false;
    }
    thrusters[i].writeMicroseconds(t.pulseNeutral);
    currentPulse[i] = t.pulseNeutral;
  }
  outputsReady = true;
  thrustersNeutral = true;
  CLI::logInfo("6 Thruster PWM outputs attached on GP2..GP7 (Neutral 1500us)");
  return true;
}

uint16_t calculateTargetPulse(size_t index, float powerPercent) {
  const auto& t = settings.thrusters[index];

  // Clamp magnitude to global safety limit
  const float maxPwr = static_cast<float>(settings.powerLimitPercent);
  const float clampedPwr = constrain(powerPercent, -maxPwr, maxPwr);

  float frac = clampedPwr / 100.0f;
  if (t.reverse) {
    frac = -frac;
  }
  frac *= t.trim;

  // Signed Intermediate Math Pattern to avoid ARM float-to-unsigned saturation
  int32_t delta = 0;
  if (frac >= 0.0f) {
    delta = lroundf(frac * (t.pulseMax - t.pulseNeutral));
  } else {
    delta = lroundf(frac * (t.pulseNeutral - t.pulseMin));
  }

  int32_t calcPulse = static_cast<int32_t>(t.pulseNeutral) + delta;
  return static_cast<uint16_t>(constrain(calcPulse, static_cast<int32_t>(t.pulseMin), static_cast<int32_t>(t.pulseMax)));
}

bool handleStartMotorTest(uint8_t id, float powerPercent, String& message) {
  if (!outputsReady) {
    message = "ERROR: Thruster hardware outputs not ready or failed to attach.";
    return false;
  }

  const float safetyLimit = static_cast<float>(settings.powerLimitPercent);
  float effPower = powerPercent;
  bool wasClamped = false;

  if (fabsf(powerPercent) > safetyLimit) {
    effPower = (powerPercent >= 0.0f) ? safetyLimit : -safetyLimit;
    wasClamped = true;
  }

  // Pre-calculate target pulse(s)
  if (id == 0) {
    // All motors
    for (size_t i = 0; i < Config::THRUSTER_COUNT; ++i) {
      testMotorPulse[i] = calculateTargetPulse(i, effPower);
    }
  } else {
    // Single motor
    const size_t idx = id - 1;
    testMotorPulse[idx] = calculateTargetPulse(idx, effPower);
  }

  testMotorId = id;
  testUntilMs = millis() + 2000;  // 2.0 second active test duration
  armed = true;
  thrustersNeutral = false;

  char buf[128];
  if (id == 0) {
    snprintf(buf, sizeof(buf), "[TEST] Running ALL 6 motors at %+.1f%% for 2.0s%s",
             effPower, wasClamped ? " (CLAMPED by safety limit)" : "");
  } else {
    snprintf(buf, sizeof(buf), "[TEST] Running Motor T%u (GP%u) at %+.1f%% (%uus) for 2.0s%s",
             id, settings.thrusters[id - 1].pin, effPower, testMotorPulse[id - 1],
             wasClamped ? " (CLAMPED by safety limit)" : "");
  }
  message = String(buf);
  return true;
}

void handleEmergencyStop() {
  testUntilMs = 0;
  testMotorId = 0;
  pidEnabled = false;
  armed = false;
  rollIntegral = 0.0f;
  pitchIntegral = 0.0f;
  depthIntegral = 0.0f;
  stopAllThrusters();
}

void handleSetPidState(bool enabled) {
  if (enabled) {
    Sensors::SensorData s{};
    Sensors::snapshot(s);
    targetDepthCm = s.depthCm;
    rollIntegral = 0.0f;
    pitchIntegral = 0.0f;
    depthIntegral = 0.0f;
    previousDepthCm = s.depthCm;
    pidEnabled = true;
    armed = true;
    thrustersNeutral = false;
  } else {
    pidEnabled = false;
    armed = false;
    stopAllThrusters();
  }
}

uint16_t handleGetCurrentPulse(size_t index) {
  if (index < Config::THRUSTER_COUNT) {
    return currentPulse[index];
  }
  return 1500;
}

bool handleSaveSettings() {
  return Config::saveSettings(settings);
}

void handleResetDefaults() {
  Config::loadDefaults(settings);
  stopAllThrusters();
}

void updatePidStabilization(float dt) {
  Sensors::SensorData sensor{};
  Sensors::snapshot(sensor);

  if (!sensor.imuReady) {
    // If IMU is offline, fail-safe to neutral
    stopAllThrusters();
    return;
  }

  // 1. Roll PID (Leveling horizontal trim)
  const float rollError = 0.0f - sensor.roll;
  rollIntegral = constrain(rollIntegral + rollError * dt, -30.0f, 30.0f);
  const float rollRate = sensor.gyroDps[0];
  float rollOutput = settings.pidRoll.kp * rollError +
                     settings.pidRoll.ki * rollIntegral -
                     settings.pidRoll.kd * rollRate;

  // 2. Pitch PID
  const float pitchError = 0.0f - sensor.pitch;
  pitchIntegral = constrain(pitchIntegral + pitchError * dt, -30.0f, 30.0f);
  const float pitchRate = sensor.gyroDps[1];
  float pitchOutput = settings.pidPitch.kp * pitchError +
                      settings.pidPitch.ki * pitchIntegral -
                      settings.pidPitch.kd * pitchRate;

  // 3. Depth PID
  float depthOutput = 0.0f;
  if (sensor.pressureReady && settings.depthZeroCalibrated) {
    const float depthError = targetDepthCm - sensor.depthCm;
    depthIntegral = constrain(depthIntegral + depthError * dt, -50.0f, 50.0f);
    const float depthRate = (dt > 0.001f) ? (sensor.depthCm - previousDepthCm) / dt : 0.0f;
    previousDepthCm = sensor.depthCm;
    depthOutput = settings.pidDepth.kp * depthError +
                  settings.pidDepth.ki * depthIntegral -
                  settings.pidDepth.kd * depthRate;
  }

  // Mixer allocation: [Surge=0, Sway=0, Heave=depthOutput, Yaw=0, Pitch=pitchOutput, Roll=rollOutput]
  const float commandedAxes[Config::AXIS_COUNT] = {
      0.0f,         // Surge
      0.0f,         // Sway
      depthOutput,  // Heave
      0.0f,         // Yaw
      pitchOutput,  // Pitch
      rollOutput    // Roll
  };

  const float safetyCap = static_cast<float>(settings.powerLimitPercent);

  for (size_t i = 0; i < Config::THRUSTER_COUNT; ++i) {
    float netPercent = 0.0f;
    for (size_t a = 0; a < Config::AXIS_COUNT; ++a) {
      netPercent += Config::DEFAULT_MIXER[i][a] * commandedAxes[a];
    }
    netPercent = constrain(netPercent, -safetyCap, safetyCap);
    const uint16_t pulse = calculateTargetPulse(i, netPercent);
    thrusters[i].writeMicroseconds(pulse);
    currentPulse[i] = pulse;
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);

  // Load configuration from EEPROM or defaults
  if (!Config::loadSettings(settings)) {
    Serial.println(F("[EEPROM] Defaults loaded (fresh EEPROM or version mismatch)."));
  } else {
    Serial.printf("[EEPROM] Configuration loaded successfully (Checksum: 0x%08lX).\r\n", settings.checksum);
  }

  // Attach thrusters GP2..GP7
  attachThrusters();

  // Initialize sensors (Wire1 on GP26/GP27)
  Sensors::begin();

  // Initialize interactive Serial CLI
  CLI::MotorControlInterface iface{
      &settings,
      &armed,
      &pidEnabled,
      handleGetCurrentPulse,
      handleStartMotorTest,
      handleEmergencyStop,
      handleSetPidState,
      handleSaveSettings,
      handleResetDefaults
  };
  CLI::begin(iface);
}

void loop() {
  const uint32_t now = millis();

  // 1. Process CLI input and stream telemetry
  CLI::update();

  // 2. Update sensor sampling & complementary filters (50 Hz)
  Sensors::update(settings);

  // 3. Periodic motor output loop (50 Hz / 20 ms)
  if (now - lastOutputAtMs >= OUTPUT_INTERVAL_MS) {
    const float dt = (lastOutputAtMs == 0) ? 0.02f : min((now - lastOutputAtMs) / 1000.0f, 0.05f);
    lastOutputAtMs = now;

    // ACTIVE MOTOR TEST GUARD:
    // If a motor test is running from CLI, continuously stream the commanded pulse at 50Hz!
    if (testUntilMs > 0 && now < testUntilMs) {
      if (testMotorId == 0) {
        // All motors
        for (size_t i = 0; i < Config::THRUSTER_COUNT; ++i) {
          thrusters[i].writeMicroseconds(testMotorPulse[i]);
          currentPulse[i] = testMotorPulse[i];
        }
      } else if (testMotorId >= 1 && testMotorId <= Config::THRUSTER_COUNT) {
        // Single motor
        const size_t targetIdx = static_cast<size_t>(testMotorId - 1);
        for (size_t i = 0; i < Config::THRUSTER_COUNT; ++i) {
          if (i == targetIdx) {
            thrusters[i].writeMicroseconds(testMotorPulse[i]);
            currentPulse[i] = testMotorPulse[i];
          } else {
            thrusters[i].writeMicroseconds(settings.thrusters[i].pulseNeutral);
            currentPulse[i] = settings.thrusters[i].pulseNeutral;
          }
        }
      }
      return;
    }

    // TEST COMPLETION TIMER:
    if (testUntilMs > 0 && now >= testUntilMs) {
      testUntilMs = 0;
      testMotorId = 0;
      if (!pidEnabled) {
        armed = false;
      }
      stopAllThrusters();
      CLI::logState("Motor test completed (returned to neutral 1500us)");
      CLI::printPrompt();
      return;
    }

    // ACTIVE CLOSED-LOOP PID STABILIZATION:
    if (pidEnabled && armed) {
      updatePidStabilization(dt);
      return;
    }

    // IDLE / DISARMED FAILSAFE:
    if (!armed && !thrustersNeutral) {
      stopAllThrusters();
    }
  }
}
