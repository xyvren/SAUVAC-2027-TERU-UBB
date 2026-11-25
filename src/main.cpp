#include <Arduino.h>
#include <Servo.h>
#include <math.h>

#include "config.h"
#include "sensors.h"
#include "mission.h"
#include "cli.h"
#include "wireless_config.h"

namespace {

Config::Settings settings{};
Servo thrusters[Config::THRUSTER_COUNT];
uint16_t currentPulse[Config::THRUSTER_COUNT];

bool armed = false;
bool pidEnabled = false;
bool outputsReady = false;
bool thrustersNeutral = true;
bool hilModeActive = false;
uint32_t hilLastCommandMs = 0;
constexpr uint32_t HIL_WATCHDOG_MS = 350;
constexpr uint16_t HIL_MAX_DELTA_US = 75;
bool bootCalibrationPending = false;
uint32_t bootCalibrationStartedAtMs = 0;
uint32_t bootCalibrationLastSampleAtMs = 0;
uint32_t bootCalibrationStableSinceMs = 0;
uint8_t bootCalibrationStableSamples = 0;
bool bootCalibrationHasPreviousSample = false;
Sensors::SensorData bootCalibrationInitialSample{};
Sensors::SensorData bootCalibrationPreviousSample{};
constexpr uint32_t BOOT_CALIBRATION_TIMEOUT_MS = 15000;
constexpr uint32_t BOOT_CALIBRATION_SAMPLE_INTERVAL_MS = 100;
constexpr uint32_t BOOT_CALIBRATION_STABLE_MS = 2000;
constexpr uint8_t BOOT_CALIBRATION_REQUIRED_SAMPLES = 20;

enum class FsmState {
  IDLE,
  COUNTDOWN,
  MISSION_RUNNING,
  MISSION_DONE,
  ABORTED,
  BOOT_CALIBRATING
};

FsmState fsmState = FsmState::IDLE;

// Countdown State
uint32_t countdownStartMs = 0;
uint8_t countdownDurationSec = 10;
int countdownLastSecLogged = -1;
bool countdownTareDone = false;
bool countdownImuDone = false;
bool countdownTareSucceeded = false;
bool countdownImuSucceeded = false;

// Mission State
size_t currentStepIndex = 0;
uint32_t stepStartedAtMs = 0;
static float activeLockedDepthCm = 0.0f;

// Motor Test Engine State
uint32_t testUntilMs = 0;
uint8_t testMotorId = 0;  // 0 = all, 1..6 = specific motor
uint16_t testMotorPulse[Config::THRUSTER_COUNT];

// ESC Calibration State
enum class EscCalibState {
  IDLE,
  MAX_THROTTLE,
  MIN_THROTTLE
};
EscCalibState escCalibState = EscCalibState::IDLE;
uint32_t escCalibPhaseStartMs = 0;

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
  activeLockedDepthCm = 0.0f;
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
      for (size_t j = 0; j < i; ++j) {
        thrusters[j].detach();
      }
      outputsReady = false;
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

  // If power is virtually zero, return exact neutral pulse
  if (fabsf(clampedPwr) < 0.01f) {
    return t.pulseNeutral;
  }

  float frac = clampedPwr / 100.0f;
  if (t.reverse) {
    frac = -frac;
  }
  frac *= t.trim;

  if (fabsf(frac) < 0.0001f) {
    return t.pulseNeutral;
  }

  // Deadband Jump Compensation:
  // Step immediately past settings.escDeadbandUs so low power commands physically turn the motor
  const uint16_t deadband = min(settings.escDeadbandUs, static_cast<uint16_t>(80));

  int32_t delta = 0;
  if (frac > 0.0f) {
    int32_t span = static_cast<int32_t>(t.pulseMax) - static_cast<int32_t>(t.pulseNeutral);
    if (span > deadband) {
      delta = static_cast<int32_t>(deadband) + lroundf(frac * (span - deadband));
    } else {
      delta = lroundf(frac * span);
    }
  } else if (frac < 0.0f) {
    int32_t span = static_cast<int32_t>(t.pulseNeutral) - static_cast<int32_t>(t.pulseMin);
    if (span > deadband) {
      delta = -static_cast<int32_t>(deadband) + lroundf(frac * (span - deadband));
    } else {
      delta = lroundf(frac * span);
    }
  }

  int32_t calcPulse = static_cast<int32_t>(t.pulseNeutral) + delta;
  return static_cast<uint16_t>(constrain(calcPulse, static_cast<int32_t>(t.pulseMin), static_cast<int32_t>(t.pulseMax)));
}

bool handleStartMotorTest(uint8_t id, float powerPercent, String& message) {
  if (!outputsReady) {
    message = "ERROR: Thruster hardware outputs not ready or failed to attach.";
    return false;
  }
  if (id > Config::THRUSTER_COUNT || !isfinite(powerPercent) ||
      fsmState != FsmState::IDLE || pidEnabled || testUntilMs != 0 ||
      escCalibState != EscCalibState::IDLE || armed) {
    message = "ERROR: Motor test requires an idle, disarmed system and a valid command.";
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

  String pwrStr = ((effPower >= 0.0f) ? "+" : "") + String(effPower, 1) + "%";
  char buf[128];
  if (id == 0) {
    snprintf(buf, sizeof(buf), "[TEST] Running ALL 6 motors at %s for 2.0s%s",
             pwrStr.c_str(), wasClamped ? " (CLAMPED by safety limit)" : "");
  } else {
    snprintf(buf, sizeof(buf), "[TEST] Running Motor T%u (GP%u) at %s (%uus) for 2.0s%s",
             id, settings.thrusters[id - 1].pin, pwrStr.c_str(), testMotorPulse[id - 1],
             wasClamped ? " (CLAMPED by safety limit)" : "");
  }
  message = String(buf);
  return true;
}

void handleEmergencyStop() {
  bootCalibrationPending = false;
  hilModeActive = false;
  testUntilMs = 0;
  testMotorId = 0;
  escCalibState = EscCalibState::IDLE;
  pidEnabled = false;
  fsmState = FsmState::IDLE;
  countdownTareSucceeded = false;
  countdownImuSucceeded = false;
  armed = false;
  rollIntegral = 0.0f;
  pitchIntegral = 0.0f;
  depthIntegral = 0.0f;
  stopAllThrusters();
}

bool handleStartMission(bool immediate) {
  if (bootCalibrationPending) {
    CLI::logWarn("Cannot start mission while automatic startup calibration is running.");
    return false;
  }
  if (!outputsReady || hilModeActive) {
    CLI::logError("Cannot start mission: outputs are not ready or HIL mode is active.");
    return false;
  }
  if (Mission::getStepCount() == 0) {
    CLI::logError("Cannot start mission: 0 steps loaded from /jalur.json!");
    return false;
  }

  if (fsmState != FsmState::IDLE || armed || pidEnabled ||
      testUntilMs != 0 || escCalibState != EscCalibState::IDLE) {
    CLI::logWarn("Cannot start mission: system is not idle and disarmed.");
    return false;
  }

  Sensors::SensorData sensor{};
  Sensors::snapshot(sensor);
  if (!sensor.imuReady || !sensor.pressureReady ||
      sensor.sampleCount == 0 || millis() - sensor.lastUpdateMs > 500) {
    CLI::logError("Cannot start mission: fresh IMU and depth sensor readings are required.");
    return false;
  }

  if (immediate || settings.autoRunDelaySec == 0) {
    if (!Sensors::calibrateDepthZero(settings) ||
        !Sensors::calibrateImuLevel(settings) || !settings.depthZeroCalibrated) {
      CLI::logError("Cannot start mission: depth and IMU calibration must both succeed.");
      return false;
    }
    fsmState = FsmState::MISSION_RUNNING;
    armed = true;
    thrustersNeutral = false;
    currentStepIndex = 0;
    stepStartedAtMs = millis();
    rollIntegral = 0.0f;
    pitchIntegral = 0.0f;
    depthIntegral = 0.0f;

    const auto* step = Mission::getStep(0);
    if (step && step->targetCm > 0.0f) {
      activeLockedDepthCm = step->targetCm;
    } else {
      activeLockedDepthCm = 0.0f;
    }
    char maskBuf[18];
    Mission::maskToString(step->motorMask, maskBuf, sizeof(maskBuf));
    if (activeLockedDepthCm > 0.0f) {
      CLI::logState("System ARMED. Starting Step 1/%u: %s [%s] (%s) Power: %.0f%% (Duration: %.1fs) | Target: %.0f cm",
                    static_cast<unsigned>(Mission::getStepCount()),
                    step->name,
                    Mission::actionToString(step->action),
                    maskBuf,
                    step->power,
                    step->durationSec,
                    activeLockedDepthCm);
    } else {
      CLI::logState("System ARMED. Starting Step 1/%u: %s [%s] (%s) Power: %.0f%% (Duration: %.1fs)",
                    static_cast<unsigned>(Mission::getStepCount()),
                    step->name,
                    Mission::actionToString(step->action),
                    maskBuf,
                    step->power,
                    step->durationSec);
    }
  } else {
    // Start countdown
    fsmState = FsmState::COUNTDOWN;
    countdownStartMs = millis();
    countdownDurationSec = settings.autoRunDelaySec;
    countdownLastSecLogged = -1;
    countdownTareDone = false;
    countdownImuDone = false;
    countdownTareSucceeded = false;
    countdownImuSucceeded = false;
    CLI::logState("Launch countdown started: %u seconds. Type 'stop' to abort.", countdownDurationSec);
  }
  return true;
}

void handleCancelMission() {
  fsmState = FsmState::IDLE;
  handleEmergencyStop();
}

bool handleIsMissionActive() {
  return fsmState == FsmState::MISSION_RUNNING;
}

bool handleIsCountdownActive() {
  return fsmState == FsmState::COUNTDOWN;
}

int handleGetCountdownRemaining() {
  if (fsmState != FsmState::COUNTDOWN) return 0;
  uint32_t elapsed = (millis() - countdownStartMs) / 1000;
  int rem = static_cast<int>(countdownDurationSec) - static_cast<int>(elapsed);
  return (rem < 0) ? 0 : rem;
}

bool handleSetPidState(bool enabled) {
  if (enabled) {
    if (!outputsReady || fsmState != FsmState::IDLE || armed ||
        testUntilMs != 0 || escCalibState != EscCalibState::IDLE) {
      return false;
    }
    Sensors::SensorData s{};
    Sensors::snapshot(s);
    if (!s.imuReady || !s.pressureReady || !settings.depthZeroCalibrated ||
        s.sampleCount == 0 || millis() - s.lastUpdateMs > 500) {
      return false;
    }
    targetDepthCm = s.depthCm;
    rollIntegral = 0.0f;
    pitchIntegral = 0.0f;
    depthIntegral = 0.0f;
    previousDepthCm = s.depthCm;
    pidEnabled = true;
    armed = true;
    thrustersNeutral = false;
    return true;
  } else {
    handleEmergencyStop();
    return true;
  }
}

uint16_t handleGetCurrentPulse(size_t index) {
  if (index < Config::THRUSTER_COUNT) {
    return currentPulse[index];
  }
  return 1500;
}

bool handleSaveSettings() {
  const bool saved = Config::saveSettings(settings);
  if (saved) {
    WirelessConfig::syncFromSettings(true);
  }
  return saved;
}

void handleResetDefaults() {
  Config::loadDefaults(settings);
  WirelessConfig::syncFromSettings(false);
  stopAllThrusters();
}

void handleSetTargetDepth(float target) {
  targetDepthCm = target;
}

float handleGetTargetDepth() {
  if (fsmState == FsmState::MISSION_RUNNING && activeLockedDepthCm > 0.0f) {
    return activeLockedDepthCm;
  }
  return targetDepthCm;
}

void updateThrusters(const float commandedAxes[Config::AXIS_COUNT]) {
  const float safetyCap = static_cast<float>(settings.powerLimitPercent);

  // Differential trim multipliers:
  // settings.diffHorizTrim (-20.0 to +20.0%): Port (T1, T3) vs Starboard (T2, T4)
  // settings.diffVertTrim (-20.0 to +20.0%): T5 vs T6
  const float portMult = 1.0f + (settings.diffHorizTrim * 0.01f);
  const float stbdMult = 1.0f - (settings.diffHorizTrim * 0.01f);
  const float t5Mult   = 1.0f + (settings.diffVertTrim * 0.01f);
  const float t6Mult   = 1.0f - (settings.diffVertTrim * 0.01f);

  for (size_t i = 0; i < Config::THRUSTER_COUNT; ++i) {
    float netPercent = 0.0f;
    for (size_t a = 0; a < Config::AXIS_COUNT; ++a) {
      netPercent += Config::DEFAULT_MIXER[i][a] * commandedAxes[a];
    }

    // Apply differential trims
    if (i == 0 || i == 2) {
      netPercent *= portMult;  // T1 (Rear-Left), T3 (Front-Left)
    } else if (i == 1 || i == 3) {
      netPercent *= stbdMult;  // T2 (Front-Right), T4 (Rear-Right)
    } else if (i == 4) {
      netPercent *= t5Mult;    // T5 (Mid-Right Vert)
    } else if (i == 5) {
      netPercent *= t6Mult;    // T6 (Mid-Left Vert)
    }

    netPercent = constrain(netPercent, -safetyCap, safetyCap);
    const uint16_t pulse = calculateTargetPulse(i, netPercent);
    thrusters[i].writeMicroseconds(pulse);
    currentPulse[i] = pulse;
  }
}

void updatePidStabilization(float dt) {
  Sensors::SensorData sensor{};
  Sensors::snapshot(sensor);

  if (!sensor.imuReady || !sensor.pressureReady || !settings.depthZeroCalibrated ||
      sensor.sampleCount == 0 || millis() - sensor.lastUpdateMs > 500) {
    CLI::logError("PID stopped: fresh IMU and calibrated depth sensor readings are required.");
    handleEmergencyStop();
    return;
  }

  // 1. Roll PID (Leveling horizontal trim)
  const float rollError = 0.0f - sensor.roll;
  rollIntegral = constrain(rollIntegral + rollError * dt, -30.0f, 30.0f);
  const float rollRate = sensor.gyroDps[0];
  float rollOutput = settings.pidRoll.kp * rollError +
                     settings.pidRoll.ki * rollIntegral -
                     settings.pidRoll.kd * rollRate;

  // 2. Depth PID
  float depthOutput = 0.0f;
  if ((sensor.pressureReady && settings.depthZeroCalibrated) || Sensors::isInjectionActive()) {
    const float depthError = targetDepthCm - sensor.depthCm;
    depthIntegral = constrain(depthIntegral + depthError * dt, -50.0f, 50.0f);
    const float depthRate = (dt > 0.001f) ? (sensor.depthCm - previousDepthCm) / dt : 0.0f;
    previousDepthCm = sensor.depthCm;
    depthOutput = settings.pidDepth.kp * depthError +
                  settings.pidDepth.ki * depthIntegral -
                  settings.pidDepth.kd * depthRate;
  }

  // Mixer allocation: pitch control remains disabled until actuator geometry is verified.
  const float commandedAxes[Config::AXIS_COUNT] = {
      0.0f,         // Surge
      0.0f,         // Sway
      depthOutput,  // Heave
      0.0f,         // Yaw
      0.0f,         // Pitch is unsupported by the verified thruster geometry
      rollOutput    // Roll
  };

  updateThrusters(commandedAxes);
}

void handleOnInjected() {
  if (pidEnabled && armed) {
    const uint32_t now = millis();
    const float dt = (lastOutputAtMs == 0) ? 0.02f : min((now - lastOutputAtMs) / 1000.0f, 0.05f);
    lastOutputAtMs = now;
    updatePidStabilization(dt);
  }
}

void updateMissionStepExecution(float dt, uint32_t now) {
  Sensors::SensorData sensor{};
  Sensors::snapshot(sensor);
  if (!outputsReady || !sensor.imuReady || !sensor.pressureReady ||
      !settings.depthZeroCalibrated || sensor.sampleCount == 0 ||
      now - sensor.lastUpdateMs > 500) {
    CLI::logError("Mission aborted: fresh sensors, calibrated depth, and thruster outputs are required.");
    handleEmergencyStop();
    return;
  }

  if (currentStepIndex >= Mission::getStepCount()) {
    fsmState = FsmState::MISSION_DONE;
    stopAllThrusters();
    armed = false;
    Serial.println();
    Serial.println(F("\033[1;32m[MISSION] =============================================================\033[0m"));
    Serial.printf("\033[1;32m[MISSION] ALL %u MISSION STEPS COMPLETED SUCCESSFULLY!\033[0m\r\n",
                  static_cast<unsigned>(Mission::getStepCount()));
    Serial.println(F("\033[1;32m[MISSION] All 6 thrusters neutralized to 1500us. System disarmed.\033[0m"));
    Serial.println(F("\033[1;32m[MISSION] =============================================================\033[0m"));
    fsmState = FsmState::IDLE;
    CLI::printPrompt();
    return;
  }

  const auto* step = Mission::getStep(currentStepIndex);
  const uint32_t stepElapsedMs = now - stepStartedAtMs;
  const uint32_t stepDurationMs = static_cast<uint32_t>(step->durationSec * 1000.0f);

  // Check if step duration has expired:
  if (stepElapsedMs >= stepDurationMs) {
    Serial.printf("[MISSION] Step %u finished: %s (Duration: %.1fs)\r\n",
                  static_cast<unsigned>(currentStepIndex + 1),
                  step->name,
                  step->durationSec);

    currentStepIndex++;
    if (currentStepIndex >= Mission::getStepCount()) {
      fsmState = FsmState::MISSION_DONE;
      activeLockedDepthCm = 0.0f;
      stopAllThrusters();
      armed = false;
      Serial.println();
      Serial.println(F("\033[1;32m[MISSION] =============================================================\033[0m"));
      Serial.printf("\033[1;32m[MISSION] ALL %u MISSION STEPS COMPLETED SUCCESSFULLY!\033[0m\r\n",
                    static_cast<unsigned>(Mission::getStepCount()));
      Serial.println(F("\033[1;32m[MISSION] All 6 thrusters neutralized to 1500us. System disarmed.\033[0m"));
      Serial.println(F("\033[1;32m[MISSION] =============================================================\033[0m"));
      fsmState = FsmState::IDLE;
      CLI::printPrompt();
      return;
    }

    stepStartedAtMs = now;
    step = Mission::getStep(currentStepIndex);
    if (step && step->targetCm > 0.0f) {
      activeLockedDepthCm = step->targetCm;
    }
    char maskBuf[18];
    Mission::maskToString(step->motorMask, maskBuf, sizeof(maskBuf));
    Serial.printf("[MISSION] Starting Step %u/%u: %s | Action: %s (%s) | Power: %.0f%% | Duration: %.1fs",
                  static_cast<unsigned>(currentStepIndex + 1),
                  static_cast<unsigned>(Mission::getStepCount()),
                  step->name,
                  Mission::actionToString(step->action),
                  maskBuf,
                  step->power,
                  step->durationSec);
    if (activeLockedDepthCm > 0.0f) {
      Serial.printf(" | Target: %.0f cm\r\n", activeLockedDepthCm);
    } else {
      Serial.println();
    }
  }

  if (step && step->targetCm > 0.0f) {
    activeLockedDepthCm = step->targetCm;
  }

  // 1. Roll leveling PID; pitch has no verified actuator authority.
  float rollOutput = 0.0f;
  if (sensor.imuReady) {
    const float rollError = 0.0f - sensor.roll;
    rollIntegral = constrain(rollIntegral + rollError * dt, -30.0f, 30.0f);
    rollOutput = settings.pidRoll.kp * rollError +
                 settings.pidRoll.ki * rollIntegral -
                 settings.pidRoll.kd * sensor.gyroDps[0];

  }

  // 2. Translational & Yaw axes from step definition
  float surge = 0.0f;
  float sway = 0.0f;
  float heave = 0.0f;
  float yaw = 0.0f;

  const float safetyCap = static_cast<float>(settings.powerLimitPercent);
  const float stepPower = constrain(step->power, 0.0f, safetyCap);

  // Helper for closed-loop depth hold
  auto calculateDepthHoldHeave = [&](float targetDepth) -> float {
    if (((sensor.pressureReady && settings.depthZeroCalibrated) || Sensors::isInjectionActive()) && targetDepth > 0.0f) {
      const float depthError = targetDepth - sensor.depthCm;
      depthIntegral = constrain(depthIntegral + depthError * dt, -50.0f, 50.0f);
      const float depthRate = (dt > 0.001f) ? (sensor.depthCm - previousDepthCm) / dt : 0.0f;
      previousDepthCm = sensor.depthCm;
      const float dOut = settings.pidDepth.kp * depthError +
                         settings.pidDepth.ki * depthIntegral -
                         settings.pidDepth.kd * depthRate;
      return constrain(dOut, -safetyCap, safetyCap);
    }
    return 0.0f;
  };

  switch (step->action) {
    case Mission::Action::DIVE:
      if (step->targetCm > 0.0f) {
        activeLockedDepthCm = step->targetCm;
        heave = calculateDepthHoldHeave(activeLockedDepthCm);
      } else if (activeLockedDepthCm > 0.0f) {
        heave = calculateDepthHoldHeave(activeLockedDepthCm);
      } else {
        heave = stepPower;  // Constant downward thrust
      }
      break;

    case Mission::Action::FORWARD:
      surge = stepPower;
      if (activeLockedDepthCm > 0.0f) heave = calculateDepthHoldHeave(activeLockedDepthCm);
      break;

    case Mission::Action::BACKWARD:
      surge = -stepPower;
      if (activeLockedDepthCm > 0.0f) heave = calculateDepthHoldHeave(activeLockedDepthCm);
      break;

    case Mission::Action::TURN:
      yaw = stepPower;
      if (activeLockedDepthCm > 0.0f) heave = calculateDepthHoldHeave(activeLockedDepthCm);
      break;

    case Mission::Action::SWAY_LEFT:
      sway = -stepPower;
      if (activeLockedDepthCm > 0.0f) heave = calculateDepthHoldHeave(activeLockedDepthCm);
      break;

    case Mission::Action::SWAY_RIGHT:
      sway = stepPower;
      if (activeLockedDepthCm > 0.0f) heave = calculateDepthHoldHeave(activeLockedDepthCm);
      break;

    case Mission::Action::HOVER:
      if (activeLockedDepthCm > 0.0f) heave = calculateDepthHoldHeave(activeLockedDepthCm);
      break;

    case Mission::Action::SURFACE:
      activeLockedDepthCm = 0.0f;
      heave = -stepPower;  // Upward push
      break;

    case Mission::Action::MOTOR_1:
    case Mission::Action::MOTOR_2:
    case Mission::Action::MOTOR_3:
    case Mission::Action::MOTOR_4:
    case Mission::Action::MOTOR_5:
    case Mission::Action::MOTOR_6:
    case Mission::Action::ALL_MOTORS: {
      uint8_t mask = step->motorMask;
      if (mask == 0) {
        if (step->action == Mission::Action::ALL_MOTORS) {
          mask = 0x3F;
        } else {
          mask = (1 << (static_cast<int>(step->action) - static_cast<int>(Mission::Action::MOTOR_1)));
        }
      }
      for (size_t i = 0; i < Config::THRUSTER_COUNT; ++i) {
        if (mask & (1 << i)) {
          const uint16_t pulse = calculateTargetPulse(i, stepPower);
          thrusters[i].writeMicroseconds(pulse);
          currentPulse[i] = pulse;
        } else {
          const uint16_t neutral = settings.thrusters[i].pulseNeutral;
          thrusters[i].writeMicroseconds(neutral);
          currentPulse[i] = neutral;
        }
      }
      return;  // Direct individual thruster execution completed
    }

    default:
      break;
  }

  // 3. Mix commanded axes
  const float commandedAxes[Config::AXIS_COUNT] = {
      surge, sway, heave, yaw, 0.0f, rollOutput
  };

  updateThrusters(commandedAxes);

  // Apply step motorMask filter if specified (disable any thruster not set in mask)
  if (step->motorMask != 0 && step->motorMask != 0x3F) {
    for (size_t i = 0; i < Config::THRUSTER_COUNT; ++i) {
      if (!(step->motorMask & (1 << i))) {
        const uint16_t neutral = settings.thrusters[i].pulseNeutral;
        thrusters[i].writeMicroseconds(neutral);
        currentPulse[i] = neutral;
      }
    }
  }
}

uint8_t handleGetFsmState() {
  return static_cast<uint8_t>(fsmState);
}

bool handleStartEscCalib() {
  if (!outputsReady) {
    CLI::logError("ESC Calibration: Outputs not ready!");
    return false;
  }
  if (fsmState != FsmState::IDLE || armed || pidEnabled || testUntilMs != 0 ||
      escCalibState != EscCalibState::IDLE) {
    CLI::logError("ESC Calibration requires an idle, disarmed system.");
    return false;
  }
  escCalibState = EscCalibState::MAX_THROTTLE;
  escCalibPhaseStartMs = millis();
  armed = true;
  thrustersNeutral = false;
  for (size_t i = 0; i < Config::THRUSTER_COUNT; ++i) {
    thrusters[i].writeMicroseconds(2000);
    currentPulse[i] = 2000;
  }
  Serial.println();
  Serial.println(F("[ESC CALIB] Phase 1/3: Outputting MAX throttle (2000us) to all 6 ESCs."));
  Serial.println(F("[ESC CALIB] Waiting 3.0s for high-throttle confirmation beeps..."));
  return true;
}

bool handleSetRawPulseAll(uint16_t pulseUs) {
  if (!outputsReady || (pulseUs != 1000 && pulseUs != 1500 && pulseUs != 2000)) {
    return false;
  }
  if (pulseUs != 1500 &&
      (fsmState != FsmState::IDLE || armed || pidEnabled || testUntilMs != 0 ||
       escCalibState != EscCalibState::IDLE)) {
    return false;
  }
  if (pulseUs == 1500) {
    handleEmergencyStop();
    return true;
  }
  armed = (pulseUs != 1500);
  thrustersNeutral = false;
  for (size_t i = 0; i < Config::THRUSTER_COUNT; ++i) {
    thrusters[i].writeMicroseconds(pulseUs);
    currentPulse[i] = pulseUs;
  }
  return true;
}

bool handleSetHilMode(bool enabled) {
  if (!enabled) {
    if (hilModeActive) handleEmergencyStop();
    return true;
  }
  if (!outputsReady || fsmState != FsmState::IDLE || armed || pidEnabled ||
      testUntilMs != 0 || escCalibState != EscCalibState::IDLE ||
      settings.autoRunEnabled || Sensors::isInjectionActive()) {
    return false;
  }
  for (size_t i = 0; i < Config::THRUSTER_COUNT; ++i) {
    if (settings.thrusters[i].pulseNeutral != 1500) return false;
  }

  hilModeActive = true;
  hilLastCommandMs = millis();
  armed = true;
  thrustersNeutral = true;
  stopAllThrusters();
  return true;
}

bool handleSetHilPulses(const uint16_t* pulses) {
  if (!pulses || !outputsReady || !hilModeActive || !armed ||
      fsmState != FsmState::IDLE || pidEnabled || testUntilMs != 0 ||
      escCalibState != EscCalibState::IDLE) {
    return false;
  }
  for (size_t i = 0; i < Config::THRUSTER_COUNT; ++i) {
    if (pulses[i] < 1500 - HIL_MAX_DELTA_US ||
        pulses[i] > 1500 + HIL_MAX_DELTA_US) {
      return false;
    }
  }

  for (size_t i = 0; i < Config::THRUSTER_COUNT; ++i) {
    thrusters[i].writeMicroseconds(pulses[i]);
    currentPulse[i] = pulses[i];
  }
  thrustersNeutral = false;
  hilLastCommandMs = millis();
  return true;
}

void updateEscCalib(uint32_t now) {
  if (escCalibState == EscCalibState::MAX_THROTTLE) {
    for (size_t i = 0; i < Config::THRUSTER_COUNT; ++i) {
      thrusters[i].writeMicroseconds(2000);
      currentPulse[i] = 2000;
    }
    if (now - escCalibPhaseStartMs >= 3000) {
      escCalibState = EscCalibState::MIN_THROTTLE;
      escCalibPhaseStartMs = now;
      for (size_t i = 0; i < Config::THRUSTER_COUNT; ++i) {
        thrusters[i].writeMicroseconds(1000);
        currentPulse[i] = 1000;
      }
      Serial.println();
      Serial.println(F("[ESC CALIB] Phase 2/3: Outputting MIN throttle (1000us) to all 6 ESCs."));
      Serial.println(F("[ESC CALIB] Waiting 3.0s for low-throttle confirmation beeps..."));
    }
    return;
  }
  if (escCalibState == EscCalibState::MIN_THROTTLE) {
    for (size_t i = 0; i < Config::THRUSTER_COUNT; ++i) {
      thrusters[i].writeMicroseconds(1000);
      currentPulse[i] = 1000;
    }
    if (now - escCalibPhaseStartMs >= 3000) {
      escCalibState = EscCalibState::IDLE;
      stopAllThrusters();
      armed = false;
      Serial.println();
      Serial.println(F("\033[1;32m[ESC CALIB] Phase 3/3: Outputting NEUTRAL (1500us). ESC calibration COMPLETE!\033[0m"));
      Serial.println(F("[ESC CALIB] All 6 thrusters neutralized to 1500us. System safe."));
      CLI::printPrompt();
    }
    return;
  }
}

void updateAutoRunCountdown(uint32_t now) {
  const uint32_t elapsedMs = now - countdownStartMs;
  const int remSec = static_cast<int>(countdownDurationSec) - static_cast<int>(elapsedMs / 1000);

  if (remSec != countdownLastSecLogged) {
    countdownLastSecLogged = remSec;
    if (remSec > 0) {
      Serial.printf("[AUTORUN] Launch countdown: T-%ds... (Send 'stop' to abort)\r\n", remSec);
    }

    // Surface Tare Calibration (MS5803 zero pressure) at configured autoRunTareSec
    if (remSec <= settings.autoRunTareSec && !countdownTareDone) {
      countdownTareDone = true;
      countdownTareSucceeded = Sensors::calibrateDepthZero(settings);
      if (countdownTareSucceeded) {
        Serial.printf("[AUTORUN] Surface tare complete: MS5803 zero reference at %.1f mbar.\r\n",
                      settings.depthZeroPressureMbar);
      } else {
        Serial.println(F("[AUTORUN] Surface tare failed; mission launch will be aborted."));
      }
    }

    // IMU Leveling Calibration (MPU-6050 level pitch/roll offsets) at configured autoRunLevelSec
    if (remSec <= settings.autoRunLevelSec && !countdownImuDone) {
      countdownImuDone = true;
      countdownImuSucceeded = Sensors::calibrateImuLevel(settings);
      if (countdownImuSucceeded) {
        Serial.printf("[AUTORUN] IMU leveling complete: Pitch offset=%.1f°, Roll offset=%.1f°.\r\n",
                      settings.pitchOffset, settings.rollOffset);
      } else {
        Serial.println(F("[AUTORUN] IMU leveling failed; mission launch will be aborted."));
      }
    }

    // T-1s: Pre-arm notification
    if (remSec == 1) {
      Serial.println(F("[AUTORUN] Arming thruster propulsion system..."));
    }
  }

  // T-0s: Transition to MISSION_RUNNING
  if (elapsedMs >= static_cast<uint32_t>(countdownDurationSec) * 1000) {
    Sensors::SensorData sensor{};
    Sensors::snapshot(sensor);
    if (Mission::getStepCount() == 0 || !outputsReady || !sensor.imuReady ||
        !sensor.pressureReady || !countdownTareSucceeded ||
        !countdownImuSucceeded || !settings.depthZeroCalibrated ||
        sensor.sampleCount == 0 || now - sensor.lastUpdateMs > 500) {
      Serial.println(F("[AUTORUN] ERROR: Mission preflight failed (route, outputs, fresh sensors, or calibration). Aborting launch."));
      handleEmergencyStop();
      CLI::printPrompt();
    } else {
      Serial.println();
      Serial.println(F("\033[1;32m[AUTORUN] T-0s: SYSTEM ARMED! Starting Autonomous Trajectory...\033[0m"));
      fsmState = FsmState::MISSION_RUNNING;
      armed = true;
      thrustersNeutral = false;
      currentStepIndex = 0;
      stepStartedAtMs = now;
      rollIntegral = 0.0f;
      pitchIntegral = 0.0f;
      depthIntegral = 0.0f;

      const auto* step = Mission::getStep(0);
      if (step && step->targetCm > 0.0f) {
        activeLockedDepthCm = step->targetCm;
      } else {
        activeLockedDepthCm = 0.0f;
      }
      Serial.printf("[MISSION] Step 1/%u: %s | Action: %s | Power: %.0f%% | Duration: %.1fs",
                    static_cast<unsigned>(Mission::getStepCount()),
                    step->name,
                    Mission::actionToString(step->action),
                    step->power,
                    step->durationSec);
      if (activeLockedDepthCm > 0.0f) {
        Serial.printf(" | Target: %.0f cm\r\n", activeLockedDepthCm);
      } else {
        Serial.println();
      }
    }
  }
}

void startAutoRunCountdown(uint32_t now) {
  fsmState = FsmState::COUNTDOWN;
  countdownStartMs = now;
  countdownDurationSec = settings.autoRunDelaySec;
  countdownLastSecLogged = -1;
  countdownTareDone = false;
  countdownImuDone = false;
  countdownTareSucceeded = false;
  countdownImuSucceeded = false;
  Serial.println();
  Serial.println(F("\033[1;36m[AUTORUN] Auto-Run on Boot is ENABLED!\033[0m"));
  Serial.printf("[AUTORUN] Launch countdown: %u seconds to launch. Send 'stop' to abort.\r\n",
                countdownDurationSec);
}

bool stableBootCalibrationSample(
    const Sensors::SensorData& sample,
    bool tareDepth,
    bool levelImu) {
  const float accelerationMagnitude = sqrtf(
      sample.accelG[0] * sample.accelG[0] +
      sample.accelG[1] * sample.accelG[1] +
      sample.accelG[2] * sample.accelG[2]);
  const bool pressureReady = !tareDepth ||
      (sample.pressureReady && isfinite(sample.pressureMbar) &&
       sample.pressureMbar >= 500.0f && sample.pressureMbar <= 2000.0f);
  const bool levelOrientation = !levelImu ||
      (fabsf(sample.rawPitch) <= 10.0f && fabsf(sample.rawRoll) <= 10.0f);
  return sample.imuReady && sample.sampleCount > 0 &&
      millis() - sample.lastUpdateMs <= 500 && pressureReady &&
      isfinite(sample.rawPitch) && isfinite(sample.rawRoll) && levelOrientation &&
      isfinite(accelerationMagnitude) && accelerationMagnitude >= 0.8f &&
      accelerationMagnitude <= 1.2f &&
      isfinite(sample.gyroDps[0]) && fabsf(sample.gyroDps[0]) <= 2.0f &&
      isfinite(sample.gyroDps[1]) && fabsf(sample.gyroDps[1]) <= 2.0f &&
      isfinite(sample.gyroDps[2]) && fabsf(sample.gyroDps[2]) <= 2.0f;
}

bool bootCalibrationSampleContinuous(
    const Sensors::SensorData& previous,
    const Sensors::SensorData& current,
    bool tareDepth) {
  return fabsf(current.rawPitch - previous.rawPitch) <= 0.6f &&
      fabsf(current.rawRoll - previous.rawRoll) <= 0.6f &&
      (!tareDepth || fabsf(current.pressureMbar - previous.pressureMbar) <= 2.0f);
}

void finishBootCalibration(bool succeeded, uint32_t now) {
  bootCalibrationPending = false;
  if (settings.autoRunEnabled) {
    if (succeeded) {
      startAutoRunCountdown(now);
    } else {
      fsmState = FsmState::ABORTED;
      Serial.println(F("[BOOT CALIB] Auto-Run cancelled because startup calibration failed."));
      CLI::printPrompt();
    }
  } else {
    fsmState = FsmState::IDLE;
  }
}

void updateBootCalibration(uint32_t now) {
  if (!bootCalibrationPending) return;
  const bool tareDepth = settings.autoDepthTareOnBoot != 0;
  const bool levelImu = settings.autoImuLevelOnBoot != 0;
  if (now - bootCalibrationStartedAtMs >= BOOT_CALIBRATION_TIMEOUT_MS) {
    Serial.println(F("[BOOT CALIB] Timed out; previous sensor calibration was kept."));
    finishBootCalibration(false, now);
    return;
  }
  if (now - bootCalibrationLastSampleAtMs < BOOT_CALIBRATION_SAMPLE_INTERVAL_MS) return;
  bootCalibrationLastSampleAtMs = now;

  Sensors::SensorData sample{};
  Sensors::snapshot(sample);
  if (Sensors::isInjectionActive() ||
      !stableBootCalibrationSample(sample, tareDepth, levelImu)) {
    bootCalibrationStableSamples = 0;
    bootCalibrationHasPreviousSample = false;
    return;
  }
  if (bootCalibrationHasPreviousSample &&
      (!bootCalibrationSampleContinuous(bootCalibrationPreviousSample, sample, tareDepth) ||
       fabsf(sample.rawPitch - bootCalibrationInitialSample.rawPitch) > 1.0f ||
       fabsf(sample.rawRoll - bootCalibrationInitialSample.rawRoll) > 1.0f ||
       (tareDepth &&
        fabsf(sample.pressureMbar - bootCalibrationInitialSample.pressureMbar) > 3.0f))) {
    bootCalibrationStableSamples = 0;
    bootCalibrationHasPreviousSample = false;
  }
  if (bootCalibrationStableSamples == 0) {
    bootCalibrationStableSinceMs = now;
    bootCalibrationInitialSample = sample;
  }
  bootCalibrationStableSamples++;
  bootCalibrationPreviousSample = sample;
  bootCalibrationHasPreviousSample = true;

  if (bootCalibrationStableSamples < BOOT_CALIBRATION_REQUIRED_SAMPLES ||
      now - bootCalibrationStableSinceMs < BOOT_CALIBRATION_STABLE_MS) {
    return;
  }

  Config::Settings calibrated = settings;
  const bool depthCalibrated =
      !tareDepth || (Sensors::calibrateDepthZero(calibrated) && calibrated.depthZeroCalibrated);
  const bool imuLeveled = !levelImu || Sensors::calibrateImuLevel(calibrated);
  if (!depthCalibrated || !imuLeveled) {
    Serial.println(F("[BOOT CALIB] Calibration failed; previous sensor calibration was kept."));
    finishBootCalibration(false, now);
    return;
  }

  const bool changed =
      (tareDepth &&
       (!settings.depthZeroCalibrated ||
        fabsf(calibrated.depthZeroPressureMbar - settings.depthZeroPressureMbar) >= 0.2f)) ||
      (levelImu &&
       (fabsf(calibrated.pitchOffset - settings.pitchOffset) >= 0.1f ||
        fabsf(calibrated.rollOffset - settings.rollOffset) >= 0.1f));
  if (changed) {
    if (!Config::saveSettings(calibrated)) {
      Serial.println(F("[BOOT CALIB] EEPROM save failed; previous sensor calibration was kept."));
      finishBootCalibration(false, now);
      return;
    }
    settings = calibrated;
    WirelessConfig::syncFromSettings(true);
  }

  Serial.printf("[BOOT CALIB] Startup calibration complete%s:",
                changed ? " and saved" : " (no EEPROM change)");
  if (tareDepth) Serial.printf(" depth zero %.1f mbar;", calibrated.depthZeroPressureMbar);
  if (levelImu) {
    Serial.printf(" pitch %.1f deg, roll %.1f deg;", calibrated.pitchOffset, calibrated.rollOffset);
  }
  Serial.println();
  finishBootCalibration(true, now);
}

}  // namespace

void setup() {
  Serial.begin(115200);

  // Load configuration from EEPROM or defaults
  if (!Config::loadSettings(settings)) {
    Serial.println(F("[EEPROM] Defaults loaded (fresh EEPROM or version mismatch)."));
    Config::saveSettings(settings);
    Serial.println(F("[EEPROM] Version 8 defaults migrated & saved to EEPROM."));
  } else {
    Serial.printf("[EEPROM] Configuration loaded successfully (Checksum: 0x%08lX).\r\n", settings.checksum);
  }

  // Attach thrusters GP2..GP7
  attachThrusters();

  // Initialize sensors (Wire1 on GP26/GP27)
  Sensors::begin();
  bootCalibrationPending =
      settings.autoDepthTareOnBoot != 0 || settings.autoImuLevelOnBoot != 0;
  bootCalibrationStartedAtMs = millis();
  bootCalibrationLastSampleAtMs =
      bootCalibrationStartedAtMs - BOOT_CALIBRATION_SAMPLE_INTERVAL_MS;
  bootCalibrationStableSamples = 0;
  bootCalibrationHasPreviousSample = false;
  bootCalibrationInitialSample = Sensors::SensorData{};

  // Initialize LittleFS & Load Mission Trajectory
  if (!Mission::begin()) {
    CLI::logError("Mission storage or route unavailable; mission launch remains disabled until repaired.");
  }

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
      handleResetDefaults,
      handleStartMission,
      handleCancelMission,
      handleIsMissionActive,
      handleIsCountdownActive,
      handleGetCountdownRemaining,
      []() -> bool { return Sensors::calibrateDepthZero(settings); },
      []() -> bool { return Sensors::calibrateImuLevel(settings); },
      handleOnInjected,
      handleSetTargetDepth,
      handleGetTargetDepth,
      handleGetFsmState,
      handleStartEscCalib,
      handleSetRawPulseAll,
      handleSetHilMode,
      handleSetHilPulses,
      []() -> bool { return hilModeActive; }
  };
  CLI::begin(iface);
  WirelessConfig::begin(
      settings, armed, handleGetFsmState, stopAllThrusters, handleGetCurrentPulse,
      []() -> bool {
        return fsmState == FsmState::IDLE && !armed && !pidEnabled &&
               thrustersNeutral && testUntilMs == 0 &&
               escCalibState == EscCalibState::IDLE && !hilModeActive &&
               !settings.autoRunEnabled && !bootCalibrationPending &&
               !Sensors::isInjectionActive();
      },
      []() { handleEmergencyStop(); });

  if (bootCalibrationPending) {
    fsmState = FsmState::BOOT_CALIBRATING;
    if (settings.autoDepthTareOnBoot) {
      Serial.println(F("[BOOT CALIB] Automatic depth zero enabled; vehicle must be at the water surface."));
    }
    if (settings.autoImuLevelOnBoot) {
      Serial.println(F("[BOOT CALIB] Automatic IMU level enabled; vehicle must be stationary and level."));
    }
    Serial.println(F("[BOOT CALIB] Waiting for stable readings; no thrusters will be activated."));
  } else if (settings.autoRunEnabled) {
    startAutoRunCountdown(millis());
  } else {
    fsmState = FsmState::IDLE;
  }
}

void loop() {
  const uint32_t now = millis();

  if (WirelessConfig::firmwareUpdateInProgress()) {
    while (Serial.available() > 0) Serial.read();
    WirelessConfig::update();
    while (Serial.available() > 0) Serial.read();
    return;
  }

  // Injection loss aborts every active mode so mission output cannot resume afterward.
  static bool injectionWasActive = false;
  bool injectionIsActive = Sensors::isInjectionActive();
  if (injectionWasActive && !injectionIsActive) {
    injectionWasActive = false;
    handleEmergencyStop();
    CLI::logWarn("Simulation injection watchdog timeout (>500ms) - mission aborted and thrusters neutralized");
  } else if (injectionIsActive) {
    injectionWasActive = true;
  }

  // 1. Process CLI input (machine packets & human commands)
  CLI::update();
  const uint32_t hilCheckMs = millis();
  if (hilModeActive && hilCheckMs - hilLastCommandMs > HIL_WATCHDOG_MS) {
    handleEmergencyStop();
    CLI::logWarn("HIL heartbeat timeout (>350ms); thrusters neutralized and disarmed.");
  }
  WirelessConfig::update();
  if (WirelessConfig::firmwareUpdateInProgress()) {
    while (Serial.available() > 0) Serial.read();
    return;
  }

  // Telemetry Stream Emission (e.g. 10 Hz non-blocking)
  if (CLI::isTelemActive()) {
    static uint32_t lastTelemEmitMs = 0;
    uint32_t telemIntervalMs = 1000 / CLI::getTelemRateHz();
    if (now - lastTelemEmitMs >= telemIntervalMs) {
      lastTelemEmitMs = now;
      CLI::sendTelemPacket();
    }
  }

  // 2. Update sensor sampling & complementary filters (50 Hz)
  Sensors::update(settings);
  updateBootCalibration(millis());

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
      if (!pidEnabled && fsmState != FsmState::MISSION_RUNNING) {
        armed = false;
      }
      stopAllThrusters();
      CLI::logState("Motor test completed (returned to neutral 1500us)");
      CLI::printPrompt();
      return;
    }

    // ESC CALIBRATION SEQUENCE GUARD:
    if (escCalibState != EscCalibState::IDLE) {
      updateEscCalib(now);
      return;
    }

    // COUNTDOWN FSM STATE:
    if (fsmState == FsmState::COUNTDOWN) {
      updateAutoRunCountdown(now);
      return;
    }

    // MISSION_RUNNING FSM STATE:
    if (fsmState == FsmState::MISSION_RUNNING && armed) {
      updateMissionStepExecution(dt, now);
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
