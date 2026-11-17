#include <Arduino.h>
#include <Servo.h>
#include <math.h>

#include "config.h"
#include "sensors.h"
#include "mission.h"
#include "cli.h"

namespace {

Config::Settings settings{};
Servo thrusters[Config::THRUSTER_COUNT];
uint16_t currentPulse[Config::THRUSTER_COUNT];

bool armed = false;
bool pidEnabled = false;
bool outputsReady = false;
bool thrustersNeutral = true;

enum class FsmState {
  IDLE,
  COUNTDOWN,
  MISSION_RUNNING,
  MISSION_DONE,
  ABORTED
};

FsmState fsmState = FsmState::IDLE;

// Countdown State
uint32_t countdownStartMs = 0;
uint8_t countdownDurationSec = 10;
int countdownLastSecLogged = -1;
bool countdownTareDone = false;
bool countdownImuDone = false;

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
  testUntilMs = 0;
  testMotorId = 0;
  escCalibState = EscCalibState::IDLE;
  pidEnabled = false;
  fsmState = FsmState::IDLE;
  armed = false;
  rollIntegral = 0.0f;
  pitchIntegral = 0.0f;
  depthIntegral = 0.0f;
  stopAllThrusters();
}

bool handleStartMission(bool immediate) {
  if (Mission::getStepCount() == 0) {
    CLI::logError("Cannot start mission: 0 steps loaded from /jalur.json!");
    return false;
  }

  if (fsmState == FsmState::MISSION_RUNNING) {
    CLI::logWarn("Mission already running!");
    return false;
  }

  if (immediate || settings.autoRunDelaySec == 0) {
    // Immediate arm and launch
    Sensors::calibrateDepthZero(settings);
    Sensors::calibrateImuLevel(settings);
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
  if ((sensor.pressureReady && settings.depthZeroCalibrated) || Sensors::isInjectionActive()) {
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

  Sensors::SensorData sensor{};
  Sensors::snapshot(sensor);

  // 1. Roll & Pitch Leveling PID
  float rollOutput = 0.0f;
  float pitchOutput = 0.0f;
  if (sensor.imuReady) {
    const float rollError = 0.0f - sensor.roll;
    rollIntegral = constrain(rollIntegral + rollError * dt, -30.0f, 30.0f);
    rollOutput = settings.pidRoll.kp * rollError +
                 settings.pidRoll.ki * rollIntegral -
                 settings.pidRoll.kd * sensor.gyroDps[0];

    const float pitchError = 0.0f - sensor.pitch;
    pitchIntegral = constrain(pitchIntegral + pitchError * dt, -30.0f, 30.0f);
    pitchOutput = settings.pidPitch.kp * pitchError +
                  settings.pidPitch.ki * pitchIntegral -
                  settings.pidPitch.kd * sensor.gyroDps[1];
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
      surge, sway, heave, yaw, pitchOutput, rollOutput
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

void handleStartEscCalib() {
  if (!outputsReady) {
    CLI::logError("ESC Calibration: Outputs not ready!");
    return;
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
}

void handleSetRawPulseAll(uint16_t pulseUs) {
  if (!outputsReady) return;
  armed = (pulseUs != 1500);
  thrustersNeutral = (pulseUs == 1500);
  for (size_t i = 0; i < Config::THRUSTER_COUNT; ++i) {
    thrusters[i].writeMicroseconds(pulseUs);
    currentPulse[i] = pulseUs;
  }
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
      if (Sensors::calibrateDepthZero(settings)) {
        Serial.printf("[AUTORUN] Surface tare complete: MS5803 zero reference at %.1f mbar.\r\n",
                      settings.depthZeroPressureMbar);
      } else {
        Serial.println(F("[AUTORUN] Surface tare: MS5803 offline, using stored reference."));
      }
    }

    // IMU Leveling Calibration (MPU-6050 level pitch/roll offsets) at configured autoRunLevelSec
    if (remSec <= settings.autoRunLevelSec && !countdownImuDone) {
      countdownImuDone = true;
      if (Sensors::calibrateImuLevel(settings)) {
        Serial.printf("[AUTORUN] IMU leveling complete: Pitch offset=%.1f°, Roll offset=%.1f°.\r\n",
                      settings.pitchOffset, settings.rollOffset);
      } else {
        Serial.println(F("[AUTORUN] IMU leveling: MPU-6050 offline, using stored offsets."));
      }
    }

    // T-1s: Pre-arm notification
    if (remSec == 1) {
      Serial.println(F("[AUTORUN] Arming thruster propulsion system..."));
    }
  }

  // T-0s: Transition to MISSION_RUNNING
  if (elapsedMs >= static_cast<uint32_t>(countdownDurationSec) * 1000) {
    if (Mission::getStepCount() == 0) {
      Serial.println(F("[AUTORUN] ERROR: No mission steps found in /jalur.json! Aborting launch."));
      fsmState = FsmState::IDLE;
      armed = false;
      stopAllThrusters();
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

}  // namespace

void setup() {
  Serial.begin(115200);

  // Load configuration from EEPROM or defaults
  if (!Config::loadSettings(settings)) {
    Serial.println(F("[EEPROM] Defaults loaded (fresh EEPROM or version mismatch)."));
    Config::saveSettings(settings);
    Serial.println(F("[EEPROM] Version 6 defaults migrated & saved to EEPROM."));
  } else {
    Serial.printf("[EEPROM] Configuration loaded successfully (Checksum: 0x%08lX).\r\n", settings.checksum);
  }

  // Attach thrusters GP2..GP7
  attachThrusters();

  // Initialize sensors (Wire1 on GP26/GP27)
  Sensors::begin();

  // Initialize LittleFS & Load Mission Trajectory
  Mission::begin();

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
      handleSetRawPulseAll
  };
  CLI::begin(iface);

  // Auto-run on boot check:
  if (settings.autoRunEnabled) {
    fsmState = FsmState::COUNTDOWN;
    countdownStartMs = millis();
    countdownDurationSec = settings.autoRunDelaySec;
    countdownLastSecLogged = -1;
    countdownTareDone = false;
    countdownImuDone = false;
    Serial.println();
    Serial.println(F("\033[1;36m[AUTORUN] Auto-Run on Boot is ENABLED!\033[0m"));
    Serial.printf("[AUTORUN] Launch countdown: %u seconds to launch. Send 'stop' to abort.\r\n", countdownDurationSec);
  } else {
    fsmState = FsmState::IDLE;
  }
}

void loop() {
  const uint32_t now = millis();

  // Watchdog for simulation sensor injection:
  // If $INJ stops for > 500ms, automatically clamp thrusters to neutral (1500us) and revert to physical sensors.
  static bool injectionWasActive = false;
  bool injectionIsActive = Sensors::isInjectionActive();
  if (injectionWasActive && !injectionIsActive) {
    injectionWasActive = false;
    stopAllThrusters();
    if (pidEnabled) {
      pidEnabled = false;
      armed = false;
    }
    CLI::logWarn("Simulation injection watchdog timeout (>500ms) - Thrusters clamped to neutral");
  } else if (injectionIsActive) {
    injectionWasActive = true;
  }

  // 1. Process CLI input (machine packets & human commands)
  CLI::update();

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
