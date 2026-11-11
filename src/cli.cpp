#include "cli.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace CLI {

namespace {

// ANSI Color Codes
constexpr const char* ANSI_RESET   = "\033[0m";
constexpr const char* ANSI_BOLD    = "\033[1m";
constexpr const char* ANSI_RED     = "\033[31m";
constexpr const char* ANSI_GREEN   = "\033[32m";
constexpr const char* ANSI_YELLOW  = "\033[33m";
constexpr const char* ANSI_BLUE    = "\033[34m";
constexpr const char* ANSI_MAGENTA = "\033[35m";
constexpr const char* ANSI_CYAN    = "\033[36m";
constexpr const char* ANSI_WHITE   = "\033[37m";

constexpr const char* ANSI_BOLD_RED    = "\033[1;31m";
constexpr const char* ANSI_BOLD_GREEN  = "\033[1;32m";
constexpr const char* ANSI_BOLD_YELLOW = "\033[1;33m";
constexpr const char* ANSI_BOLD_MAGENTA = "\033[1;35m";
constexpr const char* ANSI_BOLD_CYAN   = "\033[1;36m";
constexpr const char* ANSI_BOLD_WHITE  = "\033[1;37m";

MotorControlInterface iface{};
bool initialized = false;

String inputBuffer = "";
bool sensorStreamActive = false;
uint32_t lastSensorStreamAt = 0;
uint32_t streamStartedAt = 0;

// JSON Upload Engine State
bool jsonUploadMode = false;
String jsonUploadBuffer = "";
uint32_t jsonUploadStartedAt = 0;

// Robust float to String formatter (guaranteed to render regardless of libc printf_float link status)
String fmtFloat(float val, int decimals = 1, int width = 0, bool forceSign = false) {
  String s;
  if (forceSign && val >= 0.0f) {
    s = "+";
    s += String(val, decimals);
  } else {
    s = String(val, decimals);
  }
  if (width > 0) {
    while (static_cast<int>(s.length()) < width) {
      s = " " + s;
    }
  }
  return s;
}

size_t tokenize(const String& cmd, String tokens[], size_t maxTokens) {
  size_t count = 0;
  const int len = cmd.length();
  int i = 0;
  while (i < len && count < maxTokens) {
    while (i < len && (cmd.charAt(i) == ' ' || cmd.charAt(i) == '\t')) {
      i++;
    }
    if (i >= len) break;
    int start = i;
    while (i < len && cmd.charAt(i) != ' ' && cmd.charAt(i) != '\t') {
      i++;
    }
    tokens[count++] = cmd.substring(start, i);
  }
  return count;
}

bool parseBool(const String& str, bool& result) {
  String s = str;
  s.toLowerCase();
  if (s == "1" || s == "on" || s == "true" || s == "yes" || s == "enable") {
    result = true;
    return true;
  }
  if (s == "0" || s == "off" || s == "false" || s == "no" || s == "disable") {
    result = false;
    return true;
  }
  return false;
}

void printHelp() {
  Serial.println();
  Serial.print(ANSI_BOLD_CYAN);
  Serial.println(F("╔═════════════════════════════════════════════════════════════════════════════╗"));
  Serial.println(F("║                   AUV SAUVAC BENCH TESTBED COMMAND MENU                     ║"));
  Serial.println(F("╚═════════════════════════════════════════════════════════════════════════════╝"));
  Serial.print(ANSI_RESET);

  Serial.print(ANSI_BOLD_YELLOW);
  Serial.println(F("\r\n[ SYSTEM & TELEMETRY ]"));
  Serial.print(ANSI_RESET);
  Serial.println(F("  help, ?                    : Display this command reference"));
  Serial.println(F("  status, stat               : Live dashboard (IMU tilt, depth, arm state, limit)"));
  Serial.println(F("  sensor [stream|off]        : One-shot snapshot or live 5Hz continuous telemetry"));
  Serial.println(F("  stop                       : EMERGENCY STOP: Neutralize all motors immediately"));

  Serial.print(ANSI_BOLD_YELLOW);
  Serial.println(F("\r\n[ AUTONOMOUS MISSION & AUTORUN ]"));
  Serial.print(ANSI_RESET);
  Serial.println(F("  run [now]                  : Start autonomous mission (with countdown or 'now')"));
  Serial.println(F("  autorun <on|off>           : Enable/disable auto-run countdown on Pico W boot"));
  Serial.println(F("  autorun delay <3-60>       : Set boot launch countdown delay in seconds (default: 10)"));
  Serial.println(F("  autorun status             : View auto-run configuration and calibration status"));
  Serial.println(F("  jalur, mission             : Display ASCII table of loaded mission steps"));
  Serial.println(F("  jalur default              : Restore default sample trajectory to /jalur.json"));
  Serial.println(F("  jalur upload               : Upload JSON trajectory payload via serial"));

  Serial.print(ANSI_BOLD_YELLOW);
  Serial.println(F("\r\n[ THRUSTER TUNING & BENCH TESTING ]"));
  Serial.print(ANSI_RESET);
  Serial.println(F("  thrusters, motor           : Table of all 6 thrusters (pins, pulses, reverse, trim)"));
  Serial.println(F("  limit <1-100>              : Set global safety power limit (default: 25%)"));
  Serial.println(F("  tes <id 1-6|all> <power>   : Test motor(s) for 2.0s (-limit to +limit %)"));
  Serial.println(F("  thruster <id> <prop> <val> : Fine-tune motor parameters:"));
  Serial.println(F("                               prop: min (800-1400us), max (1600-2200us)"));
  Serial.println(F("                                     neutral (1400-1600us), reverse (0|1)"));
  Serial.println(F("                                     trim (0.10-2.50 speed multiplier)"));

  Serial.print(ANSI_BOLD_YELLOW);
  Serial.println(F("\r\n[ SENSOR CALIBRATION ]"));
  Serial.print(ANSI_RESET);
  Serial.println(F("  calib zero                 : Zero depth sensor reference pressure at surface"));
  Serial.println(F("  calib level                : Zero IMU tilt (level Pitch & Roll offsets)"));

  Serial.print(ANSI_BOLD_YELLOW);
  Serial.println(F("\r\n[ CLOSED-LOOP PID STABILIZATION ]"));
  Serial.print(ANSI_RESET);
  Serial.println(F("  pid <on|off>               : Enable or disable active closed-loop stabilization"));
  Serial.println(F("  pid <axis> <kp> <ki> <kd>  : Configure PID gains (axis: depth, roll, pitch)"));

  Serial.print(ANSI_BOLD_YELLOW);
  Serial.println(F("\r\n[ CONFIGURATION & MEMORY ]"));
  Serial.print(ANSI_RESET);
  Serial.println(F("  save                       : Persist calibration & thruster tuning to EEPROM"));
  Serial.println(F("  default                    : Reset settings to factory defaults"));
  Serial.println();
}

void printStatus() {
  if (!iface.settings) return;
  Sensors::SensorData sensor{};
  Sensors::snapshot(sensor);

  Serial.println();
  Serial.print(ANSI_BOLD_CYAN);
  Serial.println(F("╔═════════════════════════════════════════════════════════════════════════════╗"));
  Serial.println(F("║                     AUV SAUVAC BENCH TESTBED STATUS                         ║"));
  Serial.println(F("╚═════════════════════════════════════════════════════════════════════════════╝"));
  Serial.print(ANSI_RESET);

  // System State
  Serial.print(F("State            : "));
  if (iface.isCountdownActive && iface.isCountdownActive()) {
    Serial.print(ANSI_BOLD_CYAN);
    Serial.printf("[ COUNTDOWN ACTIVE (T-%ds) ]", iface.getCountdownRemaining ? iface.getCountdownRemaining() : 0);
  } else if (iface.isMissionActive && iface.isMissionActive()) {
    Serial.print(ANSI_BOLD_GREEN);
    Serial.print(F("[ MISSION RUNNING ]"));
  } else if (iface.armed && *iface.armed) {
    Serial.print(ANSI_BOLD_GREEN);
    Serial.print(F("[ ARMED / ACTIVE ]"));
  } else {
    Serial.print(ANSI_BOLD_YELLOW);
    Serial.print(F("[ DISARMED / STANDBY ]"));
  }
  Serial.print(ANSI_RESET);

  Serial.print(F("  |  PID Stabilization: "));
  if (iface.pidEnabled && *iface.pidEnabled) {
    Serial.print(ANSI_BOLD_GREEN);
    Serial.println(F("[ ON ]"));
  } else {
    Serial.print(ANSI_BOLD_YELLOW);
    Serial.println(F("[ OFF ]"));
  }
  Serial.print(ANSI_RESET);

  // Auto-Run and Mission info
  Serial.print(F("Auto-Run on Boot : "));
  if (iface.settings->autoRunEnabled) {
    Serial.print(ANSI_BOLD_GREEN);
    Serial.printf("ENABLED (%u s countdown delay)\r\n", iface.settings->autoRunDelaySec);
  } else {
    Serial.print(ANSI_YELLOW);
    Serial.println(F("DISABLED"));
  }
  Serial.print(ANSI_RESET);

  Serial.printf("Mission Jalur    : %u steps loaded (%.1f s total duration)\r\n",
                static_cast<unsigned>(Mission::getStepCount()),
                Mission::getTotalDurationSec());

  // Power Limit
  Serial.print(F("Safety Cap       : "));
  Serial.print(ANSI_BOLD_CYAN);
  Serial.print(iface.settings->powerLimitPercent);
  Serial.print(F("%"));
  Serial.print(ANSI_RESET);
  if (iface.settings->powerLimitPercent <= 25) {
    Serial.print(ANSI_GREEN);
    Serial.println(F(" (SAFE BENCH LIMIT ACTIVE)"));
  } else {
    Serial.print(ANSI_BOLD_RED);
    Serial.println(F(" (WARNING: ELEVATED LIMIT)"));
  }
  Serial.print(ANSI_RESET);

  Serial.println(F("-------------------------------------------------------------------------------"));
  if (Sensors::isInjectionActive()) {
    Serial.print(ANSI_BOLD_MAGENTA);
    Serial.println(F("  *** SIMULATION SENSOR INJECTION ACTIVE ***"));
    Serial.print(ANSI_RESET);
  }
  Serial.print(ANSI_BOLD_WHITE);
  Serial.println(F("[ SENSOR TELEMETRY ]"));
  Serial.print(ANSI_RESET);

  // MS5803
  Serial.print(F("  MS5803 Depth   : "));
  if (sensor.pressureReady) {
    Serial.print(ANSI_GREEN);
    Serial.print(F("ONLINE"));
    Serial.print(ANSI_RESET);
    Serial.printf(" | Pressure: %s mbar | Temp: %s °C\r\n",
                  fmtFloat(sensor.pressureMbar, 1, 6).c_str(),
                  fmtFloat(sensor.temperatureC, 1, 4).c_str());
    Serial.printf("                   Depth   : %s cm ",
                  fmtFloat(sensor.depthCm, 1, 6).c_str());
    if (iface.settings->depthZeroCalibrated) {
      Serial.printf("(Zero Ref: %s mbar - CALIBRATED)\r\n",
                    fmtFloat(iface.settings->depthZeroPressureMbar, 1).c_str());
    } else {
      Serial.print(ANSI_YELLOW);
      Serial.println(F("(NOT ZEROED - Run 'calib zero' at surface)"));
      Serial.print(ANSI_RESET);
    }
  } else {
    Serial.print(ANSI_RED);
    Serial.println(F("OFFLINE / NOT DETECTED (Check I2C SDA=GP26, SCL=GP27)"));
    Serial.print(ANSI_RESET);
  }

  // MPU-6050
  Serial.print(F("  MPU-6050 IMU   : "));
  if (sensor.imuReady) {
    Serial.print(ANSI_GREEN);
    Serial.print(F("ONLINE"));
    Serial.print(ANSI_RESET);
    Serial.printf(" | Pitch: %s° (raw %s°, off %s°)\r\n",
                  fmtFloat(sensor.pitch, 1, 5, true).c_str(),
                  fmtFloat(sensor.rawPitch, 1, 5, true).c_str(),
                  fmtFloat(iface.settings->pitchOffset, 1, 4, true).c_str());
    Serial.printf("                   Roll : %s° (raw %s°, off %s°)\r\n",
                  fmtFloat(sensor.roll, 1, 5, true).c_str(),
                  fmtFloat(sensor.rawRoll, 1, 5, true).c_str(),
                  fmtFloat(iface.settings->rollOffset, 1, 4, true).c_str());
    Serial.printf("                   Gyro : X=%s, Y=%s, Z=%s dps\r\n",
                  fmtFloat(sensor.gyroDps[0], 1, 5, true).c_str(),
                  fmtFloat(sensor.gyroDps[1], 1, 5, true).c_str(),
                  fmtFloat(sensor.gyroDps[2], 1, 5, true).c_str());
  } else {
    Serial.print(ANSI_RED);
    Serial.println(F("OFFLINE / NOT DETECTED (Check I2C SDA=GP26, SCL=GP27)"));
    Serial.print(ANSI_RESET);
  }

  Serial.println(F("-------------------------------------------------------------------------------"));
  Serial.print(ANSI_BOLD_WHITE);
  Serial.println(F("[ ACTIVE THRUSTER PULSES ]"));
  Serial.print(ANSI_RESET);
  for (size_t i = 0; i < Config::THRUSTER_COUNT; ++i) {
    uint16_t pulse = iface.getCurrentPulse ? iface.getCurrentPulse(i) : iface.settings->thrusters[i].pulseNeutral;
    Serial.printf("  T%u (GP%u): %4uus %s",
                  static_cast<unsigned>(i + 1),
                  iface.settings->thrusters[i].pin,
                  pulse,
                  (i == 2 || i == 5) ? "\r\n" : " | ");
  }

  Serial.println(F("-------------------------------------------------------------------------------"));
  Serial.print(ANSI_BOLD_WHITE);
  Serial.println(F("[ ACTIVE PID GAINS ]"));
  Serial.print(ANSI_RESET);
  Serial.printf("  Depth : Kp=%s, Ki=%s, Kd=%s\r\n",
                fmtFloat(iface.settings->pidDepth.kp, 3).c_str(),
                fmtFloat(iface.settings->pidDepth.ki, 3).c_str(),
                fmtFloat(iface.settings->pidDepth.kd, 3).c_str());
  Serial.printf("  Roll  : Kp=%s, Ki=%s, Kd=%s\r\n",
                fmtFloat(iface.settings->pidRoll.kp, 3).c_str(),
                fmtFloat(iface.settings->pidRoll.ki, 3).c_str(),
                fmtFloat(iface.settings->pidRoll.kd, 3).c_str());
  Serial.printf("  Pitch : Kp=%s, Ki=%s, Kd=%s\r\n",
                fmtFloat(iface.settings->pidPitch.kp, 3).c_str(),
                fmtFloat(iface.settings->pidPitch.ki, 3).c_str(),
                fmtFloat(iface.settings->pidPitch.kd, 3).c_str());
  Serial.println(F("═══════════════════════════════════════════════════════════════════════════════"));
}

void printThrusterTable() {
  if (!iface.settings) return;

  Serial.println();
  Serial.print(ANSI_BOLD_CYAN);
  Serial.println(F("╔═════════════════════════════════════════════════════════════════════════════╗"));
  Serial.println(F("║                   AUV SAUVAC 6-THRUSTER CONFIGURATION                       ║"));
  Serial.println(F("╚═════════════════════════════════════════════════════════════════════════════╝"));
  Serial.print(ANSI_RESET);

  Serial.println(F("+----+------+----------+--------------+----------+---------+--------------------+"));
  Serial.println(F("| ID | Pin  | Min (us) | Neutral (us) | Max (us) | Reverse | Speed Trim / Scale |"));
  Serial.println(F("+----+------+----------+--------------+----------+---------+--------------------+"));

  for (size_t i = 0; i < Config::THRUSTER_COUNT; ++i) {
    const auto& t = iface.settings->thrusters[i];
    Serial.printf("|  %u | GP%-2u |     %4u |         %4u |     %4u |   %s   |        %sx       |\r\n",
                  static_cast<unsigned>(i + 1),
                  t.pin,
                  t.pulseMin,
                  t.pulseNeutral,
                  t.pulseMax,
                  t.reverse ? "YES" : " NO",
                  fmtFloat(t.trim, 2, 4).c_str());
  }
  Serial.println(F("+----+------+----------+--------------+----------+---------+--------------------+"));
  Serial.printf("Global Safety Power Limit: %u%% | Cap Status: %s\r\n\r\n",
                iface.settings->powerLimitPercent,
                iface.settings->powerLimitPercent <= 25 ? "ACTIVE (Safe 25% Cap)" : "OVERRIDDEN (>25%)");
}

void printSensorSnapshot() {
  Sensors::SensorData s{};
  Sensors::snapshot(s);
  Serial.printf("[SENSOR SNAPSHOT] Depth: %s cm | Pres: %s mbar | Pitch: %s° | Roll: %s° | Gyro: [%s, %s, %s] dps | Temp: %s°C\r\n",
                fmtFloat(s.depthCm, 1, 5).c_str(),
                fmtFloat(s.pressureMbar, 1, 6).c_str(),
                fmtFloat(s.pitch, 1, 5, true).c_str(),
                fmtFloat(s.roll, 1, 5, true).c_str(),
                fmtFloat(s.gyroDps[0], 1, 5, true).c_str(),
                fmtFloat(s.gyroDps[1], 1, 5, true).c_str(),
                fmtFloat(s.gyroDps[2], 1, 5, true).c_str(),
                fmtFloat(s.temperatureC, 1, 4).c_str());
}

void handleCommand(const String& rawCmd) {
  String cmd = rawCmd;
  cmd.trim();
  if (cmd.startsWith("/")) {
    cmd = cmd.substring(1);
    cmd.trim();
  }
  if (cmd.isEmpty()) return;

  String tokens[8];
  size_t tokenCount = tokenize(cmd, tokens, 8);
  if (tokenCount == 0) return;

  String root = tokens[0];
  root.toLowerCase();

  // 1. HELP / ?
  if (root == "help" || root == "?") {
    printHelp();
    return;
  }

  // 2. STATUS / STAT
  if (root == "status" || root == "stat") {
    printStatus();
    return;
  }

  // 3. SENSOR / SENSORS
  if (root == "sensor" || root == "sensors") {
    if (tokenCount > 1) {
      String sub = tokens[1];
      sub.toLowerCase();
      if (sub == "stream" || sub == "on" || sub == "1") {
        sensorStreamActive = true;
        streamStartedAt = millis();
        Serial.print(ANSI_GREEN);
        Serial.println(F("[STREAM] Live sensor stream STARTED (5Hz). Press any key to stop."));
        Serial.print(ANSI_RESET);
        return;
      } else if (sub == "off" || sub == "0" || sub == "stop") {
        sensorStreamActive = false;
        Serial.println(F("[STREAM] Live sensor stream STOPPED."));
        return;
      }
    }
    printSensorSnapshot();
    return;
  }

  // 4. THRUSTERS / MOTOR
  if (root == "thrusters" || root == "motor" || root == "motors") {
    printThrusterTable();
    return;
  }

  // 5. LIMIT <1-100>
  if (root == "limit") {
    if (tokenCount < 2) {
      Serial.printf("Current safety power limit: %u%%\r\n", iface.settings->powerLimitPercent);
      return;
    }
    long val = tokens[1].toInt();
    if (val < 1 || val > 100) {
      Serial.print(ANSI_RED);
      Serial.println(F("ERROR: Power limit must be between 1 and 100%."));
      Serial.print(ANSI_RESET);
      return;
    }
    iface.settings->powerLimitPercent = static_cast<uint8_t>(val);
    if (val <= 25) {
      Serial.print(ANSI_GREEN);
      Serial.printf("OK: Safety power limit set to %ld%% (Safe bench testing cap active).\r\n", val);
    } else if (val <= 50) {
      Serial.print(ANSI_YELLOW);
      Serial.printf("CAUTION: Safety power limit set to %ld%%. Proceed with care.\r\n", val);
    } else {
      Serial.print(ANSI_BOLD_RED);
      Serial.printf("WARNING: High power limit set to %ld%%! Ensure motors are clear of hazards.\r\n", val);
    }
    Serial.print(ANSI_RESET);
    Serial.println(F("Type 'save' to persist to EEPROM."));
    return;
  }

  // 6. TES / TEST <id 1-6|all> <power>
  if (root == "tes" || root == "test") {
    if (tokenCount < 3) {
      Serial.print(ANSI_RED);
      Serial.println(F("ERROR: Usage: tes <id 1-6|all> <power percent> (e.g. 'tes 1 20' or 'tes all 15')"));
      Serial.print(ANSI_RESET);
      return;
    }

    uint8_t id = 0;
    if (tokens[1].equalsIgnoreCase("all")) {
      id = 0;  // 0 signifies all motors
    } else {
      id = static_cast<uint8_t>(tokens[1].toInt());
      if (id < 1 || id > Config::THRUSTER_COUNT) {
        Serial.print(ANSI_RED);
        Serial.println(F("ERROR: Motor ID must be 1 to 6 or 'all'."));
        Serial.print(ANSI_RESET);
        return;
      }
    }

    String powerStr = tokens[2];
    if (powerStr.endsWith("%")) {
      powerStr = powerStr.substring(0, powerStr.length() - 1);
    }
    float requestedPower = powerStr.toFloat();

    if (!iface.startMotorTest) {
      Serial.println(F("ERROR: Motor test callback unavailable."));
      return;
    }

    String testMsg;
    if (iface.startMotorTest(id, requestedPower, testMsg)) {
      Serial.print(ANSI_GREEN);
      Serial.println(testMsg);
      Serial.print(ANSI_RESET);
    } else {
      Serial.print(ANSI_RED);
      Serial.println(testMsg);
      Serial.print(ANSI_RESET);
    }
    return;
  }

  // 7. THRUSTER <id 1-6> <min|max|neutral|reverse|trim> <val>
  if (root == "thruster") {
    if (tokenCount < 2) {
      printThrusterTable();
      return;
    }
    if (tokenCount < 4) {
      Serial.print(ANSI_RED);
      Serial.println(F("ERROR: Usage: thruster <id 1-6> <min|max|neutral|reverse|trim> <value>"));
      Serial.print(ANSI_RESET);
      return;
    }

    int id = tokens[1].toInt();
    if (id < 1 || id > static_cast<int>(Config::THRUSTER_COUNT)) {
      Serial.print(ANSI_RED);
      Serial.println(F("ERROR: Thruster ID must be between 1 and 6."));
      Serial.print(ANSI_RESET);
      return;
    }

    size_t idx = id - 1;
    String prop = tokens[2];
    prop.toLowerCase();
    String valStr = tokens[3];
    auto& t = iface.settings->thrusters[idx];

    if (prop == "min") {
      long val = valStr.toInt();
      if (val < 800 || val > 1400) {
        Serial.println(F("ERROR: Min pulse must be between 800 and 1400 us."));
        return;
      }
      t.pulseMin = static_cast<uint16_t>(val);
    } else if (prop == "max") {
      long val = valStr.toInt();
      if (val < 1600 || val > 2200) {
        Serial.println(F("ERROR: Max pulse must be between 1600 and 2200 us."));
        return;
      }
      t.pulseMax = static_cast<uint16_t>(val);
    } else if (prop == "neutral") {
      long val = valStr.toInt();
      if (val < 1400 || val > 1600) {
        Serial.println(F("ERROR: Neutral pulse must be between 1400 and 1600 us."));
        return;
      }
      t.pulseNeutral = static_cast<uint16_t>(val);
    } else if (prop == "reverse") {
      bool b = false;
      if (!parseBool(valStr, b)) {
        Serial.println(F("ERROR: Reverse must be 0/1 or on/off."));
        return;
      }
      t.reverse = b ? 1 : 0;
    } else if (prop == "trim") {
      float val = valStr.toFloat();
      if (val < 0.10f || val > 2.50f) {
        Serial.println(F("ERROR: Trim scale multiplier must be between 0.10 and 2.50."));
        return;
      }
      t.trim = val;
    } else {
      Serial.printf("ERROR: Unknown property '%s'. Must be min, max, neutral, reverse, or trim.\r\n", prop.c_str());
      return;
    }

    Serial.print(ANSI_GREEN);
    Serial.printf("OK: Thruster T%d %s set to %s. Type 'save' to commit to EEPROM.\r\n", id, prop.c_str(), valStr.c_str());
    Serial.print(ANSI_RESET);
    return;
  }

  // 8. CALIB ZERO / LEVEL
  if (root == "calib") {
    if (tokenCount < 2) {
      Serial.println(F("ERROR: Usage: calib zero (tare depth) or calib level (zero IMU tilt)"));
      return;
    }
    String action = tokens[1];
    action.toLowerCase();
    if (action == "zero") {
      if (tokenCount >= 3) {
        float manualPres = tokens[2].toFloat();
        if (manualPres >= 500.0f && manualPres <= 2000.0f) {
          iface.settings->depthZeroPressureMbar = manualPres;
          iface.settings->depthZeroCalibrated = 1;
          Serial.print(ANSI_GREEN);
          Serial.printf("OK: Surface depth reference pressure set to %s mbar (manual). Depth zeroed!\r\n",
                        fmtFloat(manualPres, 1).c_str());
          Serial.print(ANSI_RESET);
          Serial.println(F("Type 'save' to commit to EEPROM."));
          return;
        } else {
          Serial.println(F("ERROR: Pressure must be between 500 and 2000 mbar."));
          return;
        }
      }
      if (Sensors::calibrateDepthZero(*iface.settings)) {
        Serial.print(ANSI_GREEN);
        Serial.printf("OK: Surface depth reference pressure set to %s mbar. Depth zeroed!\r\n",
                      fmtFloat(iface.settings->depthZeroPressureMbar, 1).c_str());
        Serial.print(ANSI_RESET);
        Serial.println(F("Type 'save' to commit to EEPROM."));
      } else {
        Serial.print(ANSI_RED);
        Serial.println(F("ERROR: MS5803 sensor offline or invalid reading. (Tip: 'calib zero <mbar>' for desk bench testing)"));
        Serial.print(ANSI_RESET);
      }
      return;
    } else if (action == "level") {
      if (tokenCount >= 4) {
        float pOff = tokens[2].toFloat();
        float rOff = tokens[3].toFloat();
        iface.settings->pitchOffset = pOff;
        iface.settings->rollOffset = rOff;
        Serial.print(ANSI_GREEN);
        Serial.printf("OK: IMU level offsets set to Pitch: %s°, Roll: %s° (manual). Robot leveled!\r\n",
                      fmtFloat(pOff, 2, 0, true).c_str(), fmtFloat(rOff, 2, 0, true).c_str());
        Serial.print(ANSI_RESET);
        Serial.println(F("Type 'save' to commit to EEPROM."));
        return;
      }
      if (Sensors::calibrateImuLevel(*iface.settings)) {
        Serial.print(ANSI_GREEN);
        Serial.printf("OK: IMU level offsets recorded (Pitch: %s°, Roll: %s°). Robot leveled!\r\n",
                      fmtFloat(iface.settings->pitchOffset, 2, 0, true).c_str(),
                      fmtFloat(iface.settings->rollOffset, 2, 0, true).c_str());
        Serial.print(ANSI_RESET);
        Serial.println(F("Type 'save' to commit to EEPROM."));
      } else {
        Serial.print(ANSI_RED);
        Serial.println(F("ERROR: MPU-6050 sensor offline or invalid. (Tip: 'calib level <pitch> <roll>' for desk bench testing)"));
        Serial.print(ANSI_RESET);
      }
      return;
    } else {
      Serial.printf("ERROR: Unknown calib target '%s'. Use 'calib zero' or 'calib level'.\r\n", action.c_str());
      return;
    }
  }

  // 9. PID <depth|roll|pitch> <kp> <ki> <kd> OR pid <on|off>
  if (root == "pid") {
    if (tokenCount < 2) {
      Serial.println(F("ERROR: Usage: pid <on|off> OR pid <depth|roll|pitch> <kp> <ki> <kd>"));
      return;
    }
    String sub = tokens[1];
    sub.toLowerCase();
    if (sub == "on" || sub == "1" || sub == "enable") {
      if (iface.setPidState) iface.setPidState(true);
      Serial.print(ANSI_GREEN);
      Serial.println(F("[PID] Active closed-loop stabilization ENABLED and ARMED."));
      Serial.println(F("      Tilt or disturbance will now drive thrusters to restore level/depth trim."));
      Serial.print(ANSI_RESET);
      return;
    } else if (sub == "off" || sub == "0" || sub == "disable") {
      if (iface.setPidState) iface.setPidState(false);
      Serial.print(ANSI_YELLOW);
      Serial.println(F("[PID] Active closed-loop stabilization DISABLED and DISARMED. Thrusters neutral."));
      Serial.print(ANSI_RESET);
      return;
    } else if (sub == "depth" || sub == "roll" || sub == "pitch") {
      if (tokenCount < 5) {
        Serial.printf("ERROR: Usage: pid %s <kp> <ki> <kd>\r\n", sub.c_str());
        return;
      }
      float kp = tokens[2].toFloat();
      float ki = tokens[3].toFloat();
      float kd = tokens[4].toFloat();

      if (sub == "depth") {
        iface.settings->pidDepth = {kp, ki, kd};
      } else if (sub == "roll") {
        iface.settings->pidRoll = {kp, ki, kd};
      } else {
        iface.settings->pidPitch = {kp, ki, kd};
      }

      Serial.print(ANSI_GREEN);
      Serial.printf("OK: %s PID gains set to Kp=%s, Ki=%s, Kd=%s. Type 'save' to commit.\r\n",
                    sub.c_str(),
                    fmtFloat(kp, 4).c_str(),
                    fmtFloat(ki, 4).c_str(),
                    fmtFloat(kd, 4).c_str());
      Serial.print(ANSI_RESET);
      return;
    } else {
      Serial.printf("ERROR: Unknown pid parameter '%s'. Must be on/off or depth/roll/pitch.\r\n", sub.c_str());
      return;
    }
  }

  // 10. SAVE
  if (root == "save") {
    if (iface.saveSettings && iface.saveSettings()) {
      Serial.print(ANSI_BOLD_GREEN);
      Serial.printf("[EEPROM] Configuration and calibrations successfully committed! Checksum: 0x%08lX\r\n",
                    iface.settings->checksum);
      Serial.print(ANSI_RESET);
    } else {
      Serial.print(ANSI_RED);
      Serial.println(F("[EEPROM] ERROR: Failed to write configuration to non-volatile EEPROM."));
      Serial.print(ANSI_RESET);
    }
    return;
  }

  // 11. DEFAULT
  if (root == "default") {
    if (iface.resetDefaults) iface.resetDefaults();
    Serial.print(ANSI_YELLOW);
    Serial.println(F("[CFG] Factory default settings restored in RAM. Type 'save' to persist to EEPROM."));
    Serial.print(ANSI_RESET);
    return;
  }

  // 12. STOP / DISARM
  if (root == "stop" || root == "disarm" || root == "abort") {
    if (iface.cancelMission) iface.cancelMission();
    if (iface.emergencyStop) iface.emergencyStop();
    Serial.print(ANSI_BOLD_RED);
    Serial.println(F("[EMERGENCY STOP] All 6 thrusters neutralized immediately (1500us). Disarmed."));
    Serial.print(ANSI_RESET);
    return;
  }

  // 13. AUTORUN
  if (root == "autorun") {
    if (tokenCount < 2 || tokens[1].equalsIgnoreCase("status")) {
      Serial.println();
      Serial.print(ANSI_BOLD_CYAN);
      Serial.println(F("╔═════════════════════════════════════════════════════════════════════════════╗"));
      Serial.println(F("║                   AUTORUN & AUTONOMOUS MISSION STATUS                       ║"));
      Serial.println(F("╚═════════════════════════════════════════════════════════════════════════════╝"));
      Serial.print(ANSI_RESET);

      Serial.print(F("Auto-Run on Boot : "));
      if (iface.settings->autoRunEnabled) {
        Serial.print(ANSI_BOLD_GREEN);
        Serial.println(F("ENABLED"));
      } else {
        Serial.print(ANSI_YELLOW);
        Serial.println(F("DISABLED"));
      }
      Serial.print(ANSI_RESET);

      Serial.printf("Countdown Delay  : %u seconds (range: 3-60s)\r\n", iface.settings->autoRunDelaySec);
      Serial.println(F("Surface Tare     : Automatic at T-3s (MS5803 depth zero calibration)"));
      Serial.println(F("IMU Leveling     : Automatic at T-2s (MPU-6050 Pitch/Roll zero calibration)"));
      Serial.printf("Trajectory File  : /jalur.json (%u steps loaded, %.1f s duration)\r\n",
                    static_cast<unsigned>(Mission::getStepCount()),
                    Mission::getTotalDurationSec());
      Serial.println(F("EEPROM Status    : Persist settings across boots with 'save' command."));
      Serial.println();
      return;
    }

    String sub = tokens[1];
    sub.toLowerCase();
    if (sub == "on" || sub == "1" || sub == "enable") {
      iface.settings->autoRunEnabled = 1;
      Serial.print(ANSI_GREEN);
      Serial.printf("OK: Auto-run on boot ENABLED (%u s countdown). Type 'save' to commit to EEPROM.\r\n",
                    iface.settings->autoRunDelaySec);
      Serial.print(ANSI_RESET);
      return;
    } else if (sub == "off" || sub == "0" || sub == "disable") {
      iface.settings->autoRunEnabled = 0;
      Serial.print(ANSI_YELLOW);
      Serial.println(F("OK: Auto-run on boot DISABLED. Type 'save' to commit to EEPROM."));
      Serial.print(ANSI_RESET);
      return;
    } else if (sub == "delay") {
      if (tokenCount < 3) {
        Serial.println(F("Usage: autorun delay <seconds 3-60>"));
        return;
      }
      int sec = tokens[2].toInt();
      if (sec < 3 || sec > 60) {
        Serial.print(ANSI_RED);
        Serial.println(F("ERROR: Delay must be between 3 and 60 seconds."));
        Serial.print(ANSI_RESET);
        return;
      }
      iface.settings->autoRunDelaySec = static_cast<uint8_t>(sec);
      Serial.print(ANSI_GREEN);
      Serial.printf("OK: Countdown delay set to %d seconds. Type 'save' to commit to EEPROM.\r\n", sec);
      Serial.print(ANSI_RESET);
      return;
    } else {
      Serial.printf("ERROR: Unknown autorun option '%s'. Use 'on', 'off', 'delay <sec>', or 'status'.\r\n", sub.c_str());
      return;
    }
  }

  // 14. RUN
  if (root == "run") {
    if (tokenCount > 1 && tokens[1].equalsIgnoreCase("now")) {
      if (iface.startMission) iface.startMission(true);
      return;
    }
    if (iface.startMission) iface.startMission(false);
    return;
  }

  // 15. JALUR / MISSION
  if (root == "jalur" || root == "mission") {
    if (tokenCount > 1) {
      String sub = tokens[1];
      sub.toLowerCase();
      if (sub == "default" || sub == "reset") {
        if (Mission::saveDefaultFile()) {
          Serial.print(ANSI_GREEN);
          Serial.println(F("OK: Default trajectory restored and saved to /jalur.json!"));
          Serial.print(ANSI_RESET);
          Mission::printMissionTable();
        } else {
          Serial.print(ANSI_RED);
          Serial.println(F("ERROR: Failed to save default trajectory to /jalur.json on LittleFS."));
          Serial.print(ANSI_RESET);
        }
        return;
      } else if (sub == "upload") {
        jsonUploadMode = true;
        jsonUploadBuffer = "";
        jsonUploadStartedAt = millis();
        Serial.print(ANSI_BOLD_CYAN);
        Serial.println(F("[UPLOAD] READY. Send JSON payload followed by '===EOF===' (30s timeout)..."));
        Serial.print(ANSI_RESET);
        return;
      }
    }
    Mission::printMissionTable();
    return;
  }

  // 15. DEPTH / TARGET DEPTH
  if (root == "depth" || (root == "target" && tokenCount > 1 && tokens[1].equalsIgnoreCase("depth"))) {
    size_t valIdx = (root == "depth") ? 1 : 2;
    if (tokenCount <= valIdx) {
      Sensors::SensorData s{};
      Sensors::snapshot(s);
      Serial.printf("Current Depth: %.1f cm", s.depthCm);
      if (iface.getTargetDepth) {
        Serial.printf(" | Target Depth: %.1f cm", iface.getTargetDepth());
      }
      Serial.println();
      return;
    }
    float target = tokens[valIdx].toFloat();
    if (iface.setTargetDepth) {
      iface.setTargetDepth(target);
      Serial.printf("[DEPTH] Target depth set to %.1f cm.\r\n", target);
    }
    return;
  }

  Serial.print(ANSI_RED);
  Serial.printf("ERROR: Unknown command '%s'. Type 'help' or '?' for available commands.\r\n", tokens[0].c_str());
  Serial.print(ANSI_RESET);
}

void handleInjPacket(const String& packet) {
  int starIdx = packet.indexOf('*');
  String clean = (starIdx >= 0) ? packet.substring(0, starIdx) : packet;

  const char* p = clean.c_str();
  if (strncmp(p, "$INJ,", 5) != 0) {
    return;
  }
  p += 5;

  char* endp = nullptr;
  float depth = strtof(p, &endp);
  if (endp && *endp == ',') p = endp + 1;
  float pitch = strtof(p, &endp);
  if (endp && *endp == ',') p = endp + 1;
  float roll = strtof(p, &endp);
  if (endp && *endp == ',') p = endp + 1;
  float gx = strtof(p, &endp);
  if (endp && *endp == ',') p = endp + 1;
  float gy = strtof(p, &endp);
  if (endp && *endp == ',') p = endp + 1;
  float gz = strtof(p, &endp);

  Sensors::SensorData data{};
  data.depthCm = depth;
  data.pitch = pitch;
  data.roll = roll;
  data.rawPitch = pitch;
  data.rawRoll = roll;
  data.gyroDps[0] = gx;
  data.gyroDps[1] = gy;
  data.gyroDps[2] = gz;

  float pitchRad = pitch * 0.0174532925f;
  float rollRad = roll * 0.0174532925f;
  data.accelG[0] = -sinf(pitchRad);
  data.accelG[1] = sinf(rollRad) * cosf(pitchRad);
  data.accelG[2] = cosf(rollRad) * cosf(pitchRad);
  data.pressureMbar = 1013.25f + (depth * 0.980665f);
  data.temperatureC = 25.0f;
  data.imuReady = true;
  data.pressureReady = true;
  data.i2cReady = true;
  data.lastUpdateMs = millis();
  data.sampleCount++;

  Sensors::setInjectedData(data);

  if (iface.onInjected) {
    iface.onInjected();
  }

  char actBuf[64];
  snprintf(actBuf, sizeof(actBuf), "$ACT,%u,%u,%u,%u,%u,%u\r\n",
           iface.getCurrentPulse ? iface.getCurrentPulse(0) : 1500,
           iface.getCurrentPulse ? iface.getCurrentPulse(1) : 1500,
           iface.getCurrentPulse ? iface.getCurrentPulse(2) : 1500,
           iface.getCurrentPulse ? iface.getCurrentPulse(3) : 1500,
           iface.getCurrentPulse ? iface.getCurrentPulse(4) : 1500,
           iface.getCurrentPulse ? iface.getCurrentPulse(5) : 1500);
  Serial.print(actBuf);
}

}  // namespace

void begin(const MotorControlInterface& controlInterface) {
  iface = controlInterface;
  initialized = true;
  printBanner();
  printPrompt();
}

void printBanner() {
  Serial.println();
  Serial.print(ANSI_BOLD_CYAN);
  Serial.println(F("╔═════════════════════════════════════════════════════════════════════╗"));
  Serial.println(F("║               AUV SAUVAC BENCH TESTBED (REV 0.1)                    ║"));
  Serial.println(F("║          Lightweight 6-Thruster Precision Control & Tuning          ║"));
  Serial.println(F("╚═════════════════════════════════════════════════════════════════════╝"));
  Serial.print(ANSI_RESET);
  Serial.print(ANSI_GREEN);
  Serial.println(F("System initialized at 115200 baud. Zero network overhead."));
  Serial.println(F("Type 'help' or '?' for available bench testbed commands."));
  Serial.print(ANSI_RESET);
  Serial.println();
}

void printPrompt() {
  Serial.print(ANSI_BOLD_CYAN);
  Serial.print(F("AUV> "));
  Serial.print(ANSI_RESET);
}

void update() {
  // Check for JSON Upload Mode (transferring /jalur.json over Serial)
  if (jsonUploadMode) {
    if (millis() - jsonUploadStartedAt > 30000) {
      jsonUploadMode = false;
      jsonUploadBuffer = "";
      Serial.println(F("\r\n[UPLOAD] ERROR: Timeout waiting for JSON payload (30s)."));
      printPrompt();
      return;
    }

    while (Serial.available()) {
      char c = static_cast<char>(Serial.read());
      jsonUploadBuffer += c;

      if (jsonUploadBuffer.endsWith("===EOF===\r\n") || jsonUploadBuffer.endsWith("===EOF===\n") ||
          jsonUploadBuffer.endsWith("@EOF\r\n") || jsonUploadBuffer.endsWith("@EOF\n")) {
        int eofIdx = jsonUploadBuffer.indexOf("===EOF===");
        if (eofIdx < 0) eofIdx = jsonUploadBuffer.indexOf("@EOF");
        if (eofIdx >= 0) {
          jsonUploadBuffer = jsonUploadBuffer.substring(0, eofIdx);
        }
        jsonUploadBuffer.trim();

        jsonUploadMode = false;
        String err;
        if (Mission::saveJsonString(jsonUploadBuffer, err)) {
          Serial.println(F("\r\n[UPLOAD] SUCCESS: Mission trajectory updated in /jalur.json!"));
          Mission::printMissionTable();
        } else {
          Serial.printf("\r\n[UPLOAD] ERROR: Failed to save JSON: %s\r\n", err.c_str());
        }
        jsonUploadBuffer = "";
        printPrompt();
        return;
      }
    }
    return;
  }

  // Check for incoming serial characters
  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());

    // If streaming sensor data, any key stops the stream
    if (sensorStreamActive) {
      // Discard leftover CRLF from the Enter keypress during the first 350 ms
      if (millis() - streamStartedAt < 350) {
        continue;
      }
      // Ignore lone newline or carriage return if they arrive
      if (c == '\r' || c == '\n') {
        continue;
      }
      sensorStreamActive = false;
      Serial.println(F("\r\n[STREAM] Sensor stream stopped by user keypress."));
      printPrompt();
      continue;
    }

    if (c == '\r' || c == '\n') {
      if (inputBuffer.length() > 0) {
        if (inputBuffer.startsWith("$INJ")) {
          handleInjPacket(inputBuffer);
          inputBuffer = "";
          continue;
        }

        Serial.println();

        // Check if countdown is active: any command or 'stop' aborts countdown immediately!
        if (iface.isCountdownActive && iface.isCountdownActive()) {
          if (iface.cancelMission) iface.cancelMission();
          Serial.println(F("[AUTORUN] Countdown ABORTED by operator. System returned to IDLE mode."));
          String cmd = inputBuffer;
          inputBuffer = "";
          if (!cmd.equalsIgnoreCase("stop") && !cmd.equalsIgnoreCase("abort") && !cmd.equalsIgnoreCase("disarm")) {
            handleCommand(cmd);
          }
          printPrompt();
          return;
        }

        handleCommand(inputBuffer);
        inputBuffer = "";
        printPrompt();
      }
    } else if (c == '\b' || c == 0x7F) {
      if (inputBuffer.length() > 0) {
        inputBuffer.remove(inputBuffer.length() - 1);
        if (!inputBuffer.startsWith("$")) {
          Serial.print(F("\b \b"));  // Visual backspace
        }
      }
    } else if (c >= 32 && c <= 126) {
      inputBuffer += c;
      if (!inputBuffer.startsWith("$")) {
        Serial.print(c);  // Echo character
      }
    }
  }

  // Handle sensor streaming if active (5 Hz / 200 ms)
  if (sensorStreamActive) {
    const uint32_t now = millis();
    if (now - lastSensorStreamAt >= 200) {
      lastSensorStreamAt = now;
      Sensors::SensorData s{};
      Sensors::snapshot(s);
      Serial.printf("[LIVE] P: %s° | R: %s° | Depth: %s cm | Pres: %s mbar | Gyro: [%s, %s, %s] dps\r\n",
                    fmtFloat(s.pitch, 1, 5, true).c_str(),
                    fmtFloat(s.roll, 1, 5, true).c_str(),
                    fmtFloat(s.depthCm, 1, 5).c_str(),
                    fmtFloat(s.pressureMbar, 1, 6).c_str(),
                    fmtFloat(s.gyroDps[0], 1, 5, true).c_str(),
                    fmtFloat(s.gyroDps[1], 1, 5, true).c_str(),
                    fmtFloat(s.gyroDps[2], 1, 5, true).c_str());
    }
  }
}

void logInfo(const char* fmt, ...) {
  char buf[128];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.print(ANSI_GREEN);
  Serial.print(F("[INFO] "));
  Serial.print(ANSI_RESET);
  Serial.println(buf);
}

void logWarn(const char* fmt, ...) {
  char buf[128];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.print(ANSI_YELLOW);
  Serial.print(F("[WARN] "));
  Serial.print(ANSI_RESET);
  Serial.println(buf);
}

void logError(const char* fmt, ...) {
  char buf[128];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.print(ANSI_RED);
  Serial.print(F("[ERROR] "));
  Serial.print(ANSI_RESET);
  Serial.println(buf);
}

void logState(const char* fmt, ...) {
  char buf[128];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.print(ANSI_CYAN);
  Serial.print(F("[STATE] "));
  Serial.print(ANSI_RESET);
  Serial.println(buf);
}

}  // namespace CLI
