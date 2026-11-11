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
constexpr const char* ANSI_BOLD_CYAN   = "\033[1;36m";
constexpr const char* ANSI_BOLD_WHITE  = "\033[1;37m";

MotorControlInterface iface{};
bool initialized = false;

String inputBuffer = "";
bool sensorStreamActive = false;
uint32_t lastSensorStreamAt = 0;

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
  if (iface.armed && *iface.armed) {
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
  Serial.print(ANSI_BOLD_WHITE);
  Serial.println(F("[ SENSOR TELEMETRY ]"));
  Serial.print(ANSI_RESET);

  // MS5803
  Serial.print(F("  MS5803 Depth   : "));
  if (sensor.pressureReady) {
    Serial.print(ANSI_GREEN);
    Serial.print(F("ONLINE"));
    Serial.print(ANSI_RESET);
    Serial.printf(" | Pressure: %6.1f mbar | Temp: %4.1f °C\r\n",
                  sensor.pressureMbar, sensor.temperatureC);
    Serial.printf("                   Depth   : %6.1f cm ", sensor.depthCm);
    if (iface.settings->depthZeroCalibrated) {
      Serial.printf("(Zero Ref: %.1f mbar - CALIBRATED)\r\n",
                    iface.settings->depthZeroPressureMbar);
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
    Serial.printf(" | Pitch: %+5.1f° (raw %+5.1f°, off %+4.1f°)\r\n",
                  sensor.pitch, sensor.rawPitch, iface.settings->pitchOffset);
    Serial.printf("                   Roll : %+5.1f° (raw %+5.1f°, off %+4.1f°)\r\n",
                  sensor.roll, sensor.rawRoll, iface.settings->rollOffset);
    Serial.printf("                   Gyro : X=%+5.1f, Y=%+5.1f, Z=%+5.1f dps\r\n",
                  sensor.gyroDps[0], sensor.gyroDps[1], sensor.gyroDps[2]);
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
  Serial.printf("  Depth : Kp=%.3f, Ki=%.3f, Kd=%.3f\r\n",
                iface.settings->pidDepth.kp, iface.settings->pidDepth.ki, iface.settings->pidDepth.kd);
  Serial.printf("  Roll  : Kp=%.3f, Ki=%.3f, Kd=%.3f\r\n",
                iface.settings->pidRoll.kp, iface.settings->pidRoll.ki, iface.settings->pidRoll.kd);
  Serial.printf("  Pitch : Kp=%.3f, Ki=%.3f, Kd=%.3f\r\n",
                iface.settings->pidPitch.kp, iface.settings->pidPitch.ki, iface.settings->pidPitch.kd);
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
    Serial.printf("|  %u | GP%-2u |     %4u |         %4u |     %4u |   %s   |        %4.2fx       |\r\n",
                  static_cast<unsigned>(i + 1),
                  t.pin,
                  t.pulseMin,
                  t.pulseNeutral,
                  t.pulseMax,
                  t.reverse ? "YES" : " NO",
                  t.trim);
  }
  Serial.println(F("+----+------+----------+--------------+----------+---------+--------------------+"));
  Serial.printf("Global Safety Power Limit: %u%% | Cap Status: %s\r\n\r\n",
                iface.settings->powerLimitPercent,
                iface.settings->powerLimitPercent <= 25 ? "ACTIVE (Safe 25% Cap)" : "OVERRIDDEN (>25%)");
}

void printSensorSnapshot() {
  Sensors::SensorData s{};
  Sensors::snapshot(s);
  Serial.printf("[SENSOR SNAPSHOT] Depth: %5.1f cm | Pres: %6.1f mbar | Pitch: %+5.1f° | Roll: %+5.1f° | Gyro: [%+5.1f, %+5.1f, %+5.1f] dps | Temp: %4.1f°C\r\n",
                s.depthCm, s.pressureMbar, s.pitch, s.roll,
                s.gyroDps[0], s.gyroDps[1], s.gyroDps[2], s.temperatureC);
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
          Serial.printf("OK: Surface depth reference pressure set to %.1f mbar (manual). Depth zeroed!\r\n", manualPres);
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
        Serial.printf("OK: Surface depth reference pressure set to %.1f mbar. Depth zeroed!\r\n",
                      iface.settings->depthZeroPressureMbar);
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
        Serial.printf("OK: IMU level offsets set to Pitch: %+.2f°, Roll: %+.2f° (manual). Robot leveled!\r\n", pOff, rOff);
        Serial.print(ANSI_RESET);
        Serial.println(F("Type 'save' to commit to EEPROM."));
        return;
      }
      if (Sensors::calibrateImuLevel(*iface.settings)) {
        Serial.print(ANSI_GREEN);
        Serial.printf("OK: IMU level offsets recorded (Pitch: %+.2f°, Roll: %+.2f°). Robot leveled!\r\n",
                      iface.settings->pitchOffset, iface.settings->rollOffset);
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
      Serial.printf("OK: %s PID gains set to Kp=%.4f, Ki=%.4f, Kd=%.4f. Type 'save' to commit.\r\n",
                    sub.c_str(), kp, ki, kd);
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
  if (root == "stop" || root == "disarm") {
    if (iface.emergencyStop) iface.emergencyStop();
    Serial.print(ANSI_BOLD_RED);
    Serial.println(F("[EMERGENCY STOP] All 6 thrusters neutralized immediately (1500us). Disarmed."));
    Serial.print(ANSI_RESET);
    return;
  }

  Serial.print(ANSI_RED);
  Serial.printf("ERROR: Unknown command '%s'. Type 'help' or '?' for available commands.\r\n", tokens[0].c_str());
  Serial.print(ANSI_RESET);
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
  // Check for incoming serial characters
  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());

    // If streaming sensor data, any key stops the stream
    if (sensorStreamActive) {
      sensorStreamActive = false;
      Serial.println(F("\r\n[STREAM] Sensor stream stopped by user keypress."));
      printPrompt();
      return;
    }

    if (c == '\r' || c == '\n') {
      if (inputBuffer.length() > 0) {
        Serial.println();
        handleCommand(inputBuffer);
        inputBuffer = "";
        printPrompt();
      }
    } else if (c == '\b' || c == 0x7F) {
      if (inputBuffer.length() > 0) {
        inputBuffer.remove(inputBuffer.length() - 1);
        Serial.print(F("\b \b"));  // Visual backspace
      }
    } else if (c >= 32 && c <= 126) {
      inputBuffer += c;
      Serial.print(c);  // Echo character
    }
  }

  // Handle sensor streaming if active (5 Hz / 200 ms)
  if (sensorStreamActive) {
    const uint32_t now = millis();
    if (now - lastSensorStreamAt >= 200) {
      lastSensorStreamAt = now;
      Sensors::SensorData s{};
      Sensors::snapshot(s);
      Serial.printf("[LIVE] P: %+5.1f° | R: %+5.1f° | Depth: %5.1f cm | Pres: %6.1f mbar | Gyro: [%+5.1f, %+5.1f, %+5.1f] dps\r\n",
                    s.pitch, s.roll, s.depthCm, s.pressureMbar,
                    s.gyroDps[0], s.gyroDps[1], s.gyroDps[2]);
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
