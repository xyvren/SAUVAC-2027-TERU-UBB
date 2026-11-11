#pragma once

#include <Arduino.h>
#include <EEPROM.h>
#include <stdint.h>
#include <stddef.h>

namespace Config {

constexpr uint32_t CONFIG_MAGIC = 0x41555631;  // 'AUV1'
constexpr uint8_t CONFIG_VERSION = 1;
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

// Default Direction Polarity: T5 & T6 vertical thrusters inverted for CW/CCW blade balance
constexpr uint8_t DEFAULT_THRUSTER_REVERSE[THRUSTER_COUNT] = {0, 0, 0, 0, 1, 1};
constexpr float DEFAULT_THRUSTER_TRIM[THRUSTER_COUNT] = {1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f};

// Safety Limits
constexpr uint8_t DEFAULT_POWER_LIMIT_PERCENT = 25;  // Default 25% safety cap for bench testing

// Sensor Addresses
constexpr uint8_t MPU6050_ADDR_PRIMARY = 0x68;
constexpr uint8_t MPU6050_ADDR_SECONDARY = 0x69;
constexpr uint8_t MS5803_ADDR_HIGH = 0x76;
constexpr uint8_t MS5803_ADDR_LOW = 0x77;

// Default PID Parameters
constexpr float DEFAULT_PID_DEPTH_KP = 0.040f;
constexpr float DEFAULT_PID_DEPTH_KI = 0.000f;
constexpr float DEFAULT_PID_DEPTH_KD = 0.015f;

constexpr float DEFAULT_PID_ROLL_KP = 0.020f;
constexpr float DEFAULT_PID_ROLL_KI = 0.000f;
constexpr float DEFAULT_PID_ROLL_KD = 0.008f;

constexpr float DEFAULT_PID_PITCH_KP = 0.020f;
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

  uint32_t checksum;
};

inline uint32_t calculateChecksum(const Settings& s) {
  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&s);
  uint32_t hash = 2166136261u;
  for (size_t i = 0; i < offsetof(Settings, checksum); ++i) {
    hash = (hash ^ bytes[i]) * 16777619u;
  }
  return hash;
}

inline void loadDefaults(Settings& s) {
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

  s.checksum = calculateChecksum(s);
}

inline bool loadSettings(Settings& s) {
  EEPROM.begin(sizeof(Settings));
  EEPROM.get(0, s);
  if (s.magic == CONFIG_MAGIC && s.version == CONFIG_VERSION &&
      s.checksum == calculateChecksum(s)) {
    // Validate bounds
    if (s.powerLimitPercent < 1 || s.powerLimitPercent > 100) {
      s.powerLimitPercent = DEFAULT_POWER_LIMIT_PERCENT;
    }
    if (s.waterDensity < 800.0f || s.waterDensity > 1200.0f) {
      s.waterDensity = 1000.0f;
    }
    for (size_t i = 0; i < THRUSTER_COUNT; ++i) {
      if (s.thrusters[i].pulseMin < 800 || s.thrusters[i].pulseMin > 1400) {
        s.thrusters[i].pulseMin = DEFAULT_PULSE_MIN_US;
      }
      if (s.thrusters[i].pulseNeutral < 1400 || s.thrusters[i].pulseNeutral > 1600) {
        s.thrusters[i].pulseNeutral = DEFAULT_PULSE_NEUTRAL_US;
      }
      if (s.thrusters[i].pulseMax < 1600 || s.thrusters[i].pulseMax > 2200) {
        s.thrusters[i].pulseMax = DEFAULT_PULSE_MAX_US;
      }
      if (s.thrusters[i].trim < 0.1f || s.thrusters[i].trim > 2.5f) {
        s.thrusters[i].trim = 1.00f;
      }
    }
    return true;
  }

  loadDefaults(s);
  return false;
}

inline bool saveSettings(Settings& s) {
  s.magic = CONFIG_MAGIC;
  s.version = CONFIG_VERSION;
  s.checksum = calculateChecksum(s);
  EEPROM.begin(sizeof(Settings));
  EEPROM.put(0, s);
  return EEPROM.commit();
}

}  // namespace Config
