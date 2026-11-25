#pragma once

#include <Arduino.h>
#include <stdint.h>
#include "config.h"
#include "sensors.h"
#include "mission.h"

namespace CLI {

struct MotorControlInterface {
  Config::Settings* settings;
  bool* armed;
  bool* pidEnabled;
  uint16_t (*getCurrentPulse)(size_t index);
  bool (*startMotorTest)(uint8_t id, float powerPercent, String& message);
  void (*emergencyStop)();
  bool (*setPidState)(bool enabled);
  bool (*saveSettings)();
  void (*resetDefaults)();

  // Mission & Auto-Run hooks
  bool (*startMission)(bool immediate);
  void (*cancelMission)();
  bool (*isMissionActive)();
  bool (*isCountdownActive)();
  int (*getCountdownRemaining)();
  bool (*calibrateDepthZero)();
  bool (*calibrateImuLevel)();
  void (*onInjected)();
  void (*setTargetDepth)(float targetCm);
  float (*getTargetDepth)();

  // Telemetry & Actuator hooks
  uint8_t (*getFsmState)();
  bool (*startEscCalib)();
  bool (*setRawPulseAll)(uint16_t pulseUs);
  bool (*setHilMode)(bool enabled);
  bool (*setHilPulses)(const uint16_t* pulses);
  bool (*isHilActive)();
};

void begin(const MotorControlInterface& iface);
void update();
void handleCommand(const String& rawCmd);
void printBanner();
void printPrompt();
void logInfo(const char* fmt, ...);
void logWarn(const char* fmt, ...);
void logError(const char* fmt, ...);
void logState(const char* fmt, ...);

// Telemetry stream control
bool isTelemActive();
uint8_t getTelemRateHz();
void setTelemActive(bool active, uint8_t rateHz = 10);
void sendTelemPacket();

}  // namespace CLI
