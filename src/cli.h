#pragma once

#include <Arduino.h>
#include <stdint.h>
#include "config.h"
#include "sensors.h"

namespace CLI {

struct MotorControlInterface {
  Config::Settings* settings;
  bool* armed;
  bool* pidEnabled;
  uint16_t (*getCurrentPulse)(size_t index);
  bool (*startMotorTest)(uint8_t id, float powerPercent, String& message);
  void (*emergencyStop)();
  void (*setPidState)(bool enabled);
  bool (*saveSettings)();
  void (*resetDefaults)();
};

void begin(const MotorControlInterface& iface);
void update();
void printBanner();
void printPrompt();
void logInfo(const char* fmt, ...);
void logWarn(const char* fmt, ...);
void logError(const char* fmt, ...);
void logState(const char* fmt, ...);

}  // namespace CLI
