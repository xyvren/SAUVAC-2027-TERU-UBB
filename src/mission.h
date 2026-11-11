#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>

namespace Mission {

enum class Action {
  DIVE,        // "dive" / "selam"
  FORWARD,     // "maju" / "forward"
  BACKWARD,    // "mundur" / "backward"
  TURN,        // "putar" / "turn"
  SWAY_LEFT,   // "kiri" / "sway_left"
  SWAY_RIGHT,  // "kanan" / "sway_right"
  HOVER,       // "hover" / "diam"
  SURFACE,     // "surface" / "naik"
  UNKNOWN
};

struct Step {
  uint16_t stepNum;
  char name[36];
  Action action;
  float power;
  float durationSec;
  float targetCm;
};

constexpr size_t MAX_STEPS = 32;
constexpr const char* JALUR_FILE = "/jalur.json";

bool begin();
bool loadFromFile(const char* path = JALUR_FILE);
bool saveDefaultFile(const char* path = JALUR_FILE);
bool saveJsonString(const String& jsonStr, String& errMessage);

size_t getStepCount();
const Step* getStep(size_t index);
float getTotalDurationSec();
float getMaxPowerPercent();

const char* actionToString(Action act);
Action stringToAction(const String& str);
void printMissionTable();

}  // namespace Mission
