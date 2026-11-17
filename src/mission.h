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
  MOTOR_1,     // "tes_t1" / "t1"
  MOTOR_2,     // "tes_t2" / "t2"
  MOTOR_3,     // "tes_t3" / "t3"
  MOTOR_4,     // "tes_t4" / "t4"
  MOTOR_5,     // "tes_t5" / "t5"
  MOTOR_6,     // "tes_t6" / "t6"
  ALL_MOTORS,  // "tes_all" / "all"
  UNKNOWN
};

struct Step {
  uint16_t stepNum;
  char name[36];
  Action action;
  float power;
  float durationSec;
  float targetCm;
  uint8_t motorMask;  // Bitmask: bit 0=T1, bit 1=T2, bit 2=T3, bit 3=T4, bit 4=T5, bit 5=T6
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
uint8_t getDefaultMotorMask(Action act, float targetCm = 0.0f);
void maskToString(uint8_t mask, char* out, size_t outSize);
void printMissionTable();

}  // namespace Mission
