#pragma once

#include <Arduino.h>
#include <stdint.h>
#include "config.h"

namespace Sensors {

struct SensorData {
  bool i2cReady;
  bool imuReady;
  bool pressureReady;

  float accelG[3];
  float gyroDps[3];
  float rawPitch;
  float rawRoll;
  float pitch;        // Level-compensated pitch (deg)
  float roll;         // Level-compensated roll (deg)

  float pressureMbar;
  float temperatureC;
  float depthCm;

  uint32_t lastUpdateMs;
  uint32_t sampleCount;
};

void begin();
void update(const Config::Settings& settings);
void snapshot(SensorData& dest);
bool calibrateDepthZero(Config::Settings& settings);
bool calibrateImuLevel(Config::Settings& settings);

}  // namespace Sensors
