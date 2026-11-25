#pragma once

#include "config.h"

namespace WirelessConfig {

void begin(Config::Settings& settings, bool& armed, uint8_t (*getFsmState)(),
           void (*refreshOutputs)(),
           uint16_t (*getCurrentPulse)(size_t index) = nullptr,
           bool (*canStartFirmwareUpdate)() = nullptr,
           void (*prepareFirmwareUpdate)() = nullptr);
void syncFromSettings(bool persisted);
void update();
bool firmwareUpdateInProgress();

}  // namespace WirelessConfig
