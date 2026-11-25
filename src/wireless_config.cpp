#include "wireless_config.h"

#include <ArduinoJson.h>
#include <LittleFS.h>
#include <Updater.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <limits.h>
#include <math.h>
#include <string.h>

#include "sensors.h"
#include "cli.h"
#include "mission.h"

#if defined(__has_include)
#if __has_include("wireless_secrets.h")
#include "wireless_secrets.h"
#endif
#endif

#ifndef AUV_WIFI_AP_PASSWORD
#define AUV_WIFI_AP_PASSWORD ""
#endif

namespace WirelessConfig {
namespace {

constexpr uint16_t UDP_PORT = 4210;
constexpr size_t MAX_REQUEST_BYTES = 384;
constexpr uint32_t MIN_REQUEST_INTERVAL_MS = 40;
constexpr char PLACEHOLDER_PASSWORD[] = "CHANGE_THIS_TO_A_RANDOM_16_CHAR_KEY";
constexpr char FIRMWARE_UPDATE_USERNAME[] = "sauv-update";

Config::Settings* liveSettings = nullptr;
Config::Settings pendingSettings{};
Config::Settings savedSettings{};
Config::Settings observedSettings{};
bool* armedState = nullptr;
uint8_t (*fsmStateGetter)() = nullptr;
void (*refreshOutputs)() = nullptr;
bool enabled = false;
bool pendingDirty = false;
uint32_t lastRequestAtMs = 0;
uint16_t (*getCurrentPulseCallback)(size_t index) = nullptr;
bool (*canStartFirmwareUpdateCallback)() = nullptr;
void (*prepareFirmwareUpdateCallback)() = nullptr;
WiFiUDP udp;
WebServer firmwareUpdateServer(80);
char packetBuffer[MAX_REQUEST_BYTES + 1];
char responseBuffer[1500];
StaticJsonDocument<1536> responseDocument;
bool updateInProgress = false;
bool updateAuthorized = false;
bool updateAccepted = false;
bool updateComplete = false;
bool updateActivityThisCall = false;
uint16_t updateResponseCode = 400;
String updateError;
bool missionUploadAuthorized = false;
bool missionUploadAccepted = false;
bool missionUploadInProgress = false;
bool missionUploadComplete = false;
uint16_t missionUploadResponseCode = 400;
String missionUploadError;
String missionUploadJson;

void setUpdateError(uint16_t code, const char* message) {
  updateResponseCode = code;
  updateError = message;
  updateAccepted = false;
  updateComplete = false;
  updateInProgress = false;
}

void clearUpdateRequest() {
  updateInProgress = false;
  updateAuthorized = false;
  updateAccepted = false;
  updateComplete = false;
  updateResponseCode = 400;
  updateError = "";
}

void handleFirmwareUpload() {
  if (!updateAuthorized) {
    firmwareUpdateServer.requestAuthentication();
    clearUpdateRequest();
    return;
  }
  if (updateError.length()) {
    firmwareUpdateServer.send(updateResponseCode, "text/plain; charset=utf-8", updateError);
    clearUpdateRequest();
    return;
  }
  if (!updateComplete) {
    firmwareUpdateServer.send(400, "text/plain; charset=utf-8", "Firmware upload did not complete.");
    clearUpdateRequest();
    return;
  }

  firmwareUpdateServer.send(202, "text/plain; charset=utf-8",
                            "Firmware accepted. Pico W is restarting.");
  delay(150);
  firmwareUpdateServer.client().stop();
  rp2040.restart();
}

void handleFirmwareUploadData() {
  HTTPUpload& upload = firmwareUpdateServer.upload();
  if (upload.status == UPLOAD_FILE_START) {
    clearUpdateRequest();
    updateAuthorized = firmwareUpdateServer.authenticate(
        FIRMWARE_UPDATE_USERNAME, AUV_WIFI_AP_PASSWORD);
    if (!updateAuthorized) {
      return;
    }

    String filename = upload.filename;
    filename.toLowerCase();
    if (upload.name != "firmware" || !filename.endsWith(".bin")) {
      setUpdateError(415, "Upload a compiled Pico W .bin image; UF2 files are not accepted.");
      return;
    }
    if (!canStartFirmwareUpdateCallback || !canStartFirmwareUpdateCallback()) {
      setUpdateError(409, "Firmware update requires idle, disarmed, neutral thrusters, Auto-Run off, and no active simulation injection.");
      return;
    }

    if (prepareFirmwareUpdateCallback) {
      prepareFirmwareUpdateCallback();
    }

    FSInfo filesystemInfo{};
    if (!LittleFS.info(filesystemInfo) ||
        filesystemInfo.totalBytes <= filesystemInfo.usedBytes) {
      setUpdateError(507, "LittleFS could not provide firmware staging space.");
      return;
    }
    const size_t availableBytes = filesystemInfo.totalBytes - filesystemInfo.usedBytes;
    if (!Update.begin(availableBytes, U_FLASH)) {
      Update.printError(Serial);
      setUpdateError(507, "Not enough LittleFS space to stage this firmware image.");
      return;
    }

    updateAccepted = true;
    updateInProgress = true;
    updateActivityThisCall = true;
    Serial.println(F("[OTA] Authenticated firmware upload started; outputs remain disarmed."));
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (updateAccepted && !updateError.length() &&
        Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
      Update.end();
      setUpdateError(500, "Firmware image could not be staged.");
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (updateAccepted && !updateError.length()) {
      if (Update.end(true)) {
        updateComplete = true;
        Serial.printf("[OTA] Firmware image staged: %u bytes; rebooting.\r\n",
                      static_cast<unsigned>(upload.totalSize));
      } else {
        Update.printError(Serial);
        Update.end();
        setUpdateError(400, "Pico OTA updater rejected the firmware image.");
      }
    }
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    if (updateAccepted) {
      Update.end();
    }
    setUpdateError(400, "Firmware upload was interrupted; the current firmware remains active.");
  }
}

void clearMissionUploadRequest() {
  missionUploadAuthorized = false;
  missionUploadAccepted = false;
  missionUploadInProgress = false;
  missionUploadComplete = false;
  missionUploadResponseCode = 400;
  missionUploadError = "";
  missionUploadJson = "";
}

void setMissionUploadError(uint16_t code, const char* message) {
  missionUploadResponseCode = code;
  missionUploadError = message;
  missionUploadAccepted = false;
  missionUploadComplete = false;
  missionUploadInProgress = false;
}

bool remoteMutationSafe() {
  return canStartFirmwareUpdateCallback && canStartFirmwareUpdateCallback();
}

void handleMissionUpload() {
  if (!missionUploadAuthorized) {
    firmwareUpdateServer.requestAuthentication();
    clearMissionUploadRequest();
    return;
  }
  if (missionUploadError.length()) {
    firmwareUpdateServer.send(missionUploadResponseCode, "text/plain; charset=utf-8",
                              missionUploadError);
    clearMissionUploadRequest();
    return;
  }
  if (!missionUploadComplete) {
    firmwareUpdateServer.send(400, "text/plain; charset=utf-8",
                              "Mission upload did not complete.");
    clearMissionUploadRequest();
    return;
  }

  const String message = String("Jalur berhasil divalidasi dan disimpan: ") +
                        Mission::getStepCount() + " langkah.";
  firmwareUpdateServer.send(200, "text/plain; charset=utf-8", message);
  clearMissionUploadRequest();
}

void handleMissionUploadData() {
  HTTPUpload& upload = firmwareUpdateServer.upload();
  if (upload.status == UPLOAD_FILE_START) {
    clearMissionUploadRequest();
    missionUploadAuthorized = firmwareUpdateServer.authenticate(
        FIRMWARE_UPDATE_USERNAME, AUV_WIFI_AP_PASSWORD);
    if (!missionUploadAuthorized) {
      return;
    }

    String filename = upload.filename;
    filename.toLowerCase();
    if (upload.name != "mission" || !filename.endsWith(".json")) {
      setMissionUploadError(415, "Upload jalur JSON dengan field bernama mission.");
      return;
    }
    if (!remoteMutationSafe()) {
      setMissionUploadError(409, "Unggah jalur memerlukan kondisi idle, disarmed, thruster netral, Auto-Run mati, dan tanpa simulasi aktif.");
      return;
    }

    missionUploadJson.reserve(3500);
    missionUploadAccepted = true;
    missionUploadInProgress = true;
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (!missionUploadAccepted || missionUploadError.length()) {
      return;
    }
    if (!remoteMutationSafe()) {
      setMissionUploadError(409, "Kondisi aman berubah saat unggah jalur berlangsung.");
      return;
    }
    if (missionUploadJson.length() + upload.currentSize > 3500) {
      setMissionUploadError(413, "Ukuran jalur melebihi 3.500 byte.");
      return;
    }
    if (!missionUploadJson.concat(reinterpret_cast<const char*>(upload.buf),
                                  upload.currentSize)) {
      setMissionUploadError(507, "Memori tidak cukup untuk menerima jalur.");
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    missionUploadInProgress = false;
    if (!missionUploadAccepted || missionUploadError.length()) {
      return;
    }
    if (!remoteMutationSafe()) {
      setMissionUploadError(409, "Kondisi aman berubah sebelum jalur disimpan.");
      return;
    }

    String errMessage;
    if (!Mission::saveJsonString(missionUploadJson, errMessage)) {
      setMissionUploadError(422, errMessage.c_str());
      return;
    }
    missionUploadComplete = true;
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    setMissionUploadError(400, "Unggah jalur terputus; jalur sebelumnya tetap digunakan.");
  }
}

bool readNumber(JsonVariantConst value, float& result) {
  if (value.isNull() || value.is<bool>() ||
      !(value.is<float>() || value.is<long>() || value.is<unsigned long>())) {
    return false;
  }
  result = value.as<float>();
  return isfinite(result);
}

bool readInteger(JsonVariantConst value, long& result) {
  if (value.isNull() || value.is<bool>()) {
    return false;
  }
  if (value.is<long>()) {
    result = value.as<long>();
    return true;
  }
  if (value.is<unsigned long>()) {
    const unsigned long unsignedValue = value.as<unsigned long>();
    if (unsignedValue > static_cast<unsigned long>(LONG_MAX)) {
      return false;
    }
    result = static_cast<long>(unsignedValue);
    return true;
  }

  float number = 0.0f;
  if (!readNumber(value, number) || number < static_cast<float>(LONG_MIN) ||
      number >= static_cast<float>(LONG_MAX) || truncf(number) != number) {
    return false;
  }
  result = static_cast<long>(number);
  return true;
}

template <typename TDocument>
bool requireIdle(TDocument& reply) {
  if (!armedState || *armedState || !fsmStateGetter || fsmStateGetter() != 0) {
    reply["ok"] = false;
    reply["error"] = "parameter changes require idle, disarmed state";
    return false;
  }
  return true;
}

void addSettings(JsonObject destination, const Config::Settings& settings) {
  destination["power_limit"] = settings.powerLimitPercent;
  destination["esc_deadband_us"] = settings.escDeadbandUs;
  destination["balance_horiz"] = settings.diffHorizTrim;
  destination["balance_vert"] = settings.diffVertTrim;

  JsonObject pid = destination["pid"].to<JsonObject>();
  JsonObject depth = pid["depth"].to<JsonObject>();
  depth["kp"] = settings.pidDepth.kp;
  depth["ki"] = settings.pidDepth.ki;
  depth["kd"] = settings.pidDepth.kd;
  JsonObject roll = pid["roll"].to<JsonObject>();
  roll["kp"] = settings.pidRoll.kp;
  roll["ki"] = settings.pidRoll.ki;
  roll["kd"] = settings.pidRoll.kd;

  JsonObject autorun = destination["autorun"].to<JsonObject>();
  autorun["enabled"] = settings.autoRunEnabled != 0;
  autorun["delay_sec"] = settings.autoRunDelaySec;
  autorun["tare_sec"] = settings.autoRunTareSec;
  autorun["level_sec"] = settings.autoRunLevelSec;

  JsonObject startupCalibration = destination["startup_calibration"].to<JsonObject>();
  startupCalibration["depth_tare"] = settings.autoDepthTareOnBoot != 0;
  startupCalibration["imu_level"] = settings.autoImuLevelOnBoot != 0;

  JsonArray thrusters = destination["thrusters"].to<JsonArray>();
  for (size_t i = 0; i < Config::THRUSTER_COUNT; ++i) {
    const Config::ThrusterConfig& config = settings.thrusters[i];
    JsonObject thruster = thrusters.createNestedObject();
    thruster["id"] = i + 1;
    thruster["pin"] = config.pin;
    thruster["reverse"] = config.reverse != 0;
    thruster["trim"] = config.trim;
    thruster["pulse_min"] = config.pulseMin;
    thruster["pulse_neutral"] = config.pulseNeutral;
    thruster["pulse_max"] = config.pulseMax;
  }
}

void sendReply(IPAddress remoteIp, uint16_t remotePort) {
  const size_t size = measureJson(responseDocument);
  const bool hasRequestId = responseDocument["req_id"].is<long>();
  const long requestId = hasRequestId ? responseDocument["req_id"].as<long>() : 0;
  if (responseDocument.overflowed() || size >= sizeof(responseBuffer)) {
    responseDocument.clear();
    if (hasRequestId) {
      responseDocument["req_id"] = requestId;
    }
    responseDocument["ok"] = false;
    responseDocument["error"] = "response exceeds protocol limit";
  }

  const size_t written = serializeJson(responseDocument, responseBuffer, sizeof(responseBuffer));
  if (!written || !udp.beginPacket(remoteIp, remotePort)) {
    responseDocument.clear();
    return;
  }
  udp.write(reinterpret_cast<const uint8_t*>(responseBuffer), written);
  udp.endPacket();
  responseDocument.clear();
}

void addConfigResponse() {
  responseDocument["op"] = "config";
  responseDocument["protocol_version"] = 1;
  responseDocument["dirty"] = pendingDirty;
  JsonObject settings = responseDocument["settings"].to<JsonObject>();
  addSettings(settings, pendingSettings);

  Sensors::SensorData sensor{};
  Sensors::snapshot(sensor);
  JsonObject status = responseDocument["status"].to<JsonObject>();
  status["armed"] = *armedState;
  status["fsm_state"] = fsmStateGetter ? fsmStateGetter() : 0;
  status["imu_ready"] = sensor.imuReady;
  status["pressure_ready"] = sensor.pressureReady;
  status["depth_cm"] = sensor.depthCm;
  status["pressure_mbar"] = sensor.pressureMbar;
  status["pitch"] = sensor.pitch;
  status["roll"] = sensor.roll;
  status["temperature_c"] = sensor.temperatureC;
  JsonArray pulses = status["pulses"].to<JsonArray>();
  for (size_t i = 0; i < Config::THRUSTER_COUNT; ++i) {
    pulses.add(getCurrentPulseCallback ? getCurrentPulseCallback(i) : 1500);
  }
  status["depth_calibrated"] = liveSettings->depthZeroCalibrated != 0;
  status["sensor_age_ms"] = sensor.sampleCount ? millis() - sensor.lastUpdateMs : 0xFFFFFFFFUL;
}

template <typename TRequest, typename TReply>
bool stagePid(TRequest& request, TReply& reply) {
  const char* axis = request["axis"] | "";
  const bool isDepth = strcmp(axis, "depth") == 0;
  if (!isDepth && strcmp(axis, "roll") != 0) {
    reply["ok"] = false;
    reply["error"] = "axis must be depth or roll; pitch is unsupported";
    return false;
  }

  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
  if (!readNumber(request["kp"], kp) || !readNumber(request["ki"], ki) ||
      !readNumber(request["kd"], kd) || kp < 0.0f || kp > 100.0f ||
      ki < 0.0f || ki > 100.0f || kd < 0.0f || kd > 100.0f) {
    reply["ok"] = false;
    reply["error"] = "PID gains must be finite values from 0 to 100";
    return false;
  }

  Config::Settings candidate = pendingSettings;
  Config::PidGains* candidateGains = isDepth ? &candidate.pidDepth : &candidate.pidRoll;
  candidateGains->kp = kp;
  candidateGains->ki = ki;
  candidateGains->kd = kd;
  if (!Config::validSettings(candidate)) {
    reply["ok"] = false;
    reply["error"] = "candidate settings failed validation";
    return false;
  }
  pendingSettings = candidate;
  pendingDirty = true;
  return true;
}

template <typename TRequest, typename TReply>
bool stageThruster(TRequest& request, TReply& reply) {
  long id = 0;
  if (!readInteger(request["id"], id) || id < 1 ||
      id > static_cast<long>(Config::THRUSTER_COUNT)) {
    reply["ok"] = false;
    reply["error"] = "thruster id must be an integer from 1 to 6";
    return false;
  }

  Config::Settings candidate = pendingSettings;
  Config::ThrusterConfig& thruster = candidate.thrusters[static_cast<size_t>(id - 1)];
  float number = 0.0f;
  long integer = 0;

  if (request.containsKey("reverse")) {
    if (!request["reverse"].template is<bool>()) {
      reply["ok"] = false;
      reply["error"] = "reverse must be boolean";
      return false;
    }
    thruster.reverse = request["reverse"].template as<bool>() ? 1 : 0;
  }
  if (request.containsKey("trim")) {
    if (!readNumber(request["trim"], number)) {
      reply["ok"] = false;
      reply["error"] = "trim must be a finite number";
      return false;
    }
    thruster.trim = number;
  }
  if (request.containsKey("pulse_min")) {
    if (!readInteger(request["pulse_min"], integer) || integer < 0 || integer > UINT16_MAX) {
      reply["ok"] = false;
      reply["error"] = "pulse_min must be an integer";
      return false;
    }
    thruster.pulseMin = static_cast<uint16_t>(integer);
  }
  if (request.containsKey("pulse_neutral")) {
    if (!readInteger(request["pulse_neutral"], integer) || integer < 0 || integer > UINT16_MAX) {
      reply["ok"] = false;
      reply["error"] = "pulse_neutral must be an integer";
      return false;
    }
    thruster.pulseNeutral = static_cast<uint16_t>(integer);
  }
  if (request.containsKey("pulse_max")) {
    if (!readInteger(request["pulse_max"], integer) || integer < 0 || integer > UINT16_MAX) {
      reply["ok"] = false;
      reply["error"] = "pulse_max must be an integer";
      return false;
    }
    thruster.pulseMax = static_cast<uint16_t>(integer);
  }

  if (!Config::validSettings(candidate)) {
    reply["ok"] = false;
    reply["error"] = "thruster settings failed range or pulse-order validation";
    return false;
  }
  pendingSettings = candidate;
  pendingDirty = true;
  return true;
}

template <typename TRequest, typename TReply>
bool stageRequest(TRequest& request, TReply& reply, const char* operation) {
  Config::Settings candidate = pendingSettings;
  float number = 0.0f;
  long integer = 0;

  if (strcmp(operation, "set_power_limit") == 0) {
    if (!readInteger(request["percent"], integer) || integer < 1 || integer > 100) {
      reply["ok"] = false;
      reply["error"] = "percent must be an integer from 1 to 100";
      return false;
    }
    candidate.powerLimitPercent = static_cast<uint8_t>(integer);
  } else if (strcmp(operation, "set_deadband") == 0) {
    if (!readInteger(request["microseconds"], integer) || integer < 0 || integer > 80) {
      reply["ok"] = false;
      reply["error"] = "microseconds must be an integer from 0 to 80";
      return false;
    }
    candidate.escDeadbandUs = static_cast<uint16_t>(integer);
  } else if (strcmp(operation, "set_balance") == 0) {
    const char* axis = request["axis"] | "";
    if (!readNumber(request["percent"], number) || number < -20.0f || number > 20.0f) {
      reply["ok"] = false;
      reply["error"] = "percent must be from -20 to 20";
      return false;
    }
    if (strcmp(axis, "horiz") == 0) {
      candidate.diffHorizTrim = number;
    } else if (strcmp(axis, "vert") == 0) {
      candidate.diffVertTrim = number;
    } else {
      reply["ok"] = false;
      reply["error"] = "axis must be horiz or vert";
      return false;
    }
  } else if (strcmp(operation, "set_boot_calibration") == 0) {
    bool hasCalibrationOption = false;
    if (request.containsKey("depth_tare")) {
      if (!request["depth_tare"].template is<bool>()) {
        reply["ok"] = false;
        reply["error"] = "depth_tare must be a boolean";
        return false;
      }
      candidate.autoDepthTareOnBoot = request["depth_tare"].template as<bool>() ? 1 : 0;
      hasCalibrationOption = true;
    }
    if (request.containsKey("imu_level")) {
      if (!request["imu_level"].template is<bool>()) {
        reply["ok"] = false;
        reply["error"] = "imu_level must be a boolean";
        return false;
      }
      candidate.autoImuLevelOnBoot = request["imu_level"].template as<bool>() ? 1 : 0;
      hasCalibrationOption = true;
    }
    if (!hasCalibrationOption) {
      reply["ok"] = false;
      reply["error"] = "at least one boot calibration option is required";
      return false;
    }
  } else if (strcmp(operation, "set_autorun_timing") == 0) {
    if (pendingSettings.autoRunEnabled) {
      reply["ok"] = false;
      reply["error"] = "disable autorun through USB before changing its timing wirelessly";
      return false;
    }
    if (request.containsKey("delay_sec")) {
      if (!readInteger(request["delay_sec"], integer) || integer < 0 || integer > UINT8_MAX) {
        reply["ok"] = false;
        reply["error"] = "delay_sec must be an integer";
        return false;
      }
      candidate.autoRunDelaySec = static_cast<uint8_t>(integer);
    }
    if (request.containsKey("tare_sec")) {
      if (!readInteger(request["tare_sec"], integer) || integer < 0 || integer > UINT8_MAX) {
        reply["ok"] = false;
        reply["error"] = "tare_sec must be an integer";
        return false;
      }
      candidate.autoRunTareSec = static_cast<uint8_t>(integer);
    }
    if (request.containsKey("level_sec")) {
      if (!readInteger(request["level_sec"], integer) || integer < 0 || integer > UINT8_MAX) {
        reply["ok"] = false;
        reply["error"] = "level_sec must be an integer";
        return false;
      }
      candidate.autoRunLevelSec = static_cast<uint8_t>(integer);
    }
  } else {
    reply["ok"] = false;
    reply["error"] = "unknown operation";
    return false;
  }

  if (!Config::validSettings(candidate)) {
    reply["ok"] = false;
    reply["error"] = "candidate settings failed validation";
    return false;
  }
  pendingSettings = candidate;
  pendingDirty = true;
  return true;
}

bool calibrateSensor(bool depth) {
  if (!requireIdle(responseDocument)) {
    return false;
  }
  if (!remoteMutationSafe()) {
    responseDocument["ok"] = false;
    responseDocument["error"] = "Kalibrasi memerlukan kondisi idle, disarmed, thruster netral, Auto-Run mati, dan tanpa simulasi aktif.";
    return false;
  }
  if (pendingDirty) {
    responseDocument["ok"] = false;
    responseDocument["error"] = "Simpan atau buang perubahan parameter sebelum kalibrasi.";
    return false;
  }

  Sensors::SensorData sensor{};
  Sensors::snapshot(sensor);
  if (!sensor.sampleCount || millis() - sensor.lastUpdateMs > 500) {
    responseDocument["ok"] = false;
    responseDocument["error"] = "Data sensor harus tersedia dan lebih baru dari 500 ms.";
    return false;
  }
  if ((depth && !sensor.pressureReady) || (!depth && !sensor.imuReady)) {
    responseDocument["ok"] = false;
    responseDocument["error"] = depth
        ? "Sensor tekanan MS5803 belum siap."
        : "Sensor IMU MPU-6050 belum siap.";
    return false;
  }

  Config::Settings calibrated = *liveSettings;
  const bool success = depth
      ? Sensors::calibrateDepthZero(calibrated)
      : Sensors::calibrateImuLevel(calibrated);
  if (!success) {
    responseDocument["ok"] = false;
    responseDocument["error"] = "Pembacaan sensor tidak valid; kalibrasi tidak disimpan.";
    return false;
  }
  if (!Config::saveSettings(calibrated)) {
    responseDocument["ok"] = false;
    responseDocument["error"] = "EEPROM gagal menyimpan kalibrasi.";
    return false;
  }

  *liveSettings = calibrated;
  pendingSettings = calibrated;
  savedSettings = calibrated;
  observedSettings = calibrated;
  pendingDirty = false;
  responseDocument["calibrated"] = true;
  responseDocument["saved"] = true;
  responseDocument["calibration"] = depth ? "depth_zero" : "imu_level";
  responseDocument["dirty"] = false;
  return true;
}

void handlePacket(const char* payload, IPAddress remoteIp, uint16_t remotePort) {
  StaticJsonDocument<512> request;
  const DeserializationError parseError = deserializeJson(request, payload);
  if (parseError || !request.is<JsonObject>()) {
    responseDocument["ok"] = false;
    responseDocument["error"] = "request must be a JSON object";
    sendReply(remoteIp, remotePort);
    return;
  }

  long requestId = 0;
  if (!readInteger(request["req_id"], requestId) || requestId < 0 ||
      requestId > 2147483647L) {
    responseDocument["ok"] = false;
    responseDocument["error"] = "req_id must be a non-negative 32-bit integer";
    sendReply(remoteIp, remotePort);
    return;
  }

  const char* operation = request["op"] | "";
  const bool settingsChangedElsewhere =
      memcmp(liveSettings, &observedSettings, sizeof(Config::Settings)) != 0;
  if (settingsChangedElsewhere) {
    pendingSettings = *liveSettings;
    observedSettings = *liveSettings;
    pendingDirty = false;
  }
  responseDocument["req_id"] = requestId;
  responseDocument["ok"] = true;

  if (settingsChangedElsewhere && strcmp(operation, "get") != 0) {
    responseDocument["op"] = "error";
    responseDocument["ok"] = false;
    responseDocument["error"] = "settings changed outside wireless; fetch configuration before retrying";
    sendReply(remoteIp, remotePort);
    return;
  }

  if (strcmp(operation, "get") == 0) {
    addConfigResponse();
  } else if (strcmp(operation, "set_pid") == 0 ||
             strcmp(operation, "set_thruster") == 0 ||
             strcmp(operation, "set_power_limit") == 0 ||
             strcmp(operation, "set_deadband") == 0 ||
             strcmp(operation, "set_balance") == 0 ||
             strcmp(operation, "set_boot_calibration") == 0 ||
             strcmp(operation, "set_autorun_timing") == 0) {
    if (requireIdle(responseDocument)) {
      bool staged = false;
      if (strcmp(operation, "set_pid") == 0) {
        staged = stagePid(request, responseDocument);
      } else if (strcmp(operation, "set_thruster") == 0) {
        staged = stageThruster(request, responseDocument);
      } else {
        staged = stageRequest(request, responseDocument, operation);
      }
      if (staged) {
        responseDocument["ok"] = true;
        responseDocument["dirty"] = pendingDirty;
      }
    }
  } else if (strcmp(operation, "apply") == 0) {
    if (requireIdle(responseDocument)) {
      if (!Config::validSettings(pendingSettings)) {
        responseDocument["ok"] = false;
        responseDocument["error"] = "pending settings failed validation";
      } else {
        *liveSettings = pendingSettings;
        observedSettings = *liveSettings;
        if (refreshOutputs) {
          refreshOutputs();
        }
        responseDocument["applied"] = true;
        responseDocument["dirty"] = pendingDirty;
      }
    }
  } else if (strcmp(operation, "save") == 0) {
    if (requireIdle(responseDocument)) {
      Config::Settings candidate = pendingSettings;
      if (!Config::validSettings(candidate)) {
        responseDocument["ok"] = false;
        responseDocument["error"] = "pending settings failed validation";
      } else if (!Config::saveSettings(candidate)) {
        responseDocument["ok"] = false;
        responseDocument["error"] = "EEPROM save failed; current settings unchanged";
      } else {
        *liveSettings = candidate;
        observedSettings = *liveSettings;
        if (refreshOutputs) {
          refreshOutputs();
        }
        pendingSettings = candidate;
        pendingDirty = false;
        savedSettings = candidate;
        responseDocument["saved"] = true;
        responseDocument["dirty"] = false;
        responseDocument["checksum"] = candidate.checksum;
      }
    }
  } else if (strcmp(operation, "command") == 0) {
    const char* command = request["command"] | "";
    if (strlen(command) < 1 || strlen(command) > 256) {
      responseDocument["ok"] = false;
      responseDocument["error"] = "command must contain 1-256 characters";
    } else if (strchr(command, '\n') || strchr(command, '\r') ||
               strstr(command, "&&") || strstr(command, "|") || strstr(command, "`") ||
               strstr(command, ";")) {
      responseDocument["ok"] = false;
      responseDocument["error"] = "command must be a single safe line";
    } else {
      CLI::handleCommand(String(command));
      responseDocument["executed"] = true;
      responseDocument["reply"] = "command acknowledged by firmware";
    }
  } else if (strcmp(operation, "calibrate_depth") == 0) {
    calibrateSensor(true);
  } else if (strcmp(operation, "calibrate_level") == 0) {
    calibrateSensor(false);
  } else if (strcmp(operation, "mission_upload") == 0) {
    const char* json = request["json"] | "";
    if (!requireIdle(responseDocument)) {
      // The idle check already populated the error response.
    } else if (!remoteMutationSafe()) {
      responseDocument["ok"] = false;
      responseDocument["error"] = "Unggah jalur memerlukan kondisi aman dan Auto-Run nonaktif.";
    } else if (strlen(json) < 2 || strlen(json) > 3500) {
      responseDocument["ok"] = false;
      responseDocument["error"] = "mission JSON must contain 2-3500 characters";
    } else {
      String errMessage;
      if (!Mission::saveJsonString(String(json), errMessage)) {
        responseDocument["ok"] = false;
        responseDocument["error"] = errMessage.c_str();
      } else if (!Mission::loadFromFile()) {
        responseDocument["ok"] = false;
        responseDocument["error"] = "mission file saved but could not be loaded";
      } else {
        responseDocument["uploaded"] = true;
        responseDocument["step_count"] = Mission::getStepCount();
      }
    }
  } else if (strcmp(operation, "discard") == 0) {
    if (requireIdle(responseDocument)) {
      *liveSettings = savedSettings;
      pendingSettings = savedSettings;
      observedSettings = savedSettings;
      if (refreshOutputs) {
        refreshOutputs();
      }
      pendingDirty = false;
      responseDocument["discarded"] = true;
      responseDocument["dirty"] = false;
    }
  } else {
    responseDocument["ok"] = false;
    responseDocument["error"] = "unsupported operation";
  }

  if (!responseDocument["op"].is<const char*>()) {
    responseDocument["op"] = "reply";
  }
  sendReply(remoteIp, remotePort);
}

}  // namespace

void begin(Config::Settings& settings, bool& armed, uint8_t (*getFsmState)(),
           void (*refreshOutputState)(),
           uint16_t (*getCurrentPulse)(size_t index),
           bool (*canStartFirmwareUpdate)(),
           void (*prepareFirmwareUpdate)()) {
  liveSettings = &settings;
  pendingSettings = settings;
  savedSettings = settings;
  observedSettings = settings;
  armedState = &armed;
  fsmStateGetter = getFsmState;
  refreshOutputs = refreshOutputState;
  getCurrentPulseCallback = getCurrentPulse;
  canStartFirmwareUpdateCallback = canStartFirmwareUpdate;
  prepareFirmwareUpdateCallback = prepareFirmwareUpdate;

  const size_t passwordLength = strlen(AUV_WIFI_AP_PASSWORD);
  if (passwordLength < 12 || passwordLength > 63 ||
      strcmp(AUV_WIFI_AP_PASSWORD, PLACEHOLDER_PASSWORD) == 0) {
    Serial.println(F("[WIFI] Configuration access disabled: set a private AP password (12-63 characters) in src/wireless_secrets.h."));
    return;
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1),
                    IPAddress(192, 168, 4, 1),
                    IPAddress(255, 255, 255, 0));
  uint8_t mac[6] = {};
  WiFi.softAPmacAddress(mac);
  char ssid[24];
  snprintf(ssid, sizeof(ssid), "SAUVAC-%02X%02X", mac[4], mac[5]);
  if (!WiFi.softAP(ssid, AUV_WIFI_AP_PASSWORD, 6, 1, 1)) {
    Serial.println(F("[WIFI] Access point startup failed; USB serial remains available."));
    WiFi.mode(WIFI_OFF);
    return;
  }
  if (!udp.begin(UDP_PORT)) {
    Serial.println(F("[WIFI] UDP socket startup failed; access point disabled."));
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    return;
  }

  firmwareUpdateServer.on("/update", HTTP_POST, handleFirmwareUpload,
                          handleFirmwareUploadData);
  firmwareUpdateServer.on("/mission", HTTP_POST, handleMissionUpload,
                          handleMissionUploadData);
  firmwareUpdateServer.begin();

  enabled = true;
  Serial.printf("[WIFI] Hidden AP %s ready at 192.168.4.1; UDP %u, authenticated firmware update /update and mission upload /mission.\r\n",
                ssid, UDP_PORT);
}

void syncFromSettings(bool persisted) {
  if (!liveSettings) {
    return;
  }
  pendingSettings = *liveSettings;
  pendingDirty = false;
  observedSettings = *liveSettings;
  if (persisted) {
    savedSettings = *liveSettings;
  }
}

bool firmwareUpdateInProgress() {
  return updateInProgress || updateActivityThisCall;
}

void update() {
  if (!enabled) {
    return;
  }

  updateActivityThisCall = updateInProgress;
  firmwareUpdateServer.handleClient();
  if (updateActivityThisCall || updateInProgress || missionUploadInProgress) {
    while (udp.parsePacket() > 0) {
      while (udp.available() > 0) {
        udp.read();
      }
    }
    return;
  }

  const int packetSize = udp.parsePacket();
  if (packetSize <= 0) {
    return;
  }

  const IPAddress remoteIp = udp.remoteIP();
  const uint16_t remotePort = udp.remotePort();
  const uint32_t now = millis();
  if (now - lastRequestAtMs < MIN_REQUEST_INTERVAL_MS) {
    while (udp.available()) {
      udp.read();
    }
    return;
  }
  lastRequestAtMs = now;

  if (remoteIp[0] != 192 || remoteIp[1] != 168 || remoteIp[2] != 4 ||
      remoteIp == WiFi.softAPIP()) {
    while (udp.available()) {
      udp.read();
    }
    return;
  }

  if (static_cast<size_t>(packetSize) > MAX_REQUEST_BYTES) {
    while (udp.available()) {
      udp.read();
    }
    responseDocument["ok"] = false;
    responseDocument["error"] = "request exceeds 384 bytes";
    sendReply(remoteIp, remotePort);
    return;
  }

  const int bytesRead = udp.read(packetBuffer, MAX_REQUEST_BYTES);
  if (bytesRead <= 0 || bytesRead != packetSize) {
    while (udp.available()) {
      udp.read();
    }
    return;
  }
  packetBuffer[bytesRead] = '\0';
  handlePacket(packetBuffer, remoteIp, remotePort);
}

}  // namespace WirelessConfig
