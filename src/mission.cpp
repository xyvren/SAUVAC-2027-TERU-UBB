#include "mission.h"
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <math.h>

namespace Mission {

namespace {

Step loadedSteps[MAX_STEPS];
size_t stepCount = 0;

constexpr const char* DEFAULT_JALUR_JSON = R"([
  {
    "langkah": 1,
    "nama": "Menyelam ke 50 cm",
    "aksi": "dive",
    "daya": 15,
    "durasi_detik": 5,
    "target_cm": 50,
    "motors": [5, 6]
  },
  {
    "langkah": 2,
    "nama": "Berenang Maju Lurus",
    "aksi": "maju",
    "daya": 20,
    "durasi_detik": 12,
    "target_cm": 50,
    "motors": [1, 2, 3, 4, 5, 6]
  },
  {
    "langkah": 3,
    "nama": "Putar Haluan Kanan",
    "aksi": "putar",
    "daya": 15,
    "durasi_detik": 4,
    "target_cm": 50,
    "motors": [1, 2, 3, 4, 5, 6]
  },
  {
    "langkah": 4,
    "nama": "Berenang Kembali",
    "aksi": "maju",
    "daya": 20,
    "durasi_detik": 12,
    "target_cm": 50,
    "motors": [1, 2, 3, 4, 5, 6]
  },
  {
    "langkah": 5,
    "nama": "Naik ke Permukaan",
    "aksi": "surface",
    "daya": 15,
    "durasi_detik": 4,
    "motors": [5, 6]
  }
])";

bool readNumber(JsonVariantConst value, float& result) {
  if (value.isNull() || value.is<const char*>() || value.is<bool>() ||
      value.is<JsonArrayConst>() || value.is<JsonObjectConst>()) {
    return false;
  }
  result = value.as<float>();
  return isfinite(result);
}

bool parseJsonDoc(const JsonDocument& doc, Step* parsedSteps, size_t& parsedCount,
                  String& errMessage) {
  JsonArrayConst arr = doc.as<JsonArrayConst>();
  if (arr.isNull()) {
    errMessage = "JSON root must be an array of step objects.";
    return false;
  }
  if (arr.size() == 0) {
    errMessage = "JSON array is empty.";
    return false;
  }
  if (arr.size() > MAX_STEPS) {
    errMessage = "Mission exceeds MAX_STEPS (32); no steps were loaded.";
    return false;
  }

  size_t count = 0;
  Step tempSteps[MAX_STEPS];

  for (JsonObjectConst item : arr) {
    if (item.isNull()) {
      errMessage = "Each mission step must be a JSON object.";
      return false;
    }

    Step& step = tempSteps[count];
    JsonVariantConst stepNumValue = item["langkah"];
    if (stepNumValue.isNull()) stepNumValue = item["step"];
    if (stepNumValue.isNull()) {
      step.stepNum = static_cast<uint16_t>(count + 1);
    } else {
      float stepNum = 0.0f;
      if (!readNumber(stepNumValue, stepNum) || stepNum < 1.0f ||
          stepNum > 65535.0f || floorf(stepNum) != stepNum) {
        errMessage = "Step number must be an integer between 1 and 65535.";
        return false;
      }
      step.stepNum = static_cast<uint16_t>(stepNum);
    }

    const char* rawName = item["nama"] | item["name"] | "";
    if (!rawName || strlen(rawName) == 0 || strlen(rawName) >= sizeof(step.name)) {
      errMessage = "Step name must contain 1-35 characters.";
      return false;
    }
    strncpy(step.name, rawName, sizeof(step.name) - 1);
    step.name[sizeof(step.name) - 1] = '\0';

    const char* rawAct = item["aksi"] | item["action"] | "";
    step.action = stringToAction(rawAct ? rawAct : "");
    if (step.action == Action::UNKNOWN) {
      errMessage = "Step action is missing or unsupported.";
      return false;
    }

    JsonVariantConst powerValue = item["daya"];
    if (powerValue.isNull()) powerValue = item["power"];
    JsonVariantConst durationValue = item["durasi_detik"];
    if (durationValue.isNull()) durationValue = item["durasi"];
    if (durationValue.isNull()) durationValue = item["duration"];
    if (durationValue.isNull()) durationValue = item["duration_sec"];
    JsonVariantConst targetValue = item["target_cm"];
    if (targetValue.isNull()) targetValue = item["target"];
    if (targetValue.isNull()) targetValue = item["depth_cm"];
    step.targetCm = 0.0f;
    if (!readNumber(powerValue, step.power) ||
        !readNumber(durationValue, step.durationSec) ||
        (!targetValue.isNull() && !readNumber(targetValue, step.targetCm))) {
      errMessage = "Power, duration, and target must be finite JSON numbers.";
      return false;
    }
    if (step.power < 0.0f || step.power > 100.0f ||
        step.durationSec < 0.1f || step.durationSec > 600.0f ||
        step.targetCm < 0.0f || step.targetCm > 140.0f) {
      errMessage = "Power must be 0-100%, duration 0.1-600s, and target 0-140cm.";
      return false;
    }

    uint8_t mask = 0;
    if (item.containsKey("motors") && item.containsKey("motor_mask")) {
      errMessage = "Specify either motors or motor_mask, not both.";
      return false;
    }
    if (item.containsKey("motors")) {
      JsonVariantConst mVar = item["motors"];
      if (mVar.is<JsonArrayConst>()) {
        JsonArrayConst motors = mVar.as<JsonArrayConst>();
        if (motors.size() == 0) {
          errMessage = "Motors array must not be empty.";
          return false;
        }
        for (JsonVariantConst v : motors) {
          float motorId = 0.0f;
          if (!readNumber(v, motorId) || motorId < 1.0f || motorId > 6.0f ||
              floorf(motorId) != motorId) {
            errMessage = "Motor IDs must be integers between 1 and 6.";
            return false;
          }
          mask |= static_cast<uint8_t>(1U << (static_cast<uint8_t>(motorId) - 1));
        }
      } else {
        float parsedMask = 0.0f;
        if (!readNumber(mVar, parsedMask) || parsedMask < 1.0f ||
            parsedMask > 63.0f || floorf(parsedMask) != parsedMask) {
          errMessage = "Motor mask must be between 1 and 63.";
          return false;
        }
        mask = static_cast<uint8_t>(parsedMask);
      }
    } else if (item.containsKey("motor_mask")) {
      float parsedMask = 0.0f;
      if (!readNumber(item["motor_mask"], parsedMask) ||
          parsedMask < 1.0f || parsedMask > 63.0f || floorf(parsedMask) != parsedMask) {
        errMessage = "Motor mask must be an integer between 1 and 63.";
        return false;
      }
      mask = static_cast<uint8_t>(parsedMask);
    }

    if (mask == 0) {
      mask = getDefaultMotorMask(step.action, step.targetCm);
    }
    if (mask == 0 || (mask & 0xC0) != 0) {
      errMessage = "Mission step must select at least one valid thruster.";
      return false;
    }
    step.motorMask = mask;

    count++;
  }

  parsedCount = count;
  for (size_t i = 0; i < count; ++i) {
    parsedSteps[i] = tempSteps[i];
  }

  return true;
}

bool parseMissionFile(const char* path, Step* parsedSteps, size_t& parsedCount,
                      String& errMessage) {
  File f = LittleFS.open(path, "r");
  if (!f) {
    errMessage = String("Failed to open '") + path + "' for reading.";
    return false;
  }
  if (f.size() == 0 || f.size() > 3500) {
    f.close();
    errMessage = "Mission file must be between 1 and 3500 bytes.";
    return false;
  }

  DynamicJsonDocument doc(4096);
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) {
    errMessage = String("JSON deserialization error: ") + err.c_str();
    return false;
  }
  if (doc.overflowed()) {
    errMessage = "Mission JSON exceeds available parser capacity.";
    return false;
  }
  return parseJsonDoc(doc, parsedSteps, parsedCount, errMessage);
}

bool saveJsonStringToPath(const String& jsonStr, const char* path, String& errMessage) {
  if (jsonStr.length() == 0 || jsonStr.length() > 3500) {
    errMessage = "Mission JSON must be between 1 and 3500 bytes.";
    return false;
  }

  DynamicJsonDocument doc(4096);
  DeserializationError err = deserializeJson(doc, jsonStr);
  if (err) {
    errMessage = String("JSON parse error: ") + err.c_str();
    return false;
  }
  if (doc.overflowed()) {
    errMessage = "Mission JSON exceeds available parser capacity.";
    return false;
  }

  Step parsedSteps[MAX_STEPS];
  size_t parsedCount = 0;
  if (!parseJsonDoc(doc, parsedSteps, parsedCount, errMessage)) {
    return false;
  }

  char tempPath[64];
  char backupPath[64];
  if (snprintf(tempPath, sizeof(tempPath), "%s.tmp", path) >= static_cast<int>(sizeof(tempPath)) ||
      snprintf(backupPath, sizeof(backupPath), "%s.bak", path) >= static_cast<int>(sizeof(backupPath))) {
    errMessage = "Mission file path is too long.";
    return false;
  }

  const size_t expectedBytes = measureJsonPretty(doc);
  File temp = LittleFS.open(tempPath, "w");
  if (!temp) {
    errMessage = "Failed to open temporary mission file for writing.";
    return false;
  }
  const size_t written = serializeJsonPretty(doc, temp);
  temp.flush();
  temp.close();
  if (written != expectedBytes) {
    LittleFS.remove(tempPath);
    errMessage = "Incomplete write while saving mission file.";
    return false;
  }

  Step verifiedSteps[MAX_STEPS];
  size_t verifiedCount = 0;
  String verifyError;
  if (!parseMissionFile(tempPath, verifiedSteps, verifiedCount, verifyError)) {
    LittleFS.remove(tempPath);
    errMessage = String("Temporary mission verification failed: ") + verifyError;
    return false;
  }

  const bool hadOriginal = LittleFS.exists(path);
  LittleFS.remove(backupPath);
  if (hadOriginal && !LittleFS.rename(path, backupPath)) {
    LittleFS.remove(tempPath);
    errMessage = "Failed to preserve the existing mission file.";
    return false;
  }
  if (!LittleFS.rename(tempPath, path)) {
    if (hadOriginal) LittleFS.rename(backupPath, path);
    LittleFS.remove(tempPath);
    errMessage = "Failed to commit the new mission file.";
    return false;
  }
  if (hadOriginal) LittleFS.remove(backupPath);

  stepCount = parsedCount;
  for (size_t i = 0; i < parsedCount; ++i) {
    loadedSteps[i] = parsedSteps[i];
  }
  return true;
}

}  // namespace

Action stringToAction(const String& str) {
  String s = str;
  s.toLowerCase();
  s.trim();
  if (s == "dive" || s == "selam") return Action::DIVE;
  if (s == "maju" || s == "forward") return Action::FORWARD;
  if (s == "mundur" || s == "backward" || s == "reverse") return Action::BACKWARD;
  if (s == "putar" || s == "turn" || s == "yaw") return Action::TURN;
  if (s == "kiri" || s == "sway_left" || s == "swayleft" || s == "left") return Action::SWAY_LEFT;
  if (s == "kanan" || s == "sway_right" || s == "swayright" || s == "right") return Action::SWAY_RIGHT;
  if (s == "hover" || s == "diam" || s == "hold") return Action::HOVER;
  if (s == "surface" || s == "naik") return Action::SURFACE;
  if (s == "tes_t1" || s == "t1" || s == "motor_1" || s == "m1") return Action::MOTOR_1;
  if (s == "tes_t2" || s == "t2" || s == "motor_2" || s == "m2") return Action::MOTOR_2;
  if (s == "tes_t3" || s == "t3" || s == "motor_3" || s == "m3") return Action::MOTOR_3;
  if (s == "tes_t4" || s == "t4" || s == "motor_4" || s == "m4") return Action::MOTOR_4;
  if (s == "tes_t5" || s == "t5" || s == "motor_5" || s == "m5") return Action::MOTOR_5;
  if (s == "tes_t6" || s == "t6" || s == "motor_6" || s == "m6") return Action::MOTOR_6;
  if (s == "tes_all" || s == "all" || s == "all_motors" || s == "semua") return Action::ALL_MOTORS;
  return Action::UNKNOWN;
}

const char* actionToString(Action act) {
  switch (act) {
    case Action::DIVE:       return "DIVE";
    case Action::FORWARD:    return "MAJU";
    case Action::BACKWARD:   return "MUNDUR";
    case Action::TURN:       return "PUTAR";
    case Action::SWAY_LEFT:  return "SWAY_L";
    case Action::SWAY_RIGHT: return "SWAY_R";
    case Action::HOVER:      return "HOVER";
    case Action::SURFACE:    return "SURFACE";
    case Action::MOTOR_1:    return "TES_T1";
    case Action::MOTOR_2:    return "TES_T2";
    case Action::MOTOR_3:    return "TES_T3";
    case Action::MOTOR_4:    return "TES_T4";
    case Action::MOTOR_5:    return "TES_T5";
    case Action::MOTOR_6:    return "TES_T6";
    case Action::ALL_MOTORS: return "TES_ALL";
    default:                 return "UNKNOWN";
  }
}

uint8_t getDefaultMotorMask(Action act, float targetCm) {
  switch (act) {
    case Action::DIVE:
    case Action::SURFACE:
    case Action::HOVER:
      return 0b110000;  // T5, T6
    case Action::FORWARD:
    case Action::BACKWARD:
    case Action::TURN:
    case Action::SWAY_LEFT:
    case Action::SWAY_RIGHT:
      return (targetCm > 0.0f) ? 0b111111 : 0b001111;  // T1-T4 (+ T5,T6 if depth target)
    case Action::MOTOR_1:
      return (1 << 0);
    case Action::MOTOR_2:
      return (1 << 1);
    case Action::MOTOR_3:
      return (1 << 2);
    case Action::MOTOR_4:
      return (1 << 3);
    case Action::MOTOR_5:
      return (1 << 4);
    case Action::MOTOR_6:
      return (1 << 5);
    case Action::ALL_MOTORS:
      return 0b111111;
    default:
      return 0b111111;
  }
}

void maskToString(uint8_t mask, char* out, size_t outSize) {
  if ((mask & 0x3F) == 0x3F) {
    snprintf(out, outSize, "T1-T6 (All)");
    return;
  }
  if ((mask & 0x3F) == 0x0F) {
    snprintf(out, outSize, "T1-T4 (Horiz)");
    return;
  }
  if ((mask & 0x3F) == 0x30) {
    snprintf(out, outSize, "T5,T6 (Vert)");
    return;
  }
  char buf[32] = "";
  bool first = true;
  for (int i = 0; i < 6; ++i) {
    if (mask & (1 << i)) {
      if (!first) strncat(buf, ",", sizeof(buf) - strlen(buf) - 1);
      char tStr[6];
      snprintf(tStr, sizeof(tStr), "T%d", i + 1);
      strncat(buf, tStr, sizeof(buf) - strlen(buf) - 1);
      first = false;
    }
  }
  if (first) {
    snprintf(out, outSize, "None");
  } else {
    strncpy(out, buf, outSize - 1);
    out[outSize - 1] = '\0';
  }
}

bool begin() {
  if (!LittleFS.begin()) {
    Serial.println(F("[LittleFS] ERROR: Mount failed; filesystem was not formatted to protect stored missions."));
    return false;
  }

  if (!LittleFS.exists(JALUR_FILE)) {
    Serial.println(F("[LittleFS] /jalur.json not found. Creating default trajectory..."));
    return saveDefaultFile();
  } else {
    return loadFromFile();
  }
}

bool saveDefaultFile(const char* path) {
  String errMessage;
  const bool saved = saveJsonStringToPath(String(DEFAULT_JALUR_JSON), path, errMessage);
  if (!saved) {
    Serial.printf("[LittleFS] Failed to save default mission: %s\r\n", errMessage.c_str());
  }
  return saved;
}

bool loadFromFile(const char* path) {
  if (!LittleFS.exists(path)) {
    Serial.printf("[JALUR] File '%s' not found on LittleFS.\r\n", path);
    return false;
  }

  Step parsedSteps[MAX_STEPS];
  size_t parsedCount = 0;
  String errMessage;
  if (!parseMissionFile(path, parsedSteps, parsedCount, errMessage)) {
    Serial.printf("[JALUR] Error parsing '%s': %s\r\n", path, errMessage.c_str());
    return false;
  }

  stepCount = parsedCount;
  for (size_t i = 0; i < parsedCount; ++i) {
    loadedSteps[i] = parsedSteps[i];
  }
  Serial.printf("[JALUR] Loaded %u steps from '%s'.\r\n",
                static_cast<unsigned>(stepCount), path);
  return true;
}

bool saveJsonString(const String& jsonStr, String& errMessage) {
  return saveJsonStringToPath(jsonStr, JALUR_FILE, errMessage);
}

size_t getStepCount() {
  return stepCount;
}

const Step* getStep(size_t index) {
  if (index < stepCount) {
    return &loadedSteps[index];
  }
  return nullptr;
}

float getTotalDurationSec() {
  float total = 0.0f;
  for (size_t i = 0; i < stepCount; ++i) {
    total += loadedSteps[i].durationSec;
  }
  return total;
}

float getMaxPowerPercent() {
  float maxP = 0.0f;
  for (size_t i = 0; i < stepCount; ++i) {
    if (loadedSteps[i].power > maxP) maxP = loadedSteps[i].power;
  }
  return maxP;
}

void printMissionTable() {
  Serial.println();
  Serial.println(F("\033[1;36m╔═════════════════════════════════════════════════════════════════════════════════════════════════════════╗"));
  Serial.println(F("║                                   AUV SAUVAC MISSION TRAJECTORY (JALUR)                                 ║"));
  Serial.println(F("║                                         Loaded from /jalur.json                                         ║"));
  Serial.println(F("╚═════════════════════════════════════════════════════════════════════════════════════════════════════════╝\033[0m"));

  if (stepCount == 0) {
    Serial.println(F("\033[33m[JALUR] No mission steps loaded in memory!\033[0m"));
    Serial.println(F("Type 'jalur default' to generate the standard SAUVC mission trajectory."));
    Serial.println();
    return;
  }

  Serial.println(F(" No | Aksi/Action | Motor Aktif   | Nama Langkah             | Daya | Durasi | Target Kedalaman"));
  Serial.println(F("----+-------------+---------------+--------------------------+------+--------+-----------------"));

  float totalDuration = 0.0f;
  float maxPwr = 0.0f;

  for (size_t i = 0; i < stepCount; ++i) {
    const Step& s = loadedSteps[i];
    totalDuration += s.durationSec;
    if (s.power > maxPwr) maxPwr = s.power;

    char depthBuf[16];
    if (s.targetCm > 0.0f) {
      snprintf(depthBuf, sizeof(depthBuf), "%.1f cm", s.targetCm);
    } else {
      strncpy(depthBuf, "---", sizeof(depthBuf));
    }

    char pwrBuf[12];
    snprintf(pwrBuf, sizeof(pwrBuf), "%.0f%%", s.power);

    char durBuf[12];
    snprintf(durBuf, sizeof(durBuf), "%.1f s", s.durationSec);

    char maskBuf[18];
    maskToString(s.motorMask, maskBuf, sizeof(maskBuf));

    Serial.printf(" %2u | %-11s | %-13s | %-24s | %4s | %6s | %15s\r\n",
                  static_cast<unsigned>(i + 1),
                  actionToString(s.action),
                  maskBuf,
                  s.name,
                  pwrBuf,
                  durBuf,
                  depthBuf);
  }

  Serial.println(F("---------------------------------------------------------------------------------------------------"));
  Serial.printf("Total: %u langkah | Estimasi Durasi: %.1f detik | Daya Maksimum: %.0f%%\r\n",
                static_cast<unsigned>(stepCount), totalDuration, maxPwr);
  Serial.println();
}

}  // namespace Mission
