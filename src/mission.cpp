#include "mission.h"
#include <LittleFS.h>
#include <ArduinoJson.h>

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
    "target_cm": 50
  },
  {
    "langkah": 2,
    "nama": "Berenang Maju Lurus",
    "aksi": "maju",
    "daya": 20,
    "durasi_detik": 12
  },
  {
    "langkah": 3,
    "nama": "Putar Haluan Kanan",
    "aksi": "putar",
    "daya": 15,
    "durasi_detik": 4
  },
  {
    "langkah": 4,
    "nama": "Berenang Kembali",
    "aksi": "maju",
    "daya": 20,
    "durasi_detik": 12
  },
  {
    "langkah": 5,
    "nama": "Naik ke Permukaan",
    "aksi": "surface",
    "daya": 15,
    "durasi_detik": 4
  }
])";

bool parseJsonDoc(const JsonDocument& doc, String& errMessage) {
  JsonArrayConst arr = doc.as<JsonArrayConst>();
  if (arr.isNull()) {
    errMessage = "JSON root must be an array of step objects.";
    return false;
  }
  if (arr.size() == 0) {
    errMessage = "JSON array is empty.";
    return false;
  }

  size_t count = 0;
  Step tempSteps[MAX_STEPS];

  for (JsonObjectConst item : arr) {
    if (count >= MAX_STEPS) {
      Serial.printf("[WARN] Trajectory exceeds MAX_STEPS (%u). Truncating.\r\n", static_cast<unsigned>(MAX_STEPS));
      break;
    }

    Step& step = tempSteps[count];
    step.stepNum = item["langkah"] | item["step"] | static_cast<uint16_t>(count + 1);

    const char* rawName = item["nama"] | item["name"] | "";
    if (rawName && strlen(rawName) > 0) {
      strncpy(step.name, rawName, sizeof(step.name) - 1);
      step.name[sizeof(step.name) - 1] = '\0';
    } else {
      snprintf(step.name, sizeof(step.name), "Langkah %u", step.stepNum);
    }

    const char* rawAct = item["aksi"] | item["action"] | "";
    step.action = stringToAction(rawAct ? rawAct : "");

    step.power = item["daya"] | item["power"] | 0.0f;
    step.durationSec = item["durasi_detik"] | item["durasi"] | item["duration"] | item["duration_sec"] | 0.0f;
    step.targetCm = item["target_cm"] | item["target"] | item["depth_cm"] | 0.0f;

    count++;
  }

  stepCount = count;
  for (size_t i = 0; i < count; ++i) {
    loadedSteps[i] = tempSteps[i];
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
    default:                 return "UNKNOWN";
  }
}

bool begin() {
  if (!LittleFS.begin()) {
    Serial.println(F("[LittleFS] Warning: LittleFS mount failed, formatting..."));
    if (LittleFS.format() && LittleFS.begin()) {
      Serial.println(F("[LittleFS] Formatted and mounted successfully."));
    } else {
      Serial.println(F("[LittleFS] ERROR: LittleFS mount failed!"));
      return false;
    }
  }

  if (!LittleFS.exists(JALUR_FILE)) {
    Serial.println(F("[LittleFS] /jalur.json not found. Creating default trajectory..."));
    saveDefaultFile();
  } else {
    loadFromFile();
  }
  return true;
}

bool saveDefaultFile(const char* path) {
  File f = LittleFS.open(path, "w");
  if (!f) {
    Serial.printf("[LittleFS] Failed to open '%s' for writing default.\r\n", path);
    return false;
  }
  f.print(DEFAULT_JALUR_JSON);
  f.close();

  return loadFromFile(path);
}

bool loadFromFile(const char* path) {
  if (!LittleFS.exists(path)) {
    Serial.printf("[JALUR] File '%s' not found on LittleFS.\r\n", path);
    return false;
  }

  File f = LittleFS.open(path, "r");
  if (!f) {
    Serial.printf("[JALUR] Failed to open '%s' for reading.\r\n", path);
    return false;
  }

  DynamicJsonDocument doc(4096);
  DeserializationError err = deserializeJson(doc, f);
  f.close();

  if (err) {
    Serial.printf("[JALUR] JSON deserialization error from '%s': %s\r\n", path, err.c_str());
    return false;
  }

  String errMessage;
  if (!parseJsonDoc(doc, errMessage)) {
    Serial.printf("[JALUR] Error parsing '%s': %s\r\n", path, errMessage.c_str());
    return false;
  }

  Serial.printf("[JALUR] Loaded %u steps from '%s'.\r\n",
                static_cast<unsigned>(stepCount), path);
  return true;
}

bool saveJsonString(const String& jsonStr, String& errMessage) {
  DynamicJsonDocument doc(4096);
  DeserializationError err = deserializeJson(doc, jsonStr);
  if (err) {
    errMessage = String("JSON parse error: ") + err.c_str();
    return false;
  }

  if (!parseJsonDoc(doc, errMessage)) {
    return false;
  }

  File f = LittleFS.open(JALUR_FILE, "w");
  if (!f) {
    errMessage = "Failed to open /jalur.json on LittleFS for writing.";
    return false;
  }

  serializeJsonPretty(doc, f);
  f.close();
  return true;
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
  Serial.println(F("\033[1;36m╔═════════════════════════════════════════════════════════════════════════════╗"));
  Serial.println(F("║                   AUV SAUVAC MISSION TRAJECTORY (JALUR)                     ║"));
  Serial.println(F("║                         Loaded from /jalur.json                             ║"));
  Serial.println(F("╚═════════════════════════════════════════════════════════════════════════════╝\033[0m"));

  if (stepCount == 0) {
    Serial.println(F("\033[33m[JALUR] No mission steps loaded in memory!\033[0m"));
    Serial.println(F("Type 'jalur default' to generate the standard SAUVC mission trajectory."));
    Serial.println();
    return;
  }

  Serial.println(F(" No | Aksi/Action | Nama Langkah             | Daya | Durasi | Target Kedalaman"));
  Serial.println(F("----+-------------+--------------------------+------+--------+-----------------"));

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

    Serial.printf(" %2u | %-11s | %-24s | %4s | %6s | %15s\r\n",
                  static_cast<unsigned>(i + 1),
                  actionToString(s.action),
                  s.name,
                  pwrBuf,
                  durBuf,
                  depthBuf);
  }

  Serial.println(F("-------------------------------------------------------------------------------"));
  Serial.printf("Total: %u langkah | Estimasi Durasi: %.1f detik | Daya Maksimum: %.0f%%\r\n",
                static_cast<unsigned>(stepCount), totalDuration, maxPwr);
  Serial.println();
}

}  // namespace Mission
