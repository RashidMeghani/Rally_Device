#include "ConfigManager.h"
#include <Preferences.h>

namespace {
Preferences prefs;
constexpr const char* NVS_NAMESPACE = "rdcfg";
constexpr const char* NVS_BLOB_KEY = "cfg";
}

void ConfigManager::loadDefaults(AppConfig& c) const {
    c = AppConfig{}; // struct default-member-initializers are the deterministic defaults
    c.magic = MAGIC;
    c.version = CURRENT_VERSION;
}

bool ConfigManager::validate(const AppConfig& c) const {
    if (c.magic != MAGIC) return false;
    if (c.gnssBaud != 4800 && c.gnssBaud != 9600 && c.gnssBaud != 19200 &&
        c.gnssBaud != 38400 && c.gnssBaud != 57600 && c.gnssBaud != 115200) return false;
    if (c.gnssRateHz < 1 || c.gnssRateHz > 10) return false;
    if (c.loggingRateHz <= 0 || c.loggingRateHz > 10) return false;
    if (c.gnssAccuracyThresholdM <= 0) return false;
    if (c.routeMatchThresholdM <= 0) return false;
    if (c.deviceId[0] == '\0') return false;
    if (c.utcOffsetMinutes < AppConst::UTC_OFFSET_MIN_LIMIT ||
        c.utcOffsetMinutes > AppConst::UTC_OFFSET_MAX_LIMIT) return false;
    return true;
}

bool ConfigManager::readFromNvs(AppConfig& out) const {
    if (!prefs.begin(NVS_NAMESPACE, /*readOnly=*/true)) return false;
    size_t expected = sizeof(AppConfig);
    size_t got = prefs.getBytes(NVS_BLOB_KEY, &out, expected);
    prefs.end();
    return got == expected && out.magic == MAGIC && out.version == CURRENT_VERSION;
}

bool ConfigManager::writeToNvs(const AppConfig& c) {
    if (!prefs.begin(NVS_NAMESPACE, /*readOnly=*/false)) return false;
    size_t written = prefs.putBytes(NVS_BLOB_KEY, &c, sizeof(AppConfig));
    prefs.end();
    return written == sizeof(AppConfig);
}

bool ConfigManager::begin() {
    AppConfig loaded;
    if (readFromNvs(loaded) && validate(loaded)) {
        _cfg = loaded;
        Serial.println("[Config] Loaded valid configuration from NVS");
        return true;
    }

    Serial.println("[Config] No valid configuration found - writing deterministic defaults");
    loadDefaults(_cfg);
    if (!writeToNvs(_cfg)) {
        Serial.println("[Config] WARNING: failed to persist defaults to NVS (running with in-RAM defaults)");
    }
    return true;
}

bool ConfigManager::save(const AppConfig& newCfg) {
    AppConfig candidate = newCfg;
    candidate.magic = MAGIC;
    candidate.version = CURRENT_VERSION;
    if (!validate(candidate)) {
        Serial.println("[Config] REJECTED: proposed configuration failed validation");
        return false;
    }
    if (!writeToNvs(candidate)) {
        Serial.println("[Config] ERROR: failed to persist configuration");
        return false;
    }
    _cfg = candidate;
    Serial.println("[Config] Configuration saved");
    return true;
}

void ConfigManager::resetToDefaults() {
    loadDefaults(_cfg);
    writeToNvs(_cfg);
}
