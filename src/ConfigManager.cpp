#include "ConfigManager.h"
#include <Preferences.h>
#include "../include/DebugLog.h"

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

    // LoRa profile. SF6 is a special implicit-header mode the driver does
    // not set up for us, so the usable range starts at 7. Transmit power is
    // bounded at 20 because that is the RA-02's PA_BOOST ceiling - and
    // anything above 17 carries the datasheet's sub-1% duty cycle
    // condition, which this device cannot honour during a Give Way
    // handshake (see AppConstants.h).
    if (c.loraFreqHz < 410000000L || c.loraFreqHz > 525000000L) return false;
    if (c.loraSpreadingFactor < 7 || c.loraSpreadingFactor > 12) return false;
    if (c.loraBandwidthHz < 7800 || c.loraBandwidthHz > 500000) return false;
    if (c.loraCodingRate4 < 5 || c.loraCodingRate4 > 8) return false;
    if (c.loraTxPowerDbm < 2 || c.loraTxPowerDbm > 20) return false;
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
        // Before the first log line that uses them, so the stored setting
        // governs from here on rather than the compiled-in default.
        DebugLog::applyConfig(_cfg.debugSerial, _cfg.debugSerialRawNmea);
        LOGLN("[Config] Loaded valid configuration from NVS");
        return true;
    }

    LOGLN("[Config] No valid configuration found - writing deterministic defaults");
    loadDefaults(_cfg);
    DebugLog::applyConfig(_cfg.debugSerial, _cfg.debugSerialRawNmea);
    if (!writeToNvs(_cfg)) {
        LOGLN("[Config] WARNING: failed to persist defaults to NVS (running with in-RAM defaults)");
    }
    return true;
}

bool ConfigManager::save(const AppConfig& newCfg) {
    AppConfig candidate = newCfg;
    candidate.magic = MAGIC;
    candidate.version = CURRENT_VERSION;
    if (!validate(candidate)) {
        LOGLN("[Config] REJECTED: proposed configuration failed validation");
        return false;
    }
    if (!writeToNvs(candidate)) {
        LOGLN("[Config] ERROR: failed to persist configuration");
        return false;
    }
    _cfg = candidate;
    // Applied immediately, so toggling the switch on the settings page
    // takes effect on the next log line rather than at the next boot.
    DebugLog::applyConfig(_cfg.debugSerial, _cfg.debugSerialRawNmea);
    LOGF("[Config] Configuration saved (serial diagnostics %s, raw NMEA %s)\n",
         _cfg.debugSerial ? "on" : "off",
         _cfg.debugSerialRawNmea ? "on" : "off");
    return true;
}

void ConfigManager::resetToDefaults() {
    loadDefaults(_cfg);
    DebugLog::applyConfig(_cfg.debugSerial, _cfg.debugSerialRawNmea);
    writeToNvs(_cfg);
}
