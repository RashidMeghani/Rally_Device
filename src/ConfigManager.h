// Persistent configuration (ESP32 Preferences/NVS) with an explicit
// version/magic marker so a blank/erased device (NVS reads back as 0xFF)
// can never be mistaken for valid configuration (spec section 4 & 18).
#pragma once

#include <Arduino.h>
#include <cstdint>

// Bitmask for the "enabled NMEA sentence types" HTML setting (section 27).
// GNGTV is carried as its own bit rather than folded into VTG until the
// literal receiver-output identifier is verified (see open questions).
enum NmeaSentenceBit : uint8_t {
    NMEA_RMC   = 1 << 0,
    NMEA_GGA   = 1 << 1,
    NMEA_VTG   = 1 << 2,
    NMEA_GNGTV = 1 << 3,
};

struct AppConfig {
    uint32_t magic = 0;    // set only by save(); distinguishes "loaded" from "defaulted"
    uint16_t version = 0;

    char deviceId[16] = "RD-01";
    char phoneNumbers[5][16] = {{0}, {0}, {0}, {0}, {0}};

    // Race logging
    float loggingRateHz = 5.0f; // configurable log rate, independent of GNSS output rate

    // Route/geofence thresholds (baseline values from AppConstants, kept
    // mutable here so they can be tuned from the HTML settings page).
    float gnssAccuracyThresholdM = 15.0f;
    float routeMatchThresholdM = 150.0f;

    // M8N settings exposed on the HTML page (section 27): baud rate,
    // refresh/update rate, enabled sentence set.
    uint32_t gnssBaud = 115200;
    uint8_t gnssRateHz = 5;         // up to project maximum of 10 Hz
    uint8_t gnssEnabledSentences = NMEA_RMC | NMEA_GGA | NMEA_GNGTV;

    // LoRa (kept centralized/configurable per section 14/24)
    long loraFreqHz = 433000000L;
};

class ConfigManager {
public:
    bool begin(); // loads from NVS, or writes+loads deterministic defaults
    const AppConfig& get() const { return _cfg; }

    // Validates and applies a full replacement config, then persists it.
    // Returns false (config unchanged in NVS) if validation fails.
    bool save(const AppConfig& newCfg);

    void resetToDefaults();

private:
    static constexpr uint32_t MAGIC = 0x52444331; // "RDC1"
    static constexpr uint16_t CURRENT_VERSION = 1;

    AppConfig _cfg;

    bool validate(const AppConfig& c) const;
    void loadDefaults(AppConfig& c) const;
    bool readFromNvs(AppConfig& out) const;
    bool writeToNvs(const AppConfig& c);
};
