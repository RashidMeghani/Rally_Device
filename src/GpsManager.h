// Live GNSS on Serial2 (u-blox M8N). Two responsibilities kept deliberately
// separate:
//   1. Raw pass-through: every byte received is handed to a caller-supplied
//      line callback verbatim, for faithful logging to ReferenceMap.log /
//      the race log. This path never depends on parsing succeeding.
//   2. Derived live fields (speed, sats, time, accuracy) via TinyGPSPlus,
//      used only for the OLED DATA page and route matching - never for the
//      authoritative log content.
//
// Also owns applying the HTML-configurable M8N settings (section 27): baud
// rate, refresh/update rate, enabled NMEA sentence set. These are "apply on
// demand" actions (triggered from WebManager or at boot with the persisted
// config), not per-fix runtime work, so they are allowed brief blocking
// delays internally - they must never be called from the hot GNSS loop.
#pragma once

#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <functional>
#include "ConfigManager.h"

class GpsManager {
public:
    using RawLineCallback = std::function<void(const char* line)>;

    // Opens Serial2 at cfg.gnssBaud. Does not yet push rate/sentence config
    // to the receiver (call applySettings() explicitly after begin() if a
    // change from the receiver's current running state is required).
    void begin(const AppConfig& cfg);

    // Non-blocking: drains whatever bytes are currently available, feeds
    // each to TinyGPSPlus and, once a full line is assembled, to the raw
    // line callback. Call every main loop iteration.
    void loop();

    void setRawLineCallback(RawLineCallback cb) { _rawCallback = std::move(cb); }

    // Applies baud/rate/sentence-set from cfg to the receiver over Serial2.
    // Brief blocking (a handful of ms per command) - call only from a
    // settings-apply action, never from the per-fix hot path.
    bool applySettings(const AppConfig& cfg);

    // --- Live derived fields for DisplayManager / RouteMatcher ---
    bool hasFix() const { return _tinyGps.location.isValid(); }
    double latitude() const { return _tinyGps.location.lat(); }
    double longitude() const { return _tinyGps.location.lng(); }
    float speedKmh() const { return _tinyGps.speed.kmph(); }
    uint8_t satellites() const { return (uint8_t)_tinyGps.satellites.value(); }
    // Approximate horizontal accuracy from HDOP (GGA). Documented
    // assumption, not an owner-specified value: accuracy ~= HDOP * UERE,
    // with UERE taken as a conservative 5 m for an uncorrected M8N. This is
    // the "chosen/verified GNSS accuracy source" the spec calls for; if the
    // receiver ever exposes a native accuracy estimate that should replace
    // this approximation.
    float accuracyMeters() const { return ((float)_tinyGps.hdop.value() / 100.0f) * 5.0f; }
    bool timeValid() const { return _tinyGps.time.isValid() && _tinyGps.date.isValid(); }
    uint8_t hour() const { return _tinyGps.time.hour(); }
    uint8_t minute() const { return _tinyGps.time.minute(); }
    uint8_t second() const { return _tinyGps.time.second(); }
    uint8_t centisecond() const { return _tinyGps.time.centisecond(); }
    uint16_t year() const { return _tinyGps.date.year(); }
    uint8_t month() const { return _tinyGps.date.month(); }
    uint8_t day() const { return _tinyGps.date.day(); }

private:
    TinyGPSPlus _tinyGps;
    RawLineCallback _rawCallback;
    char _lineBuf[128];
    size_t _lineLen = 0;
    uint32_t _currentBaud = 115200;

    bool sendPubxSentenceRate(uint8_t msgIdMajor, uint8_t msgIdMinor, uint8_t rateOnUart1);
    bool sendPubxBaud(uint32_t newBaud);
    bool sendUbxCfgRate(uint16_t measRateMs);
    void sendUbxFrame(uint8_t msgClass, uint8_t msgId, const uint8_t* payload, size_t len);
};
