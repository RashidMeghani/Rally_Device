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
#include "../include/AppConstants.h"

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
    //
    // Every "valid" query below checks BOTH isValid() and age(). TinyGPS++'s
    // isValid() only means "this field has been populated at least once
    // since boot" - it never returns to false when the fix is lost, so on
    // its own it would keep reporting a stale position/speed/satellite
    // count forever after the antenna is unplugged. Pairing it with a
    // freshness window is what makes a lost fix actually detectable.
    bool hasFix() const { return isFresh(_tinyGps.location.isValid(), _tinyGps.location.age()); }
    double latitude() const { return _tinyGps.location.lat(); }
    double longitude() const { return _tinyGps.location.lng(); }
    bool speedValid() const { return isFresh(_tinyGps.speed.isValid(), _tinyGps.speed.age()); }
    float speedKmh() const { return _tinyGps.speed.kmph(); }
    bool satellitesValid() const { return isFresh(_tinyGps.satellites.isValid(), _tinyGps.satellites.age()); }
    uint8_t satellites() const { return (uint8_t)_tinyGps.satellites.value(); }
    // Approximate horizontal accuracy from HDOP (GGA). Documented
    // assumption, not an owner-specified value: accuracy ~= HDOP * UERE,
    // with UERE taken as a conservative 5 m for an uncorrected M8N. This is
    // the "chosen/verified GNSS accuracy source" the spec calls for; if the
    // receiver ever exposes a native accuracy estimate that should replace
    // this approximation.
    float accuracyMeters() const { return ((float)_tinyGps.hdop.value() / 100.0f) * 5.0f; }
    bool accuracyValid() const { return isFresh(_tinyGps.hdop.isValid(), _tinyGps.hdop.age()); }
    bool timeValid() const {
        return isFresh(_tinyGps.time.isValid(), _tinyGps.time.age()) &&
               isFresh(_tinyGps.date.isValid(), _tinyGps.date.age());
    }
    uint8_t hour() const { return _tinyGps.time.hour(); }
    uint8_t minute() const { return _tinyGps.time.minute(); }
    uint8_t second() const { return _tinyGps.time.second(); }
    uint8_t centisecond() const { return _tinyGps.time.centisecond(); }
    uint16_t year() const { return _tinyGps.date.year(); }
    uint8_t month() const { return _tinyGps.date.month(); }
    uint8_t day() const { return _tinyGps.date.day(); }

private:
    // mutable: TinyGPS++'s accessors (lat(), hour(), value(), ...) are not
    // declared const upstream - they clear an internal "updated" flag as a
    // side effect of being read. That flag isn't part of what these
    // GpsManager accessors return, so treating _tinyGps as logically
    // read-only from a const context (via mutable) is correct here, rather
    // than stripping const off every accessor below.
    mutable TinyGPSPlus _tinyGps;
    RawLineCallback _rawCallback;
    char _lineBuf[128];
    size_t _lineLen = 0;
    uint32_t _currentBaud = 115200;

    static bool isFresh(bool valid, uint32_t ageMs) {
        return valid && ageMs < AppConst::GNSS_FIX_MAX_AGE_MS;
    }

    bool sendPubxSentenceRate(uint8_t msgIdMajor, uint8_t msgIdMinor, uint8_t rateOnUart1);
    bool sendPubxBaud(uint32_t newBaud);
    bool sendUbxCfgRate(uint16_t measRateMs);
    void sendUbxFrame(uint8_t msgClass, uint8_t msgId, const uint8_t* payload, size_t len);
};
