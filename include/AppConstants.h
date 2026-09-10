// Tunable thresholds and defaults taken directly from the spec baseline.
// Values marked "baseline" are owner-confirmed starting points and are
// exposed through ConfigManager so they can be adjusted without recompiling.
#pragma once

#include <cstdint>

namespace AppConst {

// --- GNSS / route matching -------------------------------------------------
constexpr float GNSS_ACCURACY_BASELINE_M   = 15.0f;   // required fix quality for corrected-distance acceptance
constexpr float ROUTE_MATCH_THRESHOLD_M    = 150.0f;  // lateral acceptance threshold
constexpr float ROUTE_SEGMENT_LENGTH_M     = 2000.0f; // 2 km top-level segmentation
constexpr float REFERENCE_MAP_MAX_KM       = 250.0f;
constexpr uint8_t GNSS_MAX_RATE_HZ         = 10;
constexpr uint32_t GNSS_DEFAULT_BAUD       = 115200;

// How stale a GNSS field may be before it is treated as unavailable.
// TinyGPS++'s isValid() stays true forever once a field has been set even
// once, so it alone cannot detect a lost fix - freshness must be checked
// via age() as well, or the display would keep showing the last known
// values indefinitely after the antenna is unplugged.
constexpr uint32_t GNSS_FIX_MAX_AGE_MS     = 3000;

// --- Local time --------------------------------------------------------------
// GNSS reports UTC. This offset is applied to displayed times and race-log
// filenames only - never to the raw NMEA written into the logs, which must
// stay verbatim UTC as the authoritative record. Stored in minutes so
// half/quarter-hour zones work; overridable at runtime via
// AppConfig::utcOffsetMinutes.
constexpr int16_t UTC_OFFSET_MINUTES_DEFAULT = 5 * 60; // UTC+5
constexpr int16_t UTC_OFFSET_MIN_LIMIT       = -14 * 60;
constexpr int16_t UTC_OFFSET_MAX_LIMIT       = 14 * 60;

// --- Point geofence detection ----------------------------------------------
constexpr float GEOFENCE_LABEL_SHOW_M      = 100.0f;  // show label on OLED
constexpr float GEOFENCE_PRECISE_ZONE_M    = 50.0f;   // begin closest-approach comparison
constexpr float GEOFENCE_CROSSING_MIN_KMH  = 10.0f;   // normal checkpoint crossing speed gate

// --- Race logging lifecycle -------------------------------------------------
constexpr float LOG_MOVING_MIN_KMH         = 2.0f;    // log while moving above this speed
constexpr uint32_t LOG_STOP_TIMEOUT_MS     = 20UL * 60UL * 1000UL; // 20 minutes stopped -> close log

// --- Give Way / Overtake ----------------------------------------------------
constexpr float OVERTAKE_ELIGIBLE_M        = 183.0f;  // ~600 ft
constexpr uint32_t OVERTAKE_COMM_TIMEOUT_MS = 30000;  // session reset if comms silent this long
constexpr uint32_t LORA_EVENT_RETRY_MS     = 1000;    // ~1s retry cadence for checkpoint events

// --- OLED timing -------------------------------------------------------------
constexpr uint32_t SPLASH_DURATION_MS      = 3000;
constexpr uint32_t INIT_HOLD_MS            = 3000;    // hold the INIT page so boot steps are readable
constexpr uint32_t OLED_REFRESH_INTERVAL_MS = 200;    // ~5 Hz redraw, decoupled from 10 Hz GNSS

// --- LoRa ---------------------------------------------------------------------
constexpr long LORA_FREQ_HZ = 433E6; // owner-confirmed deployment frequency (Rev 3)

// --- SD file layout -----------------------------------------------------------
constexpr const char* PATH_GEOFENCE_FILE   = "/GeoFencing.txt";
constexpr const char* PATH_REFERENCE_MAP   = "/ReferenceMap.log";
constexpr const char* PATH_ROUTE_INDEX_DIR = "/route";
constexpr const char* PATH_ROUTE_INDEX_HDR = "/route/index.hdr";
constexpr const char* PATH_ROUTE_INDEX_CSV = "/route/index.csv";
constexpr const char* PATH_RACE_LOG_DIR    = "/races";
constexpr const char* PATH_LORA_LOG        = "/logs/lora.log";
constexpr const char* PATH_SMS_PENDING     = "/logs/sms_pending.log";

} // namespace AppConst
