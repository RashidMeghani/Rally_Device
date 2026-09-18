// Tunable thresholds and defaults taken directly from the spec baseline.
// Values marked "baseline" are owner-confirmed starting points and are
// exposed through ConfigManager so they can be adjusted without recompiling.
#pragma once

#include <cstdint>

namespace AppConst {

// --- Serial diagnostics -----------------------------------------------------
// See DebugLog.h. false removes the output and its formatting cost entirely
// at compile time; the port is still opened, so flashing is unaffected.
//
// Turn DEBUG_SERIAL on for bench testing with a serial monitor attached, off
// for a race, where nothing is connected and the output only competes with
// GNSS consumption for loop time.
constexpr bool DEBUG_SERIAL          = true;
// The per-sentence raw NMEA echo, separately switchable because it alone is
// ~1 KB/s at 5 Hz - more than everything else put together. Leave it off
// unless the question is specifically about what the receiver is emitting.
constexpr bool DEBUG_SERIAL_RAW_NMEA = false;


// --- GNSS / route matching -------------------------------------------------
constexpr float GNSS_ACCURACY_BASELINE_M   = 15.0f;   // required fix quality for corrected-distance acceptance
constexpr float ROUTE_MATCH_THRESHOLD_M    = 150.0f;  // lateral acceptance threshold
constexpr float ROUTE_SEGMENT_LENGTH_M     = 2000.0f; // 2 km top-level segmentation

// How often the live position is re-matched against the ReferenceMap index
// and the corrected race distance re-derived from it (owner requirement).
// Between corrections, distance accrues from GNSS point-to-point movement.
// NOT YET ACTIVE: this is consumed by RouteMatcher, which is not built yet
// (see AppController.h's scope note) - the constant is here so the cadence
// is pinned down rather than rediscovered later.
constexpr uint32_t ROUTE_CORRECTION_INTERVAL_MS = 2UL * 60UL * 1000UL; // 2 minutes

// Fastest the vehicle could conceivably travel, used to bound how far the
// corrected distance may legitimately move between corrections. A match
// can be geometrically perfect yet still be the wrong place on a route
// that doubles back near itself; nothing can move further than this.
constexpr float ROUTE_MAX_PLAUSIBLE_KMH = 200.0f;

// Retry cadence while the device still has NO route match at all - i.e.
// reacquiring after a reset (spec section 7). The first fix must not wait
// a full correction interval to find out where it is. Kept short but not
// per-tick: a hintless match is a full-route scan, so failed attempts
// (off-route, or poor accuracy) must stay rate-limited.
constexpr uint32_t ROUTE_REACQUIRE_RETRY_MS = 5000;
constexpr float REFERENCE_MAP_MAX_KM       = 250.0f;
constexpr uint8_t GNSS_MAX_RATE_HZ         = 10;
constexpr uint32_t GNSS_DEFAULT_BAUD       = 115200;

// How stale a GNSS field may be before it is treated as unavailable.
// TinyGPS++'s isValid() stays true forever once a field has been set even
// once, so it alone cannot detect a lost fix - freshness must be checked
// via age() as well, or the display would keep showing the last known
// values indefinitely after the antenna is unplugged.
constexpr uint32_t GNSS_FIX_MAX_AGE_MS     = 3000;

// Largest HDOP still treated as a real reading. Receivers emit a sentinel
// HDOP (typically 99.99) on sentences with no fix, and that slips through
// an age check because such sentences keep arriving; left unfiltered it
// renders as a bogus ~500 m accuracy that flickers in and out whenever a
// single degraded sentence lands between good ones. Real-world usable HDOP
// is ~0.5-20, so 50 rejects the sentinel without ever rejecting a
// genuinely poor but real fix.
constexpr float GNSS_MAX_PLAUSIBLE_HDOP    = 50.0f;

// --- Battery -----------------------------------------------------------------
// At or below the critical percentage the device closes every open file and
// halts race operations, so the pack dies with the SD card in a consistent
// state rather than mid-write. Requires the condition to hold for several
// consecutive samples: a 2S pack sags hard under a current surge, and a
// momentary dip must not be mistaken for a flat battery. Recovery needs a
// meaningfully higher level (hysteresis) so the device cannot flap in and
// out of the halted state.
// Percentages follow a real Li-ion discharge curve (see BatteryManager),
// so these map to: 2% ~ 6.27V (3.14V/cell), 10% ~ 7.31V, 20% ~ 7.47V.
// Note how little voltage separates them - that non-linearity is exactly
// why these are expressed as percentages rather than voltage thresholds.
constexpr uint8_t BATTERY_CRITICAL_PERCENT   = 2;
constexpr uint8_t BATTERY_RECOVER_PERCENT    = 10;
constexpr uint8_t BATTERY_LOW_PERCENT        = 20;   // "LOW" warning prefix on the display
constexpr uint8_t BATTERY_CRITICAL_SAMPLES   = 6;    // x SAMPLE_INTERVAL_MS (500ms) = 3s sustained

// Below this the reading is treated as "no battery monitoring fitted"
// rather than as a flat pack. Without it, a board whose divider is not
// installed yet reads ~0V -> 0% -> and would shut itself down on boot.
constexpr float BATTERY_PRESENT_MIN_V        = 3.0f;

// Charge that exists in the cells but the device cannot actually use,
// expressed as raw-discharge-curve percent. The hardware browns out well
// above the cell's chemical empty point (measured on this board: it cut
// out at a raw-curve 4%), so the displayed percentage is rescaled to treat
// that as 0% - the way a phone reports 0% at its own shutdown voltage
// rather than at 3.0V/cell. Reporting "4% left" while the device dies is
// simply wrong, and no threshold below the brownout point can ever fire.
//
// Set 1 point above the measured 4% so the display reaches 0% just BEFORE
// the hardware gives up rather than after.
//
// RE-MEASURE whenever the load or the pack changes, because both move the
// brownout point:
//   - GSM/LoRa: a SIM800L transmit burst pulls ~2A; the added sag raises
//     the brownout point, so the reserve needs RAISING.
//   - Pack condition: this was measured on a visibly tired cell. Ageing
//     raises internal resistance, so a weak pack sags to the brownout
//     voltage while still holding charge. A healthy race pack sags less
//     and would run past this point - meaning the reserve would be too
//     high, the display would read 0% early, and the critical halt would
//     stop logging with usable charge left. Re-measure on the pack that
//     will actually be raced and LOWER the reserve accordingly.
constexpr uint8_t BATTERY_USABLE_RESERVE_PCT = 5;

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
//
// A crossing is detected as the zero-crossing of the vehicle's ALONG-TRACK
// offset from the point - how far past the point it is, measured along the
// direction the recon lap was driven there. That function is a straight
// line through zero with slope equal to road speed, so the moment of
// crossing is sharply defined and can be interpolated BETWEEN two fixes.
//
// The previous approach - watching the straight-line distance to the point
// for a local minimum - could not work well, for a reason that is in the
// shape of the curve rather than in any constant: distance-to-point is FLAT
// at its minimum (that is what a minimum is), so a few metres of GNSS noise
// moved the apparent crossing by tens of metres. That is what latched a
// crossing 17 m short of a point in testing, and why a car standing on the
// start line could never be timed correctly at all - its closest approach
// to the start point happens while it is parked, not when it drives off.
//
// Both windows below are symmetric about the point: they open on the way IN
// and stay open the same distance on the way OUT, so a checkpoint's label,
// distance and captured crossing time remain readable after it is passed
// rather than vanishing the instant the target advances.
constexpr float GEOFENCE_LABEL_SHOW_M      = 150.0f;  // show label, approaching and departing
constexpr float GEOFENCE_PRECISE_ZONE_M    = 100.0f;  // show distance, and watch for the crossing

// How far past the point the vehicle must get before a detected crossing is
// committed. The crossing TIME is already interpolated at the sign change,
// so this confirmation delay costs nothing in accuracy - it only rejects a
// sign change that noise produced and that the vehicle does not follow
// through. 10 m is far beyond what GNSS jitter can fake.
constexpr float GEOFENCE_CROSS_CONFIRM_M   = 10.0f;

// How far back across the line the vehicle must go before a detected
// crossing is abandoned as spurious. Without this hysteresis a candidate is
// dropped the moment noise nudges the offset back past zero, which at low
// speed - where a fix moves the vehicle less than the noise does - loses
// about one crossing in a hundred and biases the rest late. 3 m is inside
// the confirmation distance and outside the jitter.
constexpr float GEOFENCE_CROSS_DISCARD_M   = 3.0f;

// How far to either side of the point the crossing may occur and still
// count. The along-track test alone describes an infinite line across the
// map, which a vehicle on a parallel road would eventually cross; this
// bounds it to a corridor around the point. Wide enough to cover any road
// plus the offset between the surveyed point and the driven line.
constexpr float GEOFENCE_CROSS_CORRIDOR_M  = 50.0f;

// A crossing requires the vehicle to be moving at all. Standing still, GNSS
// noise alone makes the along-track offset wander across zero; below this,
// no sign-change test runs, so a stationary vehicle can never produce a
// candidate. This is a detector guard, not a race rule.
constexpr float GEOFENCE_MOVING_MIN_KMH    = 2.0f;

// Race rule: how fast the vehicle must be going AT THE LINE for the
// crossing to count. Speed is interpolated to the crossing instant the same
// way the time is, so this judges the moment of crossing rather than
// whatever the speed happened to be at some nearby fix.
//
// Under the previous detector this gate did double duty as noise
// suppression, which forced it high enough to reject genuine slow
// crossings. It no longer has that job - a parked vehicle cannot produce a
// crossing at all now - so it is purely the sporting rule it was meant to
// be, and the standing-start case can have its own lower value.
constexpr float GEOFENCE_CROSSING_MIN_KMH  = 10.0f;

// The gate applied when the vehicle CROSSED FROM A STANDSTILL - it was
// stationary before the line and accelerated across it. This is detected
// from the crossing itself (the sample before it was the parked anchor),
// not from which point it is, so it applies wherever a standing start
// actually happens and needs no per-point configuration.
//
// Why it needs to be lower: a car starting 2 m short of the line reaches
// only sqrt(2*a*2) at the line - about 16 km/h at 0.5 g, but 11 km/h at
// 0.25 g and under 9 km/h at 0.15 g. A 10 km/h gate would reject a gentle
// getaway; 5 km/h leaves real margin while still rejecting a crawl.
constexpr float GEOFENCE_STANDING_START_MIN_KMH = 5.0f;

// --- Race logging lifecycle -------------------------------------------------
constexpr float LOG_MOVING_MIN_KMH         = 2.0f;    // log while moving above this speed
constexpr uint32_t LOG_STOP_TIMEOUT_MS     = 20UL * 60UL * 1000UL; // 20 minutes stopped -> close log

// --- Give Way / Overtake ----------------------------------------------------
constexpr float OVERTAKE_ELIGIBLE_M        = 183.0f;  // ~600 ft
constexpr uint32_t OVERTAKE_COMM_TIMEOUT_MS = 30000;  // session reset if comms silent this long
constexpr uint32_t LORA_EVENT_RETRY_MS     = 1000;    // ~1s retry cadence for checkpoint events

// --- OLED timing -------------------------------------------------------------
constexpr uint32_t SPLASH_DURATION_MS      = 3000;
constexpr uint32_t INIT_STEP_INTERVAL_MS   = 400;     // reveal boot steps one at a time, not all at once
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
