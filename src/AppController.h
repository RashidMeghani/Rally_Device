// Race-runtime state machine + geofence-crossing orchestration (spec
// sections 8, 10, 11). Owns the single-writer fields of what the spec
// calls "AppState": race stage, corrected/traveled distance, current
// geofence target, and the last captured crossing time.
//
// -----------------------------------------------------------------------
// Honest scope note (see ARCHITECTURE.md section 5 for the full writeup)
// -----------------------------------------------------------------------
// RouteMatcher (spec section 7) is NOT implemented yet. Two consequences,
// both deliberate and flagged rather than silently faked:
//
//   1. OLED Field 5 ("covered/corrected distance") is currently RAW
//      traveled distance - accumulated haversine between consecutive GPS
//      fixes since the START crossing, snapped to each geofence's known
//      distanceFromStartM whenever one is crossed - not true route-
//      corrected distance. It will read close to the real value on a
//      route that doesn't double back on itself, but it is not the same
//      computation the spec describes.
//   2. Reset-recovery (section 8/10: "after a device reset, reacquire
//      corrected distance, skip already-passed geofences") is NOT
//      implemented. A reset today restarts GeoFenceManager from index 0,
//      which means a genuine mid-race reset would currently re-arm
//      already-passed checkpoints rather than skipping them. This needs
//      RouteMatcher (to reacquire a trustworthy corrected distance after
//      a reset) plus a persisted "race in progress" marker, both pending.
//
// Everything else in this class (closest-approach point-geofence
// detection and latching, the normal start/logging/stop/resume/finish
// state machine, button-driven restart) is fully implemented against the
// spec as written.
//
// Key4 semantics (owner revision, supersedes the original spec's single
// "1s bypass/ack" action - see ButtonManager.h): a quick tap is a Give
// Way ack pulse; a 1.5s hold is a manual log start/stop toggle that goes
// straight to LogManager and deliberately does NOT run the geofence
// crossing pipeline (no marking a point passed, no distance snap, no
// SMS/LoRa dispatch) - it only starts or stops the SD log, nothing else.
#pragma once

#include <cstdint>
#include <cstddef>
#include "GpsManager.h"
#include "GeoFenceManager.h"
#include "LogManager.h"
#include "DisplayManager.h"
#include "ButtonManager.h"
#include "BatteryManager.h"
#include "ConfigManager.h"

enum class RaceStage : uint8_t { WAIT_START, ACTIVE, STOPPED, FINISHED };

class AppController {
public:
    void begin(GpsManager& gps, GeoFenceManager& geo, LogManager& log,
               DisplayManager& display, ButtonManager& buttons, BatteryManager& battery,
               ConfigManager& config);

    // Call every main loop iteration once boot has reached normal
    // operation (i.e. after SD/config/route/geofence init has succeeded).
    void loop();

    RaceStage stage() const { return _stage; }

private:
    GpsManager* _gps = nullptr;
    GeoFenceManager* _geo = nullptr;
    LogManager* _log = nullptr;
    DisplayManager* _display = nullptr;
    ButtonManager* _buttons = nullptr;
    BatteryManager* _battery = nullptr;
    ConfigManager* _config = nullptr;

    RaceStage _stage = RaceStage::WAIT_START;

    // See "Honest scope note" above: raw traveled distance, not yet
    // route-corrected.
    double _rawTraveledDistanceM = 0;
    bool _hasPrevFix = false;
    double _prevLat = 0, _prevLon = 0;

    // Closest-approach tracking for the CURRENT target geofence only;
    // reset whenever GeoFenceManager::nextIndex() changes.
    size_t _trackedTargetIndex = static_cast<size_t>(-1);
    bool _hasPrevSample = false;
    float _prevDist = 0;
    bool _prevWasShrinking = false;
    uint8_t _prevHh = 0, _prevMm = 0, _prevSs = 0, _prevCs = 0;

    // OLED Field 7/8 gating, updated every tick by updateGeofenceCrossing().
    bool _geofenceLabelValid = false;
    bool _geofenceDistValid = false;
    float _geofenceDistanceM = 0;

    // Closest-approach sample's date, carried alongside the time so the
    // UTC->local conversion can roll the date correctly near midnight.
    uint16_t _prevYear = 0;
    uint8_t _prevMonth = 0, _prevDay = 0;

    // OLED Field 2: last captured geofence crossing time (already
    // converted to local time via the configured UTC offset).
    bool _lastCrossingValid = false;
    uint8_t _lastCrossHh = 0, _lastCrossMm = 0, _lastCrossSs = 0, _lastCrossCs = 0;

    // One-tick signal from updateGeofenceCrossing() to updateRaceStage().
    bool _crossedThisTick = false;
    size_t _crossedIndexThisTick = 0;

    void updateTraveledDistance();
    void updateGeofenceCrossing();
    void acceptCrossing(size_t index);
    void updateRaceStage();
    void dispatchCheckpointEvent(const GeoFencePoint& point);
    void handleButtonEvent(ButtonEvent evt);
    void updateDisplayModel();

    // Opens a race log named with the current GNSS time converted to
    // local time via the configured UTC offset.
    void openLogWithLocalTime();
};
