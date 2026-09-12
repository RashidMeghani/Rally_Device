// Race-runtime state machine + geofence-crossing orchestration (spec
// sections 8, 10, 11). Owns the single-writer fields of what the spec
// calls "AppState": race stage, corrected/traveled distance, current
// geofence target, and the last captured crossing time.
//
// -----------------------------------------------------------------------
// Scope note (see ARCHITECTURE.md section 5 for the full writeup)
// -----------------------------------------------------------------------
// Corrected distance IS now route-corrected: RouteMatcher projects the
// live position onto the ReferenceMap every
// AppConst::ROUTE_CORRECTION_INTERVAL_MS (2 min), gated on GNSS accuracy
// and the lateral match threshold, and each geofence crossing snaps to
// that point's known distanceFromStartM. Between corrections the value
// advances by GPS movement, accumulated only while the race is actively
// being logged (LogManager::isActivelyWriting).
//
// STILL NOT IMPLEMENTED - flagged rather than silently faked:
//   Reset-recovery (section 8/10: "after a device reset, reacquire
//   corrected distance, skip already-passed geofences"). RouteMatcher now
//   supplies the reacquired distance (a hintless match triggers a full
//   route scan), so the remaining piece is deciding, on the first match
//   after boot, whether to treat it as a mid-race resume and call
//   GeoFenceManager::skipPassedBefore(). Deliberately left out of this
//   change so route correction can be validated in the field on its own
//   before anything is allowed to skip checkpoints.
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
#include "route/RouteMatcher.h"

enum class RaceStage : uint8_t { WAIT_START, ACTIVE, STOPPED, FINISHED };

class AppController {
public:
    void begin(GpsManager& gps, GeoFenceManager& geo, LogManager& log,
               DisplayManager& display, ButtonManager& buttons, BatteryManager& battery,
               ConfigManager& config, RouteMatcher& route);

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
    RouteMatcher* _route = nullptr;

    RaceStage _stage = RaceStage::WAIT_START;

    // Two distances are kept deliberately (spec section 7 asks for both
    // for diagnostics):
    //   _rawTraveledDistanceM - pure GPS point-to-point accumulation,
    //       never overwritten, so drift against the route stays visible.
    //   _correctedDistanceM   - what the DATA page shows: snapped to the
    //       route by RouteMatcher every ROUTE_CORRECTION_INTERVAL_MS and
    //       to a checkpoint's known distance on each crossing, advancing
    //       by GPS movement in between.
    double _rawTraveledDistanceM = 0;
    double _correctedDistanceM = 0;
    bool _haveRouteMatch = false;
    uint32_t _lastCorrectionMs = 0;
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
    void updateRouteCorrection();
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
