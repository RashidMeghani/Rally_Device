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
// Geofence detection note: the closest-approach comparison advances only
// on a NEW GNSS fix (GpsManager::fixSequence), never per loop tick. This
// matters because loop() runs thousands of times a second against a
// receiver that commits a position 1-10 times a second - comparing a fix
// with itself is what previously prevented any crossing from latching.
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
    // Forces the next tick to correct regardless of how recently the last
    // correction ran. The periodic interval is a ceiling on how long the
    // device may go WITHOUT correcting - it must never delay a correction
    // that some event has made due.
    bool _correctionRequested = false;
    bool _hasPrevFix = false;
    double _prevLat = 0, _prevLon = 0;

    // Closest-approach tracking for the CURRENT target geofence only;
    // reset whenever GeoFenceManager::nextIndex() changes.
    //
    // The detector keeps the single CLOSEST sample seen inside the precise
    // zone and commits it once the distance has been growing again for
    // GEOFENCE_DEPART_CONFIRM_SAMPLES consecutive NEW fixes (or, as a
    // fallback, when the vehicle leaves the zone with a minimum recorded
    // but unconfirmed). Keeping the minimum rather than only the previous
    // sample is what makes the crossing time correct: the crossing happened
    // at that sample, not at whichever later sample happened to trip the
    // test.
    size_t _trackedTargetIndex = static_cast<size_t>(-1);
    // Guards against advancing the sample-to-sample comparison on a tick
    // where the receiver has not delivered a new position - comparing a fix
    // with itself is what previously prevented crossings from latching.
    uint32_t _lastGeofenceFixSeq = 0;
    bool _hasMinSample = false;
    float _minDist = 0;
    float _minSpeedKmh = 0;
    uint8_t _departingSamples = 0;
    // Set when the speed gate has already rejected this target's confirmed
    // closest approach, so the detector does not re-arm and re-reject every
    // few fixes while the vehicle sits beside the point.
    bool _gateRejected = false;
    // One "approaching X" serial line per target, so the approach is
    // visible during testing without spamming every tick.
    bool _announcedApproach = false;
    uint8_t _minHh = 0, _minMm = 0, _minSs = 0, _minCs = 0;

    // OLED Field 7/8 gating, updated every tick by updateGeofenceCrossing().
    // The label is copied here rather than looked up from nextIndex() at
    // draw time, because the point on screen may be the one just PASSED -
    // which nextIndex has already moved beyond.
    bool _geofenceLabelValid = false;
    bool _geofenceDistValid = false;
    float _geofenceDistanceM = 0;
    char _geofenceLabel[16] = {0};

    // The most recently crossed point stays on the display while the
    // vehicle drives away from it, for the same window it was shown in on
    // the way in. Cleared once it falls outside GEOFENCE_LABEL_SHOW_M,
    // which also clears the captured crossing time.
    bool _passedPointValid = false;
    size_t _passedIndex = 0;

    // Closest-approach sample's date, carried alongside the time so the
    // UTC->local conversion can roll the date correctly near midnight.
    uint16_t _minYear = 0;
    uint8_t _minMonth = 0, _minDay = 0;

    // OLED Field 2: last captured geofence crossing time (already
    // converted to local time via the configured UTC offset).
    // _lastCrossingValid is the latch ("a time has been captured and not
    // yet aged out"); _crossingTimeVisible is the per-tick answer to
    // "should it be on screen right now", which additionally requires the
    // point it belongs to still to be inside its label window and a live
    // fix to measure that. The time therefore always names a checkpoint
    // the driver can still see labelled, and disappears with that label.
    bool _crossingTimeVisible = false;
    bool _lastCrossingValid = false;
    uint8_t _lastCrossHh = 0, _lastCrossMm = 0, _lastCrossSs = 0, _lastCrossCs = 0;

    // One-tick signal from updateGeofenceCrossing() to updateRaceStage().
    bool _crossedThisTick = false;
    size_t _crossedIndexThisTick = 0;

    void updateTraveledDistance();
    void updateRouteCorrection();
    void updateGeofenceCrossing();
    void resetApproachTracking();
    bool tryCommitCrossing(size_t index); // true if the crossing was latched
    void acceptCrossing(size_t index);
    void updateRaceStage();
    void dispatchCheckpointEvent(const GeoFencePoint& point);
    void handleButtonEvent(ButtonEvent evt);
    void updateDisplayModel();

    // Opens a race log named with the current GNSS time converted to
    // local time via the configured UTC offset.
    void openLogWithLocalTime();
};
