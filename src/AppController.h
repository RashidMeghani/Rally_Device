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
// Reset-recovery (section 8/10) is implemented: see maybeRecoverAfterReset().
// On the first route match after boot the device marks every point more than
// RESET_RECOVERY_MARGIN_M behind it as passed, moves the race stage to
// ACTIVE and resumes logging. The margin is what keeps a device powered up
// beside the start line from skipping the start itself.
//
// Everything else in this class (point-geofence crossing detection and
// latching, the normal start/logging/stop/resume/finish state machine,
// button-driven restart) is fully implemented against the spec as written.
//
// Crossing detection (see updateGeofenceCrossing) works on the vehicle's
// ALONG-TRACK offset from a point - how far past it the vehicle is, measured
// along the direction the recon lap was driven there (GeoFencePoint::
// bearingDeg, resolved at boot). That offset is a straight line through zero
// with slope equal to road speed, so the crossing instant is sharp and can
// be interpolated between the two fixes either side of it. The vehicle is
// never sampled exactly on the line; the time reported is computed, not
// sampled, which is what makes it more precise than the GNSS fix interval.
//
// Three consequences worth knowing:
//   - Direction is free. Offset going negative->positive is a forward
//     crossing; positive->negative is the vehicle coming back through the
//     point the wrong way, which is detected and NOT timed.
//   - A standing start works. A car parked short of the line has a negative
//     offset and simply sits there; the crossing is the moment it drives
//     across, not the moment it was closest (which is while parked).
//   - No speed gate is needed beyond "is it moving at all". The old 10 km/h
//     checkpoint gate existed to suppress noise that this method does not
//     produce, and would now reject a legitimate slow crossing.
//
// Points are tracked by PROXIMITY, not by race order: whichever point is
// nearest within GEOFENCE_LABEL_SHOW_M is the one being watched, passed or
// not. So driving back over a checkpoint runs the whole process again -
// label, distance, captured time, distance snap, checkpoint dispatch. Only
// crossing the CURRENT target (GeoFenceManager::nextIndex) additionally
// advances the race sequence and touches the log file; a repeat crossing
// never opens or closes a log (owner's rule).
//
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
#include "LoRaTransport.h"
#include "OvertakeManager.h"
#include "util/TimeUtil.h"

enum class RaceStage : uint8_t { WAIT_START, ACTIVE, STOPPED, FINISHED };

class AppController {
public:
    void begin(GpsManager& gps, GeoFenceManager& geo, LogManager& log,
               DisplayManager& display, ButtonManager& buttons, BatteryManager& battery,
               ConfigManager& config, RouteMatcher& route, LoRaTransport& lora,
               OvertakeManager& overtake);

    // Call every main loop iteration once boot has reached normal
    // operation (i.e. after SD/config/route/geofence init has succeeded).
    void loop();

    RaceStage stage() const { return _stage; }

    // Entry point for packets arriving over LoRa. Called from
    // LoRaTransport::loop() on the main thread.
    void onLoRaMessage(const LoRaMessage& msg);

private:
    GpsManager* _gps = nullptr;
    GeoFenceManager* _geo = nullptr;
    LogManager* _log = nullptr;
    DisplayManager* _display = nullptr;
    ButtonManager* _buttons = nullptr;
    BatteryManager* _battery = nullptr;
    ConfigManager* _config = nullptr;
    RouteMatcher* _route = nullptr;
    LoRaTransport* _lora = nullptr;
    OvertakeManager* _overtake = nullptr;

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
    // Reset-recovery runs exactly once, on the first route match after boot.
    bool _resetRecoveryDone = false;
    bool _hasPrevFix = false;
    double _prevLat = 0, _prevLon = 0;

    // Crossing tracking for whichever point is nearest inside
    // GEOFENCE_LABEL_SHOW_M - passed or not, so a point driven over twice is
    // detected twice. Reset whenever the nearest point changes.
    static constexpr size_t NO_POINT = static_cast<size_t>(-1);
    size_t _trackedIndex = NO_POINT;
    // Guards against advancing the sample-to-sample comparison on a tick
    // where the receiver has not delivered a new position - comparing a fix
    // with itself is what previously prevented crossings from latching.
    uint32_t _lastGeofenceFixSeq = 0;
    // One "approaching X" serial line per target, so the approach is
    // visible during testing without spamming every tick.
    bool _announcedApproach = false;

    // Previous NEW fix, for the sign-change test.
    bool _hasPrevSample = false;
    float _prevAlongM = 0;
    float _prevLateralM = 0;
    float _prevSpeedKmh = 0;
    GnssInstant _prevInstant;

    // True once the vehicle has been stationary during this approach and has
    // not yet crossed - i.e. this is a standing start, and the lower speed
    // gate applies.
    //
    // It must stay set for the WHOLE approach, not just the first moving
    // fix. A car starting 2 m short of the line takes several fixes to reach
    // it, and an earlier one-fix version of this flag classified the
    // eventual crossing as an ordinary checkpoint and rejected a genuine
    // standing start at 9.6 km/h against the 10 km/h gate.
    bool _approachFromStandstill = false;

    // Where the vehicle sat while stationary, smoothed.
    //
    // A standing start crosses the line on its FIRST moving fix: parked 2 m
    // short at 1 Hz, the next fix is already past the point, so without a
    // stationary predecessor to pair with there is no sign change to see and
    // the crossing is missed outright. The parked position has to serve as
    // that predecessor.
    //
    // It is smoothed because a single parked fix is one noisy sample: 2 m
    // from the line with ~1.5 m of jitter, roughly one parked fix in eleven
    // reads as already being on the far side, which would lose the crossing.
    // Averaging while stationary is valid precisely because the vehicle is
    // not moving, and it drives that failure rate to nothing.
    bool _hasParkedAnchor = false;
    float _parkedAlongM = 0;
    float _parkedLateralM = 0;
    GnssInstant _parkedInstant;

    // A detected sign change, held until the vehicle has followed through by
    // GEOFENCE_CROSS_CONFIRM_M. The time is already computed at this point,
    // so the wait costs nothing in accuracy.
    bool _pendingCrossing = false;
    bool _pendingForward = false;
    GnssInstant _pendingInstant;
    // Speed interpolated to the crossing instant, and which gate it must
    // clear. Judging the speed AT THE LINE rather than at some nearby fix
    // matters most exactly where the gate is tightest: a standing start is
    // accelerating hard, so a reading taken a fix later flatters it and one
    // taken a fix earlier condemns it.
    float _pendingSpeedKmh = 0;
    bool _pendingFromStandstill = false;

    // OLED Field 7/8 gating, updated every tick by updateGeofenceCrossing().
    // The label is copied here rather than looked up at draw time, because
    // the point on screen is whichever is nearest - which after a crossing
    // is the point just PASSED, one the race sequence has moved beyond.
    // Tracking by proximity is also what keeps a crossed point on screen
    // while the vehicle drives away from it, with no separate bookkeeping.
    bool _geofenceLabelValid = false;
    bool _geofenceDistValid = false;
    float _geofenceDistanceM = 0;
    char _geofenceLabel[16] = {0};

    // OLED Field 2: last captured geofence crossing time (already converted
    // to local time via the configured UTC offset).
    //
    // _crossingIndex is the point that time belongs to. The time is shown
    // only while that point is still the tracked one, so what is on screen
    // always names a checkpoint the driver can still see labelled, and the
    // two disappear together.
    bool _lastCrossingValid = false;
    // The same instant in UTC, kept because that is what goes on the air:
    // local time is a presentation choice at each end, and the checkpoint
    // station has no reason to share this device's offset.
    GnssInstant _lastCrossUtc;
    size_t _crossingIndex = NO_POINT;
    bool _crossingTimeVisible = false;
    uint8_t _lastCrossHh = 0, _lastCrossMm = 0, _lastCrossSs = 0, _lastCrossCs = 0;

    // One-tick signal from updateGeofenceCrossing() to updateRaceStage().
    bool _crossedThisTick = false;
    size_t _crossedIndexThisTick = 0;

    void updateTraveledDistance();
    void updateRouteCorrection();
    // Spec section 8/10: after a reset the device must work out where on the
    // route it is, treat the points behind it as passed, and pick the race
    // back up - rather than sitting in WAIT_START waiting for a start line
    // that is already kilometres behind.
    void maybeRecoverAfterReset();
    void updateGeofenceCrossing();
    void resetApproachTracking();
    // Nearest point within GEOFENCE_LABEL_SHOW_M, or NO_POINT.
    size_t nearestPointWithinWindow(double lat, double lon, float& outDistM) const;
    // Signed along-track offset from a point (negative = short of it,
    // positive = past it) and the across-track offset, both in metres.
    static void offsetsFromPoint(const GeoFencePoint& pt, double lat, double lon,
                                 float& alongM, float& lateralM);
    // Applies the speed-at-the-line rule to the pending candidate, logging
    // the reason when it fails. Standing starts get their own lower gate.
    bool pendingCrossingPassesGate(const char* label);
    void acceptCrossing(size_t index, const GnssInstant& whenUtc);
    void updateRaceStage();
    void dispatchCheckpointEvent(size_t index, const GeoFencePoint& point);

    // A crossing is announced over LoRa until the checkpoint station at
    // that point acknowledges it. Three things end the attempt, and all
    // three matter:
    //
    //   - the station answers (the intended case);
    //   - the vehicle leaves GEOFENCE_LABEL_SHOW_M of the point, because
    //     past that there is no station left to hear it and the crossing
    //     will have to be reconciled from the SMS and the race log;
    //   - LORA_EVENT_MAX_ATTEMPTS, which only catches a vehicle stopped
    //     inside the radius with no station answering - without it that
    //     device would transmit for the rest of the race.
    //
    // The event id is what an acknowledgement is matched on. Every retry
    // gets a fresh transport sequence number, so matching on the sequence
    // would only recognise the acknowledgement of one particular retry.
    void updateCheckpointBroadcast();

    bool _cpPending = false;
    size_t _cpPointIndex = 0;
    uint16_t _cpEventId = 0;
    uint8_t _cpAttempts = 0;
    uint32_t _cpNextSendMs = 0;
    int32_t _cpDistanceCm = 0;
    GnssInstant _cpWhenUtc;
    char _cpLabel[LORA_MAX_PAYLOAD + 1] = {0};
    void handleButtonEvent(ButtonEvent evt);
    void updateDisplayModel();

    void openLogWithLocalTime();
};
