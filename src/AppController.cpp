#include "AppController.h"
#include "../include/AppConstants.h"
#include "util/NmeaUtil.h"
#include "util/TimeUtil.h"
#include <Arduino.h>
#include <cstring>
#include <cmath>
#include "../include/DebugLog.h"

void AppController::begin(GpsManager& gps, GeoFenceManager& geo, LogManager& log,
                           DisplayManager& display, ButtonManager& buttons, BatteryManager& battery,
                           ConfigManager& config, RouteMatcher& route, LoRaTransport& lora) {
    _gps = &gps; _geo = &geo; _log = &log;
    _display = &display; _buttons = &buttons; _battery = &battery;
    _config = &config; _route = &route; _lora = &lora;
}

void AppController::openLogWithLocalTime() {
    bool timeValid = _gps->timeValid();
    LocalDateTime local = TimeUtil::applyUtcOffset(
        _gps->year(), _gps->month(), _gps->day(),
        _gps->hour(), _gps->minute(), _gps->second(),
        _config->get().utcOffsetMinutes);
    _log->startNewLog(timeValid, local.year, local.month, local.day,
                      local.hour, local.minute, local.second);

    // Every route into logging comes through here - START crossing, resume
    // after the 20-minute stop, manual Key4, and reset recovery when that
    // lands - so this single hook makes a fresh log always begin from a
    // corrected distance rather than inheriting a stale one and waiting up
    // to a full interval to fix it.
    _correctionRequested = true;
}

void AppController::updateTraveledDistance() {
    if (!_gps->hasFix()) {
        // Drop the previous position so distance accumulation restarts
        // from the reacquired fix. Bridging the gap instead would add a
        // straight-line jump computed against a possibly-wild first fix -
        // and the next checkpoint crossing re-snaps distance anyway.
        _hasPrevFix = false;
        return;
    }
    double lat = _gps->latitude(), lon = _gps->longitude();

    // Distance only accumulates while the race is actually being logged.
    // The previous position is tracked either way, so that resuming after
    // a pause measures from where the vehicle is now rather than bridging
    // the whole un-logged gap in one step.
    if (_hasPrevFix && _log->isActivelyWriting()) {
        const double step = NmeaUtil::haversineMeters(_prevLat, _prevLon, lat, lon);
        _rawTraveledDistanceM += step;
        _correctedDistanceM += step; // re-snapped by the matcher / crossings
    }
    _prevLat = lat; _prevLon = lon; _hasPrevFix = true;
}

void AppController::updateRouteCorrection() {
    if (!_route->isReady() || !_gps->hasFix()) return;

    // A reacquisition scan already in flight is stepped every tick and is
    // self-limiting (one segment per step), so it bypasses the interval.
    if (_route->fullScanActive()) {
        RouteMatch scan;
        if (!_route->stepFullScan(scan)) return; // still working through the route

        if (!scan.valid) {
            LOGF("[Route] Reacquisition failed - nearest route point %.0fm away\n",
                          scan.lateralErrorM);
            return;
        }
        _correctedDistanceM = scan.correctedDistanceM;
        _haveRouteMatch = true;
        _lastCorrectionMs = millis();
        // This is the reacquisition after a reset - the moment the device
        // first knows where it is - so it is where recovery belongs.
        maybeRecoverAfterReset();
        // The scan searched against the position captured when it STARTED,
        // and the vehicle has moved since. Now that there is a hint, ask
        // for an immediate cheap hinted match to refine against where the
        // vehicle actually is now.
        _correctionRequested = true;
        LOGF("[Route] Reacquired at %.0fm (lateral %.1fm, seg %u) - refining\n",
                      _correctedDistanceM, scan.lateralErrorM, (unsigned)scan.segmentIndex);
        return;
    }

    // Until the device has ANY match it is reacquiring - after a reset it
    // must establish where it is promptly, not sit for a full correction
    // interval first. Retries stay rate-limited because starting a scan
    // walks the whole route.
    const uint32_t interval = _haveRouteMatch ? AppConst::ROUTE_CORRECTION_INTERVAL_MS
                                              : AppConst::ROUTE_REACQUIRE_RETRY_MS;
    // A requested correction jumps the queue: the interval caps how long
    // the device may go without correcting, it never postpones one that an
    // event (logging starting, reacquisition) has already made due.
    if (!_correctionRequested && millis() - _lastCorrectionMs < interval) return;
    _correctionRequested = false;
    _lastCorrectionMs = millis();

    // Accuracy gate (spec section 7): a correction is only as trustworthy
    // as the fix behind it, and snapping to the route on a poor fix would
    // inject error rather than remove it.
    const float threshold = _config->get().gnssAccuracyThresholdM;
    if (!_gps->accuracyValid() || _gps->accuracyMeters() > threshold) {
        LOGF("[Route] Skipped - accuracy %.1fm worse than %.1fm limit\n",
                      _gps->accuracyValid() ? _gps->accuracyMeters() : -1.0f, threshold);
        return;
    }

    // Nothing to hint with means reacquisition: kick off the incremental
    // whole-route scan and let subsequent ticks work through it.
    if (!_haveRouteMatch) {
        LOGLN("[Route] No reference position - starting full-route reacquisition scan");
        _route->startFullScan(_gps->latitude(), _gps->longitude());
        return;
    }

    const RouteMatch m = _route->match(_gps->latitude(), _gps->longitude(),
                                        (float)_correctedDistanceM);

    if (!m.valid) {
        LOGF("[Route] No match%s - nearest route point %.0fm away, beyond the %.0fm threshold\n",
                      m.fullScan ? " (full scan)" : "", m.lateralErrorM,
                      AppConst::ROUTE_MATCH_THRESHOLD_M);
        return;
    }

    // Implausible-jump guard: even a geometrically valid match can be the
    // wrong place on a route that doubles back near itself. Nothing can
    // have moved further than the elapsed time allows.
    if (_haveRouteMatch) {
        const float jump = fabsf((float)m.correctedDistanceM - (float)_correctedDistanceM);
        const float maxJump = (AppConst::ROUTE_CORRECTION_INTERVAL_MS / 1000.0f) *
                              (AppConst::ROUTE_MAX_PLAUSIBLE_KMH / 3.6f);
        if (jump > maxJump) {
            LOGF("[Route] REJECTED - %.0fm jump exceeds the %.0fm possible in this interval\n",
                          jump, maxJump);
            return;
        }
    }

    const double before = _correctedDistanceM;
    _correctedDistanceM = m.correctedDistanceM;
    _haveRouteMatch = true;
    // Belt and braces: normally the full scan above gets there first, but a
    // hinted match is the only path if a position was ever established
    // without one. Self-guarded, so a second call does nothing.
    maybeRecoverAfterReset();

    LOGF("[Route] Corrected %.0fm -> %.0fm (delta %+.0fm, lateral %.1fm, seg %u%s) | raw %.0fm\n",
                  before, _correctedDistanceM, _correctedDistanceM - before,
                  m.lateralErrorM, (unsigned)m.segmentIndex,
                  m.fullScan ? ", full scan" : "", _rawTraveledDistanceM);
}

// Spec section 8/10. Called once, the first time the device establishes
// where it is on the route after boot.
//
// A reset wipes RAM, so the race restarts from nextIndex 0 and stage
// WAIT_START - the device sits waiting for a start line that may be
// kilometres behind it. A field test confirmed both halves of the damage:
// logging did not resume, and when the vehicle later reached the FINISH the
// crossing was classed as a repeat (because the target was still the start)
// so the log was never closed.
void AppController::maybeRecoverAfterReset() {
    if (_resetRecoveryDone) return;
    _resetRecoveryDone = true;
    if (_geo->count() == 0) return;

    // Only points comfortably behind us count as passed - see
    // RESET_RECOVERY_MARGIN_M for why the margin is not optional.
    const float skipBefore = (float)_correctedDistanceM - AppConst::RESET_RECOVERY_MARGIN_M;
    _geo->skipPassedBefore(skipBefore);
    const size_t next = _geo->nextIndex();

    if (next == 0) {
        LOGF("[Recovery] Reacquired at %.0fm - not past %s yet, normal start applies\n",
             (double)_correctedDistanceM, _geo->at(0).label);
        return;
    }

    if (next >= _geo->count()) {
        // Everything is behind us: the run was already over when power was
        // lost. Nothing to resume, and no log should be opened.
        _stage = RaceStage::FINISHED;
        LOGF("[Recovery] Reacquired at %.0fm - past the final point, nothing to resume\n",
             (double)_correctedDistanceM);
        return;
    }

    // Mid-race. The stage must advance out of WAIT_START: the finish is only
    // acted on from a running race, so leaving the stage at WAIT_START is
    // precisely what stopped the log closing at the final point.
    _stage = RaceStage::ACTIVE;
    if (!_log->isLogging() && !_log->isFinished()) {
        openLogWithLocalTime();
        LOGLN("[Recovery] Logging resumed in a new file");
    }
    LOGF("[Recovery] Reacquired at %.0fm mid-race - %u point(s) behind marked passed, "
         "next target %s, stage ACTIVE\n",
         (double)_correctedDistanceM, (unsigned)next, _geo->at(next).label);
}

void AppController::resetApproachTracking() {
    _hasPrevSample = false;
    _prevAlongM = 0;
    _approachFromStandstill = false;
    _hasParkedAnchor = false;
    _pendingCrossing = false;
    _announcedApproach = false;
}

size_t AppController::nearestPointWithinWindow(double lat, double lon, float& outDistM) const {
    size_t best = NO_POINT;
    float bestDist = AppConst::GEOFENCE_LABEL_SHOW_M;
    // Every point is a candidate, passed or not: driving back over a
    // checkpoint must run the whole process again (owner's rule). The list
    // is tens of entries and this runs once per GNSS fix, not per tick.
    for (size_t i = 0; i < _geo->count(); ++i) {
        const GeoFencePoint& pt = _geo->at(i);
        const float d = (float)NmeaUtil::haversineMeters(lat, lon, pt.lat, pt.lon);
        if (d < bestDist) { bestDist = d; best = i; }
    }
    outDistM = bestDist;
    return best;
}

// Splits the vehicle's offset from a point into two components, in a local
// east/north frame centred on the point:
//
//   alongM   - how far PAST the point, measured along the direction the
//              recon lap was driven here. Negative before it, positive
//              after. This is the quantity whose zero-crossing is the
//              actual crossing event.
//   lateralM - how far to the side. Only used to bound the crossing to a
//              corridor around the point, so that the (infinite) line the
//              along-track test describes cannot be tripped by a vehicle on
//              a parallel road.
void AppController::offsetsFromPoint(const GeoFencePoint& pt, double lat, double lon,
                                      float& alongM, float& lateralM) {
    const double mPerDegLat = 111320.0;
    const double mPerDegLon = 111320.0 * cos(pt.lat * M_PI / 180.0);

    const double east  = (lon - pt.lon) * mPerDegLon;
    const double north = (lat - pt.lat) * mPerDegLat;

    const double theta = pt.bearingDeg * M_PI / 180.0;
    const double tEast = sin(theta), tNorth = cos(theta); // unit vector along travel

    alongM   = (float)(east * tEast  + north * tNorth);
    lateralM = (float)(east * tNorth - north * tEast);
}

void AppController::updateGeofenceCrossing() {
    _geofenceLabelValid = false;
    _geofenceDistValid = false;
    _crossingTimeVisible = false;

    // No fix means no distance to any point, so every geofence field blanks
    // - consistent with the rule that live fields never show stale values.
    if (!_gps->hasFix() || _geo->count() == 0) return;

    const double lat = _gps->latitude(), lon = _gps->longitude();

    float dist = 0;
    const size_t idx = nearestPointWithinWindow(lat, lon, dist);
    if (idx != _trackedIndex) {
        _trackedIndex = idx;
        resetApproachTracking();
    }
    if (idx == NO_POINT) return;

    const GeoFencePoint& target = _geo->at(idx);

    // OLED gating is a readout of the live distance, so it updates every
    // tick regardless of whether the fix underneath is new.
    _geofenceLabelValid = true;
    strncpy(_geofenceLabel, target.label, sizeof(_geofenceLabel) - 1);
    _geofenceLabel[sizeof(_geofenceLabel) - 1] = '\0';
    if (dist <= AppConst::GEOFENCE_PRECISE_ZONE_M) {
        _geofenceDistValid = true;
        _geofenceDistanceM = dist;
    }
    // The captured time belongs to a specific point, and is shown only
    // while that point is the one on screen.
    _crossingTimeVisible = _lastCrossingValid && (_crossingIndex == idx);

    // ---------------------------------------------------------------------
    // Everything below compares this fix with the previous one and must
    // therefore advance only on a genuinely NEW position from the receiver.
    // loop() runs thousands of times a second against a receiver committing
    // a position a handful of times a second; comparing a fix with itself is
    // what once made crossings undetectable entirely.
    // ---------------------------------------------------------------------
    const uint32_t seq = _gps->fixSequence();
    if (seq == _lastGeofenceFixSeq) return;
    _lastGeofenceFixSeq = seq;

    if (dist > AppConst::GEOFENCE_PRECISE_ZONE_M) {
        // Outside the working zone: still on the display, but not worth
        // running the crossing test for.
        _hasPrevSample = false;
        _pendingCrossing = false;
        return;
    }

    if (!target.hasBearing) {
        // No ReferenceMap heading for this point, so there is no "forward"
        // to measure against and no crossing can be timed. Said once per
        // approach rather than per fix.
        if (!_announcedApproach) {
            _announcedApproach = true;
            LOGF("[Geofence] %s has no heading from the ReferenceMap - "
                          "cannot time a crossing here\n", target.label);
        }
        return;
    }

    if (!_announcedApproach) {
        _announcedApproach = true;
        LOGF("[Geofence] Approaching %s - %.0fm, watching for the crossing\n",
                      target.label, dist);
    }

    float alongM = 0, lateralM = 0;
    offsetsFromPoint(target, lat, lon, alongM, lateralM);

    const float speed = _gps->speedValid() ? _gps->speedKmh() : 0.0f;
    const bool moving = speed > AppConst::GEOFENCE_MOVING_MIN_KMH;

    GnssInstant now;
    now.year = _gps->year(); now.month = _gps->month(); now.day = _gps->day();
    now.hour = _gps->hour(); now.minute = _gps->minute();
    now.second = _gps->second(); now.centisecond = _gps->centisecond();

    if (!moving) {
        // A candidate detected just before stopping is real - it came from
        // two moving fixes - so commit it now rather than waiting for a
        // confirmation distance the vehicle may never travel. Losing the
        // crossing of a checkpoint the car stops at would be the worse
        // failure.
        if (_pendingCrossing) {
            const bool stillBeyond = _pendingForward ? (alongM > 0) : (alongM < 0);
            if (stillBeyond && _pendingForward) {
                if (pendingCrossingPassesGate(target.label)) {
                    LOGF("[Geofence] %s confirmed by stopping past it\n", target.label);
                    acceptCrossing(idx, _pendingInstant);
                }
            } else if (stillBeyond) {
                LOGF("[Geofence] %s crossed in REVERSE - not timed\n", target.label);
            }
            _pendingCrossing = false;
        }

        // Keep a smoothed record of where it is parked, to serve as the
        // predecessor for the first moving fix (see AppController.h). No
        // sign-change test runs while stopped, so a stationary vehicle's own
        // noise can never produce a crossing.
        if (!_hasParkedAnchor) {
            _hasParkedAnchor = true;
            _parkedAlongM = alongM;
            _parkedLateralM = lateralM;
        } else {
            _parkedAlongM   = 0.9f * _parkedAlongM   + 0.1f * alongM;
            _parkedLateralM = 0.9f * _parkedLateralM + 0.1f * lateralM;
        }
        _parkedInstant = now;
        _hasPrevSample = false;
        return;
    }

    // Moving again: if the vehicle has just pulled away from a stop, the
    // smoothed parked position is the sample the sign-change test needs to
    // compare against. This is what makes a standing start work at all.
    if (!_hasPrevSample && _hasParkedAnchor) {
        _prevAlongM = _parkedAlongM;
        _prevLateralM = _parkedLateralM;
        _prevInstant = _parkedInstant;
        _prevSpeedKmh = 0.0f;                 // it was stationary, by definition
        _approachFromStandstill = true;       // sticky until this approach resolves
        _hasPrevSample = true;
    }
    _hasParkedAnchor = false;

    // --- follow-through on a crossing detected earlier -------------------
    if (_pendingCrossing) {
        // Hysteresis: confirm once the vehicle is CONFIRM_M beyond the line,
        // abandon only once it is DISCARD_M back on the near side. Testing
        // against zero in both directions would let noise cancel a real
        // crossing at low speed, where one fix moves the vehicle less than
        // the jitter moves the measurement.
        const float forwardOffset = _pendingForward ? alongM : -alongM;
        if (forwardOffset <= -AppConst::GEOFENCE_CROSS_DISCARD_M) {
            LOGF("[Geofence] %s candidate discarded - vehicle came back %.1fm "
                          "across before confirming\n", target.label, -forwardOffset);
            _pendingCrossing = false;
        } else if (forwardOffset >= AppConst::GEOFENCE_CROSS_CONFIRM_M) {
            if (_pendingForward) {
                if (pendingCrossingPassesGate(target.label)) {
                    acceptCrossing(idx, _pendingInstant);
                }
            } else {
                LOGF("[Geofence] %s crossed in REVERSE - not timed\n", target.label);
            }
            _pendingCrossing = false;
            // This approach is resolved. Another standing start at the same
            // point sets the flag again by parking there again.
            _approachFromStandstill = false;
        }
    }

    // --- sign change between this fix and the previous one ---------------
    // A newer sign change SUPERSEDES an older unconfirmed candidate rather
    // than being blocked by it. At low speed a fix moves the vehicle less
    // than noise moves the measurement, so the offset can wobble across
    // zero several times around the real crossing; if the first wobble
    // (which may even be a spurious REVERSE) held the detector until it
    // cleared, the genuine crossing would already be behind us by then and
    // would be missed outright. Whichever flip is still standing when the
    // vehicle reaches the confirmation distance is the one that counts.
    if (_hasPrevSample) {
        const bool forward = (_prevAlongM < 0.0f && alongM >= 0.0f);
        const bool reverse = (_prevAlongM > 0.0f && alongM <= 0.0f);
        if (forward || reverse) {
            // The vehicle was on the line when the along-track offset was
            // zero. Between two fixes it travelled in a straight line at
            // near-constant speed, so that instant sits at this fraction of
            // the interval - and the clock is interpolated to match. This is
            // how the crossing is timed more finely than the fix interval:
            // the reported instant was never sampled, it was computed.
            const float denom = _prevAlongM - alongM;
            const float f = (denom != 0.0f) ? (_prevAlongM / denom) : 0.0f;

            // Where the crossing actually happened, to check it was on this
            // road rather than a parallel one.
            const float crossLateral = _prevLateralM + f * (lateralM - _prevLateralM);
            if (fabsf(crossLateral) > AppConst::GEOFENCE_CROSS_CORRIDOR_M) {
                LOGF("[Geofence] %s line crossed %.0fm off to the side - outside the "
                              "%.0fm corridor, ignored\n",
                              target.label, crossLateral, AppConst::GEOFENCE_CROSS_CORRIDOR_M);
            } else {
                _pendingCrossing = true;
                _pendingForward = forward;
                _pendingInstant = TimeUtil::interpolateUtc(_prevInstant, now, f);
                // Speed at the line, interpolated the same way the time is.
                // Under constant acceleration speed is linear in time, so
                // this is exact for the standing-start case rather than an
                // approximation.
                _pendingSpeedKmh = _prevSpeedKmh + f * (speed - _prevSpeedKmh);
                _pendingFromStandstill = _approachFromStandstill;
                LOGF("[Geofence] %s %s crossing at %02u:%02u:%02u.%02u UTC "
                              "(%.1fm -> %.1fm, f=%.2f, %.1f km/h) - confirming\n",
                              target.label, forward ? "forward" : "REVERSE",
                              _pendingInstant.hour, _pendingInstant.minute,
                              _pendingInstant.second, _pendingInstant.centisecond,
                              _prevAlongM, alongM, f, speed);
            }
        }
    }

    _prevAlongM = alongM;
    _prevLateralM = lateralM;
    _prevSpeedKmh = speed;
    _prevInstant = now;
    _hasPrevSample = true;
}

bool AppController::pendingCrossingPassesGate(const char* label) {
    // Which gate applies is decided by how the vehicle arrived, not by which
    // point this is. A standing start is recognisable from the crossing
    // itself - the sample before it was the parked anchor - so no point ever
    // has to be labelled as a start line, and a restart anywhere on the
    // route is handled the same way.
    const float gate = _pendingFromStandstill ? AppConst::GEOFENCE_STANDING_START_MIN_KMH
                                              : AppConst::GEOFENCE_CROSSING_MIN_KMH;
    if (_pendingSpeedKmh >= gate) return true;

    LOGF("[Geofence] %s NOT counted - %.1f km/h at the line is below the "
                  "%.0f km/h %s gate\n",
                  label, _pendingSpeedKmh, gate,
                  _pendingFromStandstill ? "standing-start" : "checkpoint");
    return false;
}

void AppController::acceptCrossing(size_t idx, const GnssInstant& whenUtc) {
    GeoFencePoint& target = _geo->at(idx);

    // The crossing instant is GNSS UTC; convert to local for display. The
    // raw NMEA in the log stays UTC either way.
    LocalDateTime local = TimeUtil::applyUtcOffset(
        whenUtc.year, whenUtc.month, whenUtc.day,
        whenUtc.hour, whenUtc.minute, whenUtc.second,
        _config->get().utcOffsetMinutes);
    _lastCrossHh = local.hour; _lastCrossMm = local.minute;
    _lastCrossSs = local.second; _lastCrossCs = whenUtc.centisecond;
    _lastCrossingValid = true;
    _lastCrossUtc = whenUtc;
    _crossingIndex = idx;

    // A crossing is the most trustworthy correction available: the point's
    // distance-from-start is surveyed, not inferred. It is never blocked by
    // the periodic interval - it applies the moment the crossing is
    // detected - and it restarts that interval, so the next periodic
    // correction is due 2 minutes from HERE rather than firing redundantly
    // seconds after this one. This applies to a repeat crossing too: the
    // vehicle is demonstrably at that point either way.
    _correctedDistanceM = target.distanceFromStartM;
    _haveRouteMatch = true;
    _lastCorrectionMs = millis();

    dispatchCheckpointEvent(idx, target);

    // Only crossing the CURRENT target advances the race sequence and is
    // allowed to open or close a log. A repeat crossing of a point already
    // behind us does everything else - time, display, distance snap,
    // checkpoint dispatch - but never touches the log file (owner's rule).
    const bool isCurrentTarget = (idx == _geo->nextIndex());
    // Crossing the FINISH ends the run whatever the device believed its
    // progress to be. Without this, anything that leaves nextIndex stale -
    // a mid-race reset, a missed checkpoint - makes the real finish read as
    // an ordinary repeat crossing, and the log is never closed. There is no
    // case where a vehicle legitimately crosses the final point, forward and
    // above the speed gate, and the race should continue.
    const bool isFinishPoint = (_geo->count() > 1 && idx + 1 == _geo->count());
    if (isCurrentTarget || (isFinishPoint && _stage != RaceStage::FINISHED)) {
        _geo->markPassed(idx);
        _crossedThisTick = true;
        _crossedIndexThisTick = idx;
    }

    LOGF("[Geofence] CROSSED %s at %02u:%02u:%02u.%02u local (index %u, %.1f km/h%s, %s) "
                  "- distance snapped to %.0fm\n",
                  target.label, _lastCrossHh, _lastCrossMm, _lastCrossSs, _lastCrossCs,
                  (unsigned)idx, _pendingSpeedKmh,
                  _pendingFromStandstill ? ", standing start" : "",
                  isCurrentTarget ? "race target" : (isFinishPoint ? "FINISH out of sequence" 
                                                 : "repeat crossing, log untouched"),
                  (double)_correctedDistanceM);
}

void AppController::dispatchCheckpointEvent(size_t index, const GeoFencePoint& point) {
    // TODO(next phase): queue SMS to the 5 configured numbers via GsmManager.

    // Announce the crossing over LoRa, and keep announcing until the
    // checkpoint station answers. Queued rather than sent inline: the
    // transport transmits asynchronously so ~125 ms of airtime never stalls
    // the race loop.
    _cpPending = true;
    _cpPointIndex = index;
    _cpEventId++;
    if (_cpEventId == 0) _cpEventId = 1;     // 0 reads as "no event pending"
    _cpAttempts = 0;
    _cpDistanceCm = (int32_t)llround(_correctedDistanceM * 100.0);
    _cpWhenUtc = _lastCrossUtc;
    strncpy(_cpLabel, point.label, LORA_MAX_PAYLOAD);
    _cpLabel[LORA_MAX_PAYLOAD] = '\0';
    _cpNextSendMs = millis();                // first copy goes out on this tick
    updateCheckpointBroadcast();
}

void AppController::updateCheckpointBroadcast() {
    if (!_cpPending) return;

    // Out of range of the station: nothing left to hear this, so stop
    // rather than transmit into empty desert for the rest of the lap.
    if (_gps->hasFix() && _cpPointIndex < _geo->count()) {
        const GeoFencePoint& pt = _geo->at(_cpPointIndex);
        const float away = (float)NmeaUtil::haversineMeters(
            _gps->latitude(), _gps->longitude(), pt.lat, pt.lon);
        if (away > AppConst::GEOFENCE_LABEL_SHOW_M) {
            LOGF("[Event] %s NOT acknowledged - %.0fm past it after %u attempt(s), "
                 "giving up (the SMS and the race log still have the crossing)\n",
                 _cpLabel, away, (unsigned)_cpAttempts);
            _cpPending = false;
            return;
        }
    }

    if (_cpAttempts >= AppConst::LORA_EVENT_MAX_ATTEMPTS) {
        LOGF("[Event] %s NOT acknowledged after %u attempts - giving up\n",
             _cpLabel, (unsigned)_cpAttempts);
        _cpPending = false;
        return;
    }

    if ((int32_t)(millis() - _cpNextSendMs) < 0) return;

    // Payload: UTC date and time of the crossing, then the label.
    uint8_t payload[LORA_MAX_PAYLOAD + 1];
    payload[0] = (uint8_t)(_cpWhenUtc.year & 0xFF);
    payload[1] = (uint8_t)(_cpWhenUtc.year >> 8);
    payload[2] = _cpWhenUtc.month;
    payload[3] = _cpWhenUtc.day;
    payload[4] = _cpWhenUtc.hour;
    payload[5] = _cpWhenUtc.minute;
    payload[6] = _cpWhenUtc.second;
    payload[7] = _cpWhenUtc.centisecond;
    size_t labelLen = strlen(_cpLabel);
    if (labelLen > LORA_MAX_PAYLOAD - LORA_CP_TIME_LEN) {
        labelLen = LORA_MAX_PAYLOAD - LORA_CP_TIME_LEN;
    }
    memcpy(payload + LORA_CP_TIME_LEN, _cpLabel, labelLen);

    const bool queued = _lora->sendRaw(LoRaMsgType::CHECKPOINT_EVENT, LORA_BROADCAST_ID,
                                       _cpEventId, _cpDistanceCm,
                                       payload, LORA_CP_TIME_LEN + labelLen);
    _cpAttempts++;
    _cpNextSendMs = millis() + AppConst::LORA_EVENT_RETRY_MS;

    LOGF("[Event] %s crossing at %02u:%02u:%02u.%02u UTC, %ld cm - attempt %u%s\n",
         _cpLabel, _cpWhenUtc.hour, _cpWhenUtc.minute, _cpWhenUtc.second,
         _cpWhenUtc.centisecond, (long)_cpDistanceCm, (unsigned)_cpAttempts,
         queued ? "" : " (NOT QUEUED - radio unavailable or queue full)");
}

void AppController::onLoRaMessage(const LoRaMessage& msg) {
    switch (msg.type) {
        case LoRaMsgType::CHECKPOINT_ACK:
            // The transport has already discarded anything addressed to
            // another device, so reaching here means this is for us. The
            // event id is what ties it to a specific crossing rather than
            // to whichever retry happened to be heard.
            if (_cpPending && msg.sessionId == _cpEventId) {
                LOGF("[Event] %s ACKNOWLEDGED by station %u after %u attempt(s) "
                     "(RSSI %d, SNR %.1f)\n",
                     _cpLabel, (unsigned)msg.srcDeviceId, (unsigned)_cpAttempts,
                     msg.rssi, msg.snr);
                _cpPending = false;
            } else {
                LOGF("[Event] Ignored a stale acknowledgement (event %u, expecting %u)\n",
                     (unsigned)msg.sessionId, (unsigned)(_cpPending ? _cpEventId : 0));
            }
            break;

        default:
            // Give Way message types land here once OvertakeManager exists.
            LOGF("[LoRa] Received type %u from device %u (RSSI %d) - no handler yet\n",
                 (unsigned)msg.type, (unsigned)msg.srcDeviceId, msg.rssi);
            break;
    }
}

void AppController::handleButtonEvent(ButtonEvent evt) {
    switch (evt) {
        case ButtonEvent::KEY1_RESTART:
            LOGLN("[Button] Key1 held 1.5s - restarting");
            ESP.restart();
            break;
        case ButtonEvent::KEY4_GIVEWAY_ACK:
            LOGLN("[Button] Key4 quick tap - Give Way ack pulse (OvertakeManager not yet implemented)");
            // TODO(Phase 6): forward to OvertakeManager as the ahead-driver ack.
            break;
        case ButtonEvent::KEY4_LOG_TOGGLE:
            if (_log->isLogging()) {
                _log->stopManually();
                LOGLN("[Button] Key4 1.5s - manual log stop");
            } else {
                openLogWithLocalTime();
                LOGLN("[Button] Key4 1.5s - manual log start (logging only, "
                                "no geofence marking/SMS/LoRa)");
            }
            break;
        case ButtonEvent::KEY2_GIVEWAY_TOGGLE:
            LOGLN("[Button] Key2 2s - Give Way toggle (OvertakeManager not yet implemented)");
            break;
        case ButtonEvent::KEY2_WIFI_TOGGLE:
            LOGLN("[Button] Key2 5s - Wi-Fi AP toggle (WebManager not yet implemented)");
            break;
        case ButtonEvent::NONE:
        default:
            break;
    }
}

void AppController::updateDisplayModel() {
    RaceDataModel model;

    // Each live GNSS field is gated on its OWN freshness, so a lost fix
    // clears them independently instead of leaving the last known values
    // frozen on screen (GpsManager::isFresh explains why isValid() alone
    // is not enough). DisplayManager renders "--" for anything invalid.
    if (_gps->speedValid()) {
        model.speedValid = true;
        model.speedKmh = _gps->speedKmh();
    }
    if (_gps->satellitesValid()) {
        model.satsValid = true;
        model.satCount = _gps->satellites();
    }
    if (_gps->accuracyValid()) {
        model.accuracyValid = true;
        model.accuracyM = _gps->accuracyMeters();
    }

    // Covered distance and the last crossing time are accumulated/captured
    // race state, not live sensor readings - they stay on screen through a
    // GNSS dropout because they remain true, unlike a stale speed.
    model.distanceValid = true;
    model.correctedDistanceM = (float)_correctedDistanceM;

    // 'L' tracks actively-writing, not merely file-open: below the 2 km/h
    // logging threshold writing pauses, and spec section 10 says to remove
    // the indicator when logging is paused/stopped. 'O' shows the file is
    // open, which is the state that persists through such a pause.
    model.loggingActive = _log->isActivelyWriting();
    model.logFileOpen = _log->isLogging();

    model.batteryValid = _battery->isPresent();
    model.batteryPercent = _battery->percentEstimate();
    model.batteryLow = _battery->isLow();

    if (_crossingTimeVisible) {
        model.crossingTimeValid = true;
        model.xh = _lastCrossHh; model.xm = _lastCrossMm;
        model.xs = _lastCrossSs; model.xcs = _lastCrossCs;
    }

    // The label comes from _geofenceLabel, not from nextIndex(): the point
    // on screen may be the one just PASSED, which nextIndex has already
    // moved beyond.
    if (_geofenceLabelValid) {
        model.geofenceLabelValid = true;
        strncpy(model.geofenceLabel, _geofenceLabel, sizeof(model.geofenceLabel) - 1);
        model.geofenceLabel[sizeof(model.geofenceLabel) - 1] = '\0';
    }
    if (_geofenceDistValid) {
        model.geofenceDistValid = true;
        model.geofenceDistanceM = _geofenceDistanceM;
    }

    // Fields 1/6 (ahead device distance/ID) stay at their default
    // "unavailable" state - Give Way / OvertakeManager not implemented yet.

    _display->updateDataModel(model);
}

void AppController::loop() {
    updateTraveledDistance();

    ButtonEvent evt = _buttons->loop();
    handleButtonEvent(evt);

    updateRouteCorrection();
    updateGeofenceCrossing();
    updateCheckpointBroadcast();
    updateRaceStage();

    // A stale/absent speed reading counts as stopped: without a trustworthy
    // speed we must not keep writing lines as if the vehicle were moving.
    _log->updateSpeed(_gps->speedValid() ? _gps->speedKmh() : 0.0f);
    _log->loop();

    // BatteryManager is sampled by main.cpp at device level (it must keep
    // running even when race operations are halted), so AppController only
    // reads its state here.
    updateDisplayModel();
}
