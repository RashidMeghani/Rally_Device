#include "AppController.h"
#include "../include/AppConstants.h"
#include "util/NmeaUtil.h"
#include "util/TimeUtil.h"
#include <Arduino.h>
#include <cstring>
#include <cmath>

void AppController::begin(GpsManager& gps, GeoFenceManager& geo, LogManager& log,
                           DisplayManager& display, ButtonManager& buttons, BatteryManager& battery,
                           ConfigManager& config, RouteMatcher& route) {
    _gps = &gps; _geo = &geo; _log = &log;
    _display = &display; _buttons = &buttons; _battery = &battery;
    _config = &config; _route = &route;
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
            Serial.printf("[Route] Reacquisition failed - nearest route point %.0fm away\n",
                          scan.lateralErrorM);
            return;
        }
        _correctedDistanceM = scan.correctedDistanceM;
        _haveRouteMatch = true;
        _lastCorrectionMs = millis();
        // The scan searched against the position captured when it STARTED,
        // and the vehicle has moved since. Now that there is a hint, ask
        // for an immediate cheap hinted match to refine against where the
        // vehicle actually is now.
        _correctionRequested = true;
        Serial.printf("[Route] Reacquired at %.0fm (lateral %.1fm, seg %u) - refining\n",
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
        Serial.printf("[Route] Skipped - accuracy %.1fm worse than %.1fm limit\n",
                      _gps->accuracyValid() ? _gps->accuracyMeters() : -1.0f, threshold);
        return;
    }

    // Nothing to hint with means reacquisition: kick off the incremental
    // whole-route scan and let subsequent ticks work through it.
    if (!_haveRouteMatch) {
        Serial.println("[Route] No reference position - starting full-route reacquisition scan");
        _route->startFullScan(_gps->latitude(), _gps->longitude());
        return;
    }

    const RouteMatch m = _route->match(_gps->latitude(), _gps->longitude(),
                                        (float)_correctedDistanceM);

    if (!m.valid) {
        Serial.printf("[Route] No match%s - nearest route point %.0fm away, beyond the %.0fm threshold\n",
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
            Serial.printf("[Route] REJECTED - %.0fm jump exceeds the %.0fm possible in this interval\n",
                          jump, maxJump);
            return;
        }
    }

    const double before = _correctedDistanceM;
    _correctedDistanceM = m.correctedDistanceM;
    _haveRouteMatch = true;

    Serial.printf("[Route] Corrected %.0fm -> %.0fm (delta %+.0fm, lateral %.1fm, seg %u%s) | raw %.0fm\n",
                  before, _correctedDistanceM, _correctedDistanceM - before,
                  m.lateralErrorM, (unsigned)m.segmentIndex,
                  m.fullScan ? ", full scan" : "", _rawTraveledDistanceM);
}

void AppController::resetApproachTracking() {
    _hasMinSample = false;
    _minDist = 0;
    _minSpeedKmh = 0;
    _departingSamples = 0;
    _announcedApproach = false;
    _gateRejected = false;
    _lastLoggedSampleDist = -1000.0f;
}

void AppController::updateGeofenceCrossing() {
    _geofenceLabelValid = false;
    _geofenceDistValid = false;
    _crossingTimeVisible = false;

    // No fix means no distance to any point, so every geofence field blanks
    // - consistent with the rule that live fields never show stale values.
    if (!_gps->hasFix()) return;

    const double lat = _gps->latitude(), lon = _gps->longitude();

    // ---------------------------------------------------------------------
    // The POINT JUST PASSED stays on screen while the vehicle drives away
    // from it (owner requirement). Without this, crossing a checkpoint
    // blanks the label, the distance and the freshly captured crossing time
    // in the same instant, because nextIndex has moved on to a target
    // kilometres ahead - the driver never gets to read the very thing the
    // crossing just produced. The departing windows are the same size as
    // the approaching ones, so a point is visible for LABEL_SHOW_M either
    // side of itself.
    // ---------------------------------------------------------------------
    float passedDist = 0;
    if (_passedPointValid) {
        const GeoFencePoint& passed = _geo->at(_passedIndex);
        passedDist = (float)NmeaUtil::haversineMeters(lat, lon, passed.lat, passed.lon);
        if (passedDist > AppConst::GEOFENCE_LABEL_SHOW_M) {
            // Out of the departing window: the label goes, and the captured
            // crossing time goes with it.
            _passedPointValid = false;
            _lastCrossingValid = false;
            Serial.printf("[Geofence] %s now %.0fm behind - clearing its label and crossing time\n",
                          passed.label, passedDist);
        }
    }

    const bool haveTarget = _geo->nextIndex() < _geo->count();
    size_t idx = _geo->nextIndex();
    float dist = 0;

    if (haveTarget) {
        if (idx != _trackedTargetIndex) {
            _trackedTargetIndex = idx;
            resetApproachTracking(); // new target: tracking starts clean
        }
        const GeoFencePoint& t = _geo->at(idx);
        dist = (float)NmeaUtil::haversineMeters(lat, lon, t.lat, t.lon);
    }

    // Which point owns the label/distance fields: the NEARER of the point
    // just passed and the point ahead. Normally only one is in range at
    // all, but checkpoints closer together than the window would otherwise
    // fight over the fields, and the nearer one is the one that matters.
    const GeoFencePoint* shown = nullptr;
    float shownDist = 0;
    if (_passedPointValid && (!haveTarget || passedDist <= dist)) {
        shown = &_geo->at(_passedIndex);
        shownDist = passedDist;
    } else if (haveTarget) {
        shown = &_geo->at(idx);
        shownDist = dist;
    }

    // OLED gating is a readout of the live distance, so it updates every
    // tick regardless of whether the fix underneath is new.
    if (shown) {
        if (shownDist <= AppConst::GEOFENCE_LABEL_SHOW_M) {
            _geofenceLabelValid = true;
            strncpy(_geofenceLabel, shown->label, sizeof(_geofenceLabel) - 1);
            _geofenceLabel[sizeof(_geofenceLabel) - 1] = '\0';
        }
        if (shownDist <= AppConst::GEOFENCE_PRECISE_ZONE_M) {
            _geofenceDistValid = true;
            _geofenceDistanceM = shownDist;
        }
    }

    // The captured crossing time lives and dies with its point's label
    // window, so the time on screen always belongs to a checkpoint the
    // driver can still see named.
    _crossingTimeVisible = _passedPointValid && _lastCrossingValid;

    if (!haveTarget) return; // every point crossed: nothing left to detect

    const GeoFencePoint& target = _geo->at(idx);

    // ---------------------------------------------------------------------
    // Everything below is a SAMPLE-TO-SAMPLE comparison and must therefore
    // advance only on a genuinely new position from the receiver.
    //
    // This is the bug that stopped crossings from ever latching in the
    // field: loop() runs thousands of times a second while the M8N commits
    // a position once or ten times a second, so the overwhelming majority
    // of ticks recomputed the SAME distance from the SAME fix. Comparing a
    // fix against itself yields "not closer" - which the old code read as
    // "no longer approaching" - so by the time a real fix arrived showing
    // the distance growing, the approach state had already been cleared and
    // the closest-approach test could not fire. The display kept working
    // throughout, because it never depended on that comparison.
    // ---------------------------------------------------------------------
    const uint32_t seq = _gps->fixSequence();
    if (seq == _lastGeofenceFixSeq) return;
    _lastGeofenceFixSeq = seq;

    if (dist > AppConst::GEOFENCE_PRECISE_ZONE_M) {
        // Left the zone. If a closest approach was recorded but never
        // confirmed (a fast pass through, or the confirmation samples ran
        // out), the vehicle still demonstrably passed the point - commit at
        // the recorded minimum rather than losing the checkpoint entirely.
        if (_hasMinSample) {
            const float exitSpeed = _gps->speedValid() ? _gps->speedKmh() : 0.0f;
            Serial.printf("[Geofence] %s left the %.0fm zone with no confirmed turnaround - "
                          "committing recorded closest approach %.0fm\n",
                          target.label, AppConst::GEOFENCE_PRECISE_ZONE_M, _minDist);
            tryCommitCrossing(idx, exitSpeed);
        }
        resetApproachTracking();
        return;
    }

    if (!_announcedApproach) {
        _announcedApproach = true;
        Serial.printf("[Geofence] Approaching %s - %.0fm, watching for closest approach\n",
                      target.label, dist);
    }

    // Already judged too slow for this checkpoint: the vehicle is still
    // beside the point and still slow, so re-arming would reject (and log)
    // again every few fixes. Stay quiet until the target changes or the
    // zone is left.
    if (_gateRejected) return;

    const float speed = _gps->speedValid() ? _gps->speedKmh() : 0.0f;
    const bool moving = speed > AppConst::GEOFENCE_DEPART_MIN_KMH;

    if (!_hasMinSample || dist < _minDist) {
        // Closest yet: this sample becomes the candidate crossing, and any
        // partial departure count is discarded.
        _hasMinSample = true;
        _minDist = dist;
        _minSpeedKmh = speed;
        _minHh = _gps->hour(); _minMm = _gps->minute();
        _minSs = _gps->second(); _minCs = _gps->centisecond();
        _minYear = _gps->year(); _minMonth = _gps->month(); _minDay = _gps->day();
        _departingSamples = 0;
    } else if (moving) {
        // Growing distance only counts as departing when the vehicle is
        // actually moving. Standing still, GNSS noise makes the distance
        // wander by a few metres and would otherwise confirm a departure
        // that never happened - which is exactly how a stationary device
        // latched a start point it was still 27 m short of.
        _departingSamples++;
    }

    // One line per NEW fix inside the precise zone, but only when something
    // has actually changed: parked inside the zone waiting for the race to
    // start, an unconditional line per fix would bury the log.
    if (fabsf(dist - _lastLoggedSampleDist) >= 1.0f || _departingSamples > 0) {
        _lastLoggedSampleDist = dist;
        Serial.printf("[Geofence] %s fix: %.1fm (min %.1fm, departing %u/%u, %.1f km/h%s)\n",
                      target.label, dist, _minDist, (unsigned)_departingSamples,
                      (unsigned)AppConst::GEOFENCE_DEPART_CONFIRM_SAMPLES, speed,
                      moving ? "" : ", stationary - not departing");
    }

    if (_departingSamples >= AppConst::GEOFENCE_DEPART_CONFIRM_SAMPLES) {
        const CrossingVerdict verdict = tryCommitCrossing(idx, speed);
        resetApproachTracking();
        // Only a too-slow crossing latches. A too-far one must stay
        // re-armable: the vehicle may yet turn around and come through the
        // point properly while still inside the zone.
        _gateRejected = (verdict == CrossingVerdict::REJECTED_SLOW);
    }
}

// Decides whether the recorded closest-approach sample really was a
// crossing, and latches it if so. Three conditions, each rejecting a
// different way of being wrong:
//
//   1. PROXIMITY - the vehicle must actually have reached the point. This
//      is what stops a point being claimed by someone who parked near it
//      and drove off.
//   2. DEPARTURE - it must then have left under power. A stationary device
//      never departs: the distance only wanders with GNSS noise.
//   3. CHECKPOINT SPEED - normal checkpoints additionally require the
//      spec's 10 km/h at the moment of closest approach.
//
// Conditions 1 and 3 judge the sample AT THE MINIMUM, not the reading now:
// the crossing happened back at that sample, and judging it by a later one
// would be judging the wrong moment.
AppController::CrossingVerdict AppController::tryCommitCrossing(size_t idx, float departSpeedKmh) {
    if (!_hasMinSample) return CrossingVerdict::REJECTED_FAR;

    const char* label = _geo->at(idx).label;

    if (_minDist > AppConst::GEOFENCE_CROSSING_MAX_CLOSEST_M) {
        Serial.printf("[Geofence] %s NOT counted - closest approach was only %.0fm, "
                      "further than the %.0fm a real crossing must reach\n",
                      label, _minDist, AppConst::GEOFENCE_CROSSING_MAX_CLOSEST_M);
        return CrossingVerdict::REJECTED_FAR;
    }

    if (departSpeedKmh <= AppConst::GEOFENCE_DEPART_MIN_KMH) {
        Serial.printf("[Geofence] %s NOT counted - closest approach %.0fm but the vehicle is "
                      "not moving away (%.1f km/h, needs above %.0f km/h)\n",
                      label, _minDist, departSpeedKmh, AppConst::GEOFENCE_DEPART_MIN_KMH);
        return CrossingVerdict::REJECTED_FAR;
    }

    // The 10 km/h gate applies to NORMAL checkpoints only. A race start
    // happens from standstill - the car sits on the line and accelerates
    // away, so its closest approach is at ~0 km/h and the gate would reject
    // the start every time. Spec section 8 separates the two cases for
    // exactly this reason: the gate is specified "for normal checkpoints",
    // with the initial/start comparison handled differently. The START is
    // not ungated, though - conditions 1 and 2 above still apply to it, and
    // they are what make a genuine start distinguishable from a device
    // sitting still somewhere near the line.
    const bool isStartPoint = (idx == 0);
    if (!isStartPoint && _minSpeedKmh <= AppConst::GEOFENCE_CROSSING_MIN_KMH) {
        Serial.printf("[Geofence] %s closest approach %.0fm NOT counted - speed %.1f km/h "
                      "is below the %.0f km/h checkpoint gate\n",
                      label, _minDist, _minSpeedKmh, AppConst::GEOFENCE_CROSSING_MIN_KMH);
        return CrossingVerdict::REJECTED_SLOW;
    }

    acceptCrossing(idx);
    return CrossingVerdict::ACCEPTED;
}

void AppController::acceptCrossing(size_t idx) {
    GeoFencePoint& target = _geo->at(idx);

    // The captured crossing instant is the GNSS UTC of the closest-approach
    // sample; convert to local time for display (the raw NMEA in the log
    // stays UTC either way).
    LocalDateTime local = TimeUtil::applyUtcOffset(
        _minYear, _minMonth, _minDay, _minHh, _minMm, _minSs,
        _config->get().utcOffsetMinutes);
    _lastCrossHh = local.hour; _lastCrossMm = local.minute;
    _lastCrossSs = local.second; _lastCrossCs = _minCs;
    _lastCrossingValid = true;

    // Remember it so its label, distance and crossing time stay on screen
    // while the vehicle drives away, instead of blanking the moment
    // markPassed() advances the target.
    _passedPointValid = true;
    _passedIndex = idx;

    _geo->markPassed(idx);

    // A crossing is the most trustworthy correction available: the point's
    // distance-from-start is surveyed, not inferred. It is never blocked by
    // the periodic interval - it applies the moment the crossing is
    // detected - and it restarts that interval, so the next periodic
    // correction is due 2 minutes from HERE rather than firing redundantly
    // seconds after this one.
    _correctedDistanceM = target.distanceFromStartM;
    _haveRouteMatch = true;
    _lastCorrectionMs = millis();

    Serial.printf("[Geofence] CROSSED %s at %02u:%02u:%02u.%02u (index %u, closest %.0fm, "
                  "%.1f km/h) - distance snapped to %.0fm\n",
                  target.label, _lastCrossHh, _lastCrossMm, _lastCrossSs, _lastCrossCs,
                  (unsigned)idx, _minDist, _minSpeedKmh, (double)_correctedDistanceM);

    dispatchCheckpointEvent(target);

    _crossedThisTick = true;
    _crossedIndexThisTick = idx;
}

void AppController::dispatchCheckpointEvent(const GeoFencePoint& point) {
    // TODO(Phase 4): queue SMS to the 5 configured numbers via GsmManager.
    // TODO(Phase 5): transmit the LoRa checkpoint event via LoRaTransport,
    //                retrying at ~1s intervals per spec section 11.
    Serial.printf("[Event] Checkpoint '%s' - SMS/LoRa dispatch not yet implemented "
                  "(GsmManager/LoRaTransport pending)\n", point.label);
}

void AppController::updateRaceStage() {
    bool justCrossedFinish = _crossedThisTick && _geo->count() > 0 &&
                              _crossedIndexThisTick == _geo->count() - 1;
    bool justCrossedStart = _crossedThisTick && _crossedIndexThisTick == 0;

    switch (_stage) {
        case RaceStage::WAIT_START:
            if (justCrossedStart) {
                // Raw distance is "travelled since logging began", so it
                // legitimately restarts at zero. Corrected distance must
                // NOT: acceptCrossing() just snapped it to the start
                // point's surveyed distance-from-start, which in the
                // confirmed example file is 131m rather than 0. Zeroing it
                // would contradict both GeoFencing.txt and the
                // ReferenceMap's cumulative distance, so the next
                // correction would visibly jump back up.
                _rawTraveledDistanceM = 0;
                _hasPrevFix = false;
                // Don't clobber a log the driver already started manually
                // via Key4 before reaching the actual start line.
                if (!_log->isLogging()) {
                    openLogWithLocalTime();
                    Serial.println("[Race] START crossed - stage ACTIVE, log opened");
                } else {
                    Serial.println("[Race] START crossed - stage ACTIVE, log already running (manual start)");
                }
                _stage = RaceStage::ACTIVE;
            }
            break;

        case RaceStage::ACTIVE:
            if (justCrossedFinish) {
                _log->finishAndClose();
                _stage = RaceStage::FINISHED;
                Serial.println("[Race] FINISH crossed - stage FINISHED, log closed permanently");
            } else if (!_log->isLogging() && !_log->isFinished()) {
                // LogManager auto-closed on its own (20-minute stop timeout).
                _stage = RaceStage::STOPPED;
                Serial.println("[Race] Stopped 20 min - stage STOPPED");
            }
            break;

        case RaceStage::STOPPED:
            if (_gps->speedValid() && _gps->speedKmh() > AppConst::LOG_MOVING_MIN_KMH) {
                openLogWithLocalTime();
                _stage = RaceStage::ACTIVE;
                Serial.println("[Race] Movement resumed - stage ACTIVE, new log opened");
            }
            break;

        case RaceStage::FINISHED:
            break; // terminal for this run
    }

    _crossedThisTick = false;
}

void AppController::handleButtonEvent(ButtonEvent evt) {
    switch (evt) {
        case ButtonEvent::KEY1_RESTART:
            Serial.println("[Button] Key1 held 1.5s - restarting");
            ESP.restart();
            break;
        case ButtonEvent::KEY4_GIVEWAY_ACK:
            Serial.println("[Button] Key4 quick tap - Give Way ack pulse (OvertakeManager not yet implemented)");
            // TODO(Phase 6): forward to OvertakeManager as the ahead-driver ack.
            break;
        case ButtonEvent::KEY4_LOG_TOGGLE:
            if (_log->isLogging()) {
                _log->stopManually();
                Serial.println("[Button] Key4 1.5s - manual log stop");
            } else {
                openLogWithLocalTime();
                Serial.println("[Button] Key4 1.5s - manual log start (logging only, "
                                "no geofence marking/SMS/LoRa)");
            }
            break;
        case ButtonEvent::KEY2_GIVEWAY_TOGGLE:
            Serial.println("[Button] Key2 2s - Give Way toggle (OvertakeManager not yet implemented)");
            break;
        case ButtonEvent::KEY2_WIFI_TOGGLE:
            Serial.println("[Button] Key2 5s - Wi-Fi AP toggle (WebManager not yet implemented)");
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
