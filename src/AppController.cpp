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

    // Until the device has ANY match it is reacquiring - after a reset it
    // must establish where it is promptly, not sit for a full correction
    // interval first. Retries stay rate-limited because a hintless match
    // scans the whole route.
    const uint32_t interval = _haveRouteMatch ? AppConst::ROUTE_CORRECTION_INTERVAL_MS
                                              : AppConst::ROUTE_REACQUIRE_RETRY_MS;
    if (millis() - _lastCorrectionMs < interval) return;
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

    // Without a previous match there is nothing to hint with, so the
    // matcher does a full scan. That is the reacquisition path.
    const float hint = _haveRouteMatch ? (float)_correctedDistanceM : -1.0f;
    const RouteMatch m = _route->match(_gps->latitude(), _gps->longitude(), hint);

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

void AppController::updateGeofenceCrossing() {
    _geofenceLabelValid = false;
    _geofenceDistValid = false;

    if (_geo->nextIndex() >= _geo->count()) return; // no target left this run
    if (!_gps->hasFix()) return;

    size_t idx = _geo->nextIndex();
    if (idx != _trackedTargetIndex) {
        _trackedTargetIndex = idx;
        _hasPrevSample = false; // new target: closest-approach tracking starts clean
    }

    GeoFencePoint& target = _geo->at(idx);
    float dist = (float)NmeaUtil::haversineMeters(_gps->latitude(), _gps->longitude(), target.lat, target.lon);

    if (dist <= AppConst::GEOFENCE_LABEL_SHOW_M) {
        _geofenceLabelValid = true;
    }
    if (dist <= AppConst::GEOFENCE_PRECISE_ZONE_M) {
        _geofenceDistValid = true;
        _geofenceDistanceM = dist;
    }

    if (dist <= AppConst::GEOFENCE_PRECISE_ZONE_M) {
        if (_hasPrevSample && dist > _prevDist && _prevWasShrinking) {
            // The local minimum (closest approach) occurred at the
            // PREVIOUS sample, not this one - that previous sample's
            // captured GNSS time is the crossing time.
            if (_gps->speedKmh() > AppConst::GEOFENCE_CROSSING_MIN_KMH) {
                acceptCrossing(idx);
            }
        }
        bool shrinkingNow = _hasPrevSample ? (dist < _prevDist) : true;
        _prevWasShrinking = shrinkingNow;
        _prevDist = dist;
        _prevHh = _gps->hour(); _prevMm = _gps->minute();
        _prevSs = _gps->second(); _prevCs = _gps->centisecond();
        _prevYear = _gps->year(); _prevMonth = _gps->month(); _prevDay = _gps->day();
        _hasPrevSample = true;
    } else {
        _hasPrevSample = false;
    }
}

void AppController::acceptCrossing(size_t idx) {
    GeoFencePoint& target = _geo->at(idx);

    // The captured crossing instant is GNSS UTC; convert to local time for
    // display (the raw NMEA in the log stays UTC either way).
    LocalDateTime local = TimeUtil::applyUtcOffset(
        _prevYear, _prevMonth, _prevDay, _prevHh, _prevMm, _prevSs,
        _config->get().utcOffsetMinutes);
    _lastCrossHh = local.hour; _lastCrossMm = local.minute;
    _lastCrossSs = local.second; _lastCrossCs = _prevCs;
    _lastCrossingValid = true;

    _geo->markPassed(idx);

    // A crossing is the most trustworthy correction available: the point's
    // distance-from-start is surveyed, not inferred. Snap to it and treat
    // it as a route match so the next correction can hint from here.
    _correctedDistanceM = target.distanceFromStartM;
    _haveRouteMatch = true;

    Serial.printf("[Geofence] Crossed %s at %02u:%02u:%02u.%02u (index %u)\n",
                  target.label, _lastCrossHh, _lastCrossMm, _lastCrossSs, _lastCrossCs, (unsigned)idx);

    dispatchCheckpointEvent(target);

    _crossedThisTick = true;
    _crossedIndexThisTick = idx;
    _hasPrevSample = false; // next target (if any) starts clean next tick
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
                _rawTraveledDistanceM = 0;
                _correctedDistanceM = 0;
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

    if (_lastCrossingValid) {
        model.crossingTimeValid = true;
        model.xh = _lastCrossHh; model.xm = _lastCrossMm;
        model.xs = _lastCrossSs; model.xcs = _lastCrossCs;
    }

    if (_geofenceLabelValid && _geo->nextIndex() < _geo->count()) {
        model.geofenceLabelValid = true;
        strncpy(model.geofenceLabel, _geo->at(_geo->nextIndex()).label, sizeof(model.geofenceLabel) - 1);
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
