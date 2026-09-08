#include "GeoFenceManager.h"

bool GeoFenceManager::load(fs::FS& fs, const char* geoFencePath) {
    _points.clear();
    _nextIndex = 0;

    File f = fs.open(geoFencePath, FILE_READ);
    if (!f) {
        Serial.printf("[GeoFence] ERROR: cannot open %s\n", geoFencePath);
        return false;
    }

    int lineNo = 0;
    while (f.available()) {
        String rawLine = f.readStringUntil('\n');
        lineNo++;
        rawLine.trim(); // strips leading/trailing whitespace and CR
        if (rawLine.length() == 0) continue; // tolerate final/blank newline

        // Expected: latitude,longitude,distance_from_start_m,label
        int c1 = rawLine.indexOf(',');
        int c2 = c1 >= 0 ? rawLine.indexOf(',', c1 + 1) : -1;
        int c3 = c2 >= 0 ? rawLine.indexOf(',', c2 + 1) : -1;
        if (c1 < 0 || c2 < 0 || c3 < 0) {
            Serial.printf("[GeoFence] WARN: line %d malformed (expected 4 fields): \"%s\"\n",
                          lineNo, rawLine.c_str());
            continue;
        }

        String latS = rawLine.substring(0, c1);
        String lonS = rawLine.substring(c1 + 1, c2);
        String distS = rawLine.substring(c2 + 1, c3);
        String label = rawLine.substring(c3 + 1);
        label.trim();

        latS.trim(); lonS.trim(); distS.trim();
        if (latS.length() == 0 || lonS.length() == 0 || distS.length() == 0 || label.length() == 0) {
            Serial.printf("[GeoFence] WARN: line %d has an empty field: \"%s\"\n", lineNo, rawLine.c_str());
            continue;
        }

        double lat = atof(latS.c_str());
        double lon = atof(lonS.c_str());
        float dist = distS.toFloat();

        if (lat < -90.0 || lat > 90.0) {
            Serial.printf("[GeoFence] WARN: line %d latitude out of range: %s\n", lineNo, latS.c_str());
            continue;
        }
        if (lon < -180.0 || lon > 180.0) {
            Serial.printf("[GeoFence] WARN: line %d longitude out of range: %s\n", lineNo, lonS.c_str());
            continue;
        }
        if (dist < 0.0f) {
            Serial.printf("[GeoFence] WARN: line %d negative distance: %s\n", lineNo, distS.c_str());
            continue;
        }
        if (label.length() >= sizeof(GeoFencePoint::label)) {
            Serial.printf("[GeoFence] WARN: line %d label too long, truncating: %s\n", lineNo, label.c_str());
        }

        GeoFencePoint pt;
        pt.lat = lat;
        pt.lon = lon;
        pt.distanceFromStartM = dist;
        strncpy(pt.label, label.c_str(), sizeof(pt.label) - 1);
        _points.push_back(pt); // file row order is preserved (== race order)
    }
    f.close();

    Serial.printf("[GeoFence] Loaded %u valid point(s) from %s\n",
                  (unsigned)_points.size(), geoFencePath);
    return !_points.empty();
}

void GeoFenceManager::markPassed(size_t index) {
    if (index >= _points.size()) return;
    _points[index].passed = true;
    while (_nextIndex < _points.size() && _points[_nextIndex].passed) {
        _nextIndex++;
    }
}

void GeoFenceManager::skipPassedBefore(float correctedDistanceM) {
    // Reset-recovery rule: any point behind the reacquired corrected
    // distance is marked passed/skipped so it is never re-triggered, and we
    // never navigate backward to fire an old checkpoint.
    for (size_t i = 0; i < _points.size(); ++i) {
        if (_points[i].distanceFromStartM < correctedDistanceM) {
            _points[i].passed = true;
        }
    }
    _nextIndex = 0;
    while (_nextIndex < _points.size() && _points[_nextIndex].passed) {
        _nextIndex++;
    }
}

void GeoFenceManager::reset() {
    for (auto& p : _points) p.passed = false;
    _nextIndex = 0;
}
