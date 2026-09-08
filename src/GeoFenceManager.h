// Parses and holds the ordered list of point geofences from GeoFencing.txt.
//
// Confirmed format (Rev 3, section 25): plain CSV, no header, row order is
//   latitude,longitude,distance_from_start_m,label
// The file's row order is the race order (first row = start, last row =
// finish). Geofences are POINTS, never gate lines.
#pragma once

#include <Arduino.h>
#include <FS.h>
#include <vector>

struct GeoFencePoint {
    double lat = 0;
    double lon = 0;
    float distanceFromStartM = 0;
    char label[16] = {0};
    bool passed = false; // latched once crossed (or explicitly skipped on recovery)
};

class GeoFenceManager {
public:
    // Parses geoFencePath from `fs`. Malformed rows are reported on Serial
    // and skipped (never silently substituted with a guessed value).
    // Returns true if at least one valid point was loaded.
    bool load(fs::FS& fs, const char* geoFencePath);

    size_t count() const { return _points.size(); }
    const GeoFencePoint& at(size_t i) const { return _points[i]; }
    GeoFencePoint& at(size_t i) { return _points[i]; }

    // Index of the first not-yet-passed point, or count() if all passed.
    size_t nextIndex() const { return _nextIndex; }

    // Marks `index` passed (normal crossing) and advances nextIndex if it
    // was the current target.
    void markPassed(size_t index);

    // Reset-recovery rule (spec section 8/10): given a reacquired corrected
    // race distance, mark every point whose distanceFromStartM is behind it
    // as passed/skipped, without re-triggering them. Never moves backward.
    void skipPassedBefore(float correctedDistanceM);

    void reset();

private:
    std::vector<GeoFencePoint> _points;
    size_t _nextIndex = 0;
};
