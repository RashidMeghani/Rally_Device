// Maps a live GNSS position onto the ReferenceMap route and returns the
// corrected distance from race start (spec section 7).
//
// -----------------------------------------------------------------------
// Why this is not just "find the nearest recorded point"
// -----------------------------------------------------------------------
// Two refinements matter:
//
// 1. PROJECTION, not nearest-point. Snapping to the nearest recorded fix
//    quantises distance to the reference track's sample spacing - at a
//    1 Hz recon lap driven at 60 km/h that is ~17 m steps, and the value
//    would visibly jump rather than advance. Instead the live point is
//    projected onto the line between two consecutive route points and the
//    cumulative distance interpolated along it, which is smooth and also
//    yields the lateral (off-route) error for free.
//
// 2. HINTED SEARCH, not full scan. The route can be ~250 km / tens of
//    thousands of points. Scanning all of it per fix is exactly what the
//    spec forbids. The last accepted distance says which ~2 km segment to
//    look in, so a normal correction reads one or two segment files. The
//    search widens outward only when that fails, and a full scan happens
//    solely when there is no hint at all - i.e. reacquiring after a reset.
//
// Segment files are streamed row by row, never loaded whole, so RAM stays
// bounded regardless of how densely the reference lap was recorded.
#pragma once

#include <FS.h>
#include <cstdint>
#include <cstddef>

struct RouteMatch {
    bool valid = false;             // within the lateral threshold
    float correctedDistanceM = 0;   // distance from race start along the route
    float lateralErrorM = 0;        // how far off-route the live point sat
    uint32_t segmentIndex = 0;      // which segment matched (diagnostics)
    bool fullScan = false;          // true if this needed a whole-route search
};

class RouteMatcher {
public:
    // Loads the top-level segment index built by ReferenceMapIndexer.
    // routeDir is the directory holding seg_NNNNN.csv files.
    bool begin(fs::FS& fs, const char* routeDir, const char* indexCsvPath);

    bool isReady() const { return _segmentCount > 0; }
    size_t segmentCount() const { return _segmentCount; }
    float routeLengthM() const { return _segmentCount ? _segEnd[_segmentCount - 1] : 0.0f; }

    // hintDistanceM < 0 means "no idea where we are" and triggers a full
    // scan - use it only for reacquisition, not per-fix.
    RouteMatch match(double lat, double lon, float hintDistanceM);

private:
    // 250 km / 2 km = 125 segments; headroom for a longer route or a
    // smaller segment length. Fixed array rather than a vector: this runs
    // for hours in a vibrating vehicle, and not fragmenting the heap is
    // worth ~1 KB of static RAM.
    static constexpr size_t MAX_SEGMENTS = 192;
    // How far either side of the hinted segment to widen before giving up.
    // 3 segments is ~6 km of route either way - far beyond the 150 m
    // lateral threshold, so failing this means genuinely off-route rather
    // than merely mis-hinted.
    static constexpr int MAX_SEARCH_RADIUS = 3;

    fs::FS* _fs = nullptr;
    char _routeDir[24] = {0};
    float _segStart[MAX_SEGMENTS] = {0};
    float _segEnd[MAX_SEGMENTS] = {0};
    size_t _segmentCount = 0;

    // Scans one segment file, keeping `best` if it finds anything closer.
    void searchSegment(size_t index, double lat, double lon, RouteMatch& best, float& bestLateral);
    int segmentForDistance(float distanceM) const;
};
