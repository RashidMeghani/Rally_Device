#include "RouteMatcher.h"
#include "../util/FileUtil.h"
#include "../../include/AppConstants.h"
#include <Arduino.h>
#include <cstring>
#include <cstdlib>
#include <cmath>

namespace {

constexpr size_t LINE_BUF_LEN = 64;

// Parses "lat,lon,cumulativeDistanceM" as written by ReferenceMapIndexer.
bool parsePointRow(char* line, double& lat, double& lon, float& distanceM) {
    char* c1 = strchr(line, ',');
    if (!c1) return false;
    char* c2 = strchr(c1 + 1, ',');
    if (!c2) return false;
    *c1 = '\0';
    *c2 = '\0';
    lat = atof(line);
    lon = atof(c1 + 1);
    distanceM = (float)atof(c2 + 1);
    return true;
}

} // namespace

bool RouteMatcher::begin(fs::FS& fs, const char* routeDir, const char* indexCsvPath) {
    _fs = &fs;
    _segmentCount = 0;
    strncpy(_routeDir, routeDir, sizeof(_routeDir) - 1);

    File f = fs.open(indexCsvPath, FILE_READ);
    if (!f) {
        Serial.printf("[RouteMatch] No route index at %s - correction unavailable\n", indexCsvPath);
        return false;
    }

    // index.csv rows: segIndex,startDistanceM,endDistanceM,fileName
    // Only the distance span is kept; the file path is rebuilt from the
    // row position, which matches how the indexer names them.
    char line[LINE_BUF_LEN];
    while (FileUtil::readLine(f, line, sizeof(line))) {
        if (line[0] == '\0') continue;
        if (_segmentCount >= MAX_SEGMENTS) {
            Serial.printf("[RouteMatch] WARNING: route has more than %u segments - "
                          "ignoring the rest\n", (unsigned)MAX_SEGMENTS);
            break;
        }
        char* c1 = strchr(line, ',');
        if (!c1) continue;
        char* c2 = strchr(c1 + 1, ',');
        if (!c2) continue;
        char* c3 = strchr(c2 + 1, ',');
        if (c3) *c3 = '\0';
        *c2 = '\0';
        _segStart[_segmentCount] = (float)atof(c1 + 1);
        _segEnd[_segmentCount] = (float)atof(c2 + 1);
        _segmentCount++;
    }
    f.close();

    Serial.printf("[RouteMatch] Loaded %u segments, route length %.0f m\n",
                  (unsigned)_segmentCount, routeLengthM());
    return _segmentCount > 0;
}

int RouteMatcher::segmentForDistance(float distanceM) const {
    for (size_t i = 0; i < _segmentCount; ++i) {
        if (distanceM <= _segEnd[i]) return (int)i;
    }
    return (int)_segmentCount - 1; // past the end: search the last segment
}

void RouteMatcher::searchSegment(size_t index, double lat, double lon,
                                  RouteMatch& best, float& bestLateral) {
    if (index >= _segmentCount) return;

    char path[40];
    snprintf(path, sizeof(path), "%s/seg_%05u.csv", _routeDir, (unsigned)index);
    File f = _fs->open(path, FILE_READ);
    if (!f) return;

    // Local planar frame centred on the live point: over the ~2 km a
    // segment spans, treating lat/lon as flat is accurate to well under a
    // metre, and it makes the projection plain 2D vector maths.
    const double mPerDegLat = 111320.0;
    const double mPerDegLon = 111320.0 * cos(lat * M_PI / 180.0);

    char line[LINE_BUF_LEN];
    bool havePrev = false;
    double prevX = 0, prevY = 0;
    float prevDist = 0;

    while (FileUtil::readLine(f, line, sizeof(line))) {
        double pLat, pLon;
        float pDist;
        if (!parsePointRow(line, pLat, pLon, pDist)) continue;

        const double x = (pLon - lon) * mPerDegLon;
        const double y = (pLat - lat) * mPerDegLat;

        if (havePrev) {
            // Project the live point (the origin of this frame) onto the
            // line between the previous and current route points.
            const double abx = x - prevX;
            const double aby = y - prevY;
            const double len2 = abx * abx + aby * aby;

            double t = 0.0;
            if (len2 > 0.0) {
                t = -(prevX * abx + prevY * aby) / len2;
                if (t < 0.0) t = 0.0;
                else if (t > 1.0) t = 1.0;
            }

            const double cx = prevX + t * abx;
            const double cy = prevY + t * aby;
            const float lateral = (float)sqrt(cx * cx + cy * cy);

            if (lateral < bestLateral) {
                bestLateral = lateral;
                best.correctedDistanceM = prevDist + (float)t * (pDist - prevDist);
                best.lateralErrorM = lateral;
                best.segmentIndex = (uint32_t)index;
            }
        }

        prevX = x; prevY = y; prevDist = pDist;
        havePrev = true;
    }
    f.close();
}

RouteMatch RouteMatcher::match(double lat, double lon, float hintDistanceM) {
    RouteMatch best;
    if (_segmentCount == 0) return best;

    float bestLateral = 1.0e9f;

    if (hintDistanceM >= 0.0f) {
        const int centre = segmentForDistance(hintDistanceM);
        for (int radius = 0; radius <= MAX_SEARCH_RADIUS; ++radius) {
            searchSegment((size_t)(centre - radius), lat, lon, best, bestLateral);
            if (radius > 0) {
                searchSegment((size_t)(centre + radius), lat, lon, best, bestLateral);
            }
            // Stop as soon as a good enough match is in hand: widening
            // further can only find something farther along the route.
            if (bestLateral <= AppConst::ROUTE_MATCH_THRESHOLD_M) break;
        }
        // Negative indices wrap huge and are rejected by the bounds check
        // inside searchSegment, so no extra guarding is needed here.
    } else {
        // Reacquisition: nothing is known about where we are, so the whole
        // route has to be considered. Deliberately rare - once per reset.
        best.fullScan = true;
        for (size_t i = 0; i < _segmentCount; ++i) {
            searchSegment(i, lat, lon, best, bestLateral);
        }
    }

    best.valid = (bestLateral <= AppConst::ROUTE_MATCH_THRESHOLD_M);
    return best;
}
