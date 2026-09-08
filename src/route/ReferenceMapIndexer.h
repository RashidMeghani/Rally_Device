// Builds the searchable route index/cache from the raw-NMEA ReferenceMap.
//
// -----------------------------------------------------------------------
// Why dedup is needed, and how it's done (Rev 3, section 26)
// -----------------------------------------------------------------------
// ReferenceMap.log is the authoritative raw capture: every enabled NMEA
// sentence, verbatim, exactly as received. A single u-blox M8N GNSS epoch
// (one physical position fix) is normally reported through *multiple*
// sentence types in the same output burst - e.g. GNRMC and GNGGA both carry
// a full lat/lon for the same instant, and GNRMC/GNGGA/GNVTG all repeat the
// same UTC time-of-day field for that instant. If every position-bearing
// sentence were inserted into the route-search index, the same physical
// fix would be duplicated once per sentence type, corrupting cumulative
// distance (double counting) and roughly doubling+ the index size for no
// benefit.
//
// The indexer never touches ReferenceMap.log itself (it is opened read-only
// and every sentence in it - GGA, RMC, VTG, or anything else the receiver
// emits - is preserved forever as the authoritative source/replay log).
// Instead it streams the file once and groups sentences into "epochs" by
// their shared UTC time-of-day field:
//
//   1. GNRMC and GNGGA both carry the same "hhmmss.ss" time field for a
//      given epoch (VTG carries no position and is ignored for indexing).
//   2. While consecutive sentences share the same time field, they are the
//      *same physical fix*: the first sentence in the group that yields a
//      valid lat/lon "wins" and is buffered as the pending point; any later
//      sentence in the same group is inspected only for a fix-quality
//      upgrade path (see NOTE below) and never produces a second point.
//   3. When the time field changes (a new epoch begins), the previously
//      buffered point - if any - is emitted exactly once into the current
//      2 km segment file, with cumulative distance computed via haversine
//      from the last emitted point.
//   4. The end of the file flushes whatever epoch is still pending.
//
// This keeps the position stream strictly one-record-per-physical-fix
// while remaining agnostic to which particular sentence types are enabled
// (RMC/GGA today; would keep working unchanged if GLL or GNS were added).
//
// NOTE on GNGTV: the owner's enabled-sentence list names "GNGTV", which is
// very likely the commonly used VTG (course/speed over ground) sentence
// under a transcription difference. VTG never carries latitude/longitude in
// NMEA-0183, so regardless of whether the literal identifier is "VTG" or
// something else, it cannot contribute a position and is correctly ignored
// by the indexer's dedup key. This is flagged in the open-questions list
// for verification against the actual receiver output, but it does not
// block or change the indexing algorithm.
//
// -----------------------------------------------------------------------
// Segment/index file layout on SD
// -----------------------------------------------------------------------
//   /route/index.hdr   - small header: {sourceSize, fingerprintCrc,
//                         pointCount, segmentCount, totalDistanceM,
//                         builderVersion}. Used to decide whether a rebuild
//                         is needed (see needsRebuild()).
//   /route/index.csv    - top-level index, one row per 2 km segment:
//                         segIndex,startDistanceM,endDistanceM,fileName
//                         This whole file is small (~125 rows for a 250 km
//                         track) and is kept fully resident in RAM by
//                         RouteMatcher at runtime.
//   /route/seg_00000.csv, seg_00001.csv, ... - one file per ~2 km segment,
//                         each row: lat,lon,cumulativeDistanceM
//                         Only the 1-2 segments relevant to the current
//                         search hint need to be opened at a time; the
//                         indexer never loads the full ~250 km track into
//                         RAM, and neither does the runtime matcher.
//
// Complexity: O(number of raw NMEA lines) time, O(1) RAM for the indexer
// itself (a fixed line buffer + the small pending-epoch state); O(segment
// count) RAM (~2.5 KB for a 250 km track) for the runtime top-level index.
#pragma once

#include <FS.h>
#include <cstdint>

struct RouteIndexHeader {
    uint32_t sourceSize = 0;
    uint32_t fingerprintCrc = 0;
    uint32_t pointCount = 0;
    uint32_t segmentCount = 0;
    float totalDistanceM = 0;
    uint16_t builderVersion = 1;
};

struct RouteIndexResult {
    bool ok = false;
    bool rebuilt = false; // false if the on-disk index was already up to date
    RouteIndexHeader header;
};

class ReferenceMapIndexer {
public:
    // Cheap fingerprint check (file size + CRC32 of the first/last 256
    // bytes) against the stored header. This intentionally does NOT
    // rehash the whole ~250 km file on every boot - see class-level notes
    // in ReferenceMapIndexer.cpp for the trade-off this implies.
    static bool needsRebuild(fs::FS& fs, const char* rawMapPath, const char* indexHeaderPath);

    // Streams rawMapPath once, deduplicates physical fixes as described
    // above, and (re)writes the segment + top-level index files under
    // outDir. Uses temp-file-then-rename so a power loss mid-build cannot
    // leave a half-written index that looks valid.
    static RouteIndexResult build(fs::FS& fs, const char* rawMapPath, const char* outDir,
                                   const char* indexHeaderPath, const char* indexCsvPath,
                                   float segmentLengthM);

    // Convenience: calls needsRebuild() and only calls build() if required.
    static RouteIndexResult buildIfNeeded(fs::FS& fs, const char* rawMapPath, const char* outDir,
                                           const char* indexHeaderPath, const char* indexCsvPath,
                                           float segmentLengthM);
};
