#include "ReferenceMapIndexer.h"
#include "../util/NmeaUtil.h"
#include <Arduino.h>
#include <cstring>

namespace {

constexpr size_t LINE_BUF_LEN = 128;
constexpr int MAX_FIELDS = 20;
constexpr size_t FINGERPRINT_SAMPLE_BYTES = 256;

// Trade-off (documented, not silent): the "did the source file change"
// check below is a cheap fingerprint (size + CRC32 of the first/last 256
// bytes), not a full-file hash. Rebuilding by rehashing a ~250 km raw NMEA
// file on every boot would itself violate the "don't scan the whole route
// every cycle" cost concern this index exists to avoid. A same-size,
// interior-only edit of a ReferenceMap would not be detected by this
// fingerprint; in practice ReferenceMap.log is produced by a single
// recon/logging pass and is either replaced wholesale or only appended to,
// both of which this fingerprint reliably catches. If stricter detection is
// ever required, a full-file CRC32 can be substituted at the cost of a full
// read on every boot.
uint32_t fingerprintOf(fs::FS& fs, const char* path, uint32_t& outSize) {
    File f = fs.open(path, FILE_READ);
    if (!f) { outSize = 0; return 0; }
    outSize = f.size();
    uint32_t crc = 0;
    uint8_t buf[FINGERPRINT_SAMPLE_BYTES];

    size_t headLen = f.read(buf, min((size_t)outSize, FINGERPRINT_SAMPLE_BYTES));
    crc = NmeaUtil::crc32Update(crc, buf, headLen);

    if (outSize > FINGERPRINT_SAMPLE_BYTES) {
        size_t tailLen = min((size_t)FINGERPRINT_SAMPLE_BYTES, (size_t)outSize - FINGERPRINT_SAMPLE_BYTES);
        f.seek(outSize - tailLen, SeekSet);
        size_t got = f.read(buf, tailLen);
        crc = NmeaUtil::crc32Update(crc, buf, got);
    }
    f.close();
    return crc;
}

bool readHeader(fs::FS& fs, const char* path, RouteIndexHeader& hdr) {
    File f = fs.open(path, FILE_READ);
    if (!f) return false;
    size_t n = f.read(reinterpret_cast<uint8_t*>(&hdr), sizeof(hdr));
    f.close();
    return n == sizeof(hdr);
}

bool writeHeader(fs::FS& fs, const char* finalPath, const RouteIndexHeader& hdr) {
    String tmpPath = String(finalPath) + ".tmp";
    File f = fs.open(tmpPath, FILE_WRITE);
    if (!f) return false;
    size_t n = f.write(reinterpret_cast<const uint8_t*>(&hdr), sizeof(hdr));
    f.close();
    if (n != sizeof(hdr)) { fs.remove(tmpPath); return false; }
    fs.remove(finalPath);
    return fs.rename(tmpPath.c_str(), finalPath);
}

// Reads one line (without trailing CR/LF) into `out` (size LINE_BUF_LEN).
// Returns false at EOF with nothing read.
bool readLine(File& f, char* out, size_t outLen) {
    size_t i = 0;
    bool any = false;
    while (f.available()) {
        int c = f.read();
        if (c < 0) break;
        any = true;
        if (c == '\n') break;
        if (c == '\r') continue;
        if (i < outLen - 1) out[i++] = (char)c;
    }
    out[i] = '\0';
    return any;
}

struct PendingEpoch {
    char timeField[16] = {0};
    bool hasCoord = false;
    double lat = 0, lon = 0;

    void clear() { timeField[0] = '\0'; hasCoord = false; }
    bool isSameEpoch(const char* t) const { return timeField[0] != '\0' && strcmp(timeField, t) == 0; }
};

} // namespace

bool ReferenceMapIndexer::needsRebuild(fs::FS& fs, const char* rawMapPath, const char* indexHeaderPath) {
    RouteIndexHeader stored;
    if (!readHeader(fs, indexHeaderPath, stored)) return true; // no valid index yet

    uint32_t size = 0;
    uint32_t crc = fingerprintOf(fs, rawMapPath, size);
    if (size == 0) return true; // source missing/unreadable -> caller will report the error on build()

    return !(stored.sourceSize == size && stored.fingerprintCrc == crc);
}

RouteIndexResult ReferenceMapIndexer::build(fs::FS& fs, const char* rawMapPath, const char* outDir,
                                             const char* indexHeaderPath, const char* indexCsvPath,
                                             float segmentLengthM) {
    RouteIndexResult result;
    result.rebuilt = true;

    File src = fs.open(rawMapPath, FILE_READ);
    if (!src) {
        Serial.printf("[RouteIndex] ERROR: cannot open ReferenceMap at %s\n", rawMapPath);
        return result;
    }

    fs.mkdir(outDir);
    String idxCsvTmp = String(indexCsvPath) + ".tmp";
    File idxCsv = fs.open(idxCsvTmp, FILE_WRITE);
    if (!idxCsv) {
        Serial.println("[RouteIndex] ERROR: cannot create index.csv.tmp");
        src.close();
        return result;
    }

    char line[LINE_BUF_LEN];
    char fieldsBuf[LINE_BUF_LEN];
    char* fields[MAX_FIELDS];

    PendingEpoch pending;
    bool havePrevPoint = false;
    double prevLat = 0, prevLon = 0;
    double cumulativeDist = 0;
    double segStartDist = 0;
    uint32_t pointCount = 0;
    uint32_t segIndex = 0;
    char segFileName[32];
    File curSeg;

    auto openSegment = [&](uint32_t idx) {
        snprintf(segFileName, sizeof(segFileName), "%s/seg_%05u.csv", outDir, (unsigned)idx);
        curSeg = fs.open(segFileName, FILE_WRITE);
    };
    openSegment(segIndex);

    auto emitPoint = [&](double lat, double lon) {
        double stepDist = 0;
        if (havePrevPoint) {
            stepDist = NmeaUtil::haversineMeters(prevLat, prevLon, lat, lon);
            cumulativeDist += stepDist;
        }
        prevLat = lat; prevLon = lon; havePrevPoint = true;
        pointCount++;

        if (!curSeg) return;
        curSeg.printf("%.6f,%.6f,%.2f\n", lat, lon, cumulativeDist);

        if (cumulativeDist - segStartDist >= (double)segmentLengthM) {
            curSeg.flush();
            curSeg.close();
            idxCsv.printf("%u,%.2f,%.2f,%s\n", (unsigned)segIndex, segStartDist, cumulativeDist, segFileName);
            segStartDist = cumulativeDist;
            segIndex++;
            openSegment(segIndex);
        }
    };

    auto flushPending = [&]() {
        if (pending.hasCoord) emitPoint(pending.lat, pending.lon);
        pending.clear();
    };

    while (readLine(src, line, sizeof(line))) {
        if (line[0] != '$') continue;
        if (!NmeaUtil::checksumValid(line)) continue; // corrupt sentence: skip for indexing (still preserved in raw log)

        char id[4];
        if (!NmeaUtil::sentenceId(line, id)) continue;
        bool isRmc = strcmp(id, "RMC") == 0;
        bool isGga = strcmp(id, "GGA") == 0;
        if (!isRmc && !isGga) continue; // VTG etc. carry no position, irrelevant to dedup

        strncpy(fieldsBuf, line, sizeof(fieldsBuf) - 1);
        fieldsBuf[sizeof(fieldsBuf) - 1] = '\0';
        int n = NmeaUtil::splitFields(fieldsBuf, fields, MAX_FIELDS);

        // RMC: 0=id,1=time,2=status(A/V),3=lat,4=N/S,5=lon,6=E/W,...
        // GGA: 0=id,1=time,2=lat,3=N/S,4=lon,5=E/W,6=fixQuality,...
        const char* timeField = (n > 1) ? fields[1] : "";
        if (!timeField[0]) continue;

        if (!pending.isSameEpoch(timeField)) {
            flushPending();
            strncpy(pending.timeField, timeField, sizeof(pending.timeField) - 1);
        }

        if (!pending.hasCoord) {
            double lat, lon;
            bool valid = false;
            if (isRmc && n > 6 && fields[2][0] == 'A') {
                valid = NmeaUtil::parseLat(fields[3], fields[4][0], lat) &&
                        NmeaUtil::parseLon(fields[5], fields[6][0], lon);
            } else if (isGga && n > 6 && atoi(fields[6]) > 0) {
                valid = NmeaUtil::parseLat(fields[2], fields[3][0], lat) &&
                        NmeaUtil::parseLon(fields[4], fields[5][0], lon);
            }
            if (valid) { pending.lat = lat; pending.lon = lon; pending.hasCoord = true; }
        }
        // If pending.hasCoord is already true, a second sentence in the same
        // epoch is deliberately ignored here: first-valid-wins, so the
        // physical fix is stored exactly once regardless of how many
        // sentence types reported it.
    }
    flushPending();
    src.close();

    if (curSeg) {
        curSeg.flush();
        curSeg.close();
        if (cumulativeDist > segStartDist) {
            idxCsv.printf("%u,%.2f,%.2f,%s\n", (unsigned)segIndex, segStartDist, cumulativeDist, segFileName);
        } else {
            fs.remove(segFileName); // trailing empty segment
        }
    }
    idxCsv.flush();
    idxCsv.close();
    fs.remove(indexCsvPath);
    fs.rename(idxCsvTmp.c_str(), indexCsvPath);

    RouteIndexHeader hdr;
    hdr.fingerprintCrc = fingerprintOf(fs, rawMapPath, hdr.sourceSize);
    hdr.pointCount = pointCount;
    hdr.segmentCount = segIndex + (cumulativeDist > segStartDist ? 1 : 0);
    hdr.totalDistanceM = (float)cumulativeDist;
    hdr.builderVersion = 1;

    if (!writeHeader(fs, indexHeaderPath, hdr)) {
        Serial.println("[RouteIndex] ERROR: failed to commit index header");
        return result;
    }

    Serial.printf("[RouteIndex] Rebuilt: %u points, %u segments, %.1f m total\n",
                  (unsigned)hdr.pointCount, (unsigned)hdr.segmentCount, hdr.totalDistanceM);

    result.ok = true;
    result.header = hdr;
    return result;
}

RouteIndexResult ReferenceMapIndexer::buildIfNeeded(fs::FS& fs, const char* rawMapPath, const char* outDir,
                                                     const char* indexHeaderPath, const char* indexCsvPath,
                                                     float segmentLengthM) {
    if (!needsRebuild(fs, rawMapPath, indexHeaderPath)) {
        RouteIndexResult result;
        result.ok = readHeader(fs, indexHeaderPath, result.header);
        result.rebuilt = false;
        Serial.printf("[RouteIndex] Existing index up to date (%u points, %u segments)\n",
                      (unsigned)result.header.pointCount, (unsigned)result.header.segmentCount);
        return result;
    }
    return build(fs, rawMapPath, outDir, indexHeaderPath, indexCsvPath, segmentLengthM);
}
