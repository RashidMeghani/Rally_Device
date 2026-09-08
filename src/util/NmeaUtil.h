// Minimal, dependency-free NMEA-0183 helpers shared by GpsManager (live
// stream) and ReferenceMapIndexer (offline file preprocessing). Deliberately
// separate from TinyGPSPlus: the indexer needs the *raw* talker/time fields
// verbatim (as strings) to group sentences into physical fixes, which a
// higher-level streaming parser does not expose directly.
#pragma once

#include <cstdint>
#include <cstddef>

namespace NmeaUtil {

// Validates "$...*hh" checksum if a '*' is present. Sentences without a
// checksum are treated as unverifiable and accepted (some receivers/log
// captures omit it); callers that need strictness can check separately.
bool checksumValid(const char* sentence);

// Returns the 3-letter sentence id (e.g. "RMC","GGA","VTG") for a line of
// the form "$GNRMC,...", "$GPGGA,...", etc. Returns false if the line is not
// a recognizable "$xxYYY,..." NMEA sentence.
bool sentenceId(const char* line, char out[4]);

// Splits `line` (mutated in place, NUL-terminated) on commas and optional
// trailing "*checksum". Field pointers are written into `fields` up to
// maxFields. Returns the number of fields found (field[0] is the sentence
// id, e.g. "GNRMC").
int splitFields(char* line, char* fields[], int maxFields);

// Parses NMEA ddmm.mmmm / dddmm.mmmm + hemisphere into signed decimal
// degrees. Returns false on empty/malformed input.
bool parseLat(const char* ddmm, char hemi, double& outDeg);
bool parseLon(const char* dddmm, char hemi, double& outDeg);

// Great-circle distance in meters (haversine, WGS84 mean radius).
double haversineMeters(double lat1, double lon1, double lat2, double lon2);

// Small streaming CRC32 (used for the cheap ReferenceMap "did the source
// file change" fingerprint, not for cryptographic purposes).
uint32_t crc32Update(uint32_t crc, const uint8_t* data, size_t len);

} // namespace NmeaUtil
