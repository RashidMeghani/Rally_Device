#include "NmeaUtil.h"
#include <cstring>
#include <cstdlib>
#include <cmath>

namespace NmeaUtil {

bool checksumValid(const char* sentence) {
    const char* star = strchr(sentence, '*');
    if (!star || strlen(star) < 3) return true; // no checksum present: unverifiable, accept
    uint8_t sum = 0;
    for (const char* p = sentence + 1; p < star; ++p) sum ^= (uint8_t)*p; // skip leading '$'
    uint8_t given = (uint8_t)strtol(star + 1, nullptr, 16);
    return sum == given;
}

bool sentenceId(const char* line, char out[4]) {
    if (line[0] != '$' || strlen(line) < 6) return false;
    // Talker is 2 chars (GN/GP/GL/GA/GB...), sentence id is the next 3.
    out[0] = line[3];
    out[1] = line[4];
    out[2] = line[5];
    out[3] = '\0';
    for (int i = 0; i < 3; ++i) {
        if (out[i] < 'A' || out[i] > 'Z') return false;
    }
    return true;
}

int splitFields(char* line, char* fields[], int maxFields) {
    // Terminate at '*' (checksum) or CR/LF if present.
    for (char* p = line; *p; ++p) {
        if (*p == '*' || *p == '\r' || *p == '\n') { *p = '\0'; break; }
    }
    int count = 0;
    char* start = line;
    if (start[0] == '$') start++; // drop leading '$'
    fields[count++] = start;
    for (char* p = start; *p && count < maxFields; ++p) {
        if (*p == ',') {
            *p = '\0';
            fields[count++] = p + 1;
        }
    }
    return count;
}

static bool parseCoord(const char* raw, char hemi, int degDigits, double& outDeg) {
    if (!raw || raw[0] == '\0') return false;
    // raw is "dd(d)mm.mmmm" - fixed integer degree width, remainder is minutes.
    size_t len = strlen(raw);
    if ((int)len <= degDigits) return false;
    char degPart[4] = {0};
    memcpy(degPart, raw, degDigits);
    double deg = atof(degPart);
    double minutes = atof(raw + degDigits);
    double val = deg + minutes / 60.0;
    if (hemi == 'S' || hemi == 'W') val = -val;
    outDeg = val;
    return true;
}

bool parseLat(const char* ddmm, char hemi, double& outDeg) {
    return parseCoord(ddmm, hemi, 2, outDeg);
}

bool parseLon(const char* dddmm, char hemi, double& outDeg) {
    return parseCoord(dddmm, hemi, 3, outDeg);
}

double haversineMeters(double lat1, double lon1, double lat2, double lon2) {
    constexpr double R = 6371000.0; // mean Earth radius, meters
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) *
               sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c;
}

uint32_t crc32Update(uint32_t crc, const uint8_t* data, size_t len) {
    crc = ~crc;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc >> 1) ^ (0xEDB88320U & (~(crc & 1) + 1));
        }
    }
    return ~crc;
}

} // namespace NmeaUtil
