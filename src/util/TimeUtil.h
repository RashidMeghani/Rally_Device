// UTC -> local time conversion for displayed times and race-log filenames.
//
// Applies only to what a human reads: the OLED crossing time and the log
// filename. The raw NMEA sentences written into ReferenceMap/race logs are
// never touched - they stay verbatim UTC, because they are the
// authoritative record and a replay tool must be able to trust them.
#pragma once

#include <cstdint>

struct LocalDateTime {
    uint16_t year = 0;
    uint8_t month = 0;
    uint8_t day = 0;
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
};

// A GNSS instant at the receiver's own resolution: UTC date plus time to
// the centisecond, which is what NMEA carries.
struct GnssInstant {
    uint16_t year = 0;
    uint8_t month = 0, day = 0;
    uint8_t hour = 0, minute = 0, second = 0, centisecond = 0;
};

namespace TimeUtil {

// Interpolates between two consecutive GNSS instants at fraction f (0 = a,
// 1 = b), rolling the date if the pair straddles midnight.
//
// This is what lets a geofence crossing be timed more precisely than the
// receiver's fix interval: the vehicle is never sampled exactly on the
// line, so the instant it was there is computed from the two fixes either
// side of it. At 100 km/h with 1 Hz fixes the nearest sample can be 14 m
// away - half a second of error - while the interpolated instant is good to
// about a tenth of that.
GnssInstant interpolateUtc(const GnssInstant& a, const GnssInstant& b, float f);

// Adds offsetMinutes to a UTC date/time, rolling the date over correctly
// (including month/year boundaries and leap years) when the offset pushes
// past midnight in either direction. offsetMinutes is expected within
// +/-14h, which is the real-world timezone range.
LocalDateTime applyUtcOffset(uint16_t year, uint8_t month, uint8_t day,
                              uint8_t hour, uint8_t minute, uint8_t second,
                              int16_t offsetMinutes);

} // namespace TimeUtil
