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

namespace TimeUtil {

// Adds offsetMinutes to a UTC date/time, rolling the date over correctly
// (including month/year boundaries and leap years) when the offset pushes
// past midnight in either direction. offsetMinutes is expected within
// +/-14h, which is the real-world timezone range.
LocalDateTime applyUtcOffset(uint16_t year, uint8_t month, uint8_t day,
                              uint8_t hour, uint8_t minute, uint8_t second,
                              int16_t offsetMinutes);

} // namespace TimeUtil
