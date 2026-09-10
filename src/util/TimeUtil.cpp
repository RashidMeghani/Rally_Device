#include "TimeUtil.h"

namespace {

bool isLeapYear(uint16_t y) {
    return (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
}

uint8_t daysInMonth(uint16_t year, uint8_t month) {
    static const uint8_t lengths[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (month < 1 || month > 12) return 30; // malformed input: don't index out of range
    if (month == 2 && isLeapYear(year)) return 29;
    return lengths[month - 1];
}

} // namespace

namespace TimeUtil {

LocalDateTime applyUtcOffset(uint16_t year, uint8_t month, uint8_t day,
                              uint8_t hour, uint8_t minute, uint8_t second,
                              int16_t offsetMinutes) {
    LocalDateTime out;
    out.year = year;
    out.month = month;
    out.day = day;
    out.second = second;

    int32_t totalMinutes = (int32_t)hour * 60 + (int32_t)minute + (int32_t)offsetMinutes;
    int8_t dayShift = 0;
    while (totalMinutes < 0)     { totalMinutes += 24 * 60; dayShift--; }
    while (totalMinutes >= 24 * 60) { totalMinutes -= 24 * 60; dayShift++; }

    out.hour = (uint8_t)(totalMinutes / 60);
    out.minute = (uint8_t)(totalMinutes % 60);

    if (dayShift > 0) {
        out.day++;
        if (out.day > daysInMonth(out.year, out.month)) {
            out.day = 1;
            out.month++;
            if (out.month > 12) { out.month = 1; out.year++; }
        }
    } else if (dayShift < 0) {
        if (out.day > 1) {
            out.day--;
        } else {
            if (out.month > 1) {
                out.month--;
            } else {
                out.month = 12;
                out.year--;
            }
            out.day = daysInMonth(out.year, out.month);
        }
    }

    return out;
}

} // namespace TimeUtil
