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

namespace {

constexpr int32_t CS_PER_DAY = 24L * 60L * 60L * 100L;

int32_t centisecondsOfDay(const GnssInstant& t) {
    return (((int32_t)t.hour * 60 + t.minute) * 60 + t.second) * 100 + t.centisecond;
}

} // namespace

namespace TimeUtil {

GnssInstant interpolateUtc(const GnssInstant& a, const GnssInstant& b, float f) {
    if (f < 0.0f) f = 0.0f;
    else if (f > 1.0f) f = 1.0f;

    const int32_t ta = centisecondsOfDay(a);
    int32_t tb = centisecondsOfDay(b);
    // b is the LATER fix, so a smaller value means the pair straddles
    // midnight rather than that time ran backwards.
    if (tb < ta) tb += CS_PER_DAY;

    int32_t tx = ta + (int32_t)((double)f * (double)(tb - ta) + 0.5);

    GnssInstant out;
    out.year = a.year; out.month = a.month; out.day = a.day;
    if (tx >= CS_PER_DAY) {
        tx -= CS_PER_DAY;
        // Advance one day, reusing the calendar rules below via a +24h
        // offset applied to midnight of a's date.
        LocalDateTime nextDay = applyUtcOffset(a.year, a.month, a.day, 0, 0, 0, 24 * 60);
        out.year = nextDay.year; out.month = nextDay.month; out.day = nextDay.day;
    }
    out.hour   = (uint8_t)(tx / 360000);
    out.minute = (uint8_t)((tx / 6000) % 60);
    out.second = (uint8_t)((tx / 100) % 60);
    out.centisecond = (uint8_t)(tx % 100);
    return out;
}

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
