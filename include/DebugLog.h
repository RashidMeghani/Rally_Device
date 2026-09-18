// Compile-time switchable serial diagnostics.
//
// The device normally runs with nothing connected to the USB port - on a
// bike or in a car - and only sees a serial monitor during bench testing.
// Leaving the diagnostics on for a race is not free:
//
//   - Serial.print() writes into the UART TX ring buffer, which drains at
//     the configured baud (~11.5 KB/s at 115200) whether or not anything is
//     listening. Write faster than that and the call BLOCKS. Blocking in
//     loop() stalls GNSS consumption, which is precisely what the 4 KB RX
//     buffer in GpsManager exists to survive - so debug output competes
//     with the thing it is meant to help debug.
//   - Formatting (printf's float conversion especially) costs real cycles
//     on every call, thrown away when no one is reading.
//
// Both switches below are `constexpr`, so when false the compiler removes
// the call AND the argument evaluation entirely - genuinely zero cost, not
// merely "a branch that is not taken".
//
// To use a serial monitor, set DEBUG_SERIAL true in AppConstants.h and
// re-flash. Serial.begin() runs either way, so uploading is unaffected and
// the port is always there.
#pragma once

#include <Arduino.h>
#include "AppConstants.h"

// Diagnostics: boot progress, geofence crossings, route corrections,
// battery, button events, file errors.
#define LOGF(...)  do { if (AppConst::DEBUG_SERIAL) Serial.printf(__VA_ARGS__); } while (0)
#define LOGLN(...) do { if (AppConst::DEBUG_SERIAL) Serial.println(__VA_ARGS__); } while (0)

// The raw NMEA echo, split out because it is by far the highest volume -
// ~15 sentences a second at 5 Hz, around 1 KB/s on its own, more than every
// other message combined. Useful when checking what the receiver is
// actually emitting; noise the rest of the time.
#define LOG_NMEA(line) \
    do { if (AppConst::DEBUG_SERIAL && AppConst::DEBUG_SERIAL_RAW_NMEA) Serial.println(line); } while (0)
