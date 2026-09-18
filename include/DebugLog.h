// Serial diagnostics, switchable at runtime and strippable at compile time.
//
// The device normally runs with nothing connected to the USB port - on a
// bike or in a car - and only sees a serial monitor during bench testing.
// Leaving the output on for a race is not free:
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
// TWO LAYERS, because they answer different questions:
//
//   Runtime  - DebugLog::serialEnabled / serialRawNmea, backed by AppConfig
//              and NVS, so the setting survives a reboot and can be turned
//              on and off from the settings page without reflashing. This
//              is the switch to use day to day.
//   Compile  - AppConst::DEBUG_SERIAL_COMPILED. false removes every call
//              and its arguments outright, for a build that is never meant
//              to log. It also removes the runtime switch along with them,
//              so a stripped build cannot be talked back into logging.
//
// Argument evaluation sits inside the condition either way, so anything
// expensive passed to a log call is skipped when logging is off - only the
// format strings remain in flash, which is what makes the runtime switch
// affordable.
#pragma once

#include <Arduino.h>
#include "AppConstants.h"

namespace DebugLog {

// Defined in DebugLog.cpp, initialised from the AppConst defaults so that
// boot messages work before ConfigManager has read NVS, then overwritten by
// the stored configuration.
extern bool serialEnabled;
extern bool serialRawNmea;

// Called by ConfigManager on load and on every save, so a change made from
// the settings page takes effect immediately rather than at the next boot.
void applyConfig(bool enabled, bool rawNmea);

} // namespace DebugLog

#define LOGF(...)                                                              \
    do {                                                                       \
        if (AppConst::DEBUG_SERIAL_COMPILED && DebugLog::serialEnabled)        \
            Serial.printf(__VA_ARGS__);                                        \
    } while (0)

#define LOGLN(...)                                                             \
    do {                                                                       \
        if (AppConst::DEBUG_SERIAL_COMPILED && DebugLog::serialEnabled)        \
            Serial.println(__VA_ARGS__);                                       \
    } while (0)

// The raw NMEA echo: ~15 sentences a second at 5 Hz, around 1 KB/s on its
// own - more than every other message combined - so it has its own switch.
#define LOG_NMEA(line)                                                         \
    do {                                                                       \
        if (AppConst::DEBUG_SERIAL_COMPILED && DebugLog::serialEnabled &&      \
            DebugLog::serialRawNmea)                                           \
            Serial.println(line);                                              \
    } while (0)
