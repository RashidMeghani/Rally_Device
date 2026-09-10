// Raw-NMEA race log lifecycle (spec section 10).
//
// Ownership split with AppController: LogManager owns WHEN to auto-close
// due to the vehicle sitting still (that's purely a function of speed and
// time, intrinsic to logging itself), while AppController owns WHEN to
// open a new file - the spec is explicit that "only the start/recovery/
// resume rules create race log files" (section 10/11), so opening is a
// deliberate external decision, not something LogManager decides on its
// own.
//
// Buffering: raw lines are accumulated in a small RAM ring buffer and
// flushed to SD either when the buffer is nearly full or every
// FLUSH_INTERVAL_MS, whichever comes first - bounds both worst-case
// power-loss data loss and SD write frequency, independent of the
// configured per-line arrival rate.
#pragma once

#include <Arduino.h>
#include <FS.h>
#include <cstdint>

class LogManager {
public:
    void begin(fs::FS& fs);

    // Opens a new date/time-stamped log file. Safe to call while already
    // open (closes the old one first, though AppController should not
    // normally do this - see class comment).
    //
    // Filename format: "DD-MM-YYYY HH.MM.SS.CC.log" (local time - the
    // caller passes already-offset values; see TimeUtil/AppConfig).
    // Note the time separators are dots, not colons: ':' is a reserved
    // character on FAT and cannot appear in a filename on the SD card.
    //
    // If timeValid is false (no GNSS fix yet at the moment a log must
    // open), falls back to a boot-relative filename rather than a
    // retroactive rename once time becomes available (FAT rename-while-
    // open is its own failure mode) - flagged as a recommendation in
    // ARCHITECTURE.md section 5.5, pending owner confirmation.
    void startNewLog(bool timeValid, uint16_t year, uint8_t month, uint8_t day,
                      uint8_t hour, uint8_t minute, uint8_t second, uint8_t centisecond);

    // Final geofence: flush+close permanently. No further startNewLog()
    // calls should occur for this run after this (AppController's job to
    // enforce via race stage).
    void finishAndClose();

    // Manual stop (e.g. Key4 long-press toggle): flush+close, but does
    // NOT mark the run "finished" - unlike finishAndClose(), a later
    // startNewLog() can still open a fresh file. Use this for any
    // stop that isn't the actual last-geofence race finish.
    void stopManually();

    // Current speed, called every tick regardless of whether a line just
    // arrived - drives the "stopped 20 minutes -> auto-close" timer even
    // during gaps in raw NMEA arrival.
    void updateSpeed(float speedKmh);

    // Raw line hand-off (same shape as GpsManager's RawLineCallback).
    // No-ops if not currently open, or if the vehicle isn't above the
    // logging-moving threshold (section 10: "while the vehicle is moving
    // above 2 km/h").
    void onRawLine(const char* line);

    // Time-based buffer flush independent of new lines arriving. Call
    // every main loop tick.
    void loop();

    // File is open. Stays true while the vehicle is stopped, right up
    // until the 20-minute stop timeout closes it - this is the right
    // question for lifecycle decisions ("should I open a new file?").
    bool isLogging() const { return _open; }

    // Lines are actually being written right now: file open AND moving
    // above the logging threshold. This is the right question for the
    // OLED 'L' indicator - spec section 10 says to remove 'L' when
    // logging is "paused/stopped", and dropping below 2 km/h is exactly
    // a pause (writing halts immediately; the file only closes after the
    // 20-minute timeout).
    bool isActivelyWriting() const { return _open && _currentSpeedKmh > MOVING_MIN_KMH; }

    bool isFinished() const { return _finished; }
    const char* currentPath() const { return _currentPath; }

private:
    static constexpr size_t RING_BUF_LEN = 2048;
    static constexpr uint32_t FLUSH_INTERVAL_MS = 2000;
    static constexpr float MOVING_MIN_KMH = 2.0f;
    static constexpr uint32_t STOP_TIMEOUT_MS = 20UL * 60UL * 1000UL;

    fs::FS* _fs = nullptr;
    File _file;
    bool _open = false;
    bool _finished = false;
    char _currentPath[48] = {0};

    char _ringBuf[RING_BUF_LEN];
    size_t _ringLen = 0;
    uint32_t _lastFlushMs = 0;

    float _currentSpeedKmh = 0;
    uint32_t _stopStartMs = 0; // 0 = not currently in a stopped period

    void appendLine(const char* line);
    void flush();
    void closeFile();
};
