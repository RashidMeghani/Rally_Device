#include "LogManager.h"
#include "../include/AppConstants.h"
#include <cstring>

void LogManager::begin(fs::FS& fs) {
    _fs = &fs;
    _fs->mkdir(AppConst::PATH_RACE_LOG_DIR);
}

void LogManager::startNewLog(bool timeValid, uint16_t year, uint8_t month, uint8_t day,
                              uint8_t hour, uint8_t minute, uint8_t second, uint8_t centisecond) {
    if (_open) closeFile(); // defensive: AppController should not normally do this - see header

    if (timeValid) {
        snprintf(_currentPath, sizeof(_currentPath), "%s/%02u-%02u-%04u %02u.%02u.%02u.%02u.log",
                 AppConst::PATH_RACE_LOG_DIR, day, month, year, hour, minute, second, centisecond);
    } else {
        snprintf(_currentPath, sizeof(_currentPath), "%s/NoTime-%lu.log",
                 AppConst::PATH_RACE_LOG_DIR, (unsigned long)millis());
    }

    _file = _fs->open(_currentPath, FILE_WRITE);
    _open = (bool)_file;
    _ringLen = 0;
    _stopStartMs = 0;
    _lastFlushMs = millis();

    if (_open) {
        Serial.printf("[Log] Opened %s\n", _currentPath);
    } else {
        Serial.printf("[Log] ERROR: failed to open %s - if this filename looks correct, the SD "
                      "long-filename (LFN) support may be disabled in this build\n", _currentPath);
    }
}

void LogManager::finishAndClose() {
    if (_open) closeFile();
    _finished = true;
}

void LogManager::stopManually() {
    if (_open) closeFile();
}

void LogManager::updateSpeed(float speedKmh) {
    _currentSpeedKmh = speedKmh;
    if (!_open) { _stopStartMs = 0; return; }

    if (speedKmh <= MOVING_MIN_KMH) {
        if (_stopStartMs == 0) {
            _stopStartMs = millis();
        } else if (millis() - _stopStartMs >= STOP_TIMEOUT_MS) {
            Serial.println("[Log] Vehicle stopped 20 min - closing log (resume will open a new file)");
            closeFile();
        }
    } else {
        _stopStartMs = 0;
    }
}

void LogManager::onRawLine(const char* line) {
    if (!_open) return;
    if (_currentSpeedKmh <= MOVING_MIN_KMH) return; // "while the vehicle is moving above 2 km/h"
    appendLine(line);
}

void LogManager::appendLine(const char* line) {
    size_t len = strlen(line);
    if (len == 0) return;
    if (len + 1 > RING_BUF_LEN) return; // pathological oversized line: nothing sane to do, drop it

    if (_ringLen + len + 1 > RING_BUF_LEN) flush();

    memcpy(_ringBuf + _ringLen, line, len);
    _ringLen += len;
    _ringBuf[_ringLen++] = '\n';
}

void LogManager::flush() {
    if (!_open || _ringLen == 0) return;
    _file.write(reinterpret_cast<const uint8_t*>(_ringBuf), _ringLen);
    _file.flush();
    _ringLen = 0;
    _lastFlushMs = millis();
}

void LogManager::loop() {
    if (_open && _ringLen > 0 && millis() - _lastFlushMs >= FLUSH_INTERVAL_MS) {
        flush();
    }
}

void LogManager::closeFile() {
    flush();
    _file.close();
    _open = false;
}
