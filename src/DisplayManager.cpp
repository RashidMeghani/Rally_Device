#include "DisplayManager.h"
#include "../include/PinConfig.h"
#include <Wire.h>
#include <cstdio>
#include <cstring>

namespace {
// Draws `text` at (x,y) but never more than maxChars characters, so a long
// device ID or label cannot overwrite an adjacent field's zone.
void drawClipped(Adafruit_SH1106G& d, int16_t x, int16_t y, const char* text, size_t maxChars) {
    char buf[24];
    size_t n = strnlen(text, sizeof(buf) - 1);
    if (n > maxChars) n = maxChars;
    if (n > sizeof(buf) - 1) n = sizeof(buf) - 1;
    memcpy(buf, text, n);
    buf[n] = '\0';
    d.setCursor(x, y);
    d.print(buf);
}
} // namespace

bool DisplayManager::begin() {
    Wire.begin(Pins::OLED_SDA, Pins::OLED_SCL);
    if (!_display.begin(Pins::OLED_I2C_ADDR, true)) {
        Serial.println("[Display] ERROR: SH1106 not found at 0x3C");
        return false;
    }
    _display.setTextColor(SH110X_WHITE);
    _display.clearDisplay();
    _display.display();
    return true;
}

void DisplayManager::setPage(OledPage p) {
    _page = p;
    _lastRefreshMs = 0; // force an immediate redraw on the next loop()
}

void DisplayManager::addInitLine(const char* msg) {
    if (_initLineCount < MAX_INIT_LINES) {
        strncpy(_initLines[_initLineCount], msg, sizeof(_initLines[0]) - 1);
        _initLineCount++;
    } else {
        // scroll: drop oldest
        for (uint8_t i = 1; i < MAX_INIT_LINES; ++i) {
            strcpy(_initLines[i - 1], _initLines[i]);
        }
        strncpy(_initLines[MAX_INIT_LINES - 1], msg, sizeof(_initLines[0]) - 1);
    }
}

void DisplayManager::updateLastInitLine(const char* msg) {
    if (_initLineCount == 0) { addInitLine(msg); return; }
    strncpy(_initLines[_initLineCount - 1], msg, sizeof(_initLines[0]) - 1);
}

void DisplayManager::clearInitLines() {
    _initLineCount = 0;
    _initLines[0][0] = '\0';
}

void DisplayManager::setSettingsInfo(const char* hostname, const char* ip) {
    strncpy(_apHost, hostname, sizeof(_apHost) - 1);
    strncpy(_apIp, ip, sizeof(_apIp) - 1);
}

void DisplayManager::loop() {
    uint32_t now = millis();
    if (now - _lastRefreshMs < REFRESH_INTERVAL_MS) return;
    _lastRefreshMs = now;

    _display.clearDisplay();
    switch (_page) {
        case OledPage::SPLASH:   drawSplash();   break;
        case OledPage::INIT:     drawInit();     break;
        case OledPage::DATA:     drawData();     break;
        case OledPage::SETTINGS: drawSettings(); break;
    }
    _display.display();
}

void DisplayManager::drawSplash() {
    _display.setTextSize(3);
    // "RASE": 4 chars * 18px/char (size3) = 72px wide, 24px tall.
    _display.setCursor((128 - 72) / 2, (64 - 24) / 2);
    _display.print("RASE");
}

void DisplayManager::drawInit() {
    // No separate title: 8 lines * 8px fills the 64px screen exactly, so
    // every real boot step gets its own line instead of some scrolling
    // off before the page is even readable.
    _display.setTextSize(1);
    for (uint8_t i = 0; i < _initLineCount; ++i) {
        _display.setCursor(0, i * 8);
        _display.print(_initLines[i]);
    }
}

void DisplayManager::drawSettings() {
    _display.setTextSize(1);
    // "Prominent" on this display's default font is approximated by a
    // pseudo-bold double-draw (x, x+1) rather than a larger font size,
    // since size2 would overflow a typical "<device-id>.local" string.
    for (int8_t dx = 0; dx <= 1; ++dx) {
        _display.setCursor(2 + dx, 14);
        _display.print(_apHost);
        _display.setCursor(2 + dx, 30);
        _display.print(_apIp);
    }
    _display.setCursor(2, 50);
    _display.print("setting page");
}

void DisplayManager::drawData() {
    const RaceDataModel& m = _model;
    char buf[24];

    _display.setTextSize(1);

    // Row 1 (y=0): Field 1 (ahead distance) | Field 6 (ahead device ID)
    if (m.aheadDistValid) snprintf(buf, sizeof(buf), "A:%.0fft", m.aheadDistanceFt);
    else strcpy(buf, "A:--");
    drawClipped(_display, 0, 0, buf, 12);

    if (m.aheadIdValid && m.aheadDeviceId[0]) snprintf(buf, sizeof(buf), "ID:%s", m.aheadDeviceId);
    else strcpy(buf, "ID:--");
    drawClipped(_display, 74, 0, buf, 9);

    // Row 2 (y=9): Field 2 (last geofence crossing time), full width
    if (m.crossingTimeValid) {
        snprintf(buf, sizeof(buf), "T:%02u:%02u:%02u.%02u", m.xh, m.xm, m.xs, m.xcs);
    } else {
        strcpy(buf, "T:--:--:--");
    }
    drawClipped(_display, 0, 9, buf, 21);

    // Row 3 (y=18): Field 3 (speed, large) | Field 7/8 (geofence label + distance)
    _display.setTextSize(2);
    if (m.speedValid) snprintf(buf, sizeof(buf), "%.0f", m.speedKmh);
    else strcpy(buf, "--");
    drawClipped(_display, 0, 18, buf, 6);
    _display.setTextSize(1);
    _display.setCursor(40, 26);
    _display.print("km/h");

    if (m.geofenceLabelValid && m.geofenceLabel[0]) snprintf(buf, sizeof(buf), "GF:%s", m.geofenceLabel);
    else strcpy(buf, "GF:--");
    drawClipped(_display, 74, 18, buf, 9);

    if (m.geofenceDistValid) snprintf(buf, sizeof(buf), "D:%.0fm", m.geofenceDistanceM);
    else strcpy(buf, "D:--");
    drawClipped(_display, 74, 26, buf, 9);

    // Row 4 (y=36): Field 5 (corrected distance), left-aligned
    if (m.distanceValid) snprintf(buf, sizeof(buf), "Dist:%.0fm", m.correctedDistanceM);
    else strcpy(buf, "Dist:--");
    drawClipped(_display, 0, 36, buf, 21);

    // Row 5 (y=46): Field 9 (satellite count) | Field 10 (accuracy)
    if (m.satsValid) snprintf(buf, sizeof(buf), "Sats:%u", m.satCount);
    else strcpy(buf, "Sats:--");
    drawClipped(_display, 0, 46, buf, 12);

    if (m.accuracyValid) snprintf(buf, sizeof(buf), "Acc:%.0fm", m.accuracyM);
    else strcpy(buf, "Acc:--");
    drawClipped(_display, 74, 46, buf, 9);

    // Row 6 (y=55): battery on the left (bonus, not one of the ten
    // mandatory fields) | 'O' + Field 4 'L' bottom-right, directly below
    // the accuracy field. Right-aligned: one size-1 glyph is 6px wide, so
    // the last column starts at 128-6=122, with 'O' sitting just left of
    // it. 'O' = a log file is open; 'L' = lines are being written right
    // now. Both can show ("OL"), or just 'O' while stopped with the file
    // still open.
    if (m.batteryValid) {
        // Battery icon (12x7 body + a 2x3 terminal nub) with the interior
        // filled proportionally to charge: reads at a glance, and costs
        // 14px where a "Bat:" label would cost 24 - which matters on a row
        // that still has to fit "LOW 100% (8.4V)" before the O/L markers.
        _display.drawRect(0, 55, 12, 7, SH110X_WHITE);
        _display.fillRect(12, 57, 2, 3, SH110X_WHITE);
        int16_t fillWidth = (int16_t)((m.batteryPercent / 100.0f) * 10.0f + 0.5f);
        if (fillWidth > 0) _display.fillRect(1, 56, fillWidth, 5, SH110X_WHITE);

        // "40%", or "LOW 40%" below the low threshold. Voltage is not
        // shown here (owner preference) but is still measured, and is
        // printed on serial for calibration and critical-battery events.
        snprintf(buf, sizeof(buf), "%s%u%%", m.batteryLow ? "LOW " : "", m.batteryPercent);
        drawClipped(_display, 20, 55, buf, 15);
    }
    if (m.logFileOpen) drawClipped(_display, 113, 55, "O", 1);
    if (m.loggingActive) drawClipped(_display, 122, 55, "L", 1);
}
