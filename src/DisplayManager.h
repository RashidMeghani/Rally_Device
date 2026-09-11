// Non-blocking OLED view layer for the 128x64 SH1106 (Rev 3, section 23/28).
//
// Four pages only, matching the confirmed project scope:
//   SPLASH   - "RASE" centered, large font, shown for ~3 s at boot.
//   INIT     - rolling log of module initialization results.
//   DATA     - normal race display; the ten mandatory fields (section 29).
//   SETTINGS - Wi-Fi AP mode: <device-id>.local + IP + "setting page".
//
// Non-blocking contract: callers push new field values into the model via
// updateDataModel()/setInitLine()/setSettingsInfo() as often as they like
// (cheap struct copy, no I2C traffic). Actual redraw + I2C flush only
// happens inside loop(), and only every OLED_REFRESH_INTERVAL_MS - this
// decouples the OLED (I2C transfer of a 1 KB frame buffer takes tens of ms)
// from the up-to-10 Hz GNSS fix rate so display work never becomes the
// bottleneck for GNSS parsing.
//
// ---------------------------------------------------------------------
// DATA page pixel layout (128x64), see ARCHITECTURE.md for the rationale
// ---------------------------------------------------------------------
//  y=0..8   (size1) "A:<ahead dist>"              x=0     | "ID:<ahead id>"        x=74
//  y=9..17  (size1) "T:<HH:MM:SS.cc>"              x=0   (full width)
//  y=18..33 (size2) "<speed>"  x=0  + (size1) "km/h" x=40,y=26 | "GF:<label>" x=74,y=18 (size1)
//                                                             | "D:<dist>m"  x=74,y=26 (size1)
//  y=36..44 (size1) "Dist:<corrected>m"            x=0   (left-aligned, full width)
//  y=46..54 (size1) "Sats:<n>"                     x=0     | "Acc:<m>m"            x=74
//  y=55..63 battery icon x=0..13 + "<pct>% (<V>V)" x=16   | "O" x=113, "L" x=122 (right-aligned)
// No horizontal divider lines - fields are separated by vertical spacing only.
// ---------------------------------------------------------------------
#pragma once

#include <Adafruit_SH110X.h>
#include <cstdint>

enum class OledPage : uint8_t { SPLASH, INIT, DATA, SETTINGS };

// All ten mandatory DATA-page fields (Rev 3, section 29), numbered as in
// the spec table. `*Valid` flags gate placeholder-vs-value rendering -
// "show a consistent placeholder rather than stale data" when a value is
// unavailable.
struct RaceDataModel {
    // 1. Distance of ahead device (Give Way peer)
    bool aheadDistValid = false;
    float aheadDistanceFt = 0;
    // 6. Ahead device ID
    bool aheadIdValid = false;
    char aheadDeviceId[8] = {0};
    // 2. Geofence crossing time last picked up (from GNSS)
    bool crossingTimeValid = false;
    uint8_t xh = 0, xm = 0, xs = 0, xcs = 0; // hour, minute, second, centisecond
    // 3. Speed
    bool speedValid = false;
    float speedKmh = 0;
    // 4. Logging symbol 'L' - lines are actively being written right now
    bool loggingActive = false;
    // Companion to Field 4: a race log FILE is open. Shown as 'O' next to
    // 'L' so the two states are distinguishable - a file stays open while
    // the vehicle is stopped (writing paused) until the 20-minute timeout,
    // so "open" and "writing" are genuinely different things to see.
    bool logFileOpen = false;
    // 5. Covered/corrected race distance
    bool distanceValid = false;
    float correctedDistanceM = 0;
    // 7. Geofence label (next/relevant)
    bool geofenceLabelValid = false;
    char geofenceLabel[8] = {0};
    // 8. Distance from geofence
    bool geofenceDistValid = false;
    float geofenceDistanceM = 0;
    // 9. Number of satellites
    bool satsValid = false;
    uint8_t satCount = 0;
    // 10. Accuracy in meters
    bool accuracyValid = false;
    float accuracyM = 0;

    // Bonus (not one of the ten mandatory fields, but explicitly requested
    // by spec section 12: "battery voltage/status when implemented").
    // Rendered in the spare row below Field 9/10 when there's room.
    bool batteryValid = false;
    float batteryVoltage = 0;
    uint8_t batteryPercent = 0;
    bool batteryLow = false;
};

class DisplayManager {
public:
    bool begin(); // Wire.begin(SDA,SCL) + Adafruit_SH1106G::begin() at 0x3C

    void setPage(OledPage p);
    OledPage page() const { return _page; }

    // Page 2 (INIT): appends a short status line (e.g. "SD: OK").
    void addInitLine(const char* msg);

    // Page 2 (INIT): replaces the most recent line in place instead of
    // appending - used for retry counters (e.g. SD-error retry) so the
    // page doesn't scroll-spam once per retry attempt.
    void updateLastInitLine(const char* msg);

    // Page 2 (INIT): wipes the accumulated lines, so the page can be
    // reused to show a different status (e.g. the battery-critical halt)
    // without the boot log still underneath it.
    void clearInitLines();

    // Page 4 (SETTINGS): mDNS host and IP shown prominently.
    void setSettingsInfo(const char* hostname, const char* ip);

    // Page 3 (DATA): cheap struct copy, no I2C traffic here.
    void updateDataModel(const RaceDataModel& model) { _model = model; }

    // Call every main loop iteration. Internally throttled; never blocks
    // for longer than a single I2C frame flush every REFRESH_INTERVAL_MS.
    void loop();

private:
    Adafruit_SH1106G _display{128, 64, &Wire, -1};
    OledPage _page = OledPage::SPLASH;
    RaceDataModel _model;
    uint32_t _lastRefreshMs = 0;

    static constexpr uint32_t REFRESH_INTERVAL_MS = 200; // ~5 Hz, see class comment
    // 8 lines * 8px (size1), no title, fills the 64px-tall screen exactly -
    // every boot step gets its own line rather than scrolling early ones
    // away before they can be read (see AppConst::INIT_HOLD_MS in main.cpp
    // for how long the page is held visible once all steps are in).
    static constexpr uint8_t MAX_INIT_LINES = 8;
    char _initLines[MAX_INIT_LINES][22] = {{0}};
    uint8_t _initLineCount = 0;

    char _apHost[32] = {0};
    char _apIp[16] = {0};

    void drawSplash();
    void drawInit();
    void drawData();
    void drawSettings();
};
