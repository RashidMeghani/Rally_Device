// Phase 1-3 increment, per the spec's own phased plan (section 24):
// hardware foundation, the ReferenceMap dedup indexer and GeoFencing.txt
// parser (Phase 2), and now the point-geofence manager, race-log
// lifecycle, and the race/recovery state machine (Phase 3) - implemented
// in AppController, promoted out of main.cpp now that there's an actual
// race-runtime state machine to own.
//
// Deliberately NOT in this increment (later phases, per section 24):
// SIM800L SMS, LoRa transport, Give Way/Overtake, Wi-Fi/WebManager, and
// the NeoPixel matrix. See AppController.h for the honest scope note on
// what's approximated pending RouteMatcher (not yet implemented either).
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

#include "../include/PinConfig.h"
#include "../include/AppConstants.h"
#include "ConfigManager.h"
#include "DisplayManager.h"
#include "GpsManager.h"
#include "GeoFenceManager.h"
#include "LogManager.h"
#include "ButtonManager.h"
#include "BatteryManager.h"
#include "AppController.h"
#include "route/ReferenceMapIndexer.h"

ConfigManager configManager;
DisplayManager displayManager;
GpsManager gpsManager;
GeoFenceManager geoFenceManager;
LogManager logManager;
ButtonManager buttonManager;
BatteryManager batteryManager;
AppController appController;

enum class BootState : uint8_t { SPLASH, INIT_ONCE, INIT_HOLD, SD_ERROR, READY };
BootState bootState = BootState::SPLASH;

uint32_t splashStartMs = 0;
uint32_t initHoldStartMs = 0;
uint32_t lastSdRetryMs = 0;
uint8_t sdRetryCount = 0;
bool sdOk = false;

void onRawGpsLine(const char* line) {
    // Live GNSS diagnostics on serial (section 5), and the race-log
    // lifecycle's raw pass-through target (LogManager itself decides
    // whether a line is actually written, based on logging-open state and
    // vehicle speed - see LogManager::onRawLine).
    Serial.println(line);
    logManager.onRawLine(line);
}

void setupSharedSpiBus() {
    SPI.begin(Pins::SPI_SCK, Pins::SPI_MISO, Pins::SPI_MOSI, Pins::SD_CS);
    // LoRa shares this SPI bus but its driver is not initialized yet in
    // this phase - hold its CS deselected (HIGH) so it cannot interfere.
    pinMode(Pins::LORA_CS, OUTPUT);
    digitalWrite(Pins::LORA_CS, HIGH);
}

bool trySdBegin() {
    return SD.begin(Pins::SD_CS, SPI);
}

void runOneTimeInitSteps() {
    // Every real boot step gets its own line on the INIT page (section 9:
    // "Show initialization progress on OLED"), including an explicit
    // presence check for each required SD file - not just the parse
    // result - so a missing file is immediately obvious rather than
    // looking identical to "present but invalid."
    displayManager.addInitLine("SD: OK");
    Serial.println("[Boot] SD initialized");

    if (!SD.exists(AppConst::PATH_GEOFENCE_FILE)) {
        displayManager.addInitLine("GeoFencing: MISSING");
        Serial.println("[Boot] WARNING: GeoFencing.txt not found on SD card");
    } else if (geoFenceManager.load(SD, AppConst::PATH_GEOFENCE_FILE)) {
        char msg[22];
        snprintf(msg, sizeof(msg), "GeoFencing: %u pts", (unsigned)geoFenceManager.count());
        displayManager.addInitLine(msg);
    } else {
        displayManager.addInitLine("GeoFencing: INVALID");
        Serial.println("[Boot] WARNING: GeoFencing.txt present but no valid points parsed");
    }

    if (!SD.exists(AppConst::PATH_REFERENCE_MAP)) {
        displayManager.addInitLine("RefMap: MISSING");
        Serial.println("[Boot] No ReferenceMap.log present yet - route matching unavailable until one is recorded");
    } else {
        RouteIndexResult r = ReferenceMapIndexer::buildIfNeeded(
            SD, AppConst::PATH_REFERENCE_MAP, AppConst::PATH_ROUTE_INDEX_DIR,
            AppConst::PATH_ROUTE_INDEX_HDR, AppConst::PATH_ROUTE_INDEX_CSV,
            AppConst::ROUTE_SEGMENT_LENGTH_M);
        char msg[22];
        snprintf(msg, sizeof(msg), "RefMap: %u seg", (unsigned)r.header.segmentCount);
        displayManager.addInitLine(r.ok ? msg : "RefMap: FAIL");
    }

    gpsManager.begin(configManager.get());
    gpsManager.setRawLineCallback(onRawGpsLine);
    displayManager.addInitLine("GPS: Serial2 up");

    logManager.begin(SD);
    displayManager.addInitLine("Log: ready");

    buttonManager.begin();
    batteryManager.begin();
    appController.begin(gpsManager, geoFenceManager, logManager, displayManager, buttonManager, batteryManager);
    displayManager.addInitLine("Buttons/Batt: ready");

    // LoRa/GSM lines will land here once GsmManager/LoRaTransport exist
    // (Phase 4/5) - intentionally not faked with a placeholder now.
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n[Boot] Desert Race Logging Device - starting");

    pinMode(Pins::BUZZER, OUTPUT);
    digitalWrite(Pins::BUZZER, LOW); // must be OFF after startup initialization

    // Keypad pin setup lives in ButtonManager::begin() (called from
    // runOneTimeInitSteps), not here - GPIO34/35/39 provide no internal
    // pull resistors on the ESP32, so the keypad requires external
    // pull-ups/downs on the PCB (engineering check, section 3).

    if (!displayManager.begin()) {
        Serial.println("[Boot] FATAL: OLED not detected - continuing headless on serial only");
    }
    displayManager.setPage(OledPage::SPLASH);
    splashStartMs = millis();

    configManager.begin();

    setupSharedSpiBus();
    sdOk = trySdBegin();
    if (!sdOk) {
        Serial.println("[Boot] ERROR: SD initialization failed - SD is a hard startup dependency");
    }
}

void loop() {
    switch (bootState) {
        case BootState::SPLASH:
            if (millis() - splashStartMs >= AppConst::SPLASH_DURATION_MS) {
                displayManager.setPage(OledPage::INIT);
                bootState = sdOk ? BootState::INIT_ONCE : BootState::SD_ERROR;
                if (!sdOk) displayManager.addInitLine("SD: FAIL - retrying");
            }
            break;

        case BootState::INIT_ONCE:
            runOneTimeInitSteps();
            initHoldStartMs = millis();
            bootState = BootState::INIT_HOLD;
            break;

        case BootState::INIT_HOLD:
            // Hold the INIT page visible for a fixed duration so the boot
            // steps just printed can actually be read, instead of flipping
            // to the DATA page the instant init finishes.
            if (millis() - initHoldStartMs >= AppConst::INIT_HOLD_MS) {
                bootState = BootState::READY;
                displayManager.setPage(OledPage::DATA);
                Serial.println("[Boot] Entering normal race/status display");
            }
            break;

        case BootState::SD_ERROR:
            // Non-blocking retry: normal race operation must not begin
            // while SD is unavailable (hard dependency, section 6).
            if (millis() - lastSdRetryMs >= 2000) {
                lastSdRetryMs = millis();
                sdRetryCount++;
                sdOk = trySdBegin();
                if (sdOk) {
                    bootState = BootState::INIT_ONCE;
                } else {
                    char msg[22];
                    snprintf(msg, sizeof(msg), "SD: FAIL retry #%u", sdRetryCount);
                    displayManager.updateLastInitLine(msg);
                }
            }
            break;

        case BootState::READY:
            gpsManager.loop();
            appController.loop();
            break;
    }

    displayManager.loop();
}
