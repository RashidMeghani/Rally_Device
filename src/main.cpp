// Phase 1-3 increment, per the spec's own phased plan (section 24):
// hardware foundation, the ReferenceMap dedup indexer and GeoFencing.txt
// parser (Phase 2), and now the point-geofence manager, race-log
// lifecycle, and the race/recovery state machine (Phase 3) - implemented
// in AppController, promoted out of main.cpp now that there's an actual
// race-runtime state machine to own.
//
// Route correction (RouteMatcher, section 7) is now wired in too, so the
// DATA page's distance is genuinely route-corrected rather than GPS
// dead-reckoned.
//
// Deliberately NOT in this increment (later phases, per section 24):
// SIM800L SMS, LoRa transport, Give Way/Overtake, Wi-Fi/WebManager, and
// the NeoPixel matrix. See AppController.h for what remains outstanding
// (reset-recovery geofence skipping).
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
#include "route/RouteMatcher.h"

ConfigManager configManager;
DisplayManager displayManager;
GpsManager gpsManager;
GeoFenceManager geoFenceManager;
LogManager logManager;
ButtonManager buttonManager;
BatteryManager batteryManager;
RouteMatcher routeMatcher;
AppController appController;

enum class BootState : uint8_t { SPLASH, INIT_ONCE, INIT_HOLD, SD_ERROR, READY, BATTERY_CRITICAL };
BootState bootState = BootState::SPLASH;

uint32_t splashStartMs = 0;
uint32_t initHoldStartMs = 0;
uint32_t lastSdRetryMs = 0;
uint8_t sdRetryCount = 0;
bool sdOk = false;
// Whether the one-time init steps have run. A battery halt can happen
// before they ever do (see the SPLASH gate), so recovery has to know
// whether to resume the race display or still run initialization.
bool initCompleted = false;

void onRawGpsLine(const char* line) {
    // Live GNSS diagnostics on serial (section 5), and the race-log
    // lifecycle's raw pass-through target (LogManager itself decides
    // whether a line is actually written, based on logging-open state and
    // vehicle speed - see LogManager::onRawLine).
    Serial.println(line);
    logManager.onRawLine(line);
}

// Not enough battery to work safely. Two ways in, needing different
// messages: mid-run the files must be closed first; at boot no file has
// been opened yet and the whole point is that none should be.
//
// stopManually() rather than finishAndClose() - the run isn't finished,
// and if the pack recovers a fresh log can open.
void enterBatteryCritical(bool filesMayBeOpen) {
    displayManager.clearInitLines();
    if (filesMayBeOpen) {
        Serial.println("[Battery] CRITICAL - closing files, halting race operations");
        logManager.stopManually();
        displayManager.addInitLine("BATTERY CRITICAL");
        displayManager.addInitLine("Files closed safely");
        displayManager.addInitLine("Operations halted");
    } else {
        Serial.println("[Battery] Too low to start safely - no files will be opened");
        displayManager.addInitLine("BATTERY TOO LOW");
        displayManager.addInitLine("Not starting up.");
        displayManager.addInitLine("No files opened.");
        displayManager.addInitLine("Waiting for charge");
    }
    displayManager.setPage(OledPage::INIT);
    bootState = BootState::BATTERY_CRITICAL;
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

// Boot steps run ONE PER TICK rather than all in a single pass, so each
// line appears on the INIT page as its step actually completes instead of
// the whole list materialising at once (section 9: "Show initialization
// progress on OLED"). The ReferenceMap index build is announced before it
// runs, because on a large recon file it is the one step slow enough that
// the operator needs to see what the device is busy with.
enum class InitStep : uint8_t {
    SD_READY, GEOFENCE, REFMAP_ANNOUNCE, REFMAP_BUILD, GPS, SUBSYSTEMS, DONE
};
InitStep initStep = InitStep::SD_READY;
uint32_t lastInitStepMs = 0;
bool refMapPresent = false;

void runNextInitStep() {
    switch (initStep) {
        case InitStep::SD_READY:
            displayManager.addInitLine("SD: OK");
            Serial.println("[Boot] SD initialized");
            initStep = InitStep::GEOFENCE;
            break;

        case InitStep::GEOFENCE:
            // Explicit presence check, distinct from the parse result, so a
            // missing file never looks identical to a malformed one.
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
            initStep = InitStep::REFMAP_ANNOUNCE;
            break;

        case InitStep::REFMAP_ANNOUNCE:
            refMapPresent = SD.exists(AppConst::PATH_REFERENCE_MAP);
            if (!refMapPresent) {
                displayManager.addInitLine("RefMap: MISSING");
                Serial.println("[Boot] No ReferenceMap.log present yet - route matching unavailable until one is recorded");
                initStep = InitStep::GPS;
            } else {
                displayManager.addInitLine("RefMap: checking...");
                initStep = InitStep::REFMAP_BUILD;
            }
            break;

        case InitStep::REFMAP_BUILD: {
            RouteIndexResult r = ReferenceMapIndexer::buildIfNeeded(
                SD, AppConst::PATH_REFERENCE_MAP, AppConst::PATH_ROUTE_INDEX_DIR,
                AppConst::PATH_ROUTE_INDEX_HDR, AppConst::PATH_ROUTE_INDEX_CSV,
                AppConst::ROUTE_SEGMENT_LENGTH_M);
            char msg[22];
            if (r.ok) {
                // Load the index we just built/validated so route
                // correction is live from the first fix.
                routeMatcher.begin(SD, AppConst::PATH_ROUTE_INDEX_DIR,
                                   AppConst::PATH_ROUTE_INDEX_CSV);
                snprintf(msg, sizeof(msg), "RefMap: %u seg", (unsigned)r.header.segmentCount);
            }
            displayManager.updateLastInitLine(r.ok ? msg : "RefMap: FAIL");
            initStep = InitStep::GPS;
            break;
        }

        case InitStep::GPS:
            gpsManager.begin(configManager.get());
            gpsManager.setRawLineCallback(onRawGpsLine);
            displayManager.addInitLine("GPS: Serial2 up");
            initStep = InitStep::SUBSYSTEMS;
            break;

        case InitStep::SUBSYSTEMS:
            // No INIT-page line of their own - the remaining screen rows are
            // reserved for the LoRa and GSM lines landing in Phase 4/5.
            logManager.begin(SD);
            buttonManager.begin();
            appController.begin(gpsManager, geoFenceManager, logManager, displayManager,
                                buttonManager, batteryManager, configManager, routeMatcher);
            initStep = InitStep::DONE;
            break;

        case InitStep::DONE:
            break;
    }
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n[Boot] Desert Race Logging Device - starting");

    pinMode(Pins::BUZZER, OUTPUT);
    digitalWrite(Pins::BUZZER, LOW); // must be OFF after startup initialization

    // Keypad pin setup lives in ButtonManager::begin() (called from
    // the SUBSYSTEMS init step), not here - GPIO34/35/39 provide no internal
    // pull resistors on the ESP32, so the keypad requires external
    // pull-ups/downs on the PCB (engineering check, section 3).

    if (!displayManager.begin()) {
        Serial.println("[Boot] FATAL: OLED not detected - continuing headless on serial only");
    }
    displayManager.setPage(OledPage::SPLASH);
    splashStartMs = millis();

    configManager.begin();

    // Battery sensing is configured here rather than in the init steps
    // because loop() samples it from the very first tick, independently of
    // the boot/race state machine.
    batteryManager.begin();

    setupSharedSpiBus();
    sdOk = trySdBegin();
    if (!sdOk) {
        Serial.println("[Boot] ERROR: SD initialization failed - SD is a hard startup dependency");
    }
}

void loop() {
    // Battery is sampled at device level, outside the race state machine,
    // so monitoring keeps running even once operations are halted - that
    // is what lets the device notice the pack recovering.
    batteryManager.loop();

    switch (bootState) {
        case BootState::SPLASH:
            if (millis() - splashStartMs >= AppConst::SPLASH_DURATION_MS) {
                displayManager.setPage(OledPage::INIT);
                // Gate on battery BEFORE the SD card is touched. The splash
                // has given the sampler several seconds of readings, so this
                // judges settled voltage rather than the switch-on transient.
                // Skipped entirely when no divider is fitted, so unmonitored
                // hardware still boots.
                if (batteryManager.isPresent() && !batteryManager.hasOperatingCharge()) {
                    enterBatteryCritical(false);
                    break;
                }
                bootState = sdOk ? BootState::INIT_ONCE : BootState::SD_ERROR;
                if (!sdOk) displayManager.addInitLine("SD: FAIL - retrying");
            }
            break;

        case BootState::INIT_ONCE:
            // One step per interval so each line lands on screen visibly,
            // with DisplayManager::loop() refreshing in between.
            if (millis() - lastInitStepMs >= AppConst::INIT_STEP_INTERVAL_MS) {
                lastInitStepMs = millis();
                runNextInitStep();
                if (initStep == InitStep::DONE) {
                    initHoldStartMs = millis();
                    bootState = BootState::INIT_HOLD;
                }
            }
            break;

        case BootState::INIT_HOLD:
            // Hold the INIT page visible for a fixed duration so the boot
            // steps just printed can actually be read, instead of flipping
            // to the DATA page the instant init finishes.
            if (millis() - initHoldStartMs >= AppConst::INIT_HOLD_MS) {
                initCompleted = true;
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
            if (batteryManager.isCritical()) {
                enterBatteryCritical(true);
                break;
            }
            gpsManager.loop();
            appController.loop();
            break;

        case BootState::BATTERY_CRITICAL:
            // Everything race-related stays stopped. Only the display and
            // the battery sampling above keep running, so the halt reason
            // stays on screen and a recovering pack can resume.
            //
            // Resuming requires hasOperatingCharge() (10%), NOT merely
            // clearing the 2% critical flag. That gap is what stops the
            // boot/brownout loop: a pack that sags out recovers a couple
            // of percent with the load off, which would otherwise be
            // enough to restart and immediately die again.
            if (batteryManager.hasOperatingCharge()) {
                if (initCompleted) {
                    Serial.println("[Battery] Recovered - resuming normal operation");
                    displayManager.setPage(OledPage::DATA);
                    bootState = BootState::READY;
                } else {
                    Serial.println("[Battery] Recovered - starting initialization");
                    displayManager.clearInitLines();
                    bootState = sdOk ? BootState::INIT_ONCE : BootState::SD_ERROR;
                    if (!sdOk) displayManager.addInitLine("SD: FAIL - retrying");
                }
            }
            break;
    }

    displayManager.loop();
}
