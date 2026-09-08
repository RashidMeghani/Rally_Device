// Phase 1 (+ start of Phase 2) increment, per the spec's own phased plan
// (section 24): hardware foundation (pins, buttons, OLED all four pages,
// SD/SPI arbitration, GNSS at configured baud) plus the ReferenceMap
// dedup indexer and GeoFencing.txt parser groundwork for Phase 2/3.
//
// Deliberately NOT in this increment (later phases, per section 24):
// SIM800L SMS, LoRa transport, Give Way/Overtake, Wi-Fi/WebManager,
// battery ADC, NeoPixel matrix, and the race logging/geofence-crossing
// state machine itself. AppController is intentionally kept as the small
// boot state machine below rather than a separate class file until the
// race-runtime state machine (section 10) is added in the next phase -
// factoring it out now would be premature.
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

#include "../include/PinConfig.h"
#include "../include/AppConstants.h"
#include "ConfigManager.h"
#include "DisplayManager.h"
#include "GpsManager.h"
#include "GeoFenceManager.h"
#include "route/ReferenceMapIndexer.h"

ConfigManager configManager;
DisplayManager displayManager;
GpsManager gpsManager;
GeoFenceManager geoFenceManager;

enum class BootState : uint8_t { SPLASH, INIT_ONCE, SD_ERROR, READY };
BootState bootState = BootState::SPLASH;

uint32_t splashStartMs = 0;
uint32_t lastSdRetryMs = 0;
uint8_t sdRetryCount = 0;
bool sdOk = false;

void onRawGpsLine(const char* line) {
    // Raw pass-through target: ReferenceMap capture / race log will attach
    // here in Phase 2/3. For this increment we only mirror to Serial so the
    // live GNSS diagnostics requirement (section 5) is already satisfied.
    Serial.println(line);
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
    displayManager.addInitLine("SD: OK");
    Serial.println("[Boot] SD initialized");

    if (geoFenceManager.load(SD, AppConst::PATH_GEOFENCE_FILE)) {
        char msg[22];
        snprintf(msg, sizeof(msg), "GeoFence: %u pts", (unsigned)geoFenceManager.count());
        displayManager.addInitLine(msg);
    } else {
        displayManager.addInitLine("GeoFence: FAIL");
        Serial.println("[Boot] WARNING: GeoFencing.txt missing or invalid - no valid start/finish points loaded");
    }

    if (SD.exists(AppConst::PATH_REFERENCE_MAP)) {
        RouteIndexResult r = ReferenceMapIndexer::buildIfNeeded(
            SD, AppConst::PATH_REFERENCE_MAP, AppConst::PATH_ROUTE_INDEX_DIR,
            AppConst::PATH_ROUTE_INDEX_HDR, AppConst::PATH_ROUTE_INDEX_CSV,
            AppConst::ROUTE_SEGMENT_LENGTH_M);
        char msg[22];
        snprintf(msg, sizeof(msg), "Route: %u seg", (unsigned)r.header.segmentCount);
        displayManager.addInitLine(r.ok ? msg : "Route: FAIL");
    } else {
        displayManager.addInitLine("Route: no map");
        Serial.println("[Boot] No ReferenceMap.log present yet - route matching unavailable until one is recorded");
    }

    gpsManager.begin(configManager.get());
    gpsManager.setRawLineCallback(onRawGpsLine);
    displayManager.addInitLine("GPS: Serial2 up");
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n[Boot] Desert Race Logging Device - starting");

    pinMode(Pins::BUZZER, OUTPUT);
    digitalWrite(Pins::BUZZER, LOW); // must be OFF after startup initialization

    // GPIO34/35/39 provide no internal pull resistors on the ESP32; the
    // keypad requires external pull-ups/downs on the PCB per the wiring
    // convention chosen (engineering check, section 3).
    pinMode(Pins::KEY1, INPUT);
    pinMode(Pins::KEY2, INPUT);
    pinMode(Pins::KEY4, INPUT);

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
            bootState = BootState::READY;
            displayManager.setPage(OledPage::DATA);
            Serial.println("[Boot] Entering normal race/status display");
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

        case BootState::READY: {
            gpsManager.loop();

            RaceDataModel model; // fields not yet owned by a manager in this
                                  // phase (ahead-device/Give-Way, geofence
                                  // label/distance, logging, corrected
                                  // distance) stay at their default
                                  // "unavailable" state and render as
                                  // placeholders until Phase 3/6 wire them.
            if (gpsManager.hasFix()) {
                model.speedValid = true;
                model.speedKmh = gpsManager.speedKmh();
                model.satsValid = true;
                model.satCount = gpsManager.satellites();
                model.accuracyValid = true;
                model.accuracyM = gpsManager.accuracyMeters();
            }
            // model.crossingTimeValid stays false here: Field 2 is the
            // GNSS-timestamped geofence *crossing* time, captured by
            // GeofenceManager on an actual crossing (Phase 3) - not the
            // live clock, so it correctly reads as "unavailable" until
            // the first crossing of this run.
            displayManager.updateDataModel(model);
            break;
        }
    }

    displayManager.loop();
}
