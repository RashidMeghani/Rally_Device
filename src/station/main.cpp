// Checkpoint station firmware — the fixed receiver that sits at each
// geofence point and acknowledges the vehicles that cross it.
//
// Built as a separate PlatformIO environment (`pio run -e checkpoint`)
// sharing LoRaTransport and the wire format with the vehicle firmware, so
// the two can never drift apart: a change to the packet layout breaks both
// builds at once rather than producing a silent mismatch in the desert.
//
// What it does, and nothing more:
//   - listens for CHECKPOINT_EVENT;
//   - replies CHECKPOINT_ACK addressed to that vehicle, echoing the event
//     id and label so the vehicle knows which crossing was received;
//   - records every crossing to SD, and prints it.
//
// It deliberately has no GNSS, no display and no race logic. A station that
// tried to be clever could reject a crossing the vehicle believes in, and
// the vehicle's own log is the authoritative record.
//
// Duplicate events are expected, not exceptional: the vehicle repeats until
// acknowledged, so a lost ACK means the same event id arrives again. The
// station answers every copy (the vehicle may have missed the first reply)
// but records the crossing only once.
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

#include "../../include/PinConfig.h"
#include "../../include/AppConstants.h"
#include "../../include/DebugLog.h"
#include "../ConfigManager.h"
#include "../LoRaTransport.h"

namespace {

ConfigManager configManager;
LoRaTransport lora;
bool sdOk = false;

// Remembers which crossings have already been written, so a repeated event
// is acknowledged again but not recorded twice. Small and fixed: a station
// sees one crossing per vehicle per lap, and a handful of vehicles.
struct SeenEvent { uint16_t deviceId; uint16_t eventId; };
constexpr size_t SEEN_LEN = 32;
SeenEvent seen[SEEN_LEN];
size_t seenCount = 0, seenNext = 0;

bool alreadyRecorded(uint16_t deviceId, uint16_t eventId) {
    for (size_t i = 0; i < seenCount; ++i) {
        if (seen[i].deviceId == deviceId && seen[i].eventId == eventId) return true;
    }
    return false;
}

void rememberRecorded(uint16_t deviceId, uint16_t eventId) {
    seen[seenNext] = {deviceId, eventId};
    seenNext = (seenNext + 1) % SEEN_LEN;
    if (seenCount < SEEN_LEN) seenCount++;
}

// One CSV row per crossing, so the organiser can open it directly:
//   utcDate,utcTime,vehicleId,label,distanceM,rssi,snr
void recordCrossing(uint16_t deviceId, const char* label,
                    uint16_t year, uint8_t month, uint8_t day,
                    uint8_t hh, uint8_t mm, uint8_t ss, uint8_t cs,
                    int32_t distanceCm, int16_t rssi, float snr) {
    if (!sdOk) return;
    File f = SD.open(AppConst::PATH_LORA_LOG, FILE_APPEND);
    if (!f) {
        LOGLN("[Station] ERROR: cannot append to the crossing log");
        return;
    }
    char line[128];
    snprintf(line, sizeof(line),
             "%04u-%02u-%02u,%02u:%02u:%02u.%02u,%u,%s,%.1f,%d,%.1f",
             year, month, day, hh, mm, ss, cs, deviceId, label,
             distanceCm / 100.0, rssi, snr);
    f.println(line);
    f.close();
}

void onMessage(const LoRaMessage& msg) {
    if (msg.type != LoRaMsgType::CHECKPOINT_EVENT) return;
    if (msg.payloadLen < LORA_CP_TIME_LEN) {
        LOGF("[Station] Ignoring a %u-byte checkpoint event - too short to hold a timestamp\n",
             (unsigned)msg.payloadLen);
        return;
    }

    const uint8_t* p = (const uint8_t*)msg.payload;
    const uint16_t year = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
    const uint8_t month = p[2], day = p[3];
    const uint8_t hh = p[4], mm = p[5], ss = p[6], cs = p[7];

    char label[LORA_MAX_PAYLOAD + 1] = {0};
    const size_t labelLen = msg.payloadLen - LORA_CP_TIME_LEN;
    memcpy(label, msg.payload + LORA_CP_TIME_LEN, labelLen);
    label[labelLen] = '\0';

    // Acknowledge FIRST, and acknowledge every copy. The vehicle is
    // retransmitting on a 500 ms cadence and stops only when it hears back;
    // answering a repeat costs one packet, while staying silent because the
    // crossing is already recorded would leave it transmitting until it
    // drove out of range.
    lora.send(LoRaMsgType::CHECKPOINT_ACK, msg.srcDeviceId, msg.sessionId,
              msg.correctedDistanceCm, label);

    if (alreadyRecorded(msg.srcDeviceId, msg.sessionId)) {
        LOGF("[Station] Repeat of event %u from vehicle %u - acknowledged again, not re-recorded\n",
             (unsigned)msg.sessionId, (unsigned)msg.srcDeviceId);
        return;
    }
    rememberRecorded(msg.srcDeviceId, msg.sessionId);

    LOGF("[Station] %u crossed %s at %04u-%02u-%02u %02u:%02u:%02u.%02u UTC, "
         "%.1f m, RSSI %d, SNR %.1f\n",
         (unsigned)msg.srcDeviceId, label, year, month, day, hh, mm, ss, cs,
         msg.correctedDistanceCm / 100.0, msg.rssi, msg.snr);

    recordCrossing(msg.srcDeviceId, label, year, month, day, hh, mm, ss, cs,
                   msg.correctedDistanceCm, msg.rssi, msg.snr);
}

} // namespace

void setup() {
    Serial.begin(115200);
    delay(200);
    LOGLN("\n[Station] Checkpoint station - starting");

    configManager.begin();

    SPI.begin(Pins::SPI_SCK, Pins::SPI_MISO, Pins::SPI_MOSI, Pins::SD_CS);
    sdOk = SD.begin(Pins::SD_CS, SPI);
    if (!sdOk) {
        // A station without a card still acknowledges vehicles; losing the
        // local CSV is bad, leaving cars transmitting into silence is worse.
        LOGLN("[Station] WARNING: no SD card - crossings will be printed but not recorded");
    } else {
        SD.mkdir("/logs");
    }

    if (!lora.begin(configManager.get())) {
        LOGLN("[Station] FATAL: no LoRa radio - this station can do nothing");
    }
    lora.setReceiveCallback(onMessage);

    LOGF("[Station] Listening as device %u\n", (unsigned)lora.deviceId());
}

void loop() {
    lora.loop();
}
