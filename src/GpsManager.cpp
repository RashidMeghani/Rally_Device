#include "GpsManager.h"
#include "../include/PinConfig.h"

void GpsManager::begin(const AppConfig& cfg) {
    _currentBaud = cfg.gnssBaud;
    // Everything runs cooperatively on one core, so any long operation in
    // loop() - notably a route correction reading segment files off the SD
    // card - stalls NMEA consumption. The UART's receive interrupt keeps
    // filling this ring buffer regardless, so its size is precisely how
    // much blocking can happen before sentences are lost. The framework
    // default of 256 bytes is only ~120ms at 10Hz; 4 KB gives ~2s, which
    // covers the worst hinted correction with room to spare.
    // MUST be called before begin() - the buffer is allocated there.
    Serial2.setRxBufferSize(4096);
    Serial2.begin(_currentBaud, SERIAL_8N1, Pins::GPS_RX, Pins::GPS_TX);
    Serial.printf("[GPS] Serial2 opened at %u baud (RX=%d, TX=%d)\n",
                  (unsigned)_currentBaud, Pins::GPS_RX, Pins::GPS_TX);
}

void GpsManager::loop() {
    while (Serial2.available()) {
        char c = (char)Serial2.read();
        _tinyGps.encode(c);

        if (c == '\n') {
            _lineBuf[_lineLen] = '\0';
            if (_lineLen > 0 && _rawCallback) _rawCallback(_lineBuf);
            _lineLen = 0;
        } else if (c != '\r') {
            if (_lineLen < sizeof(_lineBuf) - 1) _lineBuf[_lineLen++] = c;
            else _lineLen = 0; // overlong/corrupt line: drop rather than overflow
        }
    }

    // Detect a genuinely new position commit. Done here, once per tick,
    // rather than by each consumer: age() is millis() - lastCommitTime, so
    // millis() - age() is the commit instant itself. A different commit
    // instant means the receiver has delivered a new position since the
    // last tick. At <=10 Hz two commits can never share a millisecond, so
    // equality is a safe "same fix" test.
    if (_tinyGps.location.isValid()) {
        const uint32_t commitMs = millis() - _tinyGps.location.age();
        if (!_haveLocationCommit || commitMs != _lastLocationCommitMs) {
            _lastLocationCommitMs = commitMs;
            _haveLocationCommit = true;
            _fixSeq++;
        }
    }
}

void GpsManager::sendUbxFrame(uint8_t msgClass, uint8_t msgId, const uint8_t* payload, size_t len) {
    uint8_t ckA = 0, ckB = 0;
    auto accum = [&](uint8_t b) { ckA = ckA + b; ckB = ckB + ckA; };

    Serial2.write(0xB5); Serial2.write(0x62);
    Serial2.write(msgClass); accum(msgClass);
    Serial2.write(msgId);    accum(msgId);
    uint8_t lenLo = len & 0xFF, lenHi = (len >> 8) & 0xFF;
    Serial2.write(lenLo); accum(lenLo);
    Serial2.write(lenHi); accum(lenHi);
    for (size_t i = 0; i < len; ++i) { Serial2.write(payload[i]); accum(payload[i]); }
    Serial2.write(ckA);
    Serial2.write(ckB);
}

bool GpsManager::sendUbxCfgRate(uint16_t measRateMs) {
    uint8_t payload[6];
    payload[0] = measRateMs & 0xFF;
    payload[1] = (measRateMs >> 8) & 0xFF;
    payload[2] = 1; payload[3] = 0; // navRate = 1 (every measurement)
    payload[4] = 1; payload[5] = 0; // timeRef = 1 (GPS time)
    sendUbxFrame(0x06, 0x08, payload, sizeof(payload)); // CFG-RATE
    return true;
}

static uint8_t pubxChecksum(const char* body) {
    uint8_t sum = 0;
    for (const char* p = body; *p; ++p) sum ^= (uint8_t)*p;
    return sum;
}

bool GpsManager::sendPubxSentenceRate(uint8_t msgIdMajor, uint8_t msgIdMinor, uint8_t rateOnUart1) {
    char body[64];
    // PUBX,40: msgId,rddc,rus1,rus2,rusb,rspi,reserved
    snprintf(body, sizeof(body), "PUBX,40,%02d%02d,0,%d,0,0,0,0",
             msgIdMajor, msgIdMinor, rateOnUart1);
    char frame[80];
    snprintf(frame, sizeof(frame), "$%s*%02X\r\n", body, pubxChecksum(body));
    Serial2.print(frame);
    return true;
}

bool GpsManager::sendPubxBaud(uint32_t newBaud) {
    char body[48];
    snprintf(body, sizeof(body), "PUBX,41,1,0003,0003,%u,0", (unsigned)newBaud);
    char frame[64];
    snprintf(frame, sizeof(frame), "$%s*%02X\r\n", body, pubxChecksum(body));
    Serial2.print(frame);
    return true;
}

bool GpsManager::applySettings(const AppConfig& cfg) {
    Serial.printf("[GPS] Applying settings: baud=%u rateHz=%u sentences=0x%02X\n",
                  (unsigned)cfg.gnssBaud, cfg.gnssRateHz, cfg.gnssEnabledSentences);

    // 1) Update rate (must happen before/independent of sentence toggling)
    uint8_t rateHz = cfg.gnssRateHz < 1 ? 1 : cfg.gnssRateHz;
    uint16_t measRateMs = (uint16_t)(1000 / rateHz);
    sendUbxCfgRate(measRateMs);
    delay(50); // brief, deliberate: this is a one-shot settings action, not the hot loop

    // 2) Enabled sentence set. NMEA class 0xF0: 00=GGA,04=RMC,05=VTG.
    sendPubxSentenceRate(0xF0, 0x00, (cfg.gnssEnabledSentences & NMEA_GGA) ? 1 : 0);
    sendPubxSentenceRate(0xF0, 0x04, (cfg.gnssEnabledSentences & NMEA_RMC) ? 1 : 0);
    // "GNGTV" per the owner's wording is treated as an alias for VTG until
    // verified against actual receiver output (see open questions).
    sendPubxSentenceRate(0xF0, 0x05,
                         (cfg.gnssEnabledSentences & (NMEA_VTG | NMEA_GNGTV)) ? 1 : 0);
    delay(50);

    // 3) Baud rate last, since it changes how we must reopen the port.
    if (cfg.gnssBaud != _currentBaud) {
        sendPubxBaud(cfg.gnssBaud);
        delay(100);
        Serial2.updateBaudRate(cfg.gnssBaud);
        _currentBaud = cfg.gnssBaud;
        Serial.printf("[GPS] Reopened Serial2 at %u baud\n", (unsigned)_currentBaud);
    }
    return true;
}
