#include "LoRaTransport.h"
#include <LoRa.h>
#include <cstring>
#include "../include/PinConfig.h"
#include "../include/AppConstants.h"
#include "../include/DebugLog.h"

namespace {

void put16(uint8_t* p, uint16_t v) { p[0] = (uint8_t)(v & 0xFF); p[1] = (uint8_t)(v >> 8); }
uint16_t get16(const uint8_t* p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }

void put32(uint8_t* p, int32_t v) {
    const uint32_t u = (uint32_t)v;   // two's complement, defined conversion
    p[0] = (uint8_t)(u & 0xFF);
    p[1] = (uint8_t)((u >> 8) & 0xFF);
    p[2] = (uint8_t)((u >> 16) & 0xFF);
    p[3] = (uint8_t)((u >> 24) & 0xFF);
}
int32_t get32(const uint8_t* p) {
    const uint32_t u = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                       ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    return (int32_t)u;
}

bool isKnownType(uint8_t t) {
    switch (t) {
        case 1: case 2: case 3: case 4: case 5: case 6: case 7: case 8:
        case 200: case 201:
            return true;
        default:
            return false;
    }
}

} // namespace

uint16_t LoRaTransport::deviceIdFromName(const char* name) {
    if (!name) return 0;
    // Digits anywhere in the name, in order: "RD-07" -> 7, "CAR12" -> 12.
    uint32_t n = 0;
    bool any = false;
    for (const char* p = name; *p; ++p) {
        if (*p >= '0' && *p <= '9') {
            any = true;
            n = n * 10 + (uint32_t)(*p - '0');
            if (n > 0xFFFE) { n = 0xFFFE; break; }  // stay clear of the broadcast id
        }
    }
    if (any && n != 0) return (uint16_t)n;

    // No digits (or "RD-00"): fall back to a hash so two differently named
    // devices still differ on the air. Collisions are possible but far
    // better than every such device answering to the same id.
    uint16_t h = 0x1505;
    for (const char* p = name; *p; ++p) h = (uint16_t)((h * 33u) ^ (uint8_t)*p);
    if (h == 0 || h == LORA_BROADCAST_ID) h = 1;
    return h;
}

size_t LoRaTransport::encode(const LoRaMessage& msg, uint8_t* out, size_t outLen) {
    const uint8_t payloadLen = msg.payloadLen > LORA_MAX_PAYLOAD ? LORA_MAX_PAYLOAD : msg.payloadLen;
    const size_t total = LORA_HEADER_LEN + payloadLen;
    if (!out || outLen < total) return 0;

    out[0] = msg.protoVersion;
    out[1] = (uint8_t)msg.type;
    put16(out + 2,  msg.srcDeviceId);
    put16(out + 4,  msg.dstDeviceId);
    put16(out + 6,  msg.sessionId);
    put16(out + 8,  msg.sequence);
    put32(out + 10, msg.correctedDistanceCm);
    if (payloadLen) memcpy(out + LORA_HEADER_LEN, msg.payload, payloadLen);
    return total;
}

bool LoRaTransport::decode(const uint8_t* in, size_t len, LoRaMessage& out) {
    if (!in || len < LORA_HEADER_LEN) return false;
    if (in[0] != LORA_PROTO_VERSION) return false;
    if (!isKnownType(in[1])) return false;

    const size_t payloadLen = len - LORA_HEADER_LEN;
    if (payloadLen > LORA_MAX_PAYLOAD) return false;

    out.protoVersion = in[0];
    out.type = (LoRaMsgType)in[1];
    out.srcDeviceId = get16(in + 2);
    out.dstDeviceId = get16(in + 4);
    out.sessionId   = get16(in + 6);
    out.sequence    = get16(in + 8);
    out.correctedDistanceCm = get32(in + 10);

    out.payloadLen = (uint8_t)payloadLen;
    if (payloadLen) memcpy(out.payload, in + LORA_HEADER_LEN, payloadLen);
    out.payload[payloadLen] = '\0';
    return true;
}

bool LoRaTransport::begin(const AppConfig& cfg) {
    _deviceId = deviceIdFromName(cfg.deviceId);

    LoRa.setPins(Pins::LORA_CS, Pins::LORA_RST, Pins::LORA_DIO0);

    // LoRa.begin() calls SPI.begin() itself. On ESP32 that is harmless -
    // SPIClass::begin() returns immediately if the bus is already
    // initialised - so the pin mapping main.cpp set up for the SD card is
    // preserved rather than reset to defaults.
    if (!LoRa.begin(cfg.loraFreqHz)) {
        LOGF("[LoRa] RA-02 not responding at %ld Hz - radio disabled, race logging unaffected\n",
             cfg.loraFreqHz);
        _ready = false;
        return false;
    }

    LoRa.setSpreadingFactor(cfg.loraSpreadingFactor);
    LoRa.setSignalBandwidth((long)cfg.loraBandwidthHz);
    LoRa.setCodingRate4(cfg.loraCodingRate4);
    LoRa.setTxPower(cfg.loraTxPowerDbm);   // PA_BOOST, which is how the RA-02 is wired
    LoRa.setSyncWord(AppConst::LORA_SYNC_WORD);
    LoRa.enableCrc();                      // the radio drops corrupt packets for us

    // Explicitly NOT LoRa.onReceive() - see the header comment. Receive is
    // polled so that every SPI access stays on the main thread, out of the
    // way of SD card transactions.
    LoRa.receive();

    _ready = true;
    LOGF("[LoRa] Ready: %ld Hz SF%u BW%lu CR4/%u %ddBm, device id %u\n",
         cfg.loraFreqHz, cfg.loraSpreadingFactor, (unsigned long)cfg.loraBandwidthHz,
         cfg.loraCodingRate4, cfg.loraTxPowerDbm, _deviceId);
    return true;
}

bool LoRaTransport::send(LoRaMsgType type, uint16_t dstDeviceId, uint16_t sessionId,
                          int32_t correctedDistanceCm, const char* payload) {
    if (!_ready) return false;
    if (_qCount >= TX_QUEUE_LEN) {
        _dropped++;
        LOGF("[LoRa] Queue full - dropped a type %u message (%lu dropped in total)\n",
             (unsigned)type, (unsigned long)_dropped);
        return false;
    }

    LoRaMessage& m = _queue[_qTail];
    m = LoRaMessage{};
    m.type = type;
    m.srcDeviceId = _deviceId;
    m.dstDeviceId = dstDeviceId;
    m.sessionId = sessionId;
    m.sequence = ++_sequence;
    m.correctedDistanceCm = correctedDistanceCm;
    if (payload) {
        strncpy(m.payload, payload, LORA_MAX_PAYLOAD);
        m.payload[LORA_MAX_PAYLOAD] = '\0';
        m.payloadLen = (uint8_t)strlen(m.payload);
    }

    _qTail = (_qTail + 1) % TX_QUEUE_LEN;
    _qCount++;
    return true;
}

void LoRaTransport::pumpTx() {
    if (_txState == TxState::SENDING) {
        if (!LoRa.isTransmitting()) {
            _txState = TxState::IDLE;
            _sent++;
            // Back to listening: the radio does not receive while idle
            // after a transmission unless told to.
            LoRa.receive();
            return;
        }
        if (millis() - _txStartedMs > AppConst::LORA_TX_TIMEOUT_MS) {
            // Far beyond any legitimate airtime, so the driver or the radio
            // is wedged. Give up on this packet rather than blocking the
            // queue for the rest of the race.
            LOGF("[LoRa] Transmit did not complete within %lu ms - abandoning packet\n",
                 (unsigned long)AppConst::LORA_TX_TIMEOUT_MS);
            _txState = TxState::IDLE;
            _dropped++;
            LoRa.receive();
        }
        return;
    }

    if (_qCount == 0) return;

    const LoRaMessage& m = _queue[_qHead];
    uint8_t buf[LORA_MAX_PACKET];
    const size_t n = encode(m, buf, sizeof(buf));
    // Dequeue before transmitting: an un-encodable message must leave the
    // queue either way, or it blocks every message behind it forever.
    _qHead = (_qHead + 1) % TX_QUEUE_LEN;
    _qCount--;
    if (n == 0) { _dropped++; return; }

    if (!LoRa.beginPacket()) {
        // Radio busy - put nothing back, just count it; the next event will
        // queue afresh. Retrying here would need a second state.
        _dropped++;
        return;
    }
    LoRa.write(buf, n);
    // true = asynchronous. Returns as soon as the packet is handed to the
    // radio; loop() watches for completion. See the header comment.
    LoRa.endPacket(true);

    _txState = TxState::SENDING;
    _txStartedMs = millis();
}

void LoRaTransport::pollRx() {
    // parsePacket() puts the radio into receive mode, so it must never be
    // called while a transmission is in flight - it would abort it.
    if (_txState != TxState::IDLE) return;

    const int size = LoRa.parsePacket();
    if (size <= 0) return;

    uint8_t buf[LORA_MAX_PACKET];
    size_t n = 0;
    while (LoRa.available() && n < sizeof(buf)) buf[n++] = (uint8_t)LoRa.read();
    // Drain anything oversized so the next parsePacket() starts clean.
    while (LoRa.available()) (void)LoRa.read();

    LoRaMessage msg;
    if (!decode(buf, n, msg)) {
        _dropped++;
        LOGF("[LoRa] Dropped an undecodable %u-byte packet (RSSI %d)\n",
             (unsigned)n, LoRa.packetRssi());
        return;
    }

    // Our own transmissions should never arrive, but a repeater or a
    // misconfigured twin would make them; ignoring them here keeps every
    // consumer from having to remember.
    if (msg.srcDeviceId == _deviceId) return;

    // Not addressed to us and not a broadcast.
    if (msg.dstDeviceId != LORA_BROADCAST_ID && msg.dstDeviceId != _deviceId) return;

    msg.rssi = (int16_t)LoRa.packetRssi();
    msg.snr = LoRa.packetSnr();
    _lastRssi = msg.rssi;
    _received++;

    if (_onReceive) _onReceive(msg);
}

void LoRaTransport::loop() {
    if (!_ready) return;
    pumpTx();
    pollRx();
}
