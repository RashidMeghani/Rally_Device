// RA-02 (SX1278) packet transport at 433 MHz. Owns the radio; knows nothing
// about races, checkpoints or Give Way sessions - it moves typed messages
// and reports what arrives.
//
// -----------------------------------------------------------------------
// Two design decisions that are not obvious, and are not negotiable
// -----------------------------------------------------------------------
//
// 1. RECEIVE BY POLLING, NOT BY INTERRUPT.
//
//    The driver offers LoRa.onReceive(), which fires from the DIO0
//    interrupt and performs SPI reads inside the callback. This device
//    shares one SPI bus between the radio and the SD card, and the race log
//    is flushed from loop(). An ISR that starts its own SPI transaction
//    while a card write is mid-transaction corrupts one or both - and it
//    would do so rarely and unreproducibly, which is the worst kind of
//    fault to meet in a desert.
//
//    Polling parsePacket() from loop() keeps every SPI access on one
//    thread, in a defined order. The cost is latency bounded by the loop
//    period, which at a few milliseconds is nothing next to the ~165 ms a
//    packet spends in the air.
//
// 2. TRANSMIT ASYNCHRONOUSLY.
//
//    LoRa.endPacket() blocks until the packet has left the antenna. At SF9
//    that is ~165 ms; at SF12 it would be over a second. Blocking loop()
//    for that long stalls GNSS consumption, the display, the buttons and
//    the SD flush timer - the geofence detector would miss fixes at exactly
//    the moment a checkpoint event is being sent, which is when it is most
//    needed. endPacket(true) returns immediately and the state machine in
//    loop() waits for the radio to report done.
//
// Nothing here ever blocks longer than a register read.
#pragma once

#include <Arduino.h>
#include <functional>
#include "ConfigManager.h"

// Wire protocol version. Bumped only on an incompatible header change; a
// receiver drops anything it does not recognise rather than guessing.
constexpr uint8_t LORA_PROTO_VERSION = 1;

constexpr uint16_t LORA_BROADCAST_ID = 0xFFFF;

enum class LoRaMsgType : uint8_t {
    CHECKPOINT_EVENT = 1,
    OT_REQ           = 2,   // Give Way: requester asks to pass
    OT_DEV_ACK       = 3,   // ahead device acknowledges receipt
    OT_USER_ACK      = 4,   // ahead DRIVER acknowledges (Key4 tap)
    OT_USER_ACK_ACK  = 5,   // requester confirms it saw the driver ack
    OT_CANCEL        = 6,
    BUSY             = 7,   // already in a session with someone else
    SESSION_END      = 8,
    EMERGENCY        = 200, // reserved (spec section 15)
    HELP             = 201, // reserved
};

// Header is serialised field by field in little-endian order rather than
// memcpy'd: a struct's padding and byte order are compiler and target
// properties, and a wire format must not inherit them.
constexpr size_t LORA_HEADER_LEN = 14;
constexpr size_t LORA_MAX_PAYLOAD = 24;   // checkpoint label and spare
constexpr size_t LORA_MAX_PACKET = LORA_HEADER_LEN + LORA_MAX_PAYLOAD;

struct LoRaMessage {
    uint8_t protoVersion = LORA_PROTO_VERSION;
    LoRaMsgType type = LoRaMsgType::CHECKPOINT_EVENT;
    uint16_t srcDeviceId = 0;
    uint16_t dstDeviceId = LORA_BROADCAST_ID;
    uint16_t sessionId = 0;
    uint16_t sequence = 0;
    // Centimetres, signed: integer on the air, so both ends agree on the
    // value bit for bit. A float would make the relative-distance
    // comparison that drives Give Way depend on rounding at each end.
    int32_t correctedDistanceCm = 0;

    char payload[LORA_MAX_PAYLOAD + 1] = {0};  // NUL-terminated for ease of use
    uint8_t payloadLen = 0;

    // Populated on receive only.
    int16_t rssi = 0;
    float snr = 0;
};

class LoRaTransport {
public:
    using ReceiveCallback = std::function<void(const LoRaMessage&)>;

    // Configures the radio from cfg. The shared SPI bus must already be up
    // (main.cpp does this before SD). Returns false if the RA-02 does not
    // answer, in which case every send() is a no-op and the device carries
    // on without radio - a missing radio must never stop a race being
    // logged.
    bool begin(const AppConfig& cfg);
    bool isReady() const { return _ready; }

    // Call every loop iteration. Drives the transmit state machine and
    // polls for incoming packets.
    void loop();

    void setReceiveCallback(ReceiveCallback cb) { _onReceive = std::move(cb); }

    // Queues a message. Returns false only if the queue is full, which is
    // reported and counted rather than silently dropped.
    bool send(LoRaMsgType type, uint16_t dstDeviceId, uint16_t sessionId,
              int32_t correctedDistanceCm, const char* payload = nullptr);

    // This device's numeric id, derived from the digits in
    // AppConfig::deviceId ("RD-07" -> 7). Derived rather than stored so the
    // one identifier the owner configures stays the single source of truth,
    // and so adding LoRa needed no new config field.
    uint16_t deviceId() const { return _deviceId; }

    uint32_t sentCount() const { return _sent; }
    uint32_t receivedCount() const { return _received; }
    uint32_t droppedCount() const { return _dropped; }
    int16_t lastRssi() const { return _lastRssi; }

    // Exposed for tests and for the settings page: turns a configured
    // device-id string into the numeric id that goes on the air.
    static uint16_t deviceIdFromName(const char* name);
    // Wire encode/decode, exposed so a host test can round-trip them
    // without a radio. Returns bytes written, or 0 on failure.
    static size_t encode(const LoRaMessage& msg, uint8_t* out, size_t outLen);
    static bool decode(const uint8_t* in, size_t len, LoRaMessage& out);

private:
    static constexpr size_t TX_QUEUE_LEN = 8;
    enum class TxState : uint8_t { IDLE, SENDING };

    bool _ready = false;
    uint16_t _deviceId = 0;
    uint16_t _sequence = 0;

    TxState _txState = TxState::IDLE;
    uint32_t _txStartedMs = 0;

    LoRaMessage _queue[TX_QUEUE_LEN];
    size_t _qHead = 0, _qTail = 0, _qCount = 0;

    uint32_t _sent = 0, _received = 0, _dropped = 0;
    int16_t _lastRssi = 0;

    ReceiveCallback _onReceive;

    void pumpTx();
    void pollRx();
};
