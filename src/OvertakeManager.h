// Give Way / Overtake: the negotiation between the car that wants to pass
// and the car in front of it (spec section 15).
//
// -----------------------------------------------------------------------
// The exchange
// -----------------------------------------------------------------------
//
//   requester (behind)                        receiver (ahead)
//   ------------------                        ----------------
//   Key2 held 2 s
//   OT_REQ  (broadcast) ──────────────────▶   is the requester BEHIND me,
//   REQUESTING                                and within 183 m, and am I
//                                             free? then:
//                        ◀────────────── OT_DEV_ACK   DEVICE_RECEIVED
//   REQUEST_ACKED                              (device receipt - the driver
//                                               has not answered yet)
//                                             ...driver taps Key4...
//                        ◀───────────── OT_USER_ACK   DRIVER_GRANTED
//   GRANTED ─────────── OT_USER_ACK_ACK ──▶
//
//   both sides then exchange OT_POSITION every OVERTAKE_POSITION_MS until
//   the overtake completes, and either fires SESSION_END.
//
// A device already in a session with somebody else answers BUSY instead,
// and a requester that receives BUSY gives up rather than waiting.
//
// -----------------------------------------------------------------------
// Three decisions worth knowing
// -----------------------------------------------------------------------
//
// 1. THE REQUEST IS BROADCAST, NOT ADDRESSED. The car behind has no way to
//    know which device is in front of it: checkpoint events are 10-20 km
//    apart, which says nothing about who is where right now. So the request
//    goes to everyone, and each receiver decides for itself whether it is
//    the one being asked - it knows its own corrected distance and the
//    requester's travels in the packet header. Only a device that is ahead,
//    within range and free will answer.
//
// 2. POSITION IS EXCHANGED ONLY DURING A SESSION. Completion is detected
//    from the relative distance changing sign, which needs the peer's live
//    position. Beaconing that continuously would be ruinous - at ~165 ms of
//    airtime per packet, a handful of cars reporting every second would
//    saturate the channel - so it happens only while a session is open,
//    which is rare and lasts seconds.
//
// 3. THE SIGN FLIP NEEDS HYSTERESIS. Two cars running abreast sit at a
//    relative distance of roughly zero, where GNSS noise alone flips the
//    sign back and forth. The session ends only once the flip is confirmed
//    by OVERTAKE_COMPLETE_HYSTERESIS_M on the new side.
#pragma once

#include <cstdint>
#include "LoRaTransport.h"
#include "BuzzerManager.h"

enum class OvertakeState : uint8_t {
    IDLE,
    // Requester side.
    REQUESTING,        // asked, nothing has answered yet
    REQUEST_ACKED,     // a device answered; its driver has not
    GRANTED,           // the driver ahead agreed - go
    // Receiver side.
    DEVICE_RECEIVED,   // we were asked; our driver has not answered
    DRIVER_GRANTED,    // our driver agreed and told them
};

class OvertakeManager {
public:
    void begin(LoRaTransport& lora, BuzzerManager& buzzer);

    // The device's own corrected race distance, refreshed every tick. Kept
    // here rather than passed to every call so that a message arriving
    // mid-tick is judged against the same position as the rest of the tick.
    void setMyDistanceM(double m) { _myDistanceM = m; _myDistanceValid = true; }

    void loop();
    void onMessage(const LoRaMessage& msg);

    // --- driver actions ---
    // Key 2 held 2 s: ask to pass, or cancel a request already outstanding.
    void toggleRequest();
    // Key 4 tapped: the driver ahead agrees to be passed.
    void driverAck();

    OvertakeState state() const { return _state; }
    bool sessionActive() const { return _state != OvertakeState::IDLE; }

    // Peer for the OLED's ahead-device fields. Valid only in a session
    // where the peer has reported a position.
    bool hasPeerDistance() const { return _peerDistanceValid; }
    uint16_t peerId() const { return _peerId; }
    // Peer minus me, in metres: positive while the peer is still ahead.
    float relativeM() const { return (float)(_peerDistanceM - _myDistanceM); }

private:
    LoRaTransport* _lora = nullptr;
    BuzzerManager* _buzzer = nullptr;

    OvertakeState _state = OvertakeState::IDLE;
    uint16_t _peerId = 0;
    uint16_t _sessionId = 0;

    double _myDistanceM = 0;
    bool _myDistanceValid = false;
    double _peerDistanceM = 0;
    bool _peerDistanceValid = false;

    uint32_t _lastHeardMs = 0;      // last message from the peer, for the timeout
    uint32_t _nextPositionMs = 0;
    uint32_t _nextReminderMs = 0;
    uint32_t _nextRequestMs = 0;    // repeat of an unanswered OT_REQ
    // A reply to a request, held back so that the nearest car answers
    // first. While it is pending the driver has not been alerted, so
    // standing down costs them nothing.
    bool _replyPending = false;
    uint32_t _replyAtMs = 0;

    // Sign of the relative distance when the session began, so a flip can
    // be recognised. 0 means not yet established.
    int8_t _startingSign = 0;

    int32_t myDistanceCm() const { return (int32_t)llround(_myDistanceM * 100.0); }
    void send(LoRaMsgType type, uint16_t dst);
    void reset(const char* why, BuzzPattern sound);
    void enterSession(uint16_t peerId, uint16_t sessionId, double peerDistanceM);
    void updatePeerDistance(int32_t cm);
    void checkCompletion();
};
