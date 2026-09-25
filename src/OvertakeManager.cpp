#include "OvertakeManager.h"
#include <Arduino.h>
#include <cmath>
#include "../include/AppConstants.h"
#include "../include/DebugLog.h"

void OvertakeManager::begin(LoRaTransport& lora, BuzzerManager& buzzer) {
    _lora = &lora;
    _buzzer = &buzzer;
}

void OvertakeManager::send(LoRaMsgType type, uint16_t dst) {
    if (!_lora) return;
    _lora->send(type, dst, _sessionId, myDistanceCm());
}

void OvertakeManager::reset(const char* why, BuzzPattern sound) {
    if (_state != OvertakeState::IDLE) {
        LOGF("[GiveWay] Session with %u ended: %s\n", (unsigned)_peerId, why);
        // NONE means end it without a sound - used where the driver was
        // never told anything was happening, so telling them it has
        // stopped would be the first they heard of it.
        if (_buzzer && sound != BuzzPattern::NONE) _buzzer->play(sound);
    }
    _state = OvertakeState::IDLE;
    _peerId = 0;
    _sessionId = 0;
    _peerDistanceValid = false;
    _startingSign = 0;
    _replyPending = false;
}

void OvertakeManager::enterSession(uint16_t peerId, uint16_t sessionId, double peerDistanceM) {
    _peerId = peerId;
    _sessionId = sessionId;
    _peerDistanceM = peerDistanceM;
    _peerDistanceValid = true;
    _lastHeardMs = millis();
    _nextPositionMs = millis() + AppConst::OVERTAKE_POSITION_MS;
    // Which side of us the peer started on. The overtake is complete when
    // this flips - so it has to be recorded before anyone moves.
    const double rel = _peerDistanceM - _myDistanceM;
    _startingSign = (rel >= 0) ? 1 : -1;
}

void OvertakeManager::updatePeerDistance(int32_t cm) {
    _peerDistanceM = (double)cm / 100.0;
    _peerDistanceValid = true;
    _lastHeardMs = millis();
}

void OvertakeManager::toggleRequest() {
    if (!_myDistanceValid) {
        LOGLN("[GiveWay] Cannot ask to pass without a corrected distance - "
              "the car ahead has no way to tell whether it is the one being asked");
        if (_buzzer) _buzzer->play(BuzzPattern::LONG_BEEP);
        return;
    }

    // A second press while asking cancels, which is the only way out of a
    // request nobody answers before the timeout.
    if (_state == OvertakeState::REQUESTING || _state == OvertakeState::REQUEST_ACKED) {
        send(LoRaMsgType::OT_CANCEL, _peerId ? _peerId : LORA_BROADCAST_ID);
        reset("cancelled by the driver", BuzzPattern::SHORT_BEEP);
        return;
    }

    if (_state != OvertakeState::IDLE) {
        LOGF("[GiveWay] Ignoring a request - already in a session with %u\n",
             (unsigned)_peerId);
        if (_buzzer) _buzzer->play(BuzzPattern::LONG_BEEP);
        return;
    }

    // A fresh session id, so replies to a previous attempt cannot be
    // mistaken for replies to this one.
    if (++_sessionId == 0) _sessionId = 1;
    _state = OvertakeState::REQUESTING;
    _peerId = 0;
    _peerDistanceValid = false;
    _lastHeardMs = millis();

    // Broadcast: the requester does not know which device is in front of
    // it. Each receiver decides for itself, from its own distance and the
    // requester's in the header. See the header comment.
    send(LoRaMsgType::OT_REQ, LORA_BROADCAST_ID);
    _nextRequestMs = millis() + AppConst::OVERTAKE_REQUEST_RETRY_MS;
    if (_buzzer) _buzzer->play(BuzzPattern::SHORT_BEEP);
    LOGF("[GiveWay] Asking to pass (session %u, at %.0fm)\n",
         (unsigned)_sessionId, _myDistanceM);
}

void OvertakeManager::driverAck() {
    // _replyPending means we have not answered the asker yet and so have
    // not alerted our own driver either - a Key4 tap at that moment is
    // about something else, not about a request they cannot know exists.
    if (_state != OvertakeState::DEVICE_RECEIVED || _replyPending) {
        // Key 4 is also the Give Way ack when nothing is asking; saying so
        // is more useful than silence.
        LOGLN("[GiveWay] Key4 ack ignored - nobody is asking to pass");
        return;
    }
    _state = OvertakeState::DRIVER_GRANTED;
    send(LoRaMsgType::OT_USER_ACK, _peerId);
    if (_buzzer) _buzzer->play(BuzzPattern::DOUBLE_BEEP);
    LOGF("[GiveWay] Driver agreed to let %u pass\n", (unsigned)_peerId);
}

void OvertakeManager::onMessage(const LoRaMessage& msg) {
    switch (msg.type) {
        case LoRaMsgType::OT_REQ: {
            // Decide whether we are the car being asked. The requester's
            // corrected distance is in the header; ours is what we know.
            if (!_myDistanceValid) return;
            const double theirs = (double)msg.correctedDistanceCm / 100.0;
            const double rel = _myDistanceM - theirs;   // positive: we are ahead

            // The car we are ALREADY dealing with, asking again. That is a
            // repeat, not a new request: our answer was lost in the air.
            // Answering BUSY here would be a disaster - we would tell the
            // very car we are engaged with to give up, and it would.
            if (_state != OvertakeState::IDLE &&
                msg.srcDeviceId == _peerId && msg.sessionId == _sessionId) {
                _lastHeardMs = millis();
                if (_state == OvertakeState::DEVICE_RECEIVED && !_replyPending) {
                    _lora->send(LoRaMsgType::OT_DEV_ACK, LORA_BROADCAST_ID,
                                _sessionId, myDistanceCm());
                } else if (_state == OvertakeState::DRIVER_GRANTED) {
                    // Our driver has already agreed; say so again rather
                    // than dropping them back to a receipt.
                    send(LoRaMsgType::OT_USER_ACK, _peerId);
                }
                return;
            }

            if (rel <= 0) return;                                    // we are behind them
            if (rel > AppConst::OVERTAKE_ELIGIBLE_M) return;         // too far to matter

            if (_state != OvertakeState::IDLE) {
                // Somebody else already has us. Answering BUSY matters:
                // silence would leave the requester waiting out the full
                // timeout for a reply that is never coming.
                _lora->send(LoRaMsgType::BUSY, msg.srcDeviceId, msg.sessionId, myDistanceCm());
                LOGF("[GiveWay] %u asked to pass but we are busy with %u\n",
                     (unsigned)msg.srcDeviceId, (unsigned)_peerId);
                return;
            }

            _state = OvertakeState::DEVICE_RECEIVED;
            enterSession(msg.srcDeviceId, msg.sessionId, theirs);
            // The reply waits its turn: nearest car first. Nothing is said
            // to the driver until it actually goes out, so a car that
            // stands down never disturbs anybody. See the constant.
            _replyPending = true;
            _replyAtMs = millis() + (uint32_t)(rel * AppConst::OVERTAKE_REPLY_SLOT_MS_PER_M);
            LOGF("[GiveWay] %u is asking to pass, %.0fm behind - answering in %lums\n",
                 (unsigned)msg.srcDeviceId, rel,
                 (unsigned long)(rel * AppConst::OVERTAKE_REPLY_SLOT_MS_PER_M));
            break;
        }

        case LoRaMsgType::OT_DEV_ACK:
            // Broadcast, so the other candidates hear it too. One of them
            // is nearer the asker than we are and got in first; the asker
            // has paired with it, and our own reply would only confuse
            // matters. Stand down before our driver is ever told.
            if (_state == OvertakeState::DEVICE_RECEIVED && _replyPending &&
                msg.sessionId == _sessionId && msg.srcDeviceId != _peerId) {
                reset("a nearer car answered first", BuzzPattern::NONE);
                return;
            }
            if (_state != OvertakeState::REQUESTING || msg.sessionId != _sessionId) return;
            _state = OvertakeState::REQUEST_ACKED;
            enterSession(msg.srcDeviceId, _sessionId, (double)msg.correctedDistanceCm / 100.0);
            LOGF("[GiveWay] %u received the request - waiting for their driver\n",
                 (unsigned)msg.srcDeviceId);
            break;

        case LoRaMsgType::OT_USER_ACK:
            if (msg.sessionId != _sessionId) return;
            // A lost OT_DEV_ACK must not cost the driver the pass. The
            // device receipt is only progress information; the agreement
            // is the thing that matters, and it arrives with everything
            // needed to open the session.
            if (_state == OvertakeState::REQUESTING) {
                enterSession(msg.srcDeviceId, _sessionId,
                             (double)msg.correctedDistanceCm / 100.0);
                _state = OvertakeState::REQUEST_ACKED;
            }
            if (_state != OvertakeState::REQUEST_ACKED || msg.srcDeviceId != _peerId) return;
            _state = OvertakeState::GRANTED;
            updatePeerDistance(msg.correctedDistanceCm);
            // Confirm, so the other side knows its agreement landed rather
            // than waiting to find out from our behaviour.
            send(LoRaMsgType::OT_USER_ACK_ACK, _peerId);
            if (_buzzer) _buzzer->play(BuzzPattern::DOUBLE_BEEP);
            LOGF("[GiveWay] %u is letting us pass - go\n", (unsigned)_peerId);
            break;

        case LoRaMsgType::OT_USER_ACK_ACK:
            if (msg.srcDeviceId != _peerId || msg.sessionId != _sessionId) return;
            updatePeerDistance(msg.correctedDistanceCm);
            LOGF("[GiveWay] %u confirmed our agreement\n", (unsigned)_peerId);
            break;

        case LoRaMsgType::OT_POSITION:
            if (msg.srcDeviceId != _peerId || msg.sessionId != _sessionId) return;
            // A peer only reports its position once it considers the
            // overtake under way, so a position arriving while we are
            // still waiting on their driver says the agreement itself was
            // lost in the air. Take the position report as the answer.
            if (_state == OvertakeState::REQUEST_ACKED) {
                _state = OvertakeState::GRANTED;
                send(LoRaMsgType::OT_USER_ACK_ACK, _peerId);
                if (_buzzer) _buzzer->play(BuzzPattern::DOUBLE_BEEP);
                LOGF("[GiveWay] %u is letting us pass (learned from their position report) - go\n",
                     (unsigned)_peerId);
            }
            updatePeerDistance(msg.correctedDistanceCm);
            checkCompletion();
            break;

        case LoRaMsgType::BUSY:
            if (_state != OvertakeState::REQUESTING || msg.sessionId != _sessionId) return;
            LOGF("[GiveWay] %u is busy with another car\n", (unsigned)msg.srcDeviceId);
            reset("the car ahead is busy", BuzzPattern::LONG_BEEP);
            break;

        case LoRaMsgType::OT_CANCEL:
            if (msg.srcDeviceId != _peerId) return;
            reset("cancelled by the other driver", BuzzPattern::LONG_BEEP);
            break;

        case LoRaMsgType::SESSION_END:
            if (msg.srcDeviceId != _peerId) return;
            reset("overtake complete", BuzzPattern::SHORT_BEEP);
            break;

        default:
            break;
    }
}

void OvertakeManager::checkCompletion() {
    if (_state != OvertakeState::GRANTED && _state != OvertakeState::DRIVER_GRANTED) return;
    if (!_peerDistanceValid || _startingSign == 0) return;

    const double rel = _peerDistanceM - _myDistanceM;
    const int8_t sign = (rel >= 0) ? 1 : -1;
    if (sign == _startingSign) return;                       // not past each other yet
    if (fabs(rel) < AppConst::OVERTAKE_COMPLETE_HYSTERESIS_M) return;  // too close to call

    LOGF("[GiveWay] Overtake complete - now %.0fm the other side of %u\n",
         fabs(rel), (unsigned)_peerId);
    send(LoRaMsgType::SESSION_END, _peerId);
    reset("overtake complete", BuzzPattern::SHORT_BEEP);
}

void OvertakeManager::loop() {
    if (_state == OvertakeState::IDLE) return;

    // Nothing heard from the peer for OVERTAKE_COMM_TIMEOUT_MS. Either side
    // resets on its own rather than waiting to be told, because the most
    // likely reason is that the other car is out of range - in which case
    // no message is ever coming.
    if (millis() - _lastHeardMs > AppConst::OVERTAKE_COMM_TIMEOUT_MS) {
        reset("no contact for 30 s", BuzzPattern::LONG_BEEP);
        return;
    }

    // Repeat an unanswered request. Nothing else recovers a lost OT_REQ:
    // no device ever heard it, so no device is going to reply to it.
    if (_state == OvertakeState::REQUESTING &&
        (int32_t)(millis() - _nextRequestMs) >= 0) {
        send(LoRaMsgType::OT_REQ, LORA_BROADCAST_ID);
        _nextRequestMs = millis() + AppConst::OVERTAKE_REQUEST_RETRY_MS;
    }

    // Keep nudging our own driver while they have been asked but have not
    // answered. One alert at the moment of the request is easy to miss at
    // speed.
    if (_state == OvertakeState::DEVICE_RECEIVED && _replyPending &&
        (int32_t)(millis() - _replyAtMs) >= 0) {
        _replyPending = false;
        // Broadcast rather than addressed: the asker is the only device
        // that acts on it, but the other candidates need to hear it to
        // know the request has been taken.
        _lora->send(LoRaMsgType::OT_DEV_ACK, LORA_BROADCAST_ID, _sessionId, myDistanceCm());
        if (_buzzer) _buzzer->play(BuzzPattern::TRIPLE_BEEP);
        _nextReminderMs = millis() + AppConst::OVERTAKE_REMINDER_MS;
        LOGF("[GiveWay] Answered %u - tap Key4 to let them pass\n", (unsigned)_peerId);
    }

    if (_state == OvertakeState::DEVICE_RECEIVED && !_replyPending &&
        (int32_t)(millis() - _nextReminderMs) >= 0) {
        if (_buzzer) _buzzer->play(BuzzPattern::TRIPLE_BEEP);
        _nextReminderMs = millis() + AppConst::OVERTAKE_REMINDER_MS;
    }

    // Exchange positions once the overtake is actually under way. Before
    // that there is nothing to measure - the cars have not agreed to move
    // relative to each other yet - so the airtime is not spent.
    const bool underway = (_state == OvertakeState::GRANTED ||
                           _state == OvertakeState::DRIVER_GRANTED);
    if (underway && (int32_t)(millis() - _nextPositionMs) >= 0) {
        send(LoRaMsgType::OT_POSITION, _peerId);
        _nextPositionMs = millis() + AppConst::OVERTAKE_POSITION_MS;
        checkCompletion();
    }
}
