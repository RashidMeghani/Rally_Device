#include "BuzzerManager.h"
#include <Arduino.h>

void BuzzerManager::begin(int pin) {
    _pin = pin;
    pinMode(_pin, OUTPUT);
    setOutput(false);
}

void BuzzerManager::setOutput(bool on) {
    if (_pin < 0) return;
    digitalWrite(_pin, on ? HIGH : LOW);
    _on = on;
}

void BuzzerManager::play(BuzzPattern pattern) {
    switch (pattern) {
        case BuzzPattern::SHORT_BEEP:  _remaining = 1; _onMs = 80;  _offMs = 0;   break;
        case BuzzPattern::DOUBLE_BEEP: _remaining = 2; _onMs = 90;  _offMs = 90;  break;
        case BuzzPattern::TRIPLE_BEEP: _remaining = 3; _onMs = 120; _offMs = 120; break;
        case BuzzPattern::LONG_BEEP:   _remaining = 1; _onMs = 600; _offMs = 0;   break;
        case BuzzPattern::NONE:
        default:
            _remaining = 0;
            setOutput(false);
            return;
    }
    setOutput(true);
    _phaseEndsMs = millis() + _onMs;
}

void BuzzerManager::loop() {
    if (_remaining == 0) return;
    if ((int32_t)(millis() - _phaseEndsMs) < 0) return;

    if (_on) {
        setOutput(false);
        _remaining--;
        // The gap after the final beep would only delay silence, so the
        // pattern ends the moment the last tone does.
        if (_remaining == 0) return;
        _phaseEndsMs = millis() + _offMs;
    } else {
        setOutput(true);
        _phaseEndsMs = millis() + _onMs;
    }
}
