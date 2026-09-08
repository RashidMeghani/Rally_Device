#include "ButtonManager.h"
#include "../include/PinConfig.h"
#include <Arduino.h>

void ButtonManager::begin() {
    pinMode(Pins::KEY1, INPUT);
    pinMode(Pins::KEY2, INPUT);
    pinMode(Pins::KEY4, INPUT);
}

bool ButtonManager::debouncedRead(Debounce& d, int pin) {
    bool raw = ACTIVE_LOW ? (digitalRead(pin) == LOW) : (digitalRead(pin) == HIGH);
    uint32_t now = millis();
    if (raw != d.lastRaw) {
        d.lastRaw = raw;
        d.lastChangeMs = now;
    }
    if (now - d.lastChangeMs >= DEBOUNCE_MS) {
        d.stableState = d.lastRaw;
    }
    return d.stableState;
}

ButtonEvent ButtonManager::updateKey1(bool isPressed, uint32_t now) {
    if (isPressed && !_hk1.pressed) {
        _hk1.pressed = true; _hk1.pressStartMs = now; _hk1.longFired = false;
    } else if (isPressed && _hk1.pressed && !_hk1.longFired &&
               (now - _hk1.pressStartMs) >= KEY1_RESTART_HOLD_MS) {
        _hk1.longFired = true;
        return ButtonEvent::KEY1_RESTART; // deliberate long press: fire immediately, don't wait for release
    } else if (!isPressed) {
        _hk1.pressed = false;
    }
    return ButtonEvent::NONE;
}

ButtonEvent ButtonManager::updateKey2(bool isPressed, uint32_t now) {
    if (isPressed && !_hk2.pressed) {
        _hk2.pressed = true; _hk2.pressStartMs = now; _hk2.longFired = false;
    } else if (isPressed && _hk2.pressed && !_hk2.longFired &&
               (now - _hk2.pressStartMs) >= KEY2_WIFI_HOLD_MS) {
        _hk2.longFired = true;
        return ButtonEvent::KEY2_WIFI_TOGGLE; // 5s reached while still held: fire now
    } else if (!isPressed && _hk2.pressed) {
        uint32_t held = now - _hk2.pressStartMs;
        _hk2.pressed = false;
        // Only fire the 2s action if the 5s action didn't already fire this
        // press - this is the "suppress 2s when 5s is intended" rule.
        if (!_hk2.longFired && held >= KEY2_GIVEWAY_HOLD_MS) {
            return ButtonEvent::KEY2_GIVEWAY_TOGGLE;
        }
    }
    return ButtonEvent::NONE;
}

ButtonEvent ButtonManager::updateKey4(bool isPressed, uint32_t now) {
    if (isPressed && !_hk4.pressed) {
        _hk4.pressed = true; _hk4.pressStartMs = now; _hk4.longFired = false;
    } else if (isPressed && _hk4.pressed && !_hk4.longFired &&
               (now - _hk4.pressStartMs) >= KEY4_BYPASS_HOLD_MS) {
        _hk4.longFired = true;
        return ButtonEvent::KEY4_BYPASS_ACK;
    } else if (!isPressed) {
        _hk4.pressed = false;
    }
    return ButtonEvent::NONE;
}

ButtonEvent ButtonManager::loop() {
    uint32_t now = millis();
    bool p1 = debouncedRead(_db1, Pins::KEY1);
    bool p2 = debouncedRead(_db2, Pins::KEY2);
    bool p4 = debouncedRead(_db4, Pins::KEY4);

    ButtonEvent e = updateKey1(p1, now);
    if (e != ButtonEvent::NONE) return e;
    e = updateKey2(p2, now);
    if (e != ButtonEvent::NONE) return e;
    return updateKey4(p4, now);
}
