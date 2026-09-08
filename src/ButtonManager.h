// Debounce + non-conflicting long-press semantics for the 3 used keys
// (spec section 13). Key3 is wired but unused - no firmware action.
//
// Wiring assumption (flagged, not spec-stated): GPIO34/35/39 provide no
// internal pull resistors on the ESP32, and the spec's own engineering
// check phrases the risk as "if the keypad is wired active-low" -
// implying that is the intended wiring. This driver assumes active-low
// (pressed = pin reads LOW, via an external pull-up) and isolates that
// assumption in one place (ACTIVE_LOW below) so it's a one-line flip if
// the actual board is wired active-high instead.
//
// Key 2 staged long-press (2s Give Way toggle vs 5s Wi-Fi AP toggle): the
// 5s action fires immediately once the hold reaches 5000ms (so the mode
// toggle feels responsive); the 2s action fires only on release, and only
// if the 5s action did NOT already fire during this press - this is what
// keeps the two "mutually safe" per the spec's own acceptance test.
#pragma once

#include <cstdint>

enum class ButtonEvent : uint8_t {
    NONE,
    KEY1_RESTART,          // 1.5s hold: software restart
    KEY2_GIVEWAY_TOGGLE,   // 2s hold+release (only if not escalated to 5s): start/cancel Give Way request
    KEY2_WIFI_TOGGLE,      // 5s hold: enter/leave Wi-Fi settings/AP mode
    KEY4_BYPASS_ACK,       // 1s hold: bypass start-geofence condition / Give Way ahead-driver ack
};

class ButtonManager {
public:
    void begin();

    // Call every main loop iteration. Returns at most one event per call -
    // in the extremely unlikely case two keys cross their thresholds on the
    // exact same tick, Key1 takes priority over Key2 over Key4, and the
    // other event is simply picked up on the next tick's read.
    ButtonEvent loop();

private:
    static constexpr bool ACTIVE_LOW = true;
    static constexpr uint32_t DEBOUNCE_MS = 30;
    static constexpr uint32_t KEY1_RESTART_HOLD_MS = 1500;
    static constexpr uint32_t KEY2_GIVEWAY_HOLD_MS = 2000;
    static constexpr uint32_t KEY2_WIFI_HOLD_MS = 5000;
    static constexpr uint32_t KEY4_BYPASS_HOLD_MS = 1000;

    struct Debounce { bool stableState = false; bool lastRaw = false; uint32_t lastChangeMs = 0; };
    struct HoldTracker { bool pressed = false; uint32_t pressStartMs = 0; bool longFired = false; };

    Debounce _db1, _db2, _db4;
    HoldTracker _hk1, _hk2, _hk4;

    bool debouncedRead(Debounce& d, int pin);
    ButtonEvent updateKey1(bool isPressed, uint32_t now);
    ButtonEvent updateKey2(bool isPressed, uint32_t now);
    ButtonEvent updateKey4(bool isPressed, uint32_t now);
};
