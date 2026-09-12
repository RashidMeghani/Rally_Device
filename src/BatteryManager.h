// 2S Li-ion battery voltage sensing on GPIO36 (spec section 17).
//
// Baseline divider: 100 kOhm upper / 47 kOhm lower (owner's engineering
// recommendation) -> at 8.4V pack voltage the ADC sees ~2.69V. A ~100nF
// filter cap at the ADC input is assumed on the physical board (not
// something firmware controls). ESP32 ADC1 is used at 11dB attenuation to
// cover the full ~0-3.3V input range the divider produces.
//
// Calibration: the raw ADC-counts-to-volts conversion below uses the
// nominal 3.3V/4095-count reference, then applies a single multiplicative
// CALIBRATION_FACTOR. The spec explicitly asks this be "calibrated against
// a multimeter" - that calibration is exactly what CALIBRATION_FACTOR is
// for: measure the pack with a multimeter, compare to voltage() before
// calibration, and set CALIBRATION_FACTOR = multimeter_reading /
// uncalibrated_reading. Left at 1.0 (uncalibrated) until that bench step
// is done - flagged here rather than guessing a value.
//
// Percentage is explicitly a rough estimate from a 2S Li-ion open-circuit
// voltage curve, not a fuel-gauge measurement (no current sensing) - the
// spec says to treat voltage as the primary value and percentage as an
// estimate only.
#pragma once

#include <cstdint>
#include "../include/AppConstants.h"

class BatteryManager {
public:
    void begin();

    // Non-blocking: samples at most once per SAMPLE_INTERVAL_MS, using a
    // small moving average to filter ADC noise. Call every main loop tick.
    void loop();

    float voltageMeasured() const { return _voltage; }
    uint8_t percentEstimate() const { return _percent; }

    // False when the reading is implausibly low for a 2S pack, which in
    // practice means the sense divider is not fitted (or nothing is
    // connected) rather than a flat battery. Everything battery-driven is
    // gated on this so an unfitted divider cannot masquerade as 0%.
    bool isPresent() const { return _voltage >= AppConst::BATTERY_PRESENT_MIN_V; }

    // Expressed as a percentage now that the percentage follows a real
    // Li-ion curve: a voltage threshold would be misleading, since most of
    // the pack's remaining charge is decided within a narrow voltage band.
    bool isLow() const { return isPresent() && _percent <= AppConst::BATTERY_LOW_PERCENT; }

    // Debounced critical state - see AppConstants for why it is debounced
    // and why recovery uses a higher threshold than the trigger.
    bool isCritical() const { return _critical; }

    // Enough charge to safely BEGIN real work - opening files, writing to
    // SD. Deliberately much higher than the critical halt threshold, and
    // that gap is the point: a pack that has just browned out recovers a
    // little once the load is removed, so without this the device boots on
    // that phantom charge, opens files, sags under its own load and browns
    // out again mid-write. On a dying pack that becomes a boot/die loop,
    // with every cycle another chance to corrupt the card.
    bool hasOperatingCharge() const {
        return isPresent() && _percent >= AppConst::BATTERY_RECOVER_PERCENT;
    }

private:
    static constexpr float DIVIDER_UPPER_KOHM = 100.0f;
    static constexpr float DIVIDER_LOWER_KOHM = 47.0f;
    // Bench-calibrated on the prototype board: multimeter read 7.80V while
    // the uncalibrated firmware reported 7.60V, so 7.80/7.60 = 1.0263.
    // This corrects the sense divider's resistor tolerance, which is a
    // pure scale error (the ESP32's own ADC non-linearity is already
    // handled by analogReadMilliVolts' eFuse calibration curve), so one
    // multiplicative factor holds across the whole range.
    // NOTE: this value is specific to THIS board's resistors - another
    // unit built from the same BOM will need its own.
    static constexpr float CALIBRATION_FACTOR = 1.0263f;
    static constexpr uint32_t SAMPLE_INTERVAL_MS = 500;
    static constexpr uint8_t AVG_SAMPLES = 8;
    static constexpr float CELL_COUNT = 2.0f; // 2S pack

    float _voltage = 0;
    uint8_t _percent = 0;
    uint32_t _lastSampleMs = 0;
    uint8_t _criticalSamples = 0;
    bool _critical = false;

    uint8_t estimatePercent(float voltage) const;
};
