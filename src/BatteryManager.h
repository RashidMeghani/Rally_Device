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

class BatteryManager {
public:
    void begin();

    // Non-blocking: samples at most once per SAMPLE_INTERVAL_MS, using a
    // small moving average to filter ADC noise. Call every main loop tick.
    void loop();

    float voltageMeasured() const { return _voltage; }
    uint8_t percentEstimate() const { return _percent; }
    bool isLow() const { return _voltage > 0 && _voltage < LOW_VOLTAGE_THRESHOLD; }

private:
    static constexpr float DIVIDER_UPPER_KOHM = 100.0f;
    static constexpr float DIVIDER_LOWER_KOHM = 47.0f;
    static constexpr float CALIBRATION_FACTOR = 1.0f; // set after multimeter comparison
    static constexpr uint32_t SAMPLE_INTERVAL_MS = 500;
    static constexpr uint8_t AVG_SAMPLES = 8;
    static constexpr float LOW_VOLTAGE_THRESHOLD = 6.4f; // ~3.2V/cell, conservative 2S cutoff

    float _voltage = 0;
    uint8_t _percent = 0;
    uint32_t _lastSampleMs = 0;

    uint8_t estimatePercent(float voltage) const;
};
