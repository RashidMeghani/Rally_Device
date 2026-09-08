#include "BatteryManager.h"
#include "../include/PinConfig.h"
#include <Arduino.h>

void BatteryManager::begin() {
    // 11dB attenuation covers the full ~0-3.3V the divider can present at
    // the ADC pin (8.4V pack -> ~2.69V through the baseline 100k/47k divider).
    analogSetPinAttenuation(Pins::BATTERY_ADC, ADC_11db);
}

void BatteryManager::loop() {
    uint32_t now = millis();
    if (now - _lastSampleMs < SAMPLE_INTERVAL_MS) return;
    _lastSampleMs = now;

    // analogReadMilliVolts() applies the ESP32's own eFuse ADC calibration
    // curve, which is materially more accurate than a naive raw*3.3/4095
    // conversion - CALIBRATION_FACTOR is then the one remaining knob for
    // bench-calibrating against a multimeter, per the spec's instruction.
    uint32_t sumMv = 0;
    for (uint8_t i = 0; i < AVG_SAMPLES; ++i) {
        sumMv += analogReadMilliVolts(Pins::BATTERY_ADC);
    }
    float adcVolts = (sumMv / (float)AVG_SAMPLES) / 1000.0f;

    float dividerRatio = (DIVIDER_UPPER_KOHM + DIVIDER_LOWER_KOHM) / DIVIDER_LOWER_KOHM;
    _voltage = adcVolts * dividerRatio * CALIBRATION_FACTOR;
    _percent = estimatePercent(_voltage);
}

uint8_t BatteryManager::estimatePercent(float voltage) const {
    // Rough 2S Li-ion open-circuit-voltage curve, linear between empty and
    // full rather than a proper discharge-curve lookup - explicitly an
    // estimate; the spec says measured voltage is the value that matters.
    constexpr float EMPTY_V = 6.0f;  // ~3.0V/cell
    constexpr float FULL_V = 8.4f;   // 4.2V/cell
    if (voltage <= EMPTY_V) return 0;
    if (voltage >= FULL_V) return 100;
    return (uint8_t)((voltage - EMPTY_V) / (FULL_V - EMPTY_V) * 100.0f);
}
