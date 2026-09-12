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

    if (!isPresent()) {
        // No usable reading (divider not fitted / nothing connected):
        // never assert critical off the back of it.
        _criticalSamples = 0;
        _critical = false;
        return;
    }

    if (_percent <= AppConst::BATTERY_CRITICAL_PERCENT) {
        if (_criticalSamples < AppConst::BATTERY_CRITICAL_SAMPLES) _criticalSamples++;
    } else {
        _criticalSamples = 0;
    }

    if (!_critical) {
        if (_criticalSamples >= AppConst::BATTERY_CRITICAL_SAMPLES) {
            _critical = true;
            Serial.printf("[Battery] CRITICAL: %.2fV (%u%%) sustained\n", _voltage, _percent);
        }
    } else if (_percent >= AppConst::BATTERY_RECOVER_PERCENT) {
        _critical = false;
        Serial.printf("[Battery] Recovered: %.2fV (%u%%)\n", _voltage, _percent);
    }
}

namespace {

// Li-ion open-circuit voltage vs state-of-charge, PER CELL.
//
// Li-ion is strongly non-linear: most of the usable capacity sits in the
// flat ~3.7-4.0V plateau, with steep knees at both ends. Interpolating
// linearly between 3.0V and 4.2V overstates the remaining charge badly
// across the whole lower half - it reports ~58% at 3.70V/cell where the
// real figure is about 13% - which is the worst possible direction to be
// wrong in on a device that halts the race when the pack runs out.
struct OcvPoint { float cellVolts; uint8_t percent; };

constexpr OcvPoint OCV_CURVE[] = {
    {4.20f, 100}, {4.06f, 90}, {3.98f, 80}, {3.92f, 70},
    {3.87f,  60}, {3.82f, 50}, {3.79f, 40}, {3.77f, 30},
    {3.74f,  20}, {3.68f, 10}, {3.45f,  5}, {3.00f,  0},
};
constexpr size_t OCV_POINTS = sizeof(OCV_CURVE) / sizeof(OCV_CURVE[0]);

// Raw state-of-charge from the discharge curve: 0% is the cell's chemical
// empty (3.0V), which is NOT the same as the point this device stops
// working. estimatePercent() rescales this into usable charge.
float rawSocPercent(float cellVolts) {
    if (cellVolts >= OCV_CURVE[0].cellVolts) return 100.0f;
    if (cellVolts <= OCV_CURVE[OCV_POINTS - 1].cellVolts) return 0.0f;

    for (size_t i = 1; i < OCV_POINTS; ++i) {
        if (cellVolts >= OCV_CURVE[i].cellVolts) {
            const OcvPoint& hi = OCV_CURVE[i - 1];
            const OcvPoint& lo = OCV_CURVE[i];
            const float span = hi.cellVolts - lo.cellVolts;
            const float frac = (cellVolts - lo.cellVolts) / span;
            return lo.percent + frac * (hi.percent - lo.percent);
        }
    }
    return 0.0f;
}

} // namespace

uint8_t BatteryManager::estimatePercent(float voltage) const {
    // Still an estimate, not a fuel gauge: there is no current sensing, and
    // the curve is RESTING voltage. Under load the pack sags, so a loaded
    // reading maps low - which errs safe, and is why the critical cutoff is
    // debounced rather than acting on a single sagged sample.
    const float raw = rawSocPercent(voltage / CELL_COUNT);

    // Rescale so the brownout point reads 0%: charge the device cannot
    // reach is not charge remaining. State-of-charge is proportional to
    // stored energy, so rescaling it linearly is the correct operation -
    // "of the usable range, how much is left".
    constexpr float reserve = (float)AppConst::BATTERY_USABLE_RESERVE_PCT;
    if (raw <= reserve) return 0;
    return (uint8_t)((raw - reserve) / (100.0f - reserve) * 100.0f + 0.5f);
}
