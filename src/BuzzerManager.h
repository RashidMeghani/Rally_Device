// Non-blocking buzzer patterns.
//
// Every sound this device makes happens while it is also parsing GNSS,
// writing the race log and driving a radio, so nothing here may use
// delay(). A pattern is a number of beeps of a given on/off duration; the
// player advances it from loop() and the caller returns immediately.
//
// Patterns are deliberately distinguishable by ear alone, because a driver
// on a desert stage will hear them long before looking at the screen.
#pragma once

#include <cstdint>

enum class BuzzPattern : uint8_t {
    NONE,
    SHORT_BEEP,     // one 80 ms blip - acknowledgement of an action
    DOUBLE_BEEP,    // two blips - a request was granted, go
    TRIPLE_BEEP,    // three blips - someone is asking to pass you
    LONG_BEEP,      // one 600 ms tone - refused, or a session ended badly
};

class BuzzerManager {
public:
    void begin(int pin);

    // Starts a pattern, replacing whatever is playing. Replacing rather
    // than queueing is deliberate: the newest event is the one the driver
    // needs to hear, and a queue would play stale alerts late.
    void play(BuzzPattern pattern);

    // Call every loop iteration.
    void loop();

    bool isPlaying() const { return _remaining > 0; }

private:
    int _pin = -1;
    uint8_t _remaining = 0;      // beeps left, including the one sounding
    bool _on = false;
    uint32_t _phaseEndsMs = 0;
    uint32_t _onMs = 0, _offMs = 0;

    void setOutput(bool on);
};
