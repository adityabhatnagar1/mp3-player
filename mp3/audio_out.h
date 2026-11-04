#pragma once
#include <Arduino.h>

namespace audio {

static constexpr int      PIN_BCK  = 27;
static constexpr int      PIN_LRCK = 25;
static constexpr int      PIN_DOUT = 26;
static constexpr uint32_t SAMPLE_RATE_HZ = 44100;

enum Mode : uint8_t {
    MODE_OFF   = 0,   // silence
    MODE_TONE  = 1,   // 440 Hz test tone
    MODE_SCALE = 2,   // C major scale up + down, looping
};

bool begin();
void setMode(Mode m);
Mode mode();
uint32_t writeErrors();

}  // namespace audio