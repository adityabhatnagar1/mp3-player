#include "audio_levels.h"

#include <math.h>

namespace viz {
namespace {

// Band centres in Hz, roughly logarithmic.
const float CENTER_HZ[BANDS] = {80, 150, 280, 500, 900, 1600, 2800, 5000, 8500, 13000};

// Display mapping: amplitude (dBFS of an equivalent sine) -> 0..255.
// TILT_DB lifts the upper bands because music has less energy there.
const float DB_BOTTOM = -62.0f;   // bar height 0
const float DB_RANGE  =  50.0f;   // bar is full this many dB above DB_BOTTOM
const float TILT_DB[BANDS] = {0, 1, 2, 3, 4.5f, 6, 7.5f, 9, 10.5f, 12};

}  // namespace

BandAnalyzer::BandAnalyzer() { reset(32000); }

void BandAnalyzer::reset(uint32_t sampleRateHz) {
    const float fs = (float)sampleRateHz;
    for (int k = 0; k < BANDS; ++k) {
        float f = CENTER_HZ[k];
        if (f > 0.45f * fs) f = 0.45f * fs;          // keep below Nyquist at low sample rates
        coeff_[k] = 2.0f * cosf(2.0f * (float)M_PI * f / fs);
        s1_[k] = 0.0f;
        s2_[k] = 0.0f;
        out_[k] = 0;
    }
    count_ = 0;
}

bool BandAnalyzer::feed(const int16_t* lr, int frames) {
    bool published = false;
    for (int i = 0; i < frames; ++i) {
        // (L+R)/2 normalised to +-1.0
        const float x = (float)((int32_t)lr[2 * i] + (int32_t)lr[2 * i + 1]) * (1.0f / 65536.0f);
        for (int k = 0; k < BANDS; ++k) {
            const float s0 = x + coeff_[k] * s1_[k] - s2_[k];
            s2_[k] = s1_[k];
            s1_[k] = s0;
        }
        if (++count_ >= BLOCK) {
            finishBlock();
            published = true;
        }
    }
    return published;
}

void BandAnalyzer::finishBlock() {
    for (int k = 0; k < BANDS; ++k) {
        const float p = s1_[k] * s1_[k] + s2_[k] * s2_[k] - coeff_[k] * s1_[k] * s2_[k];
        const float amp = 2.0f * sqrtf(p > 0.0f ? p : 0.0f) / (float)BLOCK;   // sine amplitude, 1.0 = full scale
        const float db  = 20.0f * log10f(amp + 1e-7f);
        float v = (db - DB_BOTTOM + TILT_DB[k]) / DB_RANGE;
        if (v < 0.0f) v = 0.0f;
        if (v > 1.0f) v = 1.0f;
        out_[k] = (uint8_t)(v * 255.0f);
        s1_[k] = 0.0f;
        s2_[k] = 0.0f;
    }
    count_ = 0;
}

void BandAnalyzer::read(uint8_t out[BANDS]) const {
    for (int k = 0; k < BANDS; ++k) out[k] = out_[k];
}

}  // namespace viz
