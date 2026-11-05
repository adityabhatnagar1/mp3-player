#pragma once
// Cheap real-signal spectrum for the OLED: a bank of 10 Goertzel resonators
// (no FFT, no windowing, ~1.5 % of one core at 32 kHz) that turns PCM into
// 10 log-scaled bar heights (0..255). Pure C++11 so it can be tested on a PC.

#include <stdint.h>

namespace viz {

constexpr int BANDS = 10;
constexpr int BLOCK = 1024;      // samples per analysis block (~32 ms @ 32 kHz)

class BandAnalyzer {
 public:
  BandAnalyzer();
  void reset(uint32_t sampleRateHz);

  // frames of interleaved L,R s16. Uses (L+R)/2. Returns true when a block finished
  // and new band values were published.
  bool feed(const int16_t* lr, int frames);

  // Latest bar heights (single-byte reads, safe from another task; tearing between bands is harmless).
  void read(uint8_t out[BANDS]) const;

 private:
  void finishBlock();

  float            coeff_[BANDS];
  float            s1_[BANDS];
  float            s2_[BANDS];
  int              count_;
  volatile uint8_t out_[BANDS];
};

}  // namespace viz
