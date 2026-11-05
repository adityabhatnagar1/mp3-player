#pragma once
#include <Arduino.h>
#include "source_manager.h"

namespace audio {

static constexpr int      PIN_BCK  = 27;
static constexpr int      PIN_LRCK = 25;
static constexpr int      PIN_DOUT = 26;
static constexpr uint32_t SAMPLE_RATE_HZ = 44100;

static constexpr int NUM_BANDS = 10;

enum Mode : uint8_t {
    MODE_OFF    = 0,
    MODE_TONE   = 1,
    MODE_SCALE  = 2,
    MODE_STREAM = 3,
};

enum StreamState : uint8_t {
    STREAM_IDLE      = 0,
    STREAM_BUFFERING = 1,
    STREAM_PLAYING   = 2,
    STREAM_DRAINING  = 3,
};

struct StreamStatus {
    uint8_t  state;
    uint16_t session;
    uint16_t underruns;
    uint32_t fillBytes;
    uint32_t capacityBytes;
};

bool begin();
void setMode(Mode m);
Mode mode();
uint32_t writeErrors();

bool streamBegin(uint32_t rateHz, uint8_t channels, uint16_t session);
void streamEnd();
void streamAbort();
bool streamWrite(const uint8_t* data, uint32_t len);
StreamStatus streamStatus();

// ---- Bluetooth A2DP sink (mutually exclusive with USB serial) -------------
bool btBegin();                                      // rate supplied later
void btEnd();
bool btWrite(const uint8_t* data, uint32_t len);
bool btActive();
void btSetSampleRate(uint16_t rate);                 // called from BT callback

void    setVolume(uint8_t vol);   // 0..100
uint8_t volume();

// ---- Visualisation ---------------------------------------------------------
bool levels(uint8_t bands[NUM_BANDS]);

}  // namespace audio