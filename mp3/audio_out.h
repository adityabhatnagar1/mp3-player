#pragma once
#include <Arduino.h>

namespace audio {

static constexpr int      PIN_BCK  = 27;
static constexpr int      PIN_LRCK = 25;
static constexpr int      PIN_DOUT = 26;
static constexpr uint32_t SAMPLE_RATE_HZ = 44100;   // I2S clock for the built-in tone/scale

static constexpr int NUM_BANDS = 10;                // OLED bars fed by levels()

enum Mode : uint8_t {
    MODE_OFF    = 0,   // silence
    MODE_TONE   = 1,   // 440 Hz test tone (built-in generator)
    MODE_SCALE  = 2,   // C major scale up + down, looping (built-in generator)
    MODE_STREAM = 3,   // PCM streamed from the laptop (entered/left by the stream functions only)
};

enum StreamState : uint8_t {
    STREAM_IDLE      = 0,
    STREAM_BUFFERING = 1,   // session open, filling the ring before playback
    STREAM_PLAYING   = 2,
    STREAM_DRAINING  = 3,   // host sent END, playing out what is queued
};

struct StreamStatus {
    uint8_t  state;          // StreamState
    uint16_t session;        // session id of the last HELLO
    uint16_t underruns;      // ring ran dry while playing (this session)
    uint32_t fillBytes;      // PCM bytes queued
    uint32_t capacityBytes;  // ring size
};

// Initialises I2S and starts the audio task (core 0). Returns false on failure.
bool begin();

// Built-in generators. Ignored while a stream session is active.
// Thread-safe: the change is applied by the audio task within one DMA period.
void setMode(Mode m);
Mode mode();
uint32_t writeErrors();

// ---- Streaming (called by the UART receiver task) --------------------------
// Open a session: flushes the ring, reconfigures the I2S clock, starts buffering.
bool streamBegin(uint32_t rateHz, uint8_t channels, uint16_t session);
void streamEnd();     // no more data coming: play out the ring, then stop
void streamAbort();   // stop immediately
// Queue PCM (whole frames only). All-or-nothing; false if the ring has no room.
bool streamWrite(const uint8_t* data, uint32_t len);
StreamStatus streamStatus();

// ---- Visualisation ----------------------------------------------------------
// Fills bands[] with 0..255 bar heights derived from the PCM actually being played.
// Returns true while a stream session is active (bands are zero while buffering/silent);
// returns false when idle so the caller can fall back to its own animation.
bool levels(uint8_t bands[NUM_BANDS]);

}  // namespace audio
