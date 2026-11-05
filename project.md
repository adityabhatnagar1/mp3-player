# Project Tree

````text
.
├── ESP32-A2DP-main.zip
├── mp3
│   ├── audio_levels.cpp
│   ├── audio_levels.h
│   ├── audio_out.cpp
│   ├── audio_out.h
│   ├── bt_audio.cpp
│   ├── bt_audio.h
│   ├── mp3.ino
│   ├── player_ui.cpp
│   ├── player_ui.h
│   ├── source_manager.h
│   ├── stream_player.cpp
│   ├── stream_player.h
│   ├── stream_proto.cpp
│   ├── stream_proto.h
│   ├── stream_rx.cpp
│   └── stream_rx.h
├── play_mp3.py
├── project.md
├── test.ps1
└── tests
    ├── probe.py
    ├── run_host_tests.sh
    ├── runner.py
    ├── shim
    │   ├── Arduino.h
    │   ├── driver
    │   │   └── i2s.h
    │   └── freertos
    │       ├── FreeRTOS.h
    │       ├── queue.h
    │       └── task.h
    ├── sim_device.cpp
    ├── stubserial
    │   └── serial
    │       └── tools
    │           └── list_ports.py
    ├── test_levels.cpp
    ├── test_ownership.cpp
    └── test_proto.cpp
````

### ESP32-A2DP-main.zip

_(binary file, content omitted)_

### mp3/audio_levels.cpp

````cpp
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
````

### mp3/audio_levels.h

````cpp
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
````

### mp3/audio_out.cpp

````cpp
#include "audio_out.h"

#if __has_include("esp_arduino_version.h")
#include "esp_arduino_version.h"
#endif
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
#error "Uses legacy driver/i2s.h (Arduino-ESP32 core 2.x). Pin espressif32@6.9.0."
#endif

#include <math.h>
#include <string.h>
#include <atomic>

#include "driver/i2s.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "audio_levels.h"
#include "stream_player.h"

namespace audio {
namespace {

constexpr i2s_port_t I2S_PORT = I2S_NUM_0;

constexpr int DMA_BUF_COUNT  = 8;
constexpr int DMA_BUF_FRAMES = 256;
constexpr int CHUNK_FRAMES   = DMA_BUF_FRAMES;

static_assert(CHUNK_FRAMES <= sp::MAX_CHUNK_FRAMES, "render() chunk limit");
static_assert((int)STREAM_IDLE      == (int)sp::IDLE,      "state enums must match");
static_assert((int)STREAM_BUFFERING == (int)sp::BUFFERING, "state enums must match");
static_assert((int)STREAM_PLAYING   == (int)sp::PLAYING,   "state enums must match");
static_assert((int)STREAM_DRAINING  == (int)sp::DRAINING,  "state enums must match");
static_assert(NUM_BANDS == viz::BANDS, "band count must match audio_levels.h");

constexpr int TABLE_BITS = 10;
constexpr int TABLE_SIZE = 1 << TABLE_BITS;
int16_t sineTable[TABLE_SIZE];

constexpr int16_t SINE_PEAK = 4096;

static const uint16_t NOTE_HZ[] = {
    262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494,
    523, 554, 587, 622, 659, 698, 740, 784, 831, 880, 932, 988,
    1047
};
static constexpr uint8_t NOTE_REST = 0xFF;

struct Step { uint8_t note; uint16_t ms; };

static const Step SCALE[] = {
    { 0, 180}, { 2, 180}, { 4, 180}, { 5, 180},
    { 7, 180}, { 9, 180}, {11, 180}, {12, 420},
    {11, 180}, { 9, 180}, { 7, 180}, { 5, 180},
    { 4, 180}, { 2, 180}, { 0, 420},
    {NOTE_REST, 350},
};
constexpr uint8_t SEQ_LEN = sizeof(SCALE) / sizeof(SCALE[0]);

constexpr uint32_t TONE_HZ = 440;

constexpr int32_t  ENV_MAX         = 32767;
constexpr uint32_t ATTACK_SAMPLES  = 88;
constexpr uint32_t FADEOUT_SAMPLES = 110;
constexpr int32_t  ATTACK_STEP     = ENV_MAX / (int32_t)ATTACK_SAMPLES;
constexpr int32_t  FADEOUT_STEP    = ENV_MAX / (int32_t)FADEOUT_SAMPLES;
constexpr int32_t  DECAY_COEFF     = 32765;

enum CmdType : uint8_t { CMD_SET_MODE, CMD_STREAM_START, CMD_STREAM_END, CMD_STREAM_ABORT };
struct Cmd {
    uint8_t  type;
    uint8_t  arg;
    uint16_t session;
    uint32_t rate;
};

QueueHandle_t g_cmdQ = nullptr;

volatile Mode     g_mode        = MODE_OFF;
volatile uint32_t g_writeErrors = 0;
volatile uint32_t g_vizStampMs  = 0;
bool              g_started     = false;
uint32_t          g_i2sRate     = SAMPLE_RATE_HZ;

// --- ownership --------------------------------------------------------------
SourceManager g_source;
constexpr uint16_t BT_SESSION = 0x0000;   // reserved

// --- pending sample-rate from BT -------------------------------------------
std::atomic<uint32_t> g_pendingRate{0};
volatile uint8_t g_volume = 70;

int16_t g_chunk[CHUNK_FRAMES * 2];

sp::StreamPlayer       g_player;
viz::BandAnalyzer     g_analyzer;

inline int16_t sineLookup(uint32_t phase) {
    const uint32_t idx  = phase >> (32 - TABLE_BITS);
    const int32_t  frac = (int32_t)((phase >> (32 - TABLE_BITS - 16)) & 0xFFFFu);
    const int32_t  a    = sineTable[idx];
    const int32_t  b    = sineTable[(idx + 1) & (TABLE_SIZE - 1)];
    return (int16_t)(a + (((b - a) * frac) >> 16));
}

inline uint32_t phaseIncFor(uint32_t hz) {
    return (uint32_t)(((uint64_t)hz << 32) / SAMPLE_RATE_HZ);
}

void setClock(uint32_t rate) {
    if (rate == g_i2sRate) return;
    i2s_zero_dma_buffer(I2S_PORT);
    i2s_set_clk(I2S_PORT, rate, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
    g_i2sRate = rate;
}

void handleCommand(const Cmd& c) {
    switch (c.type) {
        case CMD_SET_MODE:
            if (g_mode == MODE_STREAM) break;
            if (c.arg != (uint8_t)MODE_STREAM) {
                setClock(SAMPLE_RATE_HZ);
                g_mode = (Mode)c.arg;
            }
            break;

        case CMD_STREAM_START: {
        uint32_t rate = c.rate;

        if (c.session == BT_SESSION) {
            uint32_t pr = g_pendingRate.exchange(0);
            if (pr != 0) rate = pr;
        }

        i2s_zero_dma_buffer(I2S_PORT);
        setClock(rate);
        g_analyzer.reset(rate);
        g_vizStampMs = 0;
        g_player.start(rate, c.arg, c.session);
        g_source.setReady(true);           // producer may now write
        g_mode = MODE_STREAM;
        break;
        }

        case CMD_STREAM_END:
            g_player.end();
            break;

        case CMD_STREAM_ABORT:
            g_player.abort();
            i2s_zero_dma_buffer(I2S_PORT);
            break;
    }
}

bool post(const Cmd& c) {
    if (!g_started || g_cmdQ == nullptr) return false;
    return xQueueSend(g_cmdQ, &c, 0) == pdTRUE;
}

bool streamBeginInternal(uint32_t rateHz, uint8_t channels, uint16_t session) {
    if (rateHz < 8000 || rateHz > 48000 || channels < 1 || channels > 2) return false;
    Cmd c = {};
    c.type    = CMD_STREAM_START;
    c.arg     = channels;
    c.session = session;
    c.rate    = rateHz;
    return post(c);
}

void audioTask(void*) {
    uint32_t phase           = 0;
    uint32_t phaseInc        = 0;
    int32_t  env             = 0;
    uint32_t attackRemaining = 0;
    uint32_t fadeRemaining   = 0;
    uint32_t noteRemaining   = 0;
    uint8_t  seqPos          = 0;
    bool     envAttacking    = false;
    bool     pendingAdvance  = false;
    Mode     lastMode        = MODE_OFF;

    for (;;) {
        Cmd cmd;
        while (xQueueReceive(g_cmdQ, &cmd, 0) == pdTRUE) handleCommand(cmd);

        // Apply pending sample-rate change (from BT) on the audio task only.
        const uint32_t pending = g_pendingRate.exchange(0);
        if (pending != 0 && pending != g_i2sRate) {
            setClock(pending);
            g_analyzer.reset(pending);
        }

        const Mode m = g_mode;

        if (m == MODE_STREAM) {
            if (lastMode != MODE_STREAM) {
                env = 0; fadeRemaining = 0; attackRemaining = 0; noteRemaining = 0;
                envAttacking = false; pendingAdvance = false; phaseInc = 0;
                lastMode = MODE_STREAM;
            }
            const int valid = g_player.render(g_chunk, CHUNK_FRAMES);
            if (valid > 0 && g_analyzer.feed(g_chunk, valid)) g_vizStampMs = millis();
            if (g_player.idle()) {
                g_source.release();                 // audio task confirms shutdown
                g_mode = MODE_OFF;
            }
        } else {
            // generators (unchanged from original)
            if (m != lastMode) {
                if (m == MODE_TONE) {
                    phase = 0; phaseInc = phaseIncFor(TONE_HZ);
                    env = 0; attackRemaining = ATTACK_SAMPLES; envAttacking = true;
                    fadeRemaining = 0; noteRemaining = 0; pendingAdvance = false;
                } else if (m == MODE_SCALE) {
                    seqPos = 0; const Step& s = SCALE[0];
                    phase = 0;
                    phaseInc = (s.note == NOTE_REST) ? 0 : phaseIncFor(NOTE_HZ[s.note]);
                    env = 0; envAttacking = (s.note != NOTE_REST);
                    attackRemaining = envAttacking ? ATTACK_SAMPLES : 0;
                    fadeRemaining = 0;
                    noteRemaining = (uint32_t)s.ms * SAMPLE_RATE_HZ / 1000;
                    pendingAdvance = false;
                } else {
                    if (env > 0) fadeRemaining = FADEOUT_SAMPLES;
                    pendingAdvance = false; noteRemaining = 0; envAttacking = false;
                }
                lastMode = m;
            }

            for (int i = 0; i < CHUNK_FRAMES; ++i) {
                if (fadeRemaining > 0) {
                    env -= FADEOUT_STEP; --fadeRemaining;
                    if (env <= 0 || fadeRemaining == 0) {
                        env = 0; fadeRemaining = 0;
                        if (pendingAdvance) {
                            pendingAdvance = false;
                            const Step& s = SCALE[seqPos];
                            phase    = 0;
                            phaseInc = (s.note == NOTE_REST) ? 0 : phaseIncFor(NOTE_HZ[s.note]);
                            if (s.note != NOTE_REST) {
                                env = 0; attackRemaining = ATTACK_SAMPLES; envAttacking = true;
                            } else { envAttacking = false; }
                            noteRemaining = (uint32_t)s.ms * SAMPLE_RATE_HZ / 1000;
                        } else if (m == MODE_OFF) { phaseInc = 0; }
                    }
                } else if (envAttacking) {
                    env += ATTACK_STEP; --attackRemaining;
                    if (attackRemaining == 0 || env >= ENV_MAX) {
                        env = ENV_MAX; envAttacking = false;
                    }
                } else if (env > 0) {
                    if (m == MODE_SCALE) {
                        env = (env * DECAY_COEFF) >> 15;
                        if (env < 16) env = 0;
                    }
                }

                if (m == MODE_SCALE && noteRemaining > 0) {
                    if (--noteRemaining == 0) {
                        seqPos = (uint8_t)((seqPos + 1) % SEQ_LEN);
                        pendingAdvance = true; fadeRemaining = FADEOUT_SAMPLES;
                    }
                }

                int16_t out = 0;
                if (env > 0 && phaseInc != 0) {
                    const int32_t s = sineLookup(phase);
                    out = (int16_t)((s * env) >> 15);
                }
                phase += phaseInc;
                g_chunk[2*i]     = out;
                g_chunk[2*i + 1] = out;
            }
        }

        const uint8_t vol = g_volume;
        if (vol < 100) {
            const int32_t gain = (int32_t)vol * 32768 / 100;
            for (int i = 0; i < CHUNK_FRAMES * 2; ++i)
                g_chunk[i] = (int16_t)(((int32_t)g_chunk[i] * gain) >> 15);
        }

        size_t written = 0;
        const esp_err_t err = i2s_write(I2S_PORT, g_chunk, sizeof(g_chunk),
                                        &written, portMAX_DELAY);
        if (err != ESP_OK || written != sizeof(g_chunk)) ++g_writeErrors;
    }
}

}  // namespace

bool begin() {
    if (g_started) return true;

    for (int i = 0; i < TABLE_SIZE; ++i) {
        sineTable[i] = (int16_t)lroundf(
            (float)SINE_PEAK * sinf(2.0f * (float)M_PI * (float)i / (float)TABLE_SIZE));
    }

    g_cmdQ = xQueueCreate(8, sizeof(Cmd));
    if (g_cmdQ == nullptr) return false;

    i2s_config_t cfg = {};
    cfg.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    cfg.sample_rate          = SAMPLE_RATE_HZ;
    cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
    cfg.channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT;
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
    cfg.dma_buf_count        = DMA_BUF_COUNT;
    cfg.dma_buf_len          = DMA_BUF_FRAMES;
    cfg.use_apll             = false;
    cfg.tx_desc_auto_clear   = true;
    cfg.fixed_mclk           = 0;

    if (i2s_driver_install(I2S_PORT, &cfg, 0, nullptr) != ESP_OK) return false;

    i2s_pin_config_t pins = {};
    pins.mck_io_num   = I2S_PIN_NO_CHANGE;
    pins.bck_io_num   = PIN_BCK;
    pins.ws_io_num    = PIN_LRCK;
    pins.data_out_num = PIN_DOUT;
    pins.data_in_num  = I2S_PIN_NO_CHANGE;

    if (i2s_set_pin(I2S_PORT, &pins) != ESP_OK) {
        i2s_driver_uninstall(I2S_PORT);
        return false;
    }
    i2s_zero_dma_buffer(I2S_PORT);

    g_started = true;
    if (xTaskCreatePinnedToCore(audioTask, "audio", 6144, nullptr, 5, nullptr, 0) != pdPASS) {
        g_started = false;
        i2s_driver_uninstall(I2S_PORT);
        return false;
    }
    return true;
}

void setMode(Mode m) {
    Cmd c = {}; c.type = CMD_SET_MODE; c.arg = (uint8_t)m; post(c);
}
Mode mode()            { return g_mode; }
uint32_t writeErrors() { return g_writeErrors; }
void    setVolume(uint8_t v) { if (v > 100) v = 100; g_volume = v; }
uint8_t volume()             { return g_volume; }

bool streamBegin(uint32_t rateHz, uint8_t channels, uint16_t session) {
    if (!g_source.acquire(Source::USB_SERIAL)) return false;
    if (!streamBeginInternal(rateHz, channels, session)) {
        g_source.release();
        return false;
    }
    return true;
}

void streamEnd() {
    if (g_source.owner() != Source::USB_SERIAL) return;
    Cmd c = {}; c.type = CMD_STREAM_END; post(c);
}

void streamAbort() {
    if (g_source.owner() != Source::USB_SERIAL) return;
    Cmd c = {}; c.type = CMD_STREAM_ABORT; post(c);
}

bool streamWrite(const uint8_t* data, uint32_t len) {
    if (g_source.owner() != Source::USB_SERIAL) return false;
    return g_player.write(data, len);
}

StreamStatus streamStatus() {
    const sp::Status s = g_player.status();
    StreamStatus out;
    out.state         = s.state;
    out.session       = s.session;
    out.underruns     = s.underruns;
    out.fillBytes     = s.fill;
    out.capacityBytes = s.capacity;
    return out;
}

// ---- Bluetooth ------------------------------------------------------------
bool btBegin() {
    if (!g_source.acquire(Source::BLUETOOTH)) return false;
    // The actual rate is supplied by btSetSampleRate(); start with 44100 default.
    if (!streamBeginInternal(44100, 2, BT_SESSION)) {
        g_source.release();
        return false;
    }
    return true;
}

void btEnd() {
    if (g_source.owner() != Source::BLUETOOTH) return;

    g_pendingRate.store(0);  // drop any rate scheduled by this session

    Cmd c = {};
    c.type = CMD_STREAM_END;
    post(c);
}

bool btWrite(const uint8_t* data, uint32_t len) {
    if (g_source.owner() != Source::BLUETOOTH) return false;
    if (!g_source.ready()) return false;         // audio task has not started yet
    return g_player.write(data, len);
}

bool btActive() { return g_source.owner() == Source::BLUETOOTH; }

void btSetSampleRate(uint16_t rate) {
    if (rate >= 8000 && rate <= 48000) g_pendingRate.store(rate);
}

bool levels(uint8_t bands[NUM_BANDS]) {
    if (g_player.status().state == sp::IDLE) return false;
    const uint32_t age = millis() - g_vizStampMs;
    if (g_vizStampMs == 0 || age > 120) {
        for (int i = 0; i < NUM_BANDS; ++i) bands[i] = 0;
    } else {
        g_analyzer.read(bands);
    }
    return true;
}

}  // namespace audio
````

### mp3/audio_out.h

````cpp
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
````

### mp3/bt_audio.cpp

````cpp
#include "bt_audio.h"
#include "audio_out.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "BluetoothA2DPSink.h"

namespace bt {
namespace {

BluetoothA2DPSink* g_sink       = nullptr;
SemaphoreHandle_t  g_metaMutex = nullptr;

volatile bool g_connected = false;
volatile bool g_paused = false;
volatile bool g_metaDirty = false;

char g_title[96]  = {0};
char g_artist[64] = {0};

// Called on the Bluetooth task for each decoded SBC frame.
void dataCallback(const uint8_t* data, uint32_t length) {
    // Defensive: the library should already deliver frame-aligned data.
    if (length < 4) return;
    length -= length % 4;                    // stereo 16-bit = 4 bytes/frame
    audio::btWrite(data, length);
}

// Called on the BT task for A2DP connection-state changes.
void connectionCallback(esp_a2d_connection_state_t state, void* /*ptr*/) {
    if (state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
        g_connected = true;
        g_paused = false;
        audio::btBegin();                    // rate supplied by sample-rate callback
    } else if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
        g_connected = false;
        g_paused = false;
        audio::btEnd();
        if (g_metaMutex && xSemaphoreTake(g_metaMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            g_title[0]  = 0;
            g_artist[0] = 0;
            g_metaDirty = true;
            xSemaphoreGive(g_metaMutex);
        }
    }
}

// Called on the BT task when AVRCP metadata arrives.
void metadataCallback(uint8_t id, const uint8_t* text) {
    if (!g_metaMutex || !text) return;
    if (xSemaphoreTake(g_metaMutex, pdMS_TO_TICKS(10)) != pdTRUE) return;
    if (id == ESP_AVRC_MD_ATTR_TITLE) {          // 0x01
        strncpy(g_title, (const char*)text, sizeof(g_title) - 1);
        g_title[sizeof(g_title) - 1] = 0;
        g_metaDirty = true;
    } else if (id == ESP_AVRC_MD_ATTR_ARTIST) {   // 0x02
        strncpy(g_artist, (const char*)text, sizeof(g_artist) - 1);
        g_artist[sizeof(g_artist) - 1] = 0;
        g_metaDirty = true;
    }
    xSemaphoreGive(g_metaMutex);
}

// Called on the BT task when the negotiated sample rate changes.
void sampleRateCallback(uint16_t rate) {
    audio::btSetSampleRate(rate);
}

}  // namespace

bool begin() {
    if (g_sink) return true;

    g_metaMutex = xSemaphoreCreateMutex();
    if (!g_metaMutex) return false;

    g_sink = new BluetoothA2DPSink();
    if (!g_sink) { vSemaphoreDelete(g_metaMutex); g_metaMutex = nullptr; return false; }

    // false = do NOT install the library's internal I2S driver.
    g_sink->set_stream_reader(dataCallback, false);
    g_sink->set_on_connection_state_changed(connectionCallback);
    g_sink->set_avrc_metadata_callback(metadataCallback);
    g_sink->set_avrc_metadata_attribute_mask(ESP_AVRC_MD_ATTR_TITLE |
                                             ESP_AVRC_MD_ATTR_ARTIST);
    g_sink->set_sample_rate_callback(sampleRateCallback);
    g_sink->start("Mochi");
    return true;
}

void end() {
    if (g_sink) { g_sink->end(); delete g_sink; g_sink = nullptr; }
    if (g_metaMutex) { vSemaphoreDelete(g_metaMutex); g_metaMutex = nullptr; }
}

Snapshot poll() {
    Snapshot s;
    s.connected       = g_connected;
    s.metadataChanged = false;
    s.title[0]        = 0;
    s.artist[0]       = 0;
    if (g_metaMutex && xSemaphoreTake(g_metaMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (g_metaDirty) {
            s.metadataChanged = true;
            strncpy(s.title,  g_title,  sizeof(s.title)  - 1);
            strncpy(s.artist, g_artist, sizeof(s.artist) - 1);
            s.title[sizeof(s.title) - 1]   = 0;
            s.artist[sizeof(s.artist) - 1] = 0;
            g_metaDirty = false;
        }
        xSemaphoreGive(g_metaMutex);
    }
    return s;
}

bool isPaused() { return g_paused; }
void pause()    { if (g_sink) { g_sink->pause(); g_paused = true;  } }
void resume()   { if (g_sink) { g_sink->play();  g_paused = false; } }

}  // namespace bt
````

### mp3/bt_audio.h

````cpp
#pragma once
#include <stdint.h>

namespace bt {

bool begin();
void end();

struct Snapshot {
    bool connected;
    bool metadataChanged;
    char title[96];
    char artist[64];
};

Snapshot poll();

bool isPaused();
void pause();
void resume();

}  // namespace bt
````

### mp3/mp3.ino

````cpp
/*
 * Retro 128x64 OLED Audio Visualizer + laptop/BT-streamed MP3 player
 * Milestone 05 â€“ A2DP Sink + Mochi UI
 */

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

#include "audio_out.h"
#include "stream_rx.h"
#include "player_ui.h"
#include "bt_audio.h"

// -----------------------------------------------------------------------------
// Verified hardware configuration
// -----------------------------------------------------------------------------
static constexpr uint8_t SDA_PIN  = 21;
static constexpr uint8_t SCL_PIN  = 22;
static constexpr uint8_t I2C_ADDR = 0x3C;

static constexpr bool    PLAY_SCALE_AT_BOOT = false;
static constexpr bool    SERIAL_PLOTTER_DEMO = false;

static constexpr uint8_t BOOT_BTN_PIN    = 0;
static constexpr uint32_t BTN_DEBOUNCE_MS = 30;

// ---- Rotary encoder (EC11) ----
static constexpr uint8_t  ENC_PIN_A  = 32;
static constexpr uint8_t  ENC_PIN_B  = 33;
static constexpr uint8_t  ENC_PIN_SW = 4;
static constexpr uint32_t ENC_SW_DEBOUNCE_MS = 25;
static constexpr int32_t  ENC_STEPS_PER_DETENT = 4;   // try 2 if too coarse
static constexpr int      ENC_VOL_STEP = 2;

static volatile int32_t encDelta     = 0;
static volatile uint8_t encLastState = 0;

DRAM_ATTR static const int8_t ENC_TBL[16] = {
     0, +1, -1,  0,
    -1,  0,  0, +1,
    +1,  0,  0, -1,
     0, -1, +1,  0
};

static void IRAM_ATTR encoderIsr() {
    const uint8_t cur = (uint8_t)((digitalRead(ENC_PIN_A) << 1) | digitalRead(ENC_PIN_B));
    encDelta += ENC_TBL[(encLastState << 2) | cur];
    encLastState = cur;
}

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
PlayerUI ui;

// -----------------------------------------------------------------------------
// Synthetic visualizer (used only by built-in test generators)
// -----------------------------------------------------------------------------
static constexpr uint8_t NUM_BANDS = 10;
static_assert(NUM_BANDS == audio::NUM_BANDS, "visualizer and audio analyzer band counts differ");

static constexpr uint8_t BEAT_PERIOD_FRAMES = 14;
static constexpr uint8_t BEAT_DECAY         = 18;

static constexpr uint32_t FRAME_INTERVAL_MS = 33;  // 30 FPS target

static uint16_t phase[NUM_BANDS];
static uint16_t lfsr[NUM_BANDS];
static uint8_t  env[NUM_BANDS];

static uint8_t  beatEnv      = 0;
static uint8_t  beatTimer    = 0;
static uint32_t lastFrame    = 0;

static const uint8_t SINE_TABLE[64] = {
    128, 140, 153, 165, 177, 188, 199, 209,
    218, 226, 234, 240, 245, 250, 253, 255,
    255, 255, 253, 250, 245, 240, 234, 226,
    218, 209, 199, 188, 177, 165, 153, 140,
    128, 116, 103,  91,  79,  68,  57,  47,
     38,  30,  22,  16,  11,   6,   3,   1,
      1,   1,   3,   6,  11,  16,  22,  30,
     38,  47,  57,  68,  79,  91, 103, 116
};

static const uint16_t BAND_SPEED[NUM_BANDS] = {
    1800, 2100, 2400, 2700, 3000, 3300, 3600, 3900, 4200, 4500
};
static const uint8_t BAND_GAIN[NUM_BANDS] = {
    235, 220, 205, 190, 178, 166, 156, 146, 136, 126
};
static const uint8_t BEAT_GAIN[NUM_BANDS] = {
    100, 88, 72, 58, 44, 32, 24, 18, 13, 10
};

static void updateVisualizer();
static void pollBootButton(uint32_t now);
static void pollEncoder(uint32_t now);
static void announceMode(audio::Mode m);
static void onConsoleChar(char c);
static void feedUI();
static void processBtSnapshot();

// -----------------------------------------------------------------------------
void setup() {
    stream::setConsoleHandler(onConsoleChar);
    const bool rxOk = stream::begin();

    pinMode(BOOT_BTN_PIN, INPUT_PULLUP);

    pinMode(ENC_PIN_A,  INPUT_PULLUP);
    pinMode(ENC_PIN_B,  INPUT_PULLUP);
    pinMode(ENC_PIN_SW, INPUT_PULLUP);
    encLastState = (uint8_t)((digitalRead(ENC_PIN_A) << 1) | digitalRead(ENC_PIN_B));
    attachInterrupt(digitalPinToInterrupt(ENC_PIN_A), encoderIsr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_PIN_B), encoderIsr, CHANGE);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);

    u8g2.setI2CAddress(I2C_ADDR << 1);
    u8g2.begin();
    u8g2.setPowerSave(0);
    Wire.setClock(400000);

    ui.begin(&u8g2);

    for (uint8_t i = 0; i < NUM_BANDS; ++i) {
        lfsr[i] = (uint16_t)(0xACE1u + (uint16_t)i * 0x1234u);
        if (lfsr[i] == 0) lfsr[i] = 0xBEEFu;
    }

    lastFrame = millis();

    if (audio::begin()) {
        Serial.println(F("[audio] I2S OK"));
        if (PLAY_SCALE_AT_BOOT) {
            audio::setMode(audio::MODE_SCALE);
            announceMode(audio::MODE_SCALE);
        }
    } else {
        Serial.println(F("[audio] ERROR: I2S init failed (display continues)"));
    }
    if (!rxOk) Serial.println(F("[stream] ERROR: receiver task failed to start"));
    Serial.print(F("[stream] ready, "));
    Serial.print(stream::BAUD);
    Serial.println(F(" baud. Run play_mp3.py on the laptop."));
    Serial.println(F("[audio] 's'=scale '1'=440Hz '0'=off 't'=toggle, BOOT=cycle modes"));
    Serial.println(F("[bt] will start in ~0.5s: pair with 'Mochi'"));
}

void loop() {
    const uint32_t now = millis();

    pollBootButton(now);
    pollEncoder(now);

    static bool btInited = false;
    if (!btInited && now > 500) {
        btInited = true;
        if (bt::begin()) Serial.println(F("[bt] A2DP ready, advertising as 'Mochi'"));
        else             Serial.println(F("[bt] ERROR: A2DP init failed"));
    }

    processBtSnapshot();

    if ((uint32_t)(now - lastFrame) < FRAME_INTERVAL_MS) {
        return;
    }
    lastFrame = now;

    if (SERIAL_PLOTTER_DEMO) {
        float plotterSine = sin(now * 2.0 * PI / 1000.0) * 100.0;
        Serial.println(plotterSine);
    }

    const audio::Mode m = audio::mode();
    if (m == audio::MODE_TONE || m == audio::MODE_SCALE) updateVisualizer();

    feedUI();
    ui.update();
}

static void processBtSnapshot() {
    const bt::Snapshot s = bt::poll();
    ui.setBluetoothConnected(s.connected);
    if (s.metadataChanged) {
        if (s.title[0] || s.artist[0]) ui.setTrackInfo(s.title, s.artist);
        else                            ui.clearTrackInfo();
    }
}

static void feedUI() {
    static bool usbTitleSet = false;

    uint8_t uiLevels[NUM_BANDS];
    memset(uiLevels, 0, sizeof(uiLevels));

    const audio::StreamStatus ss = audio::streamStatus();
    const bool streamActive = (ss.state != audio::STREAM_IDLE);
    const audio::Mode m = audio::mode();
    if (!streamActive) usbTitleSet = false;
    if (streamActive) {
        uint8_t live[NUM_BANDS];
        if (audio::levels(live)) {
            for (int i = 0; i < NUM_BANDS; ++i)
                uiLevels[i] = (uint8_t)((uint16_t)live[i] * 100u / 255u);
        }
        ui.setPlaying(!(audio::btActive() && bt::isPaused()));
        if (!bt::poll().connected && !audio::btActive()) {
            if (!usbTitleSet) {
                ui.setTrackInfo("USB STREAM", "playing");
                usbTitleSet = true;
            }
        }
    } else if (m == audio::MODE_TONE || m == audio::MODE_SCALE) {
        for (int i = 0; i < NUM_BANDS; ++i)
            uiLevels[i] = (uint8_t)((uint16_t)env[i] * 100u / 255u);
        ui.setPlaying(true);
        static audio::Mode lastTestMode = audio::MODE_OFF;
        if (m != lastTestMode) {
            ui.setTrackInfo(m == audio::MODE_TONE ? "TEST TONE" : "SCALE",
                            "built-in generator");
            lastTestMode = m;
        }
    } else {
        ui.setPlaying(false);
        if (!bt::poll().connected) ui.clearTrackInfo();
    }
    ui.setVisualizerLevels(uiLevels);
}

static bool streamBusy() {
    return audio::streamStatus().state != audio::STREAM_IDLE;
}

static void announceMode(audio::Mode m) {
    if (streamBusy()) return;
    Serial.print(F("[audio] mode: "));
    switch (m) {
        case audio::MODE_OFF:    Serial.println(F("OFF"));      break;
        case audio::MODE_TONE:   Serial.println(F("440 Hz"));   break;
        case audio::MODE_SCALE:  Serial.println(F("SCALE"));    break;
        case audio::MODE_STREAM: Serial.println(F("STREAM"));   break;
    }
}

static void onConsoleChar(char c) {
    if      (c == 's' || c == 'S') { audio::setMode(audio::MODE_SCALE); announceMode(audio::MODE_SCALE); }
    else if (c == '1')             { audio::setMode(audio::MODE_TONE);  announceMode(audio::MODE_TONE); }
    else if (c == '0')             { audio::setMode(audio::MODE_OFF);   announceMode(audio::MODE_OFF); }
    else if (c == 't' || c == 'T') {
        const audio::Mode next = (audio::mode() == audio::MODE_TONE) ? audio::MODE_OFF
                                                                     : audio::MODE_TONE;
        audio::setMode(next);
        announceMode(next);
    } else if (c == 'e' || c == 'E') {
        Serial.print(F("[audio] i2s write errors: "));
        Serial.println(audio::writeErrors());
    }
}

static void pollBootButton(uint32_t now) {
    static bool     lastRaw   = true;
    static bool     stable    = true;
    static uint32_t changedAt = 0;
    const bool raw = digitalRead(BOOT_BTN_PIN) != LOW;
    if (raw != lastRaw) {
        lastRaw   = raw;
        changedAt = now;
    } else if (raw != stable && (uint32_t)(now - changedAt) >= BTN_DEBOUNCE_MS) {
        stable = raw;
        if (!stable && !streamBusy()) {
            const auto m = audio::mode();
            audio::Mode next;
            if      (m == audio::MODE_OFF)   next = audio::MODE_TONE;
            else if (m == audio::MODE_TONE)  next = audio::MODE_SCALE;
            else                             next = audio::MODE_OFF;
            audio::setMode(next);
            announceMode(next);
        }
    }
}

static void updateVisualizer() {
    if (beatTimer > 0) { --beatTimer; }
    else { beatTimer = BEAT_PERIOD_FRAMES; beatEnv = 255; }
    if (beatEnv > BEAT_DECAY) beatEnv -= BEAT_DECAY;
    else                      beatEnv = 0;

    for (uint8_t i = 0; i < NUM_BANDS; ++i) {
        phase[i] = (uint16_t)(phase[i] + BAND_SPEED[i]);
        const uint16_t lsb = lfsr[i] & 1u;
        lfsr[i] = (uint16_t)((lfsr[i] >> 1) ^ (lsb ? 0xB400u : 0x0000u));
        const uint8_t s = SINE_TABLE[(phase[i] >> 10) & 63u];
        const uint8_t n = (uint8_t)(lfsr[i] & 0xFFu);
        const uint16_t base = ((uint16_t)s * 3u + n) >> 2;
        uint16_t target = (base * BAND_GAIN[i]) >> 8;
        target += ((uint16_t)beatEnv * BEAT_GAIN[i]) >> 8;
        if (target > 255u) target = 255u;
        const uint8_t t = (uint8_t)target;

        if (t > env[i]) {
            uint8_t step = ((t - env[i]) * 6u) >> 3;
            if (step == 0) step = 1;
            env[i] = (uint8_t)(env[i] + step);
            if (env[i] > t) env[i] = t;
        } else if (t < env[i]) {
            uint8_t step = (env[i] - t) >> 4;
            if (step == 0) step = 1;
            env[i] = (uint8_t)(env[i] - step);
            if (env[i] < t) env[i] = t;
        }
    }
}

static void pollEncoder(uint32_t now) {
    noInterrupts();
    const int32_t d = encDelta;
    encDelta = 0;
    interrupts();

    if (d != 0) {
        static int32_t accum = 0;
        accum += d;
        while (accum >= ENC_STEPS_PER_DETENT) {
            accum -= ENC_STEPS_PER_DETENT;
            int v = audio::volume() + ENC_VOL_STEP;
            if (v > 100) v = 100;
            audio::setVolume((uint8_t)v);
        }
        while (accum <= -ENC_STEPS_PER_DETENT) {
            accum += ENC_STEPS_PER_DETENT;
            int v = audio::volume() - ENC_VOL_STEP;
            if (v < 0) v = 0;
            audio::setVolume((uint8_t)v);
        }
    }

    static bool     lastRaw   = true;
    static bool     stable    = true;
    static uint32_t changedAt = 0;
    const bool raw = digitalRead(ENC_PIN_SW) != LOW;
    if (raw != lastRaw) {
        lastRaw   = raw;
        changedAt = now;
    } else if (raw != stable && (uint32_t)(now - changedAt) >= ENC_SW_DEBOUNCE_MS) {
        stable = raw;
        if (!stable && audio::btActive()) {      // press, Bluetooth only
            if (bt::isPaused()) bt::resume();
            else                bt::pause();
        }
    }
}
````

### mp3/player_ui.cpp

````cpp
#include "player_ui.h"
#include <string.h>
#include <math.h>

#define SCREEN_W 128
#define SCREEN_H 64

#define UI_FONT_TITLE   u8g2_font_6x12_tf
#define UI_FONT_ARTIST  u8g2_font_5x7_tf
#define UI_FONT_TINY    u8g2_font_4x6_tf

#define BOOT_MS             1400
#define ENTRY_WIPE_MS        200
#define SCROLL_SPEED_PX_S     22.0f
#define SCROLL_GAP_PX         12
#define BLINK_INTERVAL_MS   3400
#define BLINK_DURATION_MS    140
#define FRAME_MIN_MS          33          // 30 FPS target
#define TITLE_AVAIL_W        120
#define ARTIST_AVAIL_W       120

#define VIZ_ATTACK           0.55f
#define VIZ_RELEASE          0.14f
#define VIZ_PEAK_FALL_PER_S  60.0f

const uint8_t PlayerUI::mochi_frames_[4][20] = {
    { 0x1E,0x00, 0x3F,0x00, 0x7F,0x00, 0xE4,0xC0, 0xE4,0xC0,
      0xFF,0xC0, 0xF3,0xC0, 0x7F,0x00, 0x3F,0x00, 0x1E,0x00 },
    { 0x1E,0x00, 0x3F,0x00, 0x7F,0x00, 0xFF,0xC0, 0xE4,0xC0,
      0xFF,0xC0, 0xF3,0xC0, 0x7F,0x00, 0x3F,0x00, 0x1E,0x00 },
    { 0x1E,0x00, 0x3F,0x00, 0x7F,0x00, 0xE4,0xC0, 0xE4,0xC0,
      0xFF,0xC0, 0xE1,0xC0, 0x7F,0x00, 0x3F,0x00, 0x1E,0x00 },
    { 0x1E,0x00, 0x3F,0x00, 0x7F,0x00, 0xE6,0x40, 0xE6,0x40,
      0xFF,0xC0, 0xF3,0xC0, 0x7F,0x00, 0x3F,0x00, 0x1E,0x00 },
};

static const uint8_t kBTIcon[7]    = { 0x08,0x18,0x54,0x14,0x54,0x18,0x08 };
static const uint8_t kPlayIcon[7]  = { 0x20,0x30,0x38,0x3C,0x38,0x30,0x20 };
static const uint8_t kPauseIcon[7] = { 0x6C,0x6C,0x6C,0x6C,0x6C,0x6C,0x6C };

void PlayerUI::begin(U8G2* display) {
    u8g2_ = display;
    screen_ = UIScreen::BOOT;
    screenEnteredMs_ = millis();
    nextBlinkMs_ = millis() + 2400;
    lastRenderMs_ = millis() - FRAME_MIN_MS;
}

void PlayerUI::enterScreen(UIScreen s) {
    if (screen_ == s) return;
    screen_ = s;
    screenEnteredMs_ = millis();
    if (s != UIScreen::NOW_PLAYING && s != UIScreen::PAUSED) {
        titleScroll_  = 0;
        artistScroll_ = 0;
    }
}

UIScreen PlayerUI::computeAutoScreen() const {
    if (!btConnected_ && !playing_ && !hasTitle_ && !hasArtist_) return UIScreen::IDLE;
    if (!hasTitle_ && !hasArtist_) return UIScreen::NO_METADATA;
    return playing_ ? UIScreen::NOW_PLAYING : UIScreen::PAUSED;
}

void PlayerUI::setTrackInfo(const char* title, const char* artist) {
    if (title && *title) {
        strncpy(title_, title, sizeof(title_) - 1);
        title_[sizeof(title_) - 1] = 0;
        hasTitle_ = true;
    } else { title_[0] = 0; hasTitle_ = false; }
    if (artist && *artist) {
        strncpy(artist_, artist, sizeof(artist_) - 1);
        artist_[sizeof(artist_) - 1] = 0;
        hasArtist_ = true;
    } else { artist_[0] = 0; hasArtist_ = false; }
    if (u8g2_) {
        u8g2_->setFont(UI_FONT_TITLE);
        titleWidth_  = u8g2_->getUTF8Width(title_);
        u8g2_->setFont(UI_FONT_ARTIST);
        artistWidth_ = u8g2_->getUTF8Width(artist_);
    }
    titleScroll_ = 0; artistScroll_ = 0;
}

void PlayerUI::clearTrackInfo() {
    title_[0] = 0; artist_[0] = 0;
    hasTitle_ = hasArtist_ = false;
    titleWidth_ = artistWidth_ = 0;
    titleScroll_ = artistScroll_ = 0;
}

void PlayerUI::setPlaying(bool playing) { playing_ = playing; }

void PlayerUI::setBluetoothConnected(bool connected) {
    bool was = btConnected_;
    btConnected_ = connected;
    if (!was && connected && screen_ == UIScreen::PAIRING) {
        searching_ = false;
        autoMode_  = true;
        enterScreen(computeAutoScreen());
    }
}

void PlayerUI::setVisualizerLevels(const uint8_t levels[10]) {
    for (int i = 0; i < 10; ++i)
        vizTarget_[i] = levels[i] > 100 ? 100 : levels[i];
}

void PlayerUI::showPairing()  { autoMode_ = false; searching_ = true; enterScreen(UIScreen::PAIRING); }
void PlayerUI::setScreen(UIScreen s) { autoMode_ = false; enterScreen(s); }
void PlayerUI::setAutoMode(bool enabled) {
    autoMode_ = enabled;
    if (enabled) enterScreen(computeAutoScreen());
}

void PlayerUI::update() {
    if (!u8g2_) return;
    uint32_t now = millis();
    if (now - lastRenderMs_ < FRAME_MIN_MS) return;
    uint32_t dt_ms = now - lastRenderMs_;
    lastRenderMs_ = now;
    float dt = dt_ms / 1000.0f;

    if (screen_ == UIScreen::BOOT && (now - screenEnteredMs_) >= BOOT_MS)
        enterScreen(computeAutoScreen());

    if (autoMode_ && screen_ != UIScreen::BOOT) {
        UIScreen s = computeAutoScreen();
        if (s != screen_) enterScreen(s);
    }

    updateScroll(dt);
    updateVisualizer(dt);

    u8g2_->clearBuffer();
    render();
    u8g2_->sendBuffer();
}

void PlayerUI::updateScroll(float dt) {
    if (screen_ != UIScreen::NOW_PLAYING && screen_ != UIScreen::NO_METADATA) return;
    float adv = SCROLL_SPEED_PX_S * dt;
    if (titleWidth_ > TITLE_AVAIL_W) {
        titleScroll_ += adv;
        if (titleScroll_ > (float)(titleWidth_ + SCROLL_GAP_PX)) titleScroll_ = 0.0f;
    }
    if (artistWidth_ > ARTIST_AVAIL_W) {
        artistScroll_ += adv;
        if (artistScroll_ > (float)(artistWidth_ + SCROLL_GAP_PX)) artistScroll_ = 0.0f;
    }
}

void PlayerUI::updateVisualizer(float dt) {
    bool active = (screen_ == UIScreen::NOW_PLAYING) ||
                  (screen_ == UIScreen::NO_METADATA);
    uint32_t now = millis();
    for (int i = 0; i < 10; ++i) {
        float target;
        if (active) {
            target = (float)vizTarget_[i];
        } else {
            float t = now * 0.001f;
            target = 8.0f + 5.0f * sinf(t * 1.2f + i * 0.65f);
            if (target < 0.0f) target = 0.0f;
        }
        if (target > vizLevel_[i]) vizLevel_[i] += (target - vizLevel_[i]) * VIZ_ATTACK;
        else                       vizLevel_[i] += (target - vizLevel_[i]) * VIZ_RELEASE;

        if (vizLevel_[i] >= vizPeak_[i]) vizPeak_[i] = vizLevel_[i];
        else {
            vizPeak_[i] -= VIZ_PEAK_FALL_PER_S * dt;
            if (vizPeak_[i] < vizLevel_[i]) vizPeak_[i] = vizLevel_[i];
            if (vizPeak_[i] < 0.0f) vizPeak_[i] = 0.0f;
        }
    }
}

uint8_t PlayerUI::pickMochiFrame(uint32_t now) const {
    return ((now % BLINK_INTERVAL_MS) < BLINK_DURATION_MS) ? 1 : 0;
}

void PlayerUI::drawStatusBar(bool paused) {
    if (hasTitle_ || hasArtist_) drawPlayPauseIcon(2, 2, !paused);
    if (btConnected_)            drawBluetoothIcon(SCREEN_W - 9, 2, millis());
}

void PlayerUI::drawPlayPauseIcon(int x, int y, bool playing) {
    u8g2_->drawXBMP(x, y, 7, 7, playing ? kPlayIcon : kPauseIcon);
}

void PlayerUI::drawBluetoothIcon(int x, int y, uint32_t now) {
    if (searching_) {
        if (((now / 400) & 1) == 0) u8g2_->drawXBMP(x, y, 7, 7, kBTIcon);
    } else {
        u8g2_->drawXBMP(x, y, 7, 7, kBTIcon);
    }
}

void PlayerUI::drawMochi(int x, int y, uint8_t frame) {
    if (frame > 3) frame = 0;
    u8g2_->drawXBMP(x, y, 10, 10, mochi_frames_[frame]);
}

void PlayerUI::drawMarquee(int x, int baseline, int availW,
                           const char* text, int16_t w, float scroll,
                           const uint8_t* font) {
    if (!text || !*text) return;
    u8g2_->setClipWindow(x, baseline - 11, x + availW - 1, baseline + 2);
    u8g2_->setFont((const uint8_t*)font);
    if (w <= availW) {
        int cx = x + (availW - w) / 2;
        u8g2_->drawUTF8(cx, baseline, text);
    } else {
        int dx = x - (int)scroll;
        u8g2_->drawUTF8(dx, baseline, text);
        u8g2_->drawUTF8(dx + w + SCROLL_GAP_PX, baseline, text);
    }
    u8g2_->setMaxClipWindow();
}

void PlayerUI::drawVisualizer(int baseY, int maxH, bool active, uint32_t now) {
    (void)now; (void)active;
    const int barW = 10, gap = 2;
    const int totalW = 10 * barW + 9 * gap;
    const int startX = (SCREEN_W - totalW) / 2;
    for (int i = 0; i < 10; ++i) {
        int h     = (int)(vizLevel_[i] * maxH / 100.0f);
        int peakH = (int)(vizPeak_[i]  * maxH / 100.0f);
        int x     = startX + i * (barW + gap);
        if (h > 0) u8g2_->drawBox(x, baseY + maxH - h, barW, h);
        if (peakH > h + 1) u8g2_->drawHLine(x, baseY + maxH - peakH, barW);
    }
}

void PlayerUI::drawEntryWipe() {
    uint32_t elapsed = millis() - screenEnteredMs_;
    if (elapsed >= ENTRY_WIPE_MS) return;
    uint8_t revealH = (uint8_t)((float)elapsed / ENTRY_WIPE_MS * SCREEN_H);
    if (revealH >= SCREEN_H) return;
    u8g2_->setDrawColor(0);
    u8g2_->drawBox(0, 0, SCREEN_W, SCREEN_H - revealH);
    u8g2_->setDrawColor(1);
}

void PlayerUI::render() {
    u8g2_->setDrawColor(1);
    u8g2_->setFontPosBaseline();
    switch (screen_) {
        case UIScreen::BOOT:        renderBoot();             break;
        case UIScreen::PAIRING:     renderPairing();          break;
        case UIScreen::NOW_PLAYING: renderNowPlaying(false);  break;
        case UIScreen::PAUSED:      renderNowPlaying(true);   break;
        case UIScreen::NO_METADATA: renderNoMetadata(false);  break;
        case UIScreen::IDLE:        renderIdle();             break;
    }
    if (screen_ != UIScreen::BOOT) drawEntryWipe();
}

void PlayerUI::renderBoot() {
    uint32_t t = millis() - screenEnteredMs_;
    uint8_t frame = (t < 400) ? 1 : 0;
    if (t > 900 && t < 1040) frame = 1;
    drawMochi((SCREEN_W - 10) / 2, 20, frame);

    const char* word = "M O C H I";
    u8g2_->setFont(UI_FONT_TINY);
    int16_t ww = u8g2_->getUTF8Width(word);
    int cx = (SCREEN_W - ww) / 2;
    int baseline = 46;
    if (t >= 500) {
        float p = (t - 500) / 400.0f;
        if (p > 1.0f) p = 1.0f;
        float ease = 1.0f - powf(1.0f - p, 3.0f);
        int offset = -(int)((1.0f - ease) * 8.0f);
        u8g2_->drawUTF8(cx, baseline + offset, word);
    }
}

void PlayerUI::renderPairing() {
    uint32_t now = millis();
    if (((now / 400) & 1) == 0)
        u8g2_->drawXBMP((SCREEN_W - 7) / 2, 4, 7, 7, kBTIcon);

    uint8_t mf = ((now / 900) & 1) ? 3 : 2;
    drawMochi((SCREEN_W - 10) / 2, 18, mf);

    u8g2_->setFont(UI_FONT_ARTIST);
    const char* l1 = btConnected_ ? "CONNECTED" : "PAIRING...";
    int16_t w1 = u8g2_->getUTF8Width(l1);
    u8g2_->drawUTF8((SCREEN_W - w1) / 2, 40, l1);

    u8g2_->setFont(UI_FONT_TINY);
    const char* l2 = btConnected_ ? "ready" : "on your phone";
    int16_t w2 = u8g2_->getUTF8Width(l2);
    u8g2_->drawUTF8((SCREEN_W - w2) / 2, 52, l2);
}

void PlayerUI::renderNowPlaying(bool paused) {
    drawStatusBar(paused);
    drawMarquee(4, 24, TITLE_AVAIL_W, title_, titleWidth_,
                titleScroll_, (const uint8_t*)UI_FONT_TITLE);
    drawMarquee(4, 35, ARTIST_AVAIL_W, artist_, artistWidth_,
                artistScroll_, (const uint8_t*)UI_FONT_ARTIST);
    drawVisualizer(40, 24, !paused, millis());
}

void PlayerUI::renderNoMetadata(bool paused) {
    drawStatusBar(paused);
    uint32_t now = millis();
    bool vis = ((now / 600) & 1) == 0;
    u8g2_->setFont(UI_FONT_TITLE);
    const char* dash = vis ? "\xE2\x80\x94 \xE2\x80\x94 \xE2\x80\x94"
                           : "\xC2\xB7 \xC2\xB7 \xC2\xB7";
    int16_t w = u8g2_->getUTF8Width(dash);
    u8g2_->drawUTF8((SCREEN_W - w) / 2, 24, dash);

    u8g2_->setFont(UI_FONT_ARTIST);
    const char* unk = "Unknown Artist";
    int16_t wu = u8g2_->getUTF8Width(unk);
    u8g2_->drawUTF8((SCREEN_W - wu) / 2, 35, unk);
    drawVisualizer(40, 24, true, now);
}

void PlayerUI::renderIdle() {
    uint32_t now = millis();
    float bob = sinf(now * 0.0015f * 1.5f);
    int my = 18 + (bob > 0 ? 1 : 0);
    drawMochi((SCREEN_W - 10) / 2, my, pickMochiFrame(now));

    uint32_t zt = now % 2100;
    for (int k = 0; k < 3; ++k) {
        uint32_t zone = (zt + k * 700) % 2100;
        if (zone < 900) {
            int zx = (SCREEN_W - 10) / 2 + 8 + (int)(zone * 0.008f);
            int zy = my + 6 - (int)(zone * 0.012f);
            u8g2_->setFont(UI_FONT_TINY);
            u8g2_->drawStr(zx, zy, "z");
        }
    }

    u8g2_->setFont(UI_FONT_ARTIST);
    const char* wm = "MOCHI";
    int16_t ww = u8g2_->getUTF8Width(wm);
    u8g2_->drawUTF8((SCREEN_W - ww) / 2, 44, wm);

    u8g2_->setFont(UI_FONT_TINY);
    const char* hint = "connect phone";
    int16_t wh = u8g2_->getUTF8Width(hint);
    u8g2_->drawUTF8((SCREEN_W - wh) / 2, 58, hint);
}
````

### mp3/player_ui.h

````cpp
#ifndef PLAYER_UI_H
#define PLAYER_UI_H

#include <Arduino.h>
#include <U8g2lib.h>

enum class UIScreen : uint8_t {
    BOOT = 0,
    PAIRING,
    NOW_PLAYING,
    PAUSED,
    NO_METADATA,
    IDLE
};

class PlayerUI {
public:
    void begin(U8G2* display);
    void update();

    void setTrackInfo(const char* title, const char* artist);
    void clearTrackInfo();
    void setPlaying(bool playing);
    void setBluetoothConnected(bool connected);
    void setVisualizerLevels(const uint8_t levels[10]);

    void showPairing();
    void setScreen(UIScreen s);
    void setAutoMode(bool enabled);
    UIScreen currentScreen() const { return screen_; }

private:
    U8G2* u8g2_ = nullptr;

    UIScreen screen_ = UIScreen::BOOT;
    bool autoMode_   = true;
    bool searching_  = false;

    uint32_t screenEnteredMs_ = 0;
    uint32_t lastRenderMs_    = 0;
    uint32_t nextBlinkMs_     = 0;

    char title_[96]  = {0};
    char artist_[64] = {0};
    bool hasTitle_   = false, hasArtist_ = false;
    bool playing_    = false;
    bool btConnected_ = false;

    int16_t titleWidth_ = 0, artistWidth_ = 0;
    float   titleScroll_ = 0.0f, artistScroll_ = 0.0f;

    uint8_t vizTarget_[10] = {0};
    float   vizLevel_[10]  = {0.0f};
    float   vizPeak_[10]   = {0.0f};

    void enterScreen(UIScreen s);
    UIScreen computeAutoScreen() const;
    void updateVisualizer(float dt);
    void updateScroll(float dt);
    void render();
    void renderBoot();
    void renderPairing();
    void renderNowPlaying(bool paused);
    void renderNoMetadata(bool paused);
    void renderIdle();
    void drawStatusBar(bool paused);
    void drawMochi(int x, int y, uint8_t frame);
    void drawBluetoothIcon(int x, int y, uint32_t now);
    void drawPlayPauseIcon(int x, int y, bool playing);
    void drawVisualizer(int baseY, int maxH, bool active, uint32_t now);
    void drawMarquee(int x, int baseline, int availW,
                     const char* text, int16_t w, float scroll,
                     const uint8_t* font);
    uint8_t pickMochiFrame(uint32_t now) const;
    void drawEntryWipe();

    static const uint8_t mochi_frames_[4][20];
};

#endif
````

### mp3/source_manager.h

````cpp
#pragma once
// Explicit audio-source ownership for the single-producer / single-consumer
// StreamPlayer.  Pure C++11, no Arduino/FreeRTOS dependencies -> testable on PC.

#include <atomic>
#include <stdint.h>

namespace audio {

enum class Source : uint32_t { NONE = 0, USB_SERIAL = 1, BLUETOOTH = 2 };

class SourceManager {
 public:
  bool acquire(Source src) {
    Source expected = Source::NONE;
    return owner_.compare_exchange_strong(expected, src,
                                          std::memory_order_acq_rel,
                                          std::memory_order_acquire);
  }
  void setReady(bool r) { ready_.store(r, std::memory_order_release); }
  void release() {
    ready_.store(false, std::memory_order_release);
    owner_.store(Source::NONE, std::memory_order_release);
  }
  Source owner() const { return owner_.load(std::memory_order_acquire); }
  bool   ready() const { return ready_.load(std::memory_order_acquire); }

 private:
  std::atomic<Source> owner_{Source::NONE};
  std::atomic<bool>   ready_{false};
};

}  // namespace audio
````

### mp3/stream_player.cpp

````cpp
#include "stream_player.h"

#include <string.h>

namespace sp {

static_assert((RING_BYTES & (RING_BYTES - 1)) == 0, "RING_BYTES must be a power of two");

namespace {
constexpr uint32_t RING_MASK = RING_BYTES - 1;
}

StreamPlayer::StreamPlayer()
    : head_(0), tail_(0), st_(IDLE), endReq_(false), fadeIn_(false), channels_(1),
      frameBytes_(2), prebuffer_(4096), underruns_(0),
      pubState_(IDLE), pubSession_(0), pubUnderruns_(0) {}

// ---------------------------------------------------------------- producer --

bool StreamPlayer::write(const uint8_t* data, uint32_t len) {
    const uint32_t h = head_.load(std::memory_order_relaxed);
    const uint32_t t = tail_.load(std::memory_order_acquire);
    if (len > RING_BYTES - (h - t)) return false;            // not enough room: drop whole block
    const uint32_t idx   = h & RING_MASK;
    const uint32_t first = (len < RING_BYTES - idx) ? len : (RING_BYTES - idx);
    memcpy(ring_ + idx, data, first);
    if (len > first) memcpy(ring_, data + first, len - first);
    head_.store(h + len, std::memory_order_release);
    return true;
}

Status StreamPlayer::status() const {
    Status s;
    s.state     = (uint8_t)pubState_.load(std::memory_order_acquire);
    s.session   = (uint16_t)pubSession_.load(std::memory_order_acquire);
    s.underruns = (uint16_t)pubUnderruns_.load(std::memory_order_acquire);
    const uint32_t t = tail_.load(std::memory_order_acquire);
    const uint32_t h = head_.load(std::memory_order_acquire);
    s.fill     = h - t;
    s.capacity = RING_BYTES;
    return s;
}

// ---------------------------------------------------------------- consumer --

void StreamPlayer::publish() {
    uint32_t s = (uint32_t)st_;
    if (st_ != IDLE && endReq_) s = DRAINING;
    pubState_.store(s, std::memory_order_release);
}

void StreamPlayer::finish() {
    st_     = IDLE;
    endReq_ = false;
    fadeIn_ = false;
    publish();
}

void StreamPlayer::start(uint32_t rateHz, uint8_t channels, uint16_t session) {
    channels_   = (channels == 2) ? 2 : 1;
    frameBytes_ = (uint8_t)(2 * channels_);

    // ~0.25 s of audio before playback begins, clamped to [2 KB, half the ring].
    uint32_t pre = rateHz * frameBytes_ / 4;
    if (pre < 2048) pre = 2048;
    if (pre > RING_BYTES / 2) pre = RING_BYTES / 2;
    prebuffer_ = pre - (pre % frameBytes_);

    tail_.store(head_.load(std::memory_order_acquire), std::memory_order_release);  // flush
    endReq_    = false;
    fadeIn_    = false;
    underruns_ = 0;
    st_        = BUFFERING;
    pubUnderruns_.store(0, std::memory_order_release);
    pubSession_.store(session, std::memory_order_release);
    publish();
}

void StreamPlayer::end() {
    if (st_ == IDLE) return;
    endReq_ = true;
    publish();
}

void StreamPlayer::abort() {
    tail_.store(head_.load(std::memory_order_acquire), std::memory_order_release);
    finish();
}

uint32_t StreamPlayer::ringRead(uint8_t* dst, uint32_t n) {
    const uint32_t t     = tail_.load(std::memory_order_relaxed);
    const uint32_t idx   = t & RING_MASK;
    const uint32_t first = (n < RING_BYTES - idx) ? n : (RING_BYTES - idx);
    memcpy(dst, ring_ + idx, first);
    if (n > first) memcpy(dst + first, ring_, n - first);
    tail_.store(t + n, std::memory_order_release);
    return n;
}

int StreamPlayer::render(int16_t* out, int frames) {
    if (frames > MAX_CHUNK_FRAMES) frames = MAX_CHUNK_FRAMES;
    memset(out, 0, (size_t)frames * 4);
    if (st_ == IDLE) return 0;

    const uint32_t fb = frameBytes_;
    uint32_t fill = head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_relaxed);

    if (st_ == BUFFERING) {
        if (fill >= prebuffer_ || (endReq_ && fill > 0)) {
            st_     = PLAYING;
            fadeIn_ = true;
            publish();
        } else {
            if (endReq_) finish();      // host ended with nothing left to play
            return 0;
        }
    }

    // ---- PLAYING: pull up to one chunk of whole frames from the ring ----
    const uint32_t want = (uint32_t)frames * fb;
    uint32_t n = (fill < want) ? fill : want;
    n -= n % fb;                          // never split a frame
    const int got = (int)(n / fb);

    if (got > 0) {
        ringRead((uint8_t*)tmp_, n);
        if (channels_ == 1) {
            for (int i = 0; i < got; ++i) {
                out[2 * i]     = tmp_[i];
                out[2 * i + 1] = tmp_[i];
            }
        } else {
            memcpy(out, tmp_, n);
        }
    }

    if (fadeIn_ && got > 0) {
        const int r = (got < FADE_IN_FRAMES) ? got : FADE_IN_FRAMES;
        for (int i = 0; i < r; ++i) {
            out[2 * i]     = (int16_t)(((int32_t)out[2 * i]     * i) / r);
            out[2 * i + 1] = (int16_t)(((int32_t)out[2 * i + 1] * i) / r);
        }
        fadeIn_ = false;
    }

    if (got < frames) {
        // Ring ran dry mid-chunk: taper what we have so the cut is not a click.
        if (got > 0) {
            const int r = (got < FADE_OUT_FRAMES) ? got : FADE_OUT_FRAMES;
            for (int j = 0; j < r; ++j) {
                const int idx = got - r + j;
                const int32_t g = r - 1 - j;
                out[2 * idx]     = (int16_t)(((int32_t)out[2 * idx]     * g) / r);
                out[2 * idx + 1] = (int16_t)(((int32_t)out[2 * idx + 1] * g) / r);
            }
        }
        if (endReq_) {
            finish();                                   // normal end of track
        } else {
            ++underruns_;                               // genuine underrun: rebuffer
            pubUnderruns_.store(underruns_, std::memory_order_release);
            st_ = BUFFERING;
            publish();
        }
    }
    return got;
}

}  // namespace sp
````

### mp3/stream_player.h

````cpp
#pragma once
// PCM ring buffer + playback state machine for streamed audio.
// Pure C++11 (no Arduino / FreeRTOS / I2S), so the logic can be tested on a PC.
//
// Threading model (strict):
//   * Producer side  (stream receiver task): write(), status()
//   * Consumer side  (audio task ONLY):      start(), end(), abort(), render(), idle()
// The ring is a lock-free single-producer / single-consumer queue built on
// std::atomic<uint32_t> head/tail counters (free-running, RING_BYTES is a power of two).
// Everything shared between tasks is a 32-bit atomic (the only lock-free width on ESP32).

#include <atomic>
#include <stdint.h>

namespace sp {

constexpr uint32_t RING_BYTES       = 32768;   // power of two
constexpr int      MAX_CHUNK_FRAMES = 256;     // render() never produces more than this
constexpr int      FADE_IN_FRAMES   = 128;     // ~4 ms @ 32 kHz, removes start click
constexpr int      FADE_OUT_FRAMES  = 64;      // ~2 ms, removes click at underrun / end

enum State : uint8_t { IDLE = 0, BUFFERING = 1, PLAYING = 2, DRAINING = 3 };

struct Status {
  uint8_t  state;
  uint16_t session;
  uint16_t underruns;
  uint32_t fill;       // bytes currently queued in the ring
  uint32_t capacity;   // RING_BYTES
};

class StreamPlayer {
 public:
  StreamPlayer();

  // ---- producer side ------------------------------------------------------
  // All-or-nothing: either the whole block is queued or nothing is (returns false).
  bool write(const uint8_t* data, uint32_t len);
  Status status() const;

  // ---- consumer side (audio task) -----------------------------------------
  void start(uint32_t rateHz, uint8_t channels, uint16_t session);  // flush + BUFFERING
  void end();      // host finished: play out what is queued, then go IDLE
  void abort();    // stop now, drop everything
  bool idle() const { return st_ == IDLE; }

  // Writes exactly `frames` stereo frames (L,R interleaved s16) into out.
  // Returns how many leading frames contain real audio (the rest are silence).
  int render(int16_t* out, int frames);

 private:
  void     publish();
  void     finish();
  uint32_t ringRead(uint8_t* dst, uint32_t n);

  uint8_t               ring_[RING_BYTES];
  std::atomic<uint32_t> head_;        // written by producer
  std::atomic<uint32_t> tail_;        // written by consumer

  // consumer-owned
  State    st_;
  bool     endReq_;
  bool     fadeIn_;
  uint8_t  channels_;
  uint8_t  frameBytes_;
  uint32_t prebuffer_;
  uint32_t underruns_;
  int16_t  tmp_[MAX_CHUNK_FRAMES * 2];

  // published for other tasks
  std::atomic<uint32_t> pubState_;
  std::atomic<uint32_t> pubSession_;
  std::atomic<uint32_t> pubUnderruns_;
};

}  // namespace sp
````

### mp3/stream_proto.cpp

````cpp
#include "stream_proto.h"

namespace proto {

// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no xor-out), nibble-wise.
// Same as Python's binascii.crc_hqx(data, 0xFFFF).
static const uint16_t CRC_NIBBLE[16] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF};

uint16_t crc16(const uint8_t* p, size_t n, uint16_t crc) {
    while (n--) {
        const uint8_t b = *p++;
        crc = (uint16_t)((crc << 4) ^ CRC_NIBBLE[((crc >> 12) ^ (b >> 4)) & 0x0F]);
        crc = (uint16_t)((crc << 4) ^ CRC_NIBBLE[((crc >> 12) ^ (b & 0x0F)) & 0x0F]);
    }
    return crc;
}

size_t encode(uint8_t* out, uint8_t type, uint8_t seq, const uint8_t* payload, uint16_t len) {
    out[0] = MAGIC0;
    out[1] = MAGIC1;
    out[2] = type;
    out[3] = seq;
    out[4] = (uint8_t)(len & 0xFF);
    out[5] = (uint8_t)(len >> 8);
    for (uint16_t i = 0; i < len; ++i) out[HEADER_BYTES + i] = payload[i];
    const uint16_t crc = crc16(out + 2, 4u + len);
    out[HEADER_BYTES + len]     = (uint8_t)(crc & 0xFF);
    out[HEADER_BYTES + len + 1] = (uint8_t)(crc >> 8);
    return HEADER_BYTES + len + 2;
}

void FrameParser::reset() {
    state_  = S_MAGIC0;
    hdrPos_ = 0;
    crcPos_ = 0;
    len_    = 0;
    pos_    = 0;
    noise_  = 0;
    frame_.type = 0;
    frame_.seq = 0;
    frame_.len = 0;
    frame_.payload = payload_;
}

FrameParser::Result FrameParser::push(uint8_t b) {
    switch (state_) {
        case S_MAGIC0:
            if (b == MAGIC0) { state_ = S_MAGIC1; return R_NONE; }
            noise_ = b;
            return R_NOISE;

        case S_MAGIC1:
            if (b == MAGIC1) { state_ = S_HEADER; hdrPos_ = 0; return R_NONE; }
            if (b == MAGIC0) return R_NONE;           // AA AA 55 ...: stay armed
            state_ = S_MAGIC0;
            noise_ = b;
            return R_NOISE;

        case S_HEADER:
            hdr_[hdrPos_++] = b;
            if (hdrPos_ < 4) return R_NONE;
            len_ = (uint16_t)(hdr_[2] | ((uint16_t)hdr_[3] << 8));
            if (len_ > MAX_PAYLOAD) { state_ = S_MAGIC0; return R_BAD_LEN; }
            pos_    = 0;
            crcPos_ = 0;
            state_  = (len_ > 0) ? S_PAYLOAD : S_CRC;
            return R_NONE;

        case S_PAYLOAD:
            payload_[pos_++] = b;
            if (pos_ == len_) { state_ = S_CRC; crcPos_ = 0; }
            return R_NONE;

        case S_CRC: {
            crcBytes_[crcPos_++] = b;
            if (crcPos_ < 2) return R_NONE;
            state_ = S_MAGIC0;
            uint16_t crc = crc16(hdr_, 4);
            crc = crc16(payload_, len_, crc);
            const uint16_t rx = (uint16_t)(crcBytes_[0] | ((uint16_t)crcBytes_[1] << 8));
            if (crc != rx) return R_BAD_CRC;
            frame_.type    = hdr_[0];
            frame_.seq     = hdr_[1];
            frame_.len     = len_;
            frame_.payload = payload_;
            return R_FRAME;
        }
    }
    state_ = S_MAGIC0;
    return R_NONE;
}

}  // namespace proto
````

### mp3/stream_proto.h

````cpp
#pragma once
// Wire protocol between play_mp3.py (laptop) and the ESP32.
// Pure C++11, no Arduino/FreeRTOS dependencies (so it can be unit-tested on a PC).
//
// Frame (both directions, little-endian):
//   [0]    0xAA                  magic
//   [1]    0x55                  magic
//   [2]    type      u8
//   [3]    seq       u8          per-direction counter, wraps at 256
//   [4..5] len       u16         payload length, 0..1024
//   [6..]  payload   len bytes
//   [..]   crc16     u16         CRC-16/CCITT-FALSE over bytes [2 .. 6+len)
//
// Host -> ESP32 types:
//   PING   (0x01) no payload                  -> ESP32 replies STATUS
//   HELLO  (0x02) u32 rate, u8 channels, u8 bits(=16), u16 session
//   DATA   (0x03) interleaved s16le PCM, len multiple of channels*2
//   END    (0x04) no payload: host has no more data, play out the buffer then stop
//   ABORT  (0x05) no payload: stop now and flush
// ESP32 -> Host:
//   STATUS (0x81) u8 version, u8 state, u8 error, u8 pad, u16 session,
//                 u16 underruns, u16 errors, u32 fillBytes, u32 capacityBytes  (18 bytes)

#include <stddef.h>
#include <stdint.h>

namespace proto {

constexpr uint8_t  MAGIC0          = 0xAA;
constexpr uint8_t  MAGIC1          = 0x55;
constexpr uint16_t MAX_PAYLOAD     = 1024;
constexpr size_t   HEADER_BYTES    = 6;
constexpr size_t   MAX_FRAME_BYTES = HEADER_BYTES + MAX_PAYLOAD + 2;

constexpr uint8_t T_PING   = 0x01;
constexpr uint8_t T_HELLO  = 0x02;
constexpr uint8_t T_DATA   = 0x03;
constexpr uint8_t T_END    = 0x04;
constexpr uint8_t T_ABORT  = 0x05;
constexpr uint8_t T_STATUS = 0x81;

constexpr uint8_t  VERSION        = 1;
constexpr uint16_t HELLO_BYTES    = 8;
constexpr uint16_t STATUS_BYTES   = 18;

constexpr uint8_t ERR_NONE       = 0;
constexpr uint8_t ERR_BAD_PARAMS = 1;   // HELLO rejected
constexpr uint8_t ERR_BAD_DATA   = 2;   // DATA length not a whole number of frames
constexpr uint8_t ERR_OVERFLOW   = 3;   // DATA dropped, ring full (host ignored flow control)
constexpr uint8_t ERR_TIMEOUT    = 4;   // session aborted: host went silent

uint16_t crc16(const uint8_t* p, size_t n, uint16_t crc = 0xFFFF);

// Writes one complete frame into out (needs HEADER_BYTES + len + 2). Returns bytes written.
size_t encode(uint8_t* out, uint8_t type, uint8_t seq, const uint8_t* payload, uint16_t len);

// Byte-at-a-time frame parser. Handles partial reads naturally (feed whatever arrived),
// resynchronises on the next magic after garbage or a failed CRC.
class FrameParser {
 public:
  enum Result : uint8_t {
    R_NONE = 0,   // byte consumed as part of a frame in progress
    R_NOISE,      // byte was outside any frame (see noiseByte())
    R_FRAME,      // a valid frame completed (see frame())
    R_BAD_CRC,    // a frame completed but the CRC failed (dropped)
    R_BAD_LEN     // header announced len > MAX_PAYLOAD (dropped)
  };
  struct Frame {
    uint8_t        type;
    uint8_t        seq;
    uint16_t       len;
    const uint8_t* payload;   // valid until the next push()
  };

  FrameParser() { reset(); }
  void reset();
  Result push(uint8_t b);
  const Frame& frame() const { return frame_; }
  uint8_t noiseByte() const { return noise_; }

 private:
  enum State : uint8_t { S_MAGIC0, S_MAGIC1, S_HEADER, S_PAYLOAD, S_CRC };
  State    state_;
  uint8_t  hdr_[4];
  uint8_t  hdrPos_;
  uint8_t  crcPos_;
  uint8_t  crcBytes_[2];
  uint16_t len_;
  uint16_t pos_;
  uint8_t  noise_;
  Frame    frame_;
  uint8_t  payload_[MAX_PAYLOAD];
};

}  // namespace proto
````

### mp3/stream_rx.cpp

````cpp
#include "stream_rx.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "audio_out.h"
#include "stream_proto.h"

namespace stream {
namespace {

constexpr size_t   RX_DRIVER_BUFFER   = 8192;   // UART driver ring (~90 ms at 921600 baud)
constexpr size_t   READ_BURST         = 256;
constexpr uint32_t STATUS_PERIOD_MS   = 20;     // STATUS rate while a session is active
constexpr uint32_t SESSION_TIMEOUT_MS = 2000;   // host silent this long -> abort session
constexpr uint32_t HELLO_GRACE_MS     = 500;    // keep reporting after HELLO until audio task reacts

proto::FrameParser g_parser;
ConsoleHandler     g_console = nullptr;

uint8_t  g_txSeq        = 0;
uint8_t  g_expectSeq    = 0;
bool     g_haveSeq      = false;
uint32_t g_errors       = 0;       // lost/bad frames this session (CRC, length, seq gap, bad DATA, overflow)
uint8_t  g_lastError    = proto::ERR_NONE;
uint8_t  g_frameBytes   = 2;
uint32_t g_lastGoodMs   = 0;
uint32_t g_lastStatusMs = 0;
uint32_t g_helloMs      = 0;
bool     g_awaitStart   = false;
uint8_t  g_lastState    = audio::STREAM_IDLE;

inline void bumpError(uint8_t code) {
    ++g_errors;
    g_lastError = code;
}

inline uint32_t le32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
inline uint16_t le16(const uint8_t* p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
inline void put16(uint8_t* p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
inline void put32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

void sendStatus() {
    const audio::StreamStatus s = audio::streamStatus();
    uint8_t payload[proto::STATUS_BYTES];
    payload[0] = proto::VERSION;
    payload[1] = s.state;
    payload[2] = g_lastError;
    payload[3] = 0;
    put16(payload + 4, s.session);
    put16(payload + 6, s.underruns);
    put16(payload + 8, (uint16_t)(g_errors > 0xFFFF ? 0xFFFF : g_errors));
    put32(payload + 10, s.fillBytes);
    put32(payload + 14, s.capacityBytes);

    uint8_t frame[proto::HEADER_BYTES + proto::STATUS_BYTES + 2];
    const size_t n = proto::encode(frame, proto::T_STATUS, g_txSeq++, payload, proto::STATUS_BYTES);
    Serial.write(frame, n);
    g_lastStatusMs = millis();
}

void onHello(const proto::FrameParser::Frame& f) {
    if (f.len != proto::HELLO_BYTES) { bumpError(proto::ERR_BAD_PARAMS); sendStatus(); return; }

    const uint32_t rate    = le32(f.payload);
    const uint8_t  ch      = f.payload[4];
    const uint8_t  bits    = f.payload[5];
    const uint16_t session = le16(f.payload + 6);

    if (bits != 16 || ch < 1 || ch > 2 || rate < 8000 || rate > 48000 || session == 0) {
        g_lastError = proto::ERR_BAD_PARAMS;
        sendStatus();
        return;
    }

    // The host retries HELLO until it sees our STATUS: a repeat of a session that is
    // already open must not flush the ring.
    const audio::StreamStatus s = audio::streamStatus();
    if (s.state != audio::STREAM_IDLE && s.session == session) { sendStatus(); return; }

    g_errors     = 0;
    g_lastError  = proto::ERR_NONE;
    g_frameBytes = (uint8_t)(2 * ch);
    if (!audio::streamBegin(rate, ch, session)) {
        g_lastError = proto::ERR_BAD_PARAMS;
        sendStatus();
        return;
    }
    g_helloMs    = millis();
    g_awaitStart = true;
    sendStatus();
}

void onData(const proto::FrameParser::Frame& f) {
    if (audio::streamStatus().state == audio::STREAM_IDLE) return; // no session: ignore stray data
    if (audio::btActive()) return; // BT owns the pipeline

    if (f.len == 0 || (f.len % g_frameBytes) != 0) {
        bumpError(proto::ERR_BAD_DATA);
        return;
    }

    if (!audio::streamWrite(f.payload, f.len)) {
        bumpError(proto::ERR_OVERFLOW);
    }
}

void onFrame(const proto::FrameParser::Frame& f) {
    g_lastGoodMs = millis();

    // Every frame the host sends bumps seq, so a gap means a frame went missing.
    if (g_haveSeq && f.seq != g_expectSeq) ++g_errors;
    g_haveSeq   = true;
    g_expectSeq = (uint8_t)(f.seq + 1);

    switch (f.type) {
        case proto::T_PING:  sendStatus();                         break;
        case proto::T_HELLO: onHello(f);                           break;
        case proto::T_DATA:  onData(f);                            break;
        case proto::T_END:   audio::streamEnd();   sendStatus();   break;
        case proto::T_ABORT: audio::streamAbort(); sendStatus();   break;
        default: break;
    }
}

void periodic() {
    const uint32_t now = millis();
    const audio::StreamStatus s = audio::streamStatus();

    if (g_awaitStart && (s.state != audio::STREAM_IDLE || (uint32_t)(now - g_helloMs) > HELLO_GRACE_MS))
        g_awaitStart = false;

    if (s.state != g_lastState) {                  // state transition: tell the host right away
        g_lastState = s.state;
        sendStatus();
    } else if ((s.state != audio::STREAM_IDLE || g_awaitStart) &&
               (uint32_t)(now - g_lastStatusMs) >= STATUS_PERIOD_MS) {
        sendStatus();
    }

    // Host vanished (script killed, cable pulled): stop instead of looping on silence.
    if (s.state != audio::STREAM_IDLE && (uint32_t)(now - g_lastGoodMs) > SESSION_TIMEOUT_MS) {
        g_lastError = proto::ERR_TIMEOUT;
        audio::streamAbort();
        g_lastGoodMs = now;
    }
}

void rxTask(void*) {
    uint8_t buf[READ_BURST];
    for (;;) {
        const int avail = Serial.available();
        if (avail > 0) {
            const size_t want = (size_t)avail < READ_BURST ? (size_t)avail : READ_BURST;
            const size_t got  = Serial.read(buf, want);
            for (size_t i = 0; i < got; ++i) {
                switch (g_parser.push(buf[i])) {
                    case proto::FrameParser::R_FRAME:   onFrame(g_parser.frame()); break;
                    case proto::FrameParser::R_BAD_CRC: bumpError(proto::ERR_BAD_DATA); break;
                    case proto::FrameParser::R_BAD_LEN: bumpError(proto::ERR_BAD_DATA); break;
                    case proto::FrameParser::R_NOISE:
                        if (g_console && audio::streamStatus().state == audio::STREAM_IDLE)
                            g_console((char)g_parser.noiseByte());
                        break;
                    default: break;
                }
            }
        } else {
            vTaskDelay(1);                          // 1 ms; UART driver ring absorbs ~90 ms
        }
        periodic();
    }
}

}  // namespace

void setConsoleHandler(ConsoleHandler h) { g_console = h; }

bool begin() {
    Serial.setRxBufferSize(RX_DRIVER_BUFFER);       // must precede begin()
    Serial.begin(BAUD);
    g_lastGoodMs = millis();
    return xTaskCreatePinnedToCore(rxTask, "stream_rx", 4096, nullptr, 4, nullptr, 0) == pdPASS;
}

}  // namespace stream
````

### mp3/stream_rx.h

````cpp
#pragma once
#include <Arduino.h>

// UART receiver for the laptop -> ESP32 PCM stream (protocol: stream_proto.h).
// Owns Serial (UART0) after begin(): nothing else may read from it.

namespace stream {

static constexpr uint32_t BAUD = 921600;     // must match play_mp3.py --baud

// Optional: called (from the receiver task) for stray ASCII bytes that arrive while
// no stream session is active. Used to keep the old Serial-Monitor commands working.
typedef void (*ConsoleHandler)(char c);
void setConsoleHandler(ConsoleHandler h);

// Configures Serial and starts the receiver task (core 0). Call this FIRST in setup().
bool begin();

}  // namespace stream
````

### play_mp3.py

````python
#!/usr/bin/env python3
"""
play_mp3.py - stream an MP3 (or a generated test tone) to the ESP32 + PCM5102A over USB serial.

    python play_mp3.py --port COM5 --file "C:\\Music\\song.mp3"
    python play_mp3.py --port COM5 --test-tone            # no FFmpeg needed
    python play_mp3.py --list-ports

Pipeline:  FFmpeg (MP3 -> s16le PCM)  ->  framed + CRC'd packets  ->  USB serial  ->  ESP32 ring buffer -> I2S

The ESP32 reports its buffer fill ~50 times a second; this script only sends while the
ring has room (credit-based flow control), so playback is locked to the ESP32's own clock.

Protocol (must match stream_proto.h):
  frame = AA 55 | type u8 | seq u8 | len u16le | payload | crc16le   (CRC-16/CCITT-FALSE over type..payload)
"""

import argparse
import binascii
import collections
import math
import os
import random
import shutil
import struct
import subprocess
import sys
import threading
import time
from array import array

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial is not installed.  Run:  python -m pip install pyserial")

# ----------------------------------------------------------------------------- protocol

MAGIC = b"\xAA\x55"
T_PING, T_HELLO, T_DATA, T_END, T_ABORT = 0x01, 0x02, 0x03, 0x04, 0x05
T_STATUS = 0x81
MAX_PAYLOAD = 1024
PROTO_VERSION = 1

ST_IDLE, ST_BUFFERING, ST_PLAYING, ST_DRAINING = 0, 1, 2, 3
STATE_NAMES = {ST_IDLE: "idle", ST_BUFFERING: "buffering", ST_PLAYING: "playing", ST_DRAINING: "draining"}

ERR_NAMES = {0: "none", 1: "bad HELLO parameters", 2: "bad/corrupt data frame",
             3: "ring overflow (flow-control violated)", 4: "session timed out"}

STATUS_FMT = "<BBBBHHHII"          # version state error pad session underruns errors fill capacity
STATUS_LEN = struct.calcsize(STATUS_FMT)   # 18


def crc16(data, crc=0xFFFF):
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF)."""
    return binascii.crc_hqx(data, crc)


def encode_frame(ftype, seq, payload=b""):
    body = struct.pack("<BBH", ftype, seq & 0xFF, len(payload)) + payload
    return MAGIC + body + struct.pack("<H", crc16(body))


class Status:
    __slots__ = ("version", "state", "error", "session", "underruns", "errors", "fill", "capacity", "t")

    def __init__(self, payload):
        (self.version, self.state, self.error, _pad, self.session,
         self.underruns, self.errors, self.fill, self.capacity) = struct.unpack(STATUS_FMT, payload[:STATUS_LEN])
        self.t = time.monotonic()


class FrameReader:
    """Extracts STATUS frames from the ESP32's output, which also contains plain-text log lines.
    Anything that is not a valid frame is treated as text and skipped (and kept for --verbose)."""

    def __init__(self):
        self.buf = bytearray()
        self.text = bytearray()

    def feed(self, data):
        self.buf += data
        frames = []
        while True:
            i = self.buf.find(MAGIC)
            if i < 0:
                # keep a trailing 0xAA: it may be the first half of a magic split across reads
                keep = 1 if self.buf.endswith(b"\xAA") else 0
                self.text += self.buf[:len(self.buf) - keep]
                del self.buf[:len(self.buf) - keep]
                break
            if i > 0:
                self.text += self.buf[:i]
                del self.buf[:i]
            if len(self.buf) < 6:
                break
            ftype, seq, ln = struct.unpack_from("<BBH", self.buf, 2)
            if ln > 64:                                   # device frames are tiny: this was not a frame
                self.text += self.buf[:1]
                del self.buf[:1]
                continue
            total = 6 + ln + 2
            if len(self.buf) < total:
                break
            body = bytes(self.buf[2:6 + ln])
            (rx_crc,) = struct.unpack_from("<H", self.buf, 6 + ln)
            if crc16(body) != rx_crc:
                self.text += self.buf[:1]
                del self.buf[:1]
                continue
            frames.append((ftype, seq, bytes(self.buf[6:6 + ln])))
            del self.buf[:total]
        return frames


# ----------------------------------------------------------------------------- PCM sources

class FFmpegSource:
    """Decodes a file to interleaved s16le PCM with FFmpeg. read(n) returns b'' at EOF."""

    def __init__(self, ffmpeg, path, rate, channels, volume, swap_lr):
        filters = []
        if swap_lr and channels == 2:
            filters.append("pan=stereo|c0=c1|c1=c0")
        if volume != 1.0:
            filters.append("volume=%.4f" % volume)
        cmd = [ffmpeg, "-hide_banner", "-loglevel", "error", "-nostdin", "-i", path, "-vn"]
        if filters:
            cmd += ["-af", ",".join(filters)]
        cmd += ["-ac", str(channels), "-ar", str(rate), "-f", "s16le", "-acodec", "pcm_s16le", "pipe:1"]
        self.proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE)
        self.stderr_lines = []
        self._t = threading.Thread(target=self._drain_stderr, daemon=True)
        self._t.start()

    def _drain_stderr(self):
        for line in self.proc.stderr:
            self.stderr_lines.append(line.decode("utf-8", "replace").rstrip())

    def read(self, n):
        return self.proc.stdout.read(n)

    def close(self):
        try:
            if self.proc.poll() is None:
                self.proc.kill()
            self.proc.wait(timeout=2)
        except Exception:
            pass

    def error_text(self):
        self._t.join(timeout=1)
        return "\n".join(self.stderr_lines)

    def returncode(self):
        return self.proc.poll()


class ToneSource:
    """Generated sine test tone (stage-2 test). Needs no FFmpeg."""

    def __init__(self, rate, channels, seconds, volume, freq=440.0, which="both"):
        self.rate, self.channels = rate, channels
        self.remaining = int(seconds * rate)          # frames
        self.step = 2.0 * math.pi * freq / rate
        self.phase = 0.0
        self.amp = 0.5 * volume * 32767.0
        self.which = which

    def read(self, nbytes):
        frame_bytes = 2 * self.channels
        frames = min(self.remaining, nbytes // frame_bytes)
        if frames <= 0:
            return b""
        out = array("h")
        for _ in range(frames):
            v = int(self.amp * math.sin(self.phase))
            self.phase += self.step
            if self.channels == 1:
                out.append(v)
            else:
                out.append(v if self.which in ("both", "left") else 0)
                out.append(v if self.which in ("both", "right") else 0)
        if self.phase > 1e6:
            self.phase %= 2.0 * math.pi
        self.remaining -= frames
        if sys.byteorder == "big":
            out.byteswap()
        return out.tobytes()

    def close(self):
        pass


# ----------------------------------------------------------------------------- helpers

def fmt_time(sec):
    sec = max(0, int(sec))
    return "%d:%02d" % (sec // 60, sec % 60)


def probe_duration(path):
    ffprobe = shutil.which("ffprobe")
    if not ffprobe:
        return None
    try:
        out = subprocess.run([ffprobe, "-v", "error", "-show_entries", "format=duration",
                              "-of", "default=nw=1:nk=1", path],
                             capture_output=True, text=True, timeout=10).stdout.strip()
        return float(out)
    except Exception:
        return None


class DeviceError(Exception):
    pass


class Link:
    """Serial link: framed TX, STATUS RX."""

    def __init__(self, ser, verbose=False):
        self.ser = ser
        self.reader = FrameReader()
        self.seq = 0
        self.status = None
        self.last_tx = time.monotonic()
        self.verbose = verbose

    def send(self, ftype, payload=b""):
        frame = encode_frame(ftype, self.seq, payload)
        self.seq = (self.seq + 1) & 0xFF
        try:
            self.ser.write(frame)
        except serial.SerialTimeoutException:
            raise DeviceError("serial write timed out - the ESP32 stopped reading "
                              "(reset, crashed, or USB cable unplugged?)")
        except (serial.SerialException, OSError) as e:
            raise DeviceError("serial write failed: %s" % e)
        self.last_tx = time.monotonic()

    def poll(self):
        """Read whatever arrived, return the list of new STATUS objects (also stored in self.status)."""
        try:
            n = self.ser.in_waiting
            data = self.ser.read(n) if n else b""
        except (serial.SerialException, OSError) as e:
            raise DeviceError("serial read failed: %s" % e)
        new = []
        if data:
            for ftype, _seq, payload in self.reader.feed(data):
                if ftype == T_STATUS and len(payload) >= STATUS_LEN:
                    st = Status(payload)
                    if st.version == PROTO_VERSION:
                        self.status = st
                        new.append(st)
            if self.verbose and self.reader.text:
                sys.stderr.write(self.reader.text.decode("utf-8", "replace"))
                sys.stderr.flush()
            self.reader.text.clear()
        return new


def open_serial(port, baud):
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 0
    ser.write_timeout = 3.0
    ser.rtscts = False
    ser.dsrdtr = False
    ser.xonxoff = False
    ser.dtr = False        # try not to reset the board when the port opens (many boards reset anyway;
    ser.rts = False        # the handshake below copes with both cases)
    ser.open()
    return ser


def handshake(link, timeout, verbose):
    deadline = time.monotonic() + timeout
    link.status = None
    while time.monotonic() < deadline:
        link.send(T_PING)
        t_end = time.monotonic() + 0.25
        while time.monotonic() < t_end:
            if link.poll():
                return link.status
            time.sleep(0.005)
    return None


def start_session(link, rate, channels, timeout=5.0):
    session = random.randint(1, 0xFFFF)
    hello = struct.pack("<IBBH", rate, channels, 16, session)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        link.send(T_HELLO, hello)
        t_end = time.monotonic() + 0.3
        while time.monotonic() < t_end:
            for st in link.poll():
                if st.session == session and st.state in (ST_BUFFERING, ST_PLAYING, ST_DRAINING):
                    return session, st
                if st.error == 1:
                    raise DeviceError("ESP32 rejected the stream parameters (rate=%d, channels=%d)"
                                      % (rate, channels))
            time.sleep(0.005)
    raise DeviceError("ESP32 did not accept the stream session (no matching STATUS after HELLO)")


# ----------------------------------------------------------------------------- main streaming loop

def stream(link, source, rate, channels, chunk, total_seconds, high_frac, quiet):
    bytes_per_sec = rate * channels * 2
    st = link.status
    capacity = st.capacity
    high = int(capacity * high_frac)
    est_fill = st.fill
    sent = 0
    t_start = time.monotonic()
    last_print = 0.0
    last_status_seen = time.monotonic()
    frame_bytes = 2 * channels

    recent = collections.deque()          # (time, bytes) of DATA frames sent recently
    LATENCY = 0.06                        # a STATUS may not yet include data sent in the last ~60 ms

    def absorb(new):
        """Update the fill estimate from the newest STATUS. Conservative: add what we sent
        within the last LATENCY seconds, which the ESP32 may not have counted yet."""
        nonlocal est_fill, last_status_seen
        if not new:
            return
        now = time.monotonic()
        while recent and now - recent[0][0] > LATENCY:
            recent.popleft()
        est_fill = new[-1].fill + sum(n for _, n in recent)
        last_status_seen = now

    def progress(final=False):
        nonlocal last_print
        now = time.monotonic()
        if quiet or (not final and now - last_print < 0.25):
            return
        last_print = now
        s = link.status
        played = max(0, sent - est_fill) / bytes_per_sec
        total = ("/ " + fmt_time(total_seconds)) if total_seconds else ""
        sys.stdout.write("\r  %s %s  buffer %3d%% (%4.2fs)  state=%-9s underruns=%d errors=%d   " % (
            fmt_time(played), total, 100 * est_fill // max(1, capacity), est_fill / bytes_per_sec,
            STATE_NAMES.get(s.state, "?"), s.underruns, s.errors))
        sys.stdout.flush()

    eof = False
    while not eof:
        data = source.read(chunk)
        if not data:
            break
        data = data[:len(data) - (len(data) % frame_bytes)]      # never split a sample frame
        if not data:
            break

        # Wait for room in the ESP32 ring (credit-based flow control).
        while est_fill + len(data) > high:
            absorb(link.poll())
            if link.status.state == ST_IDLE:
                raise DeviceError("ESP32 ended the session (%s)" % ERR_NAMES.get(link.status.error, "?"))
            if time.monotonic() - last_status_seen > 1.5:
                raise DeviceError("ESP32 stopped reporting status (reset or unplugged?)")
            if time.monotonic() - link.last_tx > 0.25:
                link.send(T_PING)                                  # keep-alive; also elicits a STATUS
            progress()
            time.sleep(0.002)

        link.send(T_DATA, data)
        recent.append((time.monotonic(), len(data)))
        est_fill += len(data)
        sent += len(data)
        absorb(link.poll())
        progress()

    # ---- end of data: tell the ESP32 to play out what it has, then wait for IDLE ----
    link.send(T_END)
    last_end = time.monotonic()
    deadline = time.monotonic() + est_fill / bytes_per_sec + 6.0
    while time.monotonic() < deadline:
        absorb(link.poll())
        s = link.status
        if s.state == ST_IDLE:
            break
        if s.state != ST_DRAINING and time.monotonic() - last_end > 0.5:
            link.send(T_END)                                       # END frame may have been lost
            last_end = time.monotonic()
        if time.monotonic() - last_status_seen > 2.0:
            raise DeviceError("ESP32 stopped reporting status while draining")
        progress()
        time.sleep(0.005)
    else:
        raise DeviceError("timed out waiting for the ESP32 to finish playing")

    est_fill = 0
    progress(final=True)
    return sent, time.monotonic() - t_start


# ----------------------------------------------------------------------------- CLI

def main():
    ap = argparse.ArgumentParser(description="Stream an MP3 to the ESP32 + PCM5102A over USB serial.")
    ap.add_argument("--port", help="serial port, e.g. COM5 (Windows) or /dev/ttyUSB0")
    ap.add_argument("--file", help="MP3 (or any FFmpeg-readable audio) file")
    ap.add_argument("--test-tone", action="store_true", help="stream a generated 440 Hz tone instead of a file")
    ap.add_argument("--tone-seconds", type=float, default=5.0)
    ap.add_argument("--tone-channel", choices=("both", "left", "right"), default="both",
                    help="with --channels 2: which channel carries the tone (L/R wiring check)")
    ap.add_argument("--list-ports", action="store_true", help="list serial ports and exit")
    ap.add_argument("--ping-only", action="store_true", help="only test the serial handshake, then exit")
    ap.add_argument("--baud", type=int, default=921600, help="must match stream::BAUD in the sketch")
    ap.add_argument("--rate", type=int, default=32000, help="PCM sample rate sent to the ESP32 (default 32000)")
    ap.add_argument("--channels", type=int, choices=(1, 2), default=1, help="1 = mono (default), 2 = stereo")
    ap.add_argument("--volume", type=float, default=0.5,
                    help="linear gain applied by FFmpeg, default 0.5 (-6 dB). Values >1 can clip.")
    ap.add_argument("--swap-lr", action="store_true", help="swap left/right (stereo only)")
    ap.add_argument("--chunk", type=int, default=1024, help="PCM bytes per DATA frame (<=1024)")
    ap.add_argument("--ffmpeg", default="ffmpeg", help="path to ffmpeg executable")
    ap.add_argument("--boot-wait", type=float, default=2.0,
                    help="seconds to wait after opening the port (the board may reset), default 2")
    ap.add_argument("--connect-timeout", type=float, default=15.0)
    ap.add_argument("--high-water", type=float, default=0.75,
                    help="stop sending when the ESP32 ring is this full (0.3 - 0.9)")
    ap.add_argument("--force", action="store_true", help="skip the serial-bandwidth sanity check")
    ap.add_argument("--verbose", action="store_true", help="show the ESP32's text log lines")
    ap.add_argument("--quiet", action="store_true", help="no progress line")
    args = ap.parse_args()

    if args.list_ports:
        ports = list(list_ports.comports())
        if not ports:
            print("No serial ports found.")
        for p in ports:
            print("%-12s %s" % (p.device, p.description))
        return 0

    if not args.port:
        ap.error("--port is required (use --list-ports to see what is available)")
    if not args.test_tone and not args.file and not args.ping_only:
        ap.error("give --file PATH (or --test-tone, or --ping-only)")
    if args.ping_only:
        args.test_tone, args.tone_seconds = True, 0.0
    if not (8000 <= args.rate <= 48000):
        ap.error("--rate must be between 8000 and 48000")
    if not (1 <= args.chunk <= MAX_PAYLOAD):
        ap.error("--chunk must be 1..%d" % MAX_PAYLOAD)
    if not (0.3 <= args.high_water <= 0.9):
        ap.error("--high-water must be 0.3..0.9")
    if not (0.0 < args.volume <= 4.0):
        ap.error("--volume must be > 0 and <= 4")
    args.chunk -= args.chunk % (2 * args.channels)

    # ---- serial bandwidth sanity check ----
    need = args.rate * args.channels * 2 * (1 + 8.0 / args.chunk)      # payload + 8 bytes framing per chunk
    avail = args.baud / 10.0                                           # 8N1 = 10 bits per byte
    print("Stream: %d Hz, %s, 16-bit  = %.1f KB/s of %.1f KB/s serial capacity (%.0f%%)" % (
        args.rate, "mono" if args.channels == 1 else "stereo", need / 1000, avail / 1000, 100 * need / avail))
    if need > 0.90 * avail and not args.force:
        print("ERROR: that leaves no headroom on the serial link. Use --channels 1 and/or a lower --rate "
              "(e.g. --rate 32000), or raise the baud rate on BOTH sides (stream::BAUD in the sketch and "
              "--baud here). Override with --force.", file=sys.stderr)
        return 2

    # ---- source ----
    duration = None
    if args.test_tone:
        source = ToneSource(args.rate, args.channels, args.tone_seconds, args.volume, which=args.tone_channel)
        duration = args.tone_seconds
        print("Source: generated 440 Hz test tone, %.1f s" % args.tone_seconds)
    else:
        if not os.path.isfile(args.file):
            print("ERROR: file not found: %s" % args.file, file=sys.stderr)
            return 2
        if shutil.which(args.ffmpeg) is None and not os.path.isfile(args.ffmpeg):
            print("ERROR: FFmpeg not found ('%s'). Install it and make sure 'ffmpeg -version' works in a new "
                  "terminal, or pass --ffmpeg C:\\path\\to\\ffmpeg.exe" % args.ffmpeg, file=sys.stderr)
            return 2
        duration = probe_duration(args.file)
        source = FFmpegSource(args.ffmpeg, args.file, args.rate, args.channels, args.volume, args.swap_lr)
        print("Source: %s%s" % (args.file, (" (%s)" % fmt_time(duration)) if duration else ""))

    # ---- serial ----
    try:
        ser = open_serial(args.port, args.baud)
    except (serial.SerialException, OSError) as e:
        print("ERROR: cannot open %s: %s\nClose the Arduino Serial Monitor/Plotter (it holds the port) and "
              "check --list-ports." % (args.port, e), file=sys.stderr)
        source.close()
        return 2

    link = Link(ser, args.verbose)
    rc = 0
    try:
        if args.boot_wait > 0:
            time.sleep(args.boot_wait)             # board may be rebooting because the port opened
        ser.reset_input_buffer()

        st = handshake(link, args.connect_timeout, args.verbose)
        if st is None:
            raise DeviceError("no reply from the ESP32 on %s at %d baud. Check: right COM port, sketch flashed "
                              "(Serial Monitor must be closed), baud rate matches stream::BAUD, USB cable carries data."
                              % (args.port, args.baud))
        print("ESP32 connected: protocol v%d, ring %d bytes, state %s"
              % (st.version, st.capacity, STATE_NAMES.get(st.state, "?")))

        if args.ping_only:
            print("Handshake OK.")
            return 0

        session, st = start_session(link, args.rate, args.channels)
        if not args.quiet:
            print("Playing (Ctrl+C to stop)...")
        sent, secs = stream(link, source, args.rate, args.channels, args.chunk, duration,
                            args.high_water, args.quiet)
        s = link.status
        if not args.test_tone and source.returncode() not in (None, 0):
            print("\nWARNING: FFmpeg exited with code %s:\n%s" % (source.returncode(), source.error_text()),
                  file=sys.stderr)
        print("\nDone: %.1f s of audio sent in %.1f s. Underruns: %d, frame errors: %d."
              % (sent / (args.rate * args.channels * 2.0), secs, s.underruns, s.errors))
        if s.underruns or s.errors:
            print("Note: underruns mean the laptop could not keep the buffer filled (CPU/USB load); "
                  "errors mean frames were lost or corrupted on the link.")

    except KeyboardInterrupt:
        print("\nStopping...")
        try:
            link.send(T_ABORT)
            time.sleep(0.2)
        except Exception:
            pass
        rc = 130
    except DeviceError as e:
        print("\nERROR: %s" % e, file=sys.stderr)
        rc = 1
    finally:
        source.close()
        try:
            ser.close()
        except Exception:
            pass
    return rc


if __name__ == "__main__":
    sys.exit(main())
````

### test.ps1

````powershell
$out = 'project.md'
$root = (Get-Location).Path
$skipDirs = '\\(\.git|__pycache__|node_modules|\.venv|build|\.vs)(\\|$)'
$binExt = '.mp3','.wav','.png','.jpg','.jpeg','.gif','.zip','.exe','.bin','.pyc','.pdf','.ico'
$fence = '````'
$lang = @{ '.cpp'='cpp'; '.h'='cpp'; '.ino'='cpp'; '.py'='python'; '.sh'='bash'; '.md'='markdown'; '.json'='json'; '.ps1'='powershell' }

$files = Get-ChildItem -Recurse -File -Force |
  Where-Object { $_.FullName -notmatch $skipDirs -and $_.Name -ne $out } |
  Sort-Object FullName

$tree = if (Get-Command eza -ErrorAction SilentlyContinue) { eza --tree -I '__pycache__|node_modules|build' } else { tree /F /A }

$sb = [System.Text.StringBuilder]::new()
[void]$sb.AppendLine('# Project Tree').AppendLine().AppendLine($fence + 'text')
[void]$sb.AppendLine(($tree -join "`n")).AppendLine($fence).AppendLine()

foreach ($f in $files) {
  $rel = $f.FullName.Substring($root.Length + 1) -replace '\\','/'
  [void]$sb.AppendLine("### $rel").AppendLine()
  if ($binExt -contains $f.Extension.ToLower()) {
    [void]$sb.AppendLine('_(binary file, content omitted)_').AppendLine()
    } else {
    $l = $lang[$f.Extension.ToLower()]
    $content = Get-Content -Raw -LiteralPath $f.FullName
    if ($null -eq $content) { $content = '' }
    [void]$sb.AppendLine($fence + $l)
    [void]$sb.AppendLine($content.TrimEnd())
    [void]$sb.AppendLine($fence).AppendLine()
  }
}
$sb.ToString() | Set-Content -Path $out -Encoding UTF8
"Wrote $out ($($files.Count) files)"
````

### tests/probe.py

````python
import sys, time
sys.path.insert(0,'stubserial'); sys.path.insert(0,'..')
import play_mp3 as P, serial
ser = P.open_serial(sys.argv[1], 921600); link = P.Link(ser)
st = P.handshake(link, 3, False)
print("probe: state=%s error=%s underruns=%d fill=%d" % (P.STATE_NAMES[st.state], P.ERR_NAMES[st.error], st.underruns, st.fill))
````

### tests/run_host_tests.sh

````bash
#!/usr/bin/env bash
# PC-side tests of the REAL firmware sources (Linux/macOS/WSL, needs g++, python3, numpy, ffmpeg for the MP3 test).
# UART is a pty, I2S is a fake that records the PCM. Does not need an ESP32.
set -e
cd "$(dirname "$0")"; FW=../mp3
g++ -std=gnu++11 -O2 -Wall -Wextra -pthread -Ishim -I$FW -o sim_device sim_device.cpp \
    $FW/stream_rx.cpp $FW/stream_proto.cpp $FW/audio_out.cpp $FW/stream_player.cpp $FW/audio_levels.cpp
g++ -std=gnu++11 -O2 -I$FW -o test_proto test_proto.cpp $FW/stream_proto.cpp
g++ -std=gnu++11 -O2 -I$FW -o test_levels test_levels.cpp $FW/audio_levels.cpp
./test_levels s
SIM_SPEED=1 SIM_OUT=/tmp/i2s_tone.raw ./sim_device > /tmp/sim.port 2>/tmp/sim.log & SIM=$!
sleep 1; PORT=$(head -1 /tmp/sim.port)
PYTHONPATH=stubserial python3 ../play_mp3.py --port "$PORT" --test-tone --tone-seconds 4 --boot-wait 0.2 --quiet
kill $SIM
echo "For the full-MP3 bit-exact comparison see README section 'What was verified'."
````

### tests/runner.py

````python
import signal, sys, runpy
signal.signal(signal.SIGINT, signal.default_int_handler)
sys.argv = ["play_mp3.py"] + sys.argv[1:]
runpy.run_path("../play_mp3.py", run_name="__main__")
````

### tests/shim/Arduino.h

````cpp
#pragma once
// Minimal host-side stand-in for Arduino.h so firmware logic can run on a PC.
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#define F(x) x
uint32_t millis();
class SimSerial {
 public:
  void   setRxBufferSize(size_t) {}
  void   begin(uint32_t) {}
  int    available();
  size_t read(uint8_t* buf, size_t n);
  size_t write(const uint8_t* buf, size_t n);
  size_t print(const char* s)      { return write((const uint8_t*)s, strlen(s)); }
  size_t println(const char* s)    { size_t n = print(s); n += print("\r\n"); return n; }
  size_t print(unsigned long v)    { char b[24]; snprintf(b, sizeof b, "%lu", v); return print(b); }
  size_t println(unsigned long v)  { size_t n = print(v); n += print("\r\n"); return n; }
  size_t print(unsigned v)         { return print((unsigned long)v); }
  size_t println(unsigned v)       { return println((unsigned long)v); }
};
extern SimSerial Serial;
````

### tests/shim/driver/i2s.h

````cpp
#pragma once
// Host-side fake of the legacy ESP32 I2S driver: captures what would reach the DAC,
// paced like real DMA (optionally sped up with SIM_SPEED).
#include <stddef.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
typedef enum { I2S_NUM_0 = 0 } i2s_port_t;
typedef enum { I2S_MODE_MASTER = 1, I2S_MODE_TX = 2 } i2s_mode_t;
typedef enum { I2S_BITS_PER_SAMPLE_16BIT = 16 } i2s_bits_per_sample_t;
typedef enum { I2S_CHANNEL_FMT_RIGHT_LEFT = 0 } i2s_channel_fmt_t;
typedef enum { I2S_COMM_FORMAT_STAND_I2S = 1 } i2s_comm_format_t;
typedef enum { I2S_CHANNEL_STEREO = 2 } i2s_channel_t;
#define ESP_INTR_FLAG_LEVEL1 2
#define I2S_PIN_NO_CHANGE (-1)
struct i2s_config_t {
  i2s_mode_t mode; uint32_t sample_rate; i2s_bits_per_sample_t bits_per_sample; i2s_channel_fmt_t channel_format;
  i2s_comm_format_t communication_format; int intr_alloc_flags; int dma_buf_count; int dma_buf_len;
  bool use_apll; bool tx_desc_auto_clear; int fixed_mclk;
};
struct i2s_pin_config_t { int mck_io_num, bck_io_num, ws_io_num, data_out_num, data_in_num; };
esp_err_t i2s_driver_install(i2s_port_t, const i2s_config_t*, int, void*);
esp_err_t i2s_driver_uninstall(i2s_port_t);
esp_err_t i2s_set_pin(i2s_port_t, const i2s_pin_config_t*);
esp_err_t i2s_zero_dma_buffer(i2s_port_t);
esp_err_t i2s_set_clk(i2s_port_t, uint32_t rate, i2s_bits_per_sample_t, i2s_channel_t);
esp_err_t i2s_write(i2s_port_t, const void* src, size_t size, size_t* written, TickType_t);
````

### tests/shim/freertos/FreeRTOS.h

````cpp
#pragma once
#include <stdint.h>
typedef int      BaseType_t;
typedef uint32_t TickType_t;
#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define portMAX_DELAY 0xFFFFFFFFu
#define pdMS_TO_TICKS(x) (x)
````

### tests/shim/freertos/queue.h

````cpp
#pragma once
#include "FreeRTOS.h"
#include <deque>
#include <mutex>
#include <vector>
#include <string.h>
#include <chrono>
#include <thread>
struct SimQueue { std::mutex m; std::deque<std::vector<uint8_t>> q; size_t item; size_t len; };
typedef SimQueue* QueueHandle_t;
inline QueueHandle_t xQueueCreate(size_t len, size_t item) { auto* q = new SimQueue(); q->item = item; q->len = len; return q; }
inline BaseType_t xQueueSend(QueueHandle_t q, const void* it, TickType_t) {
    std::lock_guard<std::mutex> g(q->m);
    if (q->q.size() >= q->len) return pdFALSE;
    q->q.emplace_back((const uint8_t*)it, (const uint8_t*)it + q->item);
    return pdTRUE;
}
inline BaseType_t xQueueReceive(QueueHandle_t q, void* it, TickType_t) {
    std::lock_guard<std::mutex> g(q->m);
    if (q->q.empty()) return pdFALSE;
    memcpy(it, q->q.front().data(), q->item);
    q->q.pop_front();
    return pdTRUE;
}
````

### tests/shim/freertos/task.h

````cpp
#pragma once
#include "FreeRTOS.h"
#include <thread>
#include <chrono>
typedef void (*TaskFunction_t)(void*);
typedef void* TaskHandle_t;
inline BaseType_t xTaskCreatePinnedToCore(TaskFunction_t fn, const char*, uint32_t, void* arg, unsigned, TaskHandle_t*, int) {
    std::thread([fn, arg] { fn(arg); }).detach();
    return pdPASS;
}
inline void vTaskDelay(TickType_t ticks) { std::this_thread::sleep_for(std::chrono::milliseconds(ticks)); }
````

### tests/sim_device.cpp

````cpp
// Runs the REAL firmware sources (stream_rx, stream_proto, stream_player, audio_levels,
// audio_out) on a PC. UART = a pty (play_mp3.py connects to the printed slave path),
// I2S = a fake that records the PCM and paces itself like real DMA.
#define _GNU_SOURCE
#include "Arduino.h"
#include "driver/i2s.h"
#include "audio_out.h"
#include "stream_rx.h"

#include <atomic>
#include <chrono>
#include <fcntl.h>
#include <mutex>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

using Clock = std::chrono::steady_clock;
static const Clock::time_point T0 = Clock::now();
uint32_t millis() { return (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - T0).count(); }

static int g_master = -1;
SimSerial Serial;
int SimSerial::available() { int n = 0; ioctl(g_master, FIONREAD, &n); return n; }
size_t SimSerial::read(uint8_t* b, size_t n) { ssize_t r = ::read(g_master, b, n); return r > 0 ? (size_t)r : 0; }
size_t SimSerial::write(const uint8_t* b, size_t n) { ssize_t r = ::write(g_master, b, n); return r > 0 ? (size_t)r : 0; } // drops when full, like a bridge chip

// ---- fake I2S ----
static FILE*  g_out = nullptr;
static double g_speed = 1.0;
static uint32_t g_rate = 44100;
static Clock::time_point g_playPos;            // virtual time at which everything queued so far finishes playing
static std::mutex g_m;
static uint64_t g_framesWritten = 0;
static const double DMA_SECONDS_FRAMES = 8 * 256;

static void resetPos() { g_playPos = Clock::now(); }
esp_err_t i2s_driver_install(i2s_port_t, const i2s_config_t* c, int, void*) { g_rate = c->sample_rate; resetPos(); return ESP_OK; }
esp_err_t i2s_driver_uninstall(i2s_port_t) { return ESP_OK; }
esp_err_t i2s_set_pin(i2s_port_t, const i2s_pin_config_t*) { return ESP_OK; }
esp_err_t i2s_zero_dma_buffer(i2s_port_t) { std::lock_guard<std::mutex> g(g_m); resetPos(); return ESP_OK; }
esp_err_t i2s_set_clk(i2s_port_t, uint32_t rate, i2s_bits_per_sample_t, i2s_channel_t) {
    std::lock_guard<std::mutex> g(g_m);
    g_rate = rate; resetPos();
    fprintf(stderr, "[sim-i2s] clock -> %u Hz at t=%u ms (frames written so far: %llu)\n", rate, millis(), (unsigned long long)g_framesWritten);
    return ESP_OK;
}
esp_err_t i2s_write(i2s_port_t, const void* src, size_t size, size_t* written, TickType_t) {
    const size_t frames = size / 4;
    fwrite(src, 1, size, g_out); fflush(g_out);
    g_framesWritten += frames;
    Clock::time_point wake;
    {
        std::lock_guard<std::mutex> g(g_m);
        auto now = Clock::now();
        if (g_playPos < now) g_playPos = now;                       // DMA ran dry: restart from now
        g_playPos += std::chrono::nanoseconds((long long)(1e9 * frames / g_rate / g_speed));
        // real driver only blocks once all DMA buffers are full
        wake = g_playPos - std::chrono::nanoseconds((long long)(1e9 * DMA_SECONDS_FRAMES / g_rate / g_speed));
    }
    std::this_thread::sleep_until(wake);
    *written = size;
    return ESP_OK;
}

int main() {
    if (const char* s = getenv("SIM_SPEED")) g_speed = atof(s);
    const char* outPath = getenv("SIM_OUT"); if (!outPath) outPath = "/tmp/i2s_out.raw";
    g_out = fopen(outPath, "wb");

    g_master = posix_openpt(O_RDWR | O_NOCTTY);
    grantpt(g_master); unlockpt(g_master);
    const char* sl = ptsname(g_master);
    int slave = open(sl, O_RDWR | O_NOCTTY);          // keep one slave open so the master never sees hangup
    termios t; tcgetattr(slave, &t); cfmakeraw(&t); tcsetattr(slave, TCSANOW, &t);
    fcntl(g_master, F_SETFL, fcntl(g_master, F_GETFL) | O_NONBLOCK);
    printf("%s\n", sl); fflush(stdout);

    stream::begin();
    if (!audio::begin()) { fprintf(stderr, "audio::begin failed\n"); return 1; }
    fprintf(stderr, "[sim] ready on %s speed=%.2f out=%s\n", sl, g_speed, outPath);
    for (;;) sleep(1);
}
````

### tests/stubserial/serial/__init__.py

````python
"""Tiny stand-in for pyserial (not installable offline) that talks to a pty. Same API subset as play_mp3.py uses."""
import fcntl, os, select, termios, time, tty, struct

class SerialException(OSError): pass
class SerialTimeoutException(SerialException): pass

class Serial:
    def __init__(self):
        self.port = None; self.baudrate = 9600; self.timeout = None; self.write_timeout = None
        self.rtscts = self.dsrdtr = self.xonxoff = False; self.dtr = self.rts = True
        self.fd = None
    def open(self):
        self.fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        tty.setraw(self.fd)
    @property
    def in_waiting(self):
        buf = fcntl.ioctl(self.fd, termios.FIONREAD, b"\0\0\0\0")
        return struct.unpack("i", buf)[0]
    def read(self, n):
        try: return os.read(self.fd, n)
        except BlockingIOError: return b""
    def write(self, data):
        deadline = time.monotonic() + (self.write_timeout or 1e9)
        view = memoryview(data)
        while len(view):
            try:
                k = os.write(self.fd, view); view = view[k:]
            except BlockingIOError:
                if time.monotonic() > deadline: raise SerialTimeoutException("write timeout")
                select.select([], [self.fd], [], 0.01)
        return len(data)
    def reset_input_buffer(self):
        termios.tcflush(self.fd, termios.TCIFLUSH)
    def close(self):
        if self.fd is not None: os.close(self.fd); self.fd = None
````

### tests/stubserial/serial/tools/__init__.py

````python

````

### tests/stubserial/serial/tools/list_ports.py

````python
def comports(): return []
````

### tests/test_levels.cpp

````cpp
#include "audio_levels.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <vector>
int main(int argc, char** argv) {
    viz::BandAnalyzer a; a.reset(32000);
    if (argc > 1 && argv[1][0] == 's') {          // sine sweep test
        const float F[10] = {80,150,280,500,900,1600,2800,5000,8500,13000};
        for (int k = 0; k < 10; ++k) for (float amp : {0.5f, 0.05f}) {
            a.reset(32000);
            std::vector<int16_t> buf(2*1024);
            for (int i = 0; i < 1024; ++i) { int16_t v = (int16_t)(amp*32767*sinf(6.2831853f*F[k]*i/32000.f)); buf[2*i]=buf[2*i+1]=v; }
            a.feed(buf.data(), 1024);
            uint8_t o[10]; a.read(o); int best = 0; for (int j=1;j<10;++j) if (o[j]>o[best]) best=j;
            printf("%6.0f Hz amp %.2f -> peak band %d  [", F[k], amp, best); for (int j=0;j<10;++j) printf("%3d ", o[j]); printf("]\n");
        }
        return 0;
    }
    // stream of mono s16 from stdin -> per-block bands (stereo duplicated)
    int16_t s; std::vector<int16_t> buf; unsigned long blocks = 0; double sum[10] = {0}; int mx[10] = {0};
    while (fread(&s, 2, 1, stdin) == 1) {
        buf.push_back(s); buf.push_back(s);
        if (buf.size() == 2*256) {
            if (a.feed(buf.data(), 256)) { uint8_t o[10]; a.read(o); ++blocks; for (int j=0;j<10;++j){sum[j]+=o[j]; if(o[j]>mx[j])mx[j]=o[j];} if (blocks % 200 == 0) { for (int j=0;j<10;++j) printf("%3d ", o[j]); printf("\n"); } }
            buf.clear();
        }
    }
    printf("blocks=%lu\nmean: ", blocks); for (int j=0;j<10;++j) printf("%5.0f ", sum[j]/blocks);
    printf("\nmax : "); for (int j=0;j<10;++j) printf("%5d ", mx[j]); printf("\n");
}
````

### tests/test_ownership.cpp

````cpp
#include "source_manager.h"
#include <cassert>
#include <cstdio>

using namespace audio;

int main() {
    SourceManager m;
    assert(m.owner() == Source::NONE);
    assert(!m.ready());

    // Successful acquisition.
    assert(m.acquire(Source::USB_SERIAL));
    assert(m.owner() == Source::USB_SERIAL);
    assert(!m.ready());

    // Second acquisition must fail.
    assert(!m.acquire(Source::BLUETOOTH));
    assert(m.owner() == Source::USB_SERIAL);

    // Ready after audio task start.
    m.setReady(true);
    assert(m.ready());

    // Release only from audio task path.
    m.release();
    assert(m.owner() == Source::NONE);
    assert(!m.ready());

    // BT acquisition after release.
    assert(m.acquire(Source::BLUETOOTH));
    assert(m.owner() == Source::BLUETOOTH);

    // USB serial cannot steal ownership.
    assert(!m.acquire(Source::USB_SERIAL));
    assert(m.owner() == Source::BLUETOOTH);

    // Failed startup: release immediately.
    m.release();
    assert(m.acquire(Source::USB_SERIAL));

    printf("test_ownership: PASS\n");
    return 0;
}
````

### tests/test_proto.cpp

````cpp
#include "stream_proto.h"
#include <stdio.h>
#include <unistd.h>
int main() {
    proto::FrameParser p;
    uint8_t b;
    unsigned noise = 0, badcrc = 0, badlen = 0;
    while (read(0, &b, 1) == 1) {
        switch (p.push(b)) {
            case proto::FrameParser::R_FRAME: {
                const auto& f = p.frame();
                printf("F %u %u %u %04x\n", f.type, f.seq, f.len, proto::crc16(f.payload, f.len));
                break; }
            case proto::FrameParser::R_NOISE:   ++noise; break;
            case proto::FrameParser::R_BAD_CRC: ++badcrc; break;
            case proto::FrameParser::R_BAD_LEN: ++badlen; break;
            default: break;
        }
    }
    printf("N %u %u %u\n", noise, badcrc, badlen);
}
````


