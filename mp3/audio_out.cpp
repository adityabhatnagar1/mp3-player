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