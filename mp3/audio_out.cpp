#include "audio_out.h"

#if __has_include("esp_arduino_version.h")
#include "esp_arduino_version.h"
#endif
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
#error "Uses legacy driver/i2s.h (Arduino-ESP32 core 2.x). Pin espressif32@6.9.0."
#endif

#include <math.h>
#include "driver/i2s.h"

namespace audio {
namespace {

constexpr i2s_port_t I2S_PORT = I2S_NUM_0;

constexpr int DMA_BUF_COUNT  = 8;
constexpr int DMA_BUF_FRAMES = 256;
constexpr int CHUNK_FRAMES   = DMA_BUF_FRAMES;

constexpr int TABLE_BITS = 10;
constexpr int TABLE_SIZE = 1 << TABLE_BITS;
int16_t sineTable[TABLE_SIZE];

// Sine table peak = -18 dBFS (same level as before)
constexpr int16_t SINE_PEAK = 4096;

// ---- Notes (equal temperament, A4 = 440, C4 .. C6) ----
static const uint16_t NOTE_HZ[] = {
    262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494, // C4..B4
    523, 554, 587, 622, 659, 698, 740, 784, 831, 880, 932, 988, // C5..B5
    1047                                                         // C6
};
static constexpr uint8_t NOTE_REST = 0xFF;

struct Step { uint8_t note; uint16_t ms; };

// C major scale up, then back down, then a short rest.
// note: index into NOTE_HZ, or NOTE_REST for silence.
static const Step SCALE[] = {
    { 0, 180}, { 2, 180}, { 4, 180}, { 5, 180},
    { 7, 180}, { 9, 180}, {11, 180}, {12, 420},
    {11, 180}, { 9, 180}, { 7, 180}, { 5, 180},
    { 4, 180}, { 2, 180}, { 0, 420},
    {NOTE_REST, 350},
};
constexpr uint8_t SEQ_LEN = sizeof(SCALE) / sizeof(SCALE[0]);

constexpr uint32_t TONE_HZ = 440;

// ---- Envelope constants ----
constexpr int32_t  ENV_MAX         = 32767;   // Q15 1.0
constexpr uint32_t ATTACK_SAMPLES  = 88;      // ~2 ms
constexpr uint32_t FADEOUT_SAMPLES = 110;     // ~2.5 ms
constexpr int32_t  ATTACK_STEP     = ENV_MAX / (int32_t)ATTACK_SAMPLES;
constexpr int32_t  FADEOUT_STEP    = ENV_MAX / (int32_t)FADEOUT_SAMPLES;

// Exponential decay coefficient (Q15 per-sample multiply).
// tau = 250 ms -> coeff = 32768 * exp(-1 / (0.250 * 44100)) ~= 32765
constexpr int32_t  DECAY_COEFF = 32765;

volatile Mode     g_mode        = MODE_OFF;
volatile uint32_t g_writeErrors = 0;
bool              g_started     = false;

int16_t g_chunk[CHUNK_FRAMES * 2];

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
        const Mode m = g_mode;

        // ---- Mode change ----
        if (m != lastMode) {
            if (m == MODE_TONE) {
                phase           = 0;
                phaseInc        = phaseIncFor(TONE_HZ);
                env             = 0;
                attackRemaining = ATTACK_SAMPLES;
                envAttacking    = true;
                fadeRemaining   = 0;
                noteRemaining   = 0;
                pendingAdvance  = false;
            } else if (m == MODE_SCALE) {
                seqPos = 0;
                const Step& s = SCALE[0];
                phase      = 0;
                phaseInc   = (s.note == NOTE_REST) ? 0 : phaseIncFor(NOTE_HZ[s.note]);
                env        = 0;
                envAttacking = (s.note != NOTE_REST);
                attackRemaining = envAttacking ? ATTACK_SAMPLES : 0;
                fadeRemaining = 0;
                noteRemaining = (uint32_t)s.ms * SAMPLE_RATE_HZ / 1000;
                pendingAdvance = false;
            } else { // MODE_OFF
                if (env > 0) fadeRemaining = FADEOUT_SAMPLES;
                pendingAdvance = false;
                noteRemaining  = 0;
                envAttacking   = false;
            }
            lastMode = m;
        }

        // ---- Sample generation ----
        for (int i = 0; i < CHUNK_FRAMES; ++i) {

            // Envelope state machine
            if (fadeRemaining > 0) {
                env -= FADEOUT_STEP;
                --fadeRemaining;
                if (env <= 0 || fadeRemaining == 0) {
                    env = 0;
                    fadeRemaining = 0;
                    if (pendingAdvance) {
                        pendingAdvance = false;
                        const Step& s = SCALE[seqPos];
                        phase    = 0;
                        phaseInc = (s.note == NOTE_REST) ? 0 : phaseIncFor(NOTE_HZ[s.note]);
                        if (s.note != NOTE_REST) {
                            env = 0;
                            attackRemaining = ATTACK_SAMPLES;
                            envAttacking = true;
                        } else {
                            envAttacking = false;
                        }
                        noteRemaining = (uint32_t)s.ms * SAMPLE_RATE_HZ / 1000;
                    } else if (m == MODE_OFF) {
                        phaseInc = 0;
                    }
                }
            } else if (envAttacking) {
                env += ATTACK_STEP;
                --attackRemaining;
                if (attackRemaining == 0 || env >= ENV_MAX) {
                    env = ENV_MAX;
                    envAttacking = false;
                }
            } else if (env > 0) {
                if (m == MODE_SCALE) {          // piano-style decay
                    env = (env * DECAY_COEFF) >> 15;
                    if (env < 16) env = 0;
                }
                // MODE_TONE: hold env at ENV_MAX (steady tone)
            }

            // Note timer (SCALE only)
            if (m == MODE_SCALE && noteRemaining > 0) {
                if (--noteRemaining == 0) {
                    seqPos = (uint8_t)((seqPos + 1) % SEQ_LEN);
                    pendingAdvance = true;
                    fadeRemaining  = FADEOUT_SAMPLES;
                }
            }

            // Output
            int16_t out = 0;
            if (env > 0 && phaseInc != 0) {
                const int32_t s = sineLookup(phase);
                out = (int16_t)((s * env) >> 15);
            }
            phase += phaseInc;

            g_chunk[2*i]     = out;
            g_chunk[2*i + 1] = out;
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

    if (xTaskCreatePinnedToCore(audioTask, "audio", 4096, nullptr, 5, nullptr, 0) != pdPASS) {
        i2s_driver_uninstall(I2S_PORT);
        return false;
    }

    g_started = true;
    return true;
}

void setMode(Mode m)   { g_mode = m; }
Mode mode()            { return g_mode; }
uint32_t writeErrors() { return g_writeErrors; }

}  // namespace audio