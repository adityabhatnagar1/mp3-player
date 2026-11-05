/*
 * Retro 128x64 OLED Audio Visualizer + laptop-streamed MP3 player
 * Milestone 04 - PCM5102A I2S DAC fed by PCM streamed over USB Serial
 *
 * Hardware:
 *   ESP32-WROOM-32 DevKit V1
 *   M130-12864-B-4V 1.3" OLED, SH1106, 0x3C
 *     SDA = GPIO21, SCL = GPIO22, U8g2 full framebuffer, hardware I2C
 *   PCM5102A breakout (3-wire I2S, SCK tied to GND)
 *     BCK = GPIO27, LCK = GPIO25, DIN = GPIO26   (see audio_out.h)
 *
 * Signal path:
 *   laptop: play_mp3.py -> FFmpeg (MP3 -> s16le PCM) -> USB serial, 921600 baud
 *   ESP32 : stream_rx.cpp  (UART task, core 0, framing + CRC + flow control)
 *           stream_player.cpp (lock-free ring buffer + playback state machine)
 *           audio_out.cpp  (audio task, core 0, I2S DMA)  -> PCM5102A
 *
 * Tasks:
 *   audio       core 0, prio 5  owns I2S and all playback state
 *   stream_rx   core 0, prio 4  owns Serial; the only producer into the ring
 *   loopTask    core 1, prio 1  this file: OLED rendering + BOOT button, never touches I2S/Serial RX
 *
 * The OLED bars show the spectrum of the audio actually being played (10 Goertzel
 * bands computed in the audio task). With no stream active, the original synthetic
 * animation runs exactly as in Milestone 02/03.
 *
 * Built-in test generators (no laptop needed, only while no stream is active):
 *   - Serial monitor (921600!): 's' scale, '1' 440 Hz, '0' off, 't' toggle tone, 'e' error count
 *   - BOOT button (GPIO0): cycles OFF -> TONE -> SCALE -> OFF
 */

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

#include "audio_out.h"
#include "stream_rx.h"

// -----------------------------------------------------------------------------
// Verified hardware configuration
// -----------------------------------------------------------------------------
static constexpr uint8_t SDA_PIN  = 21;
static constexpr uint8_t SCL_PIN  = 22;
static constexpr uint8_t I2C_ADDR = 0x3C;

// Play the built-in scale at boot? Off by default: opening the COM port from the laptop
// usually resets the ESP32, so the scale would play at the start of every playback.
static constexpr bool    PLAY_SCALE_AT_BOOT = false;

// The old 1 Hz sine for the Arduino Serial Plotter. Off: it would write ~30 text lines/s
// onto the same UART that carries the binary protocol.
static constexpr bool    SERIAL_PLOTTER_DEMO = false;

static constexpr uint8_t BOOT_BTN_PIN    = 0;     // DevKit BOOT button, active low
static constexpr uint32_t BTN_DEBOUNCE_MS = 30;

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// -----------------------------------------------------------------------------
// Visualizer geometry
// -----------------------------------------------------------------------------
static constexpr uint8_t NUM_BANDS = 10;
static_assert(NUM_BANDS == audio::NUM_BANDS, "visualizer and audio analyzer band counts differ");

static constexpr uint8_t BAR_X0    = 6;
static constexpr uint8_t BAR_PITCH = 12;
static constexpr uint8_t BAR_W     = 8;

static constexpr uint8_t BASELINE_Y = 59;
static constexpr uint8_t TOP_Y      = 18;
static constexpr uint8_t MAX_BAR_H  = 40;

static constexpr uint8_t SEG_H     = 2;
static constexpr uint8_t SEG_PITCH = 3;

static constexpr uint8_t PEAK_HOLD_FRAMES = 12;
static constexpr uint8_t PEAK_DECAY       = 3;

static constexpr uint8_t BEAT_PERIOD_FRAMES = 14;
static constexpr uint8_t BEAT_DECAY         = 18;

static constexpr uint32_t FRAME_INTERVAL_MS = 33;  // 30 FPS target

// -----------------------------------------------------------------------------
// Fixed-size animation state
// -----------------------------------------------------------------------------
static uint16_t phase[NUM_BANDS];
static uint16_t lfsr[NUM_BANDS];
static uint8_t  env[NUM_BANDS];
static uint8_t  peak[NUM_BANDS];
static uint8_t  hold[NUM_BANDS];

static uint8_t  beatEnv      = 0;
static uint8_t  beatTimer    = 0;
static uint8_t  overallLevel = 0;
static uint32_t lastFrame    = 0;
static bool     liveViz      = false;   // bars driven by real audio
static bool     vizBuffering = false;   // stream open but still filling the ring

// -----------------------------------------------------------------------------
// Integer-only sine table: 0..255, one full cycle across 64 entries
// -----------------------------------------------------------------------------
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

// Per-band oscillator speed and spectral tilt.
// Low bands are stronger and slightly slower; high bands are weaker and faster.
static const uint16_t BAND_SPEED[NUM_BANDS] = {
    1800, 2100, 2400, 2700, 3000, 3300, 3600, 3900, 4200, 4500
};
static const uint8_t BAND_GAIN[NUM_BANDS] = {
    235, 220, 205, 190, 178, 166, 156, 146, 136, 126
};
static const uint8_t BEAT_GAIN[NUM_BANDS] = {
    100, 88, 72, 58, 44, 32, 24, 18, 13, 10
};

// -----------------------------------------------------------------------------
// Forward declarations
// -----------------------------------------------------------------------------
static void updateVisualizer();
static void renderFrame();
static void pollBootButton(uint32_t now);
static void announceMode(audio::Mode m);
static void onConsoleChar(char c);

// -----------------------------------------------------------------------------
// Setup
// Order: stream receiver (configures Serial) -> OLED (unchanged M02 path) -> first frame
//        -> I2S/audio task.
// OLED comes before audio so the display is already up if audio init fails.
// -----------------------------------------------------------------------------
void setup() {
    stream::setConsoleHandler(onConsoleChar);
    const bool rxOk = stream::begin();       // Serial.begin(921600) + receiver task

    pinMode(BOOT_BTN_PIN, INPUT_PULLUP);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);

    u8g2.setI2CAddress(I2C_ADDR << 1);
    u8g2.begin();
    u8g2.setPowerSave(0);

    // Re-apply bus clock after U8g2 initialization.
    Wire.setClock(400000);

    // Deterministic, independent LFSR seeds.
    for (uint8_t i = 0; i < NUM_BANDS; ++i) {
        lfsr[i] = (uint16_t)(0xACE1u + (uint16_t)i * 0x1234u);
        if (lfsr[i] == 0) lfsr[i] = 0xBEEFu;
    }

    updateVisualizer();
    renderFrame();
    lastFrame = millis();

    // --- Audio ---
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
}

// -----------------------------------------------------------------------------
// Non-blocking frame scheduler (unchanged) + non-blocking BOOT button
// -----------------------------------------------------------------------------
void loop() {
    const uint32_t now = millis();

    pollBootButton(now);

    if ((uint32_t)(now - lastFrame) < FRAME_INTERVAL_MS) {
        return;
    }
    lastFrame = now;

    if (SERIAL_PLOTTER_DEMO) {
        // Generate a 1 Hz sine wave for the Serial Plotter based on uptime
        float plotterSine = sin(now * 2.0 * PI / 1000.0) * 100.0;
        Serial.println(plotterSine);
    }

    updateVisualizer();
    renderFrame();
}

// -----------------------------------------------------------------------------
// Built-in generator controls: console characters and BOOT button (debounced, no delay())
// Both are ignored while a stream session is active.
// -----------------------------------------------------------------------------
static bool streamBusy() {
    return audio::streamStatus().state != audio::STREAM_IDLE;
}

static void announceMode(audio::Mode m) {
    if (streamBusy()) return;                 // keep the UART clean during playback
    Serial.print(F("[audio] mode: "));
    switch (m) {
        case audio::MODE_OFF:    Serial.println(F("OFF"));      break;
        case audio::MODE_TONE:   Serial.println(F("440 Hz"));   break;
        case audio::MODE_SCALE:  Serial.println(F("SCALE"));    break;
        case audio::MODE_STREAM: Serial.println(F("STREAM"));   break;
    }
}

// Called from the stream_rx task for ASCII bytes that arrive while no session is active.
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
    // BOOT button: OFF -> TONE -> SCALE -> OFF
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

// -----------------------------------------------------------------------------
// Synthetic signal generation, envelope smoothing, peak hold
// -----------------------------------------------------------------------------
static void updateVisualizer() {
    // Real spectrum while a stream session is active, otherwise the synthetic animation.
    uint8_t live[NUM_BANDS];
    liveViz      = audio::levels(live);
    vizBuffering = liveViz && (audio::streamStatus().state == audio::STREAM_BUFFERING);

    if (!liveViz) {
        // Slow beat pulse, roughly 1.8 Hz at 25-30 FPS.
        if (beatTimer > 0) {
            --beatTimer;
        } else {
            beatTimer = BEAT_PERIOD_FRAMES;
            beatEnv   = 255;
        }
        if (beatEnv > BEAT_DECAY) {
            beatEnv -= BEAT_DECAY;
        } else {
            beatEnv = 0;
        }
    }

    uint16_t sum = 0;

    for (uint8_t i = 0; i < NUM_BANDS; ++i) {
        uint16_t target;

        if (liveViz) {
            target = live[i];                      // 0..255 from the PCM being played
        } else {
            // Independent phase accumulators.
            phase[i] = (uint16_t)(phase[i] + BAND_SPEED[i]);

            // 16-bit Galois LFSR -> deterministic per-band noise.
            const uint16_t lsb = lfsr[i] & 1u;
            lfsr[i] = (uint16_t)((lfsr[i] >> 1) ^ (lsb ? 0xB400u : 0x0000u));

            const uint8_t s = SINE_TABLE[(phase[i] >> 10) & 63u];
            const uint8_t n = (uint8_t)(lfsr[i] & 0xFFu);

            // 75% periodic movement, 25% noise.
            const uint16_t base = ((uint16_t)s * 3u + n) >> 2;
            target = (base * BAND_GAIN[i]) >> 8;
            target += ((uint16_t)beatEnv * BEAT_GAIN[i]) >> 8;
            if (target > 255u) target = 255u;
        }

        const uint8_t t = (uint8_t)target;

        // Fast attack, slow release, integer-only.
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

        // Peak hold with controlled decay.
        if (env[i] >= peak[i]) {
            peak[i] = env[i];
            hold[i] = PEAK_HOLD_FRAMES;
        } else {
            if (hold[i] > 0) {
                --hold[i];
            } else if (peak[i] > 0) {
                peak[i] = (peak[i] > PEAK_DECAY)
                            ? (uint8_t)(peak[i] - PEAK_DECAY)
                            : 0;
            }
            if (peak[i] < env[i]) peak[i] = env[i];
        }

        sum += env[i];
    }

    overallLevel = (uint8_t)(sum / NUM_BANDS);
}

// -----------------------------------------------------------------------------
// Rendering: full framebuffer, one sendBuffer per frame
// -----------------------------------------------------------------------------
static void renderFrame() {
    u8g2.clearBuffer();
    u8g2.setDrawColor(1);

    // Header ---------------------------------------------------------------
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(4, 8, "SPECTRUM");
    u8g2.drawStr(86, 8, liveViz ? (vizBuffering ? "BUFF" : "LIVE") : "SYNTH");
    u8g2.drawHLine(2, 10, 124);

    // Tiny three-bar level meter.
    const uint8_t m1 = (uint8_t)(2u + (((uint16_t)overallLevel * 4u) >> 8));
    const uint8_t m2 = (uint8_t)(2u + (((uint16_t)overallLevel * 3u) >> 8));
    const uint8_t m3 = (uint8_t)(2u + (((uint16_t)overallLevel * 2u) >> 8));
    u8g2.drawBox(116, (uint8_t)(9 - m1), 2, m1);
    u8g2.drawBox(120, (uint8_t)(9 - m2), 2, m2);
    u8g2.drawBox(124, (uint8_t)(9 - m3), 2, m3);

    // Subtle left-edge scale ticks.
    u8g2.drawHLine(0, 22, 2);
    u8g2.drawHLine(0, 32, 2);
    u8g2.drawHLine(0, 42, 2);
    u8g2.drawHLine(0, 52, 2);

    // Spectrum bars --------------------------------------------------------
    for (uint8_t i = 0; i < NUM_BANDS; ++i) {
        const uint8_t x = (uint8_t)(BAR_X0 + i * BAR_PITCH);

        uint8_t h = (uint8_t)(((uint16_t)env[i] * MAX_BAR_H) >> 8);
        if (h > MAX_BAR_H) h = MAX_BAR_H;

        const uint8_t segs = h / SEG_PITCH;
        const uint8_t rem  = h % SEG_PITCH;

        // Full segments, bottom-up.
        for (uint8_t s = 0; s < segs; ++s) {
            const uint8_t y = (uint8_t)(BASELINE_Y - (s + 1) * SEG_PITCH + 1);
            u8g2.drawBox(x, y, BAR_W, SEG_H);
        }

        // Partial top segment for smoother height.
        if (rem > 0) {
            const uint8_t y = (uint8_t)(BASELINE_Y - segs * SEG_PITCH - rem);
            u8g2.drawBox(x, y, BAR_W, rem);
        }

        // Floating peak-hold cap.
        uint8_t ph = (uint8_t)(((uint16_t)peak[i] * MAX_BAR_H) >> 8);
        if (ph > MAX_BAR_H) ph = MAX_BAR_H;

        // Compare as uint16_t to avoid uint8_t overflow in env[i] + 6.
        if (ph > 2 && (uint16_t)peak[i] > (uint16_t)env[i] + 6u) {
            int16_t py = (int16_t)BASELINE_Y - (int16_t)ph - 2;
            if (py < TOP_Y) py = TOP_Y;
            if (py <= (int16_t)(BASELINE_Y - 2)) {
                u8g2.drawBox(x, (uint8_t)py, BAR_W, 2);
            }
        }
    }

    // Baseline and bottom tick marks.
    u8g2.drawHLine(2, 60, 124);
    for (uint8_t i = 0; i < NUM_BANDS; ++i) {
        const uint8_t cx = (uint8_t)(BAR_X0 + i * BAR_PITCH + (BAR_W / 2));
        u8g2.drawVLine(cx, 61, 2);
    }

    u8g2.sendBuffer();
}
