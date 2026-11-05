/*
 * Retro 128x64 OLED Audio Visualizer + laptop/BT-streamed MP3 player
 * Milestone 05 – A2DP Sink + Mochi UI
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
static void announceMode(audio::Mode m);
static void onConsoleChar(char c);
static void feedUI();
static void processBtSnapshot();

// -----------------------------------------------------------------------------
void setup() {
    stream::setConsoleHandler(onConsoleChar);
    const bool rxOk = stream::begin();

    pinMode(BOOT_BTN_PIN, INPUT_PULLUP);

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
        ui.setPlaying(true);
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