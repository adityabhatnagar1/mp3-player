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
    // Cyber/aesthetic top border line
    u8g2_->drawHLine(0, 11, SCREEN_W);

    if (hasTitle_ || hasArtist_) drawPlayPauseIcon(2, 2, !paused);
    if (btConnected_)            drawBluetoothIcon(SCREEN_W - 9, 2, millis());

    // Anime/Phone style volume signal bars right before the BT icon
    int vBars = (lastVol_ + 10) / 20; // Maps 0-100 to 0-5 bars
    for (int i = 0; i < 5; ++i) {
        int h = 2 + i; // Heights: 2, 3, 4, 5, 6 px
        int bx = SCREEN_W - 15 - (4 - i) * 3;
        if (i < vBars) u8g2_->drawBox(bx, 9 - h, 2, h);
        else           u8g2_->drawFrame(bx, 9 - h, 2, h);
    }
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
    drawOverlays();
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

void PlayerUI::setVolumeLevel(uint8_t vol) { lastVol_ = vol; }
void PlayerUI::showVolumeOverlay(uint8_t vol) { lastVol_ = vol; overlayVolMs_ = millis(); }
void PlayerUI::triggerPlayPause(bool playing) { overlayPlaying_ = playing; overlayPlayPauseMs_ = millis(); }

void PlayerUI::drawOverlays() {
    uint32_t now = millis();

    // 1. Play/Pause Cyber/Anime Burst Overlay
    if (overlayPlayPauseMs_ > 0 && (now - overlayPlayPauseMs_) < 1200) {
        float p = (now - overlayPlayPauseMs_) / 1200.0f;
        // Expands sharply, collapses slowly
        int h = (p < 0.1f) ? (int)(p * 10 * 24) : ((p > 0.8f) ? (int)((1.0f - p) * 5 * 24) : 24);
        int y = (SCREEN_H - h) / 2;

        u8g2_->setDrawColor(0);
        u8g2_->drawBox(0, y, SCREEN_W, h); // Mask background
        u8g2_->setDrawColor(1);
        u8g2_->drawHLine(0, y, SCREEN_W);
        u8g2_->drawHLine(0, y + h, SCREEN_W);

        if (h >= 16) {
            u8g2_->setFont(UI_FONT_TITLE);
            const char* txt = overlayPlaying_ ? ">> P L A Y" : "|| P A U S E";
            int tw = u8g2_->getUTF8Width(txt);
            // Thick Evangelion-style emphasis blocks on the sides
            u8g2_->drawBox(0, y + 4, (SCREEN_W - tw)/2 - 4, 16);
            u8g2_->drawBox((SCREEN_W + tw)/2 + 4, y + 4, SCREEN_W, 16);
            u8g2_->drawUTF8((SCREEN_W - tw) / 2, y + 16, txt);
        }
    }

    // 2. Volume Slide-in Overlay
    if (overlayVolMs_ > 0 && (now - overlayVolMs_) < 1500) {
        float p = (now - overlayVolMs_) / 1500.0f;
        // Slide in from bottom edge
        int y = (p < 0.1f) ? SCREEN_H - (int)(p * 10 * 16) : ((p > 0.8f) ? SCREEN_H - (int)((1.0f - p) * 5 * 16) : SCREEN_H - 16);

        u8g2_->setDrawColor(0);
        u8g2_->drawBox(0, y, SCREEN_W, 16);
        u8g2_->setDrawColor(1);
        u8g2_->drawHLine(0, y, SCREEN_W);

        u8g2_->setFont(UI_FONT_TITLE);
        u8g2_->drawStr(4, y + 12, "VOL");

        // Futuristic diagonal segmented blocks
        int maxBars = 15;
        int activeBars = (lastVol_ * maxBars) / 100;
        for (int i = 0; i < maxBars; ++i) {
            int bx = 32 + i * 6;
            if (i < activeBars) u8g2_->drawBox(bx, y + 4, 4, 8);
            else                u8g2_->drawFrame(bx, y + 4, 4, 8);
        }
    }
}