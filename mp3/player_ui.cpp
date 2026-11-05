#include "player_ui.h"
#include "boot_art.h"          // 128x64 1-bit cover art, generate with tools/img2boot.py
#include <string.h>
#include <math.h>

#define SCREEN_W 128
#define SCREEN_H 64

// Fonts: efont-based Japanese sets (kana + common kanji + ASCII), so track metadata in
// Japanese renders too. Heavier than the old X11 fonts (~15 KB flash each) - fine on ESP32.
#define UI_FONT_TITLE   u8g2_font_b10_b_t_japanese1
#define UI_FONT_ARTIST  u8g2_font_b10_t_japanese1
#define UI_FONT_TINY    u8g2_font_4x6_tf

#define BOOT_MS             5000
#define ENTRY_WIPE_MS        320
#define SCROLL_SPEED_PX_S     22.0f
#define SCROLL_GAP_PX         14
#define BLINK_INTERVAL_MS   3400
#define BLINK_DURATION_MS    140
#define FRAME_MIN_MS          33          // 30 FPS target
#define TEXT_X                24          // text column, right of the mascot
#define TITLE_AVAIL_W        100
#define ARTIST_AVAIL_W       100

#define MASCOT_X               3
#define MASCOT_Y              17

#define VIZ_ATTACK           0.55f
#define VIZ_RELEASE          0.14f
#define VIZ_PEAK_FALL_PER_S  60.0f
#define BEAT_ATTACK          0.60f
#define BEAT_RELEASE         0.12f

// ---- sprites: MSB-first, drawn with drawBitmap() -------------------------------------
// 16x16 twin-tail mascot. Index = face * 2 + sway. Faces: 0 open, 1 blink, 2 happy, 3 sleep.
const uint8_t PlayerUI::mascot_[8][32] = {
    // open
    { 0x07, 0xE0, 0x1F, 0xF8, 0x3F, 0xFC, 0x6F, 0xF6, 0xED, 0xB7, 0xE8, 0x17, 0xE8, 0x17, 0xCA, 0x53, 0xCA, 0x53, 0xC8, 0x13, 0x89, 0x91, 0x84, 0x21, 0x83, 0xC1, 0x47, 0xE2, 0x45, 0xA2, 0x22, 0x44 },
    // open_s
    { 0x07, 0xE0, 0x1F, 0xF8, 0x3F, 0xFC, 0x6F, 0xF6, 0xED, 0xB7, 0xE8, 0x17, 0xE8, 0x17, 0xCA, 0x53, 0xCA, 0x53, 0xC8, 0x13, 0xC9, 0x93, 0x84, 0x21, 0x83, 0xC1, 0x87, 0xE1, 0x45, 0xA2, 0x42, 0x42 },
    // blink
    { 0x07, 0xE0, 0x1F, 0xF8, 0x3F, 0xFC, 0x6F, 0xF6, 0xED, 0xB7, 0xE8, 0x17, 0xE8, 0x17, 0xC8, 0x13, 0xCB, 0x33, 0xC8, 0x13, 0x89, 0x91, 0x84, 0x21, 0x83, 0xC1, 0x47, 0xE2, 0x45, 0xA2, 0x22, 0x44 },
    // blink_s
    { 0x07, 0xE0, 0x1F, 0xF8, 0x3F, 0xFC, 0x6F, 0xF6, 0xED, 0xB7, 0xE8, 0x17, 0xE8, 0x17, 0xC8, 0x13, 0xCB, 0x33, 0xC8, 0x13, 0xC9, 0x93, 0x84, 0x21, 0x83, 0xC1, 0x87, 0xE1, 0x45, 0xA2, 0x42, 0x42 },
    // happy
    { 0x07, 0xE0, 0x1F, 0xF8, 0x3F, 0xFC, 0x6F, 0xF6, 0xED, 0xB7, 0xE8, 0x17, 0xE8, 0x17, 0xCD, 0x2D, 0xC8, 0x13, 0xC8, 0x13, 0x89, 0x91, 0x84, 0x21, 0x83, 0xC1, 0x47, 0xE2, 0x45, 0xA2, 0x22, 0x44 },
    // happy_s
    { 0x07, 0xE0, 0x1F, 0xF8, 0x3F, 0xFC, 0x6F, 0xF6, 0xED, 0xB7, 0xE8, 0x17, 0xE8, 0x17, 0xCD, 0x2D, 0xC8, 0x13, 0xC8, 0x13, 0xC9, 0x93, 0x84, 0x21, 0x83, 0xC1, 0x87, 0xE1, 0x45, 0xA2, 0x42, 0x42 },
    // sleep
    { 0x07, 0xE0, 0x1F, 0xF8, 0x3F, 0xFC, 0x6F, 0xF6, 0xED, 0xB7, 0xE8, 0x17, 0xE8, 0x17, 0xC8, 0x13, 0xCB, 0x33, 0xC8, 0x13, 0x89, 0x91, 0x84, 0x21, 0x83, 0xC1, 0x47, 0xE2, 0x45, 0xA2, 0x22, 0x44 },
    // sleep_s
    { 0x07, 0xE0, 0x1F, 0xF8, 0x3F, 0xFC, 0x6F, 0xF6, 0xED, 0xB7, 0xE8, 0x17, 0xE8, 0x17, 0xC8, 0x13, 0xCB, 0x33, 0xC8, 0x13, 0xC9, 0x93, 0x84, 0x21, 0x83, 0xC1, 0x87, 0xE1, 0x45, 0xA2, 0x42, 0x42 },
};

static const uint8_t kBTIcon[7]    = { 0x10,0x18,0x54,0x38,0x54,0x18,0x10 };
static const uint8_t kHeart[6]     = { 0x6C,0xFE,0xFE,0x7C,0x38,0x10 };          // 7x6
static const uint8_t kNote[7]      = { 0x10,0x18,0x14,0x10,0x70,0xF0,0x60 };     // 6x7

// ---- easing + tiny draw helpers ---------------------------------------------------------
static inline float clamp01(float x) { return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x); }
static inline float easeOutCubic(float t) { t = 1.0f - t; return 1.0f - t * t * t; }
static inline float easeInCubic(float t)  { return t * t * t; }
static inline float easeOutBack(float t) {
    const float c1 = 1.70158f, c3 = c1 + 1.0f;
    float u = t - 1.0f;
    return 1.0f + c3 * u * u * u + c1 * u * u;
}
static float easeOutBounce(float t) {
    const float n1 = 7.5625f, d1 = 2.75f;
    if (t < 1.0f / d1)      return n1 * t * t;
    if (t < 2.0f / d1)    { t -= 1.5f   / d1; return n1 * t * t + 0.75f; }
    if (t < 2.5f / d1)    { t -= 2.25f  / d1; return n1 * t * t + 0.9375f; }
    t -= 2.625f / d1;
    return n1 * t * t + 0.984375f;
}

// size 0 = dot, 1 = 3x3 plus, 2 = 5x5 four-point star
static void sparkle(U8G2* g, int cx, int cy, int size) {
    if (size <= 0) { g->drawPixel(cx, cy); return; }
    g->drawHLine(cx - size, cy, 2 * size + 1);
    g->drawVLine(cx, cy - size, 2 * size + 1);
    if (size >= 2) {
        g->drawPixel(cx - 1, cy - 1); g->drawPixel(cx + 1, cy - 1);
        g->drawPixel(cx - 1, cy + 1); g->drawPixel(cx + 1, cy + 1);
    }
}

// Twinkle: sparkle that cycles dot -> plus -> star -> plus -> dot -> hidden.
static void twinkle(U8G2* g, int cx, int cy, uint32_t now, uint32_t phaseMs) {
    uint32_t t = (now + phaseMs) % 1800;
    if (t > 900) return;
    int idx = (int)(t / 180);                       // 0..4
    static const uint8_t sz[5] = {0, 1, 2, 1, 0};
    sparkle(g, cx, cy, sz[idx]);
}

static void centerText(U8G2* g, const uint8_t* font, int cx, int baseline, const char* s) {
    g->setFont(font);
    g->drawUTF8(cx - g->getUTF8Width(s) / 2, baseline, s);
}

void PlayerUI::begin(U8G2* display) {
    u8g2_ = display;
    u8g2_->setBitmapMode(1);                        // transparent bitmaps so sparkles can overlap
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

    // Beat energy (low bands) drives the mascot bounce. Fast attack, slow release.
    float bass = active ? (vizLevel_[0] + vizLevel_[1] + vizLevel_[2]) / 300.0f : 0.0f;
    beat_ += (bass - beat_) * (bass > beat_ ? BEAT_ATTACK : BEAT_RELEASE);
}

// 0 open, 1 blink, 2 happy, 3 sleep
uint8_t PlayerUI::pickFace(uint32_t now) const {
    if (screen_ == UIScreen::IDLE || screen_ == UIScreen::PAUSED) return 3;
    if (screen_ == UIScreen::NOW_PLAYING && beat_ > 0.55f)         return 2;
    return ((now % BLINK_INTERVAL_MS) < BLINK_DURATION_MS) ? 1 : 0;
}

void PlayerUI::drawMascot(int x, int y, uint8_t face, bool sway) {
    if (face > 3) face = 0;
    u8g2_->drawBitmap(x, y, 2, 16, mascot_[face * 2 + (sway ? 1 : 0)]);
}

// ---- status bar --------------------------------------------------------------------------
void PlayerUI::drawStatusBar(bool paused) {
    uint32_t now = millis();

    // Round play/pause button, pulses on the beat while playing.
    const int bx = 6, by = 5;
    if (!paused) {
        u8g2_->drawDisc(bx, by, 5);
        u8g2_->setDrawColor(0);
        u8g2_->drawTriangle(bx - 1, by - 2, bx - 1, by + 2, bx + 2, by);
        u8g2_->setDrawColor(1);
        if (beat_ > 0.45f) u8g2_->drawCircle(bx, by, 5);    // brief outline flash = beat tick
    } else {
        u8g2_->drawCircle(bx, by, 5);
        u8g2_->drawBox(bx - 2, by - 2, 2, 5);
        u8g2_->drawBox(bx + 1, by - 2, 2, 5);
    }

    // Wordmark flanked by hearts
    u8g2_->setFont(UI_FONT_TINY);
    u8g2_->drawStr(26, 8, "M I K U");
    u8g2_->drawBitmap(18, 3, 1, 6, kHeart);
    if (((now / 500) & 1) == 0) u8g2_->drawBitmap(58, 3, 1, 6, kHeart);
    else                        u8g2_->drawBitmap(59, 2, 1, 6, kHeart);

    if (btConnected_) drawBluetoothIcon(SCREEN_W - 9, 2, now);

    // Phone-style signal bars for volume, rounded tops
    int vBars = (lastVol_ + 10) / 20;                      // 0..100 -> 0..5
    for (int i = 0; i < 5; ++i) {
        int h = 2 + i;
        int vx = SCREEN_W - 15 - (4 - i) * 3;
        if (i < vBars) u8g2_->drawBox(vx, 9 - h, 2, h);
        else           u8g2_->drawFrame(vx, 9 - h, 2, h);
    }

    // Dotted divider with sparkle caps
    for (int x = 4; x < SCREEN_W - 4; x += 2) u8g2_->drawPixel(x, 11);
    sparkle(u8g2_, 1, 11, 1);
    sparkle(u8g2_, SCREEN_W - 2, 11, 1);
}

void PlayerUI::drawBluetoothIcon(int x, int y, uint32_t now) {
    if (searching_) {
        if (((now / 400) & 1) == 0) u8g2_->drawBitmap(x, y, 1, 7, kBTIcon);
    } else {
        u8g2_->drawBitmap(x, y, 1, 7, kBTIcon);
    }
}

void PlayerUI::drawMarquee(int x, int baseline, int availW,
                           const char* text, int16_t w, float scroll,
                           const uint8_t* font) {
    if (!text || !*text) return;
    u8g2_->setClipWindow(x, baseline - 11, x + availW - 1, baseline + 3);
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

// Rounded capsule bars with a floating dot as the peak marker.
void PlayerUI::drawVisualizer(int baseY, int maxH, bool active, uint32_t now) {
    (void)active; (void)now;
    const int barW = 8, gap = 4;
    const int totalW = 10 * barW + 9 * gap;
    const int startX = (SCREEN_W - totalW) / 2;
    const int floorY = baseY + maxH;                       // first row below the bars

    u8g2_->setClipWindow(0, baseY - 3, SCREEN_W - 1, floorY - 1);
    for (int i = 0; i < 10; ++i) {
        int h     = (int)(vizLevel_[i] * maxH / 100.0f);
        int peakH = (int)(vizPeak_[i]  * maxH / 100.0f);
        int x     = startX + i * (barW + gap);
        if (h >= 3) {
            // Pill is drawn taller than needed and the clip window hides the flat-bottom rounding.
            u8g2_->drawRBox(x, floorY - h, barW, h + 4, 3);
        } else if (h > 0) {
            u8g2_->drawBox(x + 1, floorY - h, barW - 2, h);
        }
        if (peakH > h + 2) u8g2_->drawBox(x + 2, floorY - peakH - 2, barW - 4, 2);
    }
    u8g2_->setMaxClipWindow();

    // Dotted floor
    for (int x = startX - 2; x < startX + totalW + 2; x += 2) u8g2_->drawPixel(x, SCREEN_H - 1);
}

// Venetian-blind reveal: 8 stripes of 8 px, each retracting with a small stagger.
void PlayerUI::drawEntryWipe() {
    uint32_t elapsed = millis() - screenEnteredMs_;
    if (elapsed >= ENTRY_WIPE_MS) return;
    float p = (float)elapsed / ENTRY_WIPE_MS;
    u8g2_->setDrawColor(0);
    for (int s = 0; s < 8; ++s) {
        float lp = clamp01((p * 1.4f) - s * 0.05f);
        int h = (int)((1.0f - easeOutCubic(lp)) * 8.0f + 0.5f);
        if (h > 0) u8g2_->drawBox(0, s * 8, SCREEN_W, h);
    }
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

    // Cover art sweeps in left-to-right behind a bright scan line.
    float rp = clamp01(t / 900.0f);
    int revealW = (int)(SCREEN_W * easeOutCubic(rp));
    if (revealW > 0) {
        u8g2_->setClipWindow(0, 0, revealW - 1, SCREEN_H - 1);
        u8g2_->drawBitmap(0, 0, SCREEN_W / 8, SCREEN_H, BOOT_ART);
        u8g2_->setMaxClipWindow();
        if (rp < 1.0f) u8g2_->drawVLine(revealW, 0, SCREEN_H);
    }

    // Title plate: pops in over the art with overshoot.
    if (t >= 1000) {
        float p = clamp01((t - 1000) / 450.0f);
        float g = easeOutBack(p);
        int pw = (int)(72 * (g < 0.0f ? 0.0f : g));
        if (pw >= 20) {
            const int ph = 24, py = 36, cx = SCREEN_W / 2;
            int px = cx - pw / 2;
            u8g2_->setDrawColor(0);
            u8g2_->drawRBox(px - 2, py - 2, pw + 4, ph + 4, 9);
            u8g2_->setDrawColor(1);
            u8g2_->drawRFrame(px, py, pw, ph, 8);
            if (pw > 60) {
                centerText(u8g2_, UI_FONT_TITLE, cx, py + 13, "ミク");
                centerText(u8g2_, UI_FONT_TINY, cx, py + 20, "P L A Y E R");
            }
        }
    }

    // Thin loading line along the bottom edge.
    int fill = (int)(SCREEN_W * clamp01((float)t / BOOT_MS));
    u8g2_->setDrawColor(0);
    u8g2_->drawBox(0, SCREEN_H - 3, SCREEN_W, 3);
    u8g2_->setDrawColor(1);
    u8g2_->drawHLine(0, SCREEN_H - 2, fill);
}

void PlayerUI::renderPairing() {
    uint32_t now = millis();
    uint32_t t   = now - screenEnteredMs_;

    // BT icon with ripples expanding outwards
    const int cx = 24, cy = 24;
    if (!btConnected_) {
        for (int k = 0; k < 3; ++k) {
            uint32_t ph = (now + k * 500) % 1500;
            int r = 5 + (int)(easeOutCubic(ph / 1500.0f) * 11.0f);
            if (ph < 1100) u8g2_->drawCircle(cx, cy, r, U8G2_DRAW_ALL);
        }
    }
    u8g2_->setDrawColor(0);
    u8g2_->drawDisc(cx, cy, 6);
    u8g2_->setDrawColor(1);
    u8g2_->drawBitmap(cx - 3, cy - 3, 1, 7, kBTIcon);

    // Mascot sways at the right
    bool sway = ((now / 450) & 1) != 0;
    int bob = (int)(sinf(now * 0.004f) * 1.4f);
    drawMascot(88, 14 + bob, btConnected_ ? 2 : pickFace(now), sway);
    twinkle(u8g2_, 82, 14, now, 0);
    twinkle(u8g2_, 112, 20, now, 700);
    (void)t;

    centerText(u8g2_, UI_FONT_TITLE, SCREEN_W / 2, 49,
               btConnected_ ? "つながった!" : "ペアリング...");
    centerText(u8g2_, UI_FONT_TINY, SCREEN_W / 2, 60,
               btConnected_ ? "ready" : "on your phone");
}

void PlayerUI::renderNowPlaying(bool paused) {
    uint32_t now = millis();
    drawStatusBar(paused);

    // Mascot: bounces with the bass, tails swing faster the harder the beat.
    int bounce = paused ? 0 : (int)(beat_ * 3.0f + 0.5f);
    uint32_t swayMs = paused ? 700 : (beat_ > 0.4f ? 180 : 320);
    drawMascot(MASCOT_X, MASCOT_Y - bounce, pickFace(now), ((now / swayMs) & 1) != 0);
    if (!paused && beat_ > 0.5f) {
        twinkle(u8g2_, MASCOT_X + 18, MASCOT_Y + 2, now, 0);
        twinkle(u8g2_, MASCOT_X - 1,  MASCOT_Y + 20, now, 600);
    }
    if (paused) {                                           // floating z's while she naps
        uint32_t zt = now % 2400;
        if (zt < 1400) u8g2_->drawStr(MASCOT_X + 17, MASCOT_Y + 8 - (int)(zt * 0.006f), "z");
    }

    drawMarquee(TEXT_X, 25, TITLE_AVAIL_W, title_, titleWidth_,
                titleScroll_, (const uint8_t*)UI_FONT_TITLE);
    drawMarquee(TEXT_X, 38, ARTIST_AVAIL_W, artist_, artistWidth_,
                artistScroll_, (const uint8_t*)UI_FONT_ARTIST);
    drawVisualizer(42, 21, !paused, now);
}

void PlayerUI::renderNoMetadata(bool paused) {
    uint32_t now = millis();
    drawStatusBar(paused);

    int bounce = (int)(beat_ * 3.0f + 0.5f);
    drawMascot(MASCOT_X, MASCOT_Y - bounce, pickFace(now), ((now / 320) & 1) != 0);

    // Three bouncing notes instead of a title
    for (int k = 0; k < 3; ++k) {
        int ny = 15 + (int)(3.0f + 3.0f * sinf(now * 0.008f + k * 1.2f));
        u8g2_->drawBitmap(TEXT_X + 30 + k * 14, ny, 1, 7, kNote);
    }

    centerText(u8g2_, UI_FONT_ARTIST, TEXT_X + ARTIST_AVAIL_W / 2, 38, "Unknown Artist");
    drawVisualizer(42, 21, true, now);
}

void PlayerUI::renderIdle() {
    uint32_t now = millis();
    float bob = sinf(now * 0.0015f * 1.5f);
    int my = 14 + (bob > 0 ? 1 : 0);
    drawMascot((SCREEN_W - 16) / 2, my, 3, ((now / 900) & 1) != 0);

    uint32_t zt = now % 2100;
    for (int k = 0; k < 3; ++k) {
        uint32_t zone = (zt + k * 700) % 2100;
        if (zone < 900) {
            int zx = (SCREEN_W - 16) / 2 + 17 + (int)(zone * 0.008f);
            int zy = my + 8 - (int)(zone * 0.012f);
            u8g2_->setFont(UI_FONT_TINY);
            u8g2_->drawStr(zx, zy, "z");
        }
    }

    twinkle(u8g2_, 22, 14, now, 0);
    twinkle(u8g2_, 104, 20, now, 450);
    twinkle(u8g2_, 14, 40, now, 900);
    twinkle(u8g2_, 114, 42, now, 1300);

    centerText(u8g2_, UI_FONT_TITLE, SCREEN_W / 2, 46, "ミク");

    const char* hint = "connect phone";
    u8g2_->setFont(UI_FONT_TINY);
    int16_t wh = u8g2_->getUTF8Width(hint);
    int hx = (SCREEN_W - wh) / 2;
    u8g2_->drawUTF8(hx, 59, hint);
    u8g2_->drawBitmap(hx - 10, 54, 1, 6, kHeart);
    u8g2_->drawBitmap(hx + wh + 3, 54, 1, 6, kHeart);
}

void PlayerUI::setVolumeLevel(uint8_t vol) { lastVol_ = vol; }
void PlayerUI::showVolumeOverlay(uint8_t vol) { lastVol_ = vol; overlayVolMs_ = millis(); }
void PlayerUI::triggerPlayPause(bool playing) { overlayPlaying_ = playing; overlayPlayPauseMs_ = millis(); }

void PlayerUI::drawOverlays() {
    uint32_t now = millis();

    // 1. Play/Pause: pill banner that pops open with overshoot, holds, then collapses.
    if (overlayPlayPauseMs_ > 0 && (now - overlayPlayPauseMs_) < 1200) {
        float p = (now - overlayPlayPauseMs_) / 1200.0f;
        float grow;
        if      (p < 0.18f) grow = easeOutBack(p / 0.18f);
        else if (p > 0.80f) grow = 1.0f - easeInCubic((p - 0.80f) / 0.20f);
        else                grow = 1.0f;
        if (grow < 0.0f) grow = 0.0f;

        const int H = 24, FULLW = 96;
        int w = (int)(FULLW * grow);
        int cx = SCREEN_W / 2, cy = SCREEN_H / 2;
        if (w >= 12) {
            int x = cx - w / 2, y = cy - H / 2;
            u8g2_->setDrawColor(0);
            u8g2_->drawRBox(x - 2, y - 2, w + 4, H + 4, 12);   // knock out what is behind
            u8g2_->setDrawColor(1);
            u8g2_->drawRFrame(x, y, w, H, 11);
            u8g2_->drawRFrame(x + 2, y + 2, w - 4, H - 4, 9);  // double outline = sticker look

            if (w > 80) {
                // Round icon badge on the left
                int ix = x + 15, iy = cy;
                u8g2_->drawDisc(ix, iy, 7);
                u8g2_->setDrawColor(0);
                if (overlayPlaying_) {
                    u8g2_->drawTriangle(ix - 2, iy - 4, ix - 2, iy + 4, ix + 4, iy);
                } else {
                    u8g2_->drawBox(ix - 3, iy - 3, 2, 7);
                    u8g2_->drawBox(ix + 1, iy - 3, 2, 7);
                }
                u8g2_->setDrawColor(1);
                u8g2_->setFont(UI_FONT_TITLE);
                u8g2_->drawStr(x + 28, cy + 4, overlayPlaying_ ? "PLAY" : "PAUSE");
                u8g2_->drawBitmap(x + w - 14, cy - 3, 1, 6, kHeart);
            }
        }

        // Sparkle burst flying out from the banner
        if (p < 0.55f) {
            static const int8_t DIR[6][2] = {{-1,-1},{1,-1},{-1,1},{1,1},{0,-1},{0,1}};
            float e = easeOutCubic(p / 0.55f);
            for (int k = 0; k < 6; ++k) {
                int sx = cx + (int)(DIR[k][0] * (30 + 30 * e));
                int sy = cy + (int)(DIR[k][1] * (14 + 14 * e));
                sparkle(u8g2_, sx, sy, p < 0.25f ? 2 : (p < 0.4f ? 1 : 0));
            }
        }
    }

    // 2. Volume: capsule bar sliding up from the bottom edge with overshoot.
    if (overlayVolMs_ > 0 && (now - overlayVolMs_) < 1500) {
        float p = (now - overlayVolMs_) / 1500.0f;
        float slide;
        if      (p < 0.12f) slide = easeOutBack(p / 0.12f);
        else if (p > 0.82f) slide = 1.0f - easeInCubic((p - 0.82f) / 0.18f);
        else                slide = 1.0f;
        if (slide < 0.0f) slide = 0.0f;

        const int H = 20;
        int y = SCREEN_H - (int)(H * slide);
        u8g2_->setDrawColor(0);
        u8g2_->drawRBox(0, y, SCREEN_W, H + 8, 7);
        u8g2_->setDrawColor(1);
        u8g2_->drawRFrame(0, y, SCREEN_W, H + 8, 7);

        u8g2_->drawBitmap(8, y + 7, 1, 6, kHeart);

        const int maxBars = 14;
        int activeBars = (lastVol_ * maxBars + 50) / 100;
        for (int i = 0; i < maxBars; ++i) {
            int bx = 20 + i * 7;
            int bh = 5 + (i * 6) / maxBars;                  // ramp: 5..10 px
            int by = y + 14 - bh;
            if (i < activeBars) u8g2_->drawRBox(bx, by, 4, bh, 1);
            else                u8g2_->drawRFrame(bx, by, 4, bh, 1);
        }
    }
}