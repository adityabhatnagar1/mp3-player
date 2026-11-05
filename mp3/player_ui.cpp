#include "player_ui.h"
#include <string.h>
#include <math.h>

#define SCREEN_W 128
#define SCREEN_H 64

// Fonts: efont-based Japanese sets (kana + common kanji + ASCII) for track metadata.
#define UI_FONT_TITLE   u8g2_font_b10_b_t_japanese1
#define UI_FONT_ARTIST  u8g2_font_b10_t_japanese1
#define UI_FONT_TINY    u8g2_font_4x6_tf
#define UI_FONT_LOGO    u8g2_font_logisoso16_tr

#define BOOT_MS             3300
#define ENTRY_WIPE_MS        320
#define SCROLL_SPEED_PX_S     22.0f
#define SCROLL_GAP_PX         14
#define BLINK_INTERVAL_MS   3400
#define BLINK_DURATION_MS    140
#define FRAME_MIN_MS          33          // 30 FPS target
#define TEXT_X                28          // text column, right of the 24px mascot
#define TITLE_AVAIL_W         98
#define ARTIST_AVAIL_W        98

#define MASCOT_X               2
#define MASCOT_Y              15

#define VIZ_ATTACK           0.55f
#define VIZ_RELEASE          0.14f
#define VIZ_PEAK_FALL_PER_S  60.0f
#define BEAT_ATTACK          0.60f
#define BEAT_RELEASE         0.12f

// ---- sprites: 24x24, MSB-first, 3 bytes/row, drawn with drawBitmap(x,y,3,24,..) ---------
// Original "mochi blob" with a floating star. Index = face * 2 + sway.
// Faces: 0 open, 1 blink, 2 happy, 3 sleep.
const uint8_t PlayerUI::mascot_[8][72] = {
    // open
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x10, 0x00, 0x00, 0x7C, 0x00, 0x00, 0x38, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0x80, 0x07, 0xFF, 0xE0, 0x1F, 0xFF, 0xF8, 0x1F, 0xFF, 0xF8, 0x3F, 0x7F, 0x78, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7C, 0x6B, 0xFF, 0xD6, 0x77, 0xDB, 0xEC, 0x7F, 0xE7, 0xFE, 0x7F, 0xFF, 0xFC, 0x3F, 0xFF, 0xF4, 0x0F, 0x55, 0x40, 0x00, 0x00, 0x00 },
    // open_s
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x04, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xC0, 0x0F, 0xFF, 0xF0, 0x1F, 0xFF, 0xF8, 0x3F, 0xFF, 0xF8, 0x7F, 0x7F, 0x7E, 0x7E, 0x7E, 0x7C, 0xFE, 0x7E, 0x7F, 0xEB, 0xFF, 0xD6, 0xF7, 0xDB, 0xEF, 0xFF, 0xE7, 0xFE, 0x7F, 0xFF, 0xF4, 0x0F, 0x55, 0x40, 0x00, 0x00, 0x00 },
    // blink
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x10, 0x00, 0x00, 0x7C, 0x00, 0x00, 0x38, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0x80, 0x07, 0xFF, 0xE0, 0x1F, 0xFF, 0xF8, 0x1F, 0xFF, 0xF8, 0x3F, 0xFF, 0xF8, 0x7F, 0xFF, 0xFE, 0x7C, 0x3C, 0x3C, 0x6B, 0xFF, 0xD6, 0x77, 0xDB, 0xEC, 0x7F, 0xE7, 0xFE, 0x7F, 0xFF, 0xFC, 0x3F, 0xFF, 0xF4, 0x0F, 0x55, 0x40, 0x00, 0x00, 0x00 },
    // blink_s
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x04, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xC0, 0x0F, 0xFF, 0xF0, 0x1F, 0xFF, 0xF8, 0x3F, 0xFF, 0xF8, 0x7F, 0xFF, 0xFE, 0x7F, 0xFF, 0xFC, 0xFC, 0x3C, 0x3F, 0xEB, 0xFF, 0xD6, 0xF7, 0xDB, 0xEF, 0xFF, 0xE7, 0xFE, 0x7F, 0xFF, 0xF4, 0x0F, 0x55, 0x40, 0x00, 0x00, 0x00 },
    // happy
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x10, 0x00, 0x00, 0x7C, 0x00, 0x00, 0x38, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0x80, 0x07, 0xFF, 0xE0, 0x1F, 0xFF, 0xF8, 0x1F, 0xFF, 0xF8, 0x3F, 0x7F, 0x78, 0x7E, 0xBE, 0xBE, 0x7D, 0xDD, 0xDC, 0x6B, 0xFF, 0xD6, 0x77, 0xC3, 0xEC, 0x7F, 0xE7, 0xFE, 0x7F, 0xFF, 0xFC, 0x3F, 0xFF, 0xF4, 0x0F, 0x55, 0x40, 0x00, 0x00, 0x00 },
    // happy_s
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x04, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xC0, 0x0F, 0xFF, 0xF0, 0x1F, 0xFF, 0xF8, 0x3F, 0xFF, 0xF8, 0x7F, 0x7F, 0x7E, 0x7E, 0xBE, 0xBC, 0xFD, 0xDD, 0xDF, 0xEB, 0xFF, 0xD6, 0xF7, 0xC3, 0xEF, 0xFF, 0xE7, 0xFE, 0x7F, 0xFF, 0xF4, 0x0F, 0x55, 0x40, 0x00, 0x00, 0x00 },
    // sleep
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x10, 0x00, 0x00, 0x7C, 0x00, 0x00, 0x38, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0x80, 0x07, 0xFF, 0xE0, 0x1F, 0xFF, 0xF8, 0x1F, 0xFF, 0xF8, 0x3F, 0xFF, 0xF8, 0x7D, 0xBD, 0xBE, 0x7E, 0x7E, 0x7C, 0x6B, 0xFF, 0xD6, 0x77, 0xE7, 0xEC, 0x7F, 0xFF, 0xFE, 0x7F, 0xFF, 0xFC, 0x3F, 0xFF, 0xF4, 0x0F, 0x55, 0x40, 0x00, 0x00, 0x00 },
    // sleep_s
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x04, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xC0, 0x0F, 0xFF, 0xF0, 0x1F, 0xFF, 0xF8, 0x3F, 0xFF, 0xF8, 0x7F, 0xFF, 0xFE, 0x7D, 0xBD, 0xBC, 0xFE, 0x7E, 0x7F, 0xEB, 0xFF, 0xD6, 0xF7, 0xE7, 0xEF, 0xFF, 0xFF, 0xFE, 0x7F, 0xFF, 0xF4, 0x0F, 0x55, 0x40, 0x00, 0x00, 0x00 },
};

static const uint8_t kBTIcon[7] = { 0x10,0x18,0x54,0x38,0x54,0x18,0x10 };
static const uint8_t kStar[7]   = { 0x10,0x38,0xFE,0x7C,0x38,0x6C,0x44 };   // 7x7 five-point star
static const uint8_t kNote[7]   = { 0x10,0x18,0x14,0x10,0x70,0xF0,0x60 };   // 6x7

// Fixed starfield: x, y, twinkle phase
static const uint8_t STARS[][3] = {
    {  5,  5, 0}, { 22,  3, 4}, { 38,  9, 7}, { 57,  4, 2}, { 74,  8, 5}, { 93,  3, 1},
    {110,  7, 6}, {121, 15, 3}, {  4, 24, 8}, {  3, 44, 2}, {120, 30, 5}, {123, 40, 9},
    {  6, 34, 1}, {124, 47, 6}, {112, 26, 4}, { 27, 51, 7}, {101, 51, 0},
};
static const int NUM_STARS = sizeof(STARS) / sizeof(STARS[0]);

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
    if (cx < 0 || cy < 0) return;
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
    int x = cx - (int)g->getUTF8Width(s) / 2;
    if (x < 0) x = 0;
    g->drawUTF8(x, baseline, s);
}

// ---- lifecycle ---------------------------------------------------------------------------
void PlayerUI::begin(U8G2* display) {
    u8g2_ = display;
    u8g2_->setBitmapMode(1);                        // transparent bitmaps so sparkles can overlap
    // Keep the panel dark until the first real frame is on its way.
    u8g2_->clearBuffer();
    u8g2_->sendBuffer();
    u8g2_->setPowerSave(1);

    screen_ = UIScreen::BOOT;
    firstFrame_ = true;
    screenEnteredMs_ = millis();                    // re-stamped when the first frame renders
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

    // The boot animation clock starts at the first frame that is actually drawn,
    // not when begin() ran, so slow init work can no longer eat the animation.
    if (firstFrame_) screenEnteredMs_ = now;

    if (screen_ == UIScreen::BOOT && (now - screenEnteredMs_) >= BOOT_MS)
        enterScreen(computeAutoScreen());

    if (autoMode_ && screen_ != UIScreen::BOOT) {
        UIScreen s = computeAutoScreen();
        if (s != screen_) enterScreen(s);
    }

    // Eased volume: every volume graphic follows this, so bars light up/down smoothly.
    float dv = (float)lastVol_ - dispVol_;
    dispVol_ += dv * 0.35f;
    if (fabsf((float)lastVol_ - dispVol_) < 0.4f) dispVol_ = (float)lastVol_;

    updateScroll(dt);
    updateVisualizer(dt);

    u8g2_->clearBuffer();
    render();
    u8g2_->sendBuffer();

    if (firstFrame_) {                      // panel turns on only once a frame is in its RAM
        firstFrame_ = false;
        u8g2_->setPowerSave(0);
    }
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
    if (x < 0 || y < 0) return;
    u8g2_->drawBitmap(x, y, 3, 24, mascot_[face * 2 + (sway ? 1 : 0)]);
}

// ---- space decor -------------------------------------------------------------------------
// Twinkling background stars (dot -> plus -> dot -> off), restricted to a y band.
void PlayerUI::drawStarfield(uint32_t now, int yMin, int yMax) {
    for (int i = 0; i < NUM_STARS; ++i) {
        const int x = STARS[i][0], y = STARS[i][1];
        if (y < yMin || y > yMax) continue;
        const int s = (int)((now / 260 + STARS[i][2]) % 10);
        if (s < 6)       u8g2_->drawPixel(x, y);
        else if (s < 8)  sparkle(u8g2_, x, y, 1);
        else if (s == 8) u8g2_->drawPixel(x, y);
    }
}

// Sony-XMB-style silk ribbons: one solid wave plus two dotted ones that drift at different speeds.
void PlayerUI::drawWaves(uint32_t now, int baseY, float amp) {
    const float t = now * 0.001f;
    for (int layer = 0; layer < 3; ++layer) {
        const float f  = 0.050f + layer * 0.018f;
        const float ph = t * (0.9f + layer * 0.35f) + layer * 1.9f;
        const float a  = amp * (1.0f - layer * 0.22f);
        int prevY = -1;
        for (int x = 0; x < SCREEN_W; x += 2) {
            float s = sinf(x * f + ph) + 0.5f * sinf(x * f * 0.45f - ph * 0.6f);
            int y = baseY + layer * 2 + (int)(s * a * 0.67f + (s < 0 ? -0.5f : 0.5f));
            if (y < 0) y = 0;
            if (y > SCREEN_H - 1) y = SCREEN_H - 1;
            if (layer == 0) { if (prevY >= 0) u8g2_->drawLine(x - 2, prevY, x, y); }
            else            { u8g2_->drawPixel(x, y); }
            prevY = y;
        }
    }
}

// ---- status bar --------------------------------------------------------------------------
// Speaker glyph + 5 rising bars. Bars fill smoothly from dispVol_, the one being crossed
// shows a partial fill, so turning the knob (or the phone slider) visibly "pours" level in/out.
void PlayerUI::drawVolumeMeter(int x, int y) {
    // speaker: magnet + cone
    u8g2_->drawBox(x, y + 2, 2, 3);
    u8g2_->drawVLine(x + 2, y + 2, 3);
    u8g2_->drawVLine(x + 3, y + 1, 5);
    u8g2_->drawVLine(x + 4, y,     7);

    if (lastVol_ == 0 && dispVol_ < 1.0f) {                 // muted: little X
        u8g2_->drawLine(x + 8,  y + 1, x + 12, y + 5);
        u8g2_->drawLine(x + 8,  y + 5, x + 12, y + 1);
        return;
    }
    const float lvl = dispVol_ * 5.0f / 100.0f;             // 0..5
    for (int i = 0; i < 5; ++i) {
        const int h  = 3 + i;                               // 3..7 px
        const int bx = x + 7 + i * 4;
        const int by = y + 7 - h;
        const float f = lvl - (float)i;
        if (f >= 0.999f) {
            u8g2_->drawBox(bx, by, 3, h);
        } else {
            u8g2_->drawFrame(bx, by, 3, h);
            const int fh = (int)(f * h + 0.5f);
            if (f > 0.0f && fh > 0) u8g2_->drawBox(bx, by + h - fh, 3, fh);
        }
    }
}

void PlayerUI::drawStatusBar(bool paused) {
    uint32_t now = millis();

    // Round play/pause button, pulses on the beat while playing.
    const int bx = 6, by = 5;
    if (!paused) {
        u8g2_->drawDisc(bx, by, 5);
        u8g2_->setDrawColor(0);
        u8g2_->drawTriangle(bx - 1, by - 2, bx - 1, by + 2, bx + 2, by);
        u8g2_->setDrawColor(1);
        if (beat_ > 0.45f) u8g2_->drawCircle(bx, by, 5);
    } else {
        u8g2_->drawCircle(bx, by, 5);
        u8g2_->drawBox(bx - 2, by - 2, 2, 5);
        u8g2_->drawBox(bx + 1, by - 2, 2, 5);
    }

    // Wordmark flanked by stars (right one twinkles)
    u8g2_->setFont(UI_FONT_TINY);
    u8g2_->drawStr(29, 8, "M O C H I");
    u8g2_->drawBitmap(18, 2, 1, 7, kStar);
    if (((now / 500) & 1) == 0) u8g2_->drawBitmap(69, 2, 1, 7, kStar);
    else                        sparkle(u8g2_, 72, 5, 2);

    drawVolumeMeter(84, 2);
    if (btConnected_) drawBluetoothIcon(SCREEN_W - 9, 2, now);

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

// Marquee that never hands a negative x to U8g2 (u8g2_uint_t is unsigned by default):
// whole glyphs that have scrolled out of the window are skipped instead.
void PlayerUI::drawMarquee(int x, int baseline, int availW,
                           const char* text, int16_t w, float scroll,
                           const uint8_t* font) {
    if (!text || !*text) return;
    u8g2_->setClipWindow(x, baseline - 11, x + availW - 1, baseline + 3);
    u8g2_->setFont((const uint8_t*)font);
    if (w <= availW) {
        u8g2_->drawUTF8(x + (availW - w) / 2, baseline, text);
    } else {
        const int total = w + SCROLL_GAP_PX;
        int off = (int)scroll;                       // px scrolled, 0..total
        for (int pass = 0; pass < 2; ++pass) {
            // pass 0 = the text, pass 1 = the wrapped copy that follows it
            int start = pass == 0 ? -off : total - off;   // text origin relative to x
            const char* p = text;
            if (start < 0) {
                // skip whole UTF-8 characters that are completely left of the window
                while (*p) {
                    const char* q = p + 1;
                    while ((*q & 0xC0) == 0x80) ++q;
                    char tmp[8]; int n = (int)(q - p); if (n > 7) n = 7;
                    memcpy(tmp, p, n); tmp[n] = 0;
                    int cw = u8g2_->getUTF8Width(tmp);
                    if (start + cw > 0) break;       // this glyph is (partly) visible
                    start += cw; p = q;
                }
                if (!*p) continue;
            }
            int dx = x + start;
            if (dx < 0) dx = 0;                      // at most a few px of glyph clipped
            if (dx <= x + availW) u8g2_->drawUTF8(dx, baseline, p);
        }
    }
    u8g2_->setMaxClipWindow();
}

// Rounded capsule bars with a floating dot as the peak marker.
void PlayerUI::drawVisualizer(int baseY, int maxH, bool active, uint32_t now) {
    (void)active; (void)now;
    const int barW = 8, gap = 4;
    const int totalW = 10 * barW + 9 * gap;
    const int startX = (SCREEN_W - totalW) / 2;
    const int floorY = baseY + maxH;

    u8g2_->setClipWindow(0, baseY - 3, SCREEN_W - 1, floorY - 1);
    for (int i = 0; i < 10; ++i) {
        int h     = (int)(vizLevel_[i] * maxH / 100.0f);
        int peakH = (int)(vizPeak_[i]  * maxH / 100.0f);
        int x     = startX + i * (barW + gap);
        if (h >= 3) {
            u8g2_->drawRBox(x, floorY - h, barW, h + 4, 3);
        } else if (h > 0) {
            u8g2_->drawBox(x + 1, floorY - h, barW - 2, h);
        }
        if (peakH > h + 2) u8g2_->drawBox(x + 2, floorY - peakH - 2, barW - 4, 2);
    }
    u8g2_->setMaxClipWindow();

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

// ---- boot --------------------------------------------------------------------------------
// stars twinkle in -> XMB waves swell -> mochi drops in and squishes -> letters pop -> tagline wipes in
void PlayerUI::renderBoot() {
    const uint32_t now = millis();
    const uint32_t t   = now - screenEnteredMs_;

    // 1. starfield appears star by star
    {
        int visible = (int)(t / 45);
        if (visible > NUM_STARS) visible = NUM_STARS;
        for (int i = 0; i < visible; ++i) {
            const int x = STARS[i][0], y = STARS[i][1];
            const int s = (int)((now / 260 + STARS[i][2]) % 10);
            if (s < 6) u8g2_->drawPixel(x, y);
            else if (s < 8) sparkle(u8g2_, x, y, 1);
        }
    }

    // 2. ribbons swell up from flat
    drawWaves(now, 54, 4.6f * easeOutCubic(clamp01(t / 1000.0f)));

    // 3. mochi drops from above with a bounce
    const int landY = 16;
    if (t >= 250) {
        float p = clamp01((t - 250) / 750.0f);
        int y = (int)(-26.0f + (landY + 26) * easeOutBounce(p));
        bool landed = p >= 1.0f;
        bool sway = !landed && (((t / 110) & 1) != 0);
        uint8_t face = landed ? (t < 1900 ? 2 : (((now % BLINK_INTERVAL_MS) < BLINK_DURATION_MS) ? 1 : 0)) : 0;
        if (y >= 0) drawMascot(14, y, face, sway);
        else        { /* still above the screen */ }
    }

    // 4. "Mochi" letters pop in, each dropping with overshoot
    static const char LOGO[] = "Mochi";
    if (t >= 1000) {
        u8g2_->setFont(UI_FONT_LOGO);
        const int x0 = 46, base = 36;
        char prefix[8] = {0};
        for (int i = 0; i < 5; ++i) {
            int32_t lt = (int32_t)t - 1000 - i * 110;
            if (lt < 0) break;
            float p = clamp01(lt / 320.0f);
            int dy = (int)((1.0f - easeOutBack(p)) * -14.0f);
            int lx = x0 + (i == 0 ? 0 : (int)u8g2_->getStrWidth(prefix));
            char one[2] = { LOGO[i], 0 };
            if (base + dy >= 0) u8g2_->drawStr(lx, base + dy, one);
            prefix[i] = LOGO[i]; prefix[i + 1] = 0;
        }
    }

    // 5. tagline wipes in left to right
    if (t >= 1750) {
        float p = clamp01((t - 1750) / 700.0f);
        int w = (int)(SCREEN_W * easeOutCubic(p));
        if (w > 0) {
            u8g2_->setClipWindow(0, 40, w - 1, 49);
            centerText(u8g2_, UI_FONT_TINY, SCREEN_W / 2, 47, "~ your cozy star speaker ~");
            u8g2_->setMaxClipWindow();
        }
    }

    // sparkles around the wordmark once it has landed
    if (t >= 1600) {
        twinkle(u8g2_, 108, 20, now, 0);
        twinkle(u8g2_, 100, 40 - 10, now, 700);
        twinkle(u8g2_, 40, 14, now, 1200);
    }
}

// ---- screens -----------------------------------------------------------------------------
void PlayerUI::renderPairing() {
    uint32_t now = millis();

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

    // Mochi sways at the right
    bool sway = ((now / 450) & 1) != 0;
    int bob = (int)(sinf(now * 0.004f) * 1.4f);
    drawMascot(84, 9 + bob + 1, btConnected_ ? 2 : pickFace(now), sway);
    twinkle(u8g2_, 78, 12, now, 0);
    twinkle(u8g2_, 116, 18, now, 700);
    twinkle(u8g2_, 116, 36, now, 1100);

    centerText(u8g2_, UI_FONT_TITLE, SCREEN_W / 2, 49,
               btConnected_ ? "connected!" : "pairing...");
    centerText(u8g2_, UI_FONT_TINY, SCREEN_W / 2, 60,
               btConnected_ ? "ready to play" : "find \"Mochi\" on your phone");
}

void PlayerUI::renderNowPlaying(bool paused) {
    uint32_t now = millis();
    drawStatusBar(paused);

    // Mochi: bounces with the bass, squishes faster the harder the beat.
    int bounce = paused ? 0 : (int)(beat_ * 3.0f + 0.5f);
    uint32_t swayMs = paused ? 700 : (beat_ > 0.4f ? 180 : 320);
    drawMascot(MASCOT_X, MASCOT_Y - bounce, pickFace(now), ((now / swayMs) & 1) != 0);
    if (paused) {                                           // floating z's while it naps
        uint32_t zt = now % 2400;
        u8g2_->setFont(UI_FONT_TINY);
        if (zt < 1400) u8g2_->drawStr(MASCOT_X + 22, MASCOT_Y + 8 - (int)(zt * 0.006f), "z");
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
        u8g2_->drawBitmap(TEXT_X + 22 + k * 14, ny, 1, 7, kNote);
    }

    centerText(u8g2_, UI_FONT_ARTIST, TEXT_X + ARTIST_AVAIL_W / 2, 38, "Unknown Artist");
    drawVisualizer(42, 21, true, now);
}

void PlayerUI::renderIdle() {
    uint32_t now = millis();

    drawStarfield(now, 0, 46);
    drawWaves(now, 54, 4.6f);

    // sleeping mochi + z's
    float bob = sinf(now * 0.0022f);
    int my = 17 + (bob > 0 ? 1 : 0);
    drawMascot(14, my, 3, ((now / 900) & 1) != 0);

    u8g2_->setFont(UI_FONT_TINY);
    uint32_t zt = now % 2100;
    for (int k = 0; k < 3; ++k) {
        uint32_t zone = (zt + k * 700) % 2100;
        if (zone < 900) {
            int zx = 14 + 25 + (int)(zone * 0.008f);
            int zy = my + 6 - (int)(zone * 0.010f);
            if (zy > 0) u8g2_->drawStr(zx, zy, "z");
        }
    }

    u8g2_->setFont(UI_FONT_LOGO);
    u8g2_->drawStr(46, 36, "Mochi");

    const char* hint = "connect your phone";
    u8g2_->setFont(UI_FONT_TINY);
    int16_t wh = u8g2_->getUTF8Width(hint);
    int hx = (SCREEN_W - wh) / 2;
    u8g2_->drawUTF8(hx, 48, hint);
    u8g2_->drawBitmap(hx - 10, 43, 1, 7, kStar);
    u8g2_->drawBitmap(hx + wh + 3, 43, 1, 7, kStar);
}

// ---- overlays ----------------------------------------------------------------------------
void PlayerUI::setVolumeLevel(uint8_t vol) { lastVol_ = vol; }
void PlayerUI::showVolumeOverlay(uint8_t vol) { lastVol_ = vol; overlayVolMs_ = millis(); }
void PlayerUI::triggerPlayPause(bool playing) { overlayPlaying_ = playing; overlayPlayPauseMs_ = millis(); }

void PlayerUI::drawOverlays() {
    if (screen_ == UIScreen::BOOT) return;
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
            u8g2_->drawRBox(x - 2, y - 2, w + 4, H + 4, 12);
            u8g2_->setDrawColor(1);
            u8g2_->drawRFrame(x, y, w, H, 11);
            u8g2_->drawRFrame(x + 2, y + 2, w - 4, H - 4, 9);

            if (w > 80) {
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
                u8g2_->drawBitmap(x + w - 14, cy - 3, 1, 7, kStar);
            }
        }

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

    // 2. Volume: capsule bar sliding up from the bottom edge. Bars + number follow dispVol_,
    //    so phone-side changes animate exactly like knob turns.
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

        u8g2_->drawBitmap(8, y + 6, 1, 7, kStar);

        const int maxBars = 14;
        const float fb = dispVol_ * maxBars / 100.0f;
        int lastLit = -1;
        for (int i = 0; i < maxBars; ++i) {
            int bx = 20 + i * 6;
            int bh = 5 + (i * 6) / maxBars;                  // ramp: 5..10 px
            int by = y + 14 - bh;
            float f = fb - (float)i;
            if (f >= 0.999f)      { u8g2_->drawRBox(bx, by, 4, bh, 1); lastLit = i; }
            else if (f > 0.0f)    { u8g2_->drawRFrame(bx, by, 4, bh, 1);
                                    int fh = (int)(f * bh + 0.5f);
                                    if (fh > 0) u8g2_->drawBox(bx + 1, by + bh - fh, 2, fh); lastLit = i; }
            else                  u8g2_->drawRFrame(bx, by, 4, bh, 1);
        }
        if (lastLit >= 0 && slide > 0.9f) {
            int sx = 20 + lastLit * 6 + 2;
            int sy = y + 14 - (5 + (lastLit * 6) / maxBars) - 4;
            sparkle(u8g2_, sx, sy, ((now / 140) & 1) ? 1 : 0);
        }

        char num[5];
        int v = (int)(dispVol_ + 0.5f);
        num[0] = v >= 100 ? '1' : ' ';
        num[1] = v >= 10  ? (char)('0' + (v / 10) % 10) : ' ';
        num[2] = (char)('0' + v % 10);
        num[3] = 0;
        u8g2_->setFont(UI_FONT_TINY);
        u8g2_->drawStr(107, y + 14, num);
    }
}