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