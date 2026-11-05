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
    void setVolumeLevel(uint8_t vol);
    void showVolumeOverlay(uint8_t vol);
    void triggerPlayPause(bool playing);
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
    bool bootFinished() const { return screen_ != UIScreen::BOOT; }
    void showStepOverlay(bool show);
    void setStepCount(uint32_t steps);

private:
    uint32_t overlayVolMs_ = 0;
    uint32_t overlayPlayPauseMs_ = 0;
    uint8_t  lastVol_ = 70;
    float    dispVol_ = 70.0f;          // eased copy of lastVol_, drives every volume graphic
    bool     overlayPlaying_ = false;

    void drawOverlays();
    U8G2* u8g2_ = nullptr;

    UIScreen screen_ = UIScreen::BOOT;
    bool autoMode_   = true;
    bool searching_  = false;
    bool firstFrame_ = true;            // boot clock starts at the first *rendered* frame

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
    float   beat_          = 0.0f;   // smoothed bass energy 0..1, drives mascot bounce

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
    void drawMascot(int x, int y, uint8_t face, bool sway);
    void drawBluetoothIcon(int x, int y, uint32_t now);
    void drawVolumeMeter(int x, int y);
    void drawStarfield(uint32_t now, int yMin, int yMax);
    void drawWaves(uint32_t now, int baseY, float amp);
    void drawVisualizer(int baseY, int maxH, bool active, uint32_t now);
    void drawMarquee(int x, int baseline, int availW,
                     const char* text, int16_t w, float scroll,
                     const uint8_t* font);
    uint8_t pickFace(uint32_t now) const;
    void drawEntryWipe();

    bool     showStepOverlay_ = false;
    uint32_t stepCount_ = 0;    
    void drawStepOverlay();

    static const uint8_t mascot_[8][72];   // 24x24, 3 bytes/row
};

#endif