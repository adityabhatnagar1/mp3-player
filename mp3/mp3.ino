// Milestone 01 - Static retro splash screen
// Board   : ESP32 DEVKIT V1 (ESP-WROOM-32), Arduino framework
// Display : M130-12864-B-4V, 1.3" 128x64 monochrome OLED, 4-pin I2C
// Library : U8g2 (olikraus)

#include <Arduino.h>
#include <U8g2lib.h>

// ---- Configuration ---------------------------------------------------------
// Controller select. 1 = SH1106 (typical for 1.3" 128x64 modules), 0 = SSD1306.
// If the image looks shifted by ~2 px or the right edge shows noise, flip this.
#define OLED_USE_SH1106 1

#define PIN_SDA   21     // ESP32 default I2C SDA
#define PIN_SCL   22     // ESP32 default I2C SCL
#define OLED_ADDR 0x3C   // 7-bit I2C address (0x3D on some modules)

#if OLED_USE_SH1106
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, PIN_SCL, PIN_SDA);
#else
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, PIN_SCL, PIN_SDA);
#endif

// ---- Splash ----------------------------------------------------------------
static void drawCentered(uint8_t baselineY, const char *text) {
  u8g2.drawStr((128 - u8g2.getStrWidth(text)) / 2, baselineY, text);
}

static void drawSplash() {
  u8g2.clearBuffer();

  // Double border with notched corners
  u8g2.drawFrame(0, 0, 128, 64);
  u8g2.drawFrame(3, 3, 122, 58);
  u8g2.setDrawColor(0);
  u8g2.drawBox(0, 0, 3, 3);
  u8g2.drawBox(125, 0, 3, 3);
  u8g2.drawBox(0, 61, 3, 3);
  u8g2.drawBox(125, 61, 3, 3);
  u8g2.setDrawColor(1);

  // Inverted title band with scanline stripes underneath
  u8g2.drawBox(7, 8, 114, 20);
  u8g2.setFont(u8g2_font_10x20_tr);
  u8g2.setDrawColor(0);
  drawCentered(25, "ESP32");
  u8g2.setDrawColor(1);
  for (uint8_t y = 31; y <= 35; y += 2) {
    u8g2.drawHLine(7, y, 114);
  }

  // Subtitle and status line
  u8g2.setFont(u8g2_font_6x10_tr);
  drawCentered(46, "MILESTONE 01");

  const char *status = "> SYSTEM READY";
  const uint8_t cursorW = 5;
  const uint8_t statusW = u8g2.getStrWidth(status);
  const uint8_t statusX = (128 - (statusW + 2 + cursorW)) / 2;
  u8g2.drawStr(statusX, 56, status);
  u8g2.drawBox(statusX + statusW + 2, 50, cursorW, 7);  // static block cursor

  u8g2.sendBuffer();
}

void setup() {
  u8g2.setI2CAddress(OLED_ADDR << 1);  // U8g2 expects the 8-bit address form
  u8g2.begin();
  drawSplash();
}

void loop() {
  // Static screen: nothing to do.
}