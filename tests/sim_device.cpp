// Runs the REAL firmware sources (stream_rx, stream_proto, stream_player, audio_levels,
// audio_out) on a PC. UART = a pty (play_mp3.py connects to the printed slave path),
// I2S = a fake that records the PCM and paces itself like real DMA.
#define _GNU_SOURCE
#include "Arduino.h"
#include "driver/i2s.h"
#include "audio_out.h"
#include "stream_rx.h"

#include <atomic>
#include <chrono>
#include <fcntl.h>
#include <mutex>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

using Clock = std::chrono::steady_clock;
static const Clock::time_point T0 = Clock::now();
uint32_t millis() { return (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - T0).count(); }

static int g_master = -1;
SimSerial Serial;
int SimSerial::available() { int n = 0; ioctl(g_master, FIONREAD, &n); return n; }
size_t SimSerial::read(uint8_t* b, size_t n) { ssize_t r = ::read(g_master, b, n); return r > 0 ? (size_t)r : 0; }
size_t SimSerial::write(const uint8_t* b, size_t n) { ssize_t r = ::write(g_master, b, n); return r > 0 ? (size_t)r : 0; } // drops when full, like a bridge chip

// ---- fake I2S ----
static FILE*  g_out = nullptr;
static double g_speed = 1.0;
static uint32_t g_rate = 44100;
static Clock::time_point g_playPos;            // virtual time at which everything queued so far finishes playing
static std::mutex g_m;
static uint64_t g_framesWritten = 0;
static const double DMA_SECONDS_FRAMES = 8 * 256;

static void resetPos() { g_playPos = Clock::now(); }
esp_err_t i2s_driver_install(i2s_port_t, const i2s_config_t* c, int, void*) { g_rate = c->sample_rate; resetPos(); return ESP_OK; }
esp_err_t i2s_driver_uninstall(i2s_port_t) { return ESP_OK; }
esp_err_t i2s_set_pin(i2s_port_t, const i2s_pin_config_t*) { return ESP_OK; }
esp_err_t i2s_zero_dma_buffer(i2s_port_t) { std::lock_guard<std::mutex> g(g_m); resetPos(); return ESP_OK; }
esp_err_t i2s_set_clk(i2s_port_t, uint32_t rate, i2s_bits_per_sample_t, i2s_channel_t) {
    std::lock_guard<std::mutex> g(g_m);
    g_rate = rate; resetPos();
    fprintf(stderr, "[sim-i2s] clock -> %u Hz at t=%u ms (frames written so far: %llu)\n", rate, millis(), (unsigned long long)g_framesWritten);
    return ESP_OK;
}
esp_err_t i2s_write(i2s_port_t, const void* src, size_t size, size_t* written, TickType_t) {
    const size_t frames = size / 4;
    fwrite(src, 1, size, g_out); fflush(g_out);
    g_framesWritten += frames;
    Clock::time_point wake;
    {
        std::lock_guard<std::mutex> g(g_m);
        auto now = Clock::now();
        if (g_playPos < now) g_playPos = now;                       // DMA ran dry: restart from now
        g_playPos += std::chrono::nanoseconds((long long)(1e9 * frames / g_rate / g_speed));
        // real driver only blocks once all DMA buffers are full
        wake = g_playPos - std::chrono::nanoseconds((long long)(1e9 * DMA_SECONDS_FRAMES / g_rate / g_speed));
    }
    std::this_thread::sleep_until(wake);
    *written = size;
    return ESP_OK;
}

int main() {
    if (const char* s = getenv("SIM_SPEED")) g_speed = atof(s);
    const char* outPath = getenv("SIM_OUT"); if (!outPath) outPath = "/tmp/i2s_out.raw";
    g_out = fopen(outPath, "wb");

    g_master = posix_openpt(O_RDWR | O_NOCTTY);
    grantpt(g_master); unlockpt(g_master);
    const char* sl = ptsname(g_master);
    int slave = open(sl, O_RDWR | O_NOCTTY);          // keep one slave open so the master never sees hangup
    termios t; tcgetattr(slave, &t); cfmakeraw(&t); tcsetattr(slave, TCSANOW, &t);
    fcntl(g_master, F_SETFL, fcntl(g_master, F_GETFL) | O_NONBLOCK);
    printf("%s\n", sl); fflush(stdout);

    stream::begin();
    if (!audio::begin()) { fprintf(stderr, "audio::begin failed\n"); return 1; }
    fprintf(stderr, "[sim] ready on %s speed=%.2f out=%s\n", sl, g_speed, outPath);
    for (;;) sleep(1);
}
