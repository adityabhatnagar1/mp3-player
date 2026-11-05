#include "stream_player.h"

#include <string.h>

namespace sp {

static_assert((RING_BYTES & (RING_BYTES - 1)) == 0, "RING_BYTES must be a power of two");

namespace {
constexpr uint32_t RING_MASK = RING_BYTES - 1;
}

StreamPlayer::StreamPlayer()
    : head_(0), tail_(0), st_(IDLE), endReq_(false), fadeIn_(false), channels_(1),
      frameBytes_(2), prebuffer_(4096), underruns_(0),
      pubState_(IDLE), pubSession_(0), pubUnderruns_(0) {}

// ---------------------------------------------------------------- producer --

bool StreamPlayer::write(const uint8_t* data, uint32_t len) {
    const uint32_t h = head_.load(std::memory_order_relaxed);
    const uint32_t t = tail_.load(std::memory_order_acquire);
    if (len > RING_BYTES - (h - t)) return false;            // not enough room: drop whole block
    const uint32_t idx   = h & RING_MASK;
    const uint32_t first = (len < RING_BYTES - idx) ? len : (RING_BYTES - idx);
    memcpy(ring_ + idx, data, first);
    if (len > first) memcpy(ring_, data + first, len - first);
    head_.store(h + len, std::memory_order_release);
    return true;
}

Status StreamPlayer::status() const {
    Status s;
    s.state     = (uint8_t)pubState_.load(std::memory_order_acquire);
    s.session   = (uint16_t)pubSession_.load(std::memory_order_acquire);
    s.underruns = (uint16_t)pubUnderruns_.load(std::memory_order_acquire);
    const uint32_t t = tail_.load(std::memory_order_acquire);
    const uint32_t h = head_.load(std::memory_order_acquire);
    s.fill     = h - t;
    s.capacity = RING_BYTES;
    return s;
}

// ---------------------------------------------------------------- consumer --

void StreamPlayer::publish() {
    uint32_t s = (uint32_t)st_;
    if (st_ != IDLE && endReq_) s = DRAINING;
    pubState_.store(s, std::memory_order_release);
}

void StreamPlayer::finish() {
    st_     = IDLE;
    endReq_ = false;
    fadeIn_ = false;
    publish();
}

void StreamPlayer::start(uint32_t rateHz, uint8_t channels, uint16_t session) {
    channels_   = (channels == 2) ? 2 : 1;
    frameBytes_ = (uint8_t)(2 * channels_);

    // ~0.25 s of audio before playback begins, clamped to [2 KB, half the ring].
    uint32_t pre = rateHz * frameBytes_ / 4;
    if (pre < 2048) pre = 2048;
    if (pre > RING_BYTES / 2) pre = RING_BYTES / 2;
    prebuffer_ = pre - (pre % frameBytes_);

    tail_.store(head_.load(std::memory_order_acquire), std::memory_order_release);  // flush
    endReq_    = false;
    fadeIn_    = false;
    underruns_ = 0;
    st_        = BUFFERING;
    pubUnderruns_.store(0, std::memory_order_release);
    pubSession_.store(session, std::memory_order_release);
    publish();
}

void StreamPlayer::end() {
    if (st_ == IDLE) return;
    endReq_ = true;
    publish();
}

void StreamPlayer::abort() {
    tail_.store(head_.load(std::memory_order_acquire), std::memory_order_release);
    finish();
}

uint32_t StreamPlayer::ringRead(uint8_t* dst, uint32_t n) {
    const uint32_t t     = tail_.load(std::memory_order_relaxed);
    const uint32_t idx   = t & RING_MASK;
    const uint32_t first = (n < RING_BYTES - idx) ? n : (RING_BYTES - idx);
    memcpy(dst, ring_ + idx, first);
    if (n > first) memcpy(dst + first, ring_, n - first);
    tail_.store(t + n, std::memory_order_release);
    return n;
}

int StreamPlayer::render(int16_t* out, int frames) {
    if (frames > MAX_CHUNK_FRAMES) frames = MAX_CHUNK_FRAMES;
    memset(out, 0, (size_t)frames * 4);
    if (st_ == IDLE) return 0;

    const uint32_t fb = frameBytes_;
    uint32_t fill = head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_relaxed);

    if (st_ == BUFFERING) {
        if (fill >= prebuffer_ || (endReq_ && fill > 0)) {
            st_     = PLAYING;
            fadeIn_ = true;
            publish();
        } else {
            if (endReq_) finish();      // host ended with nothing left to play
            return 0;
        }
    }

    // ---- PLAYING: pull up to one chunk of whole frames from the ring ----
    const uint32_t want = (uint32_t)frames * fb;
    uint32_t n = (fill < want) ? fill : want;
    n -= n % fb;                          // never split a frame
    const int got = (int)(n / fb);

    if (got > 0) {
        ringRead((uint8_t*)tmp_, n);
        if (channels_ == 1) {
            for (int i = 0; i < got; ++i) {
                out[2 * i]     = tmp_[i];
                out[2 * i + 1] = tmp_[i];
            }
        } else {
            memcpy(out, tmp_, n);
        }
    }

    if (fadeIn_ && got > 0) {
        const int r = (got < FADE_IN_FRAMES) ? got : FADE_IN_FRAMES;
        for (int i = 0; i < r; ++i) {
            out[2 * i]     = (int16_t)(((int32_t)out[2 * i]     * i) / r);
            out[2 * i + 1] = (int16_t)(((int32_t)out[2 * i + 1] * i) / r);
        }
        fadeIn_ = false;
    }

    if (got < frames) {
        // Ring ran dry mid-chunk: taper what we have so the cut is not a click.
        if (got > 0) {
            const int r = (got < FADE_OUT_FRAMES) ? got : FADE_OUT_FRAMES;
            for (int j = 0; j < r; ++j) {
                const int idx = got - r + j;
                const int32_t g = r - 1 - j;
                out[2 * idx]     = (int16_t)(((int32_t)out[2 * idx]     * g) / r);
                out[2 * idx + 1] = (int16_t)(((int32_t)out[2 * idx + 1] * g) / r);
            }
        }
        if (endReq_) {
            finish();                                   // normal end of track
        } else {
            ++underruns_;                               // genuine underrun: rebuffer
            pubUnderruns_.store(underruns_, std::memory_order_release);
            st_ = BUFFERING;
            publish();
        }
    }
    return got;
}

}  // namespace sp
