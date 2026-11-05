#pragma once
// PCM ring buffer + playback state machine for streamed audio.
// Pure C++11 (no Arduino / FreeRTOS / I2S), so the logic can be tested on a PC.
//
// Threading model (strict):
//   * Producer side  (stream receiver task): write(), status()
//   * Consumer side  (audio task ONLY):      start(), end(), abort(), render(), idle()
// The ring is a lock-free single-producer / single-consumer queue built on
// std::atomic<uint32_t> head/tail counters (free-running, RING_BYTES is a power of two).
// Everything shared between tasks is a 32-bit atomic (the only lock-free width on ESP32).

#include <atomic>
#include <stdint.h>

namespace sp {

constexpr uint32_t RING_BYTES       = 32768;   // power of two
constexpr int      MAX_CHUNK_FRAMES = 256;     // render() never produces more than this
constexpr int      FADE_IN_FRAMES   = 128;     // ~4 ms @ 32 kHz, removes start click
constexpr int      FADE_OUT_FRAMES  = 64;      // ~2 ms, removes click at underrun / end

enum State : uint8_t { IDLE = 0, BUFFERING = 1, PLAYING = 2, DRAINING = 3 };

struct Status {
  uint8_t  state;
  uint16_t session;
  uint16_t underruns;
  uint32_t fill;       // bytes currently queued in the ring
  uint32_t capacity;   // RING_BYTES
};

class StreamPlayer {
 public:
  StreamPlayer();

  // ---- producer side ------------------------------------------------------
  // All-or-nothing: either the whole block is queued or nothing is (returns false).
  bool write(const uint8_t* data, uint32_t len);
  Status status() const;

  // ---- consumer side (audio task) -----------------------------------------
  void start(uint32_t rateHz, uint8_t channels, uint16_t session);  // flush + BUFFERING
  void end();      // host finished: play out what is queued, then go IDLE
  void abort();    // stop now, drop everything
  bool idle() const { return st_ == IDLE; }

  // Writes exactly `frames` stereo frames (L,R interleaved s16) into out.
  // Returns how many leading frames contain real audio (the rest are silence).
  int render(int16_t* out, int frames);

 private:
  void     publish();
  void     finish();
  uint32_t ringRead(uint8_t* dst, uint32_t n);

  uint8_t               ring_[RING_BYTES];
  std::atomic<uint32_t> head_;        // written by producer
  std::atomic<uint32_t> tail_;        // written by consumer

  // consumer-owned
  State    st_;
  bool     endReq_;
  bool     fadeIn_;
  uint8_t  channels_;
  uint8_t  frameBytes_;
  uint32_t prebuffer_;
  uint32_t underruns_;
  int16_t  tmp_[MAX_CHUNK_FRAMES * 2];

  // published for other tasks
  std::atomic<uint32_t> pubState_;
  std::atomic<uint32_t> pubSession_;
  std::atomic<uint32_t> pubUnderruns_;
};

}  // namespace sp
