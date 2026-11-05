#pragma once
// Explicit audio-source ownership for the single-producer / single-consumer
// StreamPlayer.  Pure C++11, no Arduino/FreeRTOS dependencies -> testable on PC.

#include <atomic>
#include <stdint.h>

namespace audio {

enum class Source : uint32_t { NONE = 0, USB_SERIAL = 1, BLUETOOTH = 2 };

class SourceManager {
 public:
  bool acquire(Source src) {
    Source expected = Source::NONE;
    return owner_.compare_exchange_strong(expected, src,
                                          std::memory_order_acq_rel,
                                          std::memory_order_acquire);
  }
  void setReady(bool r) { ready_.store(r, std::memory_order_release); }
  void release() {
    ready_.store(false, std::memory_order_release);
    owner_.store(Source::NONE, std::memory_order_release);
  }
  Source owner() const { return owner_.load(std::memory_order_acquire); }
  bool   ready() const { return ready_.load(std::memory_order_acquire); }

 private:
  std::atomic<Source> owner_{Source::NONE};
  std::atomic<bool>   ready_{false};
};

}  // namespace audio