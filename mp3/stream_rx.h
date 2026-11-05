#pragma once
#include <Arduino.h>

// UART receiver for the laptop -> ESP32 PCM stream (protocol: stream_proto.h).
// Owns Serial (UART0) after begin(): nothing else may read from it.

namespace stream {

static constexpr uint32_t BAUD = 921600;     // must match play_mp3.py --baud

// Optional: called (from the receiver task) for stray ASCII bytes that arrive while
// no stream session is active. Used to keep the old Serial-Monitor commands working.
typedef void (*ConsoleHandler)(char c);
void setConsoleHandler(ConsoleHandler h);

// Configures Serial and starts the receiver task (core 0). Call this FIRST in setup().
bool begin();

}  // namespace stream
