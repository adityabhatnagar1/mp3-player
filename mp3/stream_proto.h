#pragma once
// Wire protocol between play_mp3.py (laptop) and the ESP32.
// Pure C++11, no Arduino/FreeRTOS dependencies (so it can be unit-tested on a PC).
//
// Frame (both directions, little-endian):
//   [0]    0xAA                  magic
//   [1]    0x55                  magic
//   [2]    type      u8
//   [3]    seq       u8          per-direction counter, wraps at 256
//   [4..5] len       u16         payload length, 0..1024
//   [6..]  payload   len bytes
//   [..]   crc16     u16         CRC-16/CCITT-FALSE over bytes [2 .. 6+len)
//
// Host -> ESP32 types:
//   PING   (0x01) no payload                  -> ESP32 replies STATUS
//   HELLO  (0x02) u32 rate, u8 channels, u8 bits(=16), u16 session
//   DATA   (0x03) interleaved s16le PCM, len multiple of channels*2
//   END    (0x04) no payload: host has no more data, play out the buffer then stop
//   ABORT  (0x05) no payload: stop now and flush
// ESP32 -> Host:
//   STATUS (0x81) u8 version, u8 state, u8 error, u8 pad, u16 session,
//                 u16 underruns, u16 errors, u32 fillBytes, u32 capacityBytes  (18 bytes)

#include <stddef.h>
#include <stdint.h>

namespace proto {

constexpr uint8_t  MAGIC0          = 0xAA;
constexpr uint8_t  MAGIC1          = 0x55;
constexpr uint16_t MAX_PAYLOAD     = 1024;
constexpr size_t   HEADER_BYTES    = 6;
constexpr size_t   MAX_FRAME_BYTES = HEADER_BYTES + MAX_PAYLOAD + 2;

constexpr uint8_t T_PING   = 0x01;
constexpr uint8_t T_HELLO  = 0x02;
constexpr uint8_t T_DATA   = 0x03;
constexpr uint8_t T_END    = 0x04;
constexpr uint8_t T_ABORT  = 0x05;
constexpr uint8_t T_STATUS = 0x81;

constexpr uint8_t  VERSION        = 1;
constexpr uint16_t HELLO_BYTES    = 8;
constexpr uint16_t STATUS_BYTES   = 18;

constexpr uint8_t ERR_NONE       = 0;
constexpr uint8_t ERR_BAD_PARAMS = 1;   // HELLO rejected
constexpr uint8_t ERR_BAD_DATA   = 2;   // DATA length not a whole number of frames
constexpr uint8_t ERR_OVERFLOW   = 3;   // DATA dropped, ring full (host ignored flow control)
constexpr uint8_t ERR_TIMEOUT    = 4;   // session aborted: host went silent

uint16_t crc16(const uint8_t* p, size_t n, uint16_t crc = 0xFFFF);

// Writes one complete frame into out (needs HEADER_BYTES + len + 2). Returns bytes written.
size_t encode(uint8_t* out, uint8_t type, uint8_t seq, const uint8_t* payload, uint16_t len);

// Byte-at-a-time frame parser. Handles partial reads naturally (feed whatever arrived),
// resynchronises on the next magic after garbage or a failed CRC.
class FrameParser {
 public:
  enum Result : uint8_t {
    R_NONE = 0,   // byte consumed as part of a frame in progress
    R_NOISE,      // byte was outside any frame (see noiseByte())
    R_FRAME,      // a valid frame completed (see frame())
    R_BAD_CRC,    // a frame completed but the CRC failed (dropped)
    R_BAD_LEN     // header announced len > MAX_PAYLOAD (dropped)
  };
  struct Frame {
    uint8_t        type;
    uint8_t        seq;
    uint16_t       len;
    const uint8_t* payload;   // valid until the next push()
  };

  FrameParser() { reset(); }
  void reset();
  Result push(uint8_t b);
  const Frame& frame() const { return frame_; }
  uint8_t noiseByte() const { return noise_; }

 private:
  enum State : uint8_t { S_MAGIC0, S_MAGIC1, S_HEADER, S_PAYLOAD, S_CRC };
  State    state_;
  uint8_t  hdr_[4];
  uint8_t  hdrPos_;
  uint8_t  crcPos_;
  uint8_t  crcBytes_[2];
  uint16_t len_;
  uint16_t pos_;
  uint8_t  noise_;
  Frame    frame_;
  uint8_t  payload_[MAX_PAYLOAD];
};

}  // namespace proto
