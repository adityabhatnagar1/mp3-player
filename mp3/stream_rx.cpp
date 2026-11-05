#include "stream_rx.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "audio_out.h"
#include "stream_proto.h"

namespace stream {
namespace {

constexpr size_t   RX_DRIVER_BUFFER   = 8192;   // UART driver ring (~90 ms at 921600 baud)
constexpr size_t   READ_BURST         = 256;
constexpr uint32_t STATUS_PERIOD_MS   = 20;     // STATUS rate while a session is active
constexpr uint32_t SESSION_TIMEOUT_MS = 2000;   // host silent this long -> abort session
constexpr uint32_t HELLO_GRACE_MS     = 500;    // keep reporting after HELLO until audio task reacts

proto::FrameParser g_parser;
ConsoleHandler     g_console = nullptr;

uint8_t  g_txSeq        = 0;
uint8_t  g_expectSeq    = 0;
bool     g_haveSeq      = false;
uint32_t g_errors       = 0;       // lost/bad frames this session (CRC, length, seq gap, bad DATA, overflow)
uint8_t  g_lastError    = proto::ERR_NONE;
uint8_t  g_frameBytes   = 2;
uint32_t g_lastGoodMs   = 0;
uint32_t g_lastStatusMs = 0;
uint32_t g_helloMs      = 0;
bool     g_awaitStart   = false;
uint8_t  g_lastState    = audio::STREAM_IDLE;

inline void bumpError(uint8_t code) {
    ++g_errors;
    g_lastError = code;
}

inline uint32_t le32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
inline uint16_t le16(const uint8_t* p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
inline void put16(uint8_t* p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
inline void put32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

void sendStatus() {
    const audio::StreamStatus s = audio::streamStatus();
    uint8_t payload[proto::STATUS_BYTES];
    payload[0] = proto::VERSION;
    payload[1] = s.state;
    payload[2] = g_lastError;
    payload[3] = 0;
    put16(payload + 4, s.session);
    put16(payload + 6, s.underruns);
    put16(payload + 8, (uint16_t)(g_errors > 0xFFFF ? 0xFFFF : g_errors));
    put32(payload + 10, s.fillBytes);
    put32(payload + 14, s.capacityBytes);

    uint8_t frame[proto::HEADER_BYTES + proto::STATUS_BYTES + 2];
    const size_t n = proto::encode(frame, proto::T_STATUS, g_txSeq++, payload, proto::STATUS_BYTES);
    Serial.write(frame, n);
    g_lastStatusMs = millis();
}

void onHello(const proto::FrameParser::Frame& f) {
    if (f.len != proto::HELLO_BYTES) { bumpError(proto::ERR_BAD_PARAMS); sendStatus(); return; }

    const uint32_t rate    = le32(f.payload);
    const uint8_t  ch      = f.payload[4];
    const uint8_t  bits    = f.payload[5];
    const uint16_t session = le16(f.payload + 6);

    if (bits != 16 || ch < 1 || ch > 2 || rate < 8000 || rate > 48000 || session == 0) {
        g_lastError = proto::ERR_BAD_PARAMS;
        sendStatus();
        return;
    }

    // The host retries HELLO until it sees our STATUS: a repeat of a session that is
    // already open must not flush the ring.
    const audio::StreamStatus s = audio::streamStatus();
    if (s.state != audio::STREAM_IDLE && s.session == session) { sendStatus(); return; }

    g_errors     = 0;
    g_lastError  = proto::ERR_NONE;
    g_frameBytes = (uint8_t)(2 * ch);
    if (!audio::streamBegin(rate, ch, session)) {
        g_lastError = proto::ERR_BAD_PARAMS;
        sendStatus();
        return;
    }
    g_helloMs    = millis();
    g_awaitStart = true;
    sendStatus();
}

void onData(const proto::FrameParser::Frame& f) {
    if (audio::streamStatus().state == audio::STREAM_IDLE) return;     // no session: ignore stray data
    if (f.len == 0 || (f.len % g_frameBytes) != 0) { bumpError(proto::ERR_BAD_DATA); return; }
    if (!audio::streamWrite(f.payload, f.len))      { bumpError(proto::ERR_OVERFLOW); }
}

void onFrame(const proto::FrameParser::Frame& f) {
    g_lastGoodMs = millis();

    // Every frame the host sends bumps seq, so a gap means a frame went missing.
    if (g_haveSeq && f.seq != g_expectSeq) ++g_errors;
    g_haveSeq   = true;
    g_expectSeq = (uint8_t)(f.seq + 1);

    switch (f.type) {
        case proto::T_PING:  sendStatus();                         break;
        case proto::T_HELLO: onHello(f);                           break;
        case proto::T_DATA:  onData(f);                            break;
        case proto::T_END:   audio::streamEnd();   sendStatus();   break;
        case proto::T_ABORT: audio::streamAbort(); sendStatus();   break;
        default: break;
    }
}

void periodic() {
    const uint32_t now = millis();
    const audio::StreamStatus s = audio::streamStatus();

    if (g_awaitStart && (s.state != audio::STREAM_IDLE || (uint32_t)(now - g_helloMs) > HELLO_GRACE_MS))
        g_awaitStart = false;

    if (s.state != g_lastState) {                  // state transition: tell the host right away
        g_lastState = s.state;
        sendStatus();
    } else if ((s.state != audio::STREAM_IDLE || g_awaitStart) &&
               (uint32_t)(now - g_lastStatusMs) >= STATUS_PERIOD_MS) {
        sendStatus();
    }

    // Host vanished (script killed, cable pulled): stop instead of looping on silence.
    if (s.state != audio::STREAM_IDLE && (uint32_t)(now - g_lastGoodMs) > SESSION_TIMEOUT_MS) {
        g_lastError = proto::ERR_TIMEOUT;
        audio::streamAbort();
        g_lastGoodMs = now;
    }
}

void rxTask(void*) {
    uint8_t buf[READ_BURST];
    for (;;) {
        const int avail = Serial.available();
        if (avail > 0) {
            const size_t want = (size_t)avail < READ_BURST ? (size_t)avail : READ_BURST;
            const size_t got  = Serial.read(buf, want);
            for (size_t i = 0; i < got; ++i) {
                switch (g_parser.push(buf[i])) {
                    case proto::FrameParser::R_FRAME:   onFrame(g_parser.frame()); break;
                    case proto::FrameParser::R_BAD_CRC: bumpError(proto::ERR_BAD_DATA); break;
                    case proto::FrameParser::R_BAD_LEN: bumpError(proto::ERR_BAD_DATA); break;
                    case proto::FrameParser::R_NOISE:
                        if (g_console && audio::streamStatus().state == audio::STREAM_IDLE)
                            g_console((char)g_parser.noiseByte());
                        break;
                    default: break;
                }
            }
        } else {
            vTaskDelay(1);                          // 1 ms; UART driver ring absorbs ~90 ms
        }
        periodic();
    }
}

}  // namespace

void setConsoleHandler(ConsoleHandler h) { g_console = h; }

bool begin() {
    Serial.setRxBufferSize(RX_DRIVER_BUFFER);       // must precede begin()
    Serial.begin(BAUD);
    g_lastGoodMs = millis();
    return xTaskCreatePinnedToCore(rxTask, "stream_rx", 4096, nullptr, 4, nullptr, 0) == pdPASS;
}

}  // namespace stream
