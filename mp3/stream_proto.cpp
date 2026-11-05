#include "stream_proto.h"

namespace proto {

// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no xor-out), nibble-wise.
// Same as Python's binascii.crc_hqx(data, 0xFFFF).
static const uint16_t CRC_NIBBLE[16] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF};

uint16_t crc16(const uint8_t* p, size_t n, uint16_t crc) {
    while (n--) {
        const uint8_t b = *p++;
        crc = (uint16_t)((crc << 4) ^ CRC_NIBBLE[((crc >> 12) ^ (b >> 4)) & 0x0F]);
        crc = (uint16_t)((crc << 4) ^ CRC_NIBBLE[((crc >> 12) ^ (b & 0x0F)) & 0x0F]);
    }
    return crc;
}

size_t encode(uint8_t* out, uint8_t type, uint8_t seq, const uint8_t* payload, uint16_t len) {
    out[0] = MAGIC0;
    out[1] = MAGIC1;
    out[2] = type;
    out[3] = seq;
    out[4] = (uint8_t)(len & 0xFF);
    out[5] = (uint8_t)(len >> 8);
    for (uint16_t i = 0; i < len; ++i) out[HEADER_BYTES + i] = payload[i];
    const uint16_t crc = crc16(out + 2, 4u + len);
    out[HEADER_BYTES + len]     = (uint8_t)(crc & 0xFF);
    out[HEADER_BYTES + len + 1] = (uint8_t)(crc >> 8);
    return HEADER_BYTES + len + 2;
}

void FrameParser::reset() {
    state_  = S_MAGIC0;
    hdrPos_ = 0;
    crcPos_ = 0;
    len_    = 0;
    pos_    = 0;
    noise_  = 0;
    frame_.type = 0;
    frame_.seq = 0;
    frame_.len = 0;
    frame_.payload = payload_;
}

FrameParser::Result FrameParser::push(uint8_t b) {
    switch (state_) {
        case S_MAGIC0:
            if (b == MAGIC0) { state_ = S_MAGIC1; return R_NONE; }
            noise_ = b;
            return R_NOISE;

        case S_MAGIC1:
            if (b == MAGIC1) { state_ = S_HEADER; hdrPos_ = 0; return R_NONE; }
            if (b == MAGIC0) return R_NONE;           // AA AA 55 ...: stay armed
            state_ = S_MAGIC0;
            noise_ = b;
            return R_NOISE;

        case S_HEADER:
            hdr_[hdrPos_++] = b;
            if (hdrPos_ < 4) return R_NONE;
            len_ = (uint16_t)(hdr_[2] | ((uint16_t)hdr_[3] << 8));
            if (len_ > MAX_PAYLOAD) { state_ = S_MAGIC0; return R_BAD_LEN; }
            pos_    = 0;
            crcPos_ = 0;
            state_  = (len_ > 0) ? S_PAYLOAD : S_CRC;
            return R_NONE;

        case S_PAYLOAD:
            payload_[pos_++] = b;
            if (pos_ == len_) { state_ = S_CRC; crcPos_ = 0; }
            return R_NONE;

        case S_CRC: {
            crcBytes_[crcPos_++] = b;
            if (crcPos_ < 2) return R_NONE;
            state_ = S_MAGIC0;
            uint16_t crc = crc16(hdr_, 4);
            crc = crc16(payload_, len_, crc);
            const uint16_t rx = (uint16_t)(crcBytes_[0] | ((uint16_t)crcBytes_[1] << 8));
            if (crc != rx) return R_BAD_CRC;
            frame_.type    = hdr_[0];
            frame_.seq     = hdr_[1];
            frame_.len     = len_;
            frame_.payload = payload_;
            return R_FRAME;
        }
    }
    state_ = S_MAGIC0;
    return R_NONE;
}

}  // namespace proto
