#include "stream_proto.h"
#include <stdio.h>
#include <unistd.h>
int main() {
    proto::FrameParser p;
    uint8_t b;
    unsigned noise = 0, badcrc = 0, badlen = 0;
    while (read(0, &b, 1) == 1) {
        switch (p.push(b)) {
            case proto::FrameParser::R_FRAME: {
                const auto& f = p.frame();
                printf("F %u %u %u %04x\n", f.type, f.seq, f.len, proto::crc16(f.payload, f.len));
                break; }
            case proto::FrameParser::R_NOISE:   ++noise; break;
            case proto::FrameParser::R_BAD_CRC: ++badcrc; break;
            case proto::FrameParser::R_BAD_LEN: ++badlen; break;
            default: break;
        }
    }
    printf("N %u %u %u\n", noise, badcrc, badlen);
}
