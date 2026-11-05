#pragma once
#include <stdint.h>

namespace bt {

void setVolume(uint8_t vol);

bool begin();
void end();

struct Snapshot {
    bool    connected;
    bool    metadataChanged;
    int8_t  phoneVolume;     // 0..100 if the phone just changed its volume slider, else -1
    char    title[96];
    char    artist[64];
};

Snapshot poll();             // consumes metadata + phone-volume events: call ONCE per loop

bool connected();            // side-effect-free, safe to call anywhere
bool isPaused();
void pause();
void resume();

}  // namespace bt