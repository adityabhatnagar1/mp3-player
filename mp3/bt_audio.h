#pragma once
#include <stdint.h>

namespace bt {

bool begin();
void end();

struct Snapshot {
    bool connected;
    bool metadataChanged;
    char title[96];
    char artist[64];
};

Snapshot poll();

}  // namespace bt