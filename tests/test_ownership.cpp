#include "source_manager.h"
#include <cassert>
#include <cstdio>

using namespace audio;

int main() {
    SourceManager m;
    assert(m.owner() == Source::NONE);
    assert(!m.ready());

    // Successful acquisition.
    assert(m.acquire(Source::USB_SERIAL));
    assert(m.owner() == Source::USB_SERIAL);
    assert(!m.ready());

    // Second acquisition must fail.
    assert(!m.acquire(Source::BLUETOOTH));
    assert(m.owner() == Source::USB_SERIAL);

    // Ready after audio task start.
    m.setReady(true);
    assert(m.ready());

    // Release only from audio task path.
    m.release();
    assert(m.owner() == Source::NONE);
    assert(!m.ready());

    // BT acquisition after release.
    assert(m.acquire(Source::BLUETOOTH));
    assert(m.owner() == Source::BLUETOOTH);

    // USB serial cannot steal ownership.
    assert(!m.acquire(Source::USB_SERIAL));
    assert(m.owner() == Source::BLUETOOTH);

    // Failed startup: release immediately.
    m.release();
    assert(m.acquire(Source::USB_SERIAL));

    printf("test_ownership: PASS\n");
    return 0;
}