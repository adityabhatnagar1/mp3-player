#include "bt_audio.h"
#include "audio_out.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "BluetoothA2DPSink.h"

namespace bt {
namespace {

BluetoothA2DPSink* g_sink       = nullptr;
SemaphoreHandle_t  g_metaMutex = nullptr;

volatile bool g_connected = false;
volatile bool g_metaDirty = false;

char g_title[96]  = {0};
char g_artist[64] = {0};

// Called on the Bluetooth task for each decoded SBC frame.
void dataCallback(const uint8_t* data, uint32_t length) {
    // Defensive: the library should already deliver frame-aligned data.
    if (length < 4) return;
    length -= length % 4;                    // stereo 16-bit = 4 bytes/frame
    audio::btWrite(data, length);
}

// Called on the BT task for A2DP connection-state changes.
void connectionCallback(esp_a2d_connection_state_t state, void* /*ptr*/) {
    if (state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
        g_connected = true;
        audio::btBegin();                    // rate supplied by sample-rate callback
    } else if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
        g_connected = false;
        audio::btEnd();
        if (g_metaMutex && xSemaphoreTake(g_metaMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            g_title[0]  = 0;
            g_artist[0] = 0;
            g_metaDirty = true;
            xSemaphoreGive(g_metaMutex);
        }
    }
}

// Called on the BT task when AVRCP metadata arrives.
void metadataCallback(uint8_t id, const uint8_t* text) {
    if (!g_metaMutex || !text) return;
    if (xSemaphoreTake(g_metaMutex, pdMS_TO_TICKS(10)) != pdTRUE) return;
    if (id == ESP_AVRC_MD_ATTR_TITLE) {          // 0x01
        strncpy(g_title, (const char*)text, sizeof(g_title) - 1);
        g_title[sizeof(g_title) - 1] = 0;
        g_metaDirty = true;
    } else if (id == ESP_AVRC_MD_ATTR_ARTIST) {   // 0x02
        strncpy(g_artist, (const char*)text, sizeof(g_artist) - 1);
        g_artist[sizeof(g_artist) - 1] = 0;
        g_metaDirty = true;
    }
    xSemaphoreGive(g_metaMutex);
}

// Called on the BT task when the negotiated sample rate changes.
void sampleRateCallback(uint16_t rate) {
    audio::btSetSampleRate(rate);
}

}  // namespace

bool begin() {
    if (g_sink) return true;

    g_metaMutex = xSemaphoreCreateMutex();
    if (!g_metaMutex) return false;

    g_sink = new BluetoothA2DPSink();
    if (!g_sink) { vSemaphoreDelete(g_metaMutex); g_metaMutex = nullptr; return false; }

    // false = do NOT install the library's internal I2S driver.
    g_sink->set_stream_reader(dataCallback, false);
    g_sink->set_on_connection_state_changed(connectionCallback);
    g_sink->set_avrc_metadata_callback(metadataCallback);
    g_sink->set_avrc_metadata_attribute_mask(ESP_AVRC_MD_ATTR_TITLE |
                                             ESP_AVRC_MD_ATTR_ARTIST);
    g_sink->set_sample_rate_callback(sampleRateCallback);
    g_sink->start("Mochi");
    return true;
}

void end() {
    if (g_sink) { g_sink->end(); delete g_sink; g_sink = nullptr; }
    if (g_metaMutex) { vSemaphoreDelete(g_metaMutex); g_metaMutex = nullptr; }
}

Snapshot poll() {
    Snapshot s;
    s.connected       = g_connected;
    s.metadataChanged = false;
    s.title[0]        = 0;
    s.artist[0]       = 0;
    if (g_metaMutex && xSemaphoreTake(g_metaMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (g_metaDirty) {
            s.metadataChanged = true;
            strncpy(s.title,  g_title,  sizeof(s.title)  - 1);
            strncpy(s.artist, g_artist, sizeof(s.artist) - 1);
            s.title[sizeof(s.title) - 1]   = 0;
            s.artist[sizeof(s.artist) - 1] = 0;
            g_metaDirty = false;
        }
        xSemaphoreGive(g_metaMutex);
    }
    return s;
}

}  // namespace bt