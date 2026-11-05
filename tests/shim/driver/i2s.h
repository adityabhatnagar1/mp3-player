#pragma once
// Host-side fake of the legacy ESP32 I2S driver: captures what would reach the DAC,
// paced like real DMA (optionally sped up with SIM_SPEED).
#include <stddef.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
typedef enum { I2S_NUM_0 = 0 } i2s_port_t;
typedef enum { I2S_MODE_MASTER = 1, I2S_MODE_TX = 2 } i2s_mode_t;
typedef enum { I2S_BITS_PER_SAMPLE_16BIT = 16 } i2s_bits_per_sample_t;
typedef enum { I2S_CHANNEL_FMT_RIGHT_LEFT = 0 } i2s_channel_fmt_t;
typedef enum { I2S_COMM_FORMAT_STAND_I2S = 1 } i2s_comm_format_t;
typedef enum { I2S_CHANNEL_STEREO = 2 } i2s_channel_t;
#define ESP_INTR_FLAG_LEVEL1 2
#define I2S_PIN_NO_CHANGE (-1)
struct i2s_config_t {
  i2s_mode_t mode; uint32_t sample_rate; i2s_bits_per_sample_t bits_per_sample; i2s_channel_fmt_t channel_format;
  i2s_comm_format_t communication_format; int intr_alloc_flags; int dma_buf_count; int dma_buf_len;
  bool use_apll; bool tx_desc_auto_clear; int fixed_mclk;
};
struct i2s_pin_config_t { int mck_io_num, bck_io_num, ws_io_num, data_out_num, data_in_num; };
esp_err_t i2s_driver_install(i2s_port_t, const i2s_config_t*, int, void*);
esp_err_t i2s_driver_uninstall(i2s_port_t);
esp_err_t i2s_set_pin(i2s_port_t, const i2s_pin_config_t*);
esp_err_t i2s_zero_dma_buffer(i2s_port_t);
esp_err_t i2s_set_clk(i2s_port_t, uint32_t rate, i2s_bits_per_sample_t, i2s_channel_t);
esp_err_t i2s_write(i2s_port_t, const void* src, size_t size, size_t* written, TickType_t);
