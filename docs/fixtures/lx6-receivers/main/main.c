// SPDX-License-Identifier: MIT
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "driver/dac_continuous.h"
#include "driver/gpio.h"
#include "driver/i2s_pdm.h"
#include "driver/rmt_rx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static bool rmt_received(rmt_channel_handle_t rx, const rmt_rx_done_event_data_t *event, void *context)
{
    BaseType_t wake = pdFALSE;
    xQueueSendFromISR(context, &event->num_symbols, &wake);
    return wake;
}

static void receive_nec(bool idle)
{
    rmt_channel_handle_t rx;
    const rmt_rx_channel_config_t config = {
        .gpio_num = 32,
        .clk_src = RMT_CLK_SRC_APB,
        .resolution_hz = 1000000,
        .mem_block_symbols = 64,
    };
    const rmt_receive_config_t range = {
        .signal_range_min_ns = 1000,
        .signal_range_max_ns = 12000000,
    };
    QueueHandle_t events = xQueueCreate(1, sizeof(size_t));
    const rmt_rx_event_callbacks_t callback = {.on_recv_done = rmt_received};
    ESP_ERROR_CHECK(rmt_new_rx_channel(&config, &rx));
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx, &callback, events));
    ESP_ERROR_CHECK(rmt_enable(rx));
    rmt_symbol_word_t symbols[64];
    puts("LX6 RMT RX ARMED");
    for (unsigned frame = 0; frame < (idle ? 1 : 2); frame++) {
        size_t count = 0;
        ESP_ERROR_CHECK(rmt_receive(rx, symbols, sizeof(symbols), &range));
        if (xQueueReceive(events, &count, pdMS_TO_TICKS(2400)) != pdTRUE) {
            if (!idle) abort();
            puts("LX6 RMT NO FRAME");
            break;
        }
        if (idle || count != 34 || symbols[0].level0 || !symbols[0].level1
                || symbols[0].duration0 < 8800 || symbols[0].duration0 > 9200
                || symbols[0].duration1 < 4300 || symbols[0].duration1 > 4700) {
            printf("LX6 RMT invalid count=%u start=%" PRIx32 "\n", (unsigned) count, symbols[0].val);
            abort();
        }
        uint32_t packet = 0;
        for (unsigned bit = 0; bit < 32; bit++) {
            rmt_symbol_word_t symbol = symbols[bit + 1];
            if (symbol.level0 || !symbol.level1 || symbol.duration0 < 500 || symbol.duration0 > 620) abort();
            if (symbol.duration1 >= 1600 && symbol.duration1 <= 1780) {
                packet |= UINT32_C(1) << bit;
            } else if (symbol.duration1 < 500 || symbol.duration1 > 620) {
                abort();
            }
        }
        printf("LX6 NEC RX %08" PRIx32 "\n", packet);
    }
    ESP_ERROR_CHECK(rmt_disable(rx));
    ESP_ERROR_CHECK(rmt_del_channel(rx));
    vQueueDelete(events);
}

static void receive_raw_pdm(void)
{
    i2s_chan_handle_t rx;
    i2s_chan_config_t config = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    config.dma_desc_num = 4;
    config.dma_frame_num = 64;
    const i2s_pdm_rx_config_t pdm = {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(640000),
        .slot_cfg = I2S_PDM_RX_SLOT_RAW_FMT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {.clk = 5, .din = 19},
    };
    ESP_ERROR_CHECK(i2s_new_channel(&config, NULL, &rx));
    ESP_ERROR_CHECK(i2s_channel_init_pdm_rx_mode(rx, &pdm));
    ESP_ERROR_CHECK(i2s_channel_enable(rx));
    puts("LX6 PDM RAW ARMED");
    for (unsigned interval = 0; interval < 5; interval++) {
        unsigned lows = 0, highs = 0, mixed = 0;
        for (unsigned batch = 0; batch < 64; batch++) {
            uint16_t words[64];
            size_t bytes;
            ESP_ERROR_CHECK(i2s_channel_read(rx, words, sizeof(words), &bytes, 500));
            if (bytes != sizeof(words)) abort();
            for (unsigned i = 0; i < 64; i++) {
                if (words[i] == UINT16_MAX) highs++;
                else if (words[i] == 0) lows++;
                else mixed++;
            }
        }
        printf("LX6 PDM RAW %u low=%u high=%u mixed=%u\n", interval, lows, highs, mixed);
    }
    ESP_ERROR_CHECK(i2s_channel_disable(rx));
    ESP_ERROR_CHECK(i2s_del_channel(rx));
}

static void play_dac(void)
{
    const dac_continuous_config_t config = {
        .chan_mask = DAC_CHANNEL_MASK_CH0,
        .desc_num = 4,
        .buf_size = 256,
        .freq_hz = 32000,
        .clk_src = DAC_DIGI_CLK_SRC_DEFAULT,
        .chan_mode = DAC_CHANNEL_MODE_SIMUL,
    };
    dac_continuous_handle_t dac;
    ESP_ERROR_CHECK(dac_continuous_new_channels(&config, &dac));
    ESP_ERROR_CHECK(dac_continuous_enable(dac));
    uint8_t wave[128];
    for (unsigned i = 0; i < sizeof(wave); i++) wave[i] = i % 64 < 32 ? 32 : 224;
    size_t loaded = 0;
    ESP_ERROR_CHECK(dac_continuous_write_cyclically(dac, wave, sizeof(wave), &loaded));
    if (loaded != sizeof(wave)) abort();
    puts("LX6 DAC GPIO25 CYCLIC ACTIVE");
    vTaskDelay(pdMS_TO_TICKS(220));
    ESP_ERROR_CHECK(dac_continuous_disable(dac));
    ESP_ERROR_CHECK(dac_continuous_del_channels(dac));
    puts("LX6 DAC GPIO25 CYCLIC DONE");
}

void app_main(void)
{
    const gpio_config_t selector = {
        .pin_bit_mask = (UINT64_C(1) << 16) | (UINT64_C(1) << 17),
        .mode = GPIO_MODE_INPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&selector));
    puts("LX6 RECEIVERS READY");
    vTaskDelay(pdMS_TO_TICKS(900));
    unsigned mode = (unsigned)gpio_get_level(16) | ((unsigned)gpio_get_level(17) << 1);
    printf("LX6 MODE %u\n", mode);
    switch (mode) {
        case 0: receive_nec(false); break;
        case 1: receive_nec(true); break;
        case 2: receive_raw_pdm(); break;
        case 3: play_dac(); break;
        default: abort();
    }
    puts("LX6 RECEIVERS DONE");
}
