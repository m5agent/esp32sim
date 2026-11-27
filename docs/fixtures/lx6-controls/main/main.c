// SPDX-License-Identifier: MIT
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include "driver/dac_oneshot.h"
#include "driver/ledc.h"
#include "driver/rmt_tx.h"
#include "driver/uart.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

RTC_DATA_ATTR static uint32_t boot_cookie;

static void transmit_nec(void) {
    rmt_channel_handle_t tx;
    rmt_encoder_handle_t encoder;
    const rmt_tx_channel_config_t config = {
        .gpio_num = 12, .clk_src = RMT_CLK_SRC_APB, .resolution_hz = 1000000,
        .mem_block_symbols = 64, .trans_queue_depth = 1
    };
    const rmt_copy_encoder_config_t copy = {};
    const rmt_transmit_config_t transfer = {};
    ESP_ERROR_CHECK(rmt_new_tx_channel(&config, &tx));
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy, &encoder));
    ESP_ERROR_CHECK(rmt_enable(tx));
    const uint32_t packets[] = {0xaa55ff00, 0xa956fe01};
    for (unsigned i = 0; i < 2; i++) {
        rmt_symbol_word_t symbols[34];
        symbols[0] = (rmt_symbol_word_t) {.duration0 = 9000, .level0 = 1, .duration1 = 4500};
        for (unsigned bit = 0; bit < 32; bit++) {
            symbols[1 + bit] = (rmt_symbol_word_t) {
                .duration0 = 560, .level0 = 1, .duration1 = packets[i] & (1u << bit) ? 1690 : 560
            };
        }
        symbols[33] = (rmt_symbol_word_t) {.duration0 = 560, .level0 = 1, .duration1 = 10000};
        ESP_ERROR_CHECK(rmt_transmit(tx, encoder, symbols, sizeof(symbols), &transfer));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(tx, 200));
    }
    ESP_ERROR_CHECK(rmt_disable(tx));
    ESP_ERROR_CHECK(rmt_del_encoder(encoder));
    ESP_ERROR_CHECK(rmt_del_channel(tx));
    puts("LX6 RMT NEC TX DONE");
}

void app_main(void) {
    if (esp_reset_reason() == ESP_RST_DEEPSLEEP) {
        if (boot_cookie != 0x154acb09 || esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_TIMER) abort();
        puts("LX6 RTC TIMER WAKE PASS");
        return;
    }

    const uart_config_t console = {
        .baud_rate = 115200, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_APB
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &console));
    uint32_t initial, changed;
    ESP_ERROR_CHECK(uart_get_baudrate(UART_NUM_0, &initial));
    ESP_ERROR_CHECK(uart_set_baudrate(UART_NUM_0, 230400));
    ESP_ERROR_CHECK(uart_get_baudrate(UART_NUM_0, &changed));
    printf("LX6 UART0 baud %" PRIu32 " -> %" PRIu32 "\n", initial, changed);
    // The hardware divider has a finite number of fractional bits.
    if (initial < 114000 || initial > 116400 || changed < 228000 || changed > 232800) abort();

    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0, .freq_hz = 2500, .clk_cfg = LEDC_USE_APB_CLK
    };
    const ledc_channel_config_t channel = {
        .gpio_num = 19, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0, .duty = 256
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    ESP_ERROR_CHECK(ledc_channel_config(&channel));
    vTaskDelay(pdMS_TO_TICKS(20)); // allow the first PWM boundary to latch duty
    const uint32_t duty = ledc_get_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    if (duty != 256) abort();
    printf("LX6 LEDC duty %" PRIu32 "\n", duty);

    dac_oneshot_handle_t dac;
    const dac_oneshot_config_t config = { .chan_id = DAC_CHAN_0 };
    ESP_ERROR_CHECK(dac_oneshot_new_channel(&config, &dac));
    ESP_ERROR_CHECK(dac_oneshot_output_voltage(dac, 128));
    transmit_nec();
    boot_cookie = 0x154acb09;
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(200000));
    puts("LX6 RTC TIMER SLEEP");
    fflush(stdout);
    esp_deep_sleep_start();
}
