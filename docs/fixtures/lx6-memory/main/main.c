// SPDX-License-Identifier: MIT
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_psram.h"
#include "sdkconfig.h"

#if CONFIG_SPIRAM_BANKSWITCH_ENABLE
#include "esp32/himem.h"

static void verify_banks(void) {
    const size_t page_size = ESP_HIMEM_BLKSZ;
    const size_t capacity = esp_himem_get_phys_size();
    if (capacity != 4 * 1024 * 1024 + page_size) abort();
    esp_himem_handle_t ram;
    esp_himem_rangehandle_t range;
    ESP_ERROR_CHECK(esp_himem_alloc(capacity, &ram));
    ESP_ERROR_CHECK(esp_himem_alloc_map_range(page_size, &range));

    // The last page is physical page 255. Write it through the same slot as
    // the first bank and read both back after remapping.
    const size_t offsets[] = {0, page_size, capacity - page_size};
    const uint32_t values[] = {0x7f127f12, 0x80128012, 0xff12ff12};
    for (unsigned pass = 0; pass < 2; pass++) {
        for (unsigned i = 0; i < 3; i++) {
            void *mapped;
            ESP_ERROR_CHECK(esp_himem_map(ram, range, offsets[i], 0, page_size, 0, &mapped));
            volatile uint32_t *last = (volatile uint32_t *)mapped + page_size / sizeof(uint32_t) - 1;
            if (pass == 0) {
                *last = values[i];
            } else if (*last != values[i]) {
                printf("LX6 HIMEM mismatch at %u\n", (unsigned)offsets[i]);
                abort();
            }
            ESP_ERROR_CHECK(esp_himem_unmap(range, mapped, page_size));
        }
    }

    ESP_ERROR_CHECK(esp_himem_free_map_range(range));
    ESP_ERROR_CHECK(esp_himem_free(ram));
    printf("LX6 HIMEM capacity=%u pages=127,128,255 OK\n", (unsigned)capacity);
}
#endif

void app_main(void) {
    uint32_t size;
    ESP_ERROR_CHECK(esp_flash_get_size(NULL, &size));
    if (size != 16 * 1024 * 1024) abort();
    uint32_t value = 0x25f04adb;
    uint32_t readback = 0;
    ESP_ERROR_CHECK(esp_flash_erase_region(NULL, size - 4096, 4096));
    ESP_ERROR_CHECK(esp_flash_write(NULL, &value, size - sizeof(value), sizeof(value)));
    ESP_ERROR_CHECK(esp_flash_read(NULL, &readback, size - sizeof(readback), sizeof(readback)));
    if (readback != value) abort();
    printf("LX6 flash bytes=%" PRIu32 " last_word=%08" PRIx32 " OK\n", size, readback);

    size_t physical = esp_psram_get_size();
    size_t heap = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    printf("LX6 PSRAM physical=%u heap=%u\n", (unsigned)physical, (unsigned)heap);
#if CONFIG_SPIRAM_BANKSWITCH_ENABLE
    if (physical != 8 * 1024 * 1024) abort();
    verify_banks();
#else
    if (physical != 2 * 1024 * 1024 || heap <= 32768 || heap > physical) abort();
    size_t allocation = heap - 32768;
    volatile uint32_t *ram = heap_caps_malloc(allocation, MALLOC_CAP_SPIRAM);
    if (!ram) abort();
    ram[0] = 0x12345678;
    size_t last = allocation / sizeof(*ram) - 1;
    ram[last] = 0x87654321;
    if (ram[0] != 0x12345678 || ram[last] != 0x87654321) abort();
    heap_caps_free((void *)ram);
    printf("LX6 PSRAM allocation=%u boundary readback OK\n", (unsigned)allocation);
#endif
    puts("LX6 memory PASS");
}
