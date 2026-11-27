#include <inttypes.h>
#include <stdio.h>
#include "esp_flash.h"
#include "esp_system.h"
#include "soc/lp_aon_reg.h"
#include "soc/lp_clkrst_reg.h"
#include "soc/soc.h"

// This sector is outside all partitions of the probe's default partition table.
#define SCRATCH 0x900000

void app_main(void)
{
    const uint32_t expected = 0xa529f03c;
    uint32_t reason = REG_READ(LP_CLKRST_RESET_CAUSE_REG) & 0x1f;
    printf("RESET_PROBE raw=%" PRIu32 " sdk=%d\n", reason, esp_reset_reason());
    if (reason == 1) {
        ESP_ERROR_CHECK(esp_flash_erase_region(NULL, SCRATCH, 4096));
        ESP_ERROR_CHECK(esp_flash_write(NULL, &expected, SCRATCH, sizeof(expected)));
        fflush(stdout);
        esp_restart();
    }

    uint32_t actual = 0;
    ESP_ERROR_CHECK(esp_flash_read(NULL, &actual, SCRATCH, sizeof(actual)));
    if (actual != expected || esp_reset_reason() != ESP_RST_SW) {
        printf("RESET_PROBE_FAIL raw=%" PRIu32 " flash=%08" PRIx32 "\n", reason, actual);
        return;
    }
    if (reason == 12) {
        fflush(stdout);
        REG_WRITE(LP_AON_SYS_CFG_REG, LP_AON_HPSYS_SW_RESET);
    } else if (reason == 3) {
        puts("RESET_PROBE_PASS");
    }
}
