# ESP32-C6 flash and software reset probe

Build with ESP-IDF v5.4.2 for ESP32-C6. The probe needs a disposable 16 MiB
virtual flash image because it erases and writes the sector at `0x900000`.
Run against the C6 mask ROM and a fresh image assembled from the build outputs.

```sh
idf.py build
esp32sim --chip c6 --board arduino-nesso-n1 --flash-mb 16 --boot rom \
  --bootloader build/bootloader/bootloader.bin \
  --ptable build/partition_table/partition-table.bin \
  --app build/c6_reset_probe.bin --max-seconds 5 --exit-on RESET_PROBE_PASS
```

Expect `RESET_PROBE raw=1 sdk=1`, then `raw=12 sdk=3` after
`esp_restart()`, then `raw=3 sdk=3` after a write to
`LP_AON_SYS_CFG.HPSYS_SW_RESET`. The guest checks flash contents after each
restart before reporting `RESET_PROBE_PASS`. The reset causes are from
ESP-IDF v5.4.2's [C6 ROM reset definitions](https://github.com/espressif/esp-idf/blob/v5.4.2/components/esp_rom/esp32c6/include/esp32c6/rom/rtc.h).
