# Classic ESP32 exact memory fixture

This ESP-IDF v6.1 application boots via the classic ECO3 ROM with both CPUs
enabled. It erases, programs and reads the last word of a 16 MiB Flash image.
The 2 MiB build checks the physical PSRAM size and writes to both ends of a
2,064,384-byte heap allocation. The 8 MiB build reserves one HIMEM window
and writes and reads distinct values through that window on physical pages
127, 128 and 255.

Activate ESP-IDF v6.1, then build from a copy of this directory.

```sh
idf.py -B build-2m -D SDKCONFIG=sdkconfig.2m build
idf.py -B build-8m -D SDKCONFIG=sdkconfig.8m \
  -D 'SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.himem' build
```

For each `n` in 2 and 8, run with an external ECO3 ROM and the native CLI.

```sh
esp32sim --chip esp32 --boot rom --rom "$ROM" --flash-mb 16 --psram-mb "$n" \
  --bootloader "build-${n}m/bootloader/bootloader.bin" \
  --ptable "build-${n}m/partition_table/partition-table.bin" \
  --app "build-${n}m/lx6_memory.bin" --max-seconds 1 --no-reboot --no-dump
```

Both runs must print `LX6 memory PASS`. The 8 MiB run additionally must
print `LX6 HIMEM capacity=4227072 pages=127,128,255 OK`. As a negative
control, rerun the 8 MiB application with `--psram-mb 4` and require no
`LX6 memory PASS` marker. This tests the classic native memory controller
with a synthetic application. Board-specific stock firmware and memory
timing require their own validation.
