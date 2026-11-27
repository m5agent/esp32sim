# Classic ESP32 controller fixture

The ESP-IDF v6.1 dual-core guest uses UART0 at 115200 and then 230400 baud,
programs GPIO19 to 25% low-speed LEDC duty, drives GPIO25 with DAC
code 128 and emits two NEC envelopes on GPIO12 through the RMT
transmit driver. It enters deep sleep and verifies timer wake and
RTC memory retention after the ROM boots it again.

Build from a copy of this directory after activating ESP-IDF v6.1.

```sh
idf.py -B build -D SDKCONFIG=sdkconfig.lx6 build
esp32sim --chip esp32 --rom "$ROM" --boot rom --flash-mb 4 \
  --bootloader build/bootloader/bootloader.bin \
  --ptable build/partition_table/partition-table.bin \
  --app build/lx6_controls.bin --max-seconds 1 --no-dump --vcd controls.vcd
```

The UART driver should report effective rates near 115200 and 230400,
LEDC duty 256 of 1024, RMT transmission completion, and
`LX6 RTC TIMER WAKE PASS`. The emulator should report `GPIO25: 1656 mV`.
Decode GPIO12 using `nec_frames` in [check_lx6_atoms.py](../../../tools/check_lx6_atoms.py)
and expect packets `aa55ff00` and `a956fe01`. The PWM frequency and
physical DAC voltage are outside the guest test.
