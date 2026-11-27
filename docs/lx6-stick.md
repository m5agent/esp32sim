# Classic Stick boards

Use `--chip esp32` with one of these board names and its corresponding merged
flash image.

| CLI board | Display | Flash / PSRAM | Internal peripherals |
| --- | --- | --- | --- |
| `m5stack-stickc` | 80 × 160 ST7735S | 4 / 0 MiB | AXP192, BM8563, MPU6886, PDM microphone |
| `m5stack-stickc-plus` | 135 × 240 ST7789 | 4 / 0 MiB | AXP192, BM8563, MPU6886, PDM microphone, piezo |
| `m5stack-stickc-plus-se` | 135 × 240 ST7789 | 4 / 0 MiB | AXP192, BM8563, PDM microphone, piezo |
| `m5stack-stickc-plus2` | 135 × 240 ST7789 | 8 / 2 MiB | G4 power hold, G38 battery divider, BM8563, MPU6886, PDM microphone, piezo |
| `m5stack-m5stick` | 64 × 128 SH1107 | 4 / 0 MiB | IP5306, MPU9250/AK8963 |
| `m5stack-stickt2` | 135 × 240 ST7789 | 4 / 0 MiB | AXP192, BM8563, MPU6886, Lepton 3.x and I²C encoder |

Use `press a MS` or `press b MS` for the labelled keys. The original M5Stick
has only `a`. Plus2 also accepts `press pwr MS` on GPIO35. Feed `imu AX AY AZ GX
GY GZ` in sensor axes, with acceleration in g and angular velocity in degrees
per second. M5Stick also accepts `mag X Y Z` in microteslas. Apply
`battery-mv MILLIVOLTS` for the I²C PMICs. On Plus2, `adc 38 VOLTS` sets the
voltage after the 2:1 battery divider. `vbus-mv 0` disconnects the simulated
USB supply. `pmic-key short` and `pmic-key long` latch AXP192 power-key flags.
A short Plus2 event grants a two-second initial power window for the guest
to assert G4. The BM8563 RTC advances in virtual time.

StickT2 accepts `thermal COOL HOT` in Raw14 counts, `encoder-step DELTA` and
`encoder-button down|up`. Each synthetic frame contains a warm rectangle in a
cool background. The new scene takes effect on its next complete frame. The
M5Stick MPU9250 models raw FIFO and AK8963 bypass reads. DMP and auxiliary
master FIFO payloads are outside this subset.

The PDM input accepts decoded PCM with `--mic-wav FILE` and requires the guest
to configure its PDM receive mode and the board's clock/data route. The piezo
model observes LEDC and derives a baseband signal from the guest's routed
one-bit I²S output. Raw microphone waveforms, acoustic behavior, IR receive,
and physical thermal measurements are outside these models.

To replay the pinned documentation cases and the catalog image in one tool:

```sh
python3 tools/run-lx6-stick.py stickc path/to/esp32sim path/to/agent-plan \
  path/to/stickc.bin path/to/esp32_rev300_rom.elf path/to/new-output
```

Use `--case CASE` for a scoped replay. For the factory boards,
`--probe-stock path/to/atom-lite.bin` additionally runs a generic MicroPython
controller probe on the exact board wiring. The two executions have
independent firmware identities and verdicts.
