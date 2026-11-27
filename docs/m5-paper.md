# M5Stack S3 e-paper boards

| CLI board | Panel | Frame in glass coordinates | Flash / RAM |
| --- | --- | --- | --- |
| `m5stack-paperdiy` | ET047TC1 parallel | 540 × 960 | 16 MiB flash, 8 MiB OPI PSRAM |
| `m5stack-papers3` | ET047TC1 parallel | 540 × 960 | 16 MiB flash, 8 MiB OPI PSRAM |
| `m5stack-papermono` | SSD1677 SPI | 480 × 800 | 16 MiB flash, 8 MiB OPI PSRAM |
| `m5stack-papercolor` | ED2208 SPI | 400 × 600 | 16 MiB flash, 8 MiB OPI PSRAM |

Use a compatible merged flash image and the ESP32-S3 mask ROM.

```sh
esp32sim --board m5stack-paperdiy --boot rom --flash-image sketch.merged.bin \
  --console both --max-seconds 12 --tft-png frame.png --no-dump
```

The parallel boards sample LCD_CAM I80 data at the panel's source latch. The
visible frame is updated at the end of the powered waveform sequence. The
parallel gray decoder recognizes the quality and text sequences from M5GFX
0.3.0. Other waveforms use a black or white digital endpoint. The two SPI
controllers retain their last visible frame during BUSY, including during
partial updates. PaperColor uses a 15-second virtual refresh for full and
partial updates, at the low end of the driver's documented range. The
models describe digital display behavior and approximate refresh time.

The board exposes SD through its SPI socket for PaperDIY, PaperS3 and
PaperColor, and through the S3 SDMMC host for PaperMono. Use `--sd-rw image.img`
for a private writable copy of a fixture. PaperS3 and PaperMono accept
`touch X Y DOWN` in portrait glass coordinates. PaperDIY and PaperMono
provide the script buttons `a` and `b`. PaperColor also provides `c`.

Additional script inputs:

- `paper-battery MILLIVOLTS CHARGING` on PaperMono and PaperColor.
- `adc 3 VOLTS` on PaperS3, before the 2:1 divider factor applied by the guest.
- `sht40 CELSIUS PERCENT_RH` on PaperColor.
- `imu AX AY AZ GX GY GZ` on PaperMono, with acceleration in g and angular velocity in degrees per second.
- `paper-tag HEX_UID` or `paper-tag none` on PaperMono. The modeled NFC-A reader supports 4- or 7-byte UIDs.
- `--lora-peer IN:OUT` synchronizes two PaperMono processes over loopback TCP. Give the other process the reversed port pair. See [paired radios](cli.md#paired-sx1262-and-cc1101-radios).

The board matrix runners in `tools/run-*-docs.sh` accept a simulator binary,
a directory containing prebuilt `CASE/sketch.merged.bin` images, and a new
output directory. Use `--source COMMIT --rom-dir ROM_DIR` to bind the run to
its source and ROM. Pass `--sd` with a FAT32 fixture for the SD examples, or
`--stock` with a matching UIFlow2 image to exercise stock display, USB CDC and
board input. `tools/check-paper-docs.py OUT` checks the guest and its captured
output. `tools/probe-paper-service.py` exercises the service stream via an
inherited Unix socket.

PaperMono's NFC examples cover reader-side scanning, block reads and NDEF
write/readback. The card-emulation example requires NFC power via IOE1 GPIO4
and an external reader fixture. PaperColor's GPIO48 IR decoder observes the
LEDC carrier and validates completed NEC frames. The board package's built-in
RGB LED alias shares GPIO48, so disable IRremote's send feedback for this
transmitter. The PaperColor SHT40 sketch uses `Wire1` to share the controller
that owns the internal GPIO3 SDA and GPIO2 SCL lines. The original 3.2.5
Arduino build selections are recorded in the individual receipts. The
M5Stack 3.3.10 package provides all four board IDs.

The ESP Online Tester board and I80 implementations at `81bc93c0` served
as comparative references. The pin routes, refresh commands and waveform
sequences here follow the pinned M5GFX board descriptions and drivers.
