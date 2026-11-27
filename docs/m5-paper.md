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
- `paper-tag HEX_UID` or `paper-tag none` on PaperMono. The modeled NFC-A reader supports 4- or 7-byte UIDs.
- `--lora-peer IN:OUT` connects two PaperMono processes on loopback UDP. The peer uses the Cardputer-ADV CAP channel header and SX1262 packet engine. Give the other process the reversed port pair.

The board matrix runners in `tools/run-*-docs.sh` accept a simulator binary,
a directory containing prebuilt `CASE/sketch.merged.bin` images, and a new
output directory. Use `--source COMMIT --rom-dir ROM_DIR` to bind the run to
its source and ROM. `tools/check-paper-docs.py OUT` evaluates captured frames
and the supported guest oracles. `tools/probe-paper-service.py` tests the
private service stream via an inherited Unix socket.

Open validation covers PMIC wake and power-key behavior, full NFC
read/write and card emulation, complete stock input and REPL, SD power
gating and partial-waveform cases outside the supported display drivers.
Provisional Arduino board selections remain visible in the individual
build receipts.
