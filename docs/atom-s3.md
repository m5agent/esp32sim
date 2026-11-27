# AtomS3 family on the native CLI

Select the model with `--board`. These profiles default to 8 MiB of Flash.
The QFN models default to 0 MiB PSRAM. The PICO-1 models default to 8 MiB
octal PSRAM and expose the LGA56 package eFuse field.

| Profile | Board connections |
| --- | --- |
| `m5stack-atoms3-lite` | G41 button, G35 addressable LED, G4 IR |
| `m5stack-atoms3` | ST7735S, MPU6886 on G38/G39, G41 button, G4 IR |
| `m5stack-atoms3-gc9107` | AtomS3 with GC9107 panel and its own GRAM geometry and probe ID |
| `m5stack-atoms3r` | ST7735S, LP5562, BMI270/BMM150 on G45/G0, G41 button, G47 IR |
| `m5stack-atoms3r-gc9107` | AtomS3R with GC9107 panel |
| `m5stack-atoms3u` | G41 button, G35 RGB, G12 IR, PDM microphone on G38/G39 |
| `m5stack-atom-echos3r` | G41 button, G47 IR, ES8311 codec and G18 amplifier enable |
| `m5stack-atoms3r-cam` | GC0308 camera, G18 power-down, SCCB on G12/G9, LCD_CAM and GDMA |
| `m5stack-atoms3r-m12` | OV3660 camera on the same SCCB and DVP wiring |

The VoiceS3R, CAM and M12 profiles represent the shipped C126-ECHO, C126-CAM
and C126-M12 boards without an attached display. The M5GFX source and the
product schematics define the wiring. The two LCD revisions model a shared
MOSI/readback line and different visible windows.

Use `press btn MS` for G41 and `imu AX AY AZ GX GY GZ` for accelerometer and
gyroscope inputs in sensor axes (g and dps). `mag X Y Z` supplies a BMM150
field in µT. The guest can apply its board-specific axis conversion.
The IR report lists frames observed at the transmitter pins. The LP5562 model
supports direct PWM and register readback. AtomS3U takes host PCM after the
I2S0 PDM converter when the guest has routed clock and data to the microphone.
Use `--cam-image` with a raw picture for RGB capture, or a baseline JPEG
matching the OV3660 guest-selected dimensions for JPEG mode.

```sh
python3 tools/run-atom-s3-docs.py atoms3 path/to/esp32sim path/to/firmware \
    path/to/stock.bin path/to/esp32s3_rev0_rom.elf path/to/new-output \
    --source SIMULATOR_COMMIT
python3 tools/check-atom-s3-docs.py path/to/new-output
```

Run the same command with the corresponding board ID for each matrix. The
Arduino runner expects `BOARD/CASE/sketch.merged.bin` under the firmware
directory. Use `--gc9107` for either LCD revision. FFmpeg is used for the
camera controls and framebuffer checks. The runner keeps exact command
arguments, guest output and firmware, ROM and simulator hashes in its output.

Coverage is limited to the digital paths above. LP5562 pattern engines,
physical microphone/optical behavior, and analog IR behavior are outside
the model.
