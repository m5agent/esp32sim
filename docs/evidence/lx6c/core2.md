# Core2 milestone

Virtual-device validation of 14 Arduino cases and the stock UIFlow2
2.5.3 launcher. The stock image opened SETTINGS from a physical-coordinate
touch, entered the UART0 REPL, reported separate A/B/C press and release
states and read back the injected point at (88, 110).

| Runtime source | Runtime SHA-256 | Replay coverage |
| --- | --- | --- |
| `efba9bf8e44bddbb27f1572bb4131da29b80b87b` | `32169c14f2e57ff30ba0f9ea414e19d764ab307fe15024804c39e6bbd875ddd7` | Arduino LCD, touch, PMIC, microphone, RTC and sleep |
| `831ec158d320d251787dda01e736dc3a564e429b` | `33fa5f6940d278da5afa1e64a1d751a759c8e2e77ed2c8b12375a851cc93c40b` | RMT strip, I2S WAV, stock launcher and HIMEM |
| `a47f8df8bf3cb8adb3bc8830b356db2f19aa8e2b` | `2b1b423944b9073befec44922b27f83a5fde35b5835347637931664c38f823ed` | indexed panel probe, both PMIC revisions, final SD and stock UART0 input |

Stock image SHA-256 `b228c0e2d877e77a8a352e917ed84f4da8ea92d15b25e28f431a2999620fe3c4`. The v1.1 preset
booted with 16 MiB flash and 8 MiB PSRAM. ESP-IDF exercised HIMEM pages
127, 128 and 255. The AXP192 preset separately ran the display, button
and battery examples. The PMIC IDs are 0x03 and 0x4a on their respective
presets, read at register 0x03.

The microphone wrote a silent control and two distinct 12 kHz WAVs with
733 and 1,234 Hz inputs. Core2's own WAV playback completed at a 48 kHz
capture rate. The motor supply covered 600–3,300 mV in nine transitions.
The SD run used the two published sample images resized to the documented
320×240 and read back the exact `WriteTest.txt` payload.

The QEMU native-HCI diagnostic has a separate transport. The stock Core2
catalog image was supplied as a merged BIN and the classic esp32sim
virtual VHCI hook requires matching application ELF symbols. Stock
BLE/NUS is **NOT RUN**.

See the [runner](../../../tools/run-lx6-cores.py) and
[checker](../../../tools/check-lx6-cores.py) for the fixture and stimulus
contract.
