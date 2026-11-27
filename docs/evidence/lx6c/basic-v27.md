# Basic v2.7 milestone

Virtual-device validation of the eight applicable Arduino examples, the
stock UIFlow2 2.5.4 launcher and the UART0 REPL. The inventory also
compiled the Gray IMU example. Its target has an IMU and Basic v2.7 does
not, so the application has no readings to display on this board.

| Runtime source | Runtime SHA-256 | Replay coverage |
| --- | --- | --- |
| `efba9bf8e44bddbb27f1572bb4131da29b80b87b` | `32169c14f2e57ff30ba0f9ea414e19d764ab307fe15024804c39e6bbd875ddd7` | Arduino LCD, GPIO, battery, SD, scrolling |
| `831ec158d320d251787dda01e736dc3a564e429b` | `33fa5f6940d278da5afa1e64a1d751a759c8e2e77ed2c8b12375a851cc93c40b` | speaker, WAV, deep sleep and stock UIFlow2 |

Stock image SHA-256 `613a802da0f496d339a4120c0322c0c7ec1db3d81474b4329077f8b233d4dccb`. The selected
catalog image is the 4 MiB build. The `--flash-mb 4` replay reached
DEVELOP, moved to APP RUN with A, entered the REPL with B and read each
of A, B and C through M5Unified. The guest reported one missing
`develop_bg.jpg` resource.

The Arduino tone sketch emitted 10 kHz and 4 kHz. The SD WAV sketch
completed with 733 Hz PCM captured at 96,000 samples per second. The
microSD guest wrote and closed `WriteTest.txt`, and the host read back
its exact contents. The wake example returned through ROM with
`DEEPSLEEP_RESET`.

See the [runner](../../../tools/run-lx6-cores.py) and
[checker](../../../tools/check-lx6-cores.py) for the fixture and stimulus
contract.
