# Classic ESP32 RMT RX, raw PDM RX and cyclic DAC fixture

Build this fixture with ESP-IDF v6.1 for an original ESP32. Boot the
bootloader, partition table and application using the ECO3 ROM. GPIO16
and GPIO17 select the test after boot. The replay script supplies those
inputs and the external receiver waveforms.

- RMT RX reads two active-low NEC frames on GPIO32 through the GPIO
  matrix and the ESP-IDF RX callback. An idle input gives no frame.
- I2S0 raw PDM RX samples GPIO19 using the master clock routed to GPIO5
  on AtomU. The guest reads packed 16-bit words through descriptor DMA.
  The replay checks all-zero words, all-one words and words that span
  input transitions. The VCD records the injected data pad transitions.
- The Basic v2.7 guest repeats a 500 Hz, two-level pattern through
  continuous DAC channel 0 at 32 kHz. Its WAV is a digital observation
  of GPIO25's DAC path. The continuous DAC code is owned by LX6-C,
  including the two-BCK divider at `cc89f1ad`.

```sh
idf.py -C docs/fixtures/lx6-receivers -B "$BUILD" -D SDKCONFIG="$BUILD/sdkconfig" build
python3 docs/fixtures/lx6-receivers/replay.py \
  "$LX6A_SIM" "$LX6C_SIM" "$BUILD" "$ESP32_ROM_ELF" "$NEW_OUT"
```

For a Stick-family replay, build the same guest with
`-D LX6_RECEIVER_STICK=ON` and give the replay script
`--stick-board m5stack-stickc` or another Stick profile with the
same pinout. It selects test mode on the two board buttons, reads PDM
on G34 with clock G0 and receives demodulated NEC on the Grove white
wire G33. G33 represents the external [M5Stack Unit IR](https://docs.m5stack.com/en/unit/Unit_IR)
receive line. The StickC boards have onboard IR transmitters.

The RMT and PDM controller contracts run with `cargo test -p esp32 --lib`.
The DAC controller contracts target the LX6-C implementation. In a
detached checkout of that source, include `dac_contract.rs` at the end of
`esp32/src/i2s.rs` and run
`cargo test --release -p esp32 --lib dac_contract`. They check the
cyclic DMA pointer and EOF, two-BCK timing, unsigned high-byte
conversion, stereo slot selection, and the RTC pad and SENS gates.

RMT RX consumes the GPIO waveform after carrier demodulation. The raw
PDM tap samples a routed digital source in I2S0 master mode. The fixture
does not model electrical microphone behavior or the DAC's analog
output stage.
