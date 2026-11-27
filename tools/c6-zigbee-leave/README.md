# NanoC6 Zigbee leave and rejoin probe

The coordinator sketch uses Arduino-ESP32 from M5Stack 3.2.5. Pair it with the
original M5Burner Zigbee HA On/Off light For NanoC6 v1.0
(`a736daf26ca51983bf6a9bd30dd79c3148631291edf50acea1b8f29600183504`).

```sh
arduino-cli compile \
  -b m5stack:esp32:m5stack_nano_c6:PartitionScheme=zigbee_zczr,ZigbeeMode=zczr \
  --build-path "$BUILD" tools/c6-zigbee-leave/c6_zigbee_leave
python3 tools/run-nanoc6-peers.py "$SIM" "$C6_ROM" \
  "$BUILD/c6_zigbee_leave.ino.merged.bin" "$ORIGINAL_LIGHT" "$OUT" \
  --commit "$(git rev-parse HEAD)" --seconds 38 --observe-leds
```

The coordinator sends On, then a ZDO management leave request with rejoin
enabled and finally Off. Check for `LEAVE_RESPONSE status=0` in the coordinator
log, `ZDO Leave (0x3), status: ESP_OK` in the light log, and actual LED changes
for the commands on both sides of the leave. Cooja forwards the frames between
the two software radio models.
