# Tab5 USB MSC sector readback

Build the sketch with the M5Stack 3.2.5 Arduino board package. It enumerates
a full-speed mass storage device through the ESP-IDF USB host stack. It sends
a bulk-only SCSI READ(10) for LBA 4 and prints the 512 returned bytes after
checking the command status wrapper.

```sh
arduino-cli compile -b m5stack:esp32:m5stack_tab5 \
  --build-path "$BUILD" tools/tab5-msc-probe/msc_sector
printf '2.0 usb-msc fs\n' > "$INPUTS"
esp32sim --chip p4 --board m5stack-tab5 --boot rom \
  --flash-image "$BUILD/msc_sector.ino.merged.bin" --script "$INPUTS" \
  --usb-msc-image "$IMAGE" --debug usb_otg --max-seconds 4 --no-dump \
  > "$SERIAL"
python3 - "$IMAGE" "$SERIAL" <<'PY'
from hashlib import sha256
from pathlib import Path
import re
import sys

image, serial = map(Path, sys.argv[1:])
log = serial.read_text()
assert "MSC_ENUMERATED" in log and "MSC_READ_SECTOR_4_OK" in log
match = re.search(r"MSC_SECTOR_4 ([0-9a-f]{1024})", log)
assert match, "missing complete guest sector"
guest = sha256(bytes.fromhex(match[1])).hexdigest()
host = sha256(image.read_bytes()[2048:2560]).hexdigest()
assert host == guest, (host, guest)
print(guest)
PY
```

Use a disposable raw image of at least five 512-byte sectors. The emulator
serves it read-only. The firmware prints the bytes so the same host hash
implementation can check both copies.
