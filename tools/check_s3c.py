#!/usr/bin/env python3
"""Evaluate controller traces, firmware output and framebuffer from an S3-C run."""

import argparse
import ast
from collections import Counter
from hashlib import sha256
import json
from pathlib import Path
import re
import struct
import zlib

from check_cores3 import audio, bmp_mirrored_rgb565, changed, fat32_files, require, text


def png(path, width, height):
    blob = path.read_bytes()
    require(blob.startswith(b"\x89PNG\r\n\x1a\n"), "invalid PNG")
    at = 8
    data = bytearray()
    while at < len(blob):
        size, name = struct.unpack_from(">I4s", blob, at)
        content = blob[at + 8:at + 8 + size]
        require(zlib.crc32(name + content) == struct.unpack_from(">I", blob, at + 8 + size)[0],
                "invalid PNG CRC")
        at += size + 12
        if name == b"IHDR":
            require(content == struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0),
                    f"display dimensions differ from {width}x{height}")
        if name == b"IDAT":
            data.extend(content)
    pixels = zlib.decompress(data)
    stride = 1 + 3 * width
    require(len(pixels) == stride * height and not any(pixels[::stride]), "unexpected RGB8 scanlines")
    return b"".join(pixels[off + 1:off + stride] for off in range(0, len(pixels), stride))


def capture(root, label, size):
    row = json.loads((root / f"{label}.json").read_text())
    require(row["status"] == 0, f"{label} runtime exited {row['status']}")
    serial = (root / f"{label}.log").read_text(errors="replace")
    log = (root / f"{label}.err").read_text(errors="replace")
    require("stop: Halted" in log, f"{label} did not reach the requested horizon")
    require(not re.search(r"Guru Meditation|assert failed|panic'ed|abort\(\) was called", serial),
            f"{label} firmware panicked")
    return serial, log, png(root / f"{label}.png", *size)


def field(log, name):
    match = re.search(rf"\b{re.escape(name)}=(0x[0-9a-f]+|\d+)\b", log)
    require(match, f"missing board observation {name}")
    return int(match[1], 0)


def modes(frame):
    return Counter(zip(frame[0::3], frame[1::3], frame[2::3]))


def plc_button_rows(screen):
    row = 240 * 3
    return b"".join(screen[y * row:y * row + 130 * 3] for y in range(8, 32))


def plc(root, case, serial, log, frame):
    if case == "input_output-1":
        require("Write Relay 3 to ON" in serial and "Write Relay 3 to OFF" in serial and
                field(log, "relay-states") & 0x8001 == 0x8001, "missing guest-driven all-on/all-off relay states")
        return "guest toggled all four physical relay outputs"
    if case == "input_output-2":
        require(all(f"Input: {pattern}" in serial for pattern in
                    ("0, 0, 0, 0, 0, 0, 0, 0", "1, 0, 0, 0, 0, 0, 0, 1", "0, 0, 0, 1, 0, 0, 0, 1")),
                "isolated inputs did not round-trip through AW9523")
        return "zero-input control and two mapped AW9523 input patterns"
    if case == "sensor":
        require(all(marker in serial for marker in ("Temp: 25.00", "Temp: -6.25", "Temp: 38.12",
                                                   "Power: 11.80", "Power: 15.38", "Io: 0.30", "Io: 0.70")),
                "missing default or one of the injected INA226 and LM75B readings")
        return "three distinct LM75B and INA226 readings via guest I2C"
    if case == "rtc":
        values = re.findall(r"Time: 2077/3/7 11:45:(\d+)", serial)
        require(len(set(values)) >= 3, "calendar set/read or progression missing")
        return "guest set RX8130 calendar and read advancing time"
    if case == "rgb_led":
        require(field(log, "rgb-seen") & 0x97 == 0x97, "missing off, red, green, blue or white")
        return "all five requested GPIO-expander RGB states"
    if case == "quickstart-display":
        require(len(modes(frame)) >= 3 and re.search(r"StamPLC: [1-9]\d* RAMWR, [1-9]\d* pixels", log),
                "display drawing missing")
        return "multicolor drawing over ST7789 SPI within the 240×135 crop"
    if case == "button":
        _, _, control = capture(root, case + "-control", (240, 135))
        require(changed(frame, control) >= 200, "three front key actions did not change guest framebuffer")
        require(sha256(plc_button_rows(frame)).hexdigest() ==
                "39876955805868aac729e75e117413ed6ec7b8ebcdf1d661384891a390babf0c"
                and not any(plc_button_rows(control)), "button A/B/C text rows missing")
        return "guest rendered all three button labels compared with the untouched screen"
    if case == "buzzer":
        marker = re.search(r"StamPLC LEDC buzzer \(Hz, us\): (\[[^\n]*\])", log)
        require(marker, "GPIO44 LEDC report missing")
        tones = ast.literal_eval(marker[1])
        for hz in (523, 659, 880):
            require(any(abs(frequency - hz) <= 2 and 47_000 <= micros <= 54_000
                        for frequency, micros in tones), f"GPIO44 tone {hz} Hz / 50 ms missing")
        return "GPIO44 LEDC produced 523, 659 and 880 Hz tones at 50 ms"
    if case == "can":
        require(">> Message queued for transmission" in serial and "<< Message received" in serial and
                "ID is 43691" in serial and "Data: 0 1 2 3 4 5 6 7" in serial,
                "guest CAN TX/ACK and peer payload not observed")
        return "TWAI TX and extended-ID peer reply with eight matching payload bytes"
    if case == "modbus-1":
        require(field(log, "modbus-echoes") == 3 and field(log, "relays") == 2,
                "peer coil requests, all three CRC-valid echoes and relay output did not agree")
        return "three Modbus master writes echoed over UART with mapped relay changes"
    if case == "modbus-2":
        require(all(marker in serial for marker in ("Relay 0 ON", "Relay 3 OFF", "Temperature: 27.50",
                                                    "Bus Voltage: 12.50", "Shunt Current: 0.25")),
                "missing guest Modbus coil or input-register data")
        return "guest slave-peer exchange on RS485 including coils and input registers"
    if case == "sd_card":
        require("SD card write success" in serial and "Hello, World! Count:" in serial and
                re.search(r"\[sd-spi\].* [1-9]\d* blocks written", log), "missing SD guest file round trip")
        control_serial, _, _ = capture(root, case + "-control", (240, 135))
        require("SD card write success" not in control_serial, "no-SD control unexpectedly wrote a file")
        return "SPI SD guest writes and reads test.txt, with no-media control"
    return None


def stack(root, case, serial, log, frame):
    if case in ("rgb", "quickstart-rgb"):
        require(field(log, "rgb-updates") >= 5 and field(log, "rgb-colors") & 0x17 == 0x17,
                "missing independent colors from PY32 RGB refresh")
        return "PY32 RGB565 LED refresh observed across the twelve pixels"
    if case == "display":
        require(len(modes(frame)) > 100, "multicolored drawing missing")
        return "guest-drawn multicolor circles and rectangles"
    if case in ("battery", "imu", "ltr553"):
        _, _, alternative = capture(root, case + "-alternate", (320, 240))
        _, _, control = capture(root, case + "-control", (320, 240))
        require(min(changed(frame, control), changed(alternative, control), changed(frame, alternative)) > 30,
                "one injected value did not alter the guest screen")
        if case == "battery":
            text(frame, (0, 48, 240, 120), "7.62 V and 129.97 mA", "e93f13dde839f8a287a453e75")
            text(alternative, (0, 48, 240, 120), "6.85 V and 289.93 mA", "7c85424470ab65c2de7c928f")
            return "two injected INA226 voltage and load readings on LCD"
        if case == "imu":
            text(frame, (0, 20, 170, 145), "first accelerometer/gyro vector", "afbe3ee6116ccd2672185363")
            text(alternative, (0, 20, 170, 145), "second accelerometer/gyro vector", "a53103a133ed153aaf81b17b")
            return "two distinct BMI270 accelerometer and gyro readings on LCD"
        text(frame, (25, 106, 290, 133), "PS:321 / ALS:750", "f25b27bf7a5e7d83dbf2bc10")
        text(alternative, (25, 106, 290, 133), "PS:1456 / ALS:2185", "2373668e3d4544cf6837e1f1")
        return "two distinct LTR553 ranges and a no-stimulus control"
    if case == "button":
        _, _, control = capture(root, case + "-control", (320, 240))
        text(frame, (85, 75, 235, 108), "Btn PWR in red", "41f349b8c87d6799dcadc65e")
        require(changed(frame, control) > 100, "no change after pressing PMIC button")
        return "AXP2101 PMIC key produces rendered Btn PWR"
    if case == "rtc":
        require("state Associated" in log and re.search(r"[1-9]\d* NTP answers", log),
                "virtual AP association or NTP exchange missing")
        require(len(modes(frame)) > 1, "RTC display did not render")
        return "guest associates with virtual AP, consumes NTP and draws clock"
    if case == "camera":
        _, alt_log, alt = capture(root, case + "-alternate", (320, 240))
        require(changed(frame, alt) > 1000 and all(re.search(r"camera: [1-9]\d* VSYNCs, 0 frames with dropped data", report)
                                                       for report in (log, alt_log)), "missing camera DMA delivery or distinct frames")
        for label, captured in ((case, frame), (case + "-alternate", alt)):
            require(captured == bmp_mirrored_rgb565((root / f"{label}-input.bmp").read_bytes()),
                    f"{label} framebuffer differs from expected mirrored RGB565")
        return "two distinct camera frames with VSYNC and zero dropped data"
    if case == "servo-2":
        angles = re.findall(r"StackChan servo-angle: yaw=(-?[\d.]+) pitch=(-?[\d.]+)", log)
        require(angles == [("0.000", "45.000"), ("90.000", "45.000")] and field(log, "servo-reads") > 0,
                f"expected two guest-driven angles and servo responses, got {angles}")
        return "top touch drives 45° pitch and then 90° yaw using serial servos"
    if case == "servo-1":
        require(field(log, "servo-reads") > 0 and field(log, "servo-writes") > 0 and "servo-angle:" in log,
                "missing SCSCL commands or feedback")
        return None
    if case == "ir_nec":
        require(field(log, "ir") >= 2 and "Send NEC: addr=0x0000, cmd=0x55" in serial and
                "Send NEC: addr=0x0001, cmd=0x56" in serial, "missing two decoded IR transmissions")
        return "GPIO carrier delivered two valid NEC frames"
    if case == "nfc-1":
        _, baseline, _ = capture(root, case + "-control", (320, 240))
        require(field(log, "nfc-selections") >= 2 and field(baseline, "nfc-selections") == 0 and
                "Failed to begin" not in serial, "missing tag selection or no-tag control")
        return "guest completes NFC-A UID selection with tag present and silent no-tag control"
    if case in ("sdcard", "pic"):
        disk = root / f"sd-{case}.img"
        stem = "cores3_test_picture" if case == "sdcard" else "TestPicture"
        names = [f"{stem}01.png", f"{stem}02.png"]
        if case == "sdcard":
            names.append("WriteTest.txt")
        files = fat32_files(disk, names)
        if case == "sdcard":
            require(files.get("WriteTest.txt") == b"Hello, world! \nSD card write success! \n",
                    "guest did not write the expected test file")
        matches = []
        for name in names[:2]:
            fixture = root / name
            fixture.write_bytes(files[name])
            matches.append(frame == png(fixture, 320, 240))
        require(any(matches), "LCD does not match either documented SD image")
        return "file reads are visible in the framebuffer and use SPI SD"
    if case in ("speaker", "wav", "mic"):
        _, tones = audio(root / f"{case}.wav")
        require(tones, "audio output contains no non-silent PCM")
        if case == "speaker":
            require(any(abs(tone[3] - 10_000) < 60 for tone in tones) and
                    any(abs(tone[3] - 4000) < 60 for tone in tones), f"documented tones missing: {tones}")
            return "I²S speaker emitted both documented frequency bands"
        if case == "wav":
            require("Segmented playback completed!" in serial and any(abs(tone[3] - 440) < 4 for tone in tones)
                    and any(abs(tone[3] - 660) < 4 for tone in tones), "fixture completion or frequencies missing")
            return "SD WAV decoded and played at both fixture frequencies"
        return None
    if case == "wakeup":
        _, asleep, blank = capture(root, case + "-sleep", (320, 240))
        require(not any(blank) and "powered=false" in asleep and "on_cause=0x02" in log and
                log.count("chip reset at") == 1 and len(modes(frame)) > 1, "RTC wake did not restore a powered display")
        return "display sleeps and resumes once following RTC power wake"
    return None


def check_matrix(board, root, cases, selected=None):
    size = (240, 135) if board == "stamplc" else (320, 240)
    verdicts = {}
    items = [(row["case"], row) for row in cases] + [("stock", {})]
    for case, row in items:
        if selected and case not in selected:
            continue
        if board == "stamplc" and case == "display" and not (root / f"{case}.json").exists():
            result = ("DOCS-DEFECT", "Pinned display.md line 7 snippet calls Display as a field")
        elif not (root / f"{case}.json").exists():
            result = ("NOT RUN", "no receipt for this firmware")
        else:
            try:
                serial, log, frame = capture(root, case, size)
                if case == "stock":
                    require(len(modes(frame)) > 1, "stock framebuffer remained blank")
                    require(re.search(r"(?m)^S3C_REPL_OK\r?$", serial), "stock REPL marker missing")
                    reason = "stock displayed a frame and executed the REPL probe"
                else:
                    reason = (plc if board == "stamplc" else stack)(root, case, serial, log, frame)
                result = ("PASS", reason) if reason else ("NOT RUN", "guest executed, behavioral assertion pending")
            except (OSError, KeyError, ValueError, struct.error, zlib.error) as error:
                result = ("FAIL", str(error))
        verdicts[case] = {"status": result[0], "reason": result[1], "expected": row.get("expected")}
        print(f"{case:17} {result[0]:11} {result[1]}")
    (root / "verdicts.json").write_text(json.dumps(verdicts, indent=2) + "\n")
    return int(any(item["status"] in ("FAIL", "NOT RUN") for item in verdicts.values()))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("board", choices=("stamplc", "stackchan"))
    parser.add_argument("out", type=Path)
    parser.add_argument("inventory", type=Path)
    args = parser.parse_args()
    cases = [row for row in json.loads(args.inventory.read_text())["cases"] if row["device"] == args.board]
    raise SystemExit(check_matrix(args.board, args.out, cases))
