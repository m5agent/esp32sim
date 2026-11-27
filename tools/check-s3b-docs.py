#!/usr/bin/env python3
"""Check small-display S3 boards against the captured Arduino and UIFlow2 evidence."""

import argparse
from collections import Counter
from pathlib import Path
import re
import struct
import wave
import zlib

from check_cores3 import audio, changed, color_counts, fat32_files, require

CASES = {
    "cardputer": """battery button display ir_nec keyboard mic quickstart-display
                    sdcard speaker speaker-sd-wav""",
    "dial": """button buzzer display encoder quickstart-display rfid-1 rfid-2
               rtc touch wakeup""",
    "sticks3": """battery button display imu ir_nec-1 ir_nec-2 ir_nec-3 m5pm1-1
                  m5pm1-2 m5pm1-3 mic quickstart-scrollgraph speaker wakeup-1 wakeup-2""",
    "stopwatch": """battery button display imu m5pm1_m5ioe1-1 m5pm1_m5ioe1-2
                    m5pm1_m5ioe1-3 m5pm1_m5ioe1-4 m5pm1_m5ioe1-5 m5pm1_m5ioe1-6
                    mic quickstart-bargraph rtc-1 rtc-2 speaker touch-1 touch-2 vibration""",
}
SIZE = {"cardputer": (240, 135), "dial": (240, 240), "sticks3": (135, 240), "stopwatch": (468, 466)}
FAULT = re.compile(r"Guru Meditation|Interrupt wdt timeout|assert failed|Backtrace:|stack smashing", re.I)


def frame_pixels(path, size):
    """Read the simulator's filter-0 RGB8 PNG at the board's physical size."""
    data = path.read_bytes()
    require(data.startswith(b"\x89PNG\r\n\x1a\n"), "invalid PNG")
    cursor = 8
    packed = bytearray()
    header = None
    complete = False
    while cursor + 12 <= len(data):
        length, kind = struct.unpack_from(">I4s", data, cursor)
        start = cursor + 8
        body = data[start:start + length]
        checksum, = struct.unpack_from(">I", data, start + length)
        require(zlib.crc32(kind + body) == checksum, "PNG CRC mismatch")
        cursor = start + length + 4
        if kind == b"IHDR":
            header = struct.unpack(">IIBBBBB", body)
        elif kind == b"IDAT":
            packed.extend(body)
        elif kind == b"IEND":
            complete = True
            break
    require(complete and header == (*size, 8, 2, 0, 0, 0), f"expected RGB8 PNG at {size}")
    lines = zlib.decompress(packed)
    stride = 1 + 3 * size[0]
    require(len(lines) == stride * size[1] and set(lines[::stride]) == {0}, "unexpected PNG scanline data")
    return b"".join(lines[i + 1:i + stride] for i in range(0, len(lines), stride))


def capture(out, board, label, screen=True):
    status = (out / f"{label}.status").read_text().strip()
    err = (out / f"{label}.err").read_text(errors="replace")
    serial = (out / f"{label}.log").read_text(errors="replace")
    require(status == "0", f"emulator exit {status}")
    require("[emu] stop: Halted" in err and "emulated " in err, "virtual time budget did not complete")
    require(not FAULT.search(serial), "guest crashed")
    frame = frame_pixels(out / f"{label}.png", SIZE[board])
    if screen:
        require(len(color_counts(frame)) >= 2, "guest framebuffer stayed blank or uniform")
    return frame, serial, err


def contains(text, *markers):
    for marker in markers:
        require(marker in text, f"missing {marker}")


def sensors(out, board, case, frame):
    idle, *_ = capture(out, board, case + "-idle")
    first, *_ = capture(out, board, case + "-first")
    require(changed(first, idle) >= 5 and changed(frame, first) >= 5
            and changed(frame, idle) >= 5, "sensor inputs did not produce three distinct rendered states")
    return "idle and both sensor inputs produced distinct states"


def power_case(board, case, serial, report):
    if case in {"m5pm1-1", "m5pm1_m5ioe1-1", "wakeup-2"}:
        contains(serial, "PM1 initialization successful")
        require(re.search(r"timer_wakes=[1-9]", report), "missing PM1 timer-driven power return")
        contains(report, "chip reset")
        return "PM1 timer powers the SoC back on and resets the guest"
    if case in {"m5pm1-2", "m5pm1_m5ioe1-2"}:
        contains(serial, "BMI270 AnyMotionInterrupt enabled successfully")
        contains(report, "chip reset")
        return "motion input wakes the guest after standby"
    if case in {"m5pm1-3", "m5pm1_m5ioe1-3", "m5pm1_m5ioe1-5"}:
        contains(serial, "PM1 IRQ triggered")
        return "guest observes PM1 IRQ"
    if case == "m5pm1_m5ioe1-4":
        contains(serial, "RTC IRQ enabled successfully")
        contains(report, "chip reset")
        return "RTC IRQ wakes the guest after standby"
    if case == "m5pm1_m5ioe1-6":
        contains(serial, "PM1 initialization successful", "IOE1 initialization successful")
        require("oled_transitions=" in report and re.search(r"oled_transitions=[3-9]", report),
                "missing observable OLED rail changes from IOE1")
        return "IOE1 OLED power transitions follow A/B"
    require(case in {"wakeup", "wakeup-1"}, "missing power-case oracle")
    contains(report, "chip reset")
    if board == "dial":
        require(re.search(r"wakeups=[1-9]", report), "RTC did not restore hold power")
    return "button-triggered sleep followed by timer wake and guest reset"


def verify(out, board, case):
    allow_blank = case in {"buzzer", "speaker", "speaker-sd-wav", "wakeup-2",
                           "m5pm1-1", "m5pm1-2", "m5pm1_m5ioe1-1", "m5pm1_m5ioe1-2"}
    frame, serial, report = capture(out, board, case, screen=not allow_blank and not (case == "button" and board == "cardputer"))
    if case == "display" or case.startswith("quickstart-"):
        early, *_ = capture(out, board, case + "-early")
        require(len(color_counts(frame)) >= 8 and changed(frame, early) >= 150, "missing animated display")
        return "display contains multicolor drawing and changes over time"
    if case in {"battery", "imu", "touch", "touch-1", "touch-2"}:
        return sensors(out, board, case, frame)
    if case == "button":
        presses = {
            "cardputer": ("Button A Pressed", "Button A Released"),
            "dial": ("Button A Pressed", "Button A Released"),
            "sticks3": ("BtnA", "BtnB"),
            "stopwatch": ("BtnA Pressed", "BtnB Pressed"),
        }
        if board == "sticks3":
            idle, *_ = capture(out, board, case + "-idle")
            require(changed(frame, idle) >= 30, "button-driven display did not change")
            return "A/B/P inputs change the display"
        contains(serial, *presses[board])
        return "guest reports press/release or both buttons"
    if case in {"keyboard", "encoder"}:
        idle, *_ = capture(out, board, case + "-idle")
        require(changed(frame, idle) >= 15, "input did not change guest UI")
        contains(report, "Board(\"key\"" if case == "keyboard" else "Gpio(41")
        return "guest UI changes after physical matrix or quadrature inputs"
    if case == "ir_nec":
        pairs = re.findall(r"address=0x(\w+)/command=0x(\w+)", report)
        require(len(set(pairs)) >= 2 and all(a == "1111" for a, _ in pairs), "missing GPIO44 NEC payloads")
        return "GPIO44 emits changing commands addressed to 0x1111"
    if case == "ir_nec-1":
        require(re.search(r"RMT46_frames=[1-9]", report), "no decoded NEC frames on GPIO46")
        contains(serial, "Send NEC: addr=0x0000", "Send NEC: addr=0x0001")
        return "guest serial and RMT46 GPIO data show NEC transmission"
    if case in {"ir_nec-2", "ir_nec-3"}:
        idle_frame, idle, _ = capture(out, board, case + "-idle")
        require(changed(frame, idle_frame) >= 15 and serial != idle, "IR input did not change guest output")
        require(re.search(r"0x0*10\b", serial, re.I) and re.search(r"0x0*2a\b", serial, re.I),
                "two received NEC addresses missing")
        if case.endswith("-3"):
            require(re.search(r"RMT46_frames=[1-9]", report), "no NEC frame relayed on GPIO46")
        return "two NEC inputs reached the receive path"
    if case in {"rfid-1", "rfid-2"}:
        _, idle, _ = capture(out, board, case + "-idle")
        contains(serial, "04 01 02 03", "04 09 08 07")
        require("PICC type:" not in idle and "MIFARE_Read() failed" not in serial
                and "MIFARE_Write() failed" not in serial, "negative control or virtual tag transfer failed")
        if case == "rfid-2":
            contains(serial, "Data was written")
        return "separate virtual UIDs read through I2C FIFO and no UID without a tag"
    if case in {"speaker", "buzzer"}:
        expected = (10000, 4000) if board in {"dial", "stopwatch"} else (7000, 4000)
        _, tones = audio(out / f"{case}-output.wav")
        for hz in expected:
            require(any(abs(t[3] - hz) < 150 for t in tones), f"missing {hz} Hz in captured audio")
        return f"I2S data contains {expected[0]} Hz and {expected[1]} Hz tones"
    if case == "mic":
        first, *_ = capture(out, board, "mic-tone")
        second, *_ = capture(out, board, "mic-alt")
        silent, *_ = capture(out, board, "mic-silent")
        require(min(changed(first, second), changed(first, silent), changed(second, silent)) >= 25,
                "recording waveform did not respond to both input tones")
        _, tones = audio(out / "mic-output.wav")
        require(any(abs(t[3] - 733) < 20 and t[1] > 0.2 for t in tones), "733 Hz record/playback marker missing")
        return "two capture signals differ from silence and 733 Hz playback reaches speaker"
    if case == "sdcard":
        contains(serial, "File written", "Message appended", "Hello World!", "File renamed", "1048576 bytes written")
        require(re.search(r"\[sd-spi\] \d+ commands, [1-9]\d* blocks read, [1-9]\d* blocks written", report),
                "missing bidirectional SPI SD data")
        files = fat32_files(out / "sdcard.img", ("FOO.TXT", "TEST.TXT"))
        require(files == {"FOO.TXT": b"Hello World!\n", "TEST.TXT": bytes(1048576)}, "FAT32 contents differ")
        return "SPI read and write verified by serial, sector counters and persisted FAT32 bytes"
    if case == "speaker-sd-wav":
        require(re.search(r"\[sd-spi\] \d+ commands, [1-9]\d* blocks read, 0 blocks written", report),
                "missing SPI SD reads")
        samples, runs = audio(out / "speaker-sd-wav-output.wav")
        require(samples / 48000 >= 11.9 and runs, "WAV playback is incomplete or silent")
        return "12-second media played via SPI SD and I2S"
    if case in {"rtc", "rtc-1", "rtc-2"}:
        early, *_ = capture(out, board, case + "-early")
        require(changed(frame, early) >= 5, "calendar display did not advance")
        if case != "rtc-1":
            require("SNTP" in report or "NTP Connected" in serial, "virtual SNTP exchange not observed")
        else:
            contains(serial, "RTC found.", "UTC:")
        return "RTC calendar advances in the guest"
    if case == "vibration":
        duties = re.search(r"vibration_duties=\{([^}]*)\}", report)
        expected = {level * 4095 // 255 for level in (10, 40, 70, 100, 130, 160, 190, 210, 240)}
        require(duties and {int(duty) for duty in duties[1].split(", ") if duty} == expected,
                "IOE1 PWM1 duty steps differ")
        return "all nine vibrator PWM1 duties observed"
    return power_case(board, case, serial, report)


def verify_stock(out, board):
    home, *_ = capture(out, board, "stock-home")
    active, *_ = capture(out, board, "stock-input")
    require(changed(home, active) > 50, "stock UI did not respond to control input")
    _, serial, report = capture(out, board, "stock-repl", screen=False)
    require(re.search(r"(?<!\d)7303(?!\d)", serial + report), "no USB REPL result")
    return "home frame, input-driven state change, and USB REPL arithmetic result"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("board", choices=CASES)
    parser.add_argument("out", type=Path)
    args = parser.parse_args()
    board = args.board
    print("| Case | Verdict | Evidence |")
    print("| --- | --- | --- |")
    totals = Counter()
    for case in CASES[board].split() + ["UIFlow2"]:
        marker = "stock-home" if case == "UIFlow2" else case
        if not (args.out / f"{marker}.status").exists():
            verdict, detail = "NOT RUN", "no completed run receipt"
        else:
            try:
                detail = verify_stock(args.out, board) if case == "UIFlow2" else verify(args.out, board, case)
                verdict = "PASS"
            except (OSError, ValueError, wave.Error, EOFError, struct.error, zlib.error) as exc:
                verdict, detail = "FAIL", str(exc)
        totals[verdict] += 1
        print(f"| {case} | {verdict} | {detail.replace('|', '/')} |")
    return 0 if totals["PASS"] == sum(totals.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
