#!/usr/bin/env python3
"""Check small-display S3 boards against the captured Arduino and UIFlow2 evidence."""

import argparse
from array import array
from collections import Counter
import io
from pathlib import Path
import re
import shlex
import struct
import sys
import wave
import zlib

from check_cores3 import changed, color_counts, fat32_files, frequency, require, rms

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
# The CLI enlarges panels narrower than 200 pixels for its PNG output.
SIZE = {"cardputer": (240, 135), "dial": (240, 240), "sticks3": (405, 720), "stopwatch": (466, 466)}
FAULT = re.compile(r"Guru Meditation|Interrupt wdt timeout|assert failed|Backtrace:|stack smashing", re.I)


class DocsDefect(Exception):
    """The pinned sketch contradicts its documented behavior."""


def frame_pixels(path, size):
    """Read the simulator's filter-0 RGB8 display export."""
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
    command = shlex.split((out / f"{label}.command").read_text())
    seconds = float(command[command.index("--max-seconds") + 1])
    stop = re.search(r"\[emu\] stop: Halted .*?emulated (\d+\.\d+)s", err)
    require(stop and float(stop[1]) == seconds, "virtual time budget did not complete")
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


def wav_pcm(source):
    with wave.open(source, "rb") as wav:
        require((wav.getnchannels(), wav.getsampwidth()) == (1, 2), "expected mono PCM16")
        rate = wav.getframerate()
        pcm = array("h", wav.readframes(wav.getnframes()))
    if sys.byteorder != "little":
        pcm.byteswap()
    return rate, pcm


def heard_tone(path, hz, tolerance, window_ms):
    rate, pcm = wav_pcm(str(path))
    width = rate * window_ms // 1000
    for start in range(0, len(pcm) - width + 1, width):
        segment = pcm[start:start + width]
        if rms(segment) > 100 and abs(frequency(segment, rate) - hz) < tolerance:
            return True
    return False


def power_case(out, board, case, serial, report):
    if board == "sticks3":
        wire_lines = {"m5pm1-1": 16, "m5pm1-2": 18, "m5pm1-3": 19, "wakeup-2": 16}
        if (case in wire_lines and "PM1 initialization successful" in serial and "unexpected nack" in serial
                and re.search(r"L7set_pin\w*\(a2=0x1 a3=0x2f a4=0x30\)", report)
                and "[i2c0] transfer without a bidirectional pin route" in report
                and "[peek after run]\n60004610: 0000005c\n60004614: 0000005b" in report):
            name = case.replace("-", "_")
            raise DocsDefect(f"{name}.ino:{wire_lines[case]} assigns Wire/I2C0 to GPIO47/48. "
                             "M5.update polls PM1 through M5GFX I2C1, which rewrites the same GPIO output selectors. "
                             "The later Wire operation fails after the I2C1 remap.")
        if (case == "wakeup-1" and "esp_sleep_enable_timer_wakeup(a2=0x4c4b40" in report
                and "M5PM1_Class8powerOffEv(" in report and "powered=false timer_wakes=0" in report
                and "[i2c] start addr 0x6e read=false\n[i2c]   write 0x0c\n[i2c]   write 0xa1" in report
                and "esp_deep_sleep_start: 1 functions" in report and "esp_deep_sleep_start(" not in report
                and not re.search(r"addr 0x6e read=false\n\[i2c\]   write 0x3[89abcd]\n\[i2c\]   write", report)):
            raise DocsDefect("wakeup_1.ino:18 calls timerSleep(5). The pinned library arms the ESP RTC timer "
                             "then cuts the ESP's supply through PM1 before entering deep sleep. No PM1 timer was armed.")
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
        contains(report, "ext_uart=true", "ext_usb=true")
        require(re.search(r"vibration_duties=\{[^}]*\b2047\b", report), "IOE1 did not hold its programmed motor duty")
        wav = out / (case + "-output.wav")
        require(all(heard_tone(wav, hz, 180, 10) for hz in (4000, 6000)), "audible phases missing")
        if heard_tone(wav, 10000, 180, 10):
            callbacks = re.split(r"\[fn\] [^\n]*_speaker_enabled_cb_stopwatch\w*\([^\n]*a3=(0x[01])[^\n]*\n", report)
            expander_write = "[i2c] start addr 0x4f read=false\n[i2c]   write 0x"
            require(any(enabled == "0x1" and expander_write + "05\n[i2c]   write 0x9d" in writes
                        and expander_write + "06\n[i2c]   write 0x03" in writes
                        for enabled, writes in zip(callbacks[1::2], callbacks[2::2])),
                    "unexpected 10 kHz PCM without traced audio rail and amplifier restoration")
            raise DocsDefect("m5pm1_m5ioe1_6.ino:75–78 and :107–113 call tone() and begin() after turning off "
                             "the amplifier or audio rail. The traced enable callback restores both before "
                             "the 10 kHz PCM reaches the speaker. Connector selection and motor duty passed.")
        return "IOE1 selects UART0 and USB, drives PWM1, and gates both audio paths"
    require(case in {"wakeup", "wakeup-1"}, "missing power-case oracle")
    contains(report, "chip reset")
    if board == "dial":
        require(re.search(r"wakeups=[1-9]", report), "RTC did not restore hold power")
    return "button-triggered sleep followed by timer wake and guest reset"


def verify(out, board, case):
    allow_blank = case in {"buzzer", "speaker", "speaker-sd-wav", "wakeup-1", "wakeup-2", "rfid-1", "rfid-2",
                           "m5pm1-1", "m5pm1-2", "m5pm1_m5ioe1-1", "m5pm1_m5ioe1-2"}
    frame, serial, report = capture(out, board, case, screen=not allow_blank and not (case == "button" and board != "sticks3"))
    if case == "display" or case.startswith("quickstart-"):
        early, *_ = capture(out, board, case + "-early", screen=False)
        minimum_colors = 4 if case == "display" else 8
        require(len(color_counts(frame)) >= minimum_colors and changed(frame, early) >= 150, "missing animated display")
        return "display contains multicolor drawing and changes over time"
    if case == "touch-2":
        idle, *_ = capture(out, board, case + "-idle")
        first, *_ = capture(out, board, case + "-first")
        outside, *_ = capture(out, board, case + "-outside")
        require(first == frame and outside == idle and changed(frame, idle) >= 5,
                "the image does not agree with the two button hits and the outside control")
        return "both touch targets inside the button display Pressed, with idle and outside controls Released"
    if case in {"battery", "imu", "touch", "touch-1"}:
        return sensors(out, board, case, frame)
    if case == "button":
        presses = {
            "cardputer": ("Button A Pressed", "Button A Released"),
            "dial": ("Button A Pressed", "Button A Released"),
            "sticks3": ("BtnA", "BtnB"),
            "stopwatch": ("BtnA Pressed", "BtnB Pressed"),
        }
        if board == "sticks3":
            return sensors(out, board, case, frame)
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
        if case.endswith("-2"):
            contains(serial, "Raw: 0xCB34EF10", "Raw: 0xA857D52A")
            return "both guest NEC payloads match the input address and command"
        raw_pulses = re.findall(r"IR PULSE \(us\):\(L0 \| L1\)\s*((?:\s*-\d+, \d+\n)+)", serial)
        payloads = []
        for block in raw_pulses:
            pairs = [(int(mark), int(space)) for mark, space in re.findall(r"(-\d+), (\d+)", block)]
            require(len(pairs) == 34 and pairs[0] == (-9000, 4500), "unexpected receive symbol layout")
            payloads.append(sum((space > 1000) << bit for bit, (_, space) in enumerate(pairs[1:33])))
        require(payloads == [0xCB34EF10, 0xA857D52A] and serial.count("Transmit back OK!") == 2,
                "two receive/echo payloads missing")
        contains(report, "RMT46_active_low=2", "last_nec=Some((42, 87))")
        return "both payloads received and echoed as active-low RMT symbols on GPIO46"
    if case in {"rfid-1", "rfid-2"}:
        _, idle, _ = capture(out, board, case + "-idle", screen=False)
        contains(serial, "04 01 02 03", "04 09 08 07")
        require("PICC type:" not in idle and "MIFARE_Read() failed" not in serial
                and "MIFARE_Write() failed" not in serial, "negative control or virtual tag transfer failed")
        if case == "rfid-2":
            contains(serial, "Number of bytes that match = 16", "Success :-)")
        return "separate virtual UIDs read through I2C FIFO and no UID without a tag"
    if case in {"speaker", "buzzer"}:
        expected = (10000, 4000) if board in {"dial", "stopwatch"} else (7000, 4000)
        for hz in expected:
            require(heard_tone(out / f"{case}-output.wav", hz, 180, 10), f"missing {hz} Hz in captured audio")
        route = "GPIO3 PDM output" if board == "dial" else "I2S output"
        return f"{route} contains {expected[0]} Hz and {expected[1]} Hz tones"
    if case == "mic":
        require(heard_tone(out / "mic-output.wav", 733, 20, 80), "733 Hz record/playback marker missing")
        if board == "cardputer":
            first, *_ = capture(out, board, "mic-tone")
            second, *_ = capture(out, board, "mic-alt")
            silent, *_ = capture(out, board, "mic-silent")
            require(min(changed(first, second), changed(first, silent), changed(second, silent)) >= 25,
                    "recording waveform did not respond to both input tones")
            return "two rendered capture signals differ from silence and 733 Hz reaches speaker"
        capture(out, board, "mic-alt")
        capture(out, board, "mic-silent")
        require(heard_tone(out / "mic-alt-output.wav", 997, 20, 80), "997 Hz record/playback marker missing")
        _, silence = wav_pcm(str(out / "mic-silent-output.wav"))
        require(rms(silence) < 100, "silent recording produced unintended audio")
        return "733 Hz and 997 Hz both reach the speaker and the silent control stays quiet"
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
        source = fat32_files(out / "sdcard.img", ("sample-12s.wav",))["sample-12s.wav"]
        source_hz, source_pcm = wav_pcm(io.BytesIO(source))
        rate, pcm = wav_pcm(str(out / "speaker-sd-wav-output.wav"))
        require(rate == 48000 and len(pcm) >= 12 * rate and len(source_pcm) == 12 * source_hz,
                "WAV playback is incomplete")
        for second in range(12):
            start, end = second + 0.25, second + 0.75
            original = source_pcm[int(start * source_hz):int(end * source_hz)]
            reproduced = pcm[int(start * rate):int(end * rate)]
            expected = 440 if second % 2 == 0 else 660
            require(abs(frequency(original, source_hz) - expected) < 5
                    and abs(frequency(reproduced, rate) - expected) < 5 and rms(reproduced) > 100,
                    f"wrong SD WAV playback during second {second}")
        return "all 12 one-second 440/660 Hz intervals agree across SPI SD and I2S"
    if case in {"rtc", "rtc-1", "rtc-2"}:
        early, *_ = capture(out, board, case + "-early")
        require(changed(frame, early) >= 5, "calendar display did not advance")
        if case != "rtc-1":
            require(re.search(r"[1-9]\d* NTP answers", report) or "NTP Connected" in serial,
                    "virtual SNTP exchange not observed")
        else:
            contains(serial, "RTC found.", "UTC:")
        return "RTC calendar advances in the guest"
    if case == "vibration":
        duties = re.search(r"vibration_duties=\{([^}]*)\}", report)
        expected = {level * 4095 // 255 for level in (10, 40, 70, 100, 130, 160, 190, 210, 240)}
        require(duties and {int(duty) for duty in duties[1].split(", ") if duty} == expected,
                "IOE1 PWM1 duty steps differ")
        return "all nine vibrator PWM1 duties observed"
    return power_case(out, board, case, serial, report)


def verify_stock(out, board):
    home, *_ = capture(out, board, "stock-home")
    active, *_ = capture(out, board, "stock-input")
    _, serial, report = capture(out, board, "stock-repl", screen=False)
    require(re.search(r"(?<!\d)7303(?!\d)", serial + report), "no USB REPL result")
    if board == "dial":
        require(re.findall(r"^DIAL BUTTON ([01])\s*$", serial, re.M) == ["1", "0", "1"],
                "stock guest did not read the rotary button's idle, down and released levels")
        return "home frame and USB REPL arithmetic, with the rotary button read by the stock guest"
    require(changed(home, active) > 50, "stock UI did not respond to control input")
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
            except DocsDefect as exc:
                verdict, detail = "DOCS-DEFECT", str(exc)
            except (OSError, ValueError, wave.Error, EOFError, struct.error, zlib.error) as exc:
                verdict, detail = "FAIL", str(exc)
        totals[verdict] += 1
        print(f"| {case} | {verdict} | {detail.replace('|', '/')} |")
    return 0 if totals["PASS"] == sum(totals.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
