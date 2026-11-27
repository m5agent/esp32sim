#!/usr/bin/env python3
"""Check the e-paper guest receipts, frames, and input controls."""
import argparse
from collections import Counter
from hashlib import sha256
import json
from importlib.util import spec_from_file_location, module_from_spec
from math import cos, pi, sin
from pathlib import Path
import re
import struct
import wave

from check_cores3 import color_counts, fat32_files, png_rgb, require, rms

spec = spec_from_file_location("runner", Path(__file__).with_name("run-paper-docs.py"))
runner = module_from_spec(spec)
spec.loader.exec_module(runner)

def digest(pixels, width, box=None):
    if box:
        x0, y0, x1, y1 = box
        pixels = b"".join(pixels[3 * (y * width + x0):3 * (y * width + x1)] for y in range(y0, y1))
    return sha256(pixels).hexdigest()[:24]


def loop_times(trace):
    return [float(t) for t in re.findall(r"(?m)^\[fn\].*? t=([\d.]+)s .*? _Z4loopv\(", trace or "")]


def i2c_transactions(log, address):
    for transaction in re.split(r"(?m)^\[i2c\] start addr ", log or "")[1:]:
        match = re.match(r"(0x[\da-f]+) read=(true|false)", transaction)
        if match and int(match[1], 16) == address:
            direction = "read" if match[2] == "true" else "write"
            data = re.findall(rf"(?m)^\[i2c\]\s+{direction}\s+(0x[\da-f]+)", transaction)
            yield direction, tuple(int(value, 16) for value in data)


def register_reads(log, address, register):
    pointer = None
    for direction, data in i2c_transactions(log, address):
        if direction == "write":
            pointer = data[0] if data else None
        elif pointer == register:
            yield data


def vcd_edges(path, pin):
    code = None
    instant = 0
    with path.open() as capture:
        for line in capture:
            if line.startswith("$var "):
                _, _, _, wire, name, _ = line.split()
                if name == f"gpio{pin}":
                    code = wire
            elif line.startswith("#"):
                instant = int(line[1:])
            elif code and line[0] in "01" and line[1:].strip() == code:
                yield instant / 1_000_000, line[0] == "1"  # microseconds


def nec_frames(path):
    # Arduino-IRremote software carrier. Consecutive marks in a burst
    # are separated by at most one 38 kHz period.
    rises = [time for time, high in vcd_edges(path, 48) if high]
    bursts = []
    for instant in rises:
        if not bursts or instant - bursts[-1][1] > 45:
            bursts.append([instant, instant])
        else:
            bursts[-1][1] = instant
    for index, (start, end) in enumerate(bursts[:-33]):
        if not 8500 <= end - start <= 9400:
            continue
        if not 4300 <= bursts[index + 1][0] - end <= 4900:
            continue
        bits = []
        for bit in range(index + 1, index + 33):
            first, last = bursts[bit]
            gap = bursts[bit + 1][0] - last
            if not 440 <= last - first <= 660:
                break
            if 430 <= gap <= 750:
                bits.append(0)
            elif 1500 <= gap <= 1900:
                bits.append(1)
            else:
                break
        if len(bits) == 32:
            frame = sum(bit << index for index, bit in enumerate(bits))
            address, command, inverse = frame & 0xffff, (frame >> 16) & 255, frame >> 24
            if command ^ inverse == 255:
                yield address, command


def pcm(path):
    with wave.open(str(path), "rb") as wav:
        rate, count = wav.getframerate(), wav.getnframes()
        require(wav.getnchannels() == 1 and wav.getsampwidth() == 2, "Expected signed mono PCM16")
        return rate, struct.unpack(f"<{count}h", wav.readframes(count))


def spectrum(samples, rate, hz):
    # Scale by RMS so playback volume cannot substitute for frequency.
    energy = rms(samples)
    if not energy:
        return 0
    angle = 2 * pi * hz / rate
    real = sum(value * cos(index * angle) for index, value in enumerate(samples))
    imag = sum(value * sin(index * angle) for index, value in enumerate(samples))
    return abs(complex(real, imag)) / (len(samples) * energy)


def check_tones(board, case, path):
    rate, samples = pcm(path)

    def window(start, end):
        return samples[round(start * rate):round(end * rate)]

    if board in {"papermono", "papercolor"}:
        notes = [(0.04, 0.20, 4000), (2.04, 2.19, 8000), (4.04, 4.19, 4000)]
        rests = [(0.28, 1.98), (2.28, 3.98)]
        threshold = 0.20
    elif case == "buzzer-1":
        notes = [(i * 0.5 + 0.1, i * 0.5 + 0.4, hz)
                 for i, hz in enumerate((880, 990, 1120, 1180, 1320, 1480, 1650, 1760))]
        rests = [(4.13, 4.43), (8.64, 8.93)]
        threshold = 0.20
    else:
        phrasing = [(1320, 0.3), (1320, 0.3), (1480, 0.6), (1320, 0.4),
                    (0, 0.2), (1760, 0.6), (1650, 0.6), (0, 0.3)]
        start, notes, rests = 0, [], []
        for hz, length in phrasing:
            if hz:
                notes.append((start + 0.07, start + length - 0.07, hz))
            else:
                rests.append((start + 0.04, start + length - 0.04))
            start += length + (0.03 if hz else 0)
        threshold = 0.20
    readings = [(hz, round(spectrum(window(a, b), rate, hz), 3)) for a, b, hz in notes]
    require(all(strength >= threshold for _, strength in readings),
            f"Documented tones not recovered from PCM ({readings})")
    require(all(window(a, b) and rms(window(a, b)) < 15 for a, b in rests),
            "PCM does not contain the documented silent gaps")
    return f"PCM contains {len(notes)} ordered tones and {len(rests)} silent gaps"


def check_rtc(log, control, pixels, control_frame):
    require(control is not None and control_frame is not None and pixels is not None,
            "Missing the disconnected RTC control")
    require("state Associated" in log and re.search(r"\b[1-9]\d* NTP answers", log),
            "No guest AP association and NTP answer")
    require(not re.search(r"\b[1-9]\d* NTP answers", control), "Disconnected control received NTP")
    calendar = [value for value in register_reads(log, 0x51, 2)
                if len(value) >= 7 and value[5] & 31 == 5 and value[6] == 0x26]
    require(len({value[0] & 127 for value in calendar}) >= 2,
            "RTC bus did not read an advancing May 2026 calendar after NTP")
    require(pixels != control_frame, "Clock screen still matches the disconnected control")
    return "Sandbox NTP, BM8563 calendar progression, and a changed clock screen"


def check_papers3(case, log, pixels, control, control_frame, first_frame, out):
    if case in {"battery", "imu"}:
        require(all((pixels, first_frame, control_frame)), "Missing two input frames and the control")
        # Crops read the battery measurements and IMU axes from the documented screens.
        box, expected = {
            "battery": ((0, 315, 540, 790), ("3d9ccae69c6ed7f03108aaab", "88da6b4d57466a32d9d5e45c",
                                              "3724149fa0aa8ff614c18b88")),
            "imu": ((0, 0, 540, 550), ("5ff3ec640d9ab9c5f3497ef7", "2b348b0e738a971f1764ed33",
                                     "195803b375606646057dd72b")),
        }[case]
        actual = tuple(digest(frame, 540, box) for frame in (first_frame, pixels, control_frame))
        require(actual == expected, f"Measurement crops differ from the expected values ({actual})")
        return ("3500 and 4200 mV, 4000 mV control" if case == "battery"
                else "Two injected IMU states and the stationary control match the axis readouts")
    if case == "rtc":
        return check_rtc(log, control, pixels, control_frame)
    if case == "sdcard":
        files = fat32_files(out / "sdcard.img", {"WriteTest.txt", "TestPicture01.png", "TestPicture02.png"})
        require(files["WriteTest.txt"] == b"Hello, world! \nSD card write success! \n", "Incorrect persisted TXT")
        require(all((pixels, first_frame, control_frame)), "Missing SD media or no-card control capture")
        require(digest(control_frame, 540) == "569133fa6d0a854856a3923e",
                "No-card screen should report SD card not detected")
        require(pixels != first_frame and all(len(color_counts(frame)) >= 3 for frame in (pixels, first_frame)),
                "The two seeded PNG frames were not displayed")
        return "Two seeded PNGs displayed and the exact WriteTest.txt bytes persisted"
    if case == "wakeup":
        times = loop_times(log)
        require(len(times) > 20 and "chip reset at" not in log, "Timer sleep did not resume the original loop")
        enter = re.search(r"\[rtc-sleep\] enter timer light sleep t=([\d.]+)s", log)
        wake = re.search(r"\[rtc-sleep\] timer wake t=([\d.]+)s", log)
        require(enter and wake, "Missing RTC sleep and wake timestamps")
        elapsed = float(wake[1]) - float(enter[1])
        require(4.92 < elapsed < 5.05 and times[19] < float(enter[1]) < float(wake[1]) < times[20]
                and all(.22 < b - a < .30 for a, b in zip(times[:19], times[1:20])),
                f"Expected 250 ms cadence followed by five seconds of RTC sleep, measured {elapsed:.3f} s")
        return f"Original loop resumed after {elapsed:.3f} s of RTC timer sleep"
    return check_tones("papers3", case, out / f"{case}.wav")


def check_mono_controls(case, pixels, control_frame, first_frame):
    require(all((pixels, first_frame, control_frame)), "Missing two positive frames and the control")
    if case == "button":
        # Full frames contain the button status with no other changing content.
        require(digest(control_frame, 480) == "d583dcd9ee73710c0b3939d5", "Control prompt changed")
        require(digest(pixels, 480) == "9a9c4580e3d599440ea60e83", "BtnB result missing")
        require(len(color_counts(first_frame)) > 1 and first_frame not in (pixels, control_frame),
                "BtnA did not produce a distinct status")
        return "BtnA and BtnB changed the visible status relative to the idle prompt"
    expected = {
        "battery": ("e10153923aae19419aabdc9f", "78bb6f4e54025c3787634244"),
        "imu": ("83e15fd8590691e43d9e9fed", "27399b096a1770848ddfa51a"),
    }[case]
    require(tuple(digest(frame, 480) for frame in (pixels, control_frame)) == expected,
            "Second measurement and stationary control did not match the reviewed measurements")
    require(len(color_counts(first_frame)) > 1 and first_frame not in (pixels, control_frame),
            "First measurement did not change the visible readout")
    return "Both inputs changed the visible measurement relative to the control"


def check_nfc(case, log, control, out):
    if case == "nfc-4":
        require("NFC-A emulation ready" in log and "NDEF text: PaperMONO" in log,
                "Guest did not configure NFC-A target mode")
        return "NOT RUN", "No virtual reader fixture for target-mode activation and NDEF readback"
    require(control is not None and "NFC selections=0" in control, "Missing no-tag selection control")
    require(any(high for _, high in vcd_edges(out / f"{case}.vcd", 6)), "No ST25R3916 IRQ on GPIO6")
    if case in {"nfc-1", "nfc-2"}:
        first = (out / f"{case}-first.log").read_text(errors="replace")
        marker = "PICC:" if case == "nfc-1" else "==== Dump "
        require(marker + "04331144556677" in first and marker + "04998877665544" in log and marker not in control,
                "Guest did not report both exact NFC UIDs and a quiet control")
        if case == "nfc-2":
            require(all("==== Dump complete ====" in trace for trace in (first, log)),
                    "The guest did not finish both block dumps")
        return "PASS", "Guest read both programmed UIDs via the IRQ-driven NFC-A data path"
    require("Write NDEF OK!" in log and re.search(r"TNF:\s*1 T:T", log),
            "Guest did not write and read a text NDEF record")
    require("Write NDEF OK!" not in control, "No-input control wrote an NDEF record")
    return "PASS", "Guest wrote a text NDEF record and read it back via ST25R3916 IRQ and FIFO"


def check_power(board, case, log, control, out):
    writes = {data for direction, data in i2c_transactions(log, 0x6e) if direction == "write"}
    require(writes, "No PM1 I²C transactions")
    if case.endswith("-6"):
        ioe = {data for direction, data in i2c_transactions(log, 0x4f) if direction == "write"}
        require("IOE1 initialization successful" in log and (0x57,) in register_reads(log, 0x4f, 2),
                "Guest did not initialize the W revision at 0x4f")
        # Latches show the antenna, reset, NFC control and separate LED sequences.
        for register, mask in ((5, 2), (6, 2), (5, 8), (5, 128), (6, 1)):
            values = {data[register - data[0] + 1] & mask for data in ioe
                      if data and data[0] <= register < data[0] + len(data) - 1}
            require(values == {0, mask}, f"IOE1 register {register:#x} never toggled bit {mask:#x}")
        require(any(len(data) > 1 and data[0] == 0x11 and data[1] & 4 for data in writes),
                "LoRa rail was never enabled by PM1 GPIO2")
        return "IOE1 W initialized, PM1 LoRa rail enabled, antenna/reset/NFC/LED latches toggled"
    require(control is not None, "No idle power control")
    if (board, case) in {("papercolor", "m5pm1-1"), ("papercolor", "wakeup-2"),
                         ("papermono", "m5pm1_m5ioe1-1")}:
        require((0x3d, 0xa5) in writes and re.search(r"chip reset at t=\d+\.\d+s: cause 0x1", log)
                and "chip reset at" not in control, "PM1 timer arm did not produce an isolated power-on reset")
        require(len(loop_times(log)) > 1, "Guest loop did not run after timer wake")
        return "PM1 timer arm produced a power-on reset and a running guest"
    interrupt = "BMI270 AnyMotionInterrupt enabled successfully" if case.endswith(("-2", "-3")) and board == "papermono" else "RTC IRQ enabled successfully"
    require(interrupt in log and "PM1 initialization successful" in log, "Guest did not configure its wake source")
    if case.endswith(("-3", "-5")):
        require("PM1 IRQ triggered" in log and "PM1 IRQ triggered" not in control, "Wake IRQ missing or present in control")
        if case.endswith("-3"):
            require(re.search(r"BMI270 interrupt status: 0x(?!0000)[0-9A-Fa-f]{4}", log), "BMI270 IRQ status is zero")
        return "PM1 IRQ and the guest wake status matched the input"
    require(re.search(r"chip reset at t=\d+\.\d+s: cause 0x1", log) and "chip reset at" not in control,
            "Wake source did not restore the powered-off SoC")
    return "Wake source restored the simulated power rail and restarted the guest"


def check_sht40(log, control, first_log, input_frame, first_frame, control_frame):
    readings = []
    for trace in (first_log, log, control):
        cmds = list(i2c_transactions(trace, 0x44))
        require(("write", (0xfd,)) in cmds, "SHT40 high-precision measurement write did not reach 0x44")
        values = next((data for direction, data in cmds if direction == "read" and len(data) == 6), None)
        require(values, "SHT40 read never returned six bytes")

        def crc(data):
            remainder = 0xff
            for value in data:
                remainder ^= value
                for _ in range(8):
                    remainder = ((remainder << 1) ^ (0x31 if remainder & 0x80 else 0)) & 0xff
            return remainder

        require(values[2] == crc(values[:2]) and values[5] == crc(values[3:5]), "SHT40 CRC mismatch")
        temp, humid = (int.from_bytes(bytes(values[i:i + 2]), "big") for i in (0, 3))
        readings.append((round(175 * temp / 65535 - 45, 1), round(125 * humid / 65535 - 6, 1)))
    require(readings[:2] == [(18.0, 35.0), (32.0, 71.0)] and readings[2] not in readings[:2],
            f"Unexpected temperature and humidity values {readings}")
    require(all((first_frame, input_frame, control_frame))
            and len({first_frame, input_frame, control_frame}) == 3,
            "Completed measurement frames did not distinguish both samples and the default")
    return f"CRC-backed measurements and distinct completed frames {readings}"


def check_mic(board, log, control, out):
    if board == "papermono":
        second = (out / "mic-second.log").read_text(errors="replace")
        means = []
        for trace in (log, second, control or ""):
            levels = [float(value) for value in re.findall(r"MIC rms=(\d+\.\d+)", trace)]
            require(len(levels) >= 3, "Guest microphone statistics missing")
            means.append(sorted(levels)[len(levels) // 2])
        require(means[2] < 10 and means[0] > 100 and means[1] > 2.5 * means[0],
                f"Microphone statistics do not track both amplitudes and silence ({means})")
        return f"Guest microphone RMS tracks both input amplitudes and silence ({means})"
    takes = [pcm(out / f"mic{suffix}.wav") for suffix in ("", "-second", "-control")]
    for (rate, samples), hz in zip(takes[:2], (700, 1200)):
        require(len(samples) >= rate and spectrum(samples, rate, hz) > 0.1,
                f"Recording playback does not preserve the {hz} Hz injected signal")
    require(rms(takes[2][1]) < 20, "Silence control produced audible playback")
    return "Both injected frequencies survive microphone recording and playback"


def verify(board, case, log, pixels, control, early, control_frame, first_frame, out):
    def pixel(x, y):
        pos = (y * runner.DIMS[board][0] + x) * 3
        return tuple(pixels[pos:pos + 3])

    colors = color_counts(pixels) if pixels else Counter()
    if re.search(r"Guru Meditation|assert failed|abort\(\) was called|rebooting\.\.\.", log, re.I):
        return "FAIL", "Guest panic, assertion, or reboot marker"
    if case == "stock":
        if len(colors) < 2:
            return "FAIL", "No completed nonuniform stock frame"
        return "NOT RUN", "Stock rendered; use the service probe for input and REPL"
    if board == "paperdiy" and case in {"button", "display-2"}:
        if not all((pixels, first_frame, control_frame)):
            return "NOT RUN", "Two action frames and a no-input frame are required"
        if len({pixels, first_frame, control_frame}) != 3 or len(colors) < 2:
            return "FAIL", "The two button actions did not yield distinct completed frames"
        return "PASS", "Each action changed the frame relative to the no-input control"
    if (board, case) == ("paperdiy", "m5pm1"):
        active, idle = loop_times(log), loop_times(control)
        if not active or not idle:
            return "NOT RUN", "Guest loop traces are missing"
        if ("Power-on timer configured" in log and "Power-off timer configured" in log
            and re.search(r"chip reset at t=8\.\d+s: cause 0x1", log)
            and 12 < max(active) < 14 and max(idle) > 17 and "chip reset at" not in control):
            return "PASS", "Timed restart and shutdown observed against a running control"
        return "FAIL", "Timed reset or stop did not match the guest and control traces"
    if case.startswith(("display", "quickstart")):
        if not colors or len(colors) == 1:
            return "FAIL", "No completed nonuniform application frame"
        if early is None:
            return "NOT RUN", "Completed early frame capture missing"
        if early != {(255, 255, 255)}:
            return "FAIL", "Early capture did not retain the white frame"
        if (board, case) == ("paperdiy", "quickstart-longtextscroll"):
            if first_frame is None:
                return "NOT RUN", "Earlier scroll frame missing"
            if pixels != first_frame and len(color_counts(first_frame)) > 1:
                return "PASS", "Two completed nonuniform frames show the scroll changing"
            return "FAIL", "The scroll did not move between captures"
        if (board, case) == ("paperdiy", "display-1"):
            shades = [pixel(x, 395)[0] for x in (145, 223, 297, 375)]
            if not shades[0] < shades[1] < shades[2] or shades[3] != shades[0]:
                return "FAIL", f"Expected black, dark gray, light gray, black swatches, got {shades}"
            return "PASS", "Completed grayscale frame with expected swatch ordering"
        if (board, case) == ("papermono", "display-1"):
            if len(colors) != 4 or not all(r == g == b or max(r, g, b) - min(r, g, b) <= 3 for r, g, b in colors):
                return "FAIL", f"Expected four gray levels, got {len(colors)} RGB values"
            return "PASS", "Four quantized levels and white early control"
        if (board, case) == ("papercolor", "display"):
            if len(colors) != 6:
                return "FAIL", f"Expected six Spectra 6 palette values, got {len(colors)}"
            return "PASS", "All six colors after refresh and white early control"
        if case == "quickstart-bargraph":
            require(first_frame and pixels != first_frame, "Bar heights stayed fixed")
            for frame in (first_frame, pixels):
                width, height = runner.DIMS[board]
                row = frame[:width * 3]
                require(row.count(b"\xff\xff\xff") > 1 and row.count(b"\0\0\0") > width // 2
                        and frame[-3 * width:] == bytes(3 * width)
                        and color_counts(frame)[(255, 255, 255)] > height,
                        "Bar graph did not render black background and white markers")
            return "PASS", "White markers and quantized bars animate over a black background"
        if (board, case) == ("papermono", "display-2"):
            require(all((control_frame, first_frame)) and len({pixels, first_frame, control_frame}) == 3,
                    "Touch mode changes did not produce distinct completed updates")
            return "PASS", "Two touch-selected display modes yielded completed frames distinct from idle"
        return "FAIL", "Missing display behavior checker"
    if case == "touch":
        if control is None:
            return "NOT RUN", "Completed no-input control missing"
        points = set(re.findall(r"x:\s*(\d+)[, ]+\s*y:\s*(\d+)", log))
        if len(points) < 2:
            return "FAIL", f"Guest reported {len(points)} distinct touch points, expected two"
        if re.search(r"x:\s*\d+[, ]+\s*y:\s*\d+", control or ""):
            return "FAIL", "No-input control reported a touch"
        return "PASS", "Guest reported two points and silent no-input control"
    if case in {"microsd", "sdcard"} and control is not None and "Displayed:" in log and "Displayed:" not in control:
        if board == "paperdiy" and "Contents of /PaperDIY_Log.txt:" not in log:
            return "FAIL", "Guest did not read back the log it wrote"
        return "PASS", "Guest rendered SD media while the no-card control did not"
    if case == "rtc-1" and "RTC demo time set" in log:
        values = set(re.findall(r"RTC: 2026/05/08 .*?12:00:(\d\d)", log))
        if len(values) >= 2:
            return "PASS", "Guest set the calendar and observed it ticking"
    if case == "button" and board == "papercolor":
        if control is None:
            return "NOT RUN", "Completed no-input control missing"
        if all(f"{key} was Pressed" in log and f"{key} was Pressed" not in control for key in "ABC"):
            return "PASS", "Guest reported three injected buttons and no control presses"
        return "FAIL", "Missing guest button markers or control produced a press"
    if board == "papers3":
        return "PASS", check_papers3(case, log, pixels, control, control_frame, first_frame, out)
    if board == "papermono" and case in {"battery", "button", "imu"}:
        return "PASS", check_mono_controls(case, pixels, control_frame, first_frame)
    if case == "rtc-2":
        require(control is not None and "RTC synced from SNTP" not in control,
                "No disconnected RTC control or it unexpectedly synced")
        require("state Associated" in log and re.search(r"\b[1-9]\d* NTP answers", log)
                and "RTC synced from SNTP" in log, "Guest did not set its clock from sandbox NTP")
        timestamps = set(re.findall(r"RTC:\s+2026/05/08.*?\b(\d\d:\d\d:\d\d)", log))
        require(len(timestamps) >= 2, "Synced RTC did not continue ticking")
        return "PASS", "NTP synchronized RTC, guest read back advancing time, and idle control stayed offline"
    if case.startswith("nfc-"):
        return check_nfc(case, log, control, out)
    if case.startswith("m5pm1") or case == "wakeup-2":
        return "PASS", check_power(board, case, log, control, out)
    if case == "wakeup":
        require(control and "chip reset at" not in control, "Missing uninterrupted key-idle control")
        enter = re.findall(r"\[rtc-sleep\] enter timer light sleep t=([\d.]+)s", log)
        wake = re.findall(r"\[rtc-sleep\] timer wake t=([\d.]+)s", log)
        require(len(enter) == len(wake) == 1 and 4.95 < float(wake[0]) - float(enter[0]) < 5.05
                and pixels and control_frame and pixels != control_frame, "Light-sleep key did not suspend and resume")
        return "PASS", "Button A suspended and resumed the original loop around five seconds"
    if case == "wakeup-1":
        resets = [float(value) for value in re.findall(r"chip reset at t=([\d.]+)s: cause 0x1", log)]
        require(len(resets) >= 2 and "chip reset at" not in (control or ""), "No periodic timer wake")
        require(all(5 <= b - a <= 12 for a, b in zip(resets, resets[1:])), "Wrong timer wake cadence")
        return "PASS", "Repeated timed power-on resets at the documented sleep point"
    if case == "mic":
        return "PASS", check_mic(board, log, control, out)
    if case in {"buzzer", "speaker"}:
        return "PASS", check_tones(board, case, out / f"{case}.wav")
    if case == "sht40":
        first_log = (out / "sht40-first.log").read_text(errors="replace")
        return "PASS", check_sht40(log, control, first_log, pixels, first_frame, control_frame)
    if case == "battery":
        measured = set(re.findall(r"Power info refreshed: input=(\d), level=(\d+)%, bat=(\d+)mV, vbus=(\d+)mV", log))
        require(any(v == "3678" and i == "0" and b == "0" for i, _, v, b in measured)
                and any(v == "4200" and i == "1" and b != "0" for i, _, v, b in measured)
                and "bat=3678mV" not in (control or "") and "bat=4200mV" not in (control or ""),
                f"Battery voltage and plugged state did not follow both inputs ({sorted(measured)})")
        return "PASS", "Guest reported 3678 mV unplugged and 4200 mV plugged against baseline"
    if case == "ir_nec":
        frames = set(nec_frames(out / f"{case}.vcd"))
        require({(0x1111, 0x34), (0x1111, 0x35)} <= frames,
                f"GPIO48 lacked both timing- and checksum-valid NEC frames ({sorted(frames)})")
        return "PASS", "GPIO48 carries NEC extended address 0x1111 and commands 0x34/0x35"
    if case == "rgb_led":
        if board == "papercolor":
            expected = {"505050,000000", "000000,505050", "500000,000050", "000050,500000", "005000,005000"}
            observed = set(re.findall(r"\[leds\] GPIO21 (\w+,\w+)", log))
            require(expected <= observed, f"WS2812 chain lacked both independent pixels and color order ({sorted(observed)})")
        else:
            states = {(data[1] & 0x80, data[2] & 1) for direction, data in i2c_transactions(log, 0x4f)
                      if direction == "write" and len(data) >= 3 and data[0] == 5}
            require({(0, 0), (0x80, 0), (0, 1), (0x80, 1)} <= states,
                    f"IOE1 never showed the independent blue and green channels ({sorted(states)})")
        return "PASS", "The installed LED channels followed the documented sequence"
    if case == "program":
        require(pixels and digest(pixels, 400) == "0725453568b67b3231e1b322",
                "Quickstart frame lacks the six bands and PAPER COLOR text")
        return "PASS", "Completed frame matches the six-band PAPER COLOR quickstart artwork"
    if case.startswith("lora-"):
        peer = out / "lora-result.json"
        if not peer.is_file():
            return "NOT RUN", "Two loopback peer runs and a disconnected receiver control are required"
        result = json.loads(peer.read_text())
        expected = "sender" if case == "lora-1" else "receiver"
        require(result["pass"] and len(result["runs"][expected]["payloads"]) >= 2,
                "Loopback payloads or disconnected receiver control failed")
        return "PASS", "SX1262 virtual peer exchanged multiple exact payloads"
    if case in {"microsd", "sdcard"}:
        return "FAIL", "Guest did not show the documented SD media with a quiet no-card control"
    return "FAIL", "Expected behavior did not pass its case checker"


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("output", type=Path)
    args = p.parse_args()
    out = args.output
    receipt = json.loads((out / "runs.json").read_text())
    board = receipt["board"]
    verdicts = []
    completed = {(row["case"], row["kind"]) for row in receipt["runs"] if row.get("exit") == 0}
    for row in receipt["runs"]:
        if row["kind"] != "input":
            continue
        case = row["case"]
        if row.get("state") == "NOT RUN":
            state, reason = "NOT RUN", row["reason"]
        elif row["exit"] == 124:
            state, reason = "FAIL", f"Host timeout before {row['guest_limit_seconds']} guest seconds"
        elif row["exit"] != 0:
            state, reason = "FAIL", f"Simulator exit {row['exit']}"
        else:
            frame = out / f"{case}.png"
            early = out / f"{case}-early.png"
            control = out / f"{case}-control.log"
            try:
                pixels = png_rgb(frame.read_bytes(), dimensions=runner.DIMS[board]) if frame.exists() else None
                early_colors = None
                if (case, "early") in completed:
                    early_colors = set(color_counts(png_rgb(early.read_bytes(), dimensions=runner.DIMS[board])))
                control_log = control.read_text(errors="replace") if (case, "control") in completed else None
                def frame_at(kind):
                    if (case, kind) in completed:
                        path = out / f"{case}-{kind}.png"
                        return png_rgb(path.read_bytes(), dimensions=runner.DIMS[board])
                    return None

                log = (out / f"{case}.log").read_text(errors="replace")
                stop = re.search(r"\[emu\] stop: Halted .*?emulated ([\d.]+)s", log)
                require(stop and ("guest_limit_seconds" not in row or float(stop[1]) == row["guest_limit_seconds"]),
                        "Simulator did not reach the configured guest acceptance horizon")
                state, reason = verify(board, case, log, pixels, control_log, early_colors,
                                       frame_at("control"), frame_at("first"), out)
            except (OSError, ValueError, struct.error, wave.Error) as exc:
                state, reason = "FAIL", str(exc)
        verdicts.append({"case": case, "verdict": state, "reason": reason})
        print(f"| {case} | {state} | {reason} |")
    (out / "verdicts.json").write_text(json.dumps(verdicts, indent=2) + "\n")
    return int(any(item["verdict"] != "PASS" for item in verdicts))


if __name__ == "__main__":
    raise SystemExit(main())
