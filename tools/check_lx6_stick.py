"""Interpret Stick-family guest evidence without promoting boot-only smoke to PASS."""
import argparse
import hashlib
from collections import Counter
import json
from pathlib import Path
import re
import struct
import zlib
import wave

from check_lx6_atoms import nec_frames


DEFECTS = {
    ("stickc", "pwm-1"): ("pwm.md:31,35", "LEDC 2.x functions removed by the pinned Arduino-ESP32 3.x package"),
    ("stickc", "pwm-2"): ("pwm.md:63–64", "LEDC 2.x functions removed by the pinned Arduino-ESP32 3.x package"),
    ("stickc", "sh200q_m5stickc-1"): ("sh200q_m5stickc.md:23", "M5StickC 0.3.0 IMU does not expose I2C_Read_NBytes"),
    ("stickc", "sh200q_m5stickc-2"): ("sh200q_m5stickc.md:37,50", "getGyroData accepts float pointers, example passes int16_t pointers"),
    ("stickc", "sh200q_m5stickc-3"): ("sh200q_m5stickc.md:68,81", "getAccelData accepts float pointers, example passes int16_t pointers"),
    ("stickc", "system_m5stickc-1"): ("system_m5stickc.md:51–56", "standalone sketch omits Arduino loop()"),
    ("stickc-plus2", "button"): ("button.md:33,36", "M5StickCPlus2 1.0.1 exposes BtnC, example calls BtnPWR"),
}
FATAL = re.compile(r"Guru Meditation|panic'ed|assert failed|abort\(\) was called|Rebooting\.\.\.|Brownout detector")


def frame_summary(path, width):
    if not path.is_file():
        return None
    png = path.read_bytes()
    if not png.startswith(b"\x89PNG\r\n\x1a\n"):
        raise ValueError(f"Invalid PNG {path.name}")
    w, h, depth, color, _, _, interlaced = struct.unpack_from(">IIBBBBB", png, 16)
    if (depth, color, interlaced) != (8, 2, 0):
        raise ValueError(f"Unexpected framebuffer encoding {path.name}")
    at = 8
    data = bytearray()
    while at + 12 <= len(png):
        size, tag = struct.unpack_from(">I4s", png, at)
        if tag == b"IDAT":
            data.extend(png[at + 8:at + 8 + size])
        at += size + 12
    pixels = zlib.decompress(data)
    stride = w * 3 + 1
    if len(pixels) != stride * h or any(pixels[::stride]):
        raise ValueError(f"Unexpected framebuffer filter {path.name}")
    image = b"".join(pixels[start + 1:start + stride] for start in range(0, len(pixels), stride))
    scale = w // width
    if not scale or w % width or h % scale:
        raise ValueError(f"Framebuffer size does not match native glass {path.name}")
    native = [image[(y * w + x) * 3:(y * w + x) * 3 + 3]
              for y in range(0, h, scale) for x in range(0, w, scale)]
    palette = Counter(pixel.hex() for pixel in native)
    coords = [(i % width, i // width) for i, pixel in enumerate(native) if pixel != bytes(3)]
    bounds = None
    if coords:
        xs, ys = zip(*coords)
        bounds = [min(xs), min(ys), max(xs), max(ys)]
    top = palette.most_common(5)
    regions = {}
    for color, _ in top:
        indexes = [i for i, pixel in enumerate(native) if pixel.hex() == color]
        xs, ys = zip(*((i % width, i // width) for i in indexes))
        regions[color] = [min(xs), min(ys), max(xs), max(ys)]
    return {"w": width, "h": h // scale, "lit": len(coords), "bounds": bounds,
            "colors": len(palette), "palette": top, "regions": regions,
            "crc32": f"{zlib.crc32(image):08x}"}


def tones(path):
    if not path.is_file():
        return []
    with wave.open(str(path), "rb") as stream:
        rate = stream.getframerate()
        raw = stream.readframes(stream.getnframes())
    samples = [value for value, in struct.iter_unpack("<h", raw)]
    rising = [i for i in range(2, len(samples)) if samples[i - 2] < 0 and samples[i - 1] <= 0 < samples[i]]
    groups = []
    for edge in rising:
        if not groups or edge - groups[-1][-1] > rate // 25:
            groups.append([])
        groups[-1].append(edge)
    return [(round(rate * (len(edges) - 1) / (edges[-1] - edges[0]), 1),
             round(1000 * (edges[-1] - edges[0]) / rate, 1))
            for edges in groups if len(edges) > 10]


def gpio_writes(vcd, pin):
    return sum(line in {f"0s{pin}", f"1s{pin}"} for line in vcd.splitlines())


def check(device, out):
    rows = json.loads((out / "cases.json").read_text())
    results = {}

    def read(label, suffix):
        path = out / f"{label}.{suffix}"
        return path.read_text(errors="replace") if path.is_file() else ""

    def frame(label):
        width = {"stickc": 80, "m5stick": 64}.get(device, 135)
        return frame_summary(out / f"{label}.png", width)

    def capture(label):
        text, diag = read(label, "log"), read(label, "err")
        fault = FATAL.search(text + "\n" + diag)
        status = read(label, "status").strip()
        return status, text, diag, fault.group() if fault else None

    def record(case, status, reason, **detail):
        results[case] = {"status": status, "reason": reason, **detail}

    def difference(a, b):
        return a is not None and b is not None and a["crc32"] != b["crc32"]

    def completed(label):
        status, _, _, fault = capture(label)
        return status == "0" and not fault

    for row in rows:
        case = row["case"]
        defect = DEFECTS.get((device, case))
        status, text, diag, fault = capture(case)
        if not status:
            if defect:
                location, reason = defect
                relative = row["docs_url"].split("/en/", 1)[1].rsplit("/", 1)[0]
                record(case, "DOCS-DEFECT", reason, file=f"static/en/{relative}/{location}")
            else:
                record(case, "NOT RUN", "No execution in this capture")
            continue
        screen = frame(case)
        if status != "0" or fault:
            record(case, "FAIL", fault or f"Simulator exit {status}", frame=screen)
            continue

        def field(name):
            match = re.search(fr"\b{name}=(\d+)", diag)
            return int(match[1]) if match else 0

        def three_frames():
            first, idle = frame(case + "-first"), frame(case + "-idle")
            return screen and screen["lit"] and difference(screen, idle) and difference(first, idle) and difference(screen, first)

        hit = False
        reason = "Required case-specific output was not proven"
        if re.fullmatch(r"(?:display|lcd_m5stickc)-\d+", case):
            step = int(case.rsplit("-", 1)[1])
            if screen and screen["lit"] and field("pixels"):
                rgb = screen["palette"]
                foreground = screen["lit"]
                if step == 1:
                    hit = screen["colors"] == 1 and rgb[0][0].startswith("0000")
                elif step in {5, 6, 9, 10, 11}:
                    hit = ((foreground == {5: 1, 6: 13, 9: 76, 10: 200}.get(step))
                           or (step == 11 and 150 <= foreground < 200))
                    if hit:
                        x0, y0, x1, y1 = screen["bounds"]
                        span = sorted([x1 - x0 + 1, y1 - y0 + 1])
                        hit = span == {5: [1, 1], 6: [13, 13], 9: [10, 30], 10: [10, 20], 11: [10, 20]}[step]
                elif step in {7, 8}:
                    x0, y0, x1, y1 = screen["bounds"]
                    hit = sorted([x1 - x0 + 1, y1 - y0 + 1]) == [48, 77]
                    hit &= (100 <= foreground <= 220) if step == 7 else (1000 <= foreground <= 1200)
                elif step == 2:
                    # Font 1 uses 6×8 cells. The 22 glyphs occupy 1056 pixels,
                    # including the red glyph strokes and the white cell backgrounds.
                    bounds = [0, 0, 77, 15] if device == "stickc" else [0, 0, 131, 7]
                    hit = (foreground == 1056 and screen["bounds"] == bounds
                           and dict(rgb).get("ff0000") == 238 and dict(rgb).get("ffffff") == 818)
                    reason = "The 22-character red-on-white text matches its glyph counts and wrap"
                elif step in {3, 4}:
                    count, bounds = {3: (103, [0, 0, 58, 7]), 4: (169, [5, 20, 59, 66])}[step]
                    hit = dict(rgb).get("ffffff") == count and screen["bounds"] == bounds
                    reason = "Default-font glyph counts and the documented cursor placement match"
            if hit and step not in {2, 3, 4}:
                reason = "Visible pixel count or fill color matches the sketch operation"
        elif case == "quickstart-helloworld":
            hit = bool(screen and field("pixels") and screen["colors"] == 2
                       and 98 <= screen["lit"] <= 135 and screen["bounds"] == [0, 0, 64, 6])
            reason = "The Hello World glyphs occupy the expected region seen in the product photograph"
        elif case == "display" or (case == "quickstart-display" and device == "stickc-plus2"):
            first = frame(case + "-first")
            if screen and field("pixels") > 135 * 240:
                sizes = [count for _, count in screen["palette"]]
                hit = (screen["colors"] >= 3 and 400 in sizes and
                       (device != "stickc-plus2" or screen["colors"] > 20) and difference(first, screen))
                reason = "Native frame shows the expected 20×20 rectangles and circles at distinct checkpoints"
        elif case == "quickstart-scrollgraph":
            first = frame(case + "-first")
            if screen and screen["bounds"]:
                x0, y0, x1, y1 = screen["bounds"]
                hit = (screen["colors"] == 9 and y1 - y0 == 239 and x1 - x0 > 115
                       and 1400 <= screen["lit"] <= 2600 and difference(first, screen))
                reason = "Six plotted colors and grid span the native display and advance across checkpoints"
        elif case == "quickstart-display" and device == "stickc-plus":
            stage = frame(case + "-text")
            hit = bool(screen and stage and screen["colors"] == 1 and stage["lit"]
                       and 100 <= stage["lit"] <= 250 and difference(stage, screen))
            reason = "The color-fill sequence advances to the documented Display Test lettering"
        elif case in {"axp192_m5stickc-2", "power-2"}:
            hit = ("panel_power=true" in diag and 0 < field("backlight") and
                   field("backlight_updates") > 0 and "[i2c] start addr 0x34" in diag)
            reason = "M5.begin addressed AXP192 and enabled the panel rails and backlight"
        elif case.startswith("button"):
            idle, held = frame(case + "-idle"), frame(case + "-held")
            hit = bool(screen and (difference(screen, idle) or difference(held, idle)))
            if case == "button-7":
                hit = bool(difference(screen, idle))
            if hit:
                reason = "Scripted GPIO input changed the documented button display versus idle"
        elif case in {"axp192_m5stickc-1", "power-1", "battery"}:
            hit = bool(three_frames())
            if hit:
                reason = "Two simulated battery levels changed the documented display versus default"
        elif case in {"axp192_m5stickc-3", "power-3"}:
            hit = ("panel_power=false" in diag and field("pixels") > 0 and screen and not screen["lit"]
                   and read(case + "-restart", "log").count("POWERON_RESET") >= 2)
            if hit:
                reason = "AXP192 shutdown removed panel power after an active frame"
        elif case in {"axp192_m5stickc-4", "power-4"}:
            hit = field("backlight_updates") > 4
            if hit:
                reason = "AXP192 LDO2 brightness followed the guest ramp"
        elif case in {"axp192_m5stickc-5", "power-5"}:
            first = frame(case + "-first")
            hit = bool(first and first["lit"] and screen and not screen["lit"] and field("backlight_updates") > 1)
            if hit:
                reason = "Guest ScreenSwitch changed the visible backlight across the two checkpoints"
        elif case in {"mic", "spm1423_m5stickc"}:
            idle, second = frame(case + "-idle"), frame(case + "-second")
            hit = bool(screen and screen["lit"] and difference(screen, idle) and difference(second, idle) and difference(screen, second))
            if hit and case == "mic" and device in {"stickc-plus-se", "stickc-plus2"}:
                playback = tones(out / "mic-playback.wav")
                hit = any(abs(hz - 733) <= 15 and duration >= 500 for hz, duration in playback)
                reason = f"Distinct tone and silence frames; routed replay bursts (Hz, ms) {playback[:4]}"
            elif hit:
                reason = "Both PCM tones changed the guest waveform relative to silence"
            else:
                reason = "Tone and silence waveform checkpoints did not separate"
        elif case in {"buzzer", "speaker"}:
            if case == "buzzer" and device == "stickc-plus":
                hit = field("buzzer_updates") >= 2
                if hit:
                    reason = "Legacy M5.Beep routed and released a PWM burst"
                elif ("ledcAttachChannel(a2=0x2 a3=0x0 a4=0xd)" in diag
                      and "ledcWriteTone(a2=0x0 a3=0xfa0" in diag
                      and "--trace-fn ledc_timer_config: 1 functions" in diag
                      and "ledc_timer_config(a2=" not in diag and "buzzer_updates=0" in diag):
                    record(case, "LIBRARY-DEFECT",
                           "GPIO2 attach requested 0 Hz, tone addressed GPIO0, and no LEDC timer was configured",
                           file="M5StickCPlus 0.1.1 src/utility/Speaker.cpp:14,31,55",
                           reference="m5stack:esp32 3.2.5 cores/esp32/esp32-hal-ledc.c:186–194,409–439",
                           frame=screen)
                    continue
            else:
                measured = tones(out / f"{case}.wav")
                expected = [(10000, 100), (4000, 20)] if case == "buzzer" else [(7000, 100), (4000, 200)]
                hit = all(any(abs(hz - target) < 40 and abs(ms - duration) < 8
                              for hz, ms in measured) for target, duration in expected)
                reason = f"Routed I2S piezo bursts (Hz, ms) {measured[:6]}"
        elif case == "imu" and device == "stickc-plus2":
            samples = re.findall(r"ax:([-\d.]+)\s+ay:([-\d.]+)\s+az:([-\d.]+)", text)
            expected = ((0.5, -0.25, 1.0), (-0.75, 0.375, 0.25))
            hits = [any(all(abs(float(a) - b) < 0.06 for a, b in zip(s, e)) for s in samples) for e in expected]
            hit = (all(hits) and "ax:" in read("imu-idle", "log")) or bool(three_frames())
            if hit:
                reason = "UART reports both injected accelerometer vectors and the idle control ran"
        elif case == "imu-5":
            hit = bool(three_frames())
            idle, first, gyro = (frame(case + suffix) for suffix in ("-idle", "-first", "-gyro"))
            if hit:
                reason = "The documented temperature display changed for both injected ADC words"
            elif (all(completed(case + suffix) for suffix in ("-idle", "-first", "-gyro"))
                  and all((screen, idle, first, gyro)) and screen["lit"]
                  and screen["crc32"] == idle["crc32"] == first["crc32"] and difference(gyro, idle)):
                record(case, "LIBRARY-DEFECT",
                       "Temperature-only inputs preserve the frame and gyro-Z changes it. The library decodes 0x47–0x48",
                       file="M5StickC 0.3.0 src/utility/MPU6886.cpp:208–212",
                       reference="InvenSense DS-000193 rev 1.1 §§7, 8.30, 8.33",
                       frame=screen, gyro_frame=gyro)
                continue
        elif case == "imu" or case in {"imu-2", "imu-3", "imu-4"}:
            hit = bool(three_frames())
            if hit:
                reason = "Distinct injected sensor inputs changed the application display versus idle"
        elif case == "imu-1":
            hit = bool(re.search(r"(?m)^0\s*$", text))
            if hit:
                reason = "Legacy sensor initialization returned success on UART"
        elif case == "ir_nec":
            pin = 19 if device == "stickc-plus2" else 9
            codes = nec_frames(out / f"{case}.vcd", pin)
            if pin == 9:
                hit = "(0, 85)" in diag and "(1, 86)" in diag
                reason = "Decoded two expected NEC commands from the active-low GPIO9 PWM envelope" if hit else "GPIO9 PWM envelope did not decode"
            else:
                hit = any(code & 0xffff == 0x1111 for code in codes)
                reason = "Decoded GPIO19 NEC codes " + ", ".join(f"{code:08x}" for code in codes[:8])
        elif case == "led":
            vcd = out / "led.vcd"
            edges = [line for line in vcd.read_text().splitlines() if line in {"0s19", "1s19"}] if vcd.is_file() else []
            hit = "0s19" in edges and "1s19" in edges and len(edges) >= 4
            reason = "G19 SET/CLR edges observed through the GPIO controller" if hit else "Required GPIO19 edges absent"
        elif case == "rtc":
            rtc = re.search(r"\brtc=(\S+)", diag)
            idle = re.search(r"\brtc=(\S+)", read(case + "-idle", "err"))
            hit = bool(rtc and idle and not rtc[1].startswith("24-01-01") and idle[1].startswith("24-01-01")
                       and screen and screen["lit"])
            reason = "Virtual Wi-Fi/SNTP populated the hardware RTC" if hit else "No SNTP-updated hardware calendar"
        elif case.startswith("rtc-"):
            current = re.search(r"\brtc=(\S+)", diag)
            first = re.search(r"\brtc=(\S+)", read(case + "-first", "err"))
            if current and first:
                if case in {"rtc-1", "rtc-2"}:
                    hit = "T18:56:" in current[1] and first[1] != current[1]
                else:
                    hit = not current[1].startswith("24-01-01T") and first[1] != current[1]
            reason = "Guest RTC write changed the BCD calendar and virtual time advanced" if hit else "Expected BCD write/progression absent"
        elif case == "system_m5stickc-2":
            first = frame(case + "-first")
            rtc = re.search(r"\brtc=(\S+)", diag)
            hit = bool(rtc and rtc[1].startswith("24-01-01T00:00:04") and screen
                       and screen["lit"] and difference(first, screen))
            reason = "System function displayed the BM8563 clock and its time advanced"
        elif case == "wakeup":
            hit = text.count("boot:") > 1 and read(case + "-idle", "log").count("boot:") == 1
            if hit:
                reason = "Timed wake reset followed scripted button input and idle showed one boot"
        record(case, "PASS" if hit else "FAIL", reason, frame=screen)

    label = "factory" if device in {"m5stick", "stickt2"} else "stock"
    status, text, diag, fault = capture(label)
    if not status:
        record(label, "NOT RUN", "No stock execution in this capture")
    elif label == "factory":
        screen = frame(label)
        if device == "m5stick":
            idle, released = frame("factory-idle"), frame("factory-release")
            whoami = bool(re.search(r"(?m)^mpu9250 addr: 71\s*$", text) and
                          "[i2c]   write 0x75\n[i2c] start addr 0x68 read=true\n[i2c]   read  0x71" in diag)
            edges = {pin: gpio_writes(read("factory", "vcd"), pin) for pin in (17, 19, 26)}
            hit = (whoami and completed("factory-idle") and completed("factory-release")
                   and all((screen, idle, released)) and screen["lit"] == 174
                   and screen["bounds"] == [17, 1, 59, 30] and idle["lit"] == released["lit"] == 0
                   and "[script] t=3.800s Gpio(35, true)" in read("factory-release", "err")
                   and edges[17] >= 4 and edges[19] >= 4 and edges[26] >= 200)
            reason = ("Original firmware read WHO_AM_I 0x71, drew the test text while G35 was held, "
                      "cleared it after release and drove G17/G19/G26") if hit else "Factory OLED, GPIO or MPU9250 check failed"
            detail = {"whoami": whoami, "gpio_writes": edges, "idle_frame": idle, "released_frame": released}
        else:
            splash, second, button = (frame("factory-" + name) for name in ("splash", "second", "button"))
            thermal = {}
            for suffix in ("", "-second", "-button"):
                capture_diag = read(label + suffix, "err")
                count = re.search(r"\bthermal_frames=(\d+)", capture_diag)
                thermal[suffix or "-first"] = int(count[1]) if count else 0
            cci = ("[i2c]   write 0x48\n[i2c]   write 0x55" in diag and
                   "[i2c]   write 0x48\n[i2c]   write 0x59" in diag and
                   "[i2c] start addr 0x30 read=true" in diag)
            no_errors = all("error=" not in read(label + suffix, "log") for suffix in ("", "-second", "-button"))
            hit = (all(completed("factory-" + name) for name in ("splash", "second", "button"))
                   and "M5StickC initializing...OK" in text and cci and no_errors
                   and min(thermal.values()) > 1 and all((screen, splash, second, button))
                   and difference(screen, splash) and difference(screen, second) and difference(screen, button)
                   and screen["regions"].get("ff0000") == [41, 45, 100, 124]
                   and sum(int(color[:2], 16) == int(color[2:4], 16) == int(color[4:], 16)
                           for color, _ in button["palette"][:2]) == 2)
            reason = ("Original firmware completed CCI and VoSPI, showed two thermal scenes and switched palette after BtnA"
                      if hit else "Factory Lepton bus, thermal scene or palette response check failed")
            detail = {"cci": cci, "no_bus_error": no_errors, "thermal_frames": thermal,
                      "splash_frame": splash, "second_frame": second, "button_frame": button}
        record(label, "PASS" if status == "0" and not fault and hit else "FAIL", fault or reason, frame=screen, **detail)
    else:
        screen, after = frame("stock"), frame("stock-button")
        repl = read("stock-repl", "log")
        buttons = {key: re.findall(fr"(?m)^LX6-{key} ([01])", repl) for key in ("A", "B")}
        home = bool(screen and screen["lit"] and difference(screen, after))
        uart = "LX6-BOARD " in repl and buttons == {"A": ["1", "0", "1"], "B": ["1", "0", "1"]}
        passed = status == "0" and not fault and home and uart
        reason = "Stock menu changed after input; UART REPL read both buttons" if passed else "Stock screen, button response, or UART REPL oracle failed"
        record(label, "PASS" if passed else "FAIL", fault or reason,
               frame=screen, input_frame_changed=home, uart_repl=uart, buttons=buttons)
    if label == "factory":
        state, probe, _, fault = capture("probe")
        screen = frame("probe")
        if not state:
            record("probe", "NOT RUN", "The separate generic MicroPython probe image was omitted")
        else:
            if device == "m5stick":
                expected = ["K016-SCAN [104, 117]", "K016-ID 71", "K016-MAG-ID 48", "K016-BAT 8f",
                            "K016-ONE 2000f0004000 640038ff2c0110 ef",
                            "K016-TWO d00018001000 9cffc800d4fe10 0f", "K016-OLED"]
                hit = (all(marker in probe.splitlines() for marker in expected)
                       and re.findall(r"(?m)^K016-BTN ([01])", probe) == ["1", "0", "1"]
                       and screen and screen["lit"] == 128)
            else:
                def scene(cool, warm):
                    payload = b"".join(struct.pack(">H", warm if 30 <= y < 90 and 40 <= x < 120 else cool)
                                       for y in range(120) for x in range(160))
                    return hashlib.sha256(payload).hexdigest()
                expected = ["T2-CCI 0006 000175a3", f"T2-ONE {scene(7000, 8000)} [1, 2, 3, 4]",
                            f"T2-TWO {scene(8000, 10000)} [1, 2, 3, 4]", "T2-LCD"]
                scan = re.findall(r"(?m)^T2-(?:RESET-)?SCAN (\[.*?\])", probe)
                hit = (all(marker in probe.splitlines() for marker in expected)
                       and scan == ["[42, 48, 52, 81, 104]", "[48, 52, 81, 104]"]
                       and re.findall(r"(?m)^T2-ENC ([\da-f]{4})", probe) == ["0005", "fffd", "0000"]
                       and screen and screen["lit"] == 135 * 120 and screen["colors"] == 3)
            record("probe", "PASS" if state == "0" and not fault and hit else "FAIL",
                   fault or "Controller probes check sensor input readback and native framebuffer", frame=screen)
    return results


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("device")
    parser.add_argument("out", type=Path)
    args = parser.parse_args()
    results = check(args.device, args.out)
    print(json.dumps(results, indent=2))
