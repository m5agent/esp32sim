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
    return {"w": width, "h": h // scale, "lit": len(coords), "bounds": bounds,
            "colors": len(palette), "palette": palette.most_common(5), "crc32": f"{zlib.crc32(image):08x}"}


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
                elif step in {2, 3, 4}:
                    reason = "Panel produced pixels; text and placement need a reference assertion"
            if hit:
                reason = "Visible pixel count or fill color matches the sketch operation"
        elif case.startswith("button"):
            idle, held = frame(case + "-idle"), frame(case + "-held")
            hit = bool(screen and (difference(screen, idle) or difference(held, idle)))
            if case == "button-7":
                hit = bool(screen and (difference(screen, idle) or (not screen["lit"] and idle and idle["lit"])))
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
            if hit:
                reason = "Both PCM tones changed the guest waveform relative to silence"
        elif case in {"buzzer", "speaker"}:
            if case == "buzzer" and device == "stickc-plus":
                hit = field("buzzer_updates") >= 2
                reason = "Legacy M5.Beep routed a PWM burst and then released it" if hit else "No routed PWM burst"
            else:
                audio = out / f"{case}.wav"
                if audio.is_file():
                    with wave.open(str(audio), "rb") as stream:
                        raw = stream.readframes(stream.getnframes())
                    samples = [x[0] for x in struct.iter_unpack("<h", raw)]
                    hit = len(samples) > 200 and max(samples) - min(samples) > 512 and len(set(samples)) > 4
                reason = "Routed I2S piezo data produced a varying baseband signal" if hit else "No varying routed I2S output"
        elif case == "imu" and device == "stickc-plus2":
            samples = re.findall(r"ax:([-\d.]+)\s+ay:([-\d.]+)\s+az:([-\d.]+)", text)
            expected = ((0.5, -0.25, 1.0), (-0.75, 0.375, 0.25))
            hits = [any(all(abs(float(a) - b) < 0.06 for a, b in zip(s, e)) for s in samples) for e in expected]
            hit = (all(hits) and "ax:" in read("imu-idle", "log")) or bool(three_frames())
            if hit:
                reason = "UART reports both injected accelerometer vectors and the idle control ran"
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
        record(label, "FAIL", fault or "Factory display and device-local inputs require behavioral proof",
               frame=screen, guest_boot=bool(re.search(r"mpu9250 addr|M5StickC initializing", text)))
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
                expected = ["K016-SCAN [104, 117]", "K016-ID 71", "K016-MAG-ID 48",
                            "K016-ONE 2000f0004000 640038ff2c0110 7f",
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
