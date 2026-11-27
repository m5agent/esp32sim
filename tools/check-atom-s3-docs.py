#!/usr/bin/env python3
"""Check artifacts from run-atom-s3-docs.py. Requires ffmpeg for image decoding."""
import argparse
import ast
from array import array
import json
from pathlib import Path
import re
import statistics
import subprocess
import sys
import wave

BOARD_ID = {"atoms3-lite": 137, "atoms3": 11, "atoms3r": 18, "atoms3u": 138,
            "atom-echos3r": 145, "atoms3r-cam": 144, "atoms3r-m12": 143}


def rgb(path, size):
    if not path.is_file():
        return b""
    result = subprocess.run(
        ["ffmpeg", "-hide_banner", "-loglevel", "error", "-threads", "1", "-i", str(path),
         "-vf", f"scale={size}:{size}:flags=area", "-frames:v", "1", "-f", "rawvideo",
         "-pix_fmt", "rgb24", "-threads", "1", "-"], capture_output=True)
    return result.stdout if result.returncode == 0 else b""


def frame(directory, case):
    pixels = rgb(directory / f"{case}.png", 128)
    return len(pixels) == 3 * 128**2 and len(set(zip(pixels[::3], pixels[1::3], pixels[2::3]))) >= 3


def pcm(path):
    if not path.is_file():
        return 0, []
    with wave.open(str(path), "rb") as wav:
        samples = array("h", wav.readframes(wav.getnframes()))
        if sys.byteorder != "little":
            samples.byteswap()
        return wav.getframerate() * wav.getnchannels(), samples


def tone(samples, rate, frequencies):
    from math import cos, pi, sin
    # Search the strongest short time window. A clipped tone retains its fundamental.
    length = max(rate // 20, 1)
    windows = [samples[start:start+length] for start in range(0, len(samples)-length+1, length)]
    if not windows or max(map(abs, samples)) < 8:
        return set()
    active = sorted(windows, key=lambda data: sum(map(abs, data)), reverse=True)[:60]
    found = set()
    for hz in frequencies:
        angles = [2*pi*hz*i/rate for i in range(length)]
        cs, sn = [cos(x) for x in angles], [sin(x) for x in angles]
        for data in active:
            scale = sum(map(abs, data))
            real = sum(x*y for x, y in zip(data, cs))
            imag = sum(x*y for x, y in zip(data, sn))
            if scale and (real*real + imag*imag)**.5 / scale > 0.38:
                found.add(hz)
                break
    return found


def report(directory):
    manifest = json.loads((directory / "run.json").read_text())
    board = manifest["board"]

    def log(case, kind="log"):
        path = directory / f"{case}.{kind}"
        return path.read_text(errors="replace") if path.is_file() else ""

    def result(case, success, detail):
        return {"case": case, "status": "PASS" if success else "FAIL", "detail": detail}

    def rgb_sequence(case):
        match = re.search(r"RGB updates=(\d+) recent=(\[.*\])", log(case, "err"))
        return ast.literal_eval(match[2]) if match else []

    def ir(case):
        match = re.search(r"NEC frames=\d+ recent=(\[.*?\]);", log(case, "err"))
        return ast.literal_eval(match[1]) if match else []

    cases = [p.stem for p in directory.glob("*.json") if p.stem not in {"run", "checks"}]
    rows = []
    for case in sorted({s.removesuffix("-still").removesuffix("-a").removesuffix("-b") for s in cases}):
        serial, err = log(case), log(case, "err")
        if case == "button":
            hit = len(re.findall(r"(?i)(?:^|\bwas )pressed\s*$", serial, re.M))
            released = len(re.findall(r"(?i)(?:^|\bwas )released\s*$", serial, re.M))
            control = bool(re.search(r"(?im)^(?:BtnA was )?(?:Pressed|Released)\s*$", log("button-still")))
            rows.append(result(case, hit >= 2 and released >= 2 and not control,
                               f"guest press/release {hit}/{released}, idle events={control}"))
        elif case in {"ir_nec", "ir"}:
            codes = ir(case)
            expected = ((0x1111, 0x34) if board in {"atoms3", "atoms3-lite"} else
                        (0x1234, 0x56) if board == "atom-echos3r" else (0, 0x55))
            rows.append(result(case, expected in codes and len(set(codes)) >= 2,
                               f"decoded routed NEC address/command pairs {codes[:8]}"))
        elif case == "lite_led":
            pixels = rgb_sequence(case)
            colors = {max(range(3), key=p.__getitem__) for p in pixels if max(p) and p.count(0) == 2}
            rows.append(result(case, colors == {0, 1, 2} and len(pixels) >= 3,
                               f"routed WS2812 values {pixels[:6]}"))
        elif case == "rgb_led":
            pixels = rgb_sequence(case)
            rows.append(result(case, len(pixels) > 2 and "LED status updated:" in serial,
                               f"{len(pixels)} recent RGB values and guest button-mode transition"))
        elif case in {"display", "quickstart-display", "quickstart-bargraph"}:
            rows.append(result(case, frame(directory, case) and "Guru Meditation" not in serial
                               and bool(re.search(r"LCD id_reads=[1-9]\d* frames=[1-9]", err)),
                               "128×128 LCD content, a routed ID read, and guest SPI pixels"))
        elif case == "imu":
            def vectors(variant):
                data = log(f"imu-{variant}")
                return [tuple(map(float, v)) for v in re.findall(
                    r"ax:([-\d.]+)\s+ay:([-\d.]+)\s+az:([-\d.]+)\s+"
                    r"gx:([-\d.]+)\s+gy:([-\d.]+)\s+gz:([-\d.]+)", data)]
            observed = [vectors(v) for v in ("still", "a", "b")]
            vectors_seen = [v[-1] if v else () for v in observed]
            expected = [(0, 0, 1, 0, 0, 0), (0.5, -0.25, 1, 125, -50, 25), (-0.75, 0.3, 0.25, -100, 80, -30)]
            if board == "atoms3r":
                expected = [(y, -x, z, gy, -gx, gz) for x, y, z, gx, gy, gz in expected]
            agree = all(len(v) == 6 and all(abs(x-y) <= (0.08 if i < 3 else 5)
                        for i, (x, y) in enumerate(zip(v, ref))) for v, ref in zip(vectors_seen, expected))
            rows.append(result(case, agree, f"still, vector A and vector B guest values {vectors_seen}"))
        elif case == "mic" and board == "atoms3u":
            levels = []
            for variant in ("still", "a", "b"):
                samples = re.findall(r"Level:(\d+)", log(f"mic-{variant}"))
                levels.append(statistics.median(map(int, samples[-80:])) if samples else 0)
            rows.append(result(case, 0 < levels[0] < levels[1] < levels[2],
                               f"guest idle, tone A and tone B median mic levels {levels}"))
        elif case == "mic":
            quiet = pcm(directory / "mic-still.wav")[1]
            peaks = [max(map(abs, quiet), default=0)]
            found = []
            for variant, hz in (("a", 733), ("b", 1193)):
                rate, samples = pcm(directory / f"mic-{variant}.wav")
                peaks.append(max(map(abs, samples), default=0))
                found.append(hz in tone(samples, rate, (hz,)) if rate else False)
            rows.append(result(case, all(found) and max(peaks[1:]) > peaks[0],
                               f"idle and tone peaks {peaks}, recovered input frequencies {found}"))
        elif case == "program" and board == "atoms3u":
            rows.append(result(case, serial.count("hello world!") >= 2, "repeated guest greeting"))
        elif case in {"speaker", "program"}:
            rate, samples = pcm(directory / f"{case}.wav")
            found = tone(samples, rate, (4000, 7000)) if rate else set()
            rows.append(result(case, found == {4000, 7000}, f"I2S PCM has expected tones {sorted(found)}"))
        elif case == "quickstart-camera":
            means, counts = [], []
            for variant in ("still", "a", "b"):
                capture = directory / f"{case}-{variant}.http"
                data = capture.read_bytes() if capture.is_file() else b""
                images = re.findall(b"\xff\xd8.+?\xff\xd9", data, re.S)
                counts.append(len(images))
                if not images:
                    means.append(b"")
                    continue
                image = capture.with_suffix(".jpg")
                image.write_bytes(images[0])
                means.append(rgb(image, 1))
            black, red, green = means
            ok = min(counts) >= 2 and len(black) == len(red) == len(green) == 3
            if ok:
                ok = max(black) < 45 and red[0] > max(red[1:]) + 40 and green[1] > max(green[0], green[2]) + 40
            rows.append(result(case, ok, f"guest HTTP JPEG counts {counts}, RGB means {[list(v) for v in means]}"))
        elif case == "stock":
            id_ok = re.search(rf"^ATOM_BOARD {BOARD_ID[board]}\s*$", serial, re.M)
            states = re.findall(r"^ATOM_BTN ([01])\s*$", serial, re.M)
            button_ok = "0" in states and "1" in states
            lcd_ok = board not in {"atoms3", "atoms3r"} or frame(directory, case)
            rows.append(result(case, bool(id_ok) and button_ok and lcd_ok,
                               f"CDC REPL board ID={bool(id_ok)}, guest button transition={button_ok}, screen check={lcd_ok}"))
    for p in sorted(directory.glob("*.json")):
        if p.stem in {"run", "checks"}:
            continue
        run = json.loads(p.read_text())
        if run.get("exit") != 0:
            rows.append(result(p.stem, False, f"simulator exit {run['exit']}"))
    return rows


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("directory", type=Path)
    args = p.parse_args()
    rows = report(args.directory)
    (args.directory / "checks.json").write_text(json.dumps(rows, indent=2) + "\n")
    for row in rows:
        print(row["status"], row["case"], row["detail"])
    sys.exit(not rows or any(row["status"] != "PASS" for row in rows))
