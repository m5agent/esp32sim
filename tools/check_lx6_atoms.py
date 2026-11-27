#!/usr/bin/env python3
"""Check Atom firmware output and peripheral observations from run-lx6-atoms.sh."""
import argparse
import json
from pathlib import Path
import re
import struct
import wave
import zlib


def nec_frames(path, pin=12):
    """Decode one GPIO’s NEC envelopes. Microsecond-scale carrier gaps stay inside marks."""
    if not path.exists():
        return []
    rises = []
    previous = None
    ts = 0
    for line in path.read_text().splitlines():
        if line.startswith("#"):
            ts = int(line[1:]) / 1_000_000
        elif line in {f"1s{pin}", f"0s{pin}"}:
            current = line.startswith("1")
            if previous is False and current:
                rises.append(ts)
            previous = current
    groups = []
    for ts in rises:
        if not groups or ts - groups[-1][-1] > 90:
            groups.append([])
        groups[-1].append(ts)
    frames = []
    for start, first in enumerate(groups):
        if start + 34 > len(groups):
            continue
        sequence = groups[start:start + 34]
        if not 12_000 <= sequence[1][0] - first[0] <= 15_000:
            continue
        intervals = [right[0] - left[0] for left, right in zip(sequence[1:], sequence[2:])]
        if not all(850 <= delta <= 2500 for delta in intervals):
            continue
        raw = sum((delta > 1650) << i for i, delta in enumerate(intervals))
        frames.append(raw)
    return frames


def imu_samples(log):
    values = [float(s) for s in re.findall(r"(?:Acc|Gyr) [XYZ] =\s+(-?\d+\.\d+)", log)]
    return [values[i:i + 6] for i in range(0, len(values) - 5, 6)]


def close(expected, actual):
    return all(abs(a - b) < 0.25 for a, b in zip(actual, expected))


def matrix_pixels(path):
    """Read one RGB8 value per LED from the simulator's scaled matrix PNG."""
    blob = path.read_bytes() if path.exists() else b""
    if not blob.startswith(b"\x89PNG\r\n\x1a\n"):
        return []
    width, height, depth, color, _, _, interlaced = struct.unpack_from(">IIBBBBB", blob, 16)
    if not width or width != height or width % 5 or (depth, color, interlaced) != (8, 2, 0):
        return []
    compressed = bytearray()
    at = 8
    while at + 12 <= len(blob):
        size, kind = struct.unpack_from(">I4s", blob, at)
        if kind == b"IDAT":
            compressed.extend(blob[at + 8:at + 8 + size])
        at += size + 12
    pixels = zlib.decompress(compressed)
    stride = width * 3 + 1
    if len(pixels) != stride * height or any(pixels[::stride]):
        return []
    scale = width // 5
    row_starts = range(1, len(pixels), stride * scale)
    return [tuple(pixels[row + x:row + x + 3])
            for row in row_starts for x in range(0, width * 3, 3 * scale)]


def check(device, out):
    matrix = device in {"atom-matrix", "atom-matrix-v11"}
    rows = {}

    def log(label, suffix="log"):
        path = out / f"{label}.{suffix}"
        return path.read_text(errors="replace") if path.exists() else ""

    def ran(label):
        return log(label, "status").strip() == "0"

    def record(label, passed, reason):
        rows[label] = {"status": "PASS" if passed else "FAIL", "evidence": reason}

    if device in {"atom-lite", "atom-matrix", "atom-matrix-v11"}:
        text = log("button")
        ok = (text.count("BtnA was pressed") == 2 and text.count("BtnA was released") == 2
              and "BtnA was held" in text and "BtnA was pressed" not in log("button-idle"))
        record("button", ran("button") and ran("button-idle") and ok, "two GPIO39 pulses and idle control")
        colors = {}
        for name, expected in (("red", "ff0000"), ("green", "00ff00"), ("blue", "0000ff")):
            err = log(f"rgb-{name}", "err")
            colors[name] = ran(f"rgb-{name}") and bool(re.search(
                fr"RGB frames=[1-9]\d* pixels={25 if matrix else 1} first={expected} uniform=true", err))
        record("rgb", all(colors.values()), colors)
        frames = nec_frames(out / "ir_nec.vcd")
        record("ir_nec", ran("ir_nec") and 0xaa55ff00 in frames and 0xa956fe01 in frames,
               {"gpio12_frames": [f"{frame:08x}" for frame in frames]})
        text = log("quickstart-logoutput")
        record("quickstart-logoutput", ran("quickstart-logoutput") and "count:1" in text and "count:2" in text,
               "UART0 loop log counts")
        if matrix:
            samples = imu_samples(log("imu"))
            idle = imu_samples(log("imu-idle"))
            expected = ([0.5, -0.25, 1.0, 125, -50, 25], [-0.75, 0.375, 0.25, -100, 80, -30])
            hits = [any(close(want, sample) for sample in samples) for want in expected]
            clear = bool(idle) and all(not close(want, sample) for sample in idle for want in expected)
            record("imu", ran("imu") and ran("imu-idle") and all(hits) and clear,
                   {"samples": samples, "idle_samples": idle})
        else:
            samples = imu_samples(log("imu"))
            record("imu", ran("imu") and bool(samples) and all(not any(sample) for sample in samples),
                   "Matrix-only example; Lite serves as the no-IMU control.")
    elif device == "atom-echo":
        defects = {
            "quickstart-echorest": "BaiduRest.cpp:175: WiFiClient declaration missing",
            "quickstart-playmusic": "LED_DisPlay.h:62: boolean declaration missing",
            "quickstart-recordplay": "LED_DisPlay.h:62: boolean declaration missing",
        }
        for name, defect in defects.items():
            if not (out / f"{name}.status").exists():
                rows[name] = {"status": "DOCS-DEFECT", "evidence": defect}
            else:
                rows[name] = {"status": "NOT RUN", "evidence": "Case captured; requires an app-specific stimulus and oracle."}

    stock = log("stock")
    board = {"atom-lite": 128, "atom-matrix": 141, "atom-matrix-v11": 141, "atomu": 130, "atom-echo": 142}[device]
    observed = re.findall(r"(?m)^LX6-BOARD (\d+)", stock)
    button = re.findall(r"(?m)^LX6-BUTTON ([01])", stock)
    rgb = re.search(r"RGB frames=[1-9]\d* pixels=\d+ first=204060", log("stock", "err"))
    record("stock-repl", ran("stock") and "POWERON_RESET" in stock and observed == [str(board)]
           and button == ["0", "1", "0"] and bool(rgb),
           {"board": observed, "button": button, "rgb": bool(rgb)})
    record("stock-startup", "MicroPython" in stock and ">>> " in stock and "Traceback" not in stock, "ROM to stock MicroPython prompt")
    if matrix:
        lit = {i: rgb for i, rgb in enumerate(matrix_pixels(out / "stock-layout.png")) if any(rgb)}
        record("stock-layout", ran("stock-layout") and lit == {8: (255, 0, 0), 20: (0, 0, 255)}, lit)
    if device in {"atomu", "atom-echo"}:
        audio = {}
        for suffix in ("silent", "733", "1234"):
            samples = re.search(r"(?m)^LX6-SAMPLES (\d+) (\d+) (\d+)", log(f"audio-{suffix}"))
            audio[suffix] = [int(n) for n in samples.groups()] if samples else []
        silent, tone, other = audio["silent"], audio["733"], audio["1234"]
        record("stock-mic", silent == [0, 0, 0] and len(tone) == len(other) == 3
               and tone[0] == other[0] == 0 and tone[1] > 100 and other[1] > 100 and tone != other, audio)
        if device == "atom-echo":
            path = out / "audio-733.wav"
            samples = ()
            rate = None
            if path.exists():
                with wave.open(str(path), "rb") as wav:
                    rate = wav.getframerate()
                    samples = struct.unpack(f"<{wav.getnframes()}h", wav.readframes(wav.getnframes()))
            rises = [i for i in range(2, len(samples))
                     if samples[i - 2] < 0 and samples[i - 1] <= 0 < samples[i]]
            hz = rate * (len(rises) - 1) / (rises[-1] - rises[0]) if len(rises) > 2 else 0
            record("stock-speaker", bool(re.search(r"(?m)^LX6-SPEAKER True", log("audio-733")))
                   and len(rises) > 100 and abs(hz - 440) < 4,
                   {"samples": len(samples), "nonzero": sum(x != 0 for x in samples),
                    "rate": rate, "measured_hz": round(hz, 3)})
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("device")
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    rows = check(args.device, args.output)
    result = json.dumps(rows, indent=2)
    print(result)
    (args.output / "results.json").write_text(result + "\n")
    return any(row["status"] == "FAIL" for row in rows.values())


if __name__ == "__main__":
    raise SystemExit(main())
