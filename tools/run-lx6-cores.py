#!/usr/bin/env python3
"""Run the classic Core Arduino matrix and stock UART0 replay."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

CASES = {
    "basic-v27": "battery button display imu microsd quickstart-longtextscroll speaker wakeup wav",
    "core2": "battery button display imu mic microsd quickstart-scrollgraph rgb_led rtc touch-1 touch-2 vibration wakeup wav",
}
KEY_X = (63, 160, 258)


def touch(t, x, y):
    return f"{t} touch {x} {y} 1\n{t + 0.24} touch {x} {y} 0\n"


def key(device, t, index, hold=600):
    if device == "basic-v27":
        return f"{t} press {'abc'[index]} {hold}\n"
    return f"{t} touch {KEY_X[index]} 260 1\n{t + hold / 1000} touch {KEY_X[index]} 260 0\n"


def plan(device):
    jobs = []

    def add(case, seconds, label=None, actions="", card=False, audio=False, extra=()):
        jobs.append(dict(case=case, label=label or case, seconds=seconds, actions=actions,
                         card=card, audio=audio, extra=extra))

    add("display", 1.15)
    add("display", 0.8, "display-first")
    add("button", 0.75 if device == "basic-v27" else 1.7, "button-idle")
    for index, letter in enumerate("abc"):
        if device == "basic-v27":
            add("button", 0.85, f"button-{letter}", key(device, 0.55, index, 180))
        else:
            add("button", 1.7, f"button-{letter}", key(device, 1.2, index, 1400))
    release = key(device, 0.55 if device == "basic-v27" else 1.2, 0, 180)
    add("button", 1.0 if device == "basic-v27" else 1.95, "button-release", release)
    powers = ("1 1 0", "3 0 0") if device == "basic-v27" else ("3678 42 0", "4096 87 1")
    add("battery", 1.35, "battery-idle")
    for i, value in enumerate(powers):
        ina = f"0.05 ina 0 {value.split()[0]} {[-5200, 8000][i]}\n" if device == "core2" else ""
        add("battery", 1.35, f"battery-{i + 1}", f"0.05 battery {value}\n" + ina)
    add("imu", 1.6, "imu-idle")
    if device == "core2":
        for i, sample in enumerate(("0.5 -0.25 1.0 12 -25 30", "-0.75 0.5 0.25 -60 80 -90")):
            add("imu", 1.6, f"imu-{i + 1}", f"0.65 imu {sample}\n")
        for case in ("touch-1", "touch-2"):
            add(case, 1.7, f"{case}-idle")
            for i, (x, y) in enumerate(((62, 45), (246, 153))):
                add(case, 1.7, f"{case}-{i + 1}", f"1.2 touch {x} {y} 1\n")
            add(case, 1.9, f"{case}-release", touch(1.2, 62, 45))
        add("rgb_led", 0.8, "rgb-red")
        add("rgb_led", 1.8, "rgb-green")
        add("rgb_led", 2.8, "rgb-blue")
        add("vibration", 4.9)
        add("rtc", 10, extra=("--wifi", "ssid=YOUR WIFI SSID NAME,psk=YOUR WIFI PASSWORD,epoch=1764000000"))
        actions = key(device, 1.6, 0, 140) + key(device, 3.0, 2, 140)
        for label, extra in (("mic-idle", ()), ("mic-733", ("--i2s-tone", "0:733:0.04")),
                             ("mic-1234", ("--i2s-tone", "0:1234:0.06"))):
            add("mic", 3.8, label, actions, card=True, extra=extra)
    else:
        add("speaker", 2.5, audio=True)
    scroll = "quickstart-longtextscroll" if device == "basic-v27" else "quickstart-scrollgraph"
    add(scroll, 1.25)
    add(scroll, 0.8, f"{scroll}-first")
    add("microsd", 0.9, "microsd-absent")
    add("microsd", 3.8, card=True)
    add("microsd", 2.8, "microsd-first", card=True)
    add("wav", 2.8, card=True, audio=True)
    add("wakeup", 7.8, actions=key(device, 1.15, 0, 160))
    add("stock", 2.7, "stock-home")
    navigation = key(device, 2.55, 2, 200) if device == "basic-v27" else touch(2.55, 34, 48)
    add("stock", 3.0, "stock-navigation", navigation)
    buttons = "print('LX6'+'-INPUT',M5.BtnA.isPressed(),M5.BtnB.isPressed(),M5.BtnC.isPressed())"
    actions = "3.1 uart0-cr import M5; print('LX6'+'-BOARD',M5.getBoard())\n"
    actions += f"3.25 uart0-cr M5.update(); {buttons}\n"
    for i in range(3):
        t = 3.4 + i * 0.45
        actions += key(device, t, i, 260)
        actions += f"{t + 0.13} uart0-cr M5.update(); {buttons}\n"
    actions += f"4.85 uart0-cr M5.update(); {buttons}\n"
    if device == "core2":
        actions += "5.0 touch 88 110 1\n"
        actions += "5.15 uart0-cr M5.update(); print('LX6'+'-TOUCH',M5.Touch.getCount(),M5.Touch.getX(),M5.Touch.getY())\n"
        actions += "5.3 touch 88 110 0\n"
    add("stock", 5.5, "stock-input", actions)
    return jobs


def digest(path):
    with path.open("rb") as handle:
        return hashlib.file_digest(handle, "sha256").hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("binary", type=Path)
    parser.add_argument("device", choices=CASES)
    parser.add_argument("out", type=Path)
    parser.add_argument("--board", help="override CLI board preset for a revision replay")
    parser.add_argument("--source", help="full source commit used to build the simulator")
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument("--only", nargs="+", help="run a subset of labels")
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir()
    firmware = Path(os.environ["CORE_PLAN_DIR"]) / "firmware" / args.device
    stock = Path(os.environ["CORE_STOCK_DIR"]) / f"{args.device}.bin"
    rom = Path(os.environ["ESP32SIM_ROM_DIR"]) / "esp32_rev300_rom.elf"
    seed = Path(os.environ["CORE_SD_SEED"])
    binary = out / "esp32sim"
    shutil.copy2(args.binary, binary)
    board = args.board or f"m5stack-{args.device}"
    root = Path(__file__).resolve().parent.parent
    version = subprocess.check_output(
        ["git", "-C", root, "rev-parse", f"{args.source or 'HEAD'}^{{commit}}"], text=True,
    ).strip()
    diff = subprocess.check_output(["git", "-C", root, "status", "--porcelain", "-uno"], text=True)
    jobs = [job for job in plan(args.device) if args.only is None or job["label"] in args.only]
    if not jobs:
        parser.error("no jobs selected")
    seeds = dict(simulator=digest(binary), rom=digest(rom), sd_seed=digest(seed), stock=digest(stock))

    def run(job):
        label, case = job["label"], job["case"]
        image = stock if case == "stock" else firmware / case / "sketch.merged.bin"
        options = [str(binary), "--chip", "esp32", "--board", board, "--rom", str(rom), "--boot", "rom",
                   "--flash-image", str(image), "--console", "uart0", "--net", "none", "--no-dump",
                   "--max-seconds", str(job["seconds"]), "--tft-png", str(out / f"{label}.png")]
        if job["actions"]:
            script = out / f"{label}.script"
            script.write_text(job["actions"])
            options += ["--script", str(script)]
        if job["card"]:
            card = out / f"{label}.img"
            shutil.copyfile(seed, card)
            options += ["--sd-rw", str(card)]
        if job["audio"]:
            options += ["--wav", str(out / f"{label}.wav")]
        options += list(job["extra"])
        with (out / f"{label}.log").open("wb") as serial, (out / f"{label}.err").open("wb") as report:
            try:
                status = subprocess.run(options, stdout=serial, stderr=report, timeout=1200, check=False).returncode
            except subprocess.TimeoutExpired:
                status = 124
        (out / f"{label}.status").write_text(f"{status}\n")
        print(f"{label}: {status}", flush=True)
        return {**job, "command": options, "image_sha256": digest(image), "exit_code": status}

    with ThreadPoolExecutor(args.jobs) as executor:
        results = list(executor.map(run, jobs))
    (out / "run.json").write_text(json.dumps({
        "source_commit": version, "source_status": diff, "board": board, "input_sha256": seeds, "cases": results,
    }, indent=2) + "\n")
    if args.only is None:
        return subprocess.run([sys.executable, root / "tools/check-lx6-cores.py", args.device, out], check=False).returncode
    return int(any(row["exit_code"] != 0 for row in results))


if __name__ == "__main__":
    sys.exit(main())
