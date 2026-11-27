#!/usr/bin/env python3
"""Capture each pinned Stick-family sketch and stock image on its exact board."""
import argparse
import concurrent.futures
import hashlib
import json
import math
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import wave

from check_lx6_stick import check

DEVICES = ("stickc", "stickc-plus", "stickc-plus-se", "stickc-plus2", "m5stick", "stickt2")
FACTORY = {"m5stick", "stickt2"}
FAULT_TIMEOUT = 180


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def tone_file(path, frequency):
    rate = 16000
    period = [int(5000 * math.sin(math.tau * frequency * i / rate)) for i in range(rate)]
    with wave.open(str(path), "wb") as handle:
        handle.setparams((1, 2, rate, 0, "NONE", "not compressed"))
        handle.writeframes(struct.pack("<16000h", *period) * 12)


def scenarios(device, case):
    """Pick emulated-time inputs and observation windows for the published code."""
    base = [("", 1.1 if case.startswith(("display", "lcd_", "quickstart")) else 3.2, [], [])]
    if case == "button-7":
        base = [("", 2.25, ["1.90 press a 500"], []), ("-idle", 2.25, [], [])]
    elif case.startswith("button"):
        if case in {"button-5", "button-9"}:
            pulse_ms = 3200 if case == "button-9" else 2200
            end = 3.9 if case == "button-9" else 3.2
            base = [("", end, [f"0.5 press a {pulse_ms}"], []), ("-idle", end, [], [])]
        else:
            pulse = ["0.55 press a 350"]
            if case == "button":
                pulse += ["1.1 press b 350"]
            end = 1.65 if case == "button" else 1.2
            base = [("", end, pulse, []), ("-idle", end, [], []), ("-held", 0.8, pulse[:1], [])]
    elif case.startswith("imu") and case != "imu-1":
        inputs = ["0.8 imu 0.5 -0.25 1 125 -50 25", "2.0 imu -0.75 0.375 0.25 -100 80 -30"]
        base = [("", 3.45, inputs, []), ("-idle", 3.45, [], []), ("-first", 1.9, inputs[:1], [])]
    elif case in {"battery", "axp192_m5stickc-1", "power-1"}:
        if device == "stickc-plus2":
            inputs = [["0.01 adc 38 1.72"], ["0.01 adc 38 2.04"]]
            args = ["--adc", "38"]
        else:
            inputs = [["0.01 battery-mv 3450"], ["0.01 battery-mv 4080"]]
            args = []
        base = [("", 1.45, inputs[1], args), ("-idle", 1.45, [], args), ("-first", 1.45, inputs[0], args)]
    elif case in {"axp192_m5stickc-3", "power-3"}:
        base = [("", 4.0, [], []), ("-first", 1.8, [], []), ("-restart", 6.0, ["4.1 pmic-key short"], [])]
    elif case in {"axp192_m5stickc-5", "power-5"}:
        base = [("", 1.8, [], []), ("-first", 0.9, [], [])]
    elif case == "mic":
        base = [("", 2.7, [], ["--mic-wav", "@mic733.wav"]),
                ("-idle", 2.7, [], []),
                ("-second", 2.7, [], ["--mic-wav", "@mic1234.wav"]),
                ("-playback", 4.2, ["2.8 press a 150"], ["--mic-wav", "@mic733.wav", "--wav", "@mic-playback.wav"])]
    elif case == "spm1423_m5stickc":
        inputs = []
        base = [("", 3.8, inputs, ["--mic-wav", "@mic733.wav"]),
                ("-idle", 3.8, inputs, []),
                ("-second", 3.8, inputs, ["--mic-wav", "@mic1234.wav"])]
    elif case in {"ir_nec", "led", "speaker", "buzzer"}:
        base = [("", 5.5, [], ["--vcd", f"@{case}.vcd", "--wav", f"@{case}.wav"])]
    elif case == "wakeup":
        base = [("", 9.0, ["1.3 press a 150"], ["--vcd", f"@{case}.vcd"]),
                ("-idle", 9.0, [], [])]
    elif case == "rtc" and device == "stickc-plus2":
        wifi = "ssid=YOUR WIFI SSID NAME,psk=YOUR WIFI PASSWORD,epoch=1764000000"
        base = [("", 10.0, [], ["--wifi", wifi, "--net", "none"]), ("-idle", 4.8, [], [])]
    elif case == "quickstart-display" and device == "stickc-plus":
        base += [("-text", 3.4, [], [])]
    elif case == "quickstart-scrollgraph":
        base += [("-first", 0.7, [], [])]
    elif case == "display" and device == "stickc-plus-se":
        base = [("", 2.2, [], []), ("-first", 0.55, [], [])]
    elif case in {"display", "quickstart-display"}:
        base += [("-first", 0.55, [], [])]
    elif case.startswith(("rtc", "system_")):
        base = [("", 4.8, [], []), ("-first", 1.1, [], [])]
    elif case.endswith("-2") and case.startswith(("axp192", "power")):
        base = [("", 1.1, [], ["--debug", "i2c"])]
    return base


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("device", choices=DEVICES)
    parser.add_argument("binary", type=Path)
    parser.add_argument("plan", type=Path)
    parser.add_argument("stock", type=Path, help="catalog firmware for this device")
    parser.add_argument("rom", type=Path)
    parser.add_argument("out", type=Path)
    parser.add_argument("--jobs", type=int, default=4)
    parser.add_argument("--case", action="append", default=[], help="only replay specified case(s)")
    parser.add_argument("--probe-stock", type=Path, help="generic Atom Lite MicroPython for factory-board controller probe")
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error("--jobs must be positive")
    repo = Path(__file__).resolve().parent.parent
    for name in ("binary", "plan", "stock", "rom", "out", "probe_stock"):
        if getattr(args, name) is not None:
            setattr(args, name, getattr(args, name).resolve())
    out = args.out
    out.mkdir(parents=True)
    binary = out / "esp32sim"
    shutil.copy2(args.binary, binary)
    device = args.device
    inventory = json.loads((args.plan / "arduino-inventory.json").read_text())
    source = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=repo, text=True).strip()
    dirty = subprocess.check_output(["git", "status", "--porcelain"], cwd=repo, text=True)
    (out / "identity.json").write_text(json.dumps({
        "source": source, "dirty": bool(dirty), "binary_sha256": digest(binary),
        "rom_sha256": digest(args.rom), "docs_revision": inventory["docs_revision"],
    }, indent=2) + "\n")
    jobs = []

    def schedule(label, image, seconds, script, options):
        jobs.append((label, image, seconds, script, options))

    rows = [row for row in inventory["cases"] if row["device"] == device]
    (out / "cases.json").write_text(json.dumps(rows, indent=2) + "\n")
    for row in rows:
        case = row["case"]
        if args.case and case not in args.case:
            continue
        image = args.plan / "firmware" / device / case / "sketch.merged.bin"
        if image.is_file():
            for suffix, seconds, script, options in scenarios(device, case):
                schedule(case + suffix, image, seconds, script, options)

    stock_label = "factory" if device in FACTORY else "stock"
    if not args.case or stock_label in args.case:
        schedule(stock_label, args.stock, 5.0, [], [])
        if device not in FACTORY:
            key = "b" if device == "stickc-plus2" else "a"
            schedule("stock-button", args.stock, 5.0, [f"4.1 press {key} 200"], ["--vcd", "@stock-button.vcd"])
            script = [
                "5.1 uart0-cr \x03",
                "5.3 uart0-cr import M5; from machine import Pin; print('LX6'+'-BOARD', M5.getBoard())",
                "5.6 uart0-cr print('LX6'+'-A', Pin(37, Pin.IN).value())",
                "5.8 press a 500",
                "6.0 uart0-cr print('LX6'+'-A', Pin(37, Pin.IN).value())",
                "6.5 uart0-cr print('LX6'+'-A', Pin(37, Pin.IN).value())",
                "6.7 uart0-cr print('LX6'+'-B', Pin(39, Pin.IN).value())",
                "6.9 press b 500",
                "7.1 uart0-cr print('LX6'+'-B', Pin(39, Pin.IN).value())",
                "7.5 uart0-cr print('LX6'+'-B', Pin(39, Pin.IN).value())",
            ]
            schedule("stock-repl", args.stock, 8.0, script, [])

    if args.probe_stock and device in FACTORY and (not args.case or "probe" in args.case):
        script = (repo / "tools" / "lx6-stick-probes" / f"{device}.txt").read_text().splitlines()
        schedule("probe", args.probe_stock, 10, script, ["--vcd", "@probe.vcd"])

    if any("@mic733.wav" in options for _, _, _, _, options in jobs):
        tone_file(out / "mic733.wav", 733)
        tone_file(out / "mic1234.wav", 1234)

    def run(job):
        label, firmware, seconds, script, options = job
        command = [str(binary), "--chip", "esp32", "--board", f"m5stack-{device}",
                   "--rom", str(args.rom), "--boot", "rom", "--flash-image", str(firmware),
                   "--console", "uart0", "--max-seconds", str(seconds), "--no-dump",
                   "--tft-png", str(out / f"{label}.png")]
        script_path = out / f"{label}.script"
        if script:
            script_path.write_text("\n".join(script) + "\n")
            command += ["--script", str(script_path)]
        command += [str(out / option[1:]) if option.startswith("@") else option for option in options]
        with (out / f"{label}.log").open("wb") as serial, (out / f"{label}.err").open("wb") as diagnostics:
            try:
                status = subprocess.run(command, stdout=serial, stderr=diagnostics, timeout=FAULT_TIMEOUT).returncode
            except subprocess.TimeoutExpired:
                status = 124
        (out / f"{label}.status").write_text(f"{status}\n")
        receipt = {"case": label, "command": command, "firmware_sha256": digest(firmware),
                   "script_sha256": digest(script_path) if script else None, "exit": status}
        (out / f"{label}.receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")
        print(device, label, status, flush=True)

    with concurrent.futures.ThreadPoolExecutor(args.jobs) as pool:
        list(pool.map(run, jobs))
    results = check(device, out)
    (out / "verdicts.json").write_text(json.dumps(results, indent=2) + "\n")
    selected = [v for name, v in results.items() if not args.case or name in args.case]
    return 0 if all(v["status"] in {"PASS", "DOCS-DEFECT"} for v in selected) else 1


if __name__ == "__main__":
    sys.exit(main())
