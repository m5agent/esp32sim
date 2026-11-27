#!/usr/bin/env python3
"""Replay pinned S3 e-paper sketches. A boot receipt alone carries no PASS verdict."""

import argparse
from concurrent.futures import ThreadPoolExecutor
from hashlib import file_digest
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
from time import monotonic

BOARDS = {
    "paperdiy": "button display-1 display-2 m5pm1 microsd quickstart-longtextscroll",
    "papers3": "battery buzzer-1 buzzer-2 imu rtc sdcard touch wakeup quickstart-bargraph",
    "papermono": ("battery button buzzer display-1 display-2 imu lora-1 lora-2 "
                  "m5pm1_m5ioe1-1 m5pm1_m5ioe1-2 m5pm1_m5ioe1-3 m5pm1_m5ioe1-4 "
                  "m5pm1_m5ioe1-5 m5pm1_m5ioe1-6 mic microsd nfc-1 nfc-2 nfc-3 nfc-4 "
                  "rgb_led rtc-1 rtc-2 touch wakeup quickstart-bargraph"),
    "papercolor": ("battery button display ir_nec m5pm1-1 m5pm1-2 mic microsd program "
                   "rgb_led rtc-1 rtc-2 sht40 speaker wakeup-1 wakeup-2"),
}

INPUT = {
    "button": ["4.0 press a 350", "7.0 press b 400"],
    "touch": ["5.0 touch 60 120 1", "5.4 touch 60 120 0", "8.0 touch 190 320 1", "8.5 touch 190 320 0"],
    "m5pm1": ["4.0 press a 350", "9.0 press b 350"],
    "microsd": ["6.0 press a 300", "10.0 press b 300"],
    "battery": ["35.0 paper-battery 3678 0", "65.0 paper-battery 4200 1"],
}

DIMS = {"paperdiy": (540, 960), "papers3": (540, 960), "papermono": (480, 800), "papercolor": (400, 600)}
RTC_WIFI = {
    ("papers3", "rtc"): "ssid=YOUR WIFI SSID NAME,psk=YOUR WIFI PASSWORD",
    ("papermono", "rtc-2"): "ssid=ssid,psk=password",
    ("papercolor", "rtc-2"): "ssid=ssid,psk=password",
}
RTC_EPOCH = 1778241600
FIRST = {
    "paperdiy": {"button": 5.5, "display-2": 6, "quickstart-longtextscroll": 7},
    "papers3": {"battery": 8.8, "imu": 6, "sdcard": 12, "quickstart-bargraph": 7},
    "papermono": {"battery": 8.8, "button": 6.5, "imu": 6, "quickstart-bargraph": 7,
                  "display-2": 6.5, "nfc-1": 9, "nfc-2": 9},
    "papercolor": {"sht40": 14},
}


def digest(path):
    with path.open("rb") as f:
        return file_digest(f, "sha256").hexdigest()


def run(board, binary, firmware, out, cases, args):
    def one(case, kind):
        stem = case if kind == "input" else f"{case}-{kind}"
        image = args.stock if case == "stock" else firmware / case / "sketch.merged.bin"
        if image is None or not image.is_file():
            return {"case": case, "kind": kind, "state": "NOT RUN", "reason": "No pinned merged image"}
        png, log = (out / f"{stem}.{ext}" for ext in ("png", "log"))
        if board == "papercolor" and case == "sht40":
            limit = 14
        elif (board, case) in RTC_WIFI:
            limit = 12
        elif board == "papercolor":
            limit = 95 if case in {"battery", "button", "microsd", "ir_nec", "mic", "m5pm1-1", "wakeup-2"} else 40
        elif case in {"stock", "quickstart-longtextscroll"}:
            limit = 10
        elif case.startswith(("m5pm1_m5ioe1-", "nfc-")) or (board, case) == ("papermono", "wakeup"):
            limit = 23
        elif (board, case) == ("papers3", "wakeup"):
            limit = 13.5
        elif (board, case) == ("paperdiy", "m5pm1"):
            limit = 18
        else:
            limit = 13
        if kind == "early":
            limit = 8 if board == "papercolor" else 0.25
        elif kind == "first":
            limit = FIRST[board][case]
        elif kind == "control" and (board, case) in RTC_WIFI:
            limit = 8
        elif kind == "control" and case == "wakeup-1":
            limit = 4
        command = [str(binary), "--board", f"m5stack-{board}", "--boot", "rom",
                   "--flash-image", str(image), "--console", "both", "--max-seconds", str(limit),
                   "--tft-png", str(png), "--no-dump"]
        if case in {"m5pm1", "m5pm1-1", "m5pm1-2", "wakeup", "wakeup-1", "wakeup-2"} or case.startswith("m5pm1_m5ioe1-"):
            elf = image.with_name("sketch.elf")
            if elf.is_file():
                command += ["--elf", str(elf), "--trace-fn", "_Z4loopv"]
        debug = []
        if (board, case) in RTC_WIFI or case.startswith(("m5pm1", "nfc-")) or case in {"sht40", "rgb_led", "wakeup-2"}:
            debug.append("i2c")
        if case == "wakeup":
            debug.append("rtc-sleep")
        if board == "papercolor" and case == "rgb_led":
            debug.append("leds")
        if debug:
            command += ["--debug", ",".join(debug)]
        if case in {"ir_nec", "nfc-1", "nfc-2", "nfc-3", "m5pm1_m5ioe1-3", "m5pm1_m5ioe1-5", "m5pm1-2"}:
            command += ["--vcd", str(out / f"{stem}.vcd")]
        lines = list(INPUT.get(case, ()))
        if (board, case) == ("papermono", "display-2"):
            lines = ["4.0 touch 240 299 1", "4.5 touch 240 299 0",
                     "7.0 touch 240 423 1", "7.5 touch 240 423 0",
                     "10.0 touch 240 547 1", "10.5 touch 240 547 0"]
        elif case == "display-2":
            lines = ["4.0 press a 350", "8.0 press b 400"]
        if board == "papercolor" and case in {"button", "microsd", "m5pm1-1", "m5pm1-2", "wakeup-2"}:
            lines = ["35.0 press a 350", "55.0 press b 400", "75.0 press c 350"]
        if case == "battery" and board == "papers3":
            lines = ["5.0 adc 3 1.75", "10.0 adc 3 2.1"]
        elif case == "battery" and board == "papermono":
            lines = ["5.0 paper-battery 3678 0", "10.0 paper-battery 4200 1"]
        if case == "sht40":
            lines = ["0.1 sht40 32 71"] if kind == "input" else ["0.1 sht40 18 35"]
        if case in {"nfc-1", "nfc-2"}:
            uid = "04331144556677" if kind == "first" else "04998877665544"
            lines = [f"0.1 paper-tag {uid}"]
        elif case == "nfc-3":
            lines = ["0.1 paper-tag 04998877665544", "6.0 press b 100", "10.0 paper-tag none",
                     "12.0 paper-tag 04998877665544", "16.0 press a 100"]
        elif case in {"m5pm1_m5ioe1-1", "m5pm1_m5ioe1-2", "m5pm1_m5ioe1-3",
                      "m5pm1_m5ioe1-4", "m5pm1_m5ioe1-5", "wakeup"} and board == "papermono":
            lines = ["4.0 press a 350"]
        if case == "mic" and board == "papercolor" and kind != "control":
            lines = ["35.0 press a 2000"]
        if kind in {"input", "first", "second"}:
            if (board, case) in RTC_WIFI:
                command += ["--wifi", RTC_WIFI[board, case], "--net", f"none,epoch={RTC_EPOCH}"]
            if lines:
                script = out / f"{stem}.txt"
                script.write_text("\n".join(lines) + "\n", encoding="utf-8")
                command += ["--script", str(script)]
            if case in {"microsd", "sdcard"} and args.sd:
                card = out / f"{stem}.img"
                shutil.copyfile(args.sd, card)
                command += ["--sd-rw", str(card)]
            if case == "imu" or case in {"m5pm1_m5ioe1-2", "m5pm1_m5ioe1-3"}:
                command += ["--imu-motion", "1"]
            if case == "mic" and args.mic and kind == "input":
                command += ["--mic-wav", str(args.mic)]
            elif case == "mic" and kind == "second":
                command += ["--i2s-tone", f"{int(board == 'papercolor')}:1200:0.2"]
            elif case == "mic":
                command += ["--i2s-tone", f"{int(board == 'papercolor')}:700:0.05"]
        if case == "mic" and board == "papercolor" and kind == "control":
            script = out / f"{stem}.txt"
            script.write_text("35.0 press a 2000\n", encoding="utf-8")
            command += ["--script", str(script)]
        if case.startswith("lora") and args.lora_peer and kind != "control":
            command += ["--lora-peer", args.lora_peer]
        if "buzzer" in case or case == "speaker" or (case == "mic" and board == "papercolor"):
            command += ["--wav", str(out / f"{stem}.wav")]
        started = monotonic()
        with log.open("wb") as f:
            with subprocess.Popen(command, stdout=f, stderr=subprocess.STDOUT) as process:
                try:
                    status = process.wait(timeout=args.timeout)
                except subprocess.TimeoutExpired:
                    status = 124
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=15)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
        return {"case": case, "kind": kind, "exit": status,
                "wall_seconds": round(monotonic() - started, 3), "guest_limit_seconds": limit,
                "firmware_sha256": digest(image), "log_sha256": digest(log),
                "png_sha256": digest(png) if png.is_file() else None, "command": command}

    jobs = [(case, "input") for case in cases]
    jobs += [(case, "control") for case in cases if case in INPUT
             or case in {"display-2", "imu", "sdcard", "mic"} or (board, case) in RTC_WIFI
             or case.startswith(("nfc-", "m5pm1_m5ioe1-", "wakeup-"))
             or case in {"m5pm1-1", "m5pm1-2", "wakeup", "sht40"}]
    jobs += [(case, "early") for case in cases if case.startswith(("display", "quickstart"))]
    jobs += [(case, "first") for case in cases if case in FIRST[board]]
    jobs += [(case, "second") for case in cases if case == "mic"]
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        return list(pool.map(lambda job: one(*job), jobs))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("board", choices=BOARDS)
    parser.add_argument("binary", type=Path)
    parser.add_argument("firmware", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--rom-dir", required=True, type=Path)
    parser.add_argument("--stock", type=Path)
    parser.add_argument("--sd", type=Path)
    parser.add_argument("--mic", type=Path)
    parser.add_argument("--source", required=True, help="Commit used to compile the simulator")
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument("--timeout", type=int, default=900)
    parser.add_argument("--case", action="append", help="Run only this case. Repeat to select several.")
    parser.add_argument("--lora-peer", help="PaperMono loopback ports for all LoRa cases, IN:OUT")
    args = parser.parse_args()
    cases = BOARDS[args.board].split() + (["stock"] if args.stock else [])
    if args.lora_peer and (args.board != "papermono" or args.case not in (["lora-1"], ["lora-2"])):
        parser.error("--lora-peer needs one selected PaperMono LoRa case")
    if args.case:
        if not set(args.case) <= set(cases):
            parser.error("unknown case for this board")
        cases = args.case
    out = args.output.resolve()
    out.mkdir(parents=True)
    binary = out / "esp32sim"
    shutil.copy2(args.binary, binary)
    os.environ["ESP32SIM_ROM_DIR"] = str(args.rom_dir.resolve())
    results = run(args.board, binary, args.firmware.resolve(), out, cases, args)
    receipt = {"board": args.board, "commit": args.source, "binary_sha256": digest(binary),
               "rom_sha256": digest(args.rom_dir / "esp32s3_rev0_rom.elf"),
               "sd_sha256": digest(args.sd) if args.sd else None,
               "mic_sha256": digest(args.mic) if args.mic else None, "runs": results}
    (out / "runs.json").write_text(json.dumps(receipt, indent=2) + "\n")
    return int(any(row.get("exit", 0) != 0 for row in results))


if __name__ == "__main__":
    sys.exit(main())
