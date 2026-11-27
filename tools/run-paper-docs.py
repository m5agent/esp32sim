#!/usr/bin/env python3
"""Replay pinned S3 e-paper sketches. A boot receipt alone carries no PASS verdict."""

import argparse
from concurrent.futures import ThreadPoolExecutor
from hashlib import file_digest
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

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
    "wakeup": ["8.0 touch 80 180 1", "8.4 touch 80 180 0"],
    "sht40": ["0.1 sht40 18 35", "115.0 sht40 32 71"],
    "m5pm1": ["4.0 press a 350", "9.0 press b 350"],
    "microsd": ["6.0 press a 300", "10.0 press b 300"],
    "battery": ["35.0 paper-battery 3678 0", "65.0 paper-battery 4200 1"],
    "nfc-1": ["5.0 paper-tag 04512233", "8.0 paper-tag none", "11.0 paper-tag 04998877665544"],
}

DIMS = {"paperdiy": (540, 960), "papers3": (540, 960), "papermono": (480, 800), "papercolor": (400, 600)}
RTC_WIFI = {
    ("papers3", "rtc"): "ssid=YOUR WIFI SSID NAME,psk=YOUR WIFI PASSWORD",
    ("papermono", "rtc-2"): "ssid=ssid,psk=password",
    ("papercolor", "rtc-2"): "ssid=ssid,psk=password",
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
            limit = 145
        elif board == "papercolor":
            limit = 95 if case in {"battery", "button", "microsd", "ir_nec"} else 40
        elif case in {"stock", "quickstart-longtextscroll"}:
            limit = 10
        else:
            limit = 13
        if kind == "early":
            limit = 8 if board == "papercolor" else 0.25
        command = [str(binary), "--board", f"m5stack-{board}", "--boot", "rom",
                   "--flash-image", str(image), "--console", "both", "--max-seconds", str(limit),
                   "--tft-png", str(png), "--no-dump"]
        lines = list(INPUT.get(case, ()))
        if case == "display-2":
            lines = ["4.0 press a 350", "8.0 press b 400"]
        if board == "papercolor" and case in {"button", "microsd", "ir_nec"}:
            lines = ["35.0 press a 350", "55.0 press b 400", "75.0 press c 350"]
        if case == "battery" and board == "papers3":
            lines = ["5.0 adc 3 1.75", "10.0 adc 3 2.1"]
        elif case == "battery" and board == "papermono":
            lines = ["5.0 paper-battery 3678 0", "10.0 paper-battery 4200 1"]
        if kind == "input":
            if (board, case) in RTC_WIFI:
                command += ["--wifi", RTC_WIFI[board, case], "--net", "none"]
            if lines:
                script = out / f"{stem}.txt"
                script.write_text("\n".join(lines) + "\n", encoding="utf-8")
                command += ["--script", str(script)]
            if case in {"microsd", "sdcard"} and args.sd:
                card = out / f"{stem}.img"
                shutil.copyfile(args.sd, card)
                command += ["--sd-rw", str(card)]
            if case == "imu":
                command += ["--imu-motion", "1"]
            if case == "mic" and args.mic:
                command += ["--mic-wav", str(args.mic)]
        if case.startswith("lora") and args.lora_peer:
            command += ["--lora-peer", args.lora_peer]
        if "buzzer" in case or case == "speaker":
            command += ["--wav", str(out / f"{stem}.wav")]
        with log.open("wb") as f:
            try:
                status = subprocess.run(command, stdout=f, stderr=subprocess.STDOUT, timeout=args.timeout, check=False).returncode
            except subprocess.TimeoutExpired:
                status = 124
        return {"case": case, "kind": kind, "exit": status,
                "firmware_sha256": digest(image), "log_sha256": digest(log),
                "png_sha256": digest(png) if png.is_file() else None, "command": command}

    jobs = [(case, "input") for case in cases]
    jobs += [(case, "control") for case in cases if case in INPUT
             or case in {"display-2", "imu", "sdcard", "mic"} or (board, case) in RTC_WIFI]
    jobs += [(case, "early") for case in cases if case.startswith(("display", "quickstart"))]
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
    parser.add_argument("--timeout", type=int, default=180)
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
