#!/usr/bin/env python3
"""Boot each pinned Arduino case from ROM and retain its input and output."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from hashlib import file_digest
import json
from pathlib import Path
import subprocess

NANO = ("button", "ir_nec", "led", "program", *(f"thread-{i}" for i in range(1, 7)),
        *(f"zigbee-{i}" for i in range(1, 6)))
NESSO = ("button", "buzzer", "display", "imu", "ir_nec", "led", "lora-1", "lora-2",
         "power-1", "power-2", "quickstart-bargraph", "touch")
SCRIPTS = {
    ("nanoc6", "button"): "1 press btnA 100\n2 press btnA 1100\n",
    ("nanoc6", "thread-5"): "3 serial version\n5 serial state\n",
    ("nanoc6", "stock"): "5 press btnA 180\n",
    ("nesso-n1", "button"): "2 key A down\n2.4 key A up\n4 key B down\n4.4 key B up\n",
    ("nesso-n1", "touch"): "2 touch 30 75 1\n3 touch 30 75 0\n4 touch 104 126 1\n",
    ("nesso-n1", "stock"): "5 key A down\n5.4 key A up\n8 touch 66 150 1\n8.3 touch 66 150 0\n",
}


def sha(path):
    with path.open("rb") as f:
        return file_digest(f, "sha256").hexdigest()


def run(board):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("binary", type=Path)
    parser.add_argument("rom", type=Path)
    parser.add_argument("firmware_dir", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--stock", type=Path)
    parser.add_argument("--case", action="append", help="run only the named case")
    parser.add_argument("--wall-timeout", type=int, default=180)
    parser.add_argument("--jobs", type=int, default=3)
    parser.add_argument("--commit", required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True)
    is_nesso = board == "nesso-n1"
    profile = "arduino-nesso-n1" if is_nesso else "m5stack-nanoc6"
    horizons = {"display": 1.0, "quickstart-bargraph": 2.0, "touch": 4.5,
                "button": 5.0, "thread-1": 20.0, "thread-6": 20.0}
    cases = [(case, args.firmware_dir / case / "sketch.merged.bin", horizons.get(case, 8.0), None, [])
             for case in (NESSO if is_nesso else NANO)]
    for case in ("button", "touch", "imu") if is_nesso else ("button",):
        cases.append((case + "-control", args.firmware_dir / case / "sketch.merged.bin", 3.0, "", []))
    if is_nesso:
        cases.append(("imu-motion", args.firmware_dir / "imu/sketch.merged.bin", 12.0, "",
                      ["--imu-motion", "1"]))
        cases.append(("touch-first", args.firmware_dir / "touch/sketch.merged.bin", 2.5,
                      SCRIPTS["nesso-n1", "touch"], []))
        cases.append(("power-other", args.firmware_dir / "power-2/sketch.merged.bin", 8.5, "", []))
    else:
        cases.append(("program-other", args.firmware_dir / "program/sketch.merged.bin", 8.1, "", []))
    if args.stock:
        cases.append(("stock", args.stock, 14.0, None, []))

    if args.case:
        cases = [row for row in cases if row[0] in args.case]
        if len(cases) != len(set(args.case)):
            parser.error("unknown or unavailable --case")

    def run_case(item):
        label, image, seconds, script, extra = item
        if not image.is_file():
            return {"case": label, "status": "NOT RUN", "reason": "prebuilt image unavailable"}
        command = [str(args.binary.resolve()), "--chip", "c6", "--board", profile, "--rom", str(args.rom),
                   "--flash-image", str(image), "--boot", "rom", "--max-seconds", str(seconds),
                   "--console", "usb", "--no-dump", *extra]
        if is_nesso:
            command += ["--tft-png", str(args.output / f"{label}.png")]
        if label == "buzzer":
            command += ["--wav", str(args.output / "buzzer.wav")]
        script = SCRIPTS.get((board, label)) if script is None else script
        if script:
            path = args.output / f"{label}.actions"
            path.write_text(script)
            command += ["--script", str(path)]
        with (args.output / f"{label}.log").open("wb") as log, (args.output / f"{label}.err").open("wb") as err:
            try:
                status = subprocess.run(command, stdout=log, stderr=err, timeout=args.wall_timeout).returncode
            except subprocess.TimeoutExpired:
                status = 124
        return {"case": label, "exit": status, "firmware_sha256": sha(image), "command": command}

    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        receipts = list(pool.map(run_case, cases))
    (args.output / "run.json").write_text(json.dumps({
        "commit": args.commit, "profile": profile, "binary_sha256": sha(args.binary),
        "rom_sha256": sha(args.rom), "cases": receipts,
    }, indent=2) + "\n")
