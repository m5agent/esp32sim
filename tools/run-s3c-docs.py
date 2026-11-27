#!/usr/bin/env python3
"""Run pinned StamPLC or StackChan documentation firmware with local fixtures."""

import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import shutil
import subprocess

from check_s3c import check_matrix


def fingerprint(path):
    with path.open("rb") as file:
        from hashlib import file_digest
        return file_digest(file, "sha256").hexdigest()


def playback_script():
    return "3.0 touch 164 116 1\n3.2 touch 164 116 0\n6.0 touch 164 116 1\n6.2 touch 164 116 0\n"


PLC = {
    "button": "2 plc-key a 200\n3 plc-key b 200\n4 plc-key c 200\n",
    "input_output-2": "2 plc-input 0 1\n2 plc-input 7 1\n4 plc-input 0 0\n4 plc-input 3 1\n",
    "modbus-1": "4 plc-modbus-coil 2 1\n6 plc-modbus-coil 2 0\n8 plc-modbus-coil 1 1\n",
    "sensor": "2 plc-temperature -6250\n2 plc-supply 11800 300\n4 plc-temperature 38125\n4 plc-supply 15375 700\n",
}
STACK = {
    "battery": "3 base-supply 7625 130\n",
    "button": "3 pwrkey 200\n4 touch 55 225 1\n4.2 touch 55 225 0\n5 touch 160 225 1\n5.2 touch 160 225 0\n6 touch 267 225 1\n6.2 touch 267 225 0\n",
    "imu": "3 imu 0.50 -0.25 1.0 125 -50 25\n",
    "ltr553": "3 als 1200 300 321\n",
    "mic": playback_script(),
    "servo-1": ("2.6 servo-pose 588 748\n3 touch 160 55 1\n3.2 touch 160 55 0\n"
                "3.6 servo-pose 460 620\n3.8 servo-angle\n"
                "4 touch 160 175 1\n4.2 touch 160 175 0\n4.8 servo-angle\n"),
    "servo-2": "3 head-touch 0 3\n3.4 head-touch 0 0\n4.7 servo-angle\n5 head-touch 1 2\n5.4 head-touch 1 0\n6.8 servo-angle\n",
    "touch": "3 touch 30 30 1\n3.2 touch 70 70 1\n3.4 touch 70 70 0\n4 touch 100 120 1\n4.2 touch 100 120 0\n",
    "touchsensor": "3 head-touch 0 2\n3.5 head-touch 0 0\n4 head-touch 2 3\n",
    "wakeup": "6 touch 160 120 1\n6.2 touch 160 120 0\n",
}
NFC_TOUCH = ("3.2 touch 155 105 1\n3.4 touch 155 105 0\n"
             "5.2 touch 155 105 1\n5.4 touch 155 105 0\n")
STACK.update({f"nfc-{index}": "3 nfc-tag 04a1b2c3d4e5f6\n5 nfc-tag 0456abcd123456\n" + NFC_TOUCH
              for index in (1, 4, 5, 6)})
STACK["nfc-2"] = ("3 nfc-tag 04a1b2c3d4e5f6\n3.2 touch 45 239 1\n3.4 touch 45 239 0\n"
                  "5 nfc-tag 0456abcd123456\n5.2 touch 45 239 1\n5.4 touch 45 239 0\n")
ALTERNATE = {
    "battery": "3 base-supply 6850 290\n",
    "imu": "3 imu -0.75 0.30 0.25 -100 80 -30\n",
    "ltr553": "3 als 4100 270 1456\n",
    "touch": "3 touch 240 40 1\n3.4 touch 240 40 0\n",
    "touchsensor": ("3 head-touch 0 1\n3.10 head-touch 1 2\n3.22 head-touch 2 3\n"
                    "3.4 head-touch 0 0\n3.4 head-touch 1 0\n3.4 head-touch 2 0\n"),
}
DURATIONS = {"display": 1.5, "servo-1": 6, "servo-2": 7, "imu": 5, "battery": 5,
             "camera": 5, "ltr553": 5, "touch": 5, "touchsensor": 5, "can": 5,
             "sensor": 6, "rtc": 11, "pic": 6, "sdcard": 6, "modbus-1": 10,
             "modbus-2": 12, "wakeup": 13, "wav": 16}
CONTROL_CASES = {
    "stamplc": {"button", "input_output-2", "modbus-1", "sd_card", "sensor"},
    "stackchan": {"battery", "button", "imu", "ltr553", "mic", "nfc-1", "pic", "sdcard",
                 "servo-2", "touch", "touchsensor", "wakeup"},
}


def run_case(config, binary, case, variant, image):
    label = case if variant == "main" else f"{case}-{variant}"
    out = config.out
    script = (PLC if config.board == "stamplc" else STACK).get(case, "")
    if variant == "alternate":
        script = ALTERNATE.get(case, script)
    elif variant == "control" and case not in ("mic",):
        script = NFC_TOUCH if case == "nfc-1" else ""
    if case == "stock":
        script = ("6 plc-key a 200\n" if config.board == "stamplc" else
                  "3.5 touch 265 110 1\n7.0 touch 265 110 0\n")
        usb_script = out / f"{label}-usb.script"
        usb_script.write_text("idle 8500\nattach 3000\ncdc 200 \x03\ncdc 1000 print('S3C_REPL_OK')\n")
    duration = 14 if case == "stock" else DURATIONS.get(case, 8)
    if case in ("pic", "sdcard", "sd_card") and variant == "control":
        duration = min(duration, 4.5)
    if case == "wakeup" and variant == "sleep":
        duration = 8
    args = [str(binary), "--board", f"m5stack-{config.board}", "--boot", "rom", "--flash-image",
            str(image), "--console", "both" if case == "stock" else "usb", "--max-seconds", str(duration),
            "--tft-png", str(out / f"{label}.png"), "--no-dump"]
    inputs = {"firmware": fingerprint(image)}
    if script:
        script_path = out / f"{label}.script"
        script_path.write_text(script)
        inputs["script"] = fingerprint(script_path)
        args += ["--script", str(script_path)]
    if case in ("sd_card", "sdcard", "pic", "wav") and config.sd_seed and variant != "control":
        sd = out / f"sd-{label}.img"
        shutil.copyfile(config.sd_seed, sd)
        inputs["sd_before"] = fingerprint(sd)
        args += ["--sd-rw", str(sd)]
    if case == "stock":
        inputs["usb_script"] = fingerprint(usb_script)
        args += ["--usb-host", str(usb_script)]
    if case == "camera":
        camera = config.camera_b if variant == "alternate" else config.camera_a
        if camera:
            camera_copy = out / f"{label}-input.bmp"
            shutil.copyfile(camera, camera_copy)
            inputs["camera"] = fingerprint(camera_copy)
            args += ["--cam-image", str(camera_copy)]
    if case == "mic":
        mic = None if variant == "control" else config.mic_b if variant == "alternate" else config.mic
        if mic:
            inputs["mic"] = fingerprint(mic)
            args += ["--mic-wav", str(mic)]
    if case in ("mic", "speaker", "wav", "buzzer"):
        args += ["--wav", str(out / f"{label}.wav")]
    if case == "buzzer":
        args += ["--vcd", str(out / f"{label}.vcd")]
    if case == "can":
        args += ["--debug", "TWAI"]
    if case == "rtc" and config.board == "stackchan" and config.rtc_wifi:
        args += ["--wifi", f"{config.rtc_wifi},epoch=1764000000", "--net", "none"]
    with (out / f"{label}.log").open("w") as log, (out / f"{label}.err").open("w") as err:
        try:
            status = subprocess.run(args, stdout=log, stderr=err, timeout=config.timeout).returncode
        except subprocess.TimeoutExpired:
            status = 124
    for suffix in ("log", "err", "png", "wav"):
        path = out / f"{label}.{suffix}"
        if path.exists():
            inputs[suffix] = fingerprint(path)
    (out / f"{label}.json").write_text(json.dumps({"status": status, "seconds": duration, "sha256": inputs}, indent=2) + "\n")
    print(f"{config.board} {label} exited {status}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("board", choices=("stamplc", "stackchan"))
    parser.add_argument("binary", type=Path)
    parser.add_argument("out", type=Path)
    parser.add_argument("--firmware", type=Path, required=True, help="directory containing case/sketch.merged.bin")
    parser.add_argument("--inventory", type=Path, required=True)
    parser.add_argument("--commit", required=True)
    parser.add_argument("--stock", type=Path)
    parser.add_argument("--sd-seed", type=Path)
    parser.add_argument("--camera-a", type=Path)
    parser.add_argument("--camera-b", type=Path)
    parser.add_argument("--mic", type=Path)
    parser.add_argument("--mic-b", type=Path)
    parser.add_argument("--rtc-wifi")
    parser.add_argument("--jobs", type=int, default=3)
    parser.add_argument("--timeout", type=int, default=180)
    parser.add_argument("--case", action="append", help="select an individual case, including stock")
    config = parser.parse_args()
    config.out = config.out.resolve()
    config.out.mkdir(parents=True)
    binary = config.out / "esp32sim"
    shutil.copyfile(config.binary, binary)
    binary.chmod(0o755)
    rom = Path(os.environ["ESP32SIM_ROM_DIR"]) / "esp32s3_rev0_rom.elf"
    inventory = json.loads(config.inventory.read_text())
    (config.out / "run.json").write_text(json.dumps({"commit": config.commit, "binary": fingerprint(binary),
        "rom": fingerprint(rom), "board": f"m5stack-{config.board}", "docs": inventory["docs_revision"]}, indent=2) + "\n")
    cases = [row for row in inventory["cases"] if row["device"] == config.board]
    jobs = []
    for row in cases:
        case = row["case"]
        if config.case and case not in config.case:
            continue
        image = config.firmware / case / "sketch.merged.bin"
        if not image.is_file():
            continue
        jobs.append((case, "main", image))
        if case in CONTROL_CASES[config.board]:
            jobs.append((case, "control", image))
        if config.board == "stackchan" and case in ALTERNATE.keys() | {"camera", "mic"}:
            jobs.append((case, "alternate", image))
        if case == "wakeup":
            jobs.append((case, "sleep", image))
    if config.stock and (not config.case or "stock" in config.case):
        jobs.append(("stock", "main", config.stock))
    with ThreadPoolExecutor(max_workers=config.jobs) as pool:
        futures = [pool.submit(run_case, config, binary, *job) for job in jobs]
        for future in futures:
            future.result()
    return check_matrix(config.board, config.out, cases, config.case)


if __name__ == "__main__":
    raise SystemExit(main())
