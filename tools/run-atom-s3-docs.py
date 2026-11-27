#!/usr/bin/env python3
"""Replay one Atom board's pinned Arduino docs and stock UIFlow2 image."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import shutil
import socket
import subprocess
import time

CASES = {
    "atoms3-lite": ("button", "ir_nec", "lite_led"),
    "atoms3": ("button", "display", "imu", "ir_nec", "quickstart-display"),
    "atoms3r": ("button", "display", "imu", "ir_nec", "quickstart-bargraph"),
    "atoms3u": ("button", "ir_nec", "mic", "program", "rgb_led"),
    "atom-echos3r": ("button", "ir", "mic", "program", "speaker"),
    "atoms3r-cam": ("quickstart-camera",),
    "atoms3r-m12": ("quickstart-camera",),
}
IMU = {"still": "", "a": "3 imu 0.5 -0.25 1.0 125 -50 25\n",
       "b": "3 imu -0.75 0.3 0.25 -100 80 -30\n"}
MIC = {"still": (), "a": (733, 0.008), "b": (1193, 0.04)}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def camera_input(dst, color, jpeg):
    size = "1600x1200" if jpeg else "320x240"
    subprocess.run(
        ["ffmpeg", "-hide_banner", "-loglevel", "error", "-f", "lavfi", "-i",
         f"color=c={color}:size={size}", "-threads", "1", "-frames:v", "1", "-update", "1", str(dst)],
        check=True,
    )


def stream_camera(proc, port, path):
    # Read bounded multipart HTTP bytes over the guest's normal Wi-Fi route.
    # The forwarded SYN can race guest DHCP/network setup, so retry until ready.
    until = time.monotonic() + 36
    while proc.poll() is None and time.monotonic() < until:
        data = bytearray()
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=1) as con:
                con.sendall(b"GET / HTTP/1.1\r\nHost: atom\r\nConnection: close\r\n\r\n")
                con.settimeout(3)
                while len(data) < 1 << 20:
                    part = con.recv(32 * 1024)
                    if not part:
                        break
                    data.extend(part)
                    if data.count(b"\xff\xd9") >= 2:
                        path.write_bytes(data)
                        return
        except OSError:
            if data:
                path.write_bytes(data)
        time.sleep(0.4)


def run(args, label, firmware, seconds, options=(), script="", stream_port=None):
    out = args.out
    action = out / (label + ".txt")
    if script:
        action.write_text(script)
        options = [*options, "--script", str(action)]
    cmd = [str(out / "esp32sim"), "--board", "m5stack-" + args.variant, "--boot", "rom",
           "--rom", str(args.rom), "--flash-image", str(firmware), "--console", "both",
           "--no-dump", "--timeout", "110", "--max-seconds", str(seconds),
           "--tft-png", str(out / (label + ".png")), "--wav", str(out / (label + ".wav")),
           *options]
    with (out / (label + ".log")).open("wb") as log, (out / (label + ".err")).open("wb") as err:
        with subprocess.Popen(cmd, stdout=log, stderr=err) as proc:
            if stream_port is not None:
                stream_camera(proc, stream_port, out / (label + ".http"))
            try:
                status = proc.wait(timeout=120)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
                status = 124
    (out / (label + ".json")).write_text(json.dumps(
        {"command": cmd, "exit": status, "firmware_sha256": sha(firmware)}, indent=2) + "\n")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("board", choices=CASES)
    p.add_argument("binary", type=Path)
    p.add_argument("firmware", type=Path, help="parent of per-board dirs containing CASE/sketch.merged.bin")
    p.add_argument("stock", type=Path, help="catalog UIFlow2 merged image for the selected board")
    p.add_argument("rom", type=Path, help="ESP32-S3 ROM ELF")
    p.add_argument("out", type=Path, help="new evidence directory")
    p.add_argument("--source", required=True, help="simulator source commit for BINARY")
    p.add_argument("--gc9107", action="store_true", help="select the GC9107 LCD revision")
    args = p.parse_args()
    if args.gc9107 and args.board not in {"atoms3", "atoms3r"}:
        p.error("--gc9107 is available for AtomS3 and AtomS3R")
    args.variant = args.board + ("-gc9107" if args.gc9107 else "")
    args.out = args.out.resolve()
    args.out.mkdir(parents=True)
    shutil.copyfile(args.binary, args.out / "esp32sim")
    (args.out / "esp32sim").chmod(0o755)
    (args.out / "run.json").write_text(json.dumps(
        {"source": args.source, "board": args.board, "variant": args.variant,
         "binary_sha256": sha(args.binary), "rom_sha256": sha(args.rom)}, indent=2) + "\n")
    stock = args.out / "usb-repl.txt"
    poll_button = ('import time\nfor _ in range(48):\n'
                   " M5.update(); print('ATOM_BTN', int(M5.BtnA.isPressed())); time.sleep_ms(100)")
    stock.write_text(
        "attach 8000\ncdc 4500\ncdc 100 \x03\n"
        'cdc 1000 import M5; print("ATOM_BOARD", M5.getBoard())\n'
        f"cdc 6000 exec({json.dumps(poll_button)})\n"
    )
    button = "3 press btn 200\n4 press btn 200\n"
    tasks = []
    for case in CASES[args.board]:
        fw = args.firmware / args.board / case / "sketch.merged.bin"
        if case == "quickstart-camera":
            jpeg = args.board == "atoms3r-m12"
            for suffix, color in (("still", "black"), ("a", "red"), ("b", "green")):
                dst = args.out / f"camera-{suffix}.{'jpg' if jpeg else 'bmp'}"
                camera_input(dst, color, jpeg)
                # Find an unused loopback port for the owned simulator process.
                with socket.socket() as s:
                    s.bind(("127.0.0.1", 0))
                    port = s.getsockname()[1]
                opts = ("--cam-image", str(dst), "--wifi", "ssid=yourssid,psk=yourpasswd",
                        "--hostfwd", f"tcp:127.0.0.1:{port}-:80", "--realtime", "--pwm", "21")
                run(args, f"{case}-{suffix}", fw, 14, opts, stream_port=port)
        elif case in ("imu", "mic"):
            for suffix in IMU:
                script = IMU[suffix] if case == "imu" else "5 press btn 180\n"
                tone = MIC[suffix]
                port = int(args.board == "atom-echos3r")
                opts = ("--i2s-tone", f"{port}:{tone[0]}:{tone[1]}") if case == "mic" and tone else ()
                tasks.append((f"{case}-{suffix}", fw, 10, opts, script))
        else:
            tasks.append((case, fw, 7, (), button if case in ("button", "rgb_led", "speaker") else ""))
            if case == "button":
                tasks.append(("button-still", fw, 7, (), ""))
    tasks.append(("stock", args.stock, 16, ("--usb-host", str(stock)),
                  "7.8 press btn 1000\n9.4 press btn 1000\n"))
    with ThreadPoolExecutor(3) as workers:
        runs = [workers.submit(run, args, *task) for task in tasks]
        for result in runs:
            result.result()
    subprocess.run(["python3", str(Path(__file__).with_name("check-atom-s3-docs.py")),
                    str(args.out)], check=True)


if __name__ == "__main__":
    main()
