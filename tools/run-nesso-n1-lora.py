#!/usr/bin/env python3
"""Replay the prebuilt Nesso RadioLib TX/RX examples on two local SX1262 peers."""
import argparse
from contextlib import ExitStack
from pathlib import Path
import json
from runpy import run_path
import socket
import subprocess

sha = run_path(str(Path(__file__).with_name("run-c6-docs.py")))["sha"]


def run(args):
    args.output.mkdir(parents=True)
    with socket.socket(type=socket.SOCK_DGRAM) as a, socket.socket(type=socket.SOCK_DGRAM) as b:
        a.bind(("127.0.0.1", 0))
        b.bind(("127.0.0.1", 0))
        ports = a.getsockname()[1], b.getsockname()[1]
    if set(ports) & {8787, 8830, 18787, 18853}:
        raise RuntimeError("reserved port selected")
    with ExitStack() as stack:
        children = []
        cases = {}
        for side, local, remote in ((2, ports[1], ports[0]), (1, *ports)):
            label = f"lora-{side}"
            image = args.firmware_dir / label / "sketch.merged.bin"
            command = [str(args.binary), "--chip", "c6", "--board", "arduino-nesso-n1",
                       "--boot", "rom", "--rom", str(args.rom), "--flash-image", str(image),
                       "--realtime", "--max-seconds", "16", "--console", "usb", "--no-dump"]
            if not args.isolated:
                command += ["--cap-peer", f"{local}:{remote}"]
            logfile = stack.enter_context((args.output / f"{label}.log").open("wb"))
            errfile = stack.enter_context((args.output / f"{label}.err").open("wb"))
            proc = subprocess.Popen(command, stdout=logfile, stderr=errfile)
            stack.callback(proc.wait)
            stack.callback(lambda p=proc: p.kill() if p.poll() is None else None)
            children.append((label, proc))
            cases[label] = {"firmware_sha256": sha(image), "command": command}
        for label, proc in children:
            cases[label]["exit"] = proc.wait(timeout=75)
    (args.output / "run.json").write_text(json.dumps({
        "commit": args.commit, "binary_sha256": sha(args.binary), "rom_sha256": sha(args.rom),
        "isolated": args.isolated, "cases": cases,
    }, indent=2) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("binary", "rom", "firmware_dir", "output"):
        parser.add_argument(name, type=Path)
    parser.add_argument("--commit", required=True)
    parser.add_argument("--isolated", action="store_true")
    run(parser.parse_args())
