#!/usr/bin/env python3
"""Drive ROM-booted NanoC6 guests over Cooja NDJSON, with an optional peer."""
import argparse
from contextlib import ExitStack
from pathlib import Path
import json
import select
import subprocess

from runpy import run_path
sha = run_path(str(Path(__file__).with_name("run-c6-docs.py")))["sha"]

SLICE_NS = 100_000


def exchange(proc, request):
    proc.stdin.write(json.dumps(request).encode() + b'\n')
    proc.stdin.flush()
    if not select.select([proc.stdout], [], [], 20)[0]:
        raise TimeoutError("mote produced no Cooja reply")
    reply = json.loads(proc.stdout.readline())
    if reply["type"] != "done":
        raise RuntimeError(f"unexpected reply {reply}")
    return reply


def run(args):
    args.output.mkdir(parents=True)
    if not 1 <= len(args.images) <= 2:
        raise ValueError("provide one or two images")
    (args.output / "inputs.json").write_text(json.dumps({
        "commit": args.commit, "binary_sha256": sha(args.binary), "rom_sha256": sha(args.rom),
        "firmware_sha256": [sha(image) for image in args.images],
        "seconds": args.seconds, "isolated": args.isolated, "board": args.board,
        "resets": args.reset, "observe_leds": args.observe_leds,
        "scripts": {node: sha(Path(path)) for node, path in args.script},
    }, indent=2) + "\n")
    scripts = {int(node): path for node, path in args.script}
    with ExitStack() as stack:
        motes = []
        serial = []
        for index, image in enumerate(args.images):
            stderr = stack.enter_context((args.output / f"node{index}.stderr").open("wb"))
            serial.append(stack.enter_context((args.output / f"node{index}.log").open("w")))
            command = [str(args.binary), "--chip", "c6", "--board", args.board,
                       "--rom", str(args.rom), "--flash-image", str(image), "--boot", "rom",
                       "--console", args.console, "--no-dump", "--cooja"]
            if index in scripts:
                command += ["--script", scripts[index]]
            proc = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                    stderr=stderr)
            stack.callback(proc.wait)
            stack.callback(lambda p=proc: p.kill() if p.poll() is None else None)
            motes.append(proc)
            exchange(proc, {"type": "hello", "id": index + 1, "seed": 1,
                            "args": {"observe_leds": args.observe_leds}})
        trace = stack.enter_context((args.output / "radio.ndjson").open("w"))
        last_channel = [None] * len(motes)
        pending = [[] for _ in motes]
        now = [0] * len(motes)
        inputs = {}
        for second, node, text in args.serial:
            inputs.setdefault(int(float(second) * 1e9), []).append(
                (int(node), (text + "\r").encode().hex()))
        for t, events in inputs.items():
            for node, data in events:
                pending[node].append({"type": "serial", "t": t, "data": data})
        for second, node in args.reset:
            pending[int(node)].append({"type": "reset", "t": round(float(second) * 1e9)})
        end = round(args.seconds * 1e9)
        while min(now) < end:
            index = min(range(len(motes)), key=now.__getitem__)
            peer = 1 - index
            horizon = min(end, now[index] + SLICE_NS)
            due = [ev for ev in pending[index] if ev["t"] <= horizon]
            pending[index] = [ev for ev in pending[index] if ev["t"] > horizon]
            reply = exchange(motes[index], {"type": "step", "t": horizon, "in": due})
            now[index] = reply["t"]
            for ev in reply["out"]:
                if ev["type"] == "log":
                    print(f'{ev["t"] / 1e9:.6f} {ev["line"]}', file=serial[index], flush=True)
                else:
                    print(json.dumps({"node": index, **ev}), file=trace)
                    if "ch" in ev:
                        last_channel[index] = ev["ch"]
                    if ev["type"] == "tx" and len(motes) == 2 and not args.isolated:
                        if last_channel[peer] == ev["ch"]:
                            pending[peer].append({"type": "rx", "t": ev["t"],
                                                  "frame": ev["frame"], "rssi": -55, "lqi": 255})
        for proc in motes:
            proc.stdin.write(b'{"type":"stop"}\n')
            proc.stdin.close()
            if proc.wait(timeout=10):
                raise RuntimeError("mote exited with an error")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("binary", type=Path)
    parser.add_argument("rom", type=Path)
    parser.add_argument("images", type=Path, nargs="+")
    parser.add_argument("output", type=Path)
    parser.add_argument("--board", choices=("m5stack-nanoc6", "arduino-nesso-n1"), default="m5stack-nanoc6")
    parser.add_argument("--console", choices=("usb", "uart0"), default="usb")
    parser.add_argument("--commit", required=True)
    parser.add_argument("--seconds", type=float, default=45)
    parser.add_argument("--isolated", action="store_true", help="negative control without forwarding")
    parser.add_argument("--observe-leds", action="store_true")
    parser.add_argument("--reset", action="append", nargs=2, metavar=("SECONDS", "NODE"), default=[])
    parser.add_argument("--script", action="append", nargs=2, metavar=("NODE", "FILE"), default=[])
    parser.add_argument("--serial", action="append", nargs=3, metavar=("SECONDS", "NODE", "TEXT"), default=[])
    run(parser.parse_args())
