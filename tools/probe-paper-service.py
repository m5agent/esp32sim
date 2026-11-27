#!/usr/bin/env python3
"""Capture stock UIFlow2 through the simulator's Device Service transport."""
import argparse
from hashlib import sha256
import json
from pathlib import Path
import socket
import struct
import subprocess
from threading import Thread
from time import monotonic, sleep

from importlib.util import spec_from_file_location, module_from_spec

spec = spec_from_file_location("runner", Path(__file__).with_name("run-paper-docs.py"))
runner = module_from_spec(spec)
spec.loader.exec_module(runner)


def receive(sock):
    with sock.makefile("rb") as stream:
        while header := stream.read(2):
            first, second = header
            size = second & 127
            if size == 126:
                size, = struct.unpack(">H", stream.read(2))
            elif size == 127:
                size, = struct.unpack(">Q", stream.read(8))
            yield first & 15, stream.read(size)


def probe(board, binary, image, out, seconds):
    parent, child = socket.socketpair()
    events, frames, replies = [], {}, {}

    def collect():
        for opcode, data in receive(parent):
            if opcode == 1:
                msg = json.loads(data)
                if msg.get("t") == "service":
                    replies[msg["id"]] = msg
                events.append(msg)
            elif opcode == 2 and data[:1] == b"\x06":
                ident, kind, w, h = struct.unpack_from("<IBHH", data, 1)
                if kind == 1 and len(data) == 10 + 2 * w * h:
                    pixels = data[10:]
                    (out / f"frame-{ident}.rgb565").write_bytes(pixels)
                    frames[ident] = {"size": [w, h], "colors": len(set(struct.iter_unpack("<H", pixels))),
                                     "sha256": sha256(pixels).hexdigest()}

    def send(**data):
        msg = json.dumps(data).encode()
        parent.sendall(struct.pack(">I", len(msg)) + msg)

    with parent, child, (out / "serial.log").open("wb") as log:
        command = [str(binary), "--board", f"m5stack-{board}", "--boot", "rom", "--flash-image", str(image),
                   "--service-fd", str(child.fileno()), "--console", "both", "--no-dump"]
        with subprocess.Popen(command, pass_fds=(child.fileno(),), stdout=log, stderr=subprocess.STDOUT) as process:
            child.close()
            thread = Thread(target=collect, daemon=True)
            thread.start()
            start = monotonic()
            try:
                send(t="service", id=1, op="hello")
                send(t="service", id=2, op="frame")
                deadline = start + seconds
                while monotonic() < deadline:
                    send(t="service", id=3, op="frame")
                    sleep(2)
                    if frames.get(3, {}).get("colors", 0) > 1:
                        break
                if board in {"papers3", "papermono"}:
                    send(t="service", id=4, op="touch", args="24 158 1 300" if board == "papers3" else "22 128 1 300")
                elif board == "paperdiy":
                    send(t="btn", pin=4, v=1)
                    sleep(0.2)
                    send(t="btn", pin=4, v=0)
                elif board == "papercolor":
                    send(t="btn", pin=10, v=1)
                    sleep(0.2)
                    send(t="btn", pin=10, v=0)
                deadline = monotonic() + 12
                while monotonic() < deadline:
                    send(t="service", id=5, op="frame")
                    sleep(2)
                    if frames.get(5, {}).get("sha256") not in (None, frames.get(3, {}).get("sha256")):
                        break
                send(t="key", data="\r\x03\x03\r")
                sleep(0.5)
                send(t="key", data="print('S3D_REPL_OK', 18*7)\r")
                sleep(1)
                send(t="key", src="uart0", data="\r\x03\x03\r")
                sleep(0.5)
                send(t="key", src="uart0", data="print('S3D_UART_OK', 19*7)\r")
                sleep(1)
            finally:
                parent.shutdown(socket.SHUT_RDWR)
                thread.join(timeout=5)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    process.wait(timeout=5)
    serial = (out / "serial.log").read_text(errors="replace")
    serial += "".join(str(msg.get("data", "")) for msg in events if msg.get("t") == "serial")
    result = {
        "board": board, "wall_seconds": round(monotonic() - start, 2), "reply": replies,
        "frames": frames, "exit": process.returncode,
        "usb_repl": "S3D_REPL_OK 126" in serial, "uart_repl": "S3D_UART_OK 133" in serial,
        "firmware_sha256": runner.digest(image), "binary_sha256": runner.digest(binary),
    }
    (out / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    (out / "events.json").write_text(json.dumps(events) + "\n")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("board", choices=runner.BOARDS)
    p.add_argument("binary", type=Path)
    p.add_argument("image", type=Path)
    p.add_argument("out", type=Path)
    p.add_argument("--seconds", type=float, default=22)
    args = p.parse_args()
    args.out.mkdir(parents=True)
    probe(args.board, args.binary.resolve(), args.image.resolve(), args.out, args.seconds)
