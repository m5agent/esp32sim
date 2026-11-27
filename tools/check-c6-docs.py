#!/usr/bin/env python3
"""Check guest and board markers from the C6 docs and peer runners."""
import argparse
import json
from pathlib import Path
import re
import wave


def read(directory, name):
    try:
        return (directory / name).read_text(errors="replace")
    except FileNotFoundError:
        return ""


def number(text, marker):
    match = re.search(rf"\b{marker}=(\d+)", text)
    return int(match[1]) if match else 0


def alive_board(text):
    return "lit=true" in text and number(text, "pixels") > 32000


def rgb(text):
    match = re.search(r"\brgb=\[([0-9, ]+)\]", text)
    return match[1] if match else None


def nano_cases(out, peers):
    def log(label):
        return read(out, label + ".log")

    def err(label):
        return read(out, label + ".err")

    thread_rx = read(peers / "thread-peer", "node1.log")
    thread_tx = read(peers / "thread-peer", "node0.log")
    zigbee_switch = read(peers / "zigbee-peer", "node0.log")
    zigbee_led = read(peers / "zigbee-peer", "node1.log")
    stock = read(peers / "nano-stock-repl", "node0.log")
    return {
        "button": (log("button").count("Button A was pressed") == 2
                   and log("button").count("Button A was released") == 2
                   and log("button").count("Button A was hold") == 1
                   and "Button A was" not in log("button-control")),
        "ir_nec": number(err("ir_nec"), "nec_frames") >= 2,
        "led": number(err("led"), "led_changes") >= 2,
        "program": (number(err("program"), "rgb_frames") >= 2
                    and rgb(err("program")) is not None and rgb(err("program-other")) is not None
                    and rgb(err("program")) != rgb(err("program-other"))),
        "thread-1": "Thread Node State: Leader" in log("thread-1"),
        "thread-2": ("Thread Node State: Leader" in thread_tx and "Sending UDP message..." in thread_tx
                     and "Hello,M5Stack" in thread_rx),
        "thread-3": (re.search(r"Thread Node State: (Child|Router)", thread_rx) is not None
                     and "Hello,M5Stack" in thread_rx),
        "thread-4": bool(re.search(r"\|\s*0x[0-9a-f]+\s*\|", read(peers / "thread-scan", "node1.log"), re.I)),
        "thread-5": "openthread-esp32/" in log("thread-5") and "disabled" in log("thread-5"),
        "thread-6": re.search(r"OpenThread CLI RESP===> leader", log("thread-6"), re.I) is not None,
        "zigbee-1": "Temperature received:" in read(peers / "zigbee-temp", "node0.log"),
        "zigbee-2": "Temperature reported to Coordinator" in read(peers / "zigbee-temp", "node1.log"),
        "zigbee-3": re.search(r"\b[1-9]\d* networks found:", read(peers / "zigbee-scan", "node1.log")) is not None,
        "zigbee-4": ("LED device bound successfully!" in zigbee_switch
                     and "Sending ON command" in zigbee_switch and "Sending OFF command" in zigbee_switch),
        "zigbee-5": ("Connected to Zigbee network!" in zigbee_led
                     and "LED ON" in zigbee_led and "LED OFF" in zigbee_led),
        "stock": "C6_REPL 323" in stock and "BTN_LOW 0" in stock and "BTN_HIGH 1" in stock,
    }


def frame_diff(out, first, second):
    a, b = out / (first + ".png"), out / (second + ".png")
    return a.is_file() and b.is_file() and a.read_bytes() != b.read_bytes()


def buzzer_capture(out):
    try:
        with wave.open(str(out / "buzzer.wav")) as wav:
            raw = wav.readframes(wav.getnframes())
            return wav.getsampwidth() == 2 and len(set(raw)) > 4
    except (FileNotFoundError, wave.Error, EOFError):
        return False


def nesso_cases(out, peers):
    def log(label):
        return read(out, label + ".log")

    def err(label):
        return read(out, label + ".err")

    lora_rx = read(peers / "lora-peer", "lora-2.log")
    lora_tx = read(peers / "lora-peer", "lora-1.log")
    lora_control = read(peers / "lora-isolated", "lora-2.log")
    rest = set(re.findall(r"ax:.*", log("imu-control")))
    moving = set(re.findall(r"ax:.*", log("imu-motion")))
    btn_events = ("A Btn Pressed", "A Btn Released", "B Btn Pressed", "B Btn Released")
    return {
        "button": all(e in log("button") and e not in log("button-control") for e in btn_events),
        "buzzer": buzzer_capture(out) and alive_board(err("buzzer")),
        "display": alive_board(err("display")),
        "imu": bool(rest) and len(moving - rest) >= 2,
        "ir_nec": number(err("ir_nec"), "nec_frames") >= 2,
        "led": number(err("led"), "led_changes") >= 2,
        "lora-1": lora_tx.count("transmission finished!") >= 2 and "Hello world from M5Stack!" in lora_rx,
        "lora-2": ("Hello world from M5Stack!" in lora_rx and "Hello Arduino Nesso N1 #" in lora_rx
                   and "Received packet!" not in lora_control and bool(lora_control)),
        "power-2": (number(err("power-2"), "grove_changes") >= 2 and alive_board(err("power-2"))
                    and frame_diff(out, "power-2", "power-other")),
        "quickstart-bargraph": alive_board(err("quickstart-bargraph")),
        "touch": (alive_board(err("touch")) and frame_diff(out, "touch", "touch-first")
                  and frame_diff(out, "touch-first", "touch-control")),
        "stock": (alive_board(err("stock")) and "N1_REPL 323" in
                  read(peers / "nesso-stock-repl", "node0.log")),
    }


def main(board):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("docs", type=Path)
    parser.add_argument("peers", type=Path)
    args = parser.parse_args()
    checks = (nano_cases if board == "nanoc6" else nesso_cases)(args.docs, args.peers)
    receipt = json.loads((args.docs / "run.json").read_text())
    cases = {row["case"]: row for row in receipt["cases"]}
    results = {}
    for case, passed in checks.items():
        row = cases.get(case, {})
        if row.get("exit") == 0:
            results[case] = "PASS" if passed else "FAIL"
        else:
            results[case] = "FAIL" if "exit" in row else "NOT RUN"
    if board == "nesso-n1":
        results["power-1"] = "DOCS-DEFECT" if cases.get("power-1", {}).get("status") == "NOT RUN" else "NOT RUN"
    print(json.dumps(results, indent=2))
    return any(v in {"FAIL", "NOT RUN"} for v in results.values())
