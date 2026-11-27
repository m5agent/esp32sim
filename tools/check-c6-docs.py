#!/usr/bin/env python3
"""Check guest and board markers from the C6 docs and peer runners."""
import argparse
from array import array
from math import cos, hypot, sin, tau
import json
from pathlib import Path
import re
import sys
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



def zigbee_light_transitions(peers, switch_log):
    first = re.search(r"^(\d+\.\d+) Sending (?:ON|OFF) command", switch_log, re.M)
    if first is None:
        return False
    first_command = round(float(first[1]) * 1e9)
    seen = set()
    for line in read(peers, "radio.ndjson").splitlines():
        event = json.loads(line)
        if event["type"] == "leds" and event["node"] == 1 and event["t"] >= first_command:
            seen.add(tuple(event["rgb"][1]))
    return {(0, 0, 0), (0, 0, 255)} <= seen

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
    temp_coordinator = read(peers / "zigbee-temp", "node0.log")
    temp_device = read(peers / "zigbee-temp", "node1.log")
    measured = {float(value) for value in re.findall(r"Temperature received: (-?[\d.]+)", temp_coordinator)
                if -40.0 <= float(value) <= 125.0}
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
        "thread-4": bool(re.search(r"\|\s*[0-9a-f]{4}\s*\|\s*[0-9a-f]{16}\s*\|\s*(?:1[1-9]|2[0-6])\s*\|", read(peers / "thread-scan", "node1.log"), re.I)),
        "thread-5": "openthread-esp32/" in log("thread-5") and "disabled" in log("thread-5"),
        "thread-6": re.search(r"OpenThread CLI RESP===> leader", log("thread-6"), re.I) is not None,
        "zigbee-1": "Temperature sensor bound successfully!" in temp_coordinator and len(measured) >= 2,
        "zigbee-2": ("SHT30 detected at address 0x44" in temp_device and len(measured) >= 2
                     and temp_device.count("Temperature reported to Coordinator") >= 2),
        "zigbee-3": re.search(r"\b[1-9]\d* networks found:", read(peers / "zigbee-scan", "node1.log")) is not None,
        "zigbee-4": ("LED device bound successfully!" in zigbee_switch
                     and "Sending ON command" in zigbee_switch and "Sending OFF command" in zigbee_switch),
        "zigbee-5": ("Connected to Zigbee network!" in zigbee_led
                     and zigbee_light_transitions(peers / "zigbee-peer", zigbee_switch)),
        "stock": "C6_REPL 323" in stock and "BTN_LOW 0" in stock and "BTN_HIGH 1" in stock,
    }


def frame_diff(out, first, second):
    a, b = out / (first + ".png"), out / (second + ".png")
    return a.is_file() and b.is_file() and a.read_bytes() != b.read_bytes()


def buzzer_capture(out):
    try:
        with wave.open(str(out / "buzzer.wav")) as wav:
            if wav.getsampwidth() != 2 or wav.getnchannels() != 1:
                return False
            rate = wav.getframerate()
            samples = array("h", wav.readframes(wav.getnframes()))
            if sys.byteorder == "big":
                samples.byteswap()
    except (FileNotFoundError, wave.Error, EOFError):
        return False
    size = rate // 100
    if rate <= 20000 or len(samples) < size:
        return False

    def has_tone(hz):
        cosines = [cos(tau * hz * i / rate) for i in range(size)]
        sines = [sin(tau * hz * i / rate) for i in range(size)]
        for start in range(0, len(samples) - size + 1, size):
            window = samples[start:start + size]
            a = sum(v * basis for v, basis in zip(window, cosines))
            b = sum(v * basis for v, basis in zip(window, sines))
            if hypot(a, b) / size > 1500:
                return True
        return False

    return has_tone(4000) and has_tone(10000)


def nesso_cases(out, peers):
    def log(label):
        return read(out, label + ".log")

    def err(label):
        return read(out, label + ".err")

    lora_rx = read(peers / "lora-peer", "lora-2.log")
    lora_tx = read(peers / "lora-peer", "lora-1.log")
    lora_control = read(peers / "lora-isolated", "lora-2.log")
    lora_control_tx = read(peers / "lora-isolated", "lora-1.log")
    expected = ["Hello world from M5Stack!"] + [f"Hello Arduino Nesso N1 #{i}" for i in range(13)]
    received = re.findall(r"\[SX1262\] Data:\s+([^\r\n]+)", lora_rx)
    lora_sent_ok = (lora_tx.count("transmission finished!") == 14
                    and number(read(peers / "lora-peer", "lora-1.err"), "lora_tx") == 14)
    lora_recv_ok = (received == expected and lora_rx.count("Received packet!") == 14
                    and number(read(peers / "lora-peer", "lora-2.err"), "lora_rx") == 14)
    lora_control_ok = ("Initializing ... success!" in lora_control
                       and "transmission finished!" in lora_control_tx
                       and "Received packet!" not in lora_control
                       and number(read(peers / "lora-isolated", "lora-2.err"), "lora_rx") == 0)
    btn_events = ("A Btn Pressed", "A Btn Released", "B Btn Pressed", "B Btn Released")
    return {
        "button": all(e in log("button") and e not in log("button-control") for e in btn_events),
        "buzzer": buzzer_capture(out) and alive_board(err("buzzer")),
        "display": alive_board(err("display")),
        "imu": (alive_board(err("imu")) and not frame_diff(out, "imu", "imu-control")
                and (out / "imu-control.png").is_file()
                and frame_diff(out, "imu-control", "imu-motion-first")
                and frame_diff(out, "imu-control", "imu-motion")
                and frame_diff(out, "imu-motion-first", "imu-motion")),
        "ir_nec": number(err("ir_nec"), "nec_frames") >= 2,
        "led": number(err("led"), "led_changes") > 2,
        "lora-1": lora_sent_ok and lora_recv_ok,
        "lora-2": lora_sent_ok and lora_recv_ok and lora_control_ok,
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
