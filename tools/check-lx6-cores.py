#!/usr/bin/env python3
"""Check guest-visible LCD, UART, SD, motor and PCM outcomes from the Core runner."""
from array import array
from collections import Counter
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import wave

from check_cores3 import changed, png_rgb, region, require


class Run:
    def __init__(self, directory):
        self.directory = directory

    def path(self, label, suffix):
        return self.directory / f"{label}.{suffix}"

    def read(self, label, suffix="log"):
        path = self.path(label, suffix)
        return path.read_text(errors="replace") if path.exists() else ""

    def ran(self, *labels):
        for label in labels:
            require(self.read(label, "status").strip() == "0", f"{label} did not complete")
            require("Guru Meditation" not in self.read(label), f"{label} hit a guest panic")

    def metric(self, label, name):
        match = re.search(rf"\b{name}=(\d+)", self.read(label, "err"))
        return int(match[1]) if match else 0

    def frame(self, label):
        self.ran(label)
        pixels = png_rgb(self.path(label, "png").read_bytes())
        colors = Counter(zip(pixels[0::3], pixels[1::3], pixels[2::3]))
        require(len(colors) > 1, f"{label} is blank")
        return pixels, colors

    def read_sd(self, label, name):
        return subprocess.run(["mtype", "-i", self.path(label, "img"), f"::/{name}"],
                              capture_output=True, check=True).stdout

    def png_reply(self, label):
        return self.frame(label)[0]

    def pcm(self, label):
        with wave.open(str(self.path(label, "wav")), "rb") as handle:
            require(handle.getsampwidth() == 2 and handle.getnchannels() == 1, "expected 16-bit mono")
            samples = array("h", handle.readframes(handle.getnframes()))
            if sys.byteorder != "little":
                samples.byteswap()
            return handle.getframerate(), samples


def audio_energy(rate, pcm, hz):
    """Estimate a tone's magnitude from short, overlapping active windows."""
    n = min(rate // 100, 1000)
    require(n != 0 and len(pcm) >= n, "no usable captured PCM")
    unit = [complex(math.cos(2 * math.pi * hz * i / rate), math.sin(2 * math.pi * hz * i / rate)) for i in range(n)]
    scores = []
    for start in range(0, len(pcm) - n + 1, max(1, n // 2)):
        window = pcm[start:start + n]
        dc = sum(window) / n
        scores.append(abs(sum((p - dc) * z for p, z in zip(window, unit))) / n)
    return max(scores, default=0)


def checks(device, run):
    basic = device == "basic-v27"
    rows = {}

    def check(name, verify):
        try:
            rows[name] = {"status": "PASS", "reason": verify()}
        except (ValueError, OSError, wave.Error, EOFError, subprocess.CalledProcessError) as exc:
            rows[name] = {"status": "FAIL", "reason": str(exc)}

    def display():
        _, colors = run.frame("display")
        require(run.metric("display", "pixels") >= 76800, "guest did not fill one complete frame")
        require(colors[(255, 255, 255)] > 50000 and len(colors) >= 3, "missing white background and colored primitives")
        return "320×240 framebuffer contains the filled background and visible color primitives"

    check("display", display)

    def inputs(case, identical_pressed=False):
        suffixes = ("a", "b", "c") if case == "button" else ("1", "2")
        idle = run.png_reply(f"{case}-idle")
        release = run.png_reply(f"{case}-release")
        pressed = [run.png_reply(f"{case}-{ident}") for ident in suffixes]
        require(all(changed(idle, frame) >= 20 for frame in pressed), "press did not change the guest frame")
        if not identical_pressed:
            require(len(set(pressed)) == len(pressed), "different inputs produced identical guest frames")
        require(changed(idle, release) <= 150, "release did not return to idle")
        return "Distinct input states and release observed in the guest frame"

    def basic_buttons():
        labels = ("button-idle", "button-a", "button-b", "button-c", "button-release")
        run.ran(*labels)
        for label, expected in zip(labels, ("", "A", "B", "C", "A")):
            actual = "".join(re.findall(r"Btn([ABC]) Pressed", run.read(label)))
            require(actual == expected, f"{label} printed {actual!r}, expected {expected!r}")
        return "UART0 reports the three independent button edges and no idle or release edge"

    check("button", basic_buttons if basic else lambda: inputs("button"))
    if not basic:
        check("touch-1", lambda: inputs("touch-1", True))
        check("touch-2", lambda: inputs("touch-2"))

    def sensor(case):
        frames = [run.png_reply(f"{case}-{part}") for part in ("idle", "1", "2")]
        require(len(set(frames)) == 3 and all(changed(frames[0], frame) > 30 for frame in frames[1:]),
                "both injected values must independently affect the guest")
        return "Baseline and both injected states produced distinct LCD values"

    check("battery", lambda: sensor("battery"))
    if basic:
        if run.read("imu-idle", "status").strip() == "0" and run.metric("imu-idle", "id_reads"):
            rows["imu"] = {"status": "DOCS-DEFECT", "reason": "Fixture assignment points Basic at m5core/imu.md line 9, which targets Gray. K001-V27 Basic has no onboard IMU."}
        else:
            rows["imu"] = {"status": "FAIL", "reason": "Could not qualify the no-IMU control"}
    else:
        check("imu", lambda: sensor("imu"))

    def inspect_sd():
        idle = run.png_reply("microsd-absent")
        first = run.png_reply("microsd-first")
        second = run.png_reply("microsd")
        require(min(changed(idle, first), changed(idle, second), changed(first, second)) > 1000,
                "missing differentiated decoded images or no-card state")
        expected = b"Hello, world! \nSD card write success! \n"
        require(run.read_sd("microsd", "WriteTest.txt") == expected, "missing exact SD write/readback payload")
        return "Two guest-decoded images, missing-card control and exact persisted WriteTest.txt"

    check("microsd", inspect_sd)
    scroll = "quickstart-longtextscroll" if basic else "quickstart-scrollgraph"

    def scrolls():
        require(changed(run.png_reply(scroll), run.png_reply(f"{scroll}-first")) >= 20,
                "scrolling setup produced no visible change")
        return "Two different 320×240 LCD frames during scrolling"

    check(scroll, scrolls)

    def sound(case, frequencies):
        run.ran(case)
        rate, pcm = run.pcm(case)
        require(max(pcm) - min(pcm) >= 512, "output is silent or a constant DAC value")
        for frequency in frequencies:
            require(frequency < rate / 2 and audio_energy(rate, pcm, frequency) > 150,
                    f"PCM lacks expected {frequency} Hz component")
        return f"PCM capture at {rate} Hz contains the required tone(s)"

    def playback():
        result = sound("wav", (733 if basic else 997,))
        serial = run.read("wav").lower()
        require("wav: 12000hz, 1ch, 16bit" in serial or "starting playback..." in serial,
                "did not open the seeded WAV")
        require("playback completed!" in serial, "guest did not complete playback")
        return result + " and the guest reached playback completion"

    check("wav", playback)
    if basic:
        check("speaker", lambda: sound("speaker", (10000, 4000)))

    def wakeup():
        run.ran("wakeup")
        log = run.read("wakeup")
        sim = run.read("wakeup", "err")
        require(re.search(r"DEEP.?SLEEP|DEEPSLEEP_RESET", log + sim) is not None, "no deep-sleep reset after BtnA")
        return "Button input followed by deep-sleep reset and boot"

    check("wakeup", wakeup)

    if not basic:
        def rgb():
            for i, label in enumerate(("rgb-red", "rgb-green", "rgb-blue")):
                run.ran(label)
                report = run.read(label, "err")
                match = re.search(r"rgb_pixels=10 rgb_first=([0-9a-f]{6}) rgb_uniform=true", report)
                require(match is not None, f"{label} lacks ten uniform decoded LEDs")
                color = bytes.fromhex(match[1])
                require(color[i] > 0 and sum(color) == color[i], f"{label} shows {color.hex()}")
            return "RMT drives ten uniform pixels in red, green and blue order"

        def rtc():
            run.ran("rtc")
            log = run.read("rtc")
            require("RTC found." in log and "NTP Connected." in log, "RTC or sandboxed NTP did not initialize")
            times = set(re.findall(r"RTC\s+UTC\s+:.*?\b(\d\d:\d\d:\d\d)", log))
            require(len(times) > 1, "no guest RTC progression after NTP write")
            return "Guest identifies RTC, writes from sandboxed NTP and reads successive wall-clock values"

        def motor():
            run.ran("vibration")
            report = run.read("vibration", "err")
            value = re.search(r"motor_mv=(\d+)\.\.(\d+)", report)
            require(value is not None and run.metric("vibration", "motor_changes") >= 6, "motor rail did not transition")
            require(500 <= int(value[1]) < int(value[2]) <= 3400, "motor rail voltage is out of range")
            return f"{run.metric('vibration', 'motor_changes')} DLDO1 rail transitions, {value[1]}–{value[2]} mV commanded"

        def microphone():
            levels = []
            import io
            for label in ("mic-idle", "mic-733", "mic-1234"):
                run.ran(label)
                with wave.open(io.BytesIO(run.read_sd(label, "recording.wav")), "rb") as handle:
                    require(handle.getframerate() == 12000 and handle.getnframes() >= 512,
                            f"{label} did not record at the requested sample rate")
                    samples = array("h", handle.readframes(handle.getnframes()))
                    if sys.byteorder != "little":
                        samples.byteswap()
                    levels.append([audio_energy(12000, samples, freq) for freq in (733, 1234)])
            require(max(levels[0]) < 30 and levels[1][0] > max(100, 3 * levels[1][1])
                    and levels[2][1] > max(100, 3 * levels[2][0]), "recorded SD WAVs do not distinguish silence and both tones")
            return "On-card WAV contains silence then the two separately injected PDM-to-PCM tones"

        check("rgb_led", rgb)
        check("rtc", rtc)
        check("vibration", motor)
        check("mic", microphone)

    def stock():
        home, colors = run.frame("stock-home")
        require(len(colors) >= 8 and "Startup with network type" in run.read("stock-home"), "no complete stock home screen")
        navigation = run.png_reply("stock-navigation")
        require(changed(region(home, 0, 26, 320, 238), region(navigation, 0, 26, 320, 238)) >= 800,
                "stock input did not change the launcher")
        run.ran("stock-input")
        serial = run.read("stock-input")
        require(f"LX6-BOARD {1 if basic else 2}" in serial, "UART0 did not report the expected M5Unified board")
        states = re.findall(r"^LX6-INPUT \((True|False), (True|False), (True|False)\)\s*$",
                            serial, flags=re.MULTILINE)
        require(states == [("False", "False", "False"), ("True", "False", "False"),
                           ("False", "True", "False"), ("False", "False", "True"),
                           ("False", "False", "False")], "missing A/B/C pressed and released readbacks")
        if not basic:
            require("LX6-TOUCH 1 88 110" in serial, "stock touch readback did not match the injected point")
        return "Stock UIFlow2 home screen and UART0 input round-trip"

    check("stock", stock)
    return rows


if __name__ == "__main__":
    device, output = sys.argv[1:]
    folder = Path(output)
    results = checks(device, Run(folder))
    (folder / "checks.json").write_text(json.dumps(results, indent=2) + "\n")
    for case, result in results.items():
        print(case, result["status"], result["reason"])
    sys.exit(any(result["status"] == "FAIL" for result in results.values()))
