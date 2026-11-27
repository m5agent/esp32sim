#!/usr/bin/env python3
"""Check the e-paper guest receipts, frames, and input controls."""
import argparse
from collections import Counter
import json
from importlib.util import spec_from_file_location, module_from_spec
from pathlib import Path
import re

from check_cores3 import png_rgb, color_counts

spec = spec_from_file_location("runner", Path(__file__).with_name("run-paper-docs.py"))
runner = module_from_spec(spec)
spec.loader.exec_module(runner)


def verify(board, case, log, pixels, control, early, control_frame, first_frame):
    def pixel(x, y):
        pos = (y * runner.DIMS[board][0] + x) * 3
        return tuple(pixels[pos:pos + 3])

    colors = color_counts(pixels) if pixels else Counter()
    if re.search(r"Guru Meditation|assert failed|abort\(\) was called|rebooting\.\.\.", log, re.I):
        return "FAIL", "Guest panic, assertion, or reboot marker"
    if case == "stock":
        if len(colors) < 2:
            return "FAIL", "No completed nonuniform stock frame"
        return "NOT RUN", "Stock rendered; use the service probe for input and REPL"
    if board == "paperdiy" and case in {"button", "display-2"}:
        if not all((pixels, first_frame, control_frame)):
            return "NOT RUN", "Two action frames and a no-input frame are required"
        if len({pixels, first_frame, control_frame}) != 3 or len(colors) < 2:
            return "FAIL", "The two button actions did not yield distinct completed frames"
        return "PASS", "Each action changed the frame relative to the no-input control"
    if (board, case) == ("paperdiy", "m5pm1"):
        def loop_times(trace):
            return [float(t) for t in re.findall(r"(?m)^\[fn\].*? t=([\d.]+)s .*? _Z4loopv\(", trace or "")]

        active, idle = loop_times(log), loop_times(control)
        if not active or not idle:
            return "NOT RUN", "Guest loop traces are missing"
        if ("Power-on timer configured" in log and "Power-off timer configured" in log
            and re.search(r"chip reset at t=8\.\d+s: cause 0x1", log)
            and 12 < max(active) < 14 and max(idle) > 17 and "chip reset at" not in control):
            return "PASS", "Timed restart and shutdown observed against a running control"
        return "FAIL", "Timed reset or stop did not match the guest and control traces"
    if case.startswith(("display", "quickstart")):
        if not colors or len(colors) == 1:
            return "FAIL", "No completed nonuniform application frame"
        if early is None:
            return "NOT RUN", "Completed early frame capture missing"
        if early != {(255, 255, 255)}:
            return "FAIL", "Early capture did not retain the white frame"
        if (board, case) == ("paperdiy", "quickstart-longtextscroll"):
            if first_frame is None:
                return "NOT RUN", "Earlier scroll frame missing"
            if pixels != first_frame and len(color_counts(first_frame)) > 1:
                return "PASS", "Two completed nonuniform frames show the scroll changing"
            return "FAIL", "The scroll did not move between captures"
        if (board, case) == ("paperdiy", "display-1"):
            shades = [pixel(x, 395)[0] for x in (145, 223, 297, 375)]
            if not shades[0] < shades[1] < shades[2] or shades[3] != shades[0]:
                return "FAIL", f"Expected black, dark gray, light gray, black swatches, got {shades}"
            return "PASS", "Completed grayscale frame with expected swatch ordering"
        if (board, case) == ("papermono", "display-1"):
            if len(colors) != 4 or not all(r == g == b or max(r, g, b) - min(r, g, b) <= 3 for r, g, b in colors):
                return "FAIL", f"Expected four gray levels, got {len(colors)} RGB values"
            return "PASS", "Four quantized levels and white early control"
        if (board, case) == ("papercolor", "display"):
            if len(colors) != 6:
                return "FAIL", f"Expected six Spectra 6 palette values, got {len(colors)}"
            return "PASS", "All six colors after refresh and white early control"
        return "NOT RUN", "Frame captured; case-specific layout/action oracle pending"
    if case == "touch":
        if control is None:
            return "NOT RUN", "Completed no-input control missing"
        points = set(re.findall(r"x:\s*(\d+)[, ]+\s*y:\s*(\d+)", log))
        if len(points) < 2:
            return "FAIL", f"Guest reported {len(points)} distinct touch points, expected two"
        if re.search(r"x:\s*\d+[, ]+\s*y:\s*\d+", control or ""):
            return "FAIL", "No-input control reported a touch"
        return "PASS", "Guest reported two points and silent no-input control"
    if case in {"microsd", "sdcard"} and control is not None and "Displayed:" in log and "Displayed:" not in control:
        if board == "paperdiy" and "Contents of /PaperDIY_Log.txt:" not in log:
            return "FAIL", "Guest did not read back the log it wrote"
        return "PASS", "Guest rendered SD media while the no-card control did not"
    if case == "rtc-1" and "RTC demo time set" in log:
        values = set(re.findall(r"RTC: 2026/05/08 .*?12:00:(\d\d)", log))
        if len(values) >= 2:
            return "PASS", "Guest set the calendar and observed it ticking"
    if case == "button" and board == "papercolor":
        if control is None:
            return "NOT RUN", "Completed no-input control missing"
        if all(f"{key} was Pressed" in log and f"{key} was Pressed" not in control for key in "ABC"):
            return "PASS", "Guest reported three injected buttons and no control presses"
        return "FAIL", "Missing guest button markers or control produced a press"
    if re.search(r"(?:init(?:ialization)? failed|not found\.|nack|not connected)", log, re.I):
        return "FAIL", "Guest reported an initialization or peripheral error"
    return "NOT RUN", "Firmware replayed; case-specific behavior oracle pending"


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("output", type=Path)
    args = p.parse_args()
    out = args.output
    receipt = json.loads((out / "runs.json").read_text())
    board = receipt["board"]
    verdicts = []
    completed = {(row["case"], row["kind"]) for row in receipt["runs"] if row.get("exit") == 0}
    for row in receipt["runs"]:
        if row["kind"] != "input":
            continue
        case = row["case"]
        if row.get("state") == "NOT RUN":
            state, reason = "NOT RUN", row["reason"]
        elif row["exit"] == 124:
            state, reason = "NOT RUN", "Host timeout before the guest acceptance horizon"
        elif row["exit"] != 0:
            state, reason = "FAIL", f"Simulator exit {row['exit']}"
        else:
            frame = out / f"{case}.png"
            early = out / f"{case}-early.png"
            control = out / f"{case}-control.log"
            try:
                pixels = png_rgb(frame.read_bytes(), dimensions=runner.DIMS[board]) if frame.exists() else None
                early_colors = None
                if (case, "early") in completed:
                    early_colors = set(color_counts(png_rgb(early.read_bytes(), dimensions=runner.DIMS[board])))
                control_log = control.read_text(errors="replace") if (case, "control") in completed else None
                def frame_at(kind):
                    if (case, kind) in completed:
                        path = out / f"{case}-{kind}.png"
                        return png_rgb(path.read_bytes(), dimensions=runner.DIMS[board])
                    return None

                state, reason = verify(board, case, (out / f"{case}.log").read_text(errors="replace"),
                                       pixels, control_log, early_colors, frame_at("control"), frame_at("first"))
            except (OSError, ValueError) as exc:
                state, reason = "FAIL", str(exc)
        verdicts.append({"case": case, "verdict": state, "reason": reason})
        print(f"| {case} | {state} | {reason} |")
    (out / "verdicts.json").write_text(json.dumps(verdicts, indent=2) + "\n")
    return int(any(item["verdict"] != "PASS" for item in verdicts))


if __name__ == "__main__":
    raise SystemExit(main())
