#!/usr/bin/env python3
"""Run the classic controller guest on LX6-A and the DAC guest on LX6-C."""
import argparse
from array import array
from hashlib import file_digest
import json
from pathlib import Path
import re
import statistics
import subprocess
import sys
import wave


def sha(path):
    with path.open("rb") as stream:
        return file_digest(stream, "sha256").hexdigest()


def input_nec(packet, micros, pin):
    times = [9000, 4500]
    for bit in range(32):
        times.extend((560, 1690 if packet & (1 << bit) else 560))
    times.append(560)
    for index, duration in enumerate(times):
        yield f"{micros / 1_000_000:.6f} gpio {pin} {index & 1}"
        micros += duration
    yield f"{micros / 1_000_000:.6f} gpio {pin} 1"


def inspect_dac(path):
    with wave.open(str(path), "rb") as clip:
        rate = clip.getframerate()
        assert clip.getnchannels() == 1 and clip.getsampwidth() == 2
        samples = array("h", clip.readframes(clip.getnframes()))
    if sys.byteorder != "little":
        samples.byteswap()
    assert set(samples).issubset({0, -24576, 24576})
    starts = [index for index in range(1, len(samples))
              if samples[index - 1] == -24576 and samples[index] == 24576]
    measured = rate / statistics.median(b - a for a, b in zip(starts, starts[1:]))
    assert len(starts) >= 90 and abs(measured - 500) < 5
    return {"sample_rate": rate, "rising_edges": len(starts), "frequency_hz": measured,
            "values": sorted(set(samples)), "sha256": sha(path)}


def run(args, case, binary, board, seconds, lines, more=()):
    script = args.out / f"{case}.script"
    script.write_text("\n".join(lines) + "\n")
    command = [str(binary), "--chip", "esp32", "--board", board, "--boot", "rom",
               "--rom", str(args.rom), "--bootloader", str(args.build / "bootloader/bootloader.bin"),
               "--ptable", str(args.build / "partition_table/partition-table.bin"),
               "--app", str(args.build / "lx6_receivers.bin"), "--console", "uart0",
               "--max-seconds", str(seconds), "--no-dump", "--script", str(script), *more]
    log = args.out / f"{case}.log"
    err = args.out / f"{case}.err"
    with log.open("wb") as stdout, err.open("wb") as stderr:
        completed = subprocess.run(command, stdout=stdout, stderr=stderr, timeout=120)
    status = {"return_code": completed.returncode, "serial_sha256": sha(log),
              "simulator_log_sha256": sha(err), "script_sha256": sha(script)}
    (args.out / f"{case}.json").write_text(json.dumps(status, indent=2) + "\n")
    transcript = log.read_text()
    assert completed.returncode == 0 and "LX6 RECEIVERS DONE" in transcript, case
    return transcript


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sim", type=Path, help="LX6-A native simulator binary")
    parser.add_argument("dac_sim", type=Path, help="LX6-C native simulator binary")
    parser.add_argument("build", type=Path, help="ESP-IDF build directory for this fixture")
    parser.add_argument("rom", type=Path, help="ESP32 ECO3 mask ROM ELF")
    parser.add_argument("out", type=Path, help="new replay output directory")
    parser.add_argument("--only", choices=("rmt", "pdm", "dac"))
    parser.add_argument("--stick-board", choices=("m5stack-stickc", "m5stack-stickc-plus", "m5stack-stickc-plus-se",
                        "m5stack-stickc-plus2", "m5stack-stickt2"),
                        help="use the compiled Stick fixture wiring for the NEC/PDM receiver replays")
    args = parser.parse_args()
    if args.stick_board and args.only == "dac":
        parser.error("the Stick fixture routes the receiver paths only")
    mode0, mode1, rmt_pin, pdm_pin = (37, 39, 33, 34) if args.stick_board else (16, 17, 32, 19)
    args.out.mkdir()
    results = {}
    if args.only in (None, "rmt"):
        packets = [0xAA55FF00, 0xA956FE01]
        edges = [edge for packet, when in zip(packets, (1_400_000, 1_700_000))
                 for edge in input_nec(packet, when, rmt_pin)]
        board = args.stick_board or "esp32dev"
        initial = [f"0.05 gpio {mode1} 0", f"0.1 gpio {rmt_pin} 1"]
        received = run(args, "rmt-rx", args.sim, board, 2.2,
                       [f"0.05 gpio {mode0} 0", *initial, *edges])
        seen = [int(text, 16) for text in re.findall(r"^LX6 NEC RX ([\da-f]{8})", received, re.M)]
        assert seen == packets, seen
        idle = run(args, "rmt-idle", args.sim, board, 3.8, [f"0.05 gpio {mode0} 1", *initial])
        assert "LX6 RMT NO FRAME" in idle and "LX6 NEC RX" not in idle
        results["rmt"] = {"packets": [f"{value:08x}" for value in seen], "idle": "no frame"}
    if args.only in (None, "pdm"):
        common = [f"0.05 gpio {mode0} 0", f"0.05 gpio {mode1} 1", f"0.1 gpio {pdm_pin} 0"]
        waveform = [*common, f"0.993457 gpio {pdm_pin} 1", f"1.151379 gpio {pdm_pin} 0",
                    f"1.290243 gpio {pdm_pin} 1"]
        trace = args.out / "pdm.vcd"
        board = args.stick_board or "m5stack-atomu"
        capture = run(args, "pdm", args.sim, board, 2, waveform, ("--vcd", str(trace)))
        idle = run(args, "pdm-idle", args.sim, board, 2, common)
        pattern = r"^LX6 PDM RAW \d+ low=(\d+) high=(\d+) mixed=(\d+)"
        decoded = [tuple(map(int, row)) for row in re.findall(pattern, capture, re.M)]
        silent = [tuple(map(int, row)) for row in re.findall(pattern, idle, re.M)]
        assert len(decoded) == 5 and all(sum(row) == 4096 for row in decoded)
        assert silent == [(4096, 0, 0)] * 5
        low, high, mixed = map(sum, zip(*decoded))
        assert low >= 4096 and high >= 4096 and mixed > 0, decoded
        results["raw_pdm"] = {"low_high_mixed": decoded, "idle_words": 4096 * 5,
                              "gpio_trace_sha256": sha(trace)}
    if not args.stick_board and args.only in (None, "dac"):
        capture = args.out / "dac.wav"
        audio = run(args, "dac", args.dac_sim, "m5stack-basic-v27", 1.6,
                    ["0.05 gpio 16 1", "0.05 gpio 17 1"],
                    ("--wav", str(capture)))
        assert "LX6 DAC GPIO25 CYCLIC DONE" in audio
        results["dac"] = inspect_dac(capture)
    manifest = {"results": results, "rom_sha256": sha(args.rom),
                "runtime_sha256": {"lx6a": sha(args.sim), "lx6c": sha(args.dac_sim)},
                "guest_sha256": {name: sha(args.build / name) for name in
                                 ("bootloader/bootloader.bin", "partition_table/partition-table.bin",
                                  "lx6_receivers.bin")}}
    if args.stick_board:
        manifest["board"] = args.stick_board
        manifest["pins"] = {"rmt_input": rmt_pin, "pdm_clock": 0, "pdm_data": pdm_pin}
    (args.out / "run.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
