#!/usr/bin/env bash
# Usage: ESP32SIM_ROM_DIR=... S3B_FIRMWARE_DIR=... run-s3b-docs.sh BOARD BINARY OUTDIR
# Optional: S3B_SD_SEED (Cardputer), S3B_STOCK_IMAGE (this board's UIFlow2).
set -euo pipefail
board=${1:?board required}
binary=${2:?simulator binary required}
out=${3:?new output directory required}
firmware=${S3B_FIRMWARE_DIR:?set prebuilt firmware parent directory}/$board
export ESP32SIM_ROM_DIR=${ESP32SIM_ROM_DIR:?set ESP32-S3 ROM directory}
case "$board" in cardputer|dial|sticks3|stopwatch) ;; *) exit 2 ;; esac
mkdir "$out"
out=$(cd "$out" && pwd)
cp "$binary" "$out/esp32sim"
git -C "$(dirname "$0")" rev-parse HEAD >"$out/source"
git -C "$(dirname "$0")" status --porcelain >"$out/source-status"
git -C "$(dirname "$0")" diff --binary HEAD >"$out/source.diff"
shasum -a 256 "$out/esp32sim" "$ESP32SIM_ROM_DIR/esp32s3_rev0_rom.elf" \
    "$firmware"/*/sketch.merged.bin >"$out/inputs.sha256"

printf '%s\n' '2.0 press a 150' '4.0 press a 150' '6.0 press a 900' >"$out/button.txt"
case "$board" in
  cardputer) printf '%s\n' '2.0 press g0 150' '4.0 press g0 150' '6.0 press g0 900' >"$out/button.txt" ;;
  dial) printf '%s\n' '2.0 press button 150' '4.0 press button 150' '6.0 press button 900' >"$out/button.txt" ;;
esac
printf '%s\n' '3.0 knob cw 4' '5.5 knob ccw 2' >"$out/encoder.txt"
printf '%s\n' '2.0 touch 50 90 1' '3.5 touch 50 90 0' '4.0 touch 150 160 1' '6.5 touch 150 160 0' >"$out/touch.txt"
printf '%s\n' '2.0 key a' '2.4 key space' '3.0 key b' '3.4 key fn 400' '3.5 key del 100' '4.1 key c' '4.6 key enter' >"$out/keyboard.txt"
printf '%s\n' '3.0 adc 10 1.73' '5.0 adc 10 2.00' >"$out/adc.txt"
printf '%s\n' '2.0 ir-nec 10 34' '5.0 ir-nec 2a 57' >"$out/ir-nec.txt"
printf '%s\n' '3.0 press a 150' >"$out/wakeup.txt"
printf '%s\n' '3.0 press a 150' '5.5 imu 0.8 0.0 0.7 0 0 0' '11.5 imu -0.8 0.0 0.7 0 0 0' >"$out/motion-wakeup.txt"
printf '%s\n' '4.5 touch 233 233 1' '4.8 touch 233 233 0' >"$out/stopwatch-mic.txt"
printf '%s\n' '2.0 press a 700' '3.0 press b 700' >"$out/a-b.txt"
printf '%s\n' '3.5 press button 150' >"$out/dial-wakeup.txt"
printf '%s\n' '3.0 press a 150' '4.0 press b 150' '6.0 pwrkey' >"$out/power-keys.txt"
printf '%s\n' '3.0 battery 3550 0' '5.0 battery 4080 1' >"$out/battery.txt"
printf '%s\n' '2.0 imu 0.5 -0.25 1.0 125 -50 25' '4.0 imu -0.75 0.3 0.25 -100 80 -30' >"$out/imu.txt"
printf '%s\n' '2.0 rfid 04010203' '3.8 rfid none' '5.0 rfid 04090807' >"$out/rfid.txt"
printf '%s\n' '4.0 press g0 150' >"$out/cardputer-mic.txt"
printf '%s\n' '4.0 press a 150' >"$out/sticks3-mic.txt"
python3 - "$out" <<'PY'
from array import array
from math import sin, tau
from pathlib import Path
import sys
import wave

out = Path(sys.argv[1])
rate = 17000
for name, hz, amplitude in [('mic', 733, 12000), ('mic-alt', 997, 9000), ('silence', 733, 0)]:
    samples = array('h', (round(amplitude * sin(tau * hz * i / rate)) for i in range(rate * 12)))
    if sys.byteorder == 'big':
        samples.byteswap()
    with wave.open(str(out / (name + '.wav')), 'wb') as wav:
        wav.setparams((1, 2, rate, 0, 'NONE', 'not compressed'))
        wav.writeframes(samples.tobytes())
PY
if [[ "$board" == cardputer && -n ${S3B_SD_SEED:-} ]]; then
    cp "$S3B_SD_SEED" "$out/sdcard.img"
    shasum -a 256 "$S3B_SD_SEED" >>"$out/inputs.sha256"
fi
shasum -a 256 "$out"/*.txt "$out"/*.wav >>"$out/inputs.sha256"

run() {
    local label=$1 image=$2 seconds=$3 status=0
    shift 3
    printf '%q ' "$out/esp32sim" --board "m5stack-$board" --boot rom --console both \
        --flash-image "$image" --max-seconds "$seconds" --tft-png "$out/$label.png" --no-dump "$@" \
        >"$out/$label.command"
    python3 - "$out" "$label" "$board" "$image" "$seconds" "$@" <<'PYRUN' || status=$?
import subprocess
import sys
from pathlib import Path
out, label, board, image, seconds, *flags = sys.argv[1:]
out = Path(out)
args = [str(out / "esp32sim"), "--board", "m5stack-" + board, "--boot", "rom", "--console", "both",
        "--flash-image", image, "--max-seconds", seconds, "--tft-png", str(out / (label + ".png")),
        "--no-dump", *flags]
with (out / (label + ".log")).open("wb") as serial, (out / (label + ".err")).open("wb") as diagnostics:
    try:
        sys.exit(subprocess.run(args, stdout=serial, stderr=diagnostics, timeout=300).returncode)
    except subprocess.TimeoutExpired:
        diagnostics.write(b"runner wall timeout (300 seconds)\n")
        sys.exit(124)
PYRUN
    printf '%s\n' "$status" >"$out/$label.status"
}

for image in "$firmware"/*/sketch.merged.bin; do
    label=$(basename "$(dirname "$image")")
    if [[ -n ${S3B_CASES:-} && " $S3B_CASES " != *" $label "* ]]; then continue; fi
    seconds=8
    flags=()
    case "$label" in
        display|quickstart-*) seconds=2.7 ;;
        battery) seconds=7; flags=(--script "$out/battery.txt"); [[ "$board" != cardputer ]] || flags=(--script "$out/adc.txt") ;;
        button)
            seconds=4.5
            case "$board" in
                sticks3) seconds=3.25; flags=(--script "$out/a-b.txt") ;;
                stopwatch) seconds=6; flags=(--script "$out/power-keys.txt") ;;
                *) flags=(--script "$out/button.txt") ;;
            esac ;;
        ir_nec-2|ir_nec-3) flags=(--script "$out/ir-nec.txt") ;;
        wakeup) seconds=12; flags=(--script "$out/dial-wakeup.txt") ;;
        wakeup-*|m5pm1-1|m5pm1_m5ioe1-1|m5pm1_m5ioe1-4|m5pm1_m5ioe1-5)
            seconds=17; flags=(--script "$out/wakeup.txt") ;;
        m5pm1-2|m5pm1-3|m5pm1_m5ioe1-2|m5pm1_m5ioe1-3)
            seconds=14; flags=(--script "$out/motion-wakeup.txt") ;;
        m5pm1_m5ioe1-6) seconds=13; flags=(--wav "$out/$label-output.wav") ;;
        rtc) seconds=11; flags=(--wifi 'ssid=********,psk=********,epoch=1764000000' --net none --pcap "$out/rtc.pcap") ;;
        rtc-2) seconds=11; flags=(--wifi 'ssid=ssid,psk=password,epoch=1764000000' --net none --pcap "$out/rtc-2.pcap") ;;
        keyboard|encoder|imu) flags=(--script "$out/$label.txt") ;;
        rfid-*) seconds=6; flags=(--script "$out/rfid.txt") ;;
        touch|touch-*) seconds=6; flags=(--script "$out/touch.txt") ;;
        speaker|buzzer) seconds=5; flags=(--wav "$out/$label-output.wav") ;;
        mic)
            seconds=11
            flags=(--mic-wav "$out/mic.wav" --wav "$out/mic-output.wav")
            if [[ -f "$out/$board-mic.txt" ]]; then flags+=(--script "$out/$board-mic.txt"); fi
            ;;
        sdcard|speaker-sd-wav)
            seconds=8
            [[ "$label" != speaker-sd-wav ]] || seconds=15
            if [[ -e "$out/sdcard.img" ]]; then
                if [[ "$label" == sdcard ]]; then flags=(--sd-rw "$out/sdcard.img");
                else flags=(--sd "$S3B_SD_SEED" --wav "$out/$label-output.wav"); fi
            fi
            ;;
    esac
    run "$label" "$image" "$seconds" ${flags[@]+"${flags[@]}"}
    case "$label" in
        display|quickstart-*) run "$label-early" "$image" 1.5 ;;
        rtc-1) run "$label-early" "$image" 3 ;;
        rtc|rtc-2)
            run "$label-early" "$image" 8 "${flags[@]:0:4}" ;;
        battery)
            run "$label-first" "$image" 4.9 "${flags[@]}"
            run "$label-idle" "$image" "$seconds" ;;
        imu|touch|touch-*)
            run "$label-first" "$image" 3.3 "${flags[@]}"
            run "$label-idle" "$image" "$seconds" ;;
        button)
            if [[ "$board" == sticks3 ]]; then
                run button-first "$image" 2.4 --script "$out/a-b.txt"
                run button-idle "$image" "$seconds"
            fi ;;
        rfid-*|keyboard|encoder|ir_nec-2|ir_nec-3)
            run "$label-idle" "$image" "$seconds" ;;
        mic)
            run mic-alt "$image" 3.8 --mic-wav "$out/mic-alt.wav"
            run mic-tone "$image" 3.8 --mic-wav "$out/mic.wav"
            run mic-silent "$image" 3.8 --mic-wav "$out/silence.wav"
            ;;
    esac
done
if [[ -n ${S3B_STOCK_IMAGE:-} && ( -z ${S3B_CASES:-} || " $S3B_CASES " == *" stock "* ) ]]; then
    shasum -a 256 "$S3B_STOCK_IMAGE" >>"$out/inputs.sha256"
    case "$board" in
        cardputer) printf '%s\n' '5.0 press g0 150' >"$out/stock.txt" ;;
        sticks3) printf '%s\n' '5.0 press a 150' >"$out/stock.txt" ;;
        dial|stopwatch) printf '%s\n' '5.0 touch 100 130 1' '5.4 touch 100 130 0' >"$out/stock.txt" ;;
    esac
    printf 'attach\ncdc 3000\ncdc 3000 \003\ncdc 3000 print(10+7293)\n' >"$out/stock-usb.txt"
    shasum -a 256 "$out/stock.txt" "$out/stock-usb.txt" >>"$out/inputs.sha256"
    run stock-home "$S3B_STOCK_IMAGE" 7
    run stock-input "$S3B_STOCK_IMAGE" 7 --script "$out/stock.txt"
    run stock-repl "$S3B_STOCK_IMAGE" 18 --usb-host "$out/stock-usb.txt"
fi
result=0
python3 "$(dirname "$0")/check-s3b-docs.py" "$board" "$out" >"$out/verdicts.md" || result=$?
cat "$out/verdicts.md"
exit "$result"
