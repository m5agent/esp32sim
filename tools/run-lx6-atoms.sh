#!/usr/bin/env bash
# Run the native Atom documentation and stock firmware checks.
set -euo pipefail
binary=${1:?simulator binary}
case "${2:?device name}" in
    atom-lite|atom-matrix|atom-matrix-v11|atomu|atom-echo) device=$2 ;;
    *) echo "Unknown Atom device: $2" >&2; exit 2 ;;
esac
out=${3:?new output directory}
plan=${ATOM_PLAN_DIR:?compiled firmware and inventory directory}
stock=${ATOM_STOCK_DIR:?directory containing DEVICE.bin}
rom=${ESP32SIM_ROM_DIR:?mask ROM directory}/esp32_rev300_rom.elf
tools=$(cd "$(dirname "$0")" && pwd)
mkdir "$out"
cp "$binary" "$out/esp32sim"
binary="$out/esp32sim"
git -C "$tools" rev-parse HEAD >"$out/source.txt"
git -C "$tools" status --porcelain --untracked-files=no >"$out/source-status.txt"
shasum -a 256 "$binary" "$rom" "$stock/$device.bin" >"$out/inputs.sha256"

run() {
    local label=$1 image=$2 seconds=$3 rc=0
    shift 3
    "$binary" --chip esp32 --board "m5stack-$device" --rom "$rom" --boot rom \
      --flash-image "$image" --console uart0 --max-seconds "$seconds" --no-dump "$@" \
      >"$out/$label.log" 2>"$out/$label.err" || rc=$?
    printf '%s\n' "$rc" >"$out/$label.status"
}

sketch() {
    local label=$1 case=$2 seconds=$3
    shift 3
    local image="$plan/firmware/$device/$case/sketch.merged.bin"
    if [[ ! -f $image ]]; then return; fi
    shasum -a 256 "$image" >>"$out/inputs.sha256"
    run "$label" "$image" "$seconds" "$@"
}

case "$device" in
    atom-lite|atom-matrix|atom-matrix-v11)
        cat >"$out/button.script" <<'SCRIPT'
0.8 press a 150
1.7 press a 1250
SCRIPT
        sketch button-idle button 0.75
        sketch button button 3.2 --script "$out/button.script"
        sketch quickstart-logoutput quickstart-logoutput 2.5
        sketch ir_nec ir_nec 3.15 --vcd "$out/ir_nec.vcd"
        sketch rgb-red rgb 0.45 --tft-png "$out/rgb-red.png"
        sketch rgb-green rgb 0.95 --tft-png "$out/rgb-green.png"
        sketch rgb-blue rgb 1.45 --tft-png "$out/rgb-blue.png"
        if [[ $device == atom-lite ]]; then
            sketch imu imu 1.6
        else
            cat >"$out/imu.script" <<'SCRIPT'
0.65 imu 0.5 -0.25 1.0 125 -50 25
2.1 imu -0.75 0.375 0.25 -100 80 -30
SCRIPT
            sketch imu-idle imu 3.55
            sketch imu imu 3.55 --script "$out/imu.script"
        fi
        ;;
    atom-echo)
        sketch quickstart-echorest quickstart-echorest 4.0
        sketch quickstart-playmusic quickstart-playmusic 4.0 --wav "$out/playmusic.wav"
        sketch quickstart-recordplay quickstart-recordplay 4.0 --i2s-tone 0:733:0.02 --wav "$out/recordplay.wav"
        ;;
esac

cat >"$out/stock.script" <<'SCRIPT'
1.8 uart0-cr import M5; M5.begin(); print('LX6'+'-BOARD', M5.getBoard())
2.1 uart0-cr from hardware import RGB; rgb=RGB(io=27,n=25 if M5.getBoard()==141 else 1,type='WS2812'); rgb.set_brightness(100); rgb.fill_color(0x204060)
2.4 uart0-cr M5.update(); print('LX6'+'-BUTTON', int(M5.BtnA.isPressed()))
2.5 press a 400
2.7 uart0-cr M5.update(); print('LX6'+'-BUTTON', int(M5.BtnA.isPressed()))
3.2 uart0-cr M5.update(); print('LX6'+'-BUTTON', int(M5.BtnA.isPressed()))
SCRIPT
run stock "$stock/$device.bin" 3.5 --script "$out/stock.script" --tft-png "$out/stock.png"

if [[ $device == atom-matrix || $device == atom-matrix-v11 ]]; then
    cat >"$out/layout.script" <<'LAYOUT'
1.8 uart0-cr import M5; M5.begin(); from hardware import RGB; rgb=RGB(io=27,n=25,type='WS2812')
2.1 uart0-cr rgb.set_brightness(100); rgb.fill_color(0); rgb.set_color(8,0xff0000); rgb.set_color(20,0x0000ff)
LAYOUT
    run stock-layout "$stock/$device.bin" 2.5 --script "$out/layout.script" --tft-png "$out/stock-layout.png"
fi

if [[ $device == atomu || $device == atom-echo ]]; then
    cat >"$out/audio.script" <<'SCRIPT'
1.8 uart0-cr import M5, time; M5.begin(); buf=bytearray(3200); print('LX6'+'-MIC', M5.Mic.begin(), M5.Mic.record(buf,16000,False))
2.7 uart0-cr print('LX6'+'-SAMPLES', int(M5.Mic.isRecording()), sum(1 for x in buf if x), sum(buf)); M5.Mic.end()
SCRIPT
    if [[ $device == atom-echo ]]; then
        printf '%s\n' "3.0 uart0-cr print('LX6'+'-SPEAKER', M5.Speaker.begin()); M5.Speaker.setVolumePercentage(0.6); M5.Speaker.tone(440,400)" >>"$out/audio.script"
    fi
    run audio-silent "$stock/$device.bin" 3.8 --script "$out/audio.script"
    run audio-733 "$stock/$device.bin" 3.8 --script "$out/audio.script" --i2s-tone 0:733:0.02 --wav "$out/audio-733.wav"
    run audio-1234 "$stock/$device.bin" 3.8 --script "$out/audio.script" --i2s-tone 0:1234:0.04
fi
python3 "$tools/check_lx6_atoms.py" "$device" "$out"
