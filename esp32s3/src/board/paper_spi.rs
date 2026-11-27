//! PaperMono C153 and PaperColor C151. GPIO and buses follow the manufacturer
//! schematics and M5GFX 28d47949 / M5Unified 96576a14.

use std::{cell::{Cell, RefCell}, io, net::{Ipv4Addr, UdpSocket}, rc::Rc, sync::{Arc, Mutex}};
use esp_periph::{
    i2c::{I2cDevice, Reg8Device, m5::{M5ioe1, M5pm1, M5pm1State}},
    sdmmc::{SdCard, SpiSdCard},
};
use esp_soc::{
    board::{BoardEdge, BoardModel, I2sCodecs, SpiPins, VirtualCycle},
    devices::{
        bmi270::Bmi270, ed2208::Ed2208, es7210::Es7210, es8311::Es8311,
        ft5x06::{Ft5x06, Ft5x06Points}, imu_motion::ImuMotion, rx8130::Rx8130,
        sht40::Sht40, ssd1677::Ssd1677, st25r3916::St25r3916, sx1262::Sx1262,
        ws2812::Ws2812Chain,
    },
};
use super::paper_i2c::PaperI2c;
use crate::periph::CPU_HZ;

const US: u64 = CPU_HZ / 1_000_000;
const MS: u64 = US * 1000;
const COLOR_SD: SpiPins = SpiPins { sclk: 1 << 15, mosi: 1 << 13, miso: Some(14), cs: 1 << 47 };
const LORA_CS: u64 = 1 << 41;
// C153 U14 NRST and antenna switch use IOE1 package pins 10 and 2.
// M5IOE1.h exposes their zero-based latch bits as 9 and 1.
const LORA_RESET: u16 = 1 << 9;
const LORA_ANTENNA_TX: u16 = 1 << 1;

struct Microphone(Rc<RefCell<Es7210>>);
impl I2cDevice for Microphone {
    fn start(&mut self, read: bool) -> bool { self.0.borrow_mut().start(read) }
    fn write(&mut self, byte: u8) -> bool { self.0.borrow_mut().write(byte) }
    fn read(&mut self) -> u8 { self.0.borrow_mut().read() }
}

struct NfcI2c {
    chip: Rc<RefCell<St25r3916>>,
    time: Rc<Cell<u64>>,
    outputs: Rc<Cell<u16>>,
}

impl I2cDevice for NfcI2c {
    fn start(&mut self, read: bool) -> bool {
        if !read { self.chip.borrow_mut().deselect(); }
        self.outputs.get() & (1 << 4) != 0
    }
    fn write(&mut self, byte: u8) -> bool {
        self.chip.borrow_mut().exchange(byte, self.time.get(), US);
        true
    }
    fn read(&mut self) -> u8 { self.chip.borrow_mut().exchange(0, self.time.get(), US) }
    fn stop(&mut self) { self.chip.borrow_mut().deselect(); }
}

struct RadioLink {
    chip: Sx1262,
    peer: Option<UdpSocket>,
    next_poll: u64,
    selected: bool,
    active: bool,
    tx: u64,
    rx: u64,
}

impl RadioLink {
    fn new(ports: Option<(u16, u16)>) -> io::Result<Self> {
        let peer = ports.map(|(local, remote)| -> io::Result<_> {
            let socket = UdpSocket::bind((Ipv4Addr::LOCALHOST, local))?;
            socket.connect((Ipv4Addr::LOCALHOST, remote))?;
            socket.set_nonblocking(true)?;
            Ok(socket)
        }).transpose()?;
        Ok(Self { chip: Default::default(), peer, next_poll: 0, selected: false, active: false, tx: 0, rx: 0 })
    }

    fn advance(&mut self, now: u64, antenna_tx: bool) {
        let channel = self.chip.channel();
        if let Some(data) = self.chip.advance(now) {
            self.tx += 1;
            if let Some(socket) = self.peer.as_ref().filter(|_| antenna_tx) {
                let mut datagram = vec![1];
                datagram.extend(channel);
                datagram.extend(data);
                let _ = socket.send(&datagram);
            }
        }
        if now < self.next_poll { return; }
        let Some(socket) = &self.peer else { return; };
        self.next_poll = now.saturating_add(2 * MS);
        let mut datagram = [0; 267];
        for _ in 0..16 {
            let Ok(n) = socket.recv(&mut datagram) else { break; };
            if !antenna_tx && n > 11 && datagram[0] == 1 && datagram[1..11] == channel
                && self.chip.receive(&datagram[11..n]) { self.rx += 1; }
        }
    }
}

enum Display { Mono(Ssd1677), Color(Ed2208) }

pub struct PaperSpi {
    display: Display,
    i2c: PaperI2c,
    pmic: Arc<Mutex<M5pm1State>>,
    expander: Rc<Cell<u16>>,
    touch: Ft5x06Points,
    imu: Arc<ImuMotion>,
    env: Rc<Cell<[f32; 2]>>,
    measurements: Rc<Cell<u64>>,
    now: Rc<Cell<VirtualCycle>>,
    nfc: Rc<RefCell<St25r3916>>,
    radio: Option<RadioLink>,
    dac: Es8311,
    adc: Rc<RefCell<Es7210>>,
    leds: Ws2812Chain,
    sd: Option<Box<SpiSdCard>>,
    output: u64,
    soc_was_on: bool,
    sd_selected: bool,
    busy_level: bool,
    nfc_level: bool,
    radio_levels: [bool; 2],
    edges: Vec<BoardEdge>,
}

impl PaperSpi {
    pub fn mono(ports: Option<(u16, u16)>) -> io::Result<Self> {
        let mut board = Self::new(Display::Mono(Ssd1677::new(MS)), (47, 48), false);
        board.radio = Some(RadioLink::new(ports)?);
        Ok(board)
    }

    pub fn color() -> Self { Self::new(Display::Color(Ed2208::new(MS)), (3, 2), true) }

    fn new(display: Display, pins: (u8, u8), busy_level: bool) -> Self {
        Self {
            display, i2c: PaperI2c::new(pins),
            pmic: Default::default(), expander: Default::default(),
            touch: Default::default(), imu: Default::default(),
            env: Rc::new(Cell::new([23.0, 45.0])), measurements: Default::default(), now: Default::default(),
            nfc: Default::default(), radio: None, dac: Es8311::new(),
            adc: Rc::new(RefCell::new(Es7210::new())), leds: Ws2812Chain::new(2),
            sd: None, output: 0, soc_was_on: true, sd_selected: false, busy_level, nfc_level: false,
            radio_levels: [false; 2], edges: Vec::new(),
        }
    }

    fn mono_board(&self) -> bool { matches!(self.display, Display::Mono(_)) }

    fn display_deadline(&self) -> Option<u64> {
        match &self.display { Display::Mono(p) => p.deadline(), Display::Color(p) => p.deadline() }
    }

    fn update_edges(&mut self) {
        let cycle = self.now.get();
        let busy = self.display_deadline().is_some();
        let mono = self.mono_board();
        let busy_pin = if mono { 18 } else { 11 };
        let level = if mono { busy } else { !busy };
        Self::pin_change(&mut self.edges, cycle, busy_pin, &mut self.busy_level, level);
        if mono {
            Self::pin_change(&mut self.edges, cycle, 6, &mut self.nfc_level, self.nfc.borrow().irq());
            if let Some(radio) = &self.radio {
                Self::pin_change(&mut self.edges, cycle, 5, &mut self.radio_levels[0], radio.active && radio.chip.dio1());
                Self::pin_change(&mut self.edges, cycle, 21, &mut self.radio_levels[1], radio.active && radio.chip.busy(cycle));
            }
        }
    }

    fn pin_change(edges: &mut Vec<BoardEdge>, cycle: u64, pin: u8, old: &mut bool, new: bool) {
        if *old != new {
            *old = new;
            edges.push(BoardEdge { cycle, pin, level: new });
        }
    }
}

impl BoardModel for PaperSpi {
    fn name(&self) -> &'static str { if self.mono_board() { "m5stack-papermono" } else { "m5stack-papercolor" } }

    fn i2c_devices(&mut self) -> Vec<(u8, u8, Box<dyn I2cDevice>)> {
        self.i2c.reset();
        self.i2c.attach(0x6e, M5pm1::with_power_timer(self.pmic.clone(), CPU_HZ));
        self.i2c.attach(0x32, Rx8130::new(self.now.clone(), CPU_HZ));
        if self.mono_board() {
            self.i2c.attach(0x4f, M5ioe1::new(Default::default(), 0).with_outputs(self.expander.clone()));
            self.i2c.attach(0x68, Bmi270::new(self.imu.clone(), CPU_HZ));
            self.i2c.attach(0x38, Ft5x06::new(self.touch.clone()));
            self.i2c.attach(0x75, Reg8Device::new("IP2315", &[]));
            self.i2c.attach(0x50, NfcI2c { chip: self.nfc.clone(), time: self.now.clone(), outputs: self.expander.clone() });
        } else {
            self.i2c.attach(0x44, Sht40::new(self.env.clone(), self.now.clone(), MS, self.measurements.clone()));
            self.i2c.attach(0x18, self.dac.i2c_port());
            self.i2c.attach(0x40, Microphone(self.adc.clone()));
        }
        self.i2c.controller_devices()
    }

    fn input_levels(&self) -> Vec<(u8, bool)> {
        let mut pins = self.i2c.levels().to_vec();
        // G12/G13 are the SDMMC command/clock lines. With no powered card
        // they follow the SoC's pad pulls, including the driver's board probe.
        if self.mono_board() { pins.extend([(2, true), (3, true), (4, true), (5, false), (6, false), (18, self.busy_level), (21, false)]); }
        else { pins.extend([(1, true), (9, true), (10, true), (11, self.busy_level)]); }
        pins
    }

    fn gpio_output_at(&mut self, cycle: VirtualCycle, _changes: &[(u8, bool)], enabled: u64, out: u64) {
        self.now.set(cycle);
        self.i2c.drive(cycle, enabled, out, &mut self.edges);
        let previous = std::mem::replace(&mut self.output, enabled & out);
        if let Display::Color(panel) = &mut self.display {
            if previous & (1 << 12) != 0 && self.output & (1 << 12) == 0 && enabled & (1 << 12) != 0 {
                panel.reset();
            }
            let select = enabled & !out & COLOR_SD.cs != 0;
            if self.sd_selected && !select {
                if let Some(card) = &mut self.sd { card.deselect(); }
            }
            self.sd_selected = select;
        }
        if let Some(radio) = &mut self.radio {
            let selected = enabled & !out & LORA_CS != 0;
            if radio.selected && !selected { radio.chip.deselect(cycle, US); }
            radio.selected = selected;
        }
        self.update_edges();
    }

    fn uses_spi_pins(&self) -> bool { true }
    fn has_spi_sd_slot(&self) -> bool { !self.mono_board() }
    fn insert_spi_sd(&mut self, card: SdCard) { self.sd = Some(Box::new(SpiSdCard::new(card))); }
    fn spi_transfer_pins(&mut self, _host: u8, pins: SpiPins, tx: &[u8], rx_len: usize) -> Vec<u8> {
        let now = self.now.get();
        let matches_panel = |mosi: u8, cs: u8| pins.sclk & (1 << 15) != 0 && pins.mosi & (1u64 << mosi) != 0 && pins.cs & (1u64 << cs) != 0;
        if let Some(radio) = &mut self.radio {
            if radio.active && pins.sclk & (1 << 39) != 0 && pins.mosi & (1 << 38) != 0 && pins.cs & LORA_CS != 0 {
                let mut result = vec![0xff; rx_len];
                for i in 0..tx.len().max(rx_len) {
                    let byte = radio.chip.exchange(tx.get(i).copied().unwrap_or(0));
                    if pins.miso == Some(40) {
                        if let Some(dest) = result.get_mut(i) { *dest = byte; }
                    }
                }
                return result;
            }
        }
        let mut read = None;
        match &mut self.display {
            Display::Mono(panel) if matches_panel(14, 16) => panel.write(now, self.output & (1 << 17) != 0, tx),
            Display::Color(panel) if matches_panel(13, 44) => panel.write(now, self.output & (1 << 43) != 0, tx),
            Display::Color(_) => {
                let ns = (u128::from(now) * 1_000_000_000 / u128::from(CPU_HZ)) as u64;
                read = self.sd.as_mut().and_then(|card| pins.transfer_sd(COLOR_SD, card, tx, rx_len, ns));
            }
            _ => {}
        }
        self.update_edges();
        read.unwrap_or_else(|| vec![0xff; rx_len])
    }

    fn touch_at(&mut self, cycle: VirtualCycle, x: u16, y: u16, down: bool) {
        if !self.mono_board() { return; }
        let mut points = self.touch.lock().unwrap();
        if points[0].is_some() != down {
            self.edges.push(BoardEdge { cycle, pin: 4, level: !down });
        }
        points[0] = down.then_some((x.min(479), y.min(799)));
    }

    fn next_deadline(&self) -> Option<u64> {
        self.display_deadline().into_iter()
            .chain(self.pmic.lock().unwrap().power_deadline())
            .chain(self.nfc.borrow().deadline())
            .chain(self.radio.as_ref().and_then(|r| r.chip.deadline(self.now.get())))
            .chain(self.radio.as_ref().filter(|r| r.active && r.peer.is_some()).map(|r| r.next_poll))
            .min()
    }
    fn set_imu_motion(&mut self, mode: u32) -> bool {
        if !self.mono_board() { return false; }
        self.imu.mode.store(mode, std::sync::atomic::Ordering::Relaxed);
        true
    }
    fn advance_to(&mut self, cycle: VirtualCycle) {
        self.now.set(cycle);
        let soc_on = {
            let mut pmic = self.pmic.lock().unwrap();
            pmic.advance_power_timer(cycle);
            !pmic.power_off_requested
        };
        if self.soc_was_on && !soc_on {
            match &mut self.display {
                Display::Mono(panel) => panel.power_loss(),
                Display::Color(panel) => panel.reset(),
            }
            self.nfc.borrow_mut().reset();
        }
        self.soc_was_on = soc_on;
        if self.mono_board() { self.imu.cycle.store(cycle, std::sync::atomic::Ordering::Relaxed); }
        match &mut self.display { Display::Mono(p) => p.advance_to(cycle), Display::Color(p) => p.advance_to(cycle) }
        self.nfc.borrow_mut().advance(cycle);
        if let Some(radio) = &mut self.radio {
            let outputs = self.expander.get();
            let pmic = self.pmic.lock().unwrap();
            let active = !pmic.power_off_requested && pmic.driven_high & 4 != 0
                && outputs & LORA_RESET != 0;
            drop(pmic);
            if radio.active != active {
                radio.active = active;
                radio.chip.reset(cycle);
            }
            if active { radio.advance(cycle, outputs & LORA_ANTENNA_TX != 0); }
        }
        self.update_edges();
    }
    fn soc_powered(&self) -> Option<bool> { Some(!self.pmic.lock().unwrap().power_off_requested) }
    fn take_edges(&mut self) -> Vec<BoardEdge> { std::mem::take(&mut self.edges) }

    fn display(&self) -> Option<(u32, u32, Vec<u16>, u64)> {
        let (w, h, frame, updates) = match &self.display {
            Display::Mono(p) => (Ssd1677::WIDTH, Ssd1677::HEIGHT, &p.frame, p.refreshes),
            Display::Color(p) => (Ed2208::WIDTH, Ed2208::HEIGHT, &p.frame, p.refreshes),
        };
        Some((w as u32, h as u32, frame.clone(), updates))
    }
    fn display_version(&self) -> u64 {
        match &self.display { Display::Mono(p) => p.refreshes, Display::Color(p) => p.refreshes }
    }
    fn display_frames(&self) -> u64 { self.display_version() }
    fn rmt_frame(&mut self, pin: u8, bits: &[bool]) {
        if !self.mono_board() && pin == 21 { self.leds.from_bits(bits); }
    }
    fn leds(&self) -> Option<(&[[u8; 3]], u64)> {
        (!self.mono_board()).then_some((&self.leds.leds, self.leds.updates))
    }
    fn i2s_codecs(&self) -> Option<I2sCodecs> {
        if self.mono_board() { return None; }
        let mut codecs = self.dac.i2s_codecs(0, 1, 40, 41, 38, 39);
        codecs.mck = Some(42);
        codecs.mic_lanes = self.adc.borrow().lanes();
        if self.output & (3u64 << 45) != 3u64 << 45 { codecs.speaker_gain = 0.0; }
        Some(codecs)
    }
    fn named_pin(&self, name: &str) -> Option<u8> {
        match (self.mono_board(), name) {
            (mono, "a" | "btn_a" | "button-a") => Some(if mono { 2 } else { 10 }),
            (mono, "b" | "btn_b" | "button-b") => Some(if mono { 3 } else { 9 }),
            (false, "c" | "btn_c" | "button-c") => Some(1),
            _ => None,
        }
    }
    fn accepts_command(&self, verb: &str) -> bool {
        match verb { "paper-tag" => self.mono_board(), "sht40" => !self.mono_board(), "paper-battery" => true, _ => false }
    }
    fn board_command(&mut self, _cycle: VirtualCycle, verb: &str, args: &[&str]) -> bool {
        match (verb, args) {
            ("paper-tag", [uid]) if self.mono_board() => {
                let tag = if *uid == "none" { Vec::new() } else {
                    if !uid.is_ascii() || !matches!(uid.len(), 8 | 14) || uid.starts_with("88") { return false; }
                    let tag = (0..uid.len()).step_by(2).map(|i| u8::from_str_radix(&uid[i..i + 2], 16)).collect::<Result<Vec<_>, _>>();
                    let Ok(tag) = tag else { return false; };
                    tag
                };
                self.nfc.borrow_mut().set_tag(tag);
                true
            }
            ("paper-battery", [millivolts, charging]) => {
                let (Ok(mv), Ok(chg)) = (millivolts.parse::<u16>(), charging.parse::<u8>()) else { return false; };
                if !(2500..=4500).contains(&mv) || chg > 1 { return false; }
                let mut pmic = self.pmic.lock().unwrap();
                pmic.battery_mv = mv;
                pmic.charging = chg != 0;
                true
            }
            ("sht40", [temperature, humidity]) if !self.mono_board() => {
                let (Ok(t), Ok(h)) = (temperature.parse(), humidity.parse()) else { return false; };
                if !(-40.0..=125.0).contains(&t) || !(0.0..=100.0).contains(&h) { return false; }
                self.env.set([t, h]);
                true
            }
            _ => false,
        }
    }
    fn report(&self) -> String {
        match &self.display {
            Display::Mono(p) => {
                let radio = self.radio.as_ref().unwrap();
                format!("[{}] SSD1677 refreshes={} bytes={} soft-i2c={} LoRa tx={} rx={} NFC selections={}",
                    self.name(), p.refreshes, p.bytes_written, self.i2c.transactions(), radio.tx, radio.rx, self.nfc.borrow().uid_reads)
            }
            Display::Color(p) => format!("[{}] ED2208 refreshes={} bytes={} soft-i2c={} SHT40 samples={} LED updates={}",
                self.name(), p.refreshes, p.bytes_written, self.i2c.transactions(), self.measurements.get(), self.leds.updates),
        }
    }
}
