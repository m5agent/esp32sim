//! K141 board wiring, from M5GFX 0.3.0 and M5StamPLC 1.2.1 pin_config.h.
use std::{cell::RefCell, rc::Rc};

use esp_periph::{i2c::I2cDevice, sdmmc::{SdCard, SpiSdCard}};
use esp_soc::board::{BoardEdge, BoardModel, I2sCodecs, SpiPins, VirtualCycle};
use esp_soc::devices::{DcsPanel, SpiBitBang, aw9523::Aw9523, ina226::Ina226, lm75b::Lm75b,
    pi4ioe5v6408::Pi4ioe5v6408, rx8130::Rx8130, soft_i2c::{SoftI2c, SoftOp}};
use esp_soc::pins::PinRoutes;

use crate::periph::CPU_HZ;
use super::stamplc_modbus::{DIR, Peer};
use esp_soc::uart::{UartInput, UartRoute};

const LCD_CS: u8 = 12;
const LCD_CLK: u8 = 7;
const LCD_DATA: u8 = 8;
const LCD_DC: u8 = 6;
const RESET: u8 = 3;
const SD_CS: u8 = 10;
const SDA: u8 = 13;
const SCL: u8 = 15;
const IRQ: u8 = 14;
// ESP-IDF v5.4.2 ESP32-S3 gpio_sig_map.h, indexed by UART port.
const UART_RTS_OUT: [u32; 3] = [13, 16, 19];
const IO_PINS: [usize; 8] = [4, 5, 6, 7, 12, 13, 14, 15];
const SD_PINS: SpiPins = SpiPins { sclk: 1 << LCD_CLK, mosi: 1 << LCD_DATA, miso: Some(9), cs: 1 << SD_CS };

struct Devices {
    front: Pi4ioe5v6408,
    plc: Aw9523,
    ina: Ina226,
    temp: Lm75b,
    rtc: Rx8130,
    cycle: VirtualCycle,
    reset: bool,
    rgb_seen: u8,
    relays_seen: u16,
    last_relays: u8,
}

impl Devices {
    fn device(&mut self, address: u8) -> Option<&mut dyn I2cDevice> {
        match address {
            0x43 if !self.reset => Some(&mut self.front),
            0x59 if !self.reset => Some(&mut self.plc),
            0x40 => Some(&mut self.ina),
            Lm75b::ADDRESS => Some(&mut self.temp),
            Rx8130::ADDRESS => Some(&mut self.rtc),
            _ => None,
        }
    }

    fn rgb(&self) -> u8 { ((self.front.driven() & !self.front.driven_high()) >> 4) & 7 }
    fn relays(&self) -> u8 { self.plc.driven_high() as u8 & 15 }
    fn irq_high(&self) -> bool {
        !((!self.reset && (self.front.irq() || self.plc.intn_low())) ||
          self.ina.alert_low() || self.temp.os_low() || self.rtc.irq())
    }

    fn transact(&mut self, op: SoftOp) -> u8 {
        let result = match op {
            SoftOp::Start(address, read) => {
                self.ina.advance(self.cycle);
                self.rtc.advance(self.cycle);
                u8::from(self.device(address).is_some_and(|chip| chip.start(read)))
            }
            SoftOp::Write(address, byte) => {
                let ack = self.device(address).is_some_and(|chip| chip.write(byte));
                self.rgb_seen |= 1 << self.rgb();
                let relays = self.relays();
                if self.last_relays != relays {
                    self.relays_seen |= 1 << relays;
                    self.last_relays = relays;
                }
                u8::from(ack)
            }
            SoftOp::Read(address) => self.device(address).map_or(0xff, |chip| chip.read()),
            SoftOp::Stop(address) => {
                if let Some(chip) = self.device(address) { chip.stop(); }
                0
            }
            SoftOp::Sync => 0,
        };
        result
    }
}

struct Port(Rc<RefCell<Devices>>, u8);
impl I2cDevice for Port {
    fn pins(&self) -> Option<(u8, u8)> { Some((SDA, SCL)) }
    fn start(&mut self, read: bool) -> bool { self.0.borrow_mut().transact(SoftOp::Start(self.1, read)) != 0 }
    fn write(&mut self, byte: u8) -> bool { self.0.borrow_mut().transact(SoftOp::Write(self.1, byte)) != 0 }
    fn read(&mut self) -> u8 { self.0.borrow_mut().transact(SoftOp::Read(self.1)) }
    fn stop(&mut self) { self.0.borrow_mut().transact(SoftOp::Stop(self.1)); }
}

pub struct StamPlc {
    devices: Rc<RefCell<Devices>>,
    soft: SoftI2c,
    panel: DcsPanel,
    lcd_bits: SpiBitBang,
    lcd_selected: bool,
    lcd_clk: bool,
    lcd_read: Option<usize>,
    sd: Option<SpiSdCard>,
    sd_selected: bool,
    rs485: Peer,
    buttons: [Option<VirtualCycle>; 3],
    edges: Vec<BoardEdge>,
    sda_low: bool,
    lcd_drive: Option<bool>,
    released: Vec<u8>,
    irq_high: bool,
    cycle: VirtualCycle,
    version: u64,
    buzzer: Option<(u32, VirtualCycle)>,
    tones: Vec<(u32, u64)>,
}

impl StamPlc {
    pub fn new() -> Self {
        let mut front = Pi4ioe5v6408::default();
        for pin in 0..3 { front.set_input(pin, Some(true)); }
        let mut plc = Aw9523::new();
        plc.external = 0;
        Self {
            devices: Rc::new(RefCell::new(Devices {
                front, plc, ina: Ina226::new(CPU_HZ, 10_000, false, 12_000, 125),
                temp: Lm75b::default(), rtc: Rx8130::new(CPU_HZ), cycle: 0,
                reset: false, rgb_seen: 0, relays_seen: 1, last_relays: 0,
            })),
            soft: SoftI2c::idle(), panel: DcsPanel::st7789(),
            lcd_bits: SpiBitBang::new(LCD_CLK, LCD_DATA, LCD_CS),
            lcd_selected: false, lcd_clk: false, lcd_read: None, sd: None, sd_selected: false, rs485: Peer::default(),
            buttons: [None; 3], edges: Vec::new(), sda_low: false, lcd_drive: None, released: Vec::new(),
            irq_high: true, cycle: 0, version: 0, buzzer: None, tones: Vec::new(),
        }
    }

    fn lcd_byte(&mut self, byte: u8) {
        if !self.panel.dc {
            self.lcd_read = (byte == 0x04).then_some(0);
            self.version += 1;
        }
        self.panel.byte(byte);
    }

    fn irq(&mut self) {
        let high = self.devices.borrow().irq_high();
        if high != self.irq_high {
            self.irq_high = high;
            self.edges.push(BoardEdge { cycle: self.cycle, pin: IRQ, level: high });
        }
    }

    fn visible(&self) -> Vec<u16> {
        const W: usize = 240;
        const H: usize = 135;
        let front = &self.devices.borrow().front;
        if self.panel.sleeping || !self.panel.on || (front.driven() & !front.driven_high()) & 0x80 == 0 {
            return vec![0; W * H];
        }
        (0..H).flat_map(|y| (0..W).map(move |x| {
            let color = self.panel.gram[(40 + x) * self.panel.cols + 52 + H - 1 - y];
            if self.panel.inverted { color } else { !color }
        })).collect()
    }
}

impl BoardModel for StamPlc {
    fn name(&self) -> &'static str { "m5stack-stamplc" }

    fn i2c_devices(&mut self) -> Vec<(u8, u8, Box<dyn I2cDevice>)> {
        (0..2).flat_map(|bus| [0x43, 0x59, 0x40, Lm75b::ADDRESS, Rx8130::ADDRESS]
            .map(|address| (bus, address, Box::new(Port(self.devices.clone(), address)) as Box<dyn I2cDevice>))).collect()
    }

    fn uses_gpio_waveform(&self) -> bool { true }
    fn gpio_waveform_at(&mut self, cycle: VirtualCycle, routes: PinRoutes<'_>) {
        self.cycle = cycle;
        let pad = |pin| routes.software_pad_output(pin);
        self.rs485.de = pad(DIR) == Some(true);
        self.rs485.hardware_rts = 0;
        for (port, signal) in UART_RTS_OUT.into_iter().enumerate() {
            if routes.matrix_output(usize::from(DIR), signal) {
                self.rs485.hardware_rts |= 1 << port;
            }
        }
        let reset = pad(RESET) == Some(false);
        let selected = pad(LCD_CS) == Some(false);
        let clk = pad(LCD_CLK) == Some(true);
        {
            let mut devices = self.devices.borrow_mut();
            devices.cycle = cycle;
            if reset && !devices.reset {
                self.panel.reset();
                self.version += 1;
                self.lcd_read = None;
                let mut front = Pi4ioe5v6408::default();
                for (pin, due) in self.buttons.iter().enumerate() { front.set_input(pin, Some(due.is_none())); }
                devices.front = front;
                let inputs = devices.plc.external;
                devices.plc = Aw9523::new();
                devices.plc.external = inputs;
            }
            devices.reset = reset;
            let scl = pad(SCL) != Some(false);
            let sda = pad(SDA) != Some(false);
            if scl != self.soft.scl { self.soft.levels(scl, self.soft.sda, |op| devices.transact(op)); }
            if sda != self.soft.sda { self.soft.levels(scl, sda, |op| devices.transact(op)); }
        }
        if self.soft.pull_low != self.sda_low {
            self.sda_low = self.soft.pull_low;
            if self.sda_low { self.edges.push(BoardEdge { cycle, pin: SDA, level: false }); }
            else { self.released.push(SDA); }
        }
        self.panel.dc = pad(LCD_DC) == Some(true);
        self.lcd_bits.pin(LCD_CS, !selected);
        self.lcd_bits.pin(LCD_DATA, pad(LCD_DATA) == Some(true));
        if let Some(byte) = self.lcd_bits.pin(LCD_CLK, clk) {
            if !reset && pad(LCD_DATA).is_some() { self.lcd_byte(byte); }
        }
        let mut level = None;
        if selected && !reset && self.panel.dc && pad(LCD_DATA).is_none() {
            // Sitronix ST7789V2 §9.1.3. RDDID has one dummy bit before the ID bytes.
            if let Some(bit) = &mut self.lcd_read {
                if self.lcd_clk && !clk { *bit = bit.saturating_add(1); }
                let id = [0x85, 0x85, 0x52];
                level = Some(bit.checked_sub(1).is_some_and(|bit| id.get(bit / 8)
                    .is_some_and(|byte| byte & (0x80 >> (bit % 8)) != 0)));
            }
        }
        if self.lcd_drive != level {
            self.lcd_drive = level;
            if let Some(level) = level { self.edges.push(BoardEdge { cycle, pin: LCD_DATA, level }); }
            else { self.released.push(LCD_DATA); }
        }
        if self.lcd_selected && !selected { self.lcd_read = None; }
        self.lcd_selected = selected;
        self.lcd_clk = clk;
        let sd = pad(SD_CS) == Some(false);
        if self.sd_selected && !sd {
            if let Some(card) = &mut self.sd { card.deselect(); }
        }
        self.sd_selected = sd;
        self.irq();
    }

    fn uses_uart_pins(&self) -> bool { true }
    fn uart_tx(&mut self, cycle: u64, route: UartRoute, byte: u8) { self.rs485.transmit(cycle, route, byte); }
    fn uart_rx(&mut self, cycle: u64) -> Vec<UartInput> { self.rs485.receive(cycle) }

    fn uses_spi_pins(&self) -> bool { true }
    fn has_spi_sd_slot(&self) -> bool { true }
    fn reset_spi_sd_observation(&mut self) {
        if let Some(card) = self.sd.as_mut() { card.reset_observation(); }
    }
    fn spi_sd_state(&self) -> Option<esp_periph::sdmmc::SdMediaState> { self.sd.as_ref().map(esp_periph::sdmmc::SpiSdCard::media_state) }
    fn set_spi_sd_present(&mut self, _cycle: VirtualCycle, present: bool) -> bool {
        self.sd.as_mut().is_some_and(|card| card.set_present(present))
    }
    fn insert_spi_sd(&mut self, card: SdCard) { self.sd = Some(SpiSdCard::new(card)); }
    fn spi_transfer_pins(&mut self, host: u8, pins: SpiPins, tx: &[u8], rx_len: usize) -> Vec<u8> {
        let mut rx = vec![0xff; rx_len];
        if pins.sclk & (1 << LCD_CLK) == 0 { return rx; }
        if host == 2 && !self.devices.borrow().reset && pins.cs & (1 << LCD_CS) != 0 {
            if self.panel.dc && pins.miso == Some(LCD_DATA) && rx_len != 0 {
                if let Some(bit) = &mut self.lcd_read {
                    for byte in &mut rx {
                        *byte = [0x85, 0x85, 0x52].get(*bit / 8).copied().unwrap_or(0);
                        *bit += 8;
                    }
                    return rx;
                }
            }
            if pins.mosi & (1 << LCD_DATA) != 0 {
                for &byte in tx { self.lcd_byte(byte); }
            }
        } else if let Some(card) = &mut self.sd {
            let ns = (u128::from(self.cycle) * 1_000_000_000 / u128::from(CPU_HZ)) as u64;
            if let Some(input) = pins.transfer_sd(SD_PINS, card, tx, rx_len, ns) { return input; }
        }
        rx
    }

    fn display(&self) -> Option<(u32, u32, Vec<u16>, u64)> { Some((240, 135, self.visible(), self.display_version())) }
    fn display_version(&self) -> u64 { self.version + self.panel.pixels_written }
    fn display_frames(&self) -> u64 { self.panel.frames }
    fn input_levels(&self) -> Vec<(u8, bool)> { vec![(IRQ, self.irq_high), (SDA, true), (SCL, true)] }
    fn released_inputs(&mut self) -> Vec<u8> { std::mem::take(&mut self.released) }
    fn take_edges(&mut self) -> Vec<BoardEdge> {
        self.irq();
        std::mem::take(&mut self.edges)
    }
    fn next_deadline(&self) -> Option<VirtualCycle> {
        self.buttons.iter().flatten().copied().chain(self.devices.borrow().rtc.next_irq_check())
            .chain(self.rs485.deadline().filter(|&due| due > self.cycle)).min()
    }
    fn advance_to(&mut self, cycle: VirtualCycle) {
        self.cycle = cycle;
        let mut devices = self.devices.borrow_mut();
        devices.cycle = cycle;
        devices.rtc.advance(cycle);
        devices.ina.advance(cycle);
        for (pin, due) in self.buttons.iter_mut().enumerate() {
            if due.is_some_and(|at| at <= cycle) {
                *due = None;
                devices.front.set_input(pin, Some(true));
            }
        }
        drop(devices);
        self.irq();
    }

    fn observes_ledc(&self) -> bool { true }
    fn ledc_output_at(&mut self, cycle: VirtualCycle, pwm: &dyn Fn(u8) -> Option<(f64, u32)>) {
        // M5StamPLC::tone calls Arduino tone() on the GPIO44 buzzer.
        let tone = pwm(44).filter(|&(_, duty)| (1..65535).contains(&duty)).map(|(hz, _)| hz.round() as u32);
        if self.buzzer.map(|(hz, _)| hz) == tone { return; }
        if let Some((hz, start)) = self.buzzer {
            let micros = cycle.saturating_sub(start) / (CPU_HZ / 1_000_000);
            if micros >= 1_000 && self.tones.len() < 64 { self.tones.push((hz, micros)); }
        }
        self.buzzer = tone.map(|hz| (hz, cycle));
    }

    fn i2s_codecs(&self) -> Option<I2sCodecs> {
        // M5Unified Speaker_Class buzzer mode sends I2S PDM on DOUT without external clock pins.
        Some(I2sCodecs { port: 0, mic_port: 0, bck: 255, ws: 255, mck: None,
                         dout: 44, din: 255, speaker_gain: 1.0, mic_lanes: [false; 2] })
    }

    fn accepts_command(&self, verb: &str) -> bool { matches!(verb, "plc-key" | "plc-input" | "plc-supply" | "plc-temperature" | "plc-modbus-coil") }
    fn board_command(&mut self, cycle: VirtualCycle, verb: &str, args: &[&str]) -> bool {
        self.advance_to(cycle);
        let mut devices = self.devices.borrow_mut();
        match (verb, args) {
            ("plc-key", [key, ms]) => {
                let pin = match *key { "a" => 2, "b" => 1, "c" => 0, _ => return false };
                let Ok(ms) = ms.parse::<u32>() else { return false };
                if ms == 0 { return false; }
                self.buttons[pin] = Some(cycle.saturating_add(u64::from(ms) * (CPU_HZ / 1000)));
                devices.front.set_input(pin, Some(false));
            }
            ("plc-modbus-coil", [coil, level]) => {
                let Ok(coil @ 0..=3) = coil.parse::<u16>() else { return false };
                let on = match *level { "0" => false, "1" => true, _ => return false };
                return self.rs485.master_request(coil, on);
            }
            ("plc-input", [channel, level]) => {
                let Some(pin) = channel.parse::<usize>().ok().and_then(|channel| IO_PINS.get(channel)) else { return false };
                let high = match *level { "0" => 0, "1" => 1, _ => return false };
                devices.plc.external = (devices.plc.external & !(1 << pin)) | (high << pin);
            }
            ("plc-supply", [mv, ma]) => {
                let (Ok(mv), Ok(ma)) = (mv.parse(), ma.parse()) else { return false };
                devices.ina.supply(mv, ma);
            }
            ("plc-temperature", [value]) => {
                let Ok(value) = value.parse() else { return false };
                devices.temp.set_millicelsius(value);
            }
            _ => return false,
        }
        drop(devices);
        self.irq();
        true
    }
    fn report(&self) -> String {
        let d = self.devices.borrow();
        let mut report = format!("[emu] StamPLC: {} RAMWR, {} pixels, on={} sleep={} madctl={:#04x}, rgb={:#x}, rgb-seen={:#04x}, relays={:#x}, relay-states={:#06x}, io={:#06x}, soft-i2c={}, modbus-requests={}, modbus-echoes={}",
            self.panel.frames, self.panel.pixels_written, self.panel.on, self.panel.sleeping, self.panel.madctl,
            d.rgb(), d.rgb_seen, d.relays(), d.relays_seen, d.plc.inputs(), self.soft.transactions, self.rs485.requests, self.rs485.acknowledgments);
        report += &format!("\n[emu] StamPLC LEDC buzzer (Hz, us): {:?}", self.tones);
        if let Some(sd) = &self.sd { report += &format!("\n{}", sd.report()); }
        report
    }
}
