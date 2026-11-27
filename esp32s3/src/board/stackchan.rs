//! StackChan base on the CoreS3 host. M5StackChan 1.1.1 drives the base's RGB
//! over PY32 I²C and drives the two SCSCL devices on UART1 GPIO 6/7.
use std::{cell::RefCell, rc::Rc};

use esp_periph::i2c::I2cDevice;
use esp_soc::{devices::{ina226::Ina226, scscl::ScsclPair, st25r3916::St25r3916}, uart::{UartInput, UartRoute}};

use super::cardputer_adv_ir::NecReceiver;
use crate::periph::CPU_HZ;

const SERVO_BAUD: u32 = 1_000_000;
const SERVO_ZERO: [u16; 2] = [460, 620];
const PY32: u8 = 0x6f;
const SI12T: u8 = 0x68;
const INA: u8 = 0x41;
const NFC: u8 = 0x50;
pub const IR_TX: u8 = 5;
pub const IR_RX: u8 = 10;

struct BaseExpander {
    regs: [u8; 0x70],
    leds: [[u8; 3]; 12],
    updates: u64,
    colors: u8,
}

impl BaseExpander {
    fn new() -> Self {
        let mut regs = [0; 0x70];
        regs[2] = 4; // M5GFX StackChan probe's minimum supported PY32 firmware revision.
        Self { regs, leds: [[0; 3]; 12], updates: 0, colors: 0 }
    }
    fn power(&self) -> bool { self.regs[3] & self.regs[5] & 1 != 0 }

    fn write(&mut self, address: u8, byte: u8) {
        match address {
            0x24 => {
                let count = byte & 0x3f;
                self.regs[0x24] = count;
                if byte & 0x40 != 0 && self.regs[4] & 0x20 != 0 {
                    for (pixel, word) in self.leds.iter_mut().take(usize::from(count)).zip(self.regs[0x30..].as_chunks::<2>().0) {
                        let rgb = u16::from_le_bytes(*word);
                        let (r, g, b) = (((rgb >> 11) & 31) as u8, ((rgb >> 5) & 63) as u8, (rgb & 31) as u8);
                        *pixel = [r << 3 | r >> 2, g << 2 | g >> 4, b << 3 | b >> 2];
                        let channels = u8::from(r != 0) | u8::from(g != 0) << 1 | u8::from(b != 0) << 2;
                        self.colors |= 1 << channels;
                    }
                    self.updates += 1;
                }
            }
            3..=6 | 9..=0x10 | 0x13..=0x23 | 0x25..=0x6f => self.regs[usize::from(address)] = byte,
            0x11 | 0x12 => self.regs[usize::from(address)] &= !byte,
            _ => {}
        }
    }
    fn read(&self, address: u8) -> u8 {
        match address {
            7 | 8 => {
                let port = usize::from(address - 7);
                let outputs = self.regs[3 + port];
                (outputs & self.regs[5 + port]) | (!outputs & self.regs[9 + port])
            }
            r => self.regs.get(usize::from(r)).copied().unwrap_or(0),
        }
    }
}

struct Hardware {
    cycle: u64,
    base: BaseExpander,
    touch: [u8; 0x13],
    ina: Ina226,
    nfc: St25r3916,
}

struct Port {
    address: u8,
    hw: Rc<RefCell<Hardware>>,
    cursor: Option<u8>,
}

impl I2cDevice for Port {
    fn start(&mut self, read: bool) -> bool {
        if !read { self.cursor = None; }
        let mut hw = self.hw.borrow_mut();
        let cycle = hw.cycle;
        match self.address {
            NFC if !read => hw.nfc.deselect(),
            INA => { hw.ina.advance(cycle); return hw.ina.start(read); }
            _ => {}
        }
        true
    }
    fn write(&mut self, byte: u8) -> bool {
        let mut hw = self.hw.borrow_mut();
        match self.address {
            INA => hw.ina.write(byte),
            NFC => {
                let cycle = hw.cycle;
                hw.nfc.exchange(byte, cycle, CPU_HZ / 1_000_000);
                true
            }
            _ => {
                if let Some(address) = self.cursor {
                    match self.address {
                        PY32 => hw.base.write(address, byte),
                        SI12T if (2..=0x0f).contains(&address) => hw.touch[usize::from(address)] = byte,
                        _ => {}
                    }
                    self.cursor = Some(address.wrapping_add(1));
                } else { self.cursor = Some(byte); }
                true
            }
        }
    }
    fn read(&mut self) -> u8 {
        let mut hw = self.hw.borrow_mut();
        let address = self.cursor.unwrap_or(0);
        match self.address {
            INA => hw.ina.read(),
            NFC => {
                let cycle = hw.cycle;
                hw.nfc.exchange(0, cycle, CPU_HZ / 1_000_000)
            }
            PY32 => {
                self.cursor = Some(address.wrapping_add(1));
                hw.base.read(address)
            }
            _ => {
                self.cursor = Some(address.wrapping_add(1));
                let blocked = address == 0x10 && hw.touch[9] & 4 == 0;
                if blocked { 0 } else { hw.touch.get(usize::from(address)).copied().unwrap_or(0) }
            }
        }
    }
}

pub struct StackChan {
    hw: Rc<RefCell<Hardware>>,
    pub servo: ScsclPair,
    pub ir: NecReceiver,
    leds: [[u8; 3]; 12],
    updates: u64,
    ir_active: bool,
    ir_pwm: bool,
}

impl StackChan {
    pub fn new() -> Self {
        Self {
            hw: Rc::new(RefCell::new(Hardware {
                cycle: 0, base: BaseExpander::new(), touch: [0; 0x13],
                ina: Ina226::new(CPU_HZ, 10_000, false, 7400, 120), nfc: St25r3916::default(),
            })),
            servo: ScsclPair::new(CPU_HZ, SERVO_ZERO),
            ir: NecReceiver::default(), leds: [[0; 3]; 12], updates: 0, ir_active: false, ir_pwm: false,
        }
    }

    pub fn ports(&self) -> Vec<(u8, Box<dyn I2cDevice>)> {
        [PY32, SI12T, INA, NFC].into_iter().map(|address| {
            (address, Box::new(Port { address, hw: self.hw.clone(), cursor: None }) as Box<dyn I2cDevice>)
        }).collect()
    }
    pub fn uart_tx(&mut self, cycle: u64, route: UartRoute, byte: u8) {
        if route.transmits_on(6) && route.matches_baud(SERVO_BAUD) && self.hw.borrow().base.power() {
            self.servo.transmit(cycle, byte);
        }
    }
    pub fn uart_rx(&mut self, cycle: u64) -> Vec<UartInput> {
        self.servo.receive(cycle).filter(|_| self.hw.borrow().base.power())
            .map_or_else(Vec::new, |data| vec![UartInput::new(7, SERVO_BAUD, data)])
    }
    pub fn advance(&mut self, cycle: u64) {
        let mut hw = self.hw.borrow_mut();
        hw.cycle = cycle;
        hw.nfc.advance(cycle);
        if self.updates != hw.base.updates {
            self.updates = hw.base.updates;
            self.leds = hw.base.leds;
        }
        self.servo.advance(cycle);
    }
    pub fn deadline(&self) -> Option<u64> {
        self.servo.deadline().into_iter().chain(self.hw.borrow().nfc.deadline()).min()
    }
    pub fn carrier_at(&mut self, cycle: u64, active: bool) -> Option<bool> {
        if self.ir_active == active { return None; }
        self.ir_active = active;
        self.ir.carrier(cycle, active);
        Some(!active)
    }
    pub fn gpio_ir(&mut self, cycle: u64, active: bool) -> Option<bool> {
        if self.ir_pwm { return None; }
        self.carrier_at(cycle, active)
    }
    pub fn pwm_ir(&mut self, cycle: u64, pwm: Option<(f64, u32)>) -> Option<bool> {
        let was_pwm = std::mem::replace(&mut self.ir_pwm, pwm.is_some());
        if pwm.is_none() && !was_pwm { return None; }
        self.carrier_at(cycle, pwm.is_some_and(|(hz, duty)| (30_000.0..=45_000.0).contains(&hz) && (1..65535).contains(&duty)))
    }
    pub fn leds(&self) -> (&[[u8; 3]], u64) { (&self.leds, self.updates) }

    pub fn accepts_command(verb: &str) -> bool {
        matches!(verb, "head-touch" | "base-supply" | "nfc-tag" | "nfc-field" | "nfc-reader" | "nfc-reply" | "servo-angle" | "servo-pose")
    }
    pub fn command(&mut self, verb: &str, args: &[&str]) -> bool {
        let mut hw = self.hw.borrow_mut();
        match (verb, args) {
            ("head-touch", [pad, strength]) => {
                let (Ok(pad @ 0..=2), Ok(level @ 0..=3)) = (pad.parse::<u8>(), strength.parse::<u8>()) else { return false };
                let shift = 2 * (2 - pad);
                hw.touch[0x10] = hw.touch[0x10] & !(3 << shift) | (level << shift);
            }
            ("base-supply", [mv, ma]) => {
                let (Ok(mv), Ok(ma)) = (mv.parse(), ma.parse()) else { return false };
                hw.ina.supply(mv, ma);
            }
            ("nfc-field", ["on"]) => hw.nfc.set_external_field(true),
            ("nfc-field", ["off"]) => hw.nfc.set_external_field(false),
            ("nfc-reader", [hex]) if hex.len().is_multiple_of(2) && (2..=512).contains(&hex.len()) && hex.is_ascii() => {
                let bytes: Option<Vec<_>> = (0..hex.len()).step_by(2)
                    .map(|i| u8::from_str_radix(&hex[i..i+2], 16).ok()).collect();
                let Some(bytes) = bytes else { return false };
                let cycle = hw.cycle;
                if let Some(reply) = hw.nfc.reader_frame(cycle, CPU_HZ / 1_000_000, &bytes) {
                    eprintln!("[emu] StackChan NFC auto response: {reply:02x?}");
                }
            }
            ("nfc-reply", []) => {
                if let Some(reply) = hw.nfc.take_target_reply() {
                    eprintln!("[emu] StackChan NFC guest response: {reply:02x?}");
                }
            }
            ("nfc-tag", ["none"]) => hw.nfc.set_tag(Vec::new()),
            ("nfc-tag", [hex]) if matches!(hex.len(), 8 | 14) && hex.is_ascii() => {
                let uid: Option<Vec<_>> = (0..hex.len()).step_by(2).map(|i| u8::from_str_radix(&hex[i..i+2], 16).ok()).collect();
                let Some(uid) = uid else { return false };
                hw.nfc.set_tag(uid);
            }
            ("servo-pose", [yaw, pitch]) => {
                let (Ok(yaw @ 0..=1000), Ok(pitch @ 0..=1000)) = (yaw.parse::<u16>(), pitch.parse::<u16>()) else {
                    return false;
                };
                self.servo.pose([yaw, pitch]);
            }
            ("servo-angle", []) => {
                let [yaw, pitch] = self.angles();
                eprintln!("[emu] StackChan servo-angle: yaw={yaw:.3} pitch={pitch:.3}");
            }
            _ => return false,
        }
        true
    }
    fn angles(&self) -> [f64; 2] {
        let positions = self.servo.positions();
        std::array::from_fn(|i| (f64::from(positions[i]) - f64::from(SERVO_ZERO[i])) * 0.3125)
    }
    pub fn report(&self) -> String {
        let hw = self.hw.borrow();
        let [yaw, pitch] = self.angles();
        format!("[emu] StackChan: rgb-updates={} rgb-colors={:#04x}, servo-reads={} servo-writes={}, yaw={yaw:.3} pitch={pitch:.3}, nfc-selections={}, ir={}",
                hw.base.updates, hw.base.colors, self.servo.reads, self.servo.writes, hw.nfc.uid_reads, self.ir.count)
    }
}
