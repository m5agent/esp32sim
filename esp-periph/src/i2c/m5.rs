//! M5Stack PM1 power/standby controls and IOE1 digital pin states.
//! Register sources: M5PM1 Chip User Manual v1.9, pp. 4–10, 12–18, 23;
//! M5IOE1 Chip User Manual, sections III and IV (links and wiring in docs/m5-i2c.md).
use super::{I2cDevice, Reg8Device};
use std::{cell::Cell, rc::Rc, sync::{Arc, Mutex}};

/// Host inputs and a latched shutdown request, shared with the board after I2C attachment.
#[derive(Debug)]
pub struct M5pm1State {
    pub battery_mv: u16,
    pub vbus_mv: u16,
    pub charging: bool,
    pub power_off_requested: bool,
    pub driven_high: u8,
    pub timer: Option<M5pm1Timer>,
}

impl Default for M5pm1State {
    fn default() -> Self {
        Self { battery_mv: 3900, vbus_mv: 0, charging: false, power_off_requested: false, driven_high: 0, timer: None }
    }
}

/// Virtual-time power actions for boards that wire M5PM1 to the SoC rail.
#[derive(Debug)]
pub struct M5pm1Timer {
    hz: u64,
    cycle: u64,
    alarm: Option<(u64, PowerAction)>,
}

#[derive(Debug)]
enum PowerAction { On, Off }

impl M5pm1State {
    pub fn power_deadline(&self) -> Option<u64> {
        self.timer.as_ref()?.alarm.as_ref().map(|&(cycle, _)| cycle)
    }

    pub fn advance_power_timer(&mut self, cycle: u64) {
        let Some(timer) = &mut self.timer else { return };
        timer.cycle = cycle;
        if timer.alarm.as_ref().is_some_and(|&(deadline, _)| cycle >= deadline) {
            self.power_off_requested = matches!(timer.alarm.take(), Some((_, PowerAction::Off)));
        }
    }
}

pub struct M5pm1 {
    registers: Reg8Device,
    state: Arc<Mutex<M5pm1State>>,
    charging_pin: u8,
    timer_at: Option<u64>,
    cycle: u64,
    hz: u64,
    external: u8,
    gpio_inputs: u8,
    gpio_events: u8,
    timer_wakes: u64,
}

impl M5pm1 {
    pub const ADDRESS: u8 = 0x6e;

    pub fn new(state: Arc<Mutex<M5pm1State>>) -> Self {
        {
            let mut state = state.lock().expect("M5PM1 state mutex poisoned");
            state.power_off_requested = false;
            state.driven_high = 0;
        }
        Self {
            registers: Reg8Device::new("m5pm1", &[
                (0x00, 0x50), (0x01, 0x20), (0x02, 0x05), (0x03, 0x06),
                (0x06, 0x17), (0x08, 0x40), (0x13, 0x1f), (0x49, 0x2a),
            ]),
            state, charging_pin: 2, timer_at: None, cycle: 0, hz: 240_000_000,
            external: 0, gpio_inputs: 0, gpio_events: 0, timer_wakes: 0,
        }
    }
    pub fn with_wiring(mut self, hz: u64, charging_pin: u8) -> Self {
        assert!(hz > 0 && charging_pin < 5);
        self.hz = hz;
        self.charging_pin = charging_pin;
        self
    }
    pub fn driven_high(&self) -> u8 {
        let r = &self.registers.regs;
        r[0x10] & r[0x11] & !r[0x13] & 0x1f
    }
    pub fn gpio_input(&mut self, pin: u8, level: bool) {
        if pin >= 5 { return; }
        let mask = 1 << pin;
        let old = self.gpio_inputs & mask != 0;
        self.external |= mask;
        self.gpio_inputs = (self.gpio_inputs & !mask) | if level { mask } else { 0 };
        if old == level { return; }
        let r = &mut self.registers.regs;
        if !level && r[0x43] & mask == 0 {
            r[0x40] |= mask;
            self.gpio_events |= mask;
        }
        if r[0x18] & mask != 0 && (r[0x19] & mask != 0) == level {
            self.wake(0x20);
        }
    }
    pub fn power_key(&mut self) {
        let was_off = self.state.lock().unwrap().power_off_requested;
        self.registers.regs[0x42] |= if was_off { 2 } else { 1 };
        if was_off { self.wake(4); }
    }
    pub fn gpio_events(&self) -> u8 { self.gpio_events }
    pub fn timer_wakes(&self) -> u64 { self.timer_wakes }
    pub fn drives_irq(&self, pin: u8) -> bool {
        pin < 4 && self.registers.regs[0x10] & (1 << pin) != 0
            && (self.registers.regs[0x16] >> (2 * pin)) & 3 == 1
            && self.irq_low()
    }
    fn irq_low(&self) -> bool {
        let r = &self.registers.regs;
        r[0x40] & !r[0x43] != 0 || r[0x41] & !r[0x44] != 0 || r[0x42] & !r[0x45] != 0
    }
    pub fn next_deadline(&self) -> Option<u64> { self.timer_at }
    pub fn advance(&mut self, cycle: u64) {
        self.cycle = self.cycle.max(cycle);
        if self.timer_at.is_none_or(|at| at > self.cycle) { return; }
        self.timer_at = None;
        let config = self.registers.regs[0x3c];
        match config & 7 {
            3 if self.state.lock().unwrap().power_off_requested => { self.wake(1); self.timer_wakes += 1; }
            4 => self.state.lock().unwrap().power_off_requested = true,
            _ => {}
        }
        if config & 8 != 0 { self.reload_timer(); }
    }
    fn wake(&mut self, source: u8) {
        self.registers.regs[5] |= source;
        self.state.lock().unwrap().power_off_requested = false;
    }
    fn reload_timer(&mut self) {
        let r = &self.registers.regs;
        let secs = u64::from(u32::from_le_bytes([r[0x38], r[0x39], r[0x3a], r[0x3b] & 0x7f]));
        self.timer_at = (r[0x3c] & 7 != 0 && secs != 0)
            .then(|| self.cycle.saturating_add(self.hz.saturating_mul(secs)));
    }

    pub fn with_power_timer(state: Arc<Mutex<M5pm1State>>, hz: u64) -> Self {
        state.lock().expect("M5PM1 state mutex poisoned").timer.get_or_insert(M5pm1Timer { hz, cycle: 0, alarm: None });
        Self::new(state)
    }
}

impl I2cDevice for M5pm1 {
    fn start(&mut self, read: bool) -> bool { self.registers.start(read) }

    fn write(&mut self, byte: u8) -> bool {
        if !self.registers.first {
            let reg = self.registers.ptr;
            if matches!(reg, 0x00..=0x04 | 0x12 | 0x22..=0x23) {
                self.registers.ptr = reg.wrapping_add(1);
                return true;
            }
            if matches!(reg, 0x05 | 0x40..=0x42) {
                self.registers.regs[reg as usize] &= byte;
                self.registers.ptr = reg.wrapping_add(1);
                return true;
            }
            if reg == 0x3c && byte & 7 == 0 { self.timer_at = None; }
            if reg == 0x0c && byte == 0xa1 {
                self.state.lock().expect("M5PM1 state mutex poisoned").power_off_requested = true;
            }
            if reg == 0x3d && byte == 0xa5 {
                let r = &self.registers.regs;
                let mut state = self.state.lock().expect("M5PM1 state mutex poisoned");
                if let Some(timer) = &mut state.timer {
                    let seconds = u32::from_le_bytes([r[0x38], r[0x39], r[0x3a], r[0x3b] & 0x7f]);
                    let deadline = timer.cycle.saturating_add(u64::from(seconds).saturating_mul(timer.hz));
                    // The power-on and power-off actions are idempotent. The relevant
                    // transition is the first expiry, even if ARM requests auto reload.
                    timer.alarm = match r[0x3c] & 7 {
                        3 => Some((deadline, PowerAction::On)),
                        4 => Some((deadline, PowerAction::Off)),
                        _ => None,
                    };
                } else {
                    drop(state);
                    self.reload_timer();
                }
            }
        }
        let update = !self.registers.first && matches!(self.registers.ptr, 0x10 | 0x11 | 0x13 | 0x16 | 0x17);
        let ack = self.registers.write(byte);
        if update {
            let r = &self.registers.regs;
            let functions = u16::from_le_bytes([r[0x16], r[0x17]]);
            let mut high = r[0x10] & r[0x11] & !r[0x13] & 0x1f;
            for pin in 0..5 {
                if (functions >> (2 * pin)) & 3 != 0 { high &= !(1 << pin); }
            }
            self.state.lock().expect("M5PM1 state mutex poisoned").driven_high = high;
        }
        ack
    }

    fn read(&mut self) -> u8 {
        let state = self.state.lock().expect("M5PM1 state mutex poisoned");
        let r = &mut self.registers.regs;
        let mut pulled_high = 0;
        for pin in 0..5 {
            let pull = if pin < 4 { (r[0x14] >> (pin * 2)) & 3 } else { r[0x15] & 3 };
            if pull == 1 { pulled_high |= 1 << pin; }
        }
        let charging_bit = 1 << self.charging_pin;
        let input = (pulled_high & !self.external) | (self.gpio_inputs & self.external);
        let pins = (input & !r[0x10]) | (r[0x11] & r[0x10]);
        r[0x12] = (pins & !charging_bit) | if state.charging { 0 } else { charging_bit };
        r[0x04] = if state.charging { 0 } else { 2 };
        [self.registers.regs[0x22], self.registers.regs[0x23]] = state.battery_mv.to_le_bytes();
        [self.registers.regs[0x24], self.registers.regs[0x25]] = state.vbus_mv.to_le_bytes();
        self.registers.read()
    }
}

/// Driven-high StopWatch signals. Input or released open-drain pins are treated as low;
/// external pulls and electrical behavior are not modeled.
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq)]
pub struct M5ioe1State {
    pub external_usb: bool,
    pub oled_rail: bool,
    pub oled_reset_high: bool,
    pub touch_reset_high: bool,
    pub audio_rail: bool,
    pub amp_enabled: bool,
    pub vibration_duty: u16,
    pub vibration_hz: u16,
}

pub struct M5ioe1 {
    registers: Reg8Device,
    state: Arc<Mutex<M5ioe1State>>,
    outputs: Option<Rc<Cell<u16>>>,
}

impl M5ioe1 {
    pub const ADDRESS: u8 = 0x4f;

    /// Use the firmware revision for the board's address family.
    pub fn with_revision(mut self, revision: u8) -> Self {
        self.registers.regs[0x02] = revision;
        self
    }

    /// UID is a board-supplied synthetic identity, not a fixed M5IOE1 chip ID.
    pub fn new(state: Arc<Mutex<M5ioe1State>>, uid: u16) -> Self {
        let [lo, hi] = uid.to_le_bytes();
        *state.lock().expect("M5IOE1 state mutex poisoned") = M5ioe1State::default();
        Self {
            registers: Reg8Device::new("m5ioe1", &[
                (0x00, lo), (0x01, hi), (0x02, b'A'), (0x13, 0xff), (0x14, 0x3f),
            ]),
            state, outputs: None,
        }
    }

    /// Optional physical output observation for boards that wire other GPIO banks.
    pub fn with_outputs(mut self, outputs: Rc<Cell<u16>>) -> Self {
        outputs.set(0);
        self.outputs = Some(outputs);
        self
    }
}

impl I2cDevice for M5ioe1 {
    fn start(&mut self, read: bool) -> bool { self.registers.start(read) }

    fn write(&mut self, byte: u8) -> bool {
        if !self.registers.first && self.registers.ptr <= 0x02 {
            self.registers.ptr += 1;
            return true;
        }
        let ack = self.registers.write(byte);
        let regs = &self.registers.regs;
        let high = regs[0x03] & regs[0x05] & !regs[0x13];
        if let Some(outputs) = &self.outputs {
            let high_b = regs[0x04] & regs[0x06] & !regs[0x14];
            outputs.set(u16::from_le_bytes([high, high_b]));
        }
        *self.state.lock().expect("M5IOE1 state mutex poisoned") = M5ioe1State {
            external_usb: high & 1 != 0,
            oled_rail: high & 0x80 != 0,
            oled_reset_high: high & 0x10 != 0,
            touch_reset_high: high & 0x08 != 0,
            audio_rail: high & 0x04 != 0,
            amp_enabled: regs[0x04] & regs[0x06] & !regs[0x14] & 2 != 0,
            vibration_duty: if regs[0x04] & !regs[0x14] & 1 != 0 && regs[0x1c] & 0x80 != 0 {
                u16::from_le_bytes([regs[0x1b], regs[0x1c] & 15])
            } else { 0 },
            vibration_hz: u16::from_le_bytes([regs[0x25], regs[0x26]]),
        };
        ack
    }

    fn read(&mut self) -> u8 { self.registers.read() }
}
