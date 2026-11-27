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
    pub vinout_mv: u16,
    pub temperature_c: i16,
    pub adc_mv: [u16; 2],
    pub charging: bool,
    pub power_off_requested: bool,
    pub driven_high: u8,
    pub led_enabled: bool,
    pub timer: Option<M5pm1Timer>,
    pub gpio: M5pm1Gpio,
}

#[derive(Debug)]
pub struct M5pm1Gpio {
    inputs: u8,
    external: u8,
    wake: u8,
    rising: u8,
    status: u8,
    mask: u8,
    irq_outputs: u8,
}

impl M5pm1Gpio {
    pub fn input_levels(&self) -> u8 { self.inputs }
}

impl Default for M5pm1Gpio {
    fn default() -> Self {
        Self { inputs: 0x1f, external: 0, wake: 0, rising: 0, status: 0, mask: 0x1f, irq_outputs: 0 }
    }
}

impl Default for M5pm1State {
    fn default() -> Self {
        Self { battery_mv: 3900, vbus_mv: 0, vinout_mv: 5000, temperature_c: 25, adc_mv: [1650, 1200],
            charging: false, power_off_requested: false,
            driven_high: 0, led_enabled: true, timer: None, gpio: M5pm1Gpio::default() }
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
    pub fn set_supply_fixture(&mut self, property: &str, value: &str) -> bool {
        match property {
            "charging" => match value.parse() {
                Ok(charging) => self.charging = charging,
                Err(_) => return false,
            },
            "temperature-c" => match value.parse::<i16>() {
                Ok(temperature @ -40..=125) => self.temperature_c = temperature,
                _ => return false,
            },
            "adc1-mv" | "adc2-mv" => {
                let Ok(mv @ 0..=3300) = value.parse::<u16>() else { return false };
                self.adc_mv[usize::from(property == "adc2-mv")] = mv;
            }
            "gpio-input-bits" => match value.parse::<u8>() {
                Ok(bits @ 0..=31) => for pin in 0..5 { self.gpio_input(pin, bits & (1 << pin) != 0); },
                _ => return false,
            },
            property => {
                let target = match property {
                    "vbat-mv" => &mut self.battery_mv,
                    "vin-mv" => &mut self.vbus_mv,
                    "vinout-mv" => &mut self.vinout_mv,
                    _ => return false,
                };
                let Ok(mv) = value.parse() else { return false };
                *target = mv;
            }
        }
        true
    }

    pub fn gpio_input(&mut self, pin: u8, high: bool) {
        if pin >= 5 { return; }
        let bit = 1 << pin;
        let was_high = self.gpio.inputs & bit != 0;
        self.gpio.external |= bit;
        self.gpio.inputs = (self.gpio.inputs & !bit) | (u8::from(high) << pin);
        if was_high != high && high == (self.gpio.rising & bit != 0) {
            self.gpio.status |= bit;
            if self.gpio.wake & bit != 0 {
                self.power_off_requested = false;
            }
        }
    }

    pub fn gpio_irq(&self, pin: u8) -> bool {
        self.gpio.irq_outputs & (1 << pin) != 0 && self.gpio.status & !self.gpio.mask != 0
    }

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
    pub fn registers(&self) -> &[u8] { &self.registers.regs }

    /// External input fixture, before direction, charge pin and pull resolution.
    pub fn gpio_inputs(&self) -> u8 {
        let state = self.state.lock().expect("M5PM1 state mutex poisoned");
        if state.timer.is_some() { state.gpio.input_levels() } else { self.gpio_inputs }
    }

    pub const ADDRESS: u8 = 0x6e;

    pub fn new(state: Arc<Mutex<M5pm1State>>) -> Self {
        {
            let mut state = state.lock().expect("M5PM1 state mutex poisoned");
            state.power_off_requested = false;
            state.driven_high = 0;
            state.led_enabled = true;
            state.gpio.irq_outputs = 0;
            state.gpio.wake = 0;
            state.gpio.rising = 0;
            state.gpio.mask = 0x1f;
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
        // BTN_STATUS bit 7 survives release for the guest's read_button_event.
        self.registers.regs[0x48] |= 0x80;
        if was_off { self.wake(4); }
    }
    pub fn set_wake_source(&mut self, bits: u8) {
        self.registers.regs[0x05] = bits & 0x7f;
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
        let mut device = Self::new(state);
        device.registers.regs[0x43] = 0x1f;
        device
    }

    pub fn with_wake_input(mut self, pin: u8) -> Self {
        self.registers.regs[0x18] |= 1 << pin;
        self.state.lock().expect("M5PM1 state mutex poisoned").gpio.wake |= 1 << pin;
        self
    }
}

impl I2cDevice for M5pm1 {
    fn start(&mut self, read: bool) -> bool { self.registers.start(read) }

    fn write(&mut self, byte: u8) -> bool {
        if !self.registers.first {
            let reg = self.registers.ptr;
            if matches!(reg, 0x00..=0x04 | 0x12 | 0x22..=0x25 | 0x48) {
                self.registers.ptr = reg.wrapping_add(1);
                return true;
            }
            // M5PM1 manual §ADC: bit zero starts conversion; channel 6 is
            // the internal temperature. The 12-bit result is little-endian.
            if reg == 0x2a {
                let regs = &mut self.registers.regs;
                if byte & 1 != 0 {
                    let state = self.state.lock().expect("M5PM1 state mutex poisoned");
                    let raw = match (byte >> 1) & 7 {
                        channel @ (1 | 2) => {
                            let mv = u32::from(state.adc_mv[usize::from(channel - 1)]);
                            ((mv * 4095 + 1650) / 3300) as u16
                        }
                        6 => ((i32::from(state.temperature_c) + 40) * 16).clamp(0, 4095) as u16,
                        _ => 0,
                    };
                    [regs[0x28], regs[0x29]] = raw.to_le_bytes();
                }
                return self.registers.write(byte & !1);
            }
            if matches!(reg, 0x06 | 0x18 | 0x19 | 0x40 | 0x43) {
                let mut state = self.state.lock().expect("M5PM1 state mutex poisoned");
                match reg {
                    0x06 => state.led_enabled = byte & 0x10 != 0,
                    0x18 => state.gpio.wake = byte & 0x1f,
                    0x19 => state.gpio.rising = byte & 0x1f,
                    0x40 => state.gpio.status &= byte,
                    0x43 => state.gpio.mask = byte & 0x1f,
                    _ => unreachable!(),
                }
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
            let mut irq = 0;
            for pin in 0..5 {
                let function = (functions >> (2 * pin)) & 3;
                if function != 0 { high &= !(1 << pin); }
                if function == 1 { irq |= r[0x10] & (1 << pin); }
            }
            let mut state = self.state.lock().expect("M5PM1 state mutex poisoned");
            state.driven_high = high;
            state.gpio.irq_outputs = irq;
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
        if state.timer.is_some() {
            let defaults = (pulled_high & !charging_bit) | (u8::from(!state.charging) << self.charging_pin);
            let inputs = (defaults & !state.gpio.external) | (state.gpio.inputs & state.gpio.external);
            r[0x12] = (inputs & !r[0x10]) | (r[0x11] & r[0x10]);
            r[0x04] = (r[0x04] & !1) | u8::from(state.vbus_mv > 1000);
            r[0x40] = state.gpio.status;
        } else {
            let inputs = (pulled_high & !self.external) | (self.gpio_inputs & self.external);
            let pins = (inputs & !r[0x10]) | (r[0x11] & r[0x10]);
            r[0x12] = (pins & !charging_bit) | if state.charging { 0 } else { charging_bit };
            r[0x04] = if state.charging { 0 } else { 2 };
        }
        [self.registers.regs[0x22], self.registers.regs[0x23]] = state.battery_mv.to_le_bytes();
        [self.registers.regs[0x24], self.registers.regs[0x25]] = state.vbus_mv.to_le_bytes();
        [self.registers.regs[0x26], self.registers.regs[0x27]] = state.vinout_mv.to_le_bytes();
        let reg = self.registers.ptr;
        let byte = self.registers.read();
        if reg == 0x48 { self.registers.regs[0x48] &= !0x80; }
        byte
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
    input_bits: u16,
    adc1_mv: u16,
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
            state, outputs: None, input_bits: 0x3fff, adc1_mv: 1650,
        }
    }

    pub fn input_bits(&self) -> u16 { self.input_bits }
    pub fn adc1_mv(&self) -> u16 { self.adc1_mv }

    pub fn set_fixture(&mut self, property: &str, value: &str) -> bool {
        let Ok(value) = value.parse::<u16>() else { return false };
        match property {
            "input-bits" if value <= 0x3fff => self.input_bits = value,
            "adc1-mv" if value <= 3300 => self.adc1_mv = value,
            _ => return false,
        }
        true
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

    fn read(&mut self) -> u8 {
        let regs = &mut self.registers.regs;
        for (bank, fixture) in self.input_bits.to_le_bytes().into_iter().enumerate() {
            regs[0x07 + bank] = (regs[0x03 + bank] & regs[0x05 + bank]) | (!regs[0x03 + bank] & fixture);
        }
        let raw = if matches!(regs[0x15] & 7, 0 | 1) {
            ((u32::from(self.adc1_mv) * 4095 + 1650) / 3300) as u16
        } else { 0 };
        [regs[0x16], regs[0x17]] = raw.to_le_bytes();
        self.registers.read()
    }
}
