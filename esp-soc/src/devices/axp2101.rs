//! X-Powers AXP2101 PMIC, register map from datasheet v1.0 §6.13.2 and
//! M5Unified 0.3.0 `utility/power/AXP2101_Class.inl`.
use esp_periph::i2c::I2cDevice;
use crate::board::VirtualCycle;

pub const AXP2101_ADDRESS: u8 = 0x34;

#[derive(Clone, Copy)]
struct KeyPress {
    release: VirtualCycle,
    threshold: Option<VirtualCycle>,
    long: bool,
}

pub struct Axp2101 {
    regs: [u8; 256],
    ptr: u8,
    first: bool,
    hz: u64,
    cycle: VirtualCycle,
    battery_mv: u16,
    percent: u8,
    charging: bool,
    vbus_mv: u16,
    powered: bool,
    irq_wakeup: Option<VirtualCycle>,
    key: Option<KeyPress>,
    pub last_off: Option<VirtualCycle>,
    pub last_on: Option<VirtualCycle>,
}

impl Axp2101 {
    pub fn new(hz: u64) -> Self {
        let mut a = Self {
            regs: [0; 256], ptr: 0, first: true, hz, cycle: 0,
            battery_mv: 3900, percent: 75, charging: false, vbus_mv: 5000,
            powered: true, irq_wakeup: None, key: None, last_off: None, last_on: None,
        };
        a.reset_outputs();
        a.regs[0x18] = 0x0a; // charger and gauge enabled
        a.regs[0x27] = 0x14;
        a.regs[0x30] = 0x03;
        a
    }
    fn reset_outputs(&mut self) {
        // CoreS3 starts with its system rail enabled. M5GFX enables the backlight.
        self.regs[0x80..=0x9a].fill(0);
        self.regs[0x80] = 1;
        self.regs[0x82] = 18; // DCDC1 3.3 V
        self.regs[0x40..=0x42].copy_from_slice(&[0xff, 0xfc, 0x5f]);
        self.regs[0x48..=0x4a].fill(0);
    }
    fn ms(&self, ms: u64) -> u64 { self.hz.saturating_mul(ms).div_ceil(1000).max(1) }
    pub fn set_battery(&mut self, mv: u16, percent: u8, charging: bool) {
        self.battery_mv = mv.min(0x3fff);
        self.percent = percent.min(100);
        self.charging = charging;
    }
    pub fn set_vbus_mv(&mut self, mv: u16) { self.vbus_mv = mv.min(0x3fff); }
    pub fn powered(&self) -> bool { self.powered }
    /// A register as the guest last wrote or the model set it (rail enables, voltages).
    pub fn reg(&self, r: u8) -> u8 { self.regs[r as usize] }
    // M5GFX `Light_M5StackCoreS3`, applied independently of the panel's internal on/off.
    pub fn backlight_on(&self) -> bool { self.powered && self.regs[0x90] & 0x80 != 0 }
    pub fn backlight_mv(&self) -> u16 { if self.backlight_on() { 500 + u16::from(self.regs[0x99] & 31) * 100 } else { 0 } }
    pub fn on_cause(&self) -> u8 { self.regs[0x20] }
    pub fn off_cause(&self) -> u8 { self.regs[0x21] }
    pub fn irq_low(&self) -> bool { self.powered && (0..3).any(|i| self.regs[0x40 + i] & self.regs[0x48 + i] != 0) }
    fn key_irq(&mut self, mask: u8) { self.regs[0x49] |= self.regs[0x41] & mask; }

    pub fn press_key(&mut self, duration_ms: u32) -> bool {
        if duration_ms == 0 || self.key.is_some() { return false; }
        let on_ms = [128, 512, 1000, 2000][usize::from(self.regs[0x27] & 3)];
        let irq_ms = 1000 + 500 * u64::from((self.regs[0x27] >> 4) & 3);
        let threshold_ms = if self.powered { irq_ms } else { on_ms };
        let release = self.cycle.saturating_add(self.ms(u64::from(duration_ms)));
        let threshold = (u64::from(duration_ms) >= threshold_ms)
            .then(|| self.cycle.saturating_add(self.ms(threshold_ms)));
        self.key = Some(KeyPress { release, threshold, long: false });
        if self.powered { self.key_irq(0x02); }
        true
    }

    fn power_on(&mut self, at: VirtualCycle, cause: u8) {
        self.reset_outputs();
        self.powered = true;
        self.regs[0x20] = cause;
        self.last_on = Some(at);
        self.irq_wakeup = None;
    }
    // Datasheet §6.5.4.2: the open-drain IRQ pin must be held low for 16 ms.
    pub fn set_irq_input(&mut self, low: bool) {
        if self.powered || !low { self.irq_wakeup = None; }
        else if self.irq_wakeup.is_none() {
            self.irq_wakeup = Some(self.cycle.saturating_add(self.ms(16)));
        }
    }
    pub fn next_deadline(&self) -> Option<VirtualCycle> {
        self.irq_wakeup.into_iter().chain(self.key.map(|k| k.threshold.unwrap_or(k.release)))
            .filter(|&at| at > self.cycle).min()
    }
    pub fn advance_to(&mut self, cycle: VirtualCycle) {
        self.cycle = self.cycle.max(cycle);
        if let Some(at) = self.irq_wakeup.filter(|&at| at <= cycle) { self.power_on(at, 0x02); }
        if let Some(mut key) = self.key {
            if let Some(at) = key.threshold.filter(|&at| at <= cycle) {
                if self.powered { self.key_irq(0x04); } else { self.power_on(at, 0x01); }
                key.long = true;
                key.threshold = None;
            }
            if key.release <= cycle {
                if self.powered { self.key_irq(if key.long { 0x01 } else { 0x09 }); }
                self.key = None;
            } else {
                self.key = Some(key);
            }
        }
    }

    fn read_reg(&self, reg: u8) -> u8 {
        let present = self.battery_mv != 0;
        let vbus = self.vbus_mv != 0;
        let charging = self.charging && vbus && present && self.regs[0x18] & 2 != 0;
        let adc = |channel, value: u16| {
            if self.regs[0x30] & (1u8 << channel) == 0 { return 0; }
            value.to_be_bytes()[usize::from(reg & 1)]
        };
        match reg {
            0x00 => (if vbus { 0x20 } else { 0 }) | (if present { 0x18 } else { 0 }),
            0x01 => (if charging { 0x22 } else if present && !vbus { 0x45 } else { 0x05 })
                | (u8::from(self.powered) << 4),
            0x03 => 0x4a,
            0x34 | 0x35 => adc(0u8, self.battery_mv),
            0x38 | 0x39 => adc(2u8, self.vbus_mv),
            0x3a | 0x3b => adc(3u8, if vbus { self.vbus_mv } else { self.battery_mv }),
            0xa4 => if present && self.regs[0x18] & 8 != 0 { self.percent } else { 0 },
            _ => self.regs[usize::from(reg)],
        }
    }
    fn write_reg(&mut self, reg: u8, value: u8) {
        let r = &mut self.regs[usize::from(reg)];
        match reg {
            0x00 | 0x01 | 0x03 | 0x20 | 0x21 | 0x34..=0x3d | 0xa4 => {}
            0x10 => {
                *r = value & !1;
                if self.powered && value & 1 != 0 {
                    self.powered = false;
                    self.regs[0x21] = 0x02;
                    self.last_off = Some(self.cycle);
                }
            }
            0x48..=0x4a => *r &= !value,
            0x92..=0x9a => *r = value & 0x1f,
            _ => *r = value,
        }
    }
}

impl I2cDevice for Axp2101 {
    fn start(&mut self, read: bool) -> bool { if !read { self.first = true; } true }
    fn write(&mut self, byte: u8) -> bool {
        if self.first { self.ptr = byte; self.first = false; }
        else { self.write_reg(self.ptr, byte); self.ptr = self.ptr.wrapping_add(1); }
        true
    }
    fn read(&mut self) -> u8 {
        let value = self.read_reg(self.ptr);
        self.ptr = self.ptr.wrapping_add(1);
        value
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn battery_status_and_adc_enable() {
        let mut a = Axp2101::new(1000);
        a.set_battery(4101, 89, true);
        assert_eq!(a.read_reg(0x03), 0x4a);
        assert_eq!(a.read_reg(0x00) & 0x28, 0x28);
        assert_eq!(a.read_reg(0x01) & 0x60, 0x20);
        assert_eq!(a.read_reg(0x38), 0);
        a.write_reg(0x30, 5);
        a.start(false); a.write(0x34); a.start(true);
        assert_eq!(u16::from_be_bytes([a.read(), a.read()]), 4101);
        assert_eq!(u16::from_be_bytes([a.read_reg(0x38), a.read_reg(0x39)]), 5000);
        assert_eq!(a.read_reg(0xa4), 89);
        a.set_vbus_mv(0);
        assert_eq!(a.read_reg(0x00) & 0x28, 8);
        assert_eq!(a.read_reg(0x01) & 0x60, 0x40);
    }

    #[test]
    fn key_irqs_are_timed_masked_and_write_one_to_clear() {
        let mut a = Axp2101::new(1000);
        a.write_reg(0x41, 0x0c);
        assert!(a.press_key(100));
        a.advance_to(99);
        assert!(!a.irq_low());
        a.advance_to(100);
        assert_eq!(a.read_reg(0x49), 0x08);
        a.write_reg(0x49, 0x04);
        assert!(a.irq_low());
        a.write_reg(0x49, 0x08);
        assert!(!a.irq_low());
        assert!(a.press_key(1600));
        a.advance_to(1600);
        assert_eq!(a.read_reg(0x49), 0x04);
    }

    #[test]
    fn power_off_and_debounced_irq_power_on() {
        let mut a = Axp2101::new(1000);
        a.write_reg(0x99, 20);
        a.write_reg(0x90, 0x80);
        assert_eq!(a.backlight_mv(), 2500);
        a.write_reg(0x10, 0x31);
        assert!(!a.powered() && !a.backlight_on());
        assert_eq!(a.read_reg(0x10), 0x30);
        a.set_irq_input(true);
        a.advance_to(15);
        assert!(!a.powered());
        a.set_irq_input(false);
        a.advance_to(16);
        assert!(!a.powered());
        a.set_irq_input(true);
        a.advance_to(32);
        assert!(a.powered() && !a.backlight_on());
        assert_eq!(a.read_reg(0x20), 2);
        assert_eq!(a.read_reg(0x21), 2);
    }
}
