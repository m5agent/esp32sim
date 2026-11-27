//! X-Powers AXP192, register map v1.1. Models register-controlled supplies,
//! GPIO modes, charger status and battery ADC readback used by M5Unified.
use esp_periph::i2c::I2cDevice;

pub const AXP192_ADDRESS: u8 = 0x34;

pub struct Axp192 {
    regs: [u8; 256],
    ptr: u8,
    first: bool,
    battery_mv: u16,
    battery_ma: Option<i16>,
    vbus_mv: u16,
    powered: bool,
}

impl Default for Axp192 {
    fn default() -> Self {
        let mut regs = [0; 256];
        regs[0x12] = 0x07;
        regs[0x28] = 0xcf;
        regs[0x33] = 0xc0;
        Self { regs, ptr: 0, first: true, battery_mv: 3900, battery_ma: Some(0), vbus_mv: 5000, powered: true }
    }
}

impl Axp192 {
    /// Standalone Stick PMIC startup has both display LDOs enabled and the
    /// board supply determines the simulated battery current.
    pub fn stick() -> Self {
        let mut pmic = Self { battery_ma: None, ..Self::default() };
        pmic.regs[0x12] = 0x0d;
        pmic.regs[0x28] = 0xcc;
        pmic.regs[0x36] = 0x0c;
        pmic.regs[0x42] = 3;
        pmic.regs[0x82] = 0xff;
        pmic
    }

    pub fn set_battery(&mut self, mv: u16, ma: i16) {
        self.battery_mv = mv;
        self.battery_ma = Some(ma);
    }

    pub fn reg(&self, reg: u8) -> u8 { self.regs[usize::from(reg)] }
    pub fn set_battery_mv(&mut self, mv: u16) { self.battery_mv = mv; }
    pub fn set_vbus_mv(&mut self, mv: u16) { self.vbus_mv = mv; }
    pub fn powered(&self) -> bool { self.powered && self.reg(0x12) & 1 != 0 }

    pub fn ldo_mv(&self, rail: u8) -> u16 {
        let shift = match rail { 2 => 4, 3 => 0, _ => return 0 };
        if !self.powered || self.reg(0x12) & (1 << rail) == 0 { return 0; }
        1800 + 100 * u16::from((self.reg(0x28) >> shift) & 15)
    }

    pub fn power_on(&mut self) {
        self.regs[0x12] |= 1;
        self.powered = true;
    }

    pub fn power_key(&mut self, long: bool) {
        if !self.powered() { self.power_on(); }
        self.regs[0x46] |= if long { 1 } else { 2 };
    }

    pub fn irq_low(&self) -> bool {
        [(0x40, 0x44), (0x41, 0x45), (0x42, 0x46), (0x43, 0x47), (0x4a, 0x4d)]
            .iter().any(|&(enable, status)| self.reg(enable) & self.reg(status) != 0)
    }

    fn read_reg(&self, reg: u8) -> u8 {
        let adc = |enable, value: u16, low_bits: u32| {
            if self.reg(0x82) & enable == 0 { return 0; }
            let value = value.min((256u16 << low_bits) - 1u16);
            if reg & 1 == 0 { (value >> low_bits) as u8 } else { (value & ((1u16 << low_bits) - 1u16)) as u8 }
        };
        let usb = self.vbus_mv >= 4000;
        let ma = self.battery_ma.unwrap_or_else(|| {
            if usb { if (1..4200).contains(&self.battery_mv) { 96 } else { 0 } }
            else if self.powered() { -64 } else { 0 }
        });
        let ma = if self.reg(0x33) & 0x80 == 0 { ma.min(0) } else { ma };
        match reg {
            0x00 => (u8::from(usb) * 0x30) | (u8::from(ma > 0) << 2),
            0x01 => u8::from(self.battery_mv != 0) << 5 | u8::from(ma > 0) << 6,
            0x03 => 0x03,
            0x5a | 0x5b => adc(8, (u32::from(self.vbus_mv) * 10 / 17) as u16, 4),
            0x78 | 0x79 => adc(0x80, ((u32::from(self.battery_mv) * 10) / 11) as u16, 4),
            0x7a | 0x7b => adc(0x40, (ma.max(0) as u16).saturating_mul(2), 5),
            0x7c | 0x7d => adc(0x40, ma.min(0).unsigned_abs().saturating_mul(2), 5),
            0x7e | 0x7f => {
                let aps_mv = if usb { 4700 } else { self.battery_mv };
                adc(2, (u32::from(aps_mv) * 10 / 14) as u16, 4)
            }
            _ => self.reg(reg),
        }
    }
}

impl I2cDevice for Axp192 {
    fn start(&mut self, read: bool) -> bool {
        if !read { self.first = true; }
        true
    }

    fn write(&mut self, byte: u8) -> bool {
        if self.first {
            self.ptr = byte;
            self.first = false;
        } else {
            match self.ptr {
                0x00..=0x03 | 0x56..=0x7f => {}
                0x44..=0x47 | 0x4d => self.regs[usize::from(self.ptr)] &= !byte,
                0x32 => {
                    self.powered &= byte & 0x80 == 0;
                    self.regs[0x32] = byte & 0x7f;
                }
                0xb8 => {
                    self.regs[0xb8] = byte & !0x20;
                    if byte & 0x20 != 0 { self.regs[0xb0..0xb8].fill(0); }
                }
                reg => self.regs[usize::from(reg)] = byte,
            }
            self.ptr = self.ptr.wrapping_add(1);
        }
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
    fn stick_registers_control_display_rails_power_key_and_supply() {
        let mut pmic = Axp192::stick();
        let write = |pmic: &mut Axp192, reg, value| {
            pmic.start(false); pmic.write(reg); pmic.write(value);
        };
        assert_eq!((pmic.ldo_mv(2), pmic.ldo_mv(3)), (3000, 3000));
        write(&mut pmic, 0x12, 0x09);
        write(&mut pmic, 0x28, 0x84);
        assert_eq!((pmic.ldo_mv(2), pmic.ldo_mv(3)), (0, 2200));
        pmic.set_vbus_mv(0);
        assert_eq!(pmic.read_reg(0x00) & 0x30, 0);
        assert_eq!(pmic.read_reg(0x5a), 0);
        write(&mut pmic, 0x32, 0x80);
        assert!(!pmic.powered());
        pmic.power_key(false);
        assert!(pmic.powered() && pmic.irq_low());
        write(&mut pmic, 0x46, 2);
        assert!(!pmic.irq_low());
    }

    #[test]
    fn adc_is_gated_and_voltage_and_current_have_distinct_packing() {
        let mut pmic = Axp192::default();
        pmic.set_battery(3850, 150);
        pmic.start(false); pmic.write(0x78); pmic.start(true);
        assert_eq!(pmic.read(), 0);
        pmic.start(false); pmic.write(0x82); pmic.write(0xc0);
        pmic.start(false); pmic.write(0x78); pmic.start(true);
        assert_eq!([pmic.read(), pmic.read(), pmic.read(), pmic.read()], [218, 12, 9, 12]);
    }
}
