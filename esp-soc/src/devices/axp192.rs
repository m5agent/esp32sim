//! X-Powers AXP192 I2C power and ADC registers used by the M5StickC
//! and M5Unified AXP192 drivers. External supply and battery are host inputs.
use esp_periph::i2c::I2cDevice;

pub const AXP192_ADDRESS: u8 = 0x34;

pub struct Axp192 {
    regs: [u8; 256],
    pointer: u8,
    select: bool,
    battery_mv: u16,
    vbus_mv: u16,
    powered: bool,
}

impl Default for Axp192 {
    fn default() -> Self {
        let mut regs = [0; 256];
        regs[0x03] = 0x03;
        regs[0x12] = 0x0d;
        regs[0x28] = 0xcc;
        regs[0x33] = 0xc0;
        regs[0x36] = 0x0c;
        regs[0x42] = 3;
        regs[0x82] = 0xff;
        Self { regs, pointer: 0, select: true, battery_mv: 3900, vbus_mv: 5000, powered: true }
    }
}

impl Axp192 {
    pub fn set_battery_mv(&mut self, mv: u16) { self.battery_mv = mv; }
    pub fn set_vbus_mv(&mut self, mv: u16) { self.vbus_mv = mv; }
    pub fn powered(&self) -> bool { self.powered && self.regs[0x12] & 1 != 0 }

    pub fn ldo_mv(&self, rail: u8) -> u16 {
        let shift = match rail { 2 => 4, 3 => 0, _ => return 0 };
        if !self.powered || self.regs[0x12] & (1 << rail) == 0 { return 0; }
        1800 + 100 * u16::from((self.regs[0x28] >> shift) & 15)
    }

    pub fn power_on(&mut self) {
        self.regs[0x12] |= 1;
        self.powered = true;
    }

    pub fn power_key(&mut self, long: bool) {
        if !self.powered() {
            self.power_on();
        }
        self.regs[0x46] |= if long { 1 } else { 2 };
    }

    pub fn irq_low(&self) -> bool {
        [(0x40, 0x44), (0x41, 0x45), (0x42, 0x46), (0x43, 0x47), (0x4a, 0x4d)]
            .iter().any(|&(enable, status)| self.regs[enable] & self.regs[status] != 0)
    }

    fn usb(&self) -> bool { self.vbus_mv >= 4000 }
    fn charging(&self) -> bool {
        self.usb() && (1..4200).contains(&self.battery_mv) && self.regs[0x33] & 0x80 != 0
    }

    fn adc(&self, reg: u8, mv: u16, tenths_mv: u16, enable: u8) -> u8 {
        let raw = if self.regs[0x82] & enable != 0 {
            (u32::from(mv) * 10 / u32::from(tenths_mv)).min(4095) as u16
        } else { 0 };
        if reg & 1 == 0 { (raw >> 4) as u8 } else { raw as u8 & 15 }
    }

    fn read_register(&self, reg: u8) -> u8 {
        match reg {
            0x00 => (if self.usb() { 0x28 } else { 0 })
                      | (if self.charging() { 4 } else { 0 }),
            0x01 => (u8::from(self.battery_mv != 0) * 0x20) | (u8::from(self.charging()) * 0x40),
            0x5a..=0x5b => self.adc(reg, self.vbus_mv, 17, 8),
            0x78..=0x79 => self.adc(reg, self.battery_mv, 11, 0x80),
            0x7a => if self.charging() && self.regs[0x82] & 0x40 != 0 { 6 } else { 0 },
            0x7c => if self.powered() && !self.usb() && self.regs[0x82] & 0x40 != 0 { 4 } else { 0 },
            0x7e..=0x7f => self.adc(reg, if self.usb() { 4700 } else { self.battery_mv }, 14, 2),
            _ => self.regs[usize::from(reg)],
        }
    }
}

impl I2cDevice for Axp192 {
    fn start(&mut self, read: bool) -> bool {
        self.select = !read;
        true
    }

    fn write(&mut self, byte: u8) -> bool {
        if self.select {
            self.pointer = byte;
            self.select = false;
            return true;
        }
        let reg = &mut self.regs[usize::from(self.pointer)];
        match self.pointer {
            0x00..=0x03 | 0x56..=0x7f => {}
            0x44..=0x47 | 0x4d => *reg &= !byte,
            0x32 => {
                self.powered &= byte & 0x80 == 0;
                *reg = byte & 0x7f;
            }
            0xb8 => {
                *reg = byte & !0x20;
                if byte & 0x20 != 0 { self.regs[0xb0..0xb8].fill(0); }
            }
            _ => *reg = byte,
        }
        self.pointer = self.pointer.wrapping_add(1);
        true
    }

    fn read(&mut self) -> u8 {
        let value = self.read_register(self.pointer);
        self.pointer = self.pointer.wrapping_add(1);
        value
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn write_reg(pmic: &mut Axp192, reg: u8, value: u8) {
        assert!(pmic.start(false));
        assert!(pmic.write(reg));
        assert!(pmic.write(value));
    }

    fn read_adc(pmic: &mut Axp192, reg: u8) -> u16 {
        pmic.start(false);
        pmic.write(reg);
        pmic.start(true);
        (u16::from(pmic.read()) << 4) | u16::from(pmic.read())
    }

    #[test]
    fn pmic_rails_battery_and_key_are_visible_on_the_bus() {
        let mut pmic = Axp192::default();
        assert_eq!(pmic.ldo_mv(2), 3000);
        write_reg(&mut pmic, 0x12, 0x09);
        assert_eq!((pmic.ldo_mv(2), pmic.ldo_mv(3)), (0, 3000));
        write_reg(&mut pmic, 0x28, 0x84);
        assert_eq!(pmic.ldo_mv(3), 2200);
        for mv in [3500, 4100] {
            pmic.set_battery_mv(mv);
            assert_eq!(read_adc(&mut pmic, 0x78), mv * 10 / 11);
        }
        write_reg(&mut pmic, 0x82, 0);
        assert_eq!(read_adc(&mut pmic, 0x78), 0);
        assert!(!pmic.irq_low());
        pmic.power_key(false);
        pmic.power_key(true);
        write_reg(&mut pmic, 0x46, 2);
        assert!(pmic.irq_low());
        write_reg(&mut pmic, 0x46, 1);
        assert!(!pmic.irq_low());
        write_reg(&mut pmic, 0x32, 0x80);
        assert!(!pmic.powered());
        assert_eq!(pmic.ldo_mv(3), 0);
        pmic.power_key(false);
        assert!(pmic.powered());
    }
}
