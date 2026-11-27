//! X-Powers AXP192, register map v1.1. Models register-controlled supplies,
//! GPIO modes, charger status and battery ADC readback used by M5Unified.
use esp_periph::i2c::I2cDevice;

pub struct Axp192 {
    regs: [u8; 256],
    ptr: u8,
    first: bool,
    battery_mv: u16,
    battery_ma: i16,
}

impl Default for Axp192 {
    fn default() -> Self {
        let mut regs = [0; 256];
        regs[0x12] = 0x07;
        regs[0x28] = 0xcf;
        regs[0x33] = 0xc0;
        Self { regs, ptr: 0, first: true, battery_mv: 3900, battery_ma: 0 }
    }
}

impl Axp192 {
    pub fn set_battery(&mut self, mv: u16, ma: i16) {
        self.battery_mv = mv;
        self.battery_ma = ma;
    }

    pub fn reg(&self, reg: u8) -> u8 { self.regs[usize::from(reg)] }

    fn read_reg(&self, reg: u8) -> u8 {
        let adc = |enable, value: u16, low_bits: u32| {
            if self.reg(0x82) & enable == 0 { return 0; }
            let value = value.min((256u16 << low_bits) - 1u16);
            if reg & 1 == 0 { (value >> low_bits) as u8 } else { (value & ((1u16 << low_bits) - 1u16)) as u8 }
        };
        let ma = if self.reg(0x33) & 0x80 == 0 { self.battery_ma.min(0) } else { self.battery_ma };
        match reg {
            0x00 => 0x30 | (u8::from(ma > 0) << 2),
            0x01 => u8::from(self.battery_mv != 0) << 5 | u8::from(ma > 0) << 6,
            0x03 => 0x03,
            0x78 | 0x79 => adc(0x80, ((u32::from(self.battery_mv) * 10) / 11) as u16, 4),
            0x7a | 0x7b => adc(0x40, ma.max(0) as u16 * 2, 5),
            0x7c | 0x7d => adc(0x40, ma.min(0).unsigned_abs().saturating_mul(2), 5),
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
