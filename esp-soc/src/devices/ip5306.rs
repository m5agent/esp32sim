//! Injoinic IP5306 I2C variant, register specification v1.4.
use esp_periph::i2c::I2cDevice;

pub struct Ip5306 {
    control: [u8; 3],
    charger: [u8; 5],
    mv: u16,
    usb: bool,
    pointer: u8,
    select: bool,
}

impl Default for Ip5306 {
    fn default() -> Self {
        Self { control: [0x37, 0x1d, 0x0c], charger: [0; 5],
            mv: 3900, usb: true, pointer: 0, select: true }
    }
}

impl Ip5306 {
    pub const ADDRESS: u8 = 0x75;

    pub fn supply(&mut self, battery_mv: u16, usb: bool) {
        self.mv = battery_mv;
        self.usb = usb;
    }

    pub fn output_on(&self) -> bool { self.usb || self.mv >= 3000 && self.control[0] & 0x20 != 0 }
}

impl I2cDevice for Ip5306 {
    fn start(&mut self, read: bool) -> bool {
        self.select = !read;
        true
    }

    fn write(&mut self, value: u8) -> bool {
        if self.select {
            self.pointer = value;
            self.select = false;
            return true;
        }
        match self.pointer {
            0..=2 => self.control[usize::from(self.pointer)] = value,
            0x20..=0x24 => self.charger[usize::from(self.pointer - 0x20)] = value,
            _ => {}
        }
        self.pointer = self.pointer.wrapping_add(1);
        true
    }

    fn read(&mut self) -> u8 {
        let full = self.mv >= 4200 + u16::from((self.charger[2] >> 2) & 3) * 100;
        let data = match self.pointer {
            0..=2 => self.control[usize::from(self.pointer)],
            0x20..=0x24 => self.charger[usize::from(self.pointer - 0x20)],
            0x70 => u8::from(self.usb && self.mv != 0 && self.control[0] & 0x10 != 0 && !full) << 3,
            0x71 => u8::from(full) << 3,
            0x72 => u8::from(!self.output_on()) << 2,
            0x78 => match self.mv {
                4000.. => 0x0f,
                3800.. => 0x1f,
                3650.. => 0x3f,
                3450.. => 0x7f,
                _ => 0xff,
            },
            _ => 0,
        };
        self.pointer = self.pointer.wrapping_add(1);
        data
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn read_reg(chip: &mut Ip5306, reg: u8) -> u8 {
        chip.start(false);
        chip.write(reg);
        chip.start(true);
        chip.read()
    }

    #[test]
    fn load_and_charging_use_the_applied_supply() {
        let mut chip = Ip5306::default();
        assert_eq!(read_reg(&mut chip, 0x70), 8);
        chip.supply(4200, true);
        assert_eq!(read_reg(&mut chip, 0x71), 8);
        assert_eq!(read_reg(&mut chip, 0x78), 15);
        chip.supply(3500, false);
        assert_eq!(read_reg(&mut chip, 0x70), 0);
        assert_eq!(read_reg(&mut chip, 0x78), 0x7f);
        chip.start(false);
        chip.write(0);
        chip.write(0);
        assert!(!chip.output_on());
    }
}
