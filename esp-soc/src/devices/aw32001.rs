//! AW32001 charger settings and charge-state readback, as used by M5Unified 0.3.0.

use esp_periph::i2c::I2cDevice;

pub const ADDRESS: u8 = 0x49;

pub struct Aw32001 {
    pub input_present: bool,
    pub cell_mv: u16,
    config: [u8; 8],
    pointer: u8,
    first: bool,
}

impl Aw32001 {
    pub fn new(input_present: bool, cell_mv: u16) -> Self {
        Self { input_present, cell_mv, config: [0; 8], pointer: 0, first: true }
    }
}

impl I2cDevice for Aw32001 {
    fn start(&mut self, read: bool) -> bool {
        self.first = !read;
        true
    }

    fn write(&mut self, byte: u8) -> bool {
        if self.first {
            self.pointer = byte;
            self.first = false;
        } else {
            if let Some(reg) = self.config.get_mut(usize::from(self.pointer)) {
                *reg = byte;
            }
            self.pointer = self.pointer.wrapping_add(1);
        }
        true
    }

    fn read(&mut self) -> u8 {
        let reg = self.pointer;
        self.pointer = self.pointer.wrapping_add(1);
        match reg {
            0x08 => {
                let max_mv = 3600 + 15 * u16::from(self.config[4] & 63);
                let state = if !self.input_present || self.config[1] & 8 != 0 { 0 }
                    else if self.cell_mv < 3000 { 1 }
                    else if self.cell_mv < max_mv { 2 } else { 3 };
                (state << 3) | (u8::from(self.input_present) << 2)
            }
            0x0a => 0x49,
            _ => self.config.get(usize::from(reg)).copied().unwrap_or(0),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn charge_state_tracks_supply_enable_and_voltage() {
        let mut charger = Aw32001::new(false, 3720);
        fn select(charger: &mut Aw32001, reg: u8) {
            charger.start(false); charger.write(reg);
        }
        select(&mut charger, 4); charger.write(40);
        select(&mut charger, 8);
        assert_eq!(charger.read() & 0x18, 0);
        charger.input_present = true;
        select(&mut charger, 8);
        assert_eq!(charger.read() & 0x18, 0x10);
        charger.cell_mv = 4250;
        select(&mut charger, 8);
        assert_eq!(charger.read() & 0x18, 0x18);
        select(&mut charger, 1); charger.write(8);
        select(&mut charger, 8);
        assert_eq!(charger.read() & 0x18, 0);
    }
}
