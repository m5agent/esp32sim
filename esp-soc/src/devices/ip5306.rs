//! Injoinic IP5306 I2C variant, register document v1.21.
//! Charger flags and gauge LEDs are supplied independently of the boost controls.
use esp_periph::i2c::I2cDevice;

pub const ADDRESS: u8 = 0x75;

pub struct Ip5306 {
    regs: [u8; 0x25],
    pointer: Option<u8>,
    selecting: bool,
    gauge: u8,
    charging: bool,
    full: bool,
}

impl Default for Ip5306 {
    fn default() -> Self {
        let mut regs = [0; 0x25];
        regs[0] = 0xbe;
        Self { regs, pointer: None, selecting: true, gauge: 0x0f, charging: false, full: false }
    }
}

impl Ip5306 {
    /// The gauge exposes four cumulative LED bits, with a set bit meaning off.
    pub fn set_battery(&mut self, bars: u8, charging: bool, full: bool) {
        self.gauge = (0x0f << bars.min(4)) & 0x0f;
        self.charging = charging;
        self.full = full;
    }
}

impl I2cDevice for Ip5306 {
    fn start(&mut self, read: bool) -> bool {
        self.selecting = !read;
        true
    }

    fn write(&mut self, byte: u8) -> bool {
        if self.selecting {
            self.pointer = Some(byte);
            self.selecting = false;
        } else if let Some(reg @ (0..=2 | 6 | 0x20..=0x24)) = self.pointer {
            self.regs[usize::from(reg)] = byte;
        }
        true
    }

    fn read(&mut self) -> u8 {
        match self.pointer {
            Some(0x70) => u8::from(self.charging && self.regs[0] & 0x10 != 0) << 3,
            Some(0x71) => u8::from(self.full) << 3,
            Some(0x78) => (self.gauge << 4) | 0x0f,
            Some(0x72 | 0x77) => 0,
            Some(reg) => self.regs.get(usize::from(reg)).copied().unwrap_or(0xff),
            None => 0xff,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn read_at(dev: &mut Ip5306, reg: u8) -> u8 {
        dev.start(false); dev.write(reg); dev.start(true); dev.read()
    }

    #[test]
    fn gauge_and_charge_flags_follow_inputs_and_charge_enable() {
        let mut dev = Ip5306::default();
        assert_eq!(read_at(&mut dev, 0x78), 0xff);
        assert_eq!(read_at(&mut dev, 0x70), 0);
        dev.set_battery(2, true, false);
        assert_eq!(read_at(&mut dev, 0x78) >> 4, 0x0c);
        assert_eq!(read_at(&mut dev, 0x70), 8);
        dev.start(false); dev.write(0); dev.write(0xae);
        assert_eq!(read_at(&mut dev, 0x00), 0xae);
        assert_eq!(read_at(&mut dev, 0x70), 0);
        dev.set_battery(4, true, true);
        assert_eq!(read_at(&mut dev, 0x78) >> 4, 0);
        assert_eq!(read_at(&mut dev, 0x71), 8);
    }
}
