use esp_periph::i2c::I2cDevice;

/// Diodes PI4IOE5V6408, DS40583 Rev. 3-5, Tables 2–12.
/// Pins without an external driver resolve to the configured pull or low.
pub struct Pi4ioe5v6408 {
    regs: [u8; 10],
    external: [Option<bool>; 8],
    interrupt: u8,
    pointer: u8,
    first: bool,
}

impl Default for Pi4ioe5v6408 {
    fn default() -> Self {
        Self { regs: [0xa2, 0, 0, 0xff, 0, 0xff, 0, 0, 0, 0],
               external: [None; 8], interrupt: 0, pointer: 0, first: true }
    }
}

impl Pi4ioe5v6408 {
    pub fn driven(&self) -> u8 { self.regs[1] & !self.regs[3] }
    pub fn driven_high(&self) -> u8 { self.driven() & self.regs[2] }

    pub fn levels(&self) -> u8 {
        let pulls = self.regs[5] & self.regs[6];
        self.external.iter().enumerate().fold(0, |bits, (pin, external)| {
            let bit = 1 << pin;
            let level = if self.driven() & bit != 0 { self.driven_high() & bit != 0 }
                else { external.unwrap_or(pulls & bit != 0) };
            bits | (u8::from(level) << pin)
        })
    }

    pub fn set_input(&mut self, pin: usize, level: Option<bool>) {
        let old = self.levels();
        if let Some(input) = self.external.get_mut(pin) {
            *input = level;
            self.latch_changes(old);
        }
    }

    pub fn irq(&self) -> bool { self.regs[0] & 2 != 0 || self.interrupt & !self.regs[8] & !self.regs[1] != 0 }

    pub fn service_state(&self) -> crate::json::Json {
        use crate::json::Json;
        Json::object([
            ("direction", Json::Int(self.regs[1].into())),
            ("output-bits", Json::Int(self.regs[2].into())),
            ("input-bits", Json::Int(self.levels().into())),
            ("high-impedance", Json::Int(self.regs[3].into())),
            ("irq", Json::Bool(self.irq())),
        ])
    }

    fn latch_changes(&mut self, old: u8) {
        let levels = self.levels();
        self.interrupt |= (old ^ levels) & (levels ^ self.regs[4]) & !self.regs[1];
    }
}

impl I2cDevice for Pi4ioe5v6408 {
    fn start(&mut self, read: bool) -> bool {
        if !read { self.first = true; }
        true
    }
    fn write(&mut self, byte: u8) -> bool {
        if self.first {
            self.pointer = byte;
            self.first = false;
        } else {
            match self.pointer {
                1 if byte & 1 != 0 => *self = Self { external: self.external, ..Self::default() },
                3 | 5 | 7 | 9 | 11 | 13 | 17 => {
                    let old = self.levels();
                    self.regs[usize::from(self.pointer / 2)] = byte;
                    self.interrupt &= !self.regs[1];
                    self.latch_changes(old);
                }
                _ => {}
            }
        }
        true
    }
    fn read(&mut self) -> u8 {
        match self.pointer {
            1 => { let id = self.regs[0]; self.regs[0] &= !2; id }
            0x0f => self.levels(),
            0x13 => std::mem::take(&mut self.interrupt),
            r if r & 1 != 0 && r <= 0x11 => self.regs[usize::from(r / 2)],
            _ => 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn drive_tristate_and_read_to_clear_irq() {
        let mut chip = Pi4ioe5v6408::default();
        assert!(chip.start(false) && chip.write(1) && chip.start(true));
        assert_eq!(chip.read(), 0xa2);
        assert!(!chip.irq());
        let write = |chip: &mut Pi4ioe5v6408, reg, byte| {
            assert!(chip.start(false) && chip.write(reg) && chip.write(byte));
        };
        write(&mut chip, 3, 1);
        write(&mut chip, 5, 1);
        assert_eq!(chip.driven_high(), 0);
        write(&mut chip, 7, 0xfe);
        assert_eq!(chip.levels(), 1);
        chip.set_input(2, Some(true));
        assert!(chip.irq());
        assert!(chip.start(false) && chip.write(0x13) && chip.start(true));
        assert_eq!(chip.read(), 4);
        assert!(!chip.irq());
        chip.set_input(2, Some(true));
        assert!(!chip.irq());
        chip.set_input(2, Some(false));
        write(&mut chip, 0x11, 4);
        chip.set_input(2, Some(true));
        assert!(!chip.irq());
        assert!(chip.start(false) && chip.write(0x13) && chip.start(true));
        assert_eq!(chip.read(), 4);
    }
}
