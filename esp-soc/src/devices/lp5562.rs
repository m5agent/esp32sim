//! TI LP5562 direct PWM and register readback (SNVS820B, §7.6).
//! Autonomous pattern-engine execution is outside the direct-control subset.
use esp_periph::i2c::I2cDevice;

pub struct Lp5562 {
    regs: [u8; 0x71],
    pointer: u8,
    first: bool,
    pub version: u64,
}

impl Default for Lp5562 {
    fn default() -> Self {
        let mut regs = [0; 0x71];
        regs[5..8].fill(0xaf);
        regs[0x0f] = 0xaf;
        regs[0x70] = 0x39;
        Self { regs, pointer: 0, first: true, version: 0 }
    }
}

impl Lp5562 {
    pub const ADDRESS: u8 = 0x30;

    pub fn rgb_pwm(&self) -> [u8; 3] { [self.regs[4], self.regs[3], self.regs[2]] }

    pub fn outputs(&self) -> [u8; 4] {
        let channel = |pwm, current, shift: u32| {
            if self.regs[0] & 0x40 != 0 && (self.regs[0x70] >> shift) & 3 == 0 && self.regs[current] != 0 {
                self.regs[pwm]
            } else { 0 }
        };
        [channel(4, 7, 4), channel(3, 6, 2), channel(2, 5, 0), channel(0x0e, 0x0f, 6)]
    }
}

impl I2cDevice for Lp5562 {
    fn start(&mut self, read: bool) -> bool { self.first = !read; true }
    fn write(&mut self, byte: u8) -> bool {
        if self.first { self.pointer = byte; self.first = false; return true; }
        match self.pointer {
            0x0d if byte == 0xff => { self.regs = Self::default().regs; self.version += 1; }
            0x00..=0x08 | 0x0e..=0x70 => {
                let reg = &mut self.regs[self.pointer as usize];
                if *reg != byte { *reg = byte; self.version += 1; }
            }
            _ => {}
        }
        self.pointer = self.pointer.wrapping_add(1);
        true
    }
    fn read(&mut self) -> u8 {
        let value = self.regs.get(self.pointer as usize).copied().unwrap_or(0);
        self.pointer = self.pointer.wrapping_add(1);
        value
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn w_output_requires_direct_mapping_and_chip_enable() {
        let mut d = Lp5562::default();
        let write = |d: &mut Lp5562, reg, value| { d.start(false); d.write(reg); d.write(value); };
        write(&mut d, 0x0e, 80);
        assert_eq!(d.outputs()[3], 0);
        write(&mut d, 0x00, 0x40);
        assert_eq!(d.outputs()[3], 80);
        write(&mut d, 0x0e, 160);
        assert_eq!(d.outputs()[3], 160);
        d.start(false); d.write(0x0e); d.start(true);
        assert_eq!(d.read(), 160);
        write(&mut d, 0x70, 0x40);
        assert_eq!(d.outputs()[3], 0);
        write(&mut d, 0x0d, 0xff);
        assert_eq!(d.read(), 0);
    }
}
