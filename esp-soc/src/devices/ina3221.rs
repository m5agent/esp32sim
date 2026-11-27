//! TI INA3221 three-channel shunt and bus monitor, SBOS576B §8.6.
//! Host samples supply each channel's bus millivolts and differential microvolts.
use esp_periph::i2c::I2cDevice;

pub const ADDRESS: u8 = 0x40;

pub struct Ina3221 {
    regs: [u16; 16],
    samples: [(u16, i32); 3],
    pointer: u8,
    select: bool,
    high: Option<u8>,
    low: Option<u8>,
}

impl Default for Ina3221 {
    fn default() -> Self {
        let mut regs = [0; 16];
        regs[0] = 0x7127;
        Self { regs, samples: [(0, 0); 3], pointer: 0, select: true, high: None, low: None }
    }
}

impl Ina3221 {
    pub fn set_sample(&mut self, channel: usize, bus_mv: u16, shunt_uv: i32) -> bool {
        let Some(sample) = self.samples.get_mut(channel) else { return false; };
        *sample = (bus_mv, shunt_uv);
        true
    }

    fn read_word(&self) -> u16 {
        match self.pointer {
            0xfe => 0x5449,
            0xff => 0x3220,
            register @ 1..=6 => {
                let channel = usize::from((register - 1) / 2);
                let config = self.regs[0];
                if config & (1 << (14 - channel)) == 0 { return 0; }
                let (bus_mv, shunt_uv) = self.samples[channel];
                if register & 1 == 0 && config & 2 != 0 { bus_mv.min(32760) & !7 }
                else if register & 1 != 0 && config & 1 != 0 {
                    (((shunt_uv / 40).clamp(-4096, 4095) as i16) << 3) as u16
                } else { 0 }
            }
            reg => self.regs.get(usize::from(reg)).copied().unwrap_or(0),
        }
    }
}

impl I2cDevice for Ina3221 {
    fn start(&mut self, read: bool) -> bool {
        self.select = !read;
        self.high = None;
        self.low = None;
        true
    }

    fn write(&mut self, byte: u8) -> bool {
        if self.select {
            self.pointer = byte;
            self.select = false;
        } else if let Some(high) = self.high.take() {
            let word = u16::from_be_bytes([high, byte]);
            match self.pointer {
                0 if word & 0x8000 != 0 => self.regs = Self::default().regs,
                reg @ (0 | 7..=0x0f) => self.regs[usize::from(reg)] = word,
                _ => {}
            }
        } else {
            self.high = Some(byte);
        }
        true
    }

    fn read(&mut self) -> u8 {
        if let Some(low) = self.low.take() { return low; }
        let [high, low] = self.read_word().to_be_bytes();
        self.low = Some(low);
        high
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn read_at(dev: &mut Ina3221, reg: u8) -> u16 {
        dev.start(false); dev.write(reg); dev.start(true);
        u16::from_be_bytes([dev.read(), dev.read()])
    }

    #[test]
    fn channels_encode_independent_signed_shunts_and_bus_values() {
        let mut dev = Ina3221::default();
        assert_eq!(read_at(&mut dev, 0xff), 0x3220);
        dev.set_sample(0, 3921, -1000);
        dev.set_sample(1, 4900, 2000);
        assert_eq!(read_at(&mut dev, 1) as i16, -200);
        assert_eq!(read_at(&mut dev, 2), 3920);
        assert_eq!(read_at(&mut dev, 3), 400);
        assert_eq!(read_at(&mut dev, 4), 4896);
        dev.start(false); dev.write(0); dev.write(0x20); dev.write(7);
        assert_eq!(read_at(&mut dev, 1), 0);
        assert_eq!(read_at(&mut dev, 3), 400);
    }
}
