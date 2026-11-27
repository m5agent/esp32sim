//! TI TMP105 (SLLS648D), digital temperature and configuration registers.
use esp_periph::i2c::I2cDevice;

pub struct Tmp105 {
    temperature: i16,
    config: u8,
    limits: [i16; 2],
    pointer: u8,
    offset: usize,
}

impl Tmp105 {
    pub const ADDRESS: u8 = 0x48;

    pub fn new(millicelsius: i32) -> Self {
        let temperature = (millicelsius.clamp(-40_000, 125_000) * 16 / 1000) as i16 * 16;
        Self { temperature, config: 0, limits: [75 * 256, 80 * 256], pointer: 0, offset: 0 }
    }
}

impl I2cDevice for Tmp105 {
    fn start(&mut self, _read: bool) -> bool {
        self.offset = 0;
        true
    }

    fn write(&mut self, byte: u8) -> bool {
        match (self.pointer, self.offset) {
            (_, 0) => self.pointer = byte & 3,
            (1, 1) => self.config = byte & 0x7f,
            (2 | 3, 1 | 2) => {
                let limit = &mut self.limits[usize::from(self.pointer - 2)];
                let mut bytes = limit.to_be_bytes();
                bytes[self.offset - 1] = byte;
                *limit = i16::from_be_bytes(bytes) & !15;
            }
            _ => {}
        }
        self.offset = self.offset.saturating_add(1);
        true
    }

    fn read(&mut self) -> u8 {
        let resolution = (self.config >> 5) & 3;
        let value = match self.pointer {
            0 => self.temperature & !((1 << (7 - resolution)) - 1),
            1 => i16::from_be_bytes([self.config, 0]),
            _ => self.limits[usize::from(self.pointer - 2)],
        };
        let byte = value.to_be_bytes()[self.offset & 1];
        self.offset ^= 1;
        byte
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn repeated_start_preserves_pointer_and_resolution_controls_temperature_bytes() {
        let mut dev = Tmp105::new(-6_125);
        dev.start(false);
        dev.write(0);
        dev.start(true);
        assert_eq!([dev.read(), dev.read()], [0xf9, 0x80]);
        dev.start(false);
        dev.write(1);
        dev.write(0xe0);
        dev.start(true);
        assert_eq!(dev.read(), 0x60);
        dev.start(false);
        dev.write(0);
        dev.start(true);
        assert_eq!([dev.read(), dev.read()], [0xf9, 0xe0]);
    }
}
