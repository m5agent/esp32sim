use esp_periph::i2c::I2cDevice;

/// NXP LM75B Rev. 7, §§7.4–7.8. Signed 11-bit temperature and 9-bit limits.
pub struct Lm75b {
    config: u8,
    temperature: i16,
    hysteresis: i16,
    threshold: i16,
    ptr: u8,
    phase: u8,
    os: bool,
    rising: bool,
}

impl Default for Lm75b {
    fn default() -> Self {
        Self { config: 0, temperature: 25 * 256, hysteresis: 75 * 256, threshold: 80 * 256,
               ptr: 0, phase: 0, os: false, rising: true }
    }
}

impl Lm75b {
    pub const ADDRESS: u8 = 0x48;

    pub fn set_millicelsius(&mut self, value: i32) {
        if self.config & 1 != 0 { return; }
        self.temperature = ((value.clamp(-55_000, 125_000) * 2 / 125) * 32) as i16;
        if self.rising && self.temperature >= self.threshold {
            self.os = true;
            self.rising = false;
        } else if !self.rising && self.temperature < self.hysteresis {
            self.os = self.config & 2 != 0;
            self.rising = true;
        }
    }

    pub fn os_low(&self) -> bool { self.os ^ (self.config & 4 != 0) }
}

impl I2cDevice for Lm75b {
    fn start(&mut self, _read: bool) -> bool { self.phase = 0; true }
    fn write(&mut self, byte: u8) -> bool {
        match self.phase {
            0 => self.ptr = byte & 3,
            1 if self.ptr == 1 => {
                self.config = byte & 0x1f;
                if self.config & 3 == 3 { self.os = false; }
            }
            1 | 2 if self.ptr >= 2 => {
                let reg = if self.ptr == 2 { &mut self.hysteresis } else { &mut self.threshold };
                let mut bytes = reg.to_be_bytes();
                bytes[usize::from(self.phase - 1)] = byte;
                *reg = i16::from_be_bytes(bytes) & !127;
            }
            _ => {}
        }
        self.phase = self.phase.saturating_add(1);
        true
    }
    fn read(&mut self) -> u8 {
        let value = match self.ptr {
            0 => self.temperature,
            1 => i16::from(self.config) << 8,
            2 => self.hysteresis,
            _ => self.threshold,
        };
        let byte = value.to_be_bytes()[usize::from(self.phase & 1)];
        self.phase ^= 1;
        if self.config & 2 != 0 { self.os = false; }
        byte
    }
}
