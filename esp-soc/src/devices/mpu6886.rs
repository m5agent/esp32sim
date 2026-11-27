//! TDK MPU6886 polling subset, register map rev. 1.1. Burst reads latch
//! one sample. Values pass through the configured signed ADC ranges.
use esp_periph::i2c::I2cDevice;

pub struct Mpu6886 {
    regs: [u8; 128],
    pointer: u8,
    first: bool,
    sample: [f64; 6],
    latched: [u8; 14],
}

impl Default for Mpu6886 {
    fn default() -> Self {
        let mut regs = [0; 128];
        regs[0x6b] = 0x40;
        regs[0x75] = 0x19;
        Self { regs, pointer: 0, first: true, sample: [0., 0., 1., 0., 0., 0.], latched: [0; 14] }
    }
}

impl Mpu6886 {
    pub const ADDRESS: u8 = 0x68;

    pub fn set_sample(&mut self, sample: [f64; 6]) { self.sample = sample; }

    fn latch(&mut self) {
        let accel_range = 2u16 << ((self.regs[0x1c] >> 3) & 3);
        let gyro_range = 250u16 << ((self.regs[0x1b] >> 3) & 3);
        for (axis, value) in self.sample.into_iter().enumerate() {
            let (at, range) = if axis < 3 { (axis * 2, accel_range) } else { (axis * 2 + 2, gyro_range) };
            let raw = (value * 32768. / f64::from(range)).round() as i16;
            self.latched[at..at + 2].copy_from_slice(&raw.to_be_bytes());
        }
    }
}

impl I2cDevice for Mpu6886 {
    fn start(&mut self, read: bool) -> bool {
        self.first = !read;
        if read { self.latch(); }
        true
    }
    fn write(&mut self, byte: u8) -> bool {
        if self.first { self.pointer = byte & 0x7f; self.first = false; return true; }
        match self.pointer {
            0x6b if byte & 0x80 != 0 => { self.regs = Self::default().regs; }
            0x19..=0x1e | 0x23 | 0x37..=0x38 | 0x69..=0x6c => self.regs[self.pointer as usize] = byte,
            _ => {}
        }
        self.pointer = (self.pointer + 1) & 0x7f;
        true
    }
    fn read(&mut self) -> u8 {
        let v = match self.pointer {
            0x3a => u8::from(self.regs[0x6b] & 0x40 == 0),
            0x3b..=0x48 => self.latched[(self.pointer - 0x3b) as usize],
            reg => self.regs[reg as usize],
        };
        self.pointer = (self.pointer + 1) & 0x7f;
        v
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn range_and_signed_burst_follow_the_guest_configuration() {
        let mut d = Mpu6886::default();
        d.set_sample([1., -0.5, 12., 0., 0., 0.]);
        d.start(false); d.write(0x1c); d.write(0x10);
        d.start(false); d.write(0x3b); d.start(true);
        let raw: Vec<_> = (0..6).map(|_| d.read()).collect();
        assert_eq!(raw, [0x10, 0, 0xf8, 0, 0x7f, 0xff]);
        d.start(false); d.write(0x75); d.start(true);
        assert_eq!(d.read(), 0x19);
    }
}
