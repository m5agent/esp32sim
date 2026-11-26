//! AW88298 I2C control port and digital speaker attenuation.
use esp_periph::i2c::I2cDevice;

// AW88298 datasheet v1.6 pp. 27–35: address byte, then 16-bit words MSB first.
pub struct Aw88298 {
    regs: [u16; 256],
    released: bool,
    ptr: u8,
    first: bool,
    high: Option<u8>,
    read_low: bool,
}

impl Default for Aw88298 { fn default() -> Self { Self::new() } }

impl Aw88298 {
    pub const ADDRESS: u8 = 0x36;
    pub fn new() -> Self {
        let mut d = Self { regs: [0; 256], released: false, ptr: 0, first: true, high: None, read_low: false };
        d.reset();
        d
    }
    fn reset(&mut self) {
        self.regs.fill(0);
        self.regs[0] = 0x1852;
        self.regs[4] = 0x4003; // I2S disabled, AMPPD and PWDN set
        self.regs[5] = 0x0038; // HMUTE and HDCCE set
        self.regs[6] = 0x14c0; // left channel, RX enabled
        self.regs[0x0c] = 0x0064;
        self.ptr = 0;
        self.first = true;
        self.high = None;
        self.read_low = false;
    }
    pub fn set_reset_released(&mut self, released: bool) {
        if self.released && !released { self.reset(); }
        self.released = released;
    }
    /// Linear PCM gain. HAGCCFG4 stores attenuation in the upper byte, with 6 dB coarse
    /// and 0.5 dB fine steps. The lower byte is the AGC hold time.
    pub fn gain(&self) -> f32 {
        if !self.released || self.regs[4] & 0x43 != 0x40 || self.regs[5] & 0x10 != 0 || self.regs[6] & 0x1000 == 0 { return 0.0; }
        let vol = self.regs[0x0c] >> 8;
        let db = 6.0 * f32::from(vol >> 4) + 0.5 * f32::from(vol & 15);
        10.0f32.powf(-db.min(96.0) / 20.0)
    }
}

impl I2cDevice for Aw88298 {
    fn start(&mut self, read: bool) -> bool {
        self.high = None;
        self.read_low = false;
        self.first = !read;
        self.released
    }
    fn write(&mut self, byte: u8) -> bool {
        if self.first { self.ptr = byte; self.first = false; return true; }
        let Some(high) = self.high.take() else { self.high = Some(byte); return true; };
        let word = u16::from_be_bytes([high, byte]);
        match self.ptr {
            0 if word == 0x55aa => { self.reset(); return true; }
            0..=2 | 0x10 | 0x12..=0x14 => {}
            reg => self.regs[reg as usize] = word,
        }
        self.ptr = self.ptr.wrapping_add(1);
        true
    }
    fn read(&mut self) -> u8 {
        let [high, low] = self.regs[self.ptr as usize].to_be_bytes();
        self.read_low = !self.read_low;
        if self.read_low { high } else { self.ptr = self.ptr.wrapping_add(1); low }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn write(d: &mut Aw88298, reg: u8, word: u16) {
        assert!(d.start(false));
        assert!(d.write(reg));
        for b in word.to_be_bytes() { assert!(d.write(b)); }
    }
    fn read(d: &mut Aw88298, reg: u8) -> u16 {
        assert!(d.start(false) && d.write(reg) && d.start(true));
        u16::from_be_bytes([d.read(), d.read()])
    }
    #[test]
    fn reset_endianness_power_mute_and_volume() {
        let mut d = Aw88298::new();
        assert!(!d.start(false));
        d.set_reset_released(true);
        assert_eq!(read(&mut d, 0), 0x1852);
        write(&mut d, 0, 0xabcd);
        assert_eq!(read(&mut d, 0), 0x1852);
        assert!(d.start(false) && d.write(4) && d.write(0x12));
        assert_eq!(read(&mut d, 4), 0x4003, "incomplete word");
        write(&mut d, 5, 8);
        for word in [0x4000, 0x4041, 0x4042] {
            write(&mut d, 4, word);
            assert_eq!(d.gain(), 0.0);
        }
        write(&mut d, 4, 0x4040);
        assert_eq!(d.gain(), 1.0);
        write(&mut d, 0x0c, 0x1064);
        assert!((d.gain() - 0.501187).abs() < 1e-6);
        write(&mut d, 5, 0x18);
        assert_eq!(d.gain(), 0.0);
        write(&mut d, 0, 0x55aa);
        assert_eq!(read(&mut d, 4), 0x4003);
        write(&mut d, 4, 0x4040);
        d.set_reset_released(false);
        assert_eq!(d.gain(), 0.0);
        d.set_reset_released(true);
        assert_eq!(read(&mut d, 4), 0x4003);
    }
}
