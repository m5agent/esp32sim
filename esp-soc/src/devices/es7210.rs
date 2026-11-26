//! ES7210 I2C control port and ADC1/2 enable state for the CoreS3 stereo microphones.
use esp_periph::i2c::I2cDevice;

// Everest ES7210 rev. 21.0 §7, https://files.waveshare.com/wiki/common/ES7210_DS.pdf.
// Digital PCM is supplied at the ADC output level; analog gain, HPF and AEC are outside this model.
const DEFAULTS: &[(u8, u8)] = &[
    (0x00, 0x32), (0x01, 0x20), (0x02, 0x02), (0x03, 0x04), (0x04, 0x01),
    (0x07, 0x20), (0x08, 0x10), (0x09, 0x40), (0x0a, 0x40), (0x0d, 0x09),
    (0x18, 0xf7), (0x19, 0xf7), (0x1b, 0xbf), (0x1c, 0xbf), (0x1d, 0xbf), (0x1e, 0xbf),
    (0x20, 0x26), (0x21, 0x26), (0x22, 0x06), (0x23, 0x26),
    (0x3d, 0x72), (0x3e, 0x10), (0x40, 0x80), (0x41, 0x71), (0x42, 0x71),
    (0x4b, 0xff), (0x4c, 0xff),
];
pub struct Es7210 { regs: [u8; 256], ptr: u8, first: bool }
impl Default for Es7210 { fn default() -> Self { Self::new() } }

impl Es7210 {
    pub const ADDRESS: u8 = 0x40;
    pub fn new() -> Self {
        let mut d = Self { regs: [0; 256], ptr: 0, first: true };
        d.reset_registers();
        d
    }
    fn reset_registers(&mut self) {
        self.regs.fill(0);
        for &(reg, val) in DEFAULTS { self.regs[reg as usize] = val; }
    }
    pub fn lanes(&self) -> [bool; 2] {
        let r = &self.regs;
        // M5Unified 0.3.0 _microphone_enabled_cb_cores3 selects ADC1/2, slave clocks,
        // 16-bit Philips I2S, and leaves ADC3/4 powered down. Only that wire format is modeled.
        if r[0] & 0x17 != 1 || r[1] & 0x4b != 0 || r[0x40] & 0x80 != 0
            || r[8] & 1 != 0 || r[0x11] != 0x60 || r[0x12] & 3 != 0 || r[0x4b] & 0xc0 != 0 {
            return [false; 2];
        }
        [r[0x43] & 0x10 != 0 && r[0x4b] & 0x15 == 0, r[0x44] & 0x10 != 0 && r[0x4b] & 0x2a == 0]
    }
}

impl I2cDevice for Es7210 {
    fn start(&mut self, read: bool) -> bool { if !read { self.first = true; } true }
    fn write(&mut self, b: u8) -> bool {
        if self.first { self.ptr = b; self.first = false; return true; }
        match self.ptr {
            0 => {
                if b & 4 != 0 { self.reset_registers(); }
                self.regs[0] = b;
            }
            0x3d..=0x3f => {}
            reg if self.regs[0] & 4 == 0 => self.regs[reg as usize] = b,
            _ => {}
        }
        self.ptr = self.ptr.wrapping_add(1);
        true
    }
    fn read(&mut self) -> u8 {
        let b = self.regs[self.ptr as usize];
        self.ptr = self.ptr.wrapping_add(1);
        b
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn write(d: &mut Es7210, reg: u8, b: u8) { assert!(d.start(false) && d.write(reg) && d.write(b)); }
    #[test]
    fn identity_reset_clocks_and_adc_power() {
        let mut d = Es7210::new();
        assert_eq!(d.lanes(), [false; 2]);
        write(&mut d, 0x3e, 6);
        assert!(d.start(false) && d.write(0x3d) && d.start(true));
        assert_eq!([d.read(), d.read()], [0x72, 0x10]);
        for (r, v) in [(0, 0xff), (0, 0x41), (1, 0x1f), (0x11, 0x60), (0x40, 0x42), (0x43, 0x1b), (0x44, 0x1b), (0x4b, 0)] {
            write(&mut d, r, v);
        }
        assert_eq!(d.lanes(), [false; 2]);
        write(&mut d, 1, 0x14);
        assert_eq!(d.lanes(), [true; 2]);
        write(&mut d, 0x4b, 8);
        assert_eq!(d.lanes(), [true, false]);
        write(&mut d, 0, 0xff);
        assert_eq!(d.lanes(), [false; 2]);
    }
}
