//! Everest ES8311 I2C register file and PCM power, clock and mute controls.
use esp_periph::i2c::I2cDevice;
use std::{cell::RefCell, rc::Rc};
use crate::board::I2sCodecs;

// ES8311 datasheet rev 8.0 (May 2020), sections 5 and 8. The board keeps one
// control port per bus, with a shared register file and independent pointers.
pub struct Es8311 {
    regs: Rc<RefCell<[u8; 256]>>,
    ptr: u8,
    first: bool,
}

impl Default for Es8311 { fn default() -> Self { Self::new() } }

impl Es8311 {
    pub const ADDRESS: u8 = 0x18;
    pub fn new() -> Self { Self { regs: Rc::new(RefCell::new(Self::reset_regs())), ptr: 0, first: true } }
    pub fn i2c_port(&self) -> Self { Self { regs: self.regs.clone(), ptr: 0, first: true } }

    fn reset_regs() -> [u8; 256] {
        let mut regs = [0; 256];
        for (reg, value) in [(0x00, 0x1f), (0x03, 0x10), (0x04, 0x10), (0x06, 3), (0x08, 0xff), (0x0c, 0x20),
            (0x0d, 0xfc), (0x0e, 0x6a), (0x10, 0x13), (0x11, 0x7c), (0x12, 2), (0x13, 0x40),
            (0x14, 0x10), (0x16, 4), (0x1b, 0x0c), (0x1c, 0x4c), (0x37, 8), (0xfd, 0x83), (0xfe, 0x11)] {
            regs[reg] = value;
        }
        regs
    }

    /// The codec is a serial slave driven from BCLK on ADV. Test block clocks
    /// separately so enabling one direction need not depend on the other.
    fn clocked(r: &[u8; 256], clock_mask: u8, reset_mask: u8) -> bool {
        r[0xfa] & 1 == 0 && r[0x00] & (0xd8 | reset_mask) == 0x80
            && r[0x01] & clock_mask == clock_mask && r[0x0d] & 0xe0 == 0 && r[0x0d] & 3 != 0
    }

    /// Board PCM boundary for BCLK-derived MCLK. Port is the TX controller,
    /// mic_port is the firmware's preferred RX controller for an unqualified WAV.
    pub fn i2s_codecs(&self, port: usize, mic_port: usize, bck: u8, ws: u8, dout: u8, din: u8) -> I2sCodecs {
        let r = self.regs.borrow();
        let dac = Self::clocked(&r, 0xb5, 1) && r[0x0d] & 0x08 == 0
            && r[0x09] & 0x43 == 0 && r[0x12] & 2 == 0 && r[0x13] & 0x10 != 0
            && r[0x31] & 0x60 == 0 && r[0x44] & 0x80 == 0;
        let adc = Self::clocked(&r, 0xba, 2) && r[0x0d] & 0x10 == 0
            && r[0x07] & 0x10 == 0 && r[0x0a] & 0x43 == 0 && r[0x0e] & 0x70 == 0 && r[0x14] & 0x50 == 0x10;
        let speaker_gain = if !dac { 0.0 } else if r[0x32] == 0xbf { 1.0 }
            else { 10.0f32.powf((f32::from(r[0x32]) - 191.0) / 40.0) };
        let output = (r[0x44] >> 4) & 7;
        let mic_lanes = [adc && matches!(output, 0 | 1), adc && matches!(output, 0 | 2 | 4)];
        I2sCodecs { port, mic_port, bck, ws, dout, din, mck: None, speaker_gain, mic_lanes }
    }
}

impl I2cDevice for Es8311 {
    fn start(&mut self, read: bool) -> bool { self.first = !read; true }
    fn write(&mut self, byte: u8) -> bool {
        if self.first { self.first = false; self.ptr = byte; return true; }
        let mut r = self.regs.borrow_mut();
        match self.ptr {
            0xfa => {
                if byte & 1 != 0 { *r = Self::reset_regs(); }
                r[0xfa] = byte & 3;
            }
            0x00..=0x45 if r[0xfa] & 1 == 0 => r[self.ptr as usize] = byte,
            _ => {} // Status/ID and reserved registers are not writable.
        }
        self.ptr = self.ptr.wrapping_add(1);
        true
    }
    fn read(&mut self) -> u8 {
        let byte = self.regs.borrow()[self.ptr as usize];
        self.ptr = self.ptr.wrapping_add(1);
        byte
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    // M5Unified 0.3.0 M5Unified.inl Cardputer-ADV speaker and microphone callbacks.
    const SPEAKER: &[(u8, u8)] = &[(0, 0x80), (1, 0xb5), (2, 0x18), (0x0d, 1), (0x12, 0), (0x13, 0x10), (0x32, 0xbf), (0x37, 8)];
    const MIC: &[(u8, u8)] = &[(0, 0x80), (1, 0xba), (2, 0x18), (0x0d, 1), (0x0e, 2), (0x14, 0x10), (0x17, 0xbf), (0x1c, 0x6a)];
    const MIC_OFF: &[(u8, u8)] = &[(0x0d, 0xfc), (0x0e, 0x6a), (0, 0)];

    fn write(d: &mut Es8311, pairs: &[(u8, u8)]) {
        for &(reg, val) in pairs { assert!(d.start(false) && d.write(reg) && d.write(val)); d.stop(); }
    }
    fn gates(d: &Es8311) -> (f32, [bool; 2]) {
        let c = d.i2s_codecs(1, 0, 41, 43, 42, 46);
        (c.speaker_gain, c.mic_lanes)
    }

    #[test]
    fn control_ports_share_registers_and_id_is_read_only() {
        let d = Es8311::new();
        let mut a = d.i2c_port();
        let mut b = d.i2c_port();
        write(&mut a, &[(0xfd, 0xff), (0xfe, 0xff), (0x32, 0xbf)]);
        assert!(a.start(false) && a.write(0xfd) && a.start(true));
        assert!(b.start(false) && b.write(0x32) && b.start(true));
        assert_eq!(a.read(), 0x83);
        assert_eq!(b.read(), 0xbf);
        assert_eq!(a.read(), 0x11);
        write(&mut a, &[(0, 0x1f)]);
        assert!(a.start(false) && a.write(0x32) && a.start(true));
        assert_eq!(a.read(), 0xbf, "digital reset leaves the control port intact");
        write(&mut a, &[(0xfa, 1), (0x32, 0xbf), (0xfa, 0)]);
        assert!(b.start(false) && b.write(0x32) && b.start(true));
        assert_eq!(b.read(), 0);
    }

    #[test]
    fn m5unified_callbacks_gate_each_pcm_direction() {
        let d = Es8311::new();
        let mut wire = d.i2c_port();
        assert_eq!(gates(&d), (0.0, [false; 2]));
        write(&mut wire, SPEAKER);
        assert_eq!(gates(&d), (1.0, [false; 2]));
        write(&mut wire, MIC);
        assert_eq!(gates(&d), (0.0, [true; 2]));
        write(&mut wire, &[(1, 0xbf)]); // Both analog/digital block clocks enabled.
        assert_eq!(gates(&d), (1.0, [true; 2]));
        write(&mut wire, &[(0x32, 0xb3), (0x44, 0x20)]);
        let (gain, lanes) = gates(&d);
        assert!((gain - 0.501187).abs() < 1e-6);
        assert_eq!(lanes, [false, true]);
        write(&mut wire, &[(0x31, 0x40)]);
        assert_eq!(gates(&d), (0.0, [false, true]));
        write(&mut wire, &[ (0x31, 0), (0, 0x82) ]);
        assert!((gates(&d).0 - 0.501187).abs() < 1e-6);
        assert_eq!(gates(&d).1, [false; 2]);
        write(&mut wire, MIC_OFF);
        assert_eq!(gates(&d), (0.0, [false; 2]));
        write(&mut wire, SPEAKER);
        assert_eq!(gates(&d), (1.0, [false; 2]));
    }
}
