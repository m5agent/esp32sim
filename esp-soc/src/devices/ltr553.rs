//! Lite-On LTR-553ALS-01 I2C ambient light and proximity sensor (BNS-OD-FC002/A4 register map).
//!
//! Scripted conversions supply ADC counts directly. Threshold/rate settings read back, while
//! conversion timing and the interrupt output are outside this polling model.
use esp_periph::i2c::I2cDevice;

pub const LTR553_ADDRESS: u8 = 0x23;

pub struct Ltr553 {
    regs: [u8; 256],
    ptr: u8,
    first: bool,
    sample: [u16; 3],
}

impl Default for Ltr553 { fn default() -> Self { Self::new() } }

impl Ltr553 {
    pub fn new() -> Self {
        let mut d = Ltr553 { regs: [0; 256], ptr: 0, first: true, sample: [0; 3] };
        d.reset();
        d
    }
    fn reset(&mut self) {
        self.regs = [0; 256];
        self.regs[0x82..0x88].copy_from_slice(&[0x7f, 1, 2, 3, 0x92, 0x05]);
        self.regs[0x90..0x92].copy_from_slice(&[0xff, 0x07]);
        self.regs[0x97..0x99].fill(0xff);
    }
    pub fn set_sample(&mut self, ch0: u16, ch1: u16, ps: u16) {
        self.sample = [ch0, ch1, ps];
        self.mark_ready();
    }
    pub fn service_state(&self) -> crate::json::Json {
        use crate::json::Json;
        let [ch0, ch1, ps] = self.sample;
        Json::object([
            ("light-level", Json::Int((i64::from(ch0) + i64::from(ch1)) / 2)),
            ("proximity-level", Json::Int(ps.into())),
        ])
    }

    fn als_active(&self) -> bool { self.regs[0x80] & 1 != 0 }
    fn ps_active(&self) -> bool {
        // The CoreS3 docs sketch's LTR5XX.cpp setPsMode writes 0b10 for active.
        // Accept it alongside the datasheet's 0b11.
        self.regs[0x81] & 2 != 0
    }
    fn mark_ready(&mut self) {
        self.regs[0x8c] = if self.als_active() { (self.regs[0x80] & 0x1c) << 2 | 4 } else { self.regs[0x8c] & 0x70 };
        if self.ps_active() { self.regs[0x8c] |= 1; }
    }
    fn latch(&mut self) {
        let [ch0, ch1, ps] = self.sample;
        if self.als_active() {
            self.regs[0x88..0x8a].copy_from_slice(&ch1.to_le_bytes());
            self.regs[0x8a..0x8c].copy_from_slice(&ch0.to_le_bytes());
        }
        if self.ps_active() {
            self.regs[0x8d..0x8f].copy_from_slice(&ps.min(0x7ff).to_le_bytes());
            if ps > 0x7ff && self.regs[0x81] & 0x20 != 0 { self.regs[0x8e] |= 0x80; }
        }
    }
}

impl I2cDevice for Ltr553 {
    fn start(&mut self, read: bool) -> bool {
        if read { self.latch(); } else { self.first = true; }
        true
    }
    fn write(&mut self, b: u8) -> bool {
        if self.first { self.ptr = b; self.first = false; return true; }
        match self.ptr {
            0x80 if b & 2 != 0 => self.reset(),
            0x80 => { self.regs[0x80] = b & 0x1d; self.mark_ready(); }
            0x81 => { self.regs[0x81] = b & 0x23; self.mark_ready(); }
            r @ (0x82..=0x85 | 0x8f..=0x95 | 0x97..=0x9a | 0x9e) => self.regs[usize::from(r)] = b,
            _ => {}
        }
        self.ptr = self.ptr.wrapping_add(1);
        true
    }
    fn read(&mut self) -> u8 {
        let r = self.ptr;
        self.ptr = self.ptr.wrapping_add(1);
        match r {
            0x8b => self.regs[0x8c] &= !4,
            0x8e => self.regs[0x8c] &= !1,
            _ => {}
        }
        self.regs[usize::from(r)]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn wr(d: &mut Ltr553, reg: u8, byte: u8) { d.start(false); d.write(reg); d.write(byte); d.stop(); }
    fn rd(d: &mut Ltr553, reg: u8, n: usize) -> Vec<u8> {
        d.start(false); d.write(reg); d.start(true);
        let bytes = (0..n).map(|_| d.read()).collect();
        d.stop(); bytes
    }
    #[test]
    fn mode_controls_channel_order_and_status() {
        let mut d = Ltr553::new();
        d.set_sample(0x1234, 0xabcd, 0x567);
        assert_eq!(rd(&mut d, 0x86, 2), [0x92, 0x05]);
        assert_eq!(rd(&mut d, 0x88, 7), [0; 7]);
        wr(&mut d, 0x80, 0x19); // gain 48x, ALS active
        wr(&mut d, 0x81, 2); // docs driver PS active
        assert_eq!(rd(&mut d, 0x88, 7), [0xcd, 0xab, 0x34, 0x12, 0x61, 0x67, 0x05]);
        assert_eq!(rd(&mut d, 0x8c, 1), [0x60], "reading the high bytes consumed both samples");
        d.set_sample(400, 120, 0xfff);
        wr(&mut d, 0x81, 0x23); // datasheet PS active, saturation indicator enabled
        assert_eq!(rd(&mut d, 0x8d, 2), [0xff, 0x87]);
        assert_eq!(rd(&mut d, 0x88, 4), [120, 0, 144, 1]);
        wr(&mut d, 0x86, 0);
        wr(&mut d, 0x80, 2);
        assert_eq!(rd(&mut d, 0x80, 8), [0, 0, 0x7f, 1, 2, 3, 0x92, 0x05]);
        assert_eq!(rd(&mut d, 0x88, 7), [0; 7]);
    }
}
