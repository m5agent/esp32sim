//! OV3660 SCCB register addressing and DVP output. Geometry, format and
//! enable bits follow esp32-camera v2.1.4 ov3660_regs.h and ov3660.c.
use std::{collections::BTreeMap, sync::Arc};
use esp_periph::i2c::I2cDevice;
use crate::picture::{Jpeg, JpegError, Picture, PixelFormat};
use super::HostFrameSource;

#[derive(Default)]
pub struct Ov3660 {
    regs: BTreeMap<u16, u8>,
    address: u16,
    phase: u8,
    source: HostFrameSource,
}

impl Ov3660 {
    pub const ADDRESS: u8 = 0x3c;

    fn reg(&self, address: u16) -> u8 {
        match address { 0x300a => 0x36, 0x300b => 0x60, _ => self.regs.get(&address).copied().unwrap_or(0) }
    }
    fn word(&self, address: u16) -> u32 { u16::from_be_bytes([self.reg(address), self.reg(address + 1)]) as u32 }
    fn size(&self) -> (u32, u32) { (self.word(0x3808), self.word(0x380a)) }
    pub fn set_camera_picture(&mut self, p: Picture) -> Result<(), &'static str> { self.source.set_camera_picture(p) }
    pub fn set_camera_jpeg(&mut self, jpeg: Jpeg) -> Result<(), JpegError> { self.source.set_camera_jpeg(jpeg, self.size()) }
    pub fn camera_preview(&self, w: u32, h: u32) -> Option<Vec<u8>> { self.source.camera_preview(w, h) }
    pub fn jpeg_enabled(&self) -> bool { self.reg(0x3821) & 0x20 != 0 }

    pub fn camera_frame(&mut self) -> Option<(u32, u32, Arc<Vec<u8>>)> {
        let (w, h) = self.size();
        if w == 0 || h == 0 || w > 2048 || h > 1536 || self.reg(0x3008) & 0x40 != 0 { return None; }
        if self.jpeg_enabled() { return self.source.jpeg_frame(w, h); }
        let format = match self.reg(0x4300) {
            0x61 => PixelFormat::Rgb565,
            0x30 if w.is_multiple_of(2) => PixelFormat::Yuyv,
            0x10 => PixelFormat::Grayscale,
            _ => return None,
        };
        let window = (0x3800..=0x3807).all(|r| self.regs.contains_key(&r)).then(|| {
            ([0x3800, 0x3802, 0x3804, 0x3806].map(|r| self.word(r)), 2080, 1548)
        });
        let flip = [self.reg(0x3821) & 6 != 0, self.reg(0x3820) & 6 != 0];
        self.source.oriented_frame(w, h, format, window, flip)
    }
}

impl I2cDevice for Ov3660 {
    fn start(&mut self, read: bool) -> bool { if !read { self.phase = 0; } true }
    fn write(&mut self, byte: u8) -> bool {
        match self.phase {
            0 => { self.address = u16::from(byte) << 8; self.phase = 1; }
            1 => { self.address |= u16::from(byte); self.phase = 2; }
            _ => {
                match self.address {
                    0x3008 if byte & 0x80 != 0 => { self.regs.clear(); self.regs.insert(0x3008, byte & 0x7f); }
                    0x300a | 0x300b => {}
                    _ => { self.regs.insert(self.address, byte); }
                }
                self.address = self.address.wrapping_add(1);
            }
        }
        true
    }
    fn read(&mut self) -> u8 {
        let value = self.reg(self.address);
        self.address = self.address.wrapping_add(1);
        value
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sccb_big_endian_address_and_geometry_follow_programmed_registers() {
        let mut d = Ov3660::default();
        let write = |d: &mut Ov3660, addr: u16, bytes: &[u8]| {
            d.start(false);
            for b in addr.to_be_bytes().iter().chain(bytes) { d.write(*b); }
        };
        write(&mut d, 0x300a, &[0x00, 0x00]);
        write(&mut d, 0x300a, &[]);
        d.start(true);
        assert_eq!([d.read(), d.read()], [0x36, 0x60]);
        write(&mut d, 0x3808, &[0, 4, 0, 2]);
        write(&mut d, 0x4300, &[0x61]);
        d.set_camera_picture(Picture { w: 1, h: 1, rgb: vec![255, 0, 0] }).unwrap();
        let (w, h, data) = d.camera_frame().unwrap();
        assert_eq!((w, h, data.len()), (4, 2, 16));
        write(&mut d, 0x3008, &[0x42]);
        assert!(d.camera_frame().is_none());
    }
}
