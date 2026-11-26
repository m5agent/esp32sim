//! GalaxyCore GC0308 SCCB and parallel pixel output. Register fields and reset
//! values follow the GalaxyCore GC0308 datasheet v1.0, section 6. The supported
//! formats and subsampling patterns follow esp32-camera v2.1.4 sensors/gc0308.c.
//! The host picture represents the active VGA array. Resampling approximates
//! the ISP's sampling geometry and does not simulate the analog image pipeline.

use std::sync::Arc;
use esp_periph::i2c::I2cDevice;
use crate::picture::{Picture, PixelFormat};
use super::HostFrameSource;

const WIDTH: u32 = 640;
const HEIGHT: u32 = 480;

pub struct Gc0308 {
    regs: [[u8; 256]; 2],
    page_reg: u8,
    ptr: u8,
    first: bool,
    source: HostFrameSource,
}

impl Default for Gc0308 {
    fn default() -> Self {
        let mut sensor = Self { regs: [[0; 256]; 2], page_reg: 0, ptr: 0, first: true, source: HostFrameSource::default() };
        sensor.reset();
        sensor
    }
}

impl Gc0308 {
    pub const ADDRESS: u8 = 0x21;

    fn reset(&mut self) {
        self.regs = [[0; 256]; 2];
        let [p0, p1] = &mut self.regs;
        p0[0] = 0x9b;
        p0[0x05..=0x0c].copy_from_slice(&[0, 0, 0, 0, 1, 0xe8, 2, 0x88]);
        p0[0x24] = 0xa2;
        p0[0x26] = 0x3f;
        p1[0x53..=0x55].copy_from_slice(&[0x82, 0x03, 0x03]);
        self.page_reg = 0;
        self.source.invalidate();
    }

    pub fn set_camera_picture(&mut self, picture: Picture) -> Result<(), &'static str> { self.source.set_camera_picture(picture) }
    pub fn camera_preview(&self, w: u32, h: u32) -> Option<Vec<u8>> { self.source.camera_preview(w, h) }

    pub fn camera_frame(&mut self) -> Option<(u32, u32, Arc<Vec<u8>>)> {
        let [p0, p1] = &self.regs;
        let word = |reg: usize, mask: u8| u16::from_be_bytes([p0[reg] & mask, p0[reg + 1]]) as u32;
        if p0[0x25] & 0x0f != 0x0f { return None; }
        let format = match p0[0x24] & 0x1f {
            0x06 => PixelFormat::Rgb565,
            0x02 => PixelFormat::Yuyv,
            0x11 => PixelFormat::Grayscale,
            _ => return None,
        };
        let (x, y) = (word(0x07, 3), word(0x05, 1));
        // The programmed window includes eight extra pixels/lines (datasheet section 4).
        let w = word(0x0b, 3).saturating_sub(8).min(WIDTH.saturating_sub(x));
        let h = word(0x09, 1).saturating_sub(8).min(HEIGHT.saturating_sub(y));
        let (out_w, out_h) = if p1[0x53] & 0x80 != 0 && p1[0x55] & 1 != 0 {
            (subsample(w, p1[0x54], p1[0x58], p1[0x59]),
             subsample(h, p1[0x54] >> 4, p1[0x56], p1[0x57]))
        } else { (w, h) };
        if out_w == 0 || out_h == 0 { return None; }
        let window = (x != 0 || y != 0 || w != WIDTH || h != HEIGHT).then_some(([x, y, x + w - 1, y + h - 1], WIDTH, HEIGHT));
        self.source.oriented_frame(out_w, out_h, format, window, [p0[0x14] & 1 != 0, p0[0x14] & 2 != 0])
    }
}

/// Count distinct sample positions in each group. The driver writes four
/// indices per axis (e.g. 0, 2, 4, 6 for 4/7), repeating zero for unused slots.
fn subsample(length: u32, ratio: u8, first: u8, last: u8) -> u32 {
    let period = (ratio & 7).max(1) as u32;
    let positions = (1u32 << (first >> 4)) | (1u32 << (first & 15)) | (1u32 << (last >> 4)) | (1u32 << (last & 15));
    let within_group = positions & ((1 << period) - 1);
    // Use the configured indices for the output geometry. The crop is resampled
    // to that size with the same nearest-neighbour converter as the OV sensors.
    let selected = if ratio & 8 == 0 { within_group } else { !within_group & ((1 << period) - 1) };
    length * selected.count_ones() / period
}

impl I2cDevice for Gc0308 {
    fn start(&mut self, read: bool) -> bool { if !read { self.first = true; } true }
    fn write(&mut self, byte: u8) -> bool {
        if self.first { self.ptr = byte; self.first = false; return true; }
        match self.ptr {
            0xfe if byte & 0x80 != 0 => self.reset(),
            0xfe => self.page_reg = byte & 0x11,
            0x00 if self.page_reg & 1 == 0 => {}
            reg => self.regs[(self.page_reg & 1) as usize][reg as usize] = byte,
        }
        self.ptr = self.ptr.wrapping_add(1);
        true
    }
    fn read(&mut self) -> u8 {
        let value = if self.ptr == 0xfe { self.page_reg } else { self.regs[(self.page_reg & 1) as usize][self.ptr as usize] };
        self.ptr = self.ptr.wrapping_add(1);
        value
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn write(sensor: &mut Gc0308, reg: u8, bytes: &[u8]) {
        assert!(sensor.start(false) && sensor.write(reg));
        for &byte in bytes { assert!(sensor.write(byte)); }
        sensor.stop();
    }

    fn read(sensor: &mut Gc0308, reg: u8) -> u8 {
        write(sensor, reg, &[]);
        assert!(sensor.start(true));
        let value = sensor.read();
        sensor.stop();
        value
    }

    #[test]
    fn chip_id_page_and_soft_reset() {
        let mut sensor = Gc0308::default();
        assert_eq!(read(&mut sensor, 0), 0x9b);
        write(&mut sensor, 0, &[0x55]);
        assert_eq!(read(&mut sensor, 0), 0x9b);
        write(&mut sensor, 0xfe, &[1]);
        write(&mut sensor, 0, &[0x55]);
        assert_eq!(read(&mut sensor, 0), 0x55);
        write(&mut sensor, 0xfe, &[0]);
        assert_eq!(read(&mut sensor, 0), 0x9b);
        write(&mut sensor, 0x24, &[0xa6, 0x0f]);
        write(&mut sensor, 0xfe, &[1]);
        write(&mut sensor, 0x54, &[0x11]);
        assert_eq!(sensor.camera_frame().map(|(w, h, _)| (w, h)), Some((640, 480)));
        write(&mut sensor, 0xfe, &[0x80]);
        assert!(sensor.camera_frame().is_none());
        assert_eq!(read(&mut sensor, 0x24), 0xa2);
        write(&mut sensor, 0xfe, &[1]);
        assert_eq!(read(&mut sensor, 0), 0);
    }

    #[test]
    fn qvga_formats_orientation_and_cache() {
        let mut sensor = Gc0308::default();
        let picture = || Picture { w: 2, h: 2, rgb: vec![255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 255] };
        sensor.set_camera_picture(picture()).unwrap();
        write(&mut sensor, 0x24, &[0xa6, 0x0f]);
        write(&mut sensor, 0xfe, &[1]);
        write(&mut sensor, 0x53, &[0x82, 0x22, 0x03, 0, 0, 0, 0]);
        write(&mut sensor, 0xfe, &[0]);
        let (w, h, frame) = sensor.camera_frame().unwrap();
        assert_eq!((w, h, frame.len()), (320, 240, 153_600));
        assert_eq!(&frame[..2], &[0xf8, 0]);
        assert_eq!(&frame[638..640], &[0x07, 0xe0]);
        assert_eq!(&frame[frame.len()-640..frame.len()-638], &[0, 0x1f]);
        assert!(Arc::ptr_eq(&frame, &sensor.camera_frame().unwrap().2));
        write(&mut sensor, 0x14, &[1]);
        assert_eq!(&sensor.camera_frame().unwrap().2[..2], &[0x07, 0xe0]);
        write(&mut sensor, 0x14, &[3]);
        let flipped = sensor.camera_frame().unwrap().2;
        assert_eq!(&flipped[..2], &[0xff, 0xff]);
        sensor.set_camera_picture(picture()).unwrap();
        assert!(!Arc::ptr_eq(&flipped, &sensor.camera_frame().unwrap().2));
        write(&mut sensor, 0x14, &[0]);
        write(&mut sensor, 0x24, &[0xb1]);
        let (_, _, frame) = sensor.camera_frame().unwrap();
        assert_eq!((frame.len(), frame[0]), (76_800, 76));
        write(&mut sensor, 0x24, &[0xa2]);
        let (_, _, frame) = sensor.camera_frame().unwrap();
        assert_eq!((frame.len(), &frame[..4]), (153_600, &[76, 84, 76, 255][..]));
        write(&mut sensor, 0x24, &[0xaf]);
        assert!(sensor.camera_frame().is_none());
    }

    #[test]
    fn window_and_fractional_subsampling() {
        let mut sensor = Gc0308::default();
        sensor.set_camera_picture(Picture { w: 2, h: 2, rgb: vec![255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 255] }).unwrap();
        write(&mut sensor, 0x24, &[0xa6, 0x0f]);
        write(&mut sensor, 0xfe, &[1]);
        write(&mut sensor, 0x55, &[0]);
        write(&mut sensor, 0xfe, &[0]);
        write(&mut sensor, 0x05, &[0, 240, 1, 64, 0, 248, 1, 72]);
        let (w, h, frame) = sensor.camera_frame().unwrap();
        assert_eq!((w, h), (320, 240));
        assert!(frame.chunks_exact(2).all(|p| p == [0xff, 0xff]));
        write(&mut sensor, 0xfe, &[1]);
        write(&mut sensor, 0x53, &[0x82, 0x77, 1, 0x02, 0x46, 0x02, 0x46]);
        assert_eq!(sensor.camera_frame().map(|(w, h, _)| (w, h)), Some((182, 137)));
        write(&mut sensor, 0xfe, &[0]);
        write(&mut sensor, 0x07, &[0xff, 0xff]);
        assert!(sensor.camera_frame().is_none());
    }
}
