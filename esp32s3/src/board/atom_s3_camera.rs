//! Camera daughter board as wired by the AtomS3R-CAM/M12 Arduino quickstart.
use esp_periph::i2c::I2cDevice;
use esp_soc::{camera::{Gc0308, Ov3660}, picture::{Jpeg, JpegError, Picture}};
use std::sync::Arc;

pub(super) const SCCB: (u8, u8) = (12, 9);
pub(super) const POWER: u8 = 18;
pub(super) const XCLK: u8 = 21;
pub(super) const DVP: [u8; 11] = [3, 42, 46, 48, 4, 17, 11, 13, 40, 14, 10];

enum Sensor { Gc0308(Box<Gc0308>), Ov3660(Ov3660) }

pub(super) struct Camera {
    sensor: Sensor,
    pub powered: bool,
    pub frames: u64,
    pub dimensions: (u32, u32),
    pub auto_increment: bool,
    pub frame_seed: u32,
    has_picture: bool,
}

impl Camera {
    pub fn new(m12: bool) -> Self {
        let sensor = if m12 { Sensor::Ov3660(Ov3660::default()) } else { Sensor::Gc0308(Box::default()) };
        Self { sensor, powered: false, frames: 0, dimensions: (0, 0),
               auto_increment: false, frame_seed: 0, has_picture: false }
    }

    pub fn address(&self) -> u8 {
        match self.sensor { Sensor::Gc0308(_) => Gc0308::ADDRESS, Sensor::Ov3660(_) => Ov3660::ADDRESS }
    }

    fn sccb(&mut self) -> &mut dyn I2cDevice {
        match &mut self.sensor { Sensor::Gc0308(d) => d.as_mut(), Sensor::Ov3660(d) => d }
    }

    pub fn picture(&mut self, p: Picture) -> Result<(), &'static str> {
        match &mut self.sensor { Sensor::Gc0308(d) => d.set_camera_picture(p), Sensor::Ov3660(d) => d.set_camera_picture(p) }?;
        self.has_picture = true;
        Ok(())
    }

    pub fn jpeg(&mut self, jpeg: Jpeg) -> Result<(), JpegError> {
        match &mut self.sensor { Sensor::Gc0308(_) => Err(JpegError::Unsupported), Sensor::Ov3660(d) => d.set_camera_jpeg(jpeg) }
    }

    pub fn preview(&self, w: u32, h: u32) -> Option<Vec<u8>> {
        match &self.sensor { Sensor::Gc0308(d) => d.camera_preview(w, h), Sensor::Ov3660(d) => d.camera_preview(w, h) }
    }

    pub fn frame(&mut self) -> Option<(u32, u32, Arc<Vec<u8>>)> {
        if !self.powered { return None; }
        let seed = (self.auto_increment || !self.has_picture).then_some(self.frame_seed);
        let frame = match &mut self.sensor {
            Sensor::Gc0308(d) => { d.set_test_pattern(seed); d.camera_frame() }
            Sensor::Ov3660(d) => { d.set_test_pattern(seed); d.camera_frame() }
        }?;
        if self.auto_increment { self.frame_seed = self.frame_seed.wrapping_add(1); }
        self.frames += 1;
        self.dimensions = (frame.0, frame.1);
        Some(frame)
    }
}

impl I2cDevice for Camera {
    fn start(&mut self, read: bool) -> bool { self.powered && self.sccb().start(read) }
    fn write(&mut self, byte: u8) -> bool { self.powered && self.sccb().write(byte) }
    fn read(&mut self) -> u8 { if self.powered { self.sccb().read() } else { 0xff } }
    fn stop(&mut self) { self.sccb().stop(); }
}
