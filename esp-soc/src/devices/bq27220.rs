//! BQ27220 standard measurement commands (TI SLUUBD4A, sections 4.2–4.16).
//! The model takes a battery sample in register units and latches it per read.

use esp_periph::i2c::I2cDevice;

pub const ADDRESS: u8 = 0x55;

#[derive(Clone, Copy)]
pub struct BatterySample {
    pub millivolts: u16,
    pub milliamps: i16,
    pub percent: u8,
}

pub struct Bq27220 {
    pub sample: BatterySample,
    latched: BatterySample,
    pointer: u8,
    first: bool,
}

impl Bq27220 {
    pub fn new(sample: BatterySample) -> Self {
        Self { sample, latched: sample, pointer: 0, first: true }
    }

    pub fn service_state(&self) -> crate::json::Json {
        use crate::json::Json;
        Json::object([
            ("voltage-mv", Json::Int(self.sample.millivolts.into())),
            ("current-ma", Json::Int(self.sample.milliamps.into())),
            ("state-of-charge", Json::Int(self.sample.percent.min(100).into())),
        ])
    }
}

impl I2cDevice for Bq27220 {
    fn start(&mut self, read: bool) -> bool {
        self.first = !read;
        if read { self.latched = self.sample; }
        true
    }

    fn write(&mut self, byte: u8) -> bool {
        if self.first {
            self.pointer = byte;
            self.first = false;
        }
        true
    }

    fn read(&mut self) -> u8 {
        let reg = self.pointer;
        self.pointer = self.pointer.wrapping_add(1);
        let word = match reg & !1 {
            0x08 => self.latched.millivolts,
            0x0c | 0x14 => self.latched.milliamps as u16,
            0x2c => u16::from(self.latched.percent.min(100)),
            _ => 0,
        };
        word.to_le_bytes()[usize::from(reg & 1)]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn measurement_is_signed_little_endian_and_coherent() {
        let mut gauge = Bq27220::new(BatterySample { millivolts: 3780, milliamps: -122, percent: 71 });
        gauge.start(false); gauge.write(0x14); gauge.start(true);
        let lo = gauge.read();
        gauge.sample.milliamps = 256;
        assert_eq!(i16::from_le_bytes([lo, gauge.read()]), -122);
        gauge.start(false); gauge.write(0x14); gauge.start(true);
        assert_eq!(i16::from_le_bytes([gauge.read(), gauge.read()]), 256);
    }
}
