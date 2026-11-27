//! Sensirion SHT4x datasheet v7, commands and 16-bit conversion in §§4.4–4.6.
use std::{cell::Cell, rc::Rc};
use esp_periph::i2c::I2cDevice;

pub struct Sht40 {
    pub sample: Rc<Cell<[f32; 2]>>,
    time: Rc<Cell<u64>>,
    cycles_per_ms: u64,
    ready_at: u64,
    reply: [u8; 6],
    position: usize,
    pub measurements: Rc<Cell<u64>>,
}

impl Sht40 {
    pub fn new(sample: Rc<Cell<[f32; 2]>>, time: Rc<Cell<u64>>, cycles_per_ms: u64, measurements: Rc<Cell<u64>>) -> Self {
        Self { sample, time, cycles_per_ms, ready_at: u64::MAX, reply: [0; 6], position: 6, measurements }
    }

    fn field(raw: u16) -> [u8; 3] {
        let [msb, lsb] = raw.to_be_bytes();
        let crc = [msb, lsb].into_iter().fold(0xffu8, |crc, byte| {
            (0..8).fold(crc ^ byte, |c, _| (c << 1) ^ if c & 0x80 != 0 { 0x31 } else { 0 })
        });
        [msb, lsb, crc]
    }
}

impl I2cDevice for Sht40 {
    fn start(&mut self, read: bool) -> bool {
        if read && self.time.get() < self.ready_at { return false; }
        if read { self.position = 0; }
        true
    }

    fn write(&mut self, byte: u8) -> bool {
        let ms = match byte {
            0xfd => 9,
            0xf6 => 5,
            0xe0 => 2,
            0x89 => 1,
            0x94 => { self.ready_at = u64::MAX; return true; }
            _ => return false,
        };
        let [t, rh] = self.sample.get();
        let (a, b) = if byte == 0x89 { (0x1624, 0x0040) } else {
            self.measurements.set(self.measurements.get() + 1);
            let units = |fraction: f32| (fraction.clamp(0.0, 1.0) * 65535.0).round() as u16;
            (units((t + 45.0) / 175.0), units((rh + 6.0) / 125.0))
        };
        self.reply[..3].copy_from_slice(&Self::field(a));
        self.reply[3..].copy_from_slice(&Self::field(b));
        self.ready_at = self.time.get().saturating_add(ms * self.cycles_per_ms);
        true
    }

    fn read(&mut self) -> u8 {
        let Some(&byte) = self.reply.get(self.position) else { return 0xff; };
        self.position += 1;
        if self.position == self.reply.len() { self.ready_at = u64::MAX; }
        byte
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn measure_has_crc_delay_and_changes_with_input() {
        assert_eq!(Sht40::field(0xbeef), [0xbe, 0xef, 0x92]);
        let time = Rc::new(Cell::new(0));
        let sample = Rc::new(Cell::new([20.0, 40.0]));
        let mut sensor = Sht40::new(sample.clone(), time.clone(), 1, Default::default());
        assert!(sensor.write(0xfd));
        assert!(!sensor.start(true));
        time.set(9);
        assert!(sensor.start(true));
        let first: Vec<_> = (0..6).map(|_| sensor.read()).collect();
        sample.set([33.0, 61.0]);
        assert!(sensor.write(0xfd));
        time.set(18);
        assert!(sensor.start(true));
        let second: Vec<_> = (0..6).map(|_| sensor.read()).collect();
        assert_ne!(first[..3], second[..3]);
        assert_ne!(first[3..], second[3..]);
    }
}
