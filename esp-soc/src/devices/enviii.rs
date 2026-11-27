//! Digital I2C subset of the Sensirion SHT30 and QST QMP6988 on Unit ENV-III.
use esp_periph::i2c::I2cDevice;
use std::{cell::Cell, rc::Rc};

pub const SHT30_ADDRESS: u8 = 0x44;
pub const QMP6988_ADDRESS: u8 = 0x70;

#[derive(Clone, Copy)]
pub struct Sample {
    pub temperature_c: f32,
    pub humidity_pct: f32,
}

pub struct Sht30 {
    sample: Rc<Cell<Sample>>,
    command: Option<u8>,
    reply: [u8; 6],
    cursor: usize,
}

impl Sht30 {
    pub fn new(sample: Rc<Cell<Sample>>) -> Self {
        Self { sample, command: None, reply: [0xff; 6], cursor: 0 }
    }

    fn encode(value: f32) -> [u8; 3] {
        let raw = (value.clamp(0.0, 1.0) * f32::from(u16::MAX)).round() as u16;
        let [hi, lo] = raw.to_be_bytes();
        let mut crc = 0xffu8;
        for byte in [hi, lo] {
            crc ^= byte;
            for _ in 0..8 {
                crc = (crc << 1) ^ if crc & 0x80 != 0 { 0x31 } else { 0 };
            }
        }
        [hi, lo, crc]
    }

    fn measure(&mut self) {
        let sample = self.sample.get();
        self.reply[..3].copy_from_slice(&Self::encode((sample.temperature_c + 45.0) / 175.0));
        self.reply[3..].copy_from_slice(&Self::encode(sample.humidity_pct / 100.0));
    }
}

impl I2cDevice for Sht30 {
    fn start(&mut self, read: bool) -> bool {
        if read { self.cursor = 0; } else { self.command = None; }
        true
    }

    fn write(&mut self, byte: u8) -> bool {
        if let Some(first) = self.command.take() {
            match u16::from_be_bytes([first, byte]) {
                0x2400 | 0x240b | 0x2416 | 0x2c06 | 0x2c0d | 0x2c10 => self.measure(),
                0x30a2 => self.reply.fill(0xff),
                _ => {}
            }
        } else {
            self.command = Some(byte);
        }
        true
    }

    fn read(&mut self) -> u8 {
        let data = self.reply.get(self.cursor).copied().unwrap_or(0xff);
        self.cursor += 1;
        data
    }
}

/// QMP6988 reset, ID, configuration, trim and a fixed nominal raw sample.
/// The trim encodes a0 = 102400 with the remaining coefficients at their
/// nominal code zero. M5Unit-ENV compensates the sample to 25 C and 101669 Pa.
#[derive(Default)]
pub struct Qmp6988 {
    pointer: u8,
    address_next: bool,
    config: u8,
    control: u8,
}

impl I2cDevice for Qmp6988 {
    fn start(&mut self, read: bool) -> bool {
        self.address_next = !read;
        true
    }

    fn write(&mut self, byte: u8) -> bool {
        if self.address_next {
            self.pointer = byte;
            self.address_next = false;
            return true;
        }
        match self.pointer {
            0xe0 if byte == 0xe6 => { self.config = 0; self.control = 0; }
            0xf1 => self.config = byte,
            0xf4 => self.control = byte,
            _ => {}
        }
        self.pointer = self.pointer.wrapping_add(1);
        true
    }

    fn read(&mut self) -> u8 {
        let reg = self.pointer;
        self.pointer = self.pointer.wrapping_add(1);
        match reg {
            0xb2 => 0x19, // a0[19:12], the lower 12 bits are zero.
            0xd1 => 0x5c,
            0xf1 => self.config,
            0xf4 => self.control,
            0xf7..=0xfc => [0xad, 0xc6, 0xc0, 0x80, 0, 0][usize::from(reg - 0xf7)],
            _ => 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sht30_snapshots_both_words_on_the_measurement_command() {
        let input = Rc::new(Cell::new(Sample { temperature_c: 25.0, humidity_pct: 50.0 }));
        let mut sensor = Sht30::new(input.clone());
        sensor.start(false);
        sensor.write(0x2c);
        sensor.write(0x06);
        input.set(Sample { temperature_c: 32.0, humidity_pct: 70.0 });
        sensor.start(true);
        assert_eq!(sensor.read(), 0x66);
        assert_eq!(sensor.read(), 0x66);
        assert_eq!(sensor.read(), 0x93);
        assert_eq!(sensor.read(), 0x80);
        assert_eq!(sensor.read(), 0x00);
        assert_eq!(sensor.read(), 0xa2);
    }
}
