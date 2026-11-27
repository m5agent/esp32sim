//! MPU-9250 and its bypass-connected AK8963, RM-MPU-9250A-00 rev 1.6.
//! FIFO contains the enabled raw sensor words. DMP and aux-master mode
//! are outside the polling and FIFO interfaces implemented here.
use esp_periph::i2c::I2cDevice;
use std::{cell::RefCell, collections::VecDeque, rc::Rc};

pub struct Mpu9250 {
    regs: [u8; 128],
    sample: [f64; 6],
    pointer: u8,
    select: bool,
    fifo: VecDeque<u8>,
    last_sample: u64,
}

impl Default for Mpu9250 {
    fn default() -> Self {
        let mut regs = [0; 128];
        regs[0x75] = 0x71;
        regs[0x6b] = 1;
        Self { regs, sample: [0., 0., 1., 0., 0., 0.], pointer: 0, select: true,
            fifo: VecDeque::with_capacity(512), last_sample: 0 }
    }
}

impl Mpu9250 {
    pub const ADDRESS: u8 = 0x68;

    pub fn set_sample(&mut self, sample: [f64; 6]) { self.sample = sample; }

    fn update_sample(&mut self) {
        if self.regs[0x6b] & 0x40 != 0 { return; }
        let accel_range = 2 << ((self.regs[0x1c] >> 3) & 3);
        let gyro_range = 250 << ((self.regs[0x1b] >> 3) & 3);
        for (i, v) in self.sample.into_iter().enumerate() {
            let (at, range) = if i < 3 { (0x3b + i * 2, accel_range) } else { (0x3d + i * 2, gyro_range) };
            let raw = (v * 32768. / f64::from(range)).round() as i16;
            self.regs[at..at + 2].copy_from_slice(&raw.to_be_bytes());
        }
        self.regs[0x3a] |= 1;
    }

    pub fn advance_to(&mut self, cycle: u64, hz: u64) {
        let rate_div = 1 + u64::from(self.regs[0x19]);
        let every = (hz * rate_div / 1000).max(1);
        if cycle.saturating_sub(self.last_sample) < every { return; }
        let elapsed = (cycle - self.last_sample) / every;
        self.last_sample += elapsed * every;
        self.update_sample();
        if self.regs[0x6a] & 0x40 == 0 || self.regs[0x6b] & 0x40 != 0 { return; }
        let enabled = self.regs[0x23];
        for _ in 0..elapsed.min(256) {
            for (bit, range) in [(3, 0x3b..0x41), (7, 0x41..0x43), (6, 0x43..0x45), (5, 0x45..0x47), (4, 0x47..0x49)] {
                if enabled & (1 << bit) == 0 { continue; }
                for index in range {
                    if self.fifo.len() == 512 {
                        self.regs[0x3a] |= 0x10;
                        if self.regs[0x1a] & 0x40 != 0 { break; }
                        self.fifo.pop_front();
                    }
                    self.fifo.push_back(self.regs[index]);
                }
            }
        }
    }

    fn bypass(&self) -> bool { self.regs[0x37] & 2 != 0 && self.regs[0x6a] & 0x20 == 0 }
}

impl I2cDevice for Mpu9250 {
    fn start(&mut self, read: bool) -> bool {
        self.select = !read;
        true
    }

    fn write(&mut self, byte: u8) -> bool {
        if self.select {
            self.pointer = byte & 127;
            self.select = false;
            return true;
        }
        match self.pointer {
            0x6b if byte & 0x80 != 0 => {
                let reset = Self::default();
                self.regs = reset.regs;
                self.fifo.clear();
            }
            0x6a => {
                if byte & 4 != 0 { self.fifo.clear(); }
                self.regs[0x6a] = byte & 0xf0;
            }
            0x3a..=0x48 | 0x72..=0x75 => {}
            reg => self.regs[usize::from(reg)] = byte,
        }
        self.pointer = (self.pointer + 1) & 127;
        true
    }

    fn read(&mut self) -> u8 {
        if self.pointer == 0x74 { return self.fifo.pop_front().unwrap_or(0); }
        let count = (self.fifo.len() as u16).to_be_bytes();
        let value = match self.pointer {
            0x72 => count[0],
            0x73 => count[1],
            0x3a => std::mem::take(&mut self.regs[0x3a]),
            reg => self.regs[usize::from(reg)],
        };
        self.pointer = (self.pointer + 1) & 127;
        value
    }
}

pub struct Ak8963 {
    mpu: Rc<RefCell<Mpu9250>>,
    pointer: u8,
    select: bool,
    mode: u8,
    sample: [f64; 3],
    data: [u8; 6],
}

impl Ak8963 {
    pub const ADDRESS: u8 = 0x0c;
    pub fn new(mpu: Rc<RefCell<Mpu9250>>) -> Self {
        Self { mpu, pointer: 0, select: true, mode: 0, sample: [18., 0., 43.], data: [0; 6] }
    }

    pub fn set_sample(&mut self, sample: [f64; 3]) { self.sample = sample; }
}

impl I2cDevice for Ak8963 {
    fn start(&mut self, read: bool) -> bool {
        self.select = !read;
        if !self.mpu.borrow().bypass() { return false; }
        if read {
            let (scale, min, max) = if self.mode & 0x10 != 0 { (0.15, -32760., 32760.) } else { (0.6, -8190., 8190.) };
            for (bytes, ut) in self.data.as_chunks_mut::<2>().0.iter_mut().zip(self.sample) {
                *bytes = ((ut / scale).round().clamp(min, max) as i16).to_le_bytes();
            }
        }
        true
    }

    fn write(&mut self, byte: u8) -> bool {
        if self.select {
            self.pointer = byte;
            self.select = false;
            return true;
        }
        if self.pointer == 0x0a { self.mode = byte & 0x1f; }
        if self.pointer == 0x0b && byte & 1 != 0 { self.mode = 0; }
        self.pointer = self.pointer.wrapping_add(1);
        true
    }

    fn read(&mut self) -> u8 {
        let value = match self.pointer {
            0 => 0x48,
            2 => u8::from(matches!(self.mode & 15, 1 | 2 | 4 | 6 | 8)),
            3..=8 if self.mode & 15 != 0 => self.data[usize::from(self.pointer - 3)],
            9 => {
                let bitm = self.mode & 0x10;
                if self.mode & 15 == 1 { self.mode &= 0x10; }
                bitm
            }
            0x0a => self.mode,
            0x10..=0x12 if self.mode & 15 == 15 => 128,
            _ => 0,
        };
        self.pointer = self.pointer.wrapping_add(1);
        value
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn raw_fifo_and_bypass_follow_register_settings() {
        let imu = Rc::new(RefCell::new(Mpu9250::default()));
        let mut mag = Ak8963::new(imu.clone());
        assert!(!mag.start(false));
        let mut reg = imu.borrow_mut();
        for (at, value) in [(0x37, 2), (0x6a, 0x40), (0x23, 8)] {
            reg.start(false);
            reg.write(at);
            reg.write(value);
        }
        reg.set_sample([-0.5, 0.25, 1., 0., 0., 0.]);
        reg.advance_to(1000, 1_000_000);
        reg.start(false);
        reg.write(0x72);
        reg.start(true);
        assert_eq!((reg.read(), reg.read()), (0, 6));
        assert_eq!(std::array::from_fn::<_, 6, _>(|_| reg.read()), [0xe0, 0, 0x10, 0, 0x40, 0]);
        drop(reg);
        assert!(mag.start(false));
        mag.write(0x0a);
        mag.write(0x16);
        mag.set_sample([15.0, -30.0, 45.0]);
        mag.start(false);
        mag.write(2);
        mag.start(true);
        assert_eq!(std::array::from_fn::<_, 8, _>(|_| mag.read()), [1, 100, 0, 56, 255, 44, 1, 16]);
    }
}
