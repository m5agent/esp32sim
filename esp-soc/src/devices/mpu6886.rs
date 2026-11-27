//! MPU6886 polling over I2C. M5Unified 0.3.0 MPU6886_Class.inl uses
//! WHO_AM_I, PWR_MGMT_1, INT_STATUS and a 14-byte big-endian sample burst.
//! FIFO, self-test and interrupt pins are outside this model.
use esp_periph::i2c::I2cDevice;

pub const MPU6886_ADDRESS: u8 = 0x68;

pub struct Mpu6886 {
    regs: [u8; 128],
    ptr: u8,
    select_register: bool,
    accel: [f64; 3],
    gyro: [f64; 3],
}

impl Default for Mpu6886 {
    fn default() -> Self {
        let mut sensor = Self { regs: [0; 128], ptr: 0, select_register: true, accel: [0.0, 0.0, 1.0], gyro: [0.0; 3] };
        sensor.reset();
        sensor
    }
}

impl Mpu6886 {
    fn reset(&mut self) {
        self.regs = [0; 128];
        self.regs[0x6b] = 0x40;
        self.regs[0x75] = 0x19;
    }

    /// Set accelerometer and gyroscope readings in sensor axes, in g and dps.
    pub fn set_sample(&mut self, accel: [f64; 3], gyro: [f64; 3]) {
        self.accel = accel;
        self.gyro = gyro;
    }

    fn latch(&mut self) {
        if self.regs[0x6b] & 0x40 != 0 {
            self.regs[0x3a..0x49].fill(0);
            return;
        }
        let accel_range = 2 << ((self.regs[0x1c] >> 3) & 3);
        let gyro_range = 250 << ((self.regs[0x1b] >> 3) & 3);
        for (start, axes, range) in [(0x3b, self.accel, accel_range), (0x43, self.gyro, gyro_range)] {
            for (word, sample) in self.regs[start..start + 6].as_chunks_mut::<2>().0.iter_mut().zip(axes) {
                let value = (sample * 32768.0 / f64::from(range)).round() as i16;
                word.copy_from_slice(&value.to_be_bytes());
            }
        }
        self.regs[0x3a] = 1;
    }
}

impl I2cDevice for Mpu6886 {
    fn start(&mut self, read: bool) -> bool {
        if read { self.latch(); } else { self.select_register = true; }
        true
    }

    fn write(&mut self, byte: u8) -> bool {
        if self.select_register {
            self.ptr = byte & 0x7f;
            self.select_register = false;
            return true;
        }
        match self.ptr {
            0x6b if byte & 0x80 != 0 => self.reset(),
            0x3a..=0x48 | 0x75 => {}
            _ => self.regs[usize::from(self.ptr)] = byte,
        }
        self.ptr = self.ptr.wrapping_add(1) & 0x7f;
        true
    }

    fn read(&mut self) -> u8 {
        let value = self.regs[usize::from(self.ptr)];
        self.ptr = self.ptr.wrapping_add(1) & 0x7f;
        value
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn select(imu: &mut Mpu6886, reg: u8) {
        assert!(imu.start(false));
        assert!(imu.write(reg));
    }

    #[test]
    fn id_power_ranges_and_big_endian_bursts() {
        let mut imu = Mpu6886::default();
        select(&mut imu, 0x75);
        imu.start(true);
        assert_eq!(imu.read(), 0x19);

        select(&mut imu, 0x3a);
        imu.start(true);
        assert_eq!(imu.read(), 0);

        select(&mut imu, 0x6b);
        imu.write(1);
        select(&mut imu, 0x1b);
        imu.write(0x18);
        imu.write(0x10);
        imu.set_sample([-0.5, 0.25, 1.0], [-125.0, 250.0, -500.0]);
        select(&mut imu, 0x3a);
        imu.start(true);
        assert_eq!(imu.read(), 1);
        let words: Vec<_> = (0..7).map(|_| i16::from_be_bytes([imu.read(), imu.read()])).collect();
        assert_eq!(words, [-2048, 1024, 4096, 0, -2048, 4096, -8192]);

        select(&mut imu, 0x6b);
        imu.write(0x80);
        select(&mut imu, 0x3a);
        imu.start(true);
        assert_eq!(imu.read(), 0);
    }
}
