//! TDK MPU6886 polling over I2C (register map rev. 1.1). M5Unified 0.3.0 uses
//! WHO_AM_I, PWR_MGMT_1, INT_STATUS and a 14-byte big-endian sample burst.
//! FIFO, self-test and interrupt pins are outside this model.
use esp_periph::i2c::I2cDevice;

pub struct Mpu6886 {
    regs: [u8; 128],
    ptr: u8,
    select_register: bool,
    sample: [f64; 6],
    temperature: i16,
}

impl Default for Mpu6886 {
    fn default() -> Self {
        let mut sensor = Self { regs: [0; 128], ptr: 0, select_register: true, sample: [0.0, 0.0, 1.0, 0.0, 0.0, 0.0], temperature: 0 };
        sensor.reset();
        sensor
    }
}

impl Mpu6886 {
    pub const ADDRESS: u8 = 0x68;

    fn reset(&mut self) {
        self.regs = [0; 128];
        self.regs[0x6b] = 0x40;
        self.regs[0x75] = 0x19;
    }

    /// Set readings in sensor axes in the order ax, ay, az (g), gx, gy, gz (dps).
    pub fn set_sample(&mut self, sample: [f64; 6]) {
        self.sample = sample;
    }

    pub fn set_service_axis(&mut self, property: &str, value: &str) -> bool {
        let Some((kind, axis)) = property.split_once('-') else { return false; };
        let (accel, gyro) = self.sample.split_at_mut(3);
        match kind {
            "accel" => super::imu_motion::set_axis(accel.try_into().unwrap(), axis, value, 32.0),
            "gyro" => super::imu_motion::set_axis(gyro.try_into().unwrap(), axis, value, 4000.0),
            _ => false,
        }
    }

    /// Physical inputs to the range conversion used by the I2C data burst.
    pub fn service_state(&self) -> crate::json::Json {
        use crate::json::Json;
        let [ax, ay, az, gx, gy, gz] = self.sample;
        Json::object([
            ("accel-x", Json::Float(ax)), ("accel-y", Json::Float(ay)), ("accel-z", Json::Float(az)),
            ("gyro-x", Json::Float(gx)), ("gyro-y", Json::Float(gy)), ("gyro-z", Json::Float(gz)),
        ])
    }

    /// Set the raw temperature ADC word, before the guest applies its conversion.
    pub fn set_temperature_raw(&mut self, temperature: i16) {
        self.temperature = temperature;
    }

    fn latch(&mut self) {
        if self.regs[0x6b] & 0x40 != 0 {
            self.regs[0x3a..0x49].fill(0);
            return;
        }
        let accel_range = 2 << ((self.regs[0x1c] >> 3) & 3);
        let gyro_range = 250 << ((self.regs[0x1b] >> 3) & 3);
        let [ax, ay, az, gx, gy, gz] = self.sample;
        for (start, axes, range) in [(0x3b, [ax, ay, az], accel_range), (0x43, [gx, gy, gz], gyro_range)] {
            for (word, sample) in self.regs[start..start + 6].as_chunks_mut::<2>().0.iter_mut().zip(axes) {
                let value = (sample * 32768.0 / f64::from(range)).round() as i16;
                word.copy_from_slice(&value.to_be_bytes());
            }
        }
        self.regs[0x41..0x43].copy_from_slice(&self.temperature.to_be_bytes());
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
    fn scalar_control_and_guest_burst_share_sample() {
        let mut imu = Mpu6886::default();
        select(&mut imu, 0x6b);
        imu.write(1);
        assert!(imu.set_service_axis("accel-y", "-0.5"));
        assert!(!imu.set_service_axis("accel-z", "NaN"));
        assert_eq!(imu.service_state().get("accel-y").and_then(crate::json::Json::as_f64), Some(-0.5));
        assert_eq!(imu.service_state().get("accel-z").and_then(crate::json::Json::as_f64), Some(1.0));
        select(&mut imu, 0x3d);
        assert!(imu.start(true));
        assert_eq!(i16::from_be_bytes([imu.read(), imu.read()]), -8192);
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
        imu.set_sample([-0.5, 0.25, 1.0, -125.0, 250.0, -500.0]);
        imu.set_temperature_raw(-1634);
        select(&mut imu, 0x3a);
        imu.start(true);
        assert_eq!(imu.read(), 1);
        let words: Vec<_> = (0..7).map(|_| i16::from_be_bytes([imu.read(), imu.read()])).collect();
        assert_eq!(words, [-2048, 1024, 4096, -1634, -2048, 4096, -8192]);

        select(&mut imu, 0x6b);
        imu.write(0x80);
        select(&mut imu, 0x3a);
        imu.start(true);
        assert_eq!(imu.read(), 0);
    }

    #[test]
    fn data_burst_latches_and_clips_signed_samples() {
        let mut imu = Mpu6886::default();
        select(&mut imu, 0x6b);
        imu.write(1);
        select(&mut imu, 0x1c);
        imu.write(0x10);
        imu.set_sample([1.0, -0.5, 12.0, 0.0, 0.0, 0.0]);
        select(&mut imu, 0x3b);
        imu.start(true);
        imu.set_sample([0.0; 6]);
        let raw: Vec<_> = (0..6).map(|_| imu.read()).collect();
        assert_eq!(raw, [0x10, 0, 0xf8, 0, 0x7f, 0xff]);
        select(&mut imu, 0x3b);
        imu.start(true);
        assert_eq!(imu.read(), 0);
    }
}
