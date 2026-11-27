//! M5Stack PICO-D4 Atom boards. Board detection wiring follows M5GFX
//! 28d47949bdc8 esp32_d0wdq6.inl Atom detector. I2C, RGB and audio pins
//! follow the M5Stack C008, C008-B, C008-B-V11, K117 and C008-C pin maps.
use esp_periph::i2c::I2cDevice;
use esp_soc::board::{BoardModel, I2sCodecs, VirtualCycle};
use esp_soc::devices::{Bmi270, Ws2812Chain};
use esp_soc::devices::mpu6886::Mpu6886;
use std::cell::RefCell;
use std::rc::Rc;

const BUTTON: u8 = 39;
const RGB: u8 = 27;

#[derive(Clone, Copy)]
pub(crate) enum AtomKind { Lite, Matrix, MatrixV11, AtomU, Echo }

impl AtomKind {
    pub(crate) fn from_name(name: &str) -> Option<Self> {
        Some(match name {
            "m5stack-atom-lite" => Self::Lite,
            "m5stack-atom-matrix" => Self::Matrix,
            "m5stack-atom-matrix-v11" => Self::MatrixV11,
            "m5stack-atomu" => Self::AtomU,
            "m5stack-atom-echo" => Self::Echo,
            _ => return None,
        })
    }

    pub(crate) fn name(self) -> &'static str {
        match self {
            Self::Lite => "m5stack-atom-lite",
            Self::Matrix => "m5stack-atom-matrix",
            Self::MatrixV11 => "m5stack-atom-matrix-v11",
            Self::AtomU => "m5stack-atomu",
            Self::Echo => "m5stack-atom-echo",
        }
    }

    pub(crate) fn matrix(self) -> bool { matches!(self, Self::Matrix | Self::MatrixV11) }
    pub(crate) fn sense_link(self) -> bool { self.matrix() || matches!(self, Self::Lite) }

    pub(crate) fn touch_idle(self) -> u32 {
        // M5GFX measures T7/T4 during detection. T4 stays at the classic
        // model's default 1000 counts, for ratios 26%, 43%, and 71%.
        match self {
            Self::Matrix | Self::MatrixV11 => 260,
            Self::Lite => 430,
            Self::AtomU => 710,
            Self::Echo => 1000,
        }
    }
}

enum Sensor { Mpu(Mpu6886), Bmi(Box<Bmi270>) }

impl Sensor {
    fn device(&mut self) -> &mut dyn I2cDevice {
        match self { Self::Mpu(sensor) => sensor, Self::Bmi(sensor) => sensor.as_mut() }
    }
}

#[derive(Clone)]
struct InternalImu(Rc<RefCell<Sensor>>);

impl I2cDevice for InternalImu {
    fn pins(&self) -> Option<(u8, u8)> { Some((25, 21)) }
    fn start(&mut self, read: bool) -> bool { self.0.borrow_mut().device().start(read) }
    fn write(&mut self, byte: u8) -> bool { self.0.borrow_mut().device().write(byte) }
    fn read(&mut self) -> u8 { self.0.borrow_mut().device().read() }
    fn stop(&mut self) { self.0.borrow_mut().device().stop(); }
}

pub(crate) struct Atom {
    kind: AtomKind,
    rgb: Ws2812Chain,
    imu: Option<InternalImu>,
}

impl Atom {
    pub(crate) fn new(kind: AtomKind) -> Self {
        let imu = match kind {
            AtomKind::Matrix => Some(Sensor::Mpu(Mpu6886::default())),
            AtomKind::MatrixV11 => Some(Sensor::Bmi(Box::new(Bmi270::new(Default::default(), crate::periph::CPU_HZ)))),
            _ => None,
        };
        Self {
            kind,
            rgb: Ws2812Chain::new(if kind.matrix() { 25 } else { 1 }),
            imu: imu.map(|sensor| InternalImu(Rc::new(RefCell::new(sensor)))),
        }
    }
}

impl BoardModel for Atom {
    fn name(&self) -> &'static str { self.kind.name() }
    fn named_pin(&self, name: &str) -> Option<u8> { matches!(name, "a" | "btn" | "btn1").then_some(BUTTON) }
    fn input_levels(&self) -> Vec<(u8, bool)> {
        let mut pins = vec![(BUTTON, true)];
        if self.kind.matrix() { pins.extend([(21, true), (25, true)]); }
        if matches!(self.kind, AtomKind::Echo) { pins.push((34, true)); }
        else {
            pins.push((12, false));
            if matches!(self.kind, AtomKind::AtomU) { pins.push((34, false)); }
        }
        pins
    }

    fn i2c_devices(&mut self) -> Vec<(u8, u8, Box<dyn I2cDevice>)> {
        // M5Unified selects I2C1, but either controller can route to the same pins.
        (0..2).filter_map(|bus| Some((bus, 0x68, Box::new(self.imu.as_ref()?.clone()) as Box<dyn I2cDevice>))).collect()
    }

    fn accepts_command(&self, verb: &str) -> bool { verb == "imu" && self.imu.is_some() }
    fn board_command(&mut self, _cycle: VirtualCycle, verb: &str, args: &[&str]) -> bool {
        let ("imu", [ax, ay, az, gx, gy, gz], Some(imu)) = (verb, args, &self.imu) else { return false; };
        let values = [ax, ay, az, gx, gy, gz].map(|arg| arg.parse::<f64>());
        let [Ok(ax), Ok(ay), Ok(az), Ok(gx), Ok(gy), Ok(gz)] = values else { return false; };
        if ![ax, ay, az, gx, gy, gz].iter().all(|value| value.is_finite()) { return false; }
        // M5Unified 0.3.0 IMU_Class.inl reverses sensor X and Z on both
        // Matrix generations. Host inputs are expressed in board axes.
        let accel = [-ax, ay, -az];
        let gyro = [-gx, gy, -gz];
        match &mut *imu.0.borrow_mut() {
            Sensor::Mpu(sensor) => sensor.set_sample(accel, gyro),
            Sensor::Bmi(sensor) => sensor.set_sample(accel, gyro, None),
        }
        true
    }

    fn rmt_frame(&mut self, pin: u8, bits: &[bool]) {
        if pin == RGB { self.rgb.from_bits(bits); }
    }
    fn leds(&self) -> Option<(&[[u8; 3]], u64)> { Some((&self.rgb.leds, self.rgb.updates)) }
    fn display(&self) -> Option<(u32, u32, Vec<u16>, u64)> {
        // C008-B-V11 product page, RGB LED Matrix Pixel Order Diagram:
        // each row goes from left to right in chain order.
        let side = if self.kind.matrix() { 5 } else { 1 };
        let rgb565 = self.rgb.leds.iter().map(|&[r, g, b]| {
            (u16::from(r >> 3) << 11) | (u16::from(g >> 2) << 5) | u16::from(b >> 3)
        }).collect();
        Some((side, side, rgb565, self.rgb.updates))
    }
    fn display_frames(&self) -> u64 { self.rgb.updates }
    fn display_version(&self) -> u64 { self.rgb.updates }
    fn report(&self) -> String {
        let (first, rest) = self.rgb.leds.split_first().unwrap();
        format!("[atom] {} RGB frames={} pixels={} first={:02x}{:02x}{:02x} uniform={}",
            self.name(), self.rgb.updates, self.rgb.leds.len(), first[0], first[1], first[2], rest.iter().all(|rgb| rgb == first))
    }

    fn i2s_codecs(&self) -> Option<I2sCodecs> {
        let (port, bck, ws, dout, din, speaker_gain) = match self.kind {
            AtomKind::AtomU => (0, 5, 5, 0, 19, 0.0),
            AtomKind::Echo => (1, 19, 33, 22, 23, 1.0),
            _ => return None,
        };
        Some(I2sCodecs { port, mic_port: 0, bck, ws, mck: None, dout, din, speaker_gain, mic_lanes: [true, true] })
    }
}
