//! StickC, StickC Plus, StickC Plus SE and StickC Plus2, as wired in the
//! K016-C, K016-P, K016-P-SE and K016-P2 schematics and M5GFX 28d47949.
use super::stickc_panel::Panel;
use esp_periph::i2c::I2cDevice;
use esp_soc::{
    board::{BoardEdge, BoardModel, SpiPins, VirtualCycle},
    devices::{bm8563::Bm8563, mpu6886::Mpu6886, BM8563_ADDRESS},
    devices::axp192::{Axp192, AXP192_ADDRESS},
    devices::lepton::{Lepton, LeptonEncoder},
    devices::nec::NecReceiver,
    devices::soft_i2c::{SoftI2c, SoftOp},
    pins::PinRoutes,
};
use std::{cell::RefCell, rc::Rc};

const SDA: u8 = 21;
const SCL: u8 = 22;
const RTC_INT: u8 = 35;
const HOLD: u8 = 4;
const BACKLIGHT: u8 = 27;
const BUZZER: u8 = 2;
const LCD_DATA: u8 = 15;
const LEPTON_CLOCK: u8 = 2;
const LEPTON_CS: u8 = 0;
const LEPTON_MISO: u8 = 25;
const LEPTON_RESET: u8 = 26;
const LEPTON_VSYNC: u8 = 38;
type Shared<T> = Rc<RefCell<T>>;

#[derive(Clone, Copy, Eq, PartialEq)]
pub(crate) enum StickKind { C, Plus, Se, Plus2, T2 }

impl StickKind {
    pub(crate) fn from_name(name: &str) -> Option<Self> {
        Some(match name {
            "m5stack-stickc" => Self::C,
            "m5stack-stickc-plus" => Self::Plus,
            "m5stack-stickc-plus-se" => Self::Se,
            "m5stack-stickc-plus2" => Self::Plus2,
            "m5stack-stickt2" => Self::T2,
            _ => return None,
        })
    }

    pub(crate) fn name(self) -> &'static str {
        match self {
            Self::C => "m5stack-stickc",
            Self::Plus => "m5stack-stickc-plus",
            Self::Se => "m5stack-stickc-plus-se",
            Self::Plus2 => "m5stack-stickc-plus2",
            Self::T2 => "m5stack-stickt2",
        }
    }
}

#[derive(Clone)]
pub(super) struct InternalBus(pub(super) Shared<dyn I2cDevice>);

impl I2cDevice for InternalBus {
    fn pins(&self) -> Option<(u8, u8)> { Some((SDA, SCL)) }
    fn start(&mut self, read: bool) -> bool { self.0.borrow_mut().start(read) }
    fn write(&mut self, data: u8) -> bool { self.0.borrow_mut().write(data) }
    fn read(&mut self) -> u8 { self.0.borrow_mut().read() }
    fn stop(&mut self) { self.0.borrow_mut().stop(); }
}

pub(crate) struct Stick {
    kind: StickKind,
    panel: Panel,
    wire: SoftI2c,
    devices: Vec<(u8, Shared<dyn I2cDevice>)>,
    axp: Option<Shared<Axp192>>,
    rtc: Shared<Bm8563>,
    imu: Option<Shared<Mpu6886>>,
    lepton: Option<Shared<Lepton>>,
    encoder: Option<Shared<LeptonEncoder>>,
    drives: [Option<bool>; 40],
    edges: Vec<BoardEdge>,
    releases: Vec<u8>,
    hold: bool,
    wake_until: VirtualCycle,
    cycle: VirtualCycle,
    rtc_low: bool,
    backlight_gpio: bool,
    backlight_pwm: u16,
    backlight_last: u16,
    backlight_updates: u64,
    vbus: bool,
    buzzer_pwm: Option<(f64, u32)>,
    buzzer_updates: u64,
    ir_frames: u64,
    ir: NecReceiver<{ crate::periph::CPU_HZ }>,
}

impl Stick {
    pub(crate) fn new(kind: StickKind) -> Self {
        let axp = (kind != StickKind::Plus2).then(|| Rc::new(RefCell::new(Axp192::default())));
        let imu = (kind != StickKind::Se).then(|| Rc::new(RefCell::new(Mpu6886::default())));
        let rtc = Rc::new(RefCell::new(Bm8563::new(crate::periph::CPU_HZ)));
        let mut devices: Vec<(u8, Shared<dyn I2cDevice>)> = vec![(BM8563_ADDRESS, rtc.clone())];
        if let Some(axp) = &axp { devices.push((AXP192_ADDRESS, axp.clone())); }
        if let Some(imu) = &imu { devices.push((Mpu6886::ADDRESS, imu.clone())); }
        let lepton = (kind == StickKind::T2).then(|| Rc::new(RefCell::new(Lepton::new(crate::periph::CPU_HZ))));
        let encoder = (kind == StickKind::T2).then(|| Rc::new(RefCell::new(LeptonEncoder::default())));
        if let Some(camera) = &lepton { devices.push((Lepton::ADDRESS, camera.clone())); }
        if let Some(encoder) = &encoder { devices.push((LeptonEncoder::ADDRESS, encoder.clone())); }
        Self { kind, panel: Panel::new(kind == StickKind::C, kind == StickKind::Plus2),
            wire: SoftI2c::idle(), devices, axp, rtc, imu, lepton, encoder, drives: [None; 40], edges: Vec::new(), releases: Vec::new(),
            hold: false, wake_until: 0, cycle: 0, rtc_low: false, backlight_gpio: false, backlight_pwm: 0, backlight_last: 0, backlight_updates: 0,
            vbus: true, buzzer_pwm: None, buzzer_updates: 0, ir_frames: 0, ir: NecReceiver::default() }
    }

    fn drive(&mut self, cycle: VirtualCycle, pin: u8, level: Option<bool>) {
        let previous = &mut self.drives[usize::from(pin)];
        if *previous == level { return; }
        *previous = level;
        if let Some(level) = level {
            self.releases.retain(|&p| p != pin);
            self.edges.push(BoardEdge { cycle, pin, level });
        } else {
            self.releases.push(pin);
        }
    }

    fn panel_power(&self) -> bool {
        self.axp.as_ref().map_or_else(|| self.latch_power(), |p| p.borrow().ldo_mv(3) != 0)
    }

    fn latch_power(&self) -> bool { self.vbus || self.hold || self.cycle < self.wake_until }

    fn brightness(&self) -> u16 {
        if let Some(axp) = &self.axp {
            // Scale output from LDO2 voltage. Legacy ScreenSwitch sets the
            // voltage code to zero (1.8 V, below visible LED forward voltage).
            let mv = axp.borrow().ldo_mv(2);
            return (u32::from(mv.saturating_sub(1800)) * 65535 / 1200).min(65535) as u16;
        }
        if self.backlight_gpio { u16::MAX } else { self.backlight_pwm }
    }

    fn ir_pin(&self) -> u8 { if self.kind == StickKind::Plus2 { 19 } else { 9 } }
}

impl BoardModel for Stick {
    fn name(&self) -> &'static str { self.kind.name() }
    fn observes_gpio_pads(&self) -> bool { true }
    fn soc_powered(&self) -> Option<bool> {
        Some(self.axp.as_ref().map_or_else(|| self.latch_power(), |chip| chip.borrow().powered()))
    }

    fn gpio_waveform_at(&mut self, cycle: VirtualCycle, routes: PinRoutes<'_>) {
        let scl = routes.software_pad_output(SCL) != Some(false);
        let sda = routes.software_pad_output(SDA) != Some(false);
        let devices = &self.devices;
        self.wire.levels(scl, sda, |op| {
            let address = match op {
                SoftOp::Start(a, _) | SoftOp::Write(a, _) | SoftOp::Read(a) | SoftOp::Stop(a) => a,
                SoftOp::Sync => return 0,
            };
            let Some((_, dev)) = devices.iter().find(|(a, _)| *a == address) else { return 0; };
            let mut dev = dev.borrow_mut();
            match op {
                SoftOp::Start(_, read) => u8::from(dev.start(read)),
                SoftOp::Write(_, data) => u8::from(dev.write(data)),
                SoftOp::Read(_) => dev.read(),
                SoftOp::Stop(_) => { dev.stop(); 0 }
                SoftOp::Sync => 0,
            }
        });
        self.drive(cycle, SDA, Some(sda && !self.wire.pull_low));
        self.drive(cycle, SCL, Some(scl));
        if self.kind == StickKind::Plus2 {
            self.hold = routes.software_pad_output(HOLD) == Some(true);
            self.backlight_gpio = routes.software_pad_output(BACKLIGHT) == Some(true);
        }
        if let Some(lepton) = &self.lepton {
            lepton.borrow_mut().reset_at(cycle, routes.software_pad_output(LEPTON_RESET) == Some(false));
        }
        let miso = self.panel.gpio(&routes, self.panel_power());
        self.drive(cycle, LCD_DATA, miso);
    }

    fn input_levels(&self) -> Vec<(u8, bool)> {
        let mut levels = vec![(37, true), (39, true), (RTC_INT, true), (SDA, true), (SCL, true)];
        if self.kind != StickKind::Plus2 { levels.push((18, true)); }
        levels.extend(self.drives.iter().enumerate().filter_map(|(pin, &level)| level.map(|v| (pin as u8, v))));
        levels
    }
    fn take_edges(&mut self) -> Vec<BoardEdge> { std::mem::take(&mut self.edges) }
    fn released_inputs(&mut self) -> Vec<u8> { std::mem::take(&mut self.releases) }
    fn named_pin(&self, name: &str) -> Option<u8> {
        match name {
            "a" | "btn1" => Some(37),
            "b" | "btn2" => Some(39),
            "pwr" | "c" if self.kind == StickKind::Plus2 => Some(35),
            _ => None,
        }
    }

    fn i2c_devices(&mut self) -> Vec<(u8, u8, Box<dyn I2cDevice>)> {
        (0..2).flat_map(|bus| self.devices.iter().map(move |(address, dev)|
            (bus, *address, Box::new(InternalBus(dev.clone())) as Box<dyn I2cDevice>))).collect()
    }

    fn uses_spi_pins(&self) -> bool { true }
    fn spi_transfer_pins_bits(&mut self, _host: u8, pins: SpiPins, tx: &[u8], rx_bits: usize) -> Vec<u8> {
        if let Some(lepton) = &self.lepton {
            if pins.sclk & (1 << LEPTON_CLOCK) != 0 && pins.cs & (1 << LEPTON_CS) != 0
                && pins.miso == Some(LEPTON_MISO) {
                return lepton.borrow_mut().transfer(rx_bits.div_ceil(8));
            }
        }
        self.panel.transfer(pins, tx, rx_bits, self.panel_power())
    }
    fn display(&self) -> Option<(u32, u32, Vec<u16>, u64)> {
        let (w, h, pixels, version) = self.panel.display(self.brightness(), self.panel_power());
        Some((w, h, pixels, version + self.backlight_updates))
    }
    fn display_frames(&self) -> u64 { self.panel.ram.frames }
    fn display_version(&self) -> u64 { self.panel.generation + self.panel.ram.pixels_written + self.backlight_updates }
    fn gram(&self) -> Option<(Vec<u16>, usize, usize)> {
        let p = &self.panel.ram;
        Some((p.gram.clone(), p.cols, p.rows))
    }

    fn default_analog_inputs(&self) -> &'static [(u8, f32)] {
        if self.kind == StickKind::Plus2 { &[(38, 1.95)] } else { &[] }
    }
    fn i2s_bitstream_pin(&self) -> Option<u8> {
        matches!(self.kind, StickKind::Plus | StickKind::Se | StickKind::Plus2).then_some(BUZZER)
    }
    fn pdm_mic(&self) -> Option<(u8, u8)> { Some((0, 34)) }
    fn rmt_frame(&mut self, pin: u8, _bits: &[bool]) {
        if pin == self.ir_pin() { self.ir_frames += 1; }
    }

    fn next_deadline(&self) -> Option<VirtualCycle> {
        self.rtc.borrow().next_deadline().into_iter()
            .chain(self.lepton.as_ref().and_then(|cam| cam.borrow().next_edge()))
            .chain((self.wake_until > self.cycle).then_some(self.wake_until)).min()
    }
    fn advance_inputs_with_clock(&mut self, cycle: VirtualCycle, clock: &dyn Fn(u8) -> Option<(f64, u32)>) {
        self.cycle = cycle;
        self.rtc.borrow_mut().advance_to(cycle);
        let rtc_low = self.rtc.borrow().irq_low();
        if rtc_low && !self.rtc_low && self.soc_powered() == Some(false) {
            if let Some(axp) = &self.axp { axp.borrow_mut().power_on(); }
            else { self.wake_until = cycle.saturating_add(crate::periph::CPU_HZ * 2); }
        }
        self.rtc_low = rtc_low;
        if let Some(lepton) = &self.lepton {
            let level = {
                let mut camera = lepton.borrow_mut();
                camera.advance_to(cycle);
                camera.vsync()
            };
            self.drive(cycle, LEPTON_VSYNC, Some(level));
        }
        if self.kind != StickKind::Plus2 {
            self.drive(cycle, RTC_INT, Some(!rtc_low));
        } else {
            self.backlight_pwm = clock(BACKLIGHT).map_or(0, |(_, duty)| duty.min(65535) as u16);
        }
        if self.kind == StickKind::Se {
            let carrier = clock(self.ir_pin()).is_some_and(|(hz, duty)|
                (30_000.0..=45_000.0).contains(&hz) && (1..65535).contains(&duty));
            self.ir.carrier(cycle, carrier);
        }
        let brightness = self.brightness();
        if brightness != self.backlight_last {
            self.backlight_last = brightness;
            self.backlight_updates += 1;
        }
        if matches!(self.kind, StickKind::Plus | StickKind::Se | StickKind::Plus2) {
            let pwm = clock(BUZZER).filter(|&(_, duty)| duty > 0 && duty < 65535);
            if pwm != self.buzzer_pwm {
                self.buzzer_pwm = pwm;
                self.buzzer_updates += 1;
            }
        }
    }

    fn accepts_command(&self, verb: &str) -> bool {
        match verb {
            "imu" | "imu-temp-raw" => self.imu.is_some(),
            "battery-mv" => self.axp.is_some(),
            "pmic-key" => true,
            "vbus-mv" => true,
            "thermal" | "encoder-step" | "encoder-button" => self.lepton.is_some(),
            _ => false,
        }
    }
    fn board_command(&mut self, cycle: VirtualCycle, verb: &str, args: &[&str]) -> bool {
        match (verb, args) {
            ("imu", [x, y, z, gx, gy, gz]) => {
                let Some(imu) = &self.imu else { return false; };
                let [Ok(x), Ok(y), Ok(z), Ok(gx), Ok(gy), Ok(gz)] =
                    [x, y, z, gx, gy, gz].map(|s| s.parse::<f64>()) else { return false; };
                let values = [x, y, z, gx, gy, gz];
                if !values.iter().all(|v| v.is_finite()) { return false; }
                imu.borrow_mut().set_sample(values);
            }
            ("imu-temp-raw", [value]) => {
                let (Some(imu), Ok(value)) = (&self.imu, value.parse()) else { return false; };
                imu.borrow_mut().set_temperature_raw(value);
            }
            ("battery-mv", [mv]) => {
                let (Some(axp), Ok(mv)) = (&self.axp, mv.parse()) else { return false; };
                axp.borrow_mut().set_battery_mv(mv);
            }
            ("vbus-mv", [mv]) => {
                let Ok(mv) = mv.parse() else { return false; };
                self.vbus = mv >= 4000;
                if let Some(axp) = &self.axp { axp.borrow_mut().set_vbus_mv(mv); }
            }
            ("pmic-key", [key @ ("short" | "long")]) => {
                if let Some(axp) = &self.axp {
                    axp.borrow_mut().power_key(*key == "long");
                } else if *key == "short" {
                    self.wake_until = cycle.saturating_add(crate::periph::CPU_HZ * 2);
                } else {
                    return false;
                }
            }
            ("thermal", [low, high]) => {
                let (Some(camera), Ok(low), Ok(high)) = (&self.lepton, low.parse(), high.parse()) else { return false; };
                return camera.borrow_mut().set_scene(low, high);
            }
            ("encoder-step", [delta]) => {
                let (Some(enc), Ok(delta)) = (&self.encoder, delta.parse()) else { return false; };
                enc.borrow_mut().turn(delta);
            }
            ("encoder-button", [state @ ("down" | "up")]) => {
                let Some(enc) = &self.encoder else { return false; };
                enc.borrow_mut().press(*state == "down");
            }
            _ => return false,
        }
        true
    }

    fn report(&self) -> String {
        let [sec, min, hour, day, _, month, year] = self.rtc.borrow().calendar_bcd();
        let panel = &self.panel.ram;
        let thermal = self.lepton.as_ref().map_or(0, |cam| cam.borrow().frames);
        let pmic_irq = self.axp.as_ref().is_some_and(|p| p.borrow().irq_low());
        format!("[stick] {} frames={} pixels={} id_reads={} soft_i2c={} panel_power={} backlight={} backlight_updates={} hold={} pmic_irq={} buzzer_updates={} ir_frames={} thermal_frames={} rtc={year:02x}-{month:02x}-{day:02x}T{hour:02x}:{min:02x}:{sec:02x} madctl={:02x} nec={:?}",
            self.name(), self.panel.ram.frames, self.panel.ram.pixels_written, self.panel.id_reads,
            self.wire.transactions, self.panel_power(), self.brightness(), self.backlight_updates,
            self.hold, pmic_irq, self.buzzer_updates, self.ir_frames, thermal, panel.madctl, self.ir.recent)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn variant_inventory_and_battery_divider_match_the_bom() {
        for (kind, expected) in [
            (StickKind::C, vec![0x51, 0x34, 0x68]),
            (StickKind::Plus, vec![0x51, 0x34, 0x68]),
            (StickKind::Se, vec![0x51, 0x34]),
            (StickKind::Plus2, vec![0x51, 0x68]),
            (StickKind::T2, vec![0x51, 0x34, 0x68, 0x2a, 0x30]),
        ] {
            let mut b = Stick::new(kind);
            let mut devices = b.i2c_devices();
            let actual = devices.iter_mut().filter(|(id, _, _)| *id == 1).map(|(_, addr, dev)| {
                assert_eq!(dev.pins(), Some((21, 22)));
                *addr
            }).collect::<Vec<_>>();
            assert_eq!(actual, expected);
            assert_eq!(b.default_analog_inputs().len(), usize::from(kind == StickKind::Plus2));
        }
    }
}
