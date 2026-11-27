//! AtomS3 family wiring from M5Unified 96576a14 and M5GFX 28d47949.
//! The shared hardware/soft I2C routes attach to the same off-chip devices.
use std::{cell::RefCell, rc::Rc, sync::Arc};
use esp_periph::i2c::I2cDevice;
use esp_soc::{
    board::{BoardEdge, BoardModel, I2sCodecs, SpiPins, VirtualCycle},
    devices::{bmi270::Bmi270, bmm150::Bmm150, es8311::Es8311, imu_motion::ImuMotion,
              lp5562::Lp5562, mpu6886::Mpu6886, soft_i2c::{SoftI2c, SoftOp}, ws2812::Ws2812Chain},
    picture::{Jpeg, JpegError, Picture},
    pins::PinRoutes,
};
use crate::periph::CPU_HZ;
use super::{atom_s3_camera::{self, Camera}, atom_s3_panel::{Controller, Panel}, cardputer_adv_ir::NecReceiver};

type Shared<T> = Rc<RefCell<T>>;

struct Port {
    device: Shared<dyn I2cDevice>,
    pins: (u8, u8),
}

impl I2cDevice for Port {
    fn pins(&self) -> Option<(u8, u8)> { Some(self.pins) }
    fn start(&mut self, read: bool) -> bool { self.device.borrow_mut().start(read) }
    fn write(&mut self, byte: u8) -> bool { self.device.borrow_mut().write(byte) }
    fn read(&mut self) -> u8 { self.device.borrow_mut().read() }
    fn stop(&mut self) { self.device.borrow_mut().stop(); }
}

struct Wire {
    pins: (u8, u8),
    devices: Vec<(u8, Shared<dyn I2cDevice>)>,
    soft: SoftI2c,
    acks: u64,
}

impl Wire {
    fn new(pins: (u8, u8)) -> Self { Self { pins, devices: Vec::new(), soft: SoftI2c::idle(), acks: 0 } }

    fn gpio(&mut self, routes: &PinRoutes<'_>) -> [(u8, Option<bool>); 2] {
        let (sda, scl) = self.pins;
        let (sda_out, scl_out) = (routes.software_pad_output(sda), routes.software_pad_output(scl));
        let (sda_high, scl_high) = (sda_out != Some(false), scl_out != Some(false));
        let mut bus = |op| {
            let address = match op {
                SoftOp::Start(addr, _) | SoftOp::Write(addr, _) | SoftOp::Read(addr) | SoftOp::Stop(addr) => addr,
                SoftOp::Sync => return 0,
            };
            let Some((_, dev)) = self.devices.iter().find(|(at, _)| *at == address) else {
                return if matches!(op, SoftOp::Read(_)) { 0xff } else { 0 };
            };
            let mut dev = dev.borrow_mut();
            match op {
                SoftOp::Start(_, read) => {
                    let ack = dev.start(read);
                    self.acks += u64::from(ack);
                    u8::from(ack)
                }
                SoftOp::Write(_, byte) => u8::from(dev.write(byte)),
                SoftOp::Read(_) => dev.read(),
                SoftOp::Stop(_) => { dev.stop(); 0 }
                SoftOp::Sync => 0,
            }
        };
        if scl_high != self.soft.scl { self.soft.levels(scl_high, self.soft.sda, &mut bus); }
        if sda_high != self.soft.sda { self.soft.levels(scl_high, sda_high, &mut bus); }
        let data_level = if self.soft.pull_low { Some(false) } else { sda_out.is_none().then_some(true) };
        [(sda, data_level), (scl, scl_out.is_none().then_some(true))]
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub(super) enum AtomBoard { Lite, S3, S3Gc9107, U, R, RGc9107, Voice, Cam, M12 }

impl AtomBoard {
    pub fn name(self) -> &'static str {
        match self {
            Self::Lite => "m5stack-atoms3-lite", Self::S3 => "m5stack-atoms3", Self::S3Gc9107 => "m5stack-atoms3-gc9107",
            Self::U => "m5stack-atoms3u", Self::R => "m5stack-atoms3r", Self::RGc9107 => "m5stack-atoms3r-gc9107",
            Self::Voice => "m5stack-atom-echos3r", Self::Cam => "m5stack-atoms3r-cam", Self::M12 => "m5stack-atoms3r-m12",
        }
    }

    fn r_series(self) -> bool { matches!(self, Self::R | Self::RGc9107 | Self::Voice | Self::Cam | Self::M12) }

    fn ir(self) -> u8 {
        match self { Self::Lite | Self::S3 | Self::S3Gc9107 => 4, Self::U => 12, _ => 47 }
    }
}

pub(super) struct AtomS3 {
    model: AtomBoard,
    wire: Option<Wire>,
    sccb: Option<Wire>,
    camera: Option<Shared<Camera>>,
    panel: Option<Panel>,
    mpu: Option<Shared<Mpu6886>>,
    bmi: Option<Shared<Bmi270>>,
    lp: Option<Shared<Lp5562>>,
    audio: Option<Es8311>,
    amp: bool,
    rgb: Option<Ws2812Chain>,
    rgb_recent: Vec<[u8; 3]>,
    ir: NecReceiver,
    ir_pwm: bool,
    backlight: u32,
    backlight_updates: u64,
    drive: [Option<bool>; 49],
    edges: Vec<BoardEdge>,
    releases: Vec<u8>,
}

impl AtomS3 {
    pub fn new(model: AtomBoard) -> Self {
        let r = model.r_series();
        let lcd = match model {
            AtomBoard::S3 | AtomBoard::R => Some(Controller::St7735s),
            AtomBoard::S3Gc9107 | AtomBoard::RGc9107 => Some(Controller::Gc9107),
            _ => None,
        };
        let panel = lcd.map(|chip| Panel::new(r, chip));
        let mpu = (panel.is_some() && !r).then(|| Rc::new(RefCell::new(Mpu6886::default())));
        // The C126, CAM and M12 product schematics route BMM150 to the BMI270
        // auxiliary bus. The SoC's G45/G0 bus sees only the BMI270 address.
        let bmi = (r && model != AtomBoard::Voice).then(|| {
            let sensor = Bmi270::new(Arc::new(ImuMotion::default()), CPU_HZ)
                .with_aux_bmm150(Bmm150::new(Default::default()));
            Rc::new(RefCell::new(sensor))
        });
        let lp = (r && panel.is_some()).then(|| Rc::new(RefCell::new(Lp5562::default())));
        let audio = (model == AtomBoard::Voice).then(Es8311::new);
        let rgb = matches!(model, AtomBoard::Lite | AtomBoard::U).then(|| Ws2812Chain::new(1));
        let wire = (r || mpu.is_some()).then(|| {
            let mut wire = Wire::new(if r { (45, 0) } else { (38, 39) });
            if let Some(d) = &mpu { wire.devices.push((Mpu6886::ADDRESS, d.clone())); }
            if let Some(d) = &bmi { wire.devices.push((0x68, d.clone())); }
            if let Some(d) = &lp { wire.devices.push((Lp5562::ADDRESS, d.clone())); }
            if let Some(d) = &audio { wire.devices.push((Es8311::ADDRESS, Rc::new(RefCell::new(d.i2c_port())))); }
            wire
        });
        let camera = matches!(model, AtomBoard::Cam | AtomBoard::M12)
            .then(|| Rc::new(RefCell::new(Camera::new(model == AtomBoard::M12))));
        let sccb = camera.as_ref().map(|d| {
            let mut wire = Wire::new(atom_s3_camera::SCCB);
            wire.devices.push((d.borrow().address(), d.clone()));
            wire
        });
        Self { model, wire, sccb, camera, panel, mpu, bmi, lp, audio, amp: false, rgb, rgb_recent: Vec::new(),
               ir: NecReceiver::default(), ir_pwm: false, backlight: 0, backlight_updates: 0,
               drive: [None; 49], edges: Vec::new(), releases: Vec::new() }
    }

    fn drive(&mut self, cycle: VirtualCycle, pin: u8, level: Option<bool>) {
        let previous = &mut self.drive[usize::from(pin)];
        if *previous == level { return; }
        *previous = level;
        if let Some(level) = level {
            self.releases.retain(|&p| p != pin);
            self.edges.push(BoardEdge { cycle, pin, level });
        } else {
            self.releases.push(pin);
        }
    }
}

impl BoardModel for AtomS3 {
    fn name(&self) -> &'static str { self.model.name() }
    fn uses_gpio_edges(&self) -> bool { self.wire.is_some() || self.panel.is_some() }
    fn observes_gpio_pads(&self) -> bool { true }

    fn gpio_waveform_at(&mut self, cycle: VirtualCycle, routes: PinRoutes<'_>) {
        if let Some(wire) = &mut self.wire {
            for (pin, level) in wire.gpio(&routes) { self.drive(cycle, pin, level); }
        }
        if let Some(camera) = &mut self.camera {
            camera.borrow_mut().powered = routes.software_pad_output(atom_s3_camera::POWER) == Some(false);
        }
        if let Some(wire) = &mut self.sccb {
            for (pin, level) in wire.gpio(&routes) { self.drive(cycle, pin, level); }
        }
        if let Some(panel) = &mut self.panel {
            let data = panel.gpio(&routes);
            self.drive(cycle, 21, data);
        }
        if self.audio.is_some() { self.amp = routes.software_pad_output(18) == Some(true); }
        if !self.ir_pwm {
            self.ir.carrier(cycle, routes.software_pad_output(self.model.ir()) == Some(true));
        }
    }

    fn take_edges(&mut self) -> Vec<BoardEdge> { std::mem::take(&mut self.edges) }
    fn released_inputs(&mut self) -> Vec<u8> { std::mem::take(&mut self.releases) }
    fn input_levels(&self) -> Vec<(u8, bool)> {
        let mut levels = vec![(41, true)];
        if self.rgb.is_some() { levels.extend([(16, true), (self.model.ir(), false)]); }
        levels.extend(self.drive.iter().enumerate().filter_map(|(pin, state)| state.map(|v| (pin as u8, v))));
        levels
    }
    fn named_pin(&self, name: &str) -> Option<u8> {
        match name { "btn" | "btn1" | "btnA" | "button" | "g41" => Some(41), "boot" | "g0" => Some(0), _ => None }
    }

    fn observes_ledc(&self) -> bool { true }
    fn ledc_output_at(&mut self, cycle: VirtualCycle, pwm: &dyn Fn(u8) -> Option<(f64, u32)>) {
        if matches!(self.model, AtomBoard::S3 | AtomBoard::S3Gc9107) {
            let duty = pwm(16).map_or(0, |(_, duty)| duty.min(65535) * 255 / 65535);
            if duty != self.backlight { self.backlight = duty; self.backlight_updates += 1; }
        }
        let ir = pwm(self.model.ir());
        if ir.is_some() || self.ir_pwm {
            let carrier = ir.is_some_and(|(hz, duty)| (30_000.0..=45_000.0).contains(&hz) && (1..65535).contains(&duty));
            self.ir.carrier(cycle, carrier);
        }
        self.ir_pwm = ir.is_some();
    }

    fn rmt_frame(&mut self, pin: u8, bits: &[bool]) {
        if pin != 35 { return; }
        if let Some(rgb) = &mut self.rgb {
            let previous = rgb.updates;
            rgb.from_bits(bits);
            if previous != rgb.updates && self.rgb_recent.last() != Some(&rgb.leds[0]) {
                if self.rgb_recent.len() == 16 { self.rgb_recent.remove(0); }
                self.rgb_recent.push(rgb.leds[0]);
            }
        }
    }

    fn leds(&self) -> Option<(&[[u8; 3]], u64)> { self.rgb.as_ref().map(|rgb| (rgb.leds.as_slice(), rgb.updates)) }

    fn uses_spi_pins(&self) -> bool { self.panel.is_some() }
    fn spi_transfer_pins(&mut self, _host: u8, pins: SpiPins, tx: &[u8], rx_len: usize) -> Vec<u8> {
        self.panel.as_mut().map_or_else(|| vec![0xff; rx_len], |panel| panel.transfer(pins, tx, rx_len))
    }
    fn display(&self) -> Option<(u32, u32, Vec<u16>, u64)> {
        let panel = self.panel.as_ref()?;
        let brightness = self.lp.as_ref().map_or(self.backlight, |lp| u32::from(lp.borrow().outputs()[3]));
        Some((128, 128, panel.display(brightness), self.display_version()))
    }
    fn display_version(&self) -> u64 {
        self.panel.as_ref().map_or(0, |p| p.version + p.ram.pixels_written + p.ram.resets)
            + self.lp.as_ref().map_or(self.backlight_updates, |lp| lp.borrow().version)
    }
    fn display_frames(&self) -> u64 { self.panel.as_ref().map_or(0, |p| p.ram.frames) }
    fn gram(&self) -> Option<(Vec<u16>, usize, usize)> {
        self.panel.as_ref().map(|p| (p.ram.gram.clone(), p.ram.cols, p.ram.rows))
    }

    fn i2c_devices(&mut self) -> Vec<(u8, u8, Box<dyn I2cDevice>)> {
        self.wire.iter().chain(&self.sccb).flat_map(|wire| (0..2).flat_map(move |bus| wire.devices.iter().map(move |(address, device)| {
            (bus, *address, Box::new(Port { device: device.clone(), pins: wire.pins }) as Box<dyn I2cDevice>)
        }))).collect()
    }

    fn camera_xclk_pin(&self) -> Option<u8> { self.camera.as_ref().map(|_| atom_s3_camera::XCLK) }
    fn camera_dvp_pins(&self) -> Option<[u8; 11]> { self.camera.as_ref().map(|_| atom_s3_camera::DVP) }
    fn set_camera_picture(&mut self, p: Picture) -> Result<(), &'static str> {
        self.camera.as_ref().ok_or("no camera on this board")?.borrow_mut().picture(p)
    }
    fn set_camera_jpeg(&mut self, jpeg: Jpeg) -> Result<(), JpegError> {
        self.camera.as_ref().ok_or(JpegError::Unsupported)?.borrow_mut().jpeg(jpeg)
    }
    fn service_state(&self) -> Vec<(&'static str, esp_soc::json::Json)> {
        use esp_soc::json::Json;
        let Some(camera) = &self.camera else { return Vec::new(); };
        let camera = camera.borrow();
        vec![("camera", Json::object([
            ("auto-increment", Json::Bool(camera.auto_increment)),
            ("powered", Json::Bool(camera.powered)),
            ("frame-seed", Json::counter(camera.frame_seed.into())),
            ("frames", Json::counter(camera.frames)),
        ]))]
    }
    fn set_camera_auto_increment(&mut self, enabled: bool) -> bool {
        let Some(camera) = &self.camera else { return false; };
        camera.borrow_mut().auto_increment = enabled;
        true
    }
    fn camera_frame(&mut self) -> Option<(u32, u32, Arc<Vec<u8>>)> { self.camera.as_ref()?.borrow_mut().frame() }
    fn camera_preview(&self, w: u32, h: u32) -> Option<Vec<u8>> { self.camera.as_ref()?.borrow().preview(w, h) }

    fn pdm_mic(&self) -> Option<(u8, u8)> {
        // M5Unified 96576a14: PDM clock G39, data G38.
        (self.model == AtomBoard::U).then_some((39, 38))
    }
    fn i2s_codecs(&self) -> Option<I2sCodecs> {
        let mut codec = self.audio.as_ref()?.i2s_codecs(1, 1, 17, 3, 48, 4);
        if !self.amp { codec.speaker_gain = 0.; }
        Some(codec)
    }

    fn accepts_command(&self, verb: &str) -> bool {
        (verb == "imu" && (self.mpu.is_some() || self.bmi.is_some())) || (verb == "mag" && self.bmi.is_some())
    }
    fn board_command(&mut self, _cycle: VirtualCycle, verb: &str, args: &[&str]) -> bool {
        if !self.accepts_command(verb) { return false; }
        let Some(v) = args.iter().map(|s| s.parse::<f64>().ok().filter(|v| v.is_finite())).collect::<Option<Vec<_>>>() else { return false; };
        match (verb, v.as_slice()) {
            ("imu", &[x, y, z, gx, gy, gz]) => {
                if let Some(d) = &self.mpu { d.borrow_mut().set_sample([x, y, z, gx, gy, gz]); }
                if let Some(d) = &self.bmi { d.borrow_mut().set_sample([x, y, z], [gx, gy, gz], None); }
            }
            ("mag", &[x, y, z]) => {
                if let Some(d) = &self.bmi { d.borrow_mut().set_mag_field([x, y, z]); }
            }
            _ => return false,
        }
        true
    }

    fn report(&self) -> String {
        let acks = self.wire.as_ref().map_or(0, |wire| wire.acks);
        let mut report = format!("[{}] NEC frames={} recent={:?}; soft I2C ACKs={acks}", self.name(), self.ir.count, self.ir.recent);
        if let Some(rgb) = &self.rgb { report += &format!("; RGB updates={} recent={:?}", rgb.updates, self.rgb_recent); }
        if let Some(panel) = &self.panel {
            report += &format!("; LCD id_reads={} frames={} pixels={} madctl={:02x}",
                              panel.id_reads, panel.ram.frames, panel.ram.pixels_written, panel.ram.madctl);
        }
        if let Some(lp) = &self.lp { report += &format!("; LP5562 RGBW={:?}", lp.borrow().outputs()); }
        if let Some(camera) = &self.camera {
            let camera = camera.borrow();
            report += &format!("; SCCB={:02x} powered={} frames={} size={}x{}",
                               camera.address(), camera.powered, camera.frames,
                               camera.dimensions.0, camera.dimensions.1);
        }
        report
    }
}
