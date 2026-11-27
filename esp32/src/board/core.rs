//! K001-V27 and K010/K010-V11. Pin assignments and probe sequences follow
//! M5GFX 28d47949 esp32_d0wdq6.inl, M5Unified 96576a14 and the M5Stack schematics.
use std::{cell::RefCell, rc::Rc};
use esp_periph::{i2c::I2cDevice, sdmmc::{SdCard, SpiSdCard}};
use esp_soc::{
    board::{BoardEdge, BoardModel, I2sCodecs, SpiPins, VirtualCycle},
    devices::{DcsPanel, SpiBitBang, Ws2812Chain, axp192::Axp192, axp2101::Axp2101,
              bm8563::{BM8563_ADDRESS, Bm8563}, ft5x06::{FT5X06_ADDRESS, Ft5x06, Ft5x06Points},
              ina3221::{self, Ina3221}, ip5306::Ip5306, mpu6886::Mpu6886, soft_i2c::{SoftI2c, SoftOp}},
    pins::PinRoutes,
};
use crate::periph::CPU_HZ;

const CLK: u8 = 18;
const DATA: u8 = 23;
const SD_CS: u8 = 4;
const SDA: u8 = 21;
const SCL: u8 = 22;
const TOUCH_IRQ: u8 = 39;
const RTC_IRQ: u8 = 35;
const BUTTONS: [u8; 3] = [39, 38, 37];
const LCD_ID: [u8; 3] = [0xe3, 0, 0]; // ILI9342C V102 §8.2.3, RDDID
const LCD_ID4: [u8; 3] = [0, 0x93, 0x42]; // ILI9342C V102, RDID4

#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum CoreKind { Basic, Core2, Core2Axp192 }

impl CoreKind {
    fn basic(self) -> bool { self == Self::Basic }
    fn cs(self) -> u8 { if self.basic() { 14 } else { 5 } }
    fn dc(self) -> u8 { if self.basic() { 27 } else { 15 } }
    fn miso(self) -> u8 { if self.basic() { 19 } else { 38 } }
    fn name(self) -> &'static str {
        match self {
            Self::Basic => "m5stack-basic-v27",
            Self::Core2 => "m5stack-core2",
            Self::Core2Axp192 => "m5stack-core2-axp192",
        }
    }
}

type Shared<T> = Rc<RefCell<T>>;

#[derive(Clone)]
struct Port(Shared<dyn I2cDevice>);

impl I2cDevice for Port {
    fn pins(&self) -> Option<(u8, u8)> { Some((SDA, SCL)) }
    fn start(&mut self, read: bool) -> bool { self.0.borrow_mut().start(read) }
    fn write(&mut self, byte: u8) -> bool { self.0.borrow_mut().write(byte) }
    fn read(&mut self) -> u8 { self.0.borrow_mut().read() }
    fn stop(&mut self) { self.0.borrow_mut().stop(); }
}

enum Power {
    Basic(Shared<Ip5306>),
    Axp192(Shared<Axp192>),
    Axp2101(Shared<Axp2101>),
}

#[derive(PartialEq, Eq)]
struct Rails { logic: bool, reset: bool, light: bool, speaker: bool, motor_mv: u16 }

impl Power {
    fn port(&self) -> (u8, Port) {
        match self {
            Self::Basic(dev) => (esp_soc::devices::ip5306::ADDRESS, Port(dev.clone())),
            Self::Axp192(dev) => (0x34, Port(dev.clone())),
            Self::Axp2101(dev) => (0x34, Port(dev.clone())),
        }
    }

    fn rails(&self) -> Rails {
        match self {
            Self::Basic(_) => Rails { logic: true, reset: false, light: true, speaker: true, motor_mv: 0 },
            Self::Axp192(dev) => {
                let d = dev.borrow();
                let ldo = d.reg(0x12);
                Rails { logic: ldo & 4 != 0, reset: d.reg(0x96) & 2 == 0, light: d.reg(0x28) >> 4 != 0,
                        speaker: d.reg(0x93) & 7 == 6,
                        motor_mv: if ldo & 8 != 0 { 1800 + u16::from(d.reg(0x28) & 15) * 100 } else { 0 } }
            }
            Self::Axp2101(dev) => {
                let d = dev.borrow();
                let ldo = if d.powered() { d.reg(0x90) } else { 0 };
                Rails { logic: ldo & 8 != 0, reset: ldo & 2 == 0, light: ldo & 16 != 0, speaker: ldo & 4 != 0,
                        motor_mv: if ldo & 128 != 0 { 500 + 100 * u16::from(d.reg(0x99)) } else { 0 } }
            }
        }
    }
}

pub(crate) struct Core {
    kind: CoreKind,
    power: Power,
    rails: Rails,
    rtc: Option<Shared<Bm8563>>,
    ina: Option<Shared<Ina3221>>,
    imu: Option<Shared<Mpu6886>>,
    points: Ft5x06Points,
    touch: Option<Shared<Ft5x06>>,
    wire: SoftI2c,
    ports: Vec<(u8, Port)>,
    sd: Option<SpiSdCard>,
    sd_transactions: u64,
    sd_bit: u8,
    sd_shift: u8,
    sd_clock: bool,
    panel: DcsPanel,
    spi: SpiBitBang,
    cs: bool,
    clk: bool,
    command: u8,
    read_index: usize,
    read_at: usize,
    hardware_read_bits: usize,
    id_probe: u64,
    lcd_reset: bool,
    display_version: u64,
    light: bool,
    motor_changes: u64,
    motor_min: u16,
    motor_max: u16,
    rgb: Ws2812Chain,
    cycle: VirtualCycle,
    signals: [Option<bool>; 40],
    edges: Vec<BoardEdge>,
    release: Vec<u8>,
}

impl Core {
    pub(crate) fn new(kind: CoreKind) -> Self {
        let power = match kind {
            CoreKind::Basic => Power::Basic(Rc::default()),
            CoreKind::Core2 => Power::Axp2101(Rc::new(RefCell::new(Axp2101::new(CPU_HZ)))),
            CoreKind::Core2Axp192 => Power::Axp192(Rc::default()),
        };
        let rtc = (!kind.basic()).then(|| Rc::new(RefCell::new(Bm8563::new(CPU_HZ))));
        let imu: Option<Shared<Mpu6886>> = (!kind.basic()).then(Rc::default);
        let points = Ft5x06Points::default();
        let touch = (!kind.basic()).then(|| Rc::new(RefCell::new(Ft5x06::ft6336u(points.clone()))));
        let ina: Option<Shared<Ina3221>> = (kind == CoreKind::Core2).then(Rc::default);
        let mut ports = vec![power.port()];
        if let Some(dev) = &ina { ports.push((ina3221::ADDRESS, Port(dev.clone()))); }
        if let Some(dev) = &touch { ports.push((FT5X06_ADDRESS, Port(dev.clone()))); }
        if let Some(dev) = &imu { ports.push((Mpu6886::ADDRESS, Port(dev.clone()))); }
        if let Some(dev) = &rtc { ports.push((BM8563_ADDRESS, Port(dev.clone()))); }
        let rails = power.rails();
        Self { kind, power, rtc, ina, imu, points, touch, wire: SoftI2c::idle(), ports,
               sd: None, sd_transactions: 0, sd_bit: 0, sd_shift: 0, sd_clock: false, panel: DcsPanel::new(320, 240), spi: SpiBitBang::new(CLK, DATA, kind.cs()),
               cs: true, clk: false, command: 0, read_index: 0, read_at: 0, hardware_read_bits: 0, id_probe: 0, lcd_reset: rails.reset,
               display_version: 0, light: false, motor_changes: 0, motor_min: 0, motor_max: 0, rgb: Ws2812Chain::new(10),
               cycle: 0, signals: [None; 40], edges: Vec::new(), release: Vec::new(), rails }
    }

    fn drive(&mut self, pin: u8, state: Option<bool>) {
        if self.signals[usize::from(pin)] == state { return; }
        self.signals[usize::from(pin)] = state;
        if let Some(level) = state { self.edges.push(BoardEdge { cycle: self.cycle, pin, level }); }
        else { self.release.push(pin); }
    }

    fn set_reset(&mut self, reset: bool) {
        if reset == self.lcd_reset { return; }
        if reset {
            self.panel.reset();
            self.command = 0;
            self.read_index = 0;
            self.read_at = 0;
            self.hardware_read_bits = 0;
        }
        self.lcd_reset = reset;
        self.display_version += 1;
    }

    fn panel_byte(&mut self, byte: u8) {
        if self.lcd_reset { return; }
        if !self.panel.dc {
            self.command = byte;
            self.read_at = 0;
            self.hardware_read_bits = 0;
            if byte == 0x01 { self.read_index = 0; }
        } else if self.command == 0xd9 {
            // M5GFX uses D9h to choose one numbered parameter of D3h.
            self.read_index = usize::from(byte & 0x0f).saturating_sub(1);
        }
        self.panel.byte(byte);
        self.display_version += 1;
    }

    fn response(&self) -> &[u8] {
        match self.command {
            0x04 => &LCD_ID,
            0xda => &LCD_ID[..1],
            0xdb => &LCD_ID[1..2],
            0xdc => &LCD_ID[2..],
            0xd3 => LCD_ID4.get(self.read_index..).unwrap_or(&[]),
            _ => &[],
        }
    }

    fn soft_spi(&mut self, routes: &PinRoutes<'_>) {
        let pin = |gpio| routes.software_pad_output(gpio);
        let cs = pin(self.kind.cs()) != Some(false);
        let clk = pin(CLK) == Some(true);
        let data = pin(DATA);
        if self.kind.basic() { self.set_reset(pin(33) == Some(false)); }
        self.panel.dc = pin(self.kind.dc()) == Some(true);
        self.spi.pin(self.kind.cs(), cs);
        self.spi.pin(DATA, data == Some(true));
        if let Some(byte) = self.spi.pin(CLK, clk).filter(|_| data.is_some()) { self.panel_byte(byte); }
        if cs && !self.cs { self.command = 0; self.read_at = 0; }
        let reply = if !cs && self.panel.dc && data.is_none() && !self.lcd_reset && !self.response().is_empty() {
            if self.clk && !clk {
                self.read_at = self.read_at.saturating_add(1);
                if self.read_at == 1 { self.id_probe += 1; }
            }
            // RDDID has a dummy bit. An indexed D3h read starts with the
            // parameter immediately, so M5GFX's first clock drops its MSB.
            let dummy = usize::from(self.command != 0xd3);
            Some(self.read_at.checked_sub(dummy).is_some_and(|bit|
                self.response().get(bit / 8).is_some_and(|b| b & (0x80 >> (bit & 7)) != 0)))
        } else { Some(true) };
        self.drive(DATA, if data.is_none() { reply } else { None });
        self.cs = cs;
        self.clk = clk;
    }

    fn nanoseconds(&self) -> u64 { self.cycle.saturating_mul(1000) / (CPU_HZ / 1_000_000) }

    fn soft_sd(&mut self, routes: &PinRoutes<'_>) {
        let now_ns = self.nanoseconds();
        let Some(card) = &mut self.sd else { return; };
        let clock = routes.software_pad_output(CLK);
        let cs = routes.software_pad_output(SD_CS);
        let mut reply = Some(true);
        if cs != Some(false) {
            card.deselect();
            self.sd_bit = 0;
            self.sd_shift = 0;
        } else if self.rails.logic {
            // Toggling the same pads from GPIO performs the power-up SPI
            // commands in M5GFX prepare.inl before the hardware host is set up.
            if clock == Some(true) && !self.sd_clock {
                let mosi = routes.software_pad_output(DATA) != Some(false);
                self.sd_shift = (self.sd_shift << 1) | u8::from(mosi);
                self.sd_bit += 1;
                if self.sd_bit == 8 {
                    card.exchange(self.sd_shift, now_ns);
                    self.sd_transactions += 1;
                    self.sd_bit = 0;
                    self.sd_shift = 0;
                }
            }
            if clock.is_some() {
                reply = Some(card.pending_miso(now_ns) & (0x80 >> self.sd_bit) != 0);
            }
        }
        self.sd_clock = clock == Some(true);
        self.drive(self.kind.miso(), reply);
    }

    fn soft_i2c(&mut self, routes: &PinRoutes<'_>) {
        let (sda, scl) = (routes.software_pad_output(SDA), routes.software_pad_output(SCL));
        let scl_high = scl != Some(false);
        let mut exchange = |op| {
            let address = match op {
                SoftOp::Start(a, _) | SoftOp::Write(a, _) | SoftOp::Read(a) | SoftOp::Stop(a) => a,
                SoftOp::Sync => return 0,
            };
            let Some((_, dev)) = self.ports.iter_mut().find(|(a, _)| *a == address) else {
                return if matches!(op, SoftOp::Read(_)) { 0xff } else { 0 };
            };
            match op {
                SoftOp::Start(_, read) => u8::from(dev.start(read)),
                SoftOp::Write(_, byte) => u8::from(dev.write(byte)),
                SoftOp::Read(_) => dev.read(),
                SoftOp::Stop(_) => { dev.stop(); 0 }
                SoftOp::Sync => 0,
            }
        };
        if scl_high != self.wire.scl { self.wire.levels(scl_high, self.wire.sda, &mut exchange); }
        if (sda != Some(false)) != self.wire.sda { self.wire.levels(scl_high, sda != Some(false), &mut exchange); }
        self.drive(SDA, if self.wire.pull_low { Some(false) } else { sda.is_none().then_some(true) });
        self.drive(SCL, scl.is_none().then_some(true));
    }
}

impl BoardModel for Core {
    fn name(&self) -> &'static str { self.kind.name() }
    fn named_pin(&self, name: &str) -> Option<u8> {
        if !self.kind.basic() { return None; }
        match name {
            "a" | "btnA" => Some(BUTTONS[0]),
            "b" | "btnB" => Some(BUTTONS[1]),
            "c" | "btnC" => Some(BUTTONS[2]),
            _ => None,
        }
    }

    fn input_levels(&self) -> Vec<(u8, bool)> {
        let mut levels = vec![(SDA, true), (SCL, true), (CLK, true), (DATA, true), (SD_CS, true), (self.kind.miso(), true)];
        if self.kind.basic() { levels.extend(BUTTONS.map(|pin| (pin, true))); }
        else { levels.extend([(TOUCH_IRQ, true), (RTC_IRQ, true)]); }
        levels
    }

    fn uses_gpio_waveform(&self) -> bool { true }
    fn gpio_waveform_at(&mut self, cycle: VirtualCycle, routes: PinRoutes<'_>) {
        self.cycle = cycle;
        self.soft_spi(&routes);
        self.soft_i2c(&routes);
        self.soft_sd(&routes);
        if self.kind.basic() && routes.software_pad_output(32) == Some(true) && !self.light {
            self.light = true;
            self.display_version += 1;
        }
    }

    fn advance_inputs_with_clock(&mut self, cycle: VirtualCycle, pwm: &dyn Fn(u8) -> Option<(f64, u32)>) {
        self.cycle = cycle;
        if self.kind.basic() {
            if let Some((_, duty)) = pwm(32) {
                if self.light != (duty != 0) { self.light = duty != 0; self.display_version += 1; }
            }
        } else {
            let rails = self.power.rails();
            if rails != self.rails {
                self.set_reset(rails.reset || !rails.logic);
                if !rails.logic && self.rails.logic {
                    if let Some(sd) = &mut self.sd { sd.power_off(); }
                }
                if rails.light != self.rails.light { self.display_version += 1; }
                if rails.motor_mv != self.rails.motor_mv {
                    self.motor_changes += 1;
                    if rails.motor_mv != 0 {
                        self.motor_max = self.motor_max.max(rails.motor_mv);
                        self.motor_min = if self.motor_min == 0 { rails.motor_mv } else { self.motor_min.min(rails.motor_mv) };
                    }
                }
                self.rails = rails;
            }
            if let Some(rtc) = &self.rtc {
                rtc.borrow_mut().advance_to(cycle);
                let high = !rtc.borrow().irq_low();
                self.drive(RTC_IRQ, Some(high));
            }
        }
    }

    fn next_deadline(&self) -> Option<VirtualCycle> { self.rtc.as_ref().and_then(|rtc| rtc.borrow().next_deadline()) }
    fn take_edges(&mut self) -> Vec<BoardEdge> { std::mem::take(&mut self.edges) }
    fn released_inputs(&mut self) -> Vec<u8> { std::mem::take(&mut self.release) }
    fn uses_spi_pins(&self) -> bool { true }
    fn has_spi_sd_slot(&self) -> bool { true }
    fn spi_sd_state(&self) -> Option<esp_periph::sdmmc::SdMediaState> { self.sd.as_ref().map(esp_periph::sdmmc::SpiSdCard::media_state) }
    fn set_spi_sd_present(&mut self, _cycle: VirtualCycle, present: bool) -> bool {
        self.sd.as_mut().is_some_and(|card| card.set_present(present))
    }
    fn insert_spi_sd(&mut self, sd: SdCard) { self.sd = Some(SpiSdCard::new(sd)); }

    fn spi_transfer_pins_bits(&mut self, _host: u8, pins: SpiPins, tx: &[u8], rx_bits: usize) -> Vec<u8> {
        let len = rx_bits.div_ceil(8);
        let socket = SpiPins { sclk: 1 << CLK, mosi: 1 << DATA, miso: Some(self.kind.miso()), cs: 1 << SD_CS };
        if pins.sclk & socket.sclk == 0 { return vec![0xff; len]; }
        let lcd = pins.cs & (1 << self.kind.cs()) != 0;
        let mut sd_reply = None;
        let now_ns = self.nanoseconds();
        if let Some(sd) = &mut self.sd {
            if self.rails.logic {
                if pins.cs & socket.cs != 0 { self.sd_transactions += 1; }
                // Clock both selected devices, but model MISO contention as no valid data.
                let sd_pins = if lcd { SpiPins { miso: None, ..pins } } else { pins };
                sd_reply = sd_pins.transfer_sd(socket, sd, tx, len, now_ns);
            }
        }
        if !lcd { return sd_reply.unwrap_or_else(|| vec![0xff; len]); }
        if self.lcd_reset { return vec![0xff; len]; }
        if self.panel.dc && !self.response().is_empty() && len != 0 {
            if pins.miso != Some(DATA) && pins.miso != socket.miso { return vec![0xff; len]; }
            if pins.cs & socket.cs != 0 && self.sd.is_some() && self.rails.logic { return vec![0xff; len]; }
            if self.hardware_read_bits == 0 { self.id_probe += 1; }
            let dummy = usize::from(self.command == 0x04);
            let response = self.response();
            let mut values = vec![0; len];
            for i in 0..rx_bits {
                let index = self.hardware_read_bits.saturating_add(i);
                if let Some(bit) = index.checked_sub(dummy) {
                    if response.get(bit / 8).is_some_and(|byte| byte & (0x80 >> (bit & 7)) != 0) {
                        values[i / 8] |= 0x80 >> (i & 7);
                    }
                }
            }
            self.hardware_read_bits = self.hardware_read_bits.saturating_add(rx_bits);
            return values;
        }
        if pins.mosi & socket.mosi != 0 { for &byte in tx { self.panel_byte(byte); } }
        vec![0xff; len]
    }

    fn i2c_devices(&mut self) -> Vec<(u8, u8, Box<dyn I2cDevice>)> {
        self.ports.iter().flat_map(|(address, dev)| (0..2).map(move |bus|
            (bus, *address, Box::new(dev.clone()) as Box<dyn I2cDevice>))).collect()
    }

    fn i2s_codecs(&self) -> Option<I2sCodecs> {
        (!self.kind.basic()).then_some(I2sCodecs {
            port: 1, mic_port: 0, bck: 12, ws: 0, mck: None, dout: 2, din: 34,
            speaker_gain: f32::from(self.rails.speaker), mic_lanes: [true, true],
        })
    }

    fn touch(&mut self, x: u16, y: u16, down: bool) {
        if self.kind.basic() { return; }
        // FT6336 extends below the 240-row glass. M5Unified maps those
        // coordinates to the capacitive A/B/C buttons.
        self.points.lock().unwrap()[0] = down.then_some((x.min(319), y.min(279)));
    }

    fn touch_irq(&self) -> Option<(u8, bool)> {
        (!self.kind.basic()).then(|| (TOUCH_IRQ, self.points.lock().unwrap()[0].is_none()))
    }

    fn rmt_frame(&mut self, pin: u8, bits: &[bool]) {
        if !self.kind.basic() && pin == 25 { self.rgb.from_bits(bits); }
    }

    fn leds(&self) -> Option<(&[[u8; 3]], u64)> {
        (!self.kind.basic()).then_some((&self.rgb.leds, self.rgb.updates))
    }

    fn display(&self) -> Option<(u32, u32, Vec<u16>, u64)> {
        let p = &self.panel;
        let light = if self.kind.basic() { self.light } else { self.rails.light };
        let blank = self.lcd_reset || !p.on || p.sleeping || !light;
        let exchange = p.madctl & 8 == 0;
        let pixels = p.gram.iter().map(|&px| {
            if blank { 0 }
            else if exchange { (px & 0x07e0) | ((px & 0xf800) >> 11) | ((px & 0x001f) << 11) }
            else { px }
        }).collect();
        Some((320, 240, pixels, self.display_version))
    }
    fn display_version(&self) -> u64 { self.display_version }
    fn display_frames(&self) -> u64 { self.panel.frames }
    fn gram(&self) -> Option<(Vec<u16>, usize, usize)> { Some((self.panel.gram.clone(), 320, 240)) }

    fn accepts_command(&self, verb: &str) -> bool {
        match verb {
            "battery" => true,
            "imu" => self.imu.is_some(),
            "ina" => self.ina.is_some(),
            _ => false,
        }
    }

    fn board_command(&mut self, _cycle: VirtualCycle, verb: &str, args: &[&str]) -> bool {
        match (verb, args, &self.power) {
            ("battery", [bars, charging, full], Power::Basic(pmic)) => {
                let Ok(bars @ 0..=4) = bars.parse::<u8>() else { return false; };
                if !matches!(*charging, "0" | "1") || !matches!(*full, "0" | "1") { return false; }
                pmic.borrow_mut().set_battery(bars, *charging == "1", *full == "1");
            }
            ("battery", [mv, percent, charging], Power::Axp2101(pmic)) => {
                let (Ok(mv), Ok(percent @ 0..=100)) = (mv.parse(), percent.parse::<u8>()) else { return false; };
                if !matches!(*charging, "0" | "1") { return false; }
                pmic.borrow_mut().set_battery(mv, percent, *charging == "1");
            }
            ("battery", [mv, ma], Power::Axp192(pmic)) => {
                let (Ok(mv), Ok(ma)) = (mv.parse(), ma.parse()) else { return false; };
                pmic.borrow_mut().set_battery(mv, ma);
            }
            ("ina", [channel, bus_mv, shunt_uv], _) => {
                let (Ok(channel), Ok(mv), Ok(uv), Some(ina)) = (channel.parse(), bus_mv.parse(), shunt_uv.parse(), &self.ina)
                    else { return false; };
                return ina.borrow_mut().set_sample(channel, mv, uv);
            }
            ("imu", [ax, ay, az, gx, gy, gz], _) => {
                let [Ok(ax), Ok(ay), Ok(az), Ok(gx), Ok(gy), Ok(gz)] = [ax, ay, az, gx, gy, gz].map(|v| v.parse::<f64>()) else { return false; };
                let Some(imu) = &self.imu else { return false; };
                if ![ax, ay, az, gx, gy, gz].iter().all(|v| v.is_finite()) { return false; }
                imu.borrow_mut().set_sample([ax, ay, az, gx, gy, gz]);
            }
            _ => return false,
        }
        true
    }

    fn report(&self) -> String {
        let touch_reads = self.touch.as_ref().map_or(0, |t| t.borrow().reads);
        let p = &self.panel;
        let mut report = format!("[classic-core] {} id_reads={} ramwr={} pixels={} madctl={:#04x} on={} asleep={} light={} sd_transfers={} touch_reads={} motor_changes={} motor_mv={}..{} rgb_updates={}",
                self.name(), self.id_probe, p.frames, p.pixels_written, p.madctl, p.on, p.sleeping,
                if self.kind.basic() { self.light } else { self.rails.light }, self.sd_transactions, touch_reads,
                self.motor_changes, self.motor_min, self.motor_max, self.rgb.updates);
        if !self.kind.basic() {
            let [red, green, blue] = self.rgb.leds[0];
            let uniform = self.rgb.leds.iter().all(|pixel| *pixel == [red, green, blue]);
            report.push_str(&format!(" rgb_pixels={} rgb_first={red:02x}{green:02x}{blue:02x} rgb_uniform={uniform}", self.rgb.leds.len()));
        }
        if let Some(sd) = &self.sd { report.push('\n'); report.push_str(&sd.report()); }
        report
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn read(device: &mut dyn I2cDevice, reg: u8, n: usize) -> Vec<u8> {
        assert!(device.start(false));
        assert!(device.write(reg));
        assert!(device.start(true));
        (0..n).map(|_| device.read()).collect()
    }

    #[test]
    fn core2_revisions_expose_distinct_power_ics_and_the_same_ft6336u() {
        for (kind, id) in [(CoreKind::Core2, 0x4a), (CoreKind::Core2Axp192, 0x03)] {
            let mut board = Core::new(kind);
            let (_, mut pmic) = board.power.port();
            assert_eq!(read(&mut pmic, 3, 1), [id]);
            let mut touch = Port(board.touch.as_ref().unwrap().clone());
            assert_eq!(read(&mut touch, 0xa0, 1), [0x02]);
            assert_eq!(read(&mut touch, 0xa3, 1), [0x64]);
            assert_eq!(read(&mut touch, 2, 1), [0]);
            for x in [63, 160, 258] {
                board.touch(x, 260, true);
                assert_eq!(read(&mut touch, 2, 5), [1, 0x80 | (x >> 8) as u8, x as u8, 1, 4]);
                assert_eq!(board.touch_irq(), Some((39, false)));
            }
            board.touch(0, 0, false);
            assert_eq!(board.touch_irq(), Some((39, true)));
            assert_eq!(read(&mut touch, 2, 1), [0]);
        }
    }

    #[test]
    fn rddid_keeps_a_single_bit_dummy_across_controller_transfers() {
        let mut board = Core::new(CoreKind::Basic);
        let pins = SpiPins { sclk: 1 << CLK, mosi: 1 << DATA, miso: Some(DATA), cs: 1 << 14 };
        board.spi_transfer_pins_bits(3, pins, &[0x04], 0);
        board.panel.dc = true;
        assert_eq!(board.spi_transfer_pins_bits(3, pins, &[], 1), [0]);
        assert_eq!(board.spi_transfer_pins_bits(3, pins, &[], 8), [0xe3]);
        assert_eq!(board.spi_transfer_pins_bits(3, pins, &[], 16), [0, 0]);
        let unselected = SpiPins { cs: 0, ..pins };
        assert_eq!(board.spi_transfer_pins_bits(3, unselected, &[], 8), [0xff]);
        board.panel.dc = false;
        board.spi_transfer_pins_bits(3, pins, &[0xd9], 0);
        board.panel.dc = true;
        board.spi_transfer_pins_bits(3, pins, &[0x12], 0);
        board.panel.dc = false;
        board.spi_transfer_pins_bits(3, pins, &[0xd3], 0);
        board.panel.dc = true;
        assert_eq!(board.spi_transfer_pins_bits(3, pins, &[], 16), [0x93, 0x42]);
    }
}
