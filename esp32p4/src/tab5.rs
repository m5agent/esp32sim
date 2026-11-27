//! M5Stack Tab5 display/touch and internal I²C wiring (M5GFX 0.3.0, board_detect/m5/esp32p4*).
use crate::tab5_aux::Ina226;
use esp_soc::devices::rx8130::Rx8130;
use esp_periph::i2c::{I2cDevice, Reg8Device};
use esp_soc::board::{BoardEdge, I2sCodecs, SpiPins, VirtualCycle};
use esp_soc::devices::es7210::Es7210;
use esp_soc::devices::{Bmi270, BMI270_ADDRESS};
use esp_soc::devices::imu_motion::ImuMotion;
use esp_soc::devices::gt911::{Gt911, TouchState};
use esp_soc::devices::soft_i2c::{SoftI2c, SoftOp};
use esp_soc::BoardModel;
use std::{cell::{Cell, RefCell}, rc::Rc, sync::{Arc, Mutex}};

const SDA: u8 = 31;
const SCL: u8 = 32;
const INT: u8 = 23;
const BACKLIGHT: u8 = 22;
const WIDTH: u32 = 720;
const HEIGHT: u32 = 1280;
type Points = Arc<Mutex<[TouchState; 5]>>;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Panel { Ili9881c, St7123 }

#[derive(Clone)]
struct DevicePort(Rc<RefCell<dyn I2cDevice>>);
impl DevicePort {
    fn new(device: impl I2cDevice + 'static) -> Self { Self(Rc::new(RefCell::new(device))) }
}
impl I2cDevice for DevicePort {
    fn start(&mut self, read: bool) -> bool { self.0.borrow_mut().start(read) }
    fn write(&mut self, b: u8) -> bool { self.0.borrow_mut().write(b) }
    fn read(&mut self) -> u8 { self.0.borrow_mut().read() }
    fn stop(&mut self) { self.0.borrow_mut().stop(); }
}

/// ST7123 registers as consumed by M5GFX 0.3.0 Touch_ST7123.inl. A report is
/// latched at register 0x14 so reads split across the controller FIFO stay coherent.
struct Sitronix { pointer: u16, phase: u8, points: Points, report: [u8; 35] }
impl I2cDevice for Sitronix {
    fn start(&mut self, read: bool) -> bool {
        if !read { self.phase = 0; }
        if read && self.pointer == 0x14 {
            let mut points = self.points.lock().unwrap();
            for (point, data) in points.iter_mut().zip(self.report.as_chunks_mut::<7>().0) {
                point.observe();
                let [xh, xl] = point.x.to_be_bytes();
                let [yh, yl] = point.y.to_be_bytes();
                *data = [u8::from(point.down) << 7 | xh, xl, yh, yl, 20, 20, 0];
            }
        }
        true
    }
    fn write(&mut self, byte: u8) -> bool {
        match self.phase {
            0 => { self.pointer = u16::from(byte) << 8; self.phase = 1; }
            1 => { self.pointer |= u16::from(byte); self.phase = 2; }
            _ => self.pointer = self.pointer.wrapping_add(1),
        }
        true
    }
    fn read(&mut self) -> u8 {
        let p = self.pointer;
        self.pointer = p.wrapping_add(1);
        match p {
            0 => 3,
            5..=8 => [2, 0xcf, 4, 0xff][(p - 5) as usize],
            9 => 5,
            0x10 => 1 << 3, // report includes coordinates
            0x14..=0x36 => self.report[(p - 0x14) as usize],
            _ => 0,
        }
    }
}

pub struct Tab5 {
    panel: Panel,
    screen: Vec<u16>,
    sleeping: bool, on: bool, page: u8,
    scans: u64, version: u64, reads: u64,
    contacts: Points,
    bus: Vec<(u8, DevicePort)>,
    soft: SoftI2c,
    edges: Vec<BoardEdge>,
    released: Vec<u8>,
    touch_irq: bool,
    goodix_pending: Rc<Cell<bool>>,
    backlight: Option<(f64, u32)>,
    mic: Rc<RefCell<Es7210>>,
    dac: Rc<RefCell<Reg8Device>>,
    rails: Rc<RefCell<Reg8Device>>,
    charger: Rc<RefCell<Reg8Device>>,
    rtc: Rc<RefCell<Rx8130>>,
    powered: bool,
    power_off_line: bool,
    rtc_irq: bool,
    off_at: Option<u64>,
    on_at: Option<u64>,
    battery: Rc<RefCell<Ina226>>,
    imu: Rc<RefCell<Bmi270>>,
    motion: Arc<ImuMotion>,
    sd_spi: Option<esp_periph::sdmmc::SpiSdCard>,
}
impl Tab5 {
    pub fn new(panel: Panel) -> Self {
        let contacts = Points::default();
        // The two PI4IOE5V6408s retain direction and output latches across
        // transactions, including when the guest switches from bit-bang to I2C1.
        let pi4io = || Reg8Device::new("pi4ioe5v6408", &[(1, 0xa0), (7, 0xff), (0xb, 0xff)]);
        let rails = Rc::new(RefCell::new(pi4io()));
        let charger = Rc::new(RefCell::new(pi4io()));
        charger.borrow_mut().regs[0x0f] = 1 << 6;
        let rtc = Rc::new(RefCell::new(Rx8130::new(crate::periph::CPU_HZ)));
        let battery = Rc::new(RefCell::new(Ina226::new()));
        let mic = Rc::new(RefCell::new(Es7210::new()));
        // ES8388 control registers used by M5Unified 0.3.0's Tab5 speaker callback.
        // Reset starts with CHIPPOWER/DACPOWER down and DACCONTROL3 muted.
        let dac = Rc::new(RefCell::new(Reg8Device::new("es8388", &[(2, 0xc3), (4, 0xc0), (25, 4)])));
        let mut bus = vec![(0x43, DevicePort(rails.clone())), (0x44, DevicePort(charger.clone())),
                           (Rx8130::ADDRESS, DevicePort(rtc.clone())), (Ina226::ADDRESS, DevicePort(battery.clone())),
                           (Es7210::ADDRESS, DevicePort(mic.clone())), (0x10, DevicePort(dac.clone()))];
        let motion = Arc::<ImuMotion>::default();
        let imu = Rc::new(RefCell::new(Bmi270::new(motion.clone(), crate::periph::CPU_HZ)));
        bus.push((BMI270_ADDRESS, DevicePort(imu.clone())));
        let goodix_pending = Rc::new(Cell::new(false));
        match panel {
            Panel::Ili9881c => bus.push((0x14, DevicePort::new(Gt911::new_multi(contacts.clone(), WIDTH as u16, HEIGHT as u16).with_pending(goodix_pending.clone())))),
            Panel::St7123 => bus.push((0x55, DevicePort::new(Sitronix { pointer: 0, phase: 0, points: contacts.clone(), report: [0; 35] }))),
        }
        Self { panel, screen: Vec::new(), sleeping: true, on: false, page: 0,
               scans: 0, version: 0, reads: 0, contacts, bus, soft: SoftI2c::idle(),
               edges: Vec::new(), released: Vec::new(), touch_irq: true, goodix_pending, backlight: None, mic, dac, rails, charger, rtc, powered: true, power_off_line: false, rtc_irq: false, off_at: None, on_at: None, battery, imu, motion, sd_spi: None }
    }

    fn contact(&mut self, cycle: VirtualCycle, id: usize, x: u16, y: u16, down: bool) -> bool {
        let (x, y) = (x.min(WIDTH as u16 - 1), y.min(HEIGHT as u16 - 1));
        let irq = {
            let mut points = self.contacts.lock().unwrap();
            let Some(point) = points.get_mut(id) else { return false };
            point.update(x, y, down);
            if self.panel == Panel::Ili9881c {
                self.goodix_pending.set(true);
                false
            } else { !points.iter().any(|point| point.down && !point.release_pending) }
        };
        if irq != self.touch_irq {
            self.edges.push(BoardEdge { cycle, pin: INT, level: irq });
            self.touch_irq = irq;
        }
        true
    }

    fn visible(&self) -> bool { self.on && !self.sleeping }
}

impl BoardModel for Tab5 {
    fn name(&self) -> &'static str {
        match self.panel { Panel::Ili9881c => "m5stack-tab5", Panel::St7123 => "m5stack-tab5-st7123" }
    }
    fn i2s_codecs(&self) -> Option<I2sCodecs> {
        let rails = self.rails.borrow();
        let dac = self.dac.borrow();
        // Tab5 M5Unified 0.3.0 enables amp through expander 0x43 bit 1 and
        // routes the ES8388 and ES7210 to opposite directions of I2S0.
        // PI4IOE5V6408 DS40583: DIR=1 selects an output, HIZ=1 disconnects it.
        // ES8388 rev. 5.0 §6.1.3: only the DAC power-down bits gate playback.
        let enabled = rails.regs[3] & 2 != 0 && rails.regs[5] & 2 != 0 && rails.regs[7] & 2 == 0 &&
                      dac.regs[2] & 0x55 == 0 && dac.regs[4] & 0xc0 == 0 && dac.regs[4] & 0x3c != 0 &&
                      dac.regs[25] & 4 == 0;
        Some(I2sCodecs { port: 0, mic_port: 0, bck: 27, ws: 29, mck: Some(30), dout: 26, din: 28,
                        speaker_gain: if enabled { 1.0 } else { 0.0 }, mic_lanes: self.mic.borrow().lanes() })
    }
    fn has_spi_sd_slot(&self) -> bool { true }
    fn uses_spi_pins(&self) -> bool { self.sd_spi.is_some() }
    fn insert_spi_sd(&mut self, card: esp_periph::sdmmc::SdCard) {
        self.sd_spi = Some(esp_periph::sdmmc::SpiSdCard::new(card));
    }
    fn spi_transfer_pins(&mut self, host: u8, pins: SpiPins, tx: &[u8], rx_len: usize) -> Vec<u8> {
        // Tab5 Arduino microSD example: D3/CS=42, CLK=43, CMD/MOSI=44, D0/MISO=39.
        let socket = SpiPins { sclk: 1 << 43, mosi: 1 << 44, miso: Some(39), cs: 1 << 42 };
        let time = self.motion.cycle.load(std::sync::atomic::Ordering::Relaxed);
        let ns = (u128::from(time) * 1_000_000_000 / u128::from(crate::periph::CPU_HZ)) as u64;
        self.sd_spi.as_mut().filter(|_| host == 2).and_then(|card| pins.transfer_sd(socket, card, tx, rx_len, ns))
            .unwrap_or_else(|| vec![0xff; rx_len])
    }
    fn observes_gpio_pads(&self) -> bool { true }
    fn gpio_output_at(&mut self, cycle: VirtualCycle, changes: &[(u8, bool)], enabled: u64, output: u64) {
        let high = |pin| enabled & (1u64 << pin) == 0 || output & (1u64 << pin) != 0;
        if changes.contains(&(42, true)) {
            if let Some(sd) = &mut self.sd_spi { sd.deselect(); }
        }
        let (scl, sda) = (high(SCL), high(SDA));
        let before = self.soft.pull_low;
        let mut transport = |op| {
            let addr = match op {
                SoftOp::Start(a, _) | SoftOp::Write(a, _) | SoftOp::Read(a) | SoftOp::Stop(a) => a,
                SoftOp::Sync => return 0,
            };
            let Some((_, port)) = self.bus.iter_mut().find(|(a, _)| *a == addr) else { return 0xff * u8::from(matches!(op, SoftOp::Read(_))); };
            match op {
                SoftOp::Start(_, read) => u8::from(port.start(read)),
                SoftOp::Write(_, b) => u8::from(port.write(b)),
                SoftOp::Read(_) => port.read(),
                SoftOp::Stop(_) => { port.stop(); 0 }
                SoftOp::Sync => 0,
            }
        };
        if scl != self.soft.scl { self.soft.levels(scl, self.soft.sda, &mut transport); }
        if sda != self.soft.sda { self.soft.levels(scl, sda, &mut transport); }
        match (before, self.soft.pull_low) {
            (false, true) => self.edges.push(BoardEdge { cycle, pin: SDA, level: false }),
            (true, false) => self.released.push(SDA),
            _ => {}
        }
    }
    fn i2c_devices(&mut self) -> Vec<(u8, u8, Box<dyn I2cDevice>)> {
        self.soft = SoftI2c::idle();
        self.released.push(SDA);
        self.bus.iter().map(|(addr, device)| (0, *addr, Box::new(device.clone()) as Box<dyn I2cDevice>)).collect()
    }
    fn input_levels(&self) -> Vec<(u8, bool)> { vec![(INT, self.touch_irq)] }
    fn soc_powered(&self) -> Option<bool> { Some(self.powered) }
    fn next_deadline(&self) -> Option<VirtualCycle> {
        if self.powered { None } else { self.rtc.borrow().next_irq_check() }
    }
    fn advance_to(&mut self, cycle: VirtualCycle) {
        self.motion.cycle.store(cycle, std::sync::atomic::Ordering::Relaxed);
        let mut rtc = self.rtc.borrow_mut();
        rtc.advance(cycle);
        let rtc_irq = rtc.irq();
        drop(rtc);
        // Tab5 schematic: IOE 0x44 P4 (PWROFF_PLUSE) and U30 nIRQ feed the
        // standby power supervisor U28 through its nINT_STAT_TRIG network.
        // Model the observed power latch at the line transitions.
        let ioe = self.charger.borrow();
        let power_off = ioe.regs[3] & ioe.regs[5] & !ioe.regs[7] & 0x10 != 0;
        drop(ioe);
        if self.powered && power_off && !self.power_off_line {
            self.powered = false;
            self.off_at = Some(cycle);
            self.sleeping = true;
            self.on = false;
            self.page = 0;
        } else if !self.powered && rtc_irq && !self.rtc_irq {
            self.powered = true;
            self.on_at = Some(cycle);
        }
        self.power_off_line = power_off;
        self.rtc_irq = rtc_irq;
        if self.panel == Panel::Ili9881c && !self.touch_irq && !self.goodix_pending.get() {
            self.touch_irq = true;
            self.edges.push(BoardEdge { cycle, pin: INT, level: true });
        }
    }
    fn take_edges(&mut self) -> Vec<BoardEdge> { std::mem::take(&mut self.edges) }
    fn released_inputs(&mut self) -> Vec<u8> { std::mem::take(&mut self.released) }
    fn touch_at(&mut self, cycle: VirtualCycle, x: u16, y: u16, down: bool) { self.contact(cycle, 0, x, y, down); }
    fn touch(&mut self, x: u16, y: u16, down: bool) { self.touch_at(0, x, y, down); }
    fn accepts_command(&self, verb: &str) -> bool { matches!(verb, "contact" | "imu" | "battery" | "usb-hid" | "usb-msc" | "usb-report" | "usb-key" | "usb-unplug") }
    fn board_command(&mut self, cycle: VirtualCycle, verb: &str, args: &[&str]) -> bool {
        if let ("battery", [mv, ma, charging]) = (verb, args) {
            let (Ok(mv), Ok(ma), "0" | "1") = (mv.parse::<u16>(), ma.parse::<i16>(), *charging) else { return false; };
            if mv > 16000 { return false; }
            self.battery.borrow_mut().supply(mv, ma);
            // M5Unified Tab5: IOE1 P6 reads the IP2326's active-low charging output.
            let mut charger = self.charger.borrow_mut();
            charger.regs[0x0f] = charger.regs[0x0f] & !(1 << 6) | (u8::from(*charging == "0") << 6);
            return true;
        }
        if let ("imu", [ax, ay, az, gx, gy, gz]) = (verb, args) {
            let values = [ax, ay, az, gx, gy, gz].map(|s| s.parse::<f64>().ok().filter(|v| v.is_finite()));
            let [Some(ax), Some(ay), Some(az), Some(gx), Some(gy), Some(gz)] = values else { return false; };
            self.imu.borrow_mut().set_sample([ax, ay, az], [gx, gy, gz], None);
            return true;
        }
        let ("contact", [id, x, y, down]) = (verb, args) else { return false };
        let (Ok(id), Ok(x), Ok(y), Ok(down @ 0..=1)) = (id.parse(), x.parse(), y.parse(), down.parse::<u8>()) else { return false };
        self.contact(cycle, id, x, y, down != 0)
    }
    fn dsi_write(&mut self, channel: u8, dtype: u8, data: &[u8]) {
        if channel != 0 || !matches!(dtype, 0x05 | 0x15 | 0x39 | 0x13 | 0x23 | 0x29) { return; }
        let was_visible = self.visible();
        match data {
            [0xff, 0x98, 0x81, page] if self.panel == Panel::Ili9881c => self.page = *page,
            [0x01] if self.page == 0 => { self.sleeping = true; self.on = false; }
            [0x10] if self.page == 0 => self.sleeping = true,
            [0x11] if self.page == 0 => self.sleeping = false,
            [0x28] if self.page == 0 => self.on = false,
            [0x29] if self.page == 0 => self.on = true,
            _ => {}
        }
        if self.visible() != was_visible { self.version += 1; }
    }
    fn dsi_read(&mut self, channel: u8, _dtype: u8, request: &[u8], len: usize) -> Vec<u8> {
        if channel != 0 { return Vec::new(); }
        self.reads += 1;
        let value = match (self.panel, self.page, request) {
            (Panel::Ili9881c, 1, [0]) => 0x98,
            (Panel::Ili9881c, 1, [1]) => 0x81,
            (Panel::Ili9881c, 1, [2]) => 0x0c,
            (_, 0, [0x0a]) => 0x08 | (u8::from(!self.sleeping) << 4) | (u8::from(self.on) << 2),
            _ => 0,
        };
        let mut response = vec![0; len];
        if let Some(first) = response.first_mut() { *first = value; }
        response
    }
    fn observes_ledc(&self) -> bool { true }
    fn ledc_output_at(&mut self, _: VirtualCycle, pwm: &dyn Fn(u8) -> Option<(f64, u32)>) { self.backlight = pwm(BACKLIGHT); }
    fn dsi_frame(&mut self, w: u32, h: u32, bpp: usize, data: &[u8]) {
        if (w, h) != (WIDTH, HEIGHT) || !matches!(bpp, 2 | 3) || data.len() != (w * h) as usize * bpp { return; }
        let mut changed = self.screen.is_empty();
        self.screen.resize((w * h) as usize, 0);
        for (dst, src) in self.screen.iter_mut().zip(data.chunks_exact(bpp)) {
            let value = if bpp == 2 { u16::from_le_bytes([src[0], src[1]]) }
                        else { (u16::from(src[2] & 0xf8) << 8) | (u16::from(src[1] & 0xfc) << 3) | u16::from(src[0] >> 3) };
            changed |= *dst != value;
            *dst = value;
        }
        self.scans += 1;
        self.version += u64::from(changed);
    }
    fn display(&self) -> Option<(u32, u32, Vec<u16>, u64)> {
        if self.screen.is_empty() { return None; }
        let pixels = if self.visible() { self.screen.clone() } else { vec![0; self.screen.len()] };
        Some((WIDTH, HEIGHT, pixels, self.version))
    }
    fn display_frames(&self) -> u64 { self.scans }
    fn display_version(&self) -> u64 { self.version }
    fn report(&self) -> String {
        let mut display = format!("[emu] {}: DSI scans={} reads={} on={} sleep={} software I2C transactions={} backlight PWM={:?}",
                self.name(), self.scans, self.reads, self.on, self.sleeping, self.soft.transactions, self.backlight);
        if let Some(off) = self.off_at { display += &format!("\n[emu] Tab5 power: off_cycle={off} rtc_on_cycle={:?} powered={}", self.on_at, self.powered); }
        if let Some(sd) = &self.sd_spi { format!("{display}\n{}", sd.report()) } else { display }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use esp_periph::Device;

    #[test]
    fn ili9881c_page_one_reaches_the_host_receive_fifo() {
        let mut dsi = crate::dsi::Dsi::default();
        let mut panel = Tab5::new(Panel::Ili9881c);
        dsi.write(0x04, 1); // power up
        assert_ne!(dsi.read(0x74) & (1 << 4), 0);
        dsi.write(0x70, 0x0181_98ff);
        dsi.write(0x6c, 0x0439);
        dsi.deliver(&mut panel);
        dsi.write(0x6c, 0x0137);
        dsi.deliver(&mut panel);
        for (reg, expected) in [0x98, 0x81, 0x0c].into_iter().enumerate() {
            dsi.write(0x6c, 0x06 | (reg as u32) << 8);
            dsi.deliver(&mut panel);
            assert_eq!(dsi.read(0x74) & (1 << 4), 0);
            assert_eq!(dsi.read(0x70), expected);
            assert_ne!(dsi.read(0x74) & (1 << 4), 0);
        }
    }
}
