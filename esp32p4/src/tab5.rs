//! M5Stack Tab5 display/touch and internal I²C wiring (M5GFX 0.3.0, board_detect/m5/esp32p4*).
use esp_periph::i2c::{I2cDevice, Reg8Device};
use esp_soc::board::{BoardEdge, VirtualCycle};
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
    contacts: Points, goodix: Arc<Mutex<TouchState>>,
    bus: Vec<(u8, DevicePort)>,
    soft: SoftI2c,
    edges: Vec<BoardEdge>,
    released: Vec<u8>,
    touch_irq: bool,
    goodix_pending: Rc<Cell<bool>>,
    backlight: Option<(f64, u32)>,
}
impl Tab5 {
    pub fn new(panel: Panel) -> Self {
        let (contacts, goodix): (Points, Arc<Mutex<TouchState>>) = (Default::default(), Default::default());
        // The two PI4IOE5V6408s retain direction and output latches across
        // transactions, including when the guest switches from bit-bang to I2C1.
        let pi4io = || DevicePort::new(Reg8Device::new("pi4ioe5v6408", &[(1, 0xa0), (3, 0xff)]));
        let mut bus = vec![(0x43, pi4io()), (0x44, pi4io())];
        let goodix_pending = Rc::new(Cell::new(false));
        match panel {
            Panel::Ili9881c => bus.push((0x14, DevicePort::new(Gt911::new(goodix.clone(), WIDTH as u16, HEIGHT as u16).with_pending(goodix_pending.clone())))),
            Panel::St7123 => bus.push((0x55, DevicePort::new(Sitronix { pointer: 0, phase: 0, points: contacts.clone(), report: [0; 35] }))),
        }
        Self { panel, screen: Vec::new(), sleeping: true, on: false, page: 0,
               scans: 0, version: 0, reads: 0, contacts, goodix, bus, soft: SoftI2c::idle(),
               edges: Vec::new(), released: Vec::new(), touch_irq: true, goodix_pending, backlight: None }
    }

    fn contact(&mut self, cycle: VirtualCycle, id: usize, x: u16, y: u16, down: bool) -> bool {
        let (x, y) = (x.min(WIDTH as u16 - 1), y.min(HEIGHT as u16 - 1));
        let irq = {
            let mut points = self.contacts.lock().unwrap();
            let Some(point) = points.get_mut(id) else { return false };
            if self.panel == Panel::Ili9881c && id != 0 { return false; }
            point.update(x, y, down);
            if self.panel == Panel::Ili9881c {
                self.goodix.lock().unwrap().update(x, y, down);
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
    fn observes_gpio_pads(&self) -> bool { true }
    fn gpio_output_at(&mut self, cycle: VirtualCycle, _: &[(u8, bool)], enabled: u64, output: u64) {
        let high = |pin| enabled & (1u64 << pin) == 0 || output & (1u64 << pin) != 0;
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
    fn advance_to(&mut self, cycle: VirtualCycle) {
        if self.panel == Panel::Ili9881c && !self.touch_irq && !self.goodix_pending.get() {
            self.touch_irq = true;
            self.edges.push(BoardEdge { cycle, pin: INT, level: true });
        }
    }
    fn take_edges(&mut self) -> Vec<BoardEdge> { std::mem::take(&mut self.edges) }
    fn released_inputs(&mut self) -> Vec<u8> { std::mem::take(&mut self.released) }
    fn touch_at(&mut self, cycle: VirtualCycle, x: u16, y: u16, down: bool) { self.contact(cycle, 0, x, y, down); }
    fn touch(&mut self, x: u16, y: u16, down: bool) { self.touch_at(0, x, y, down); }
    fn accepts_command(&self, verb: &str) -> bool { matches!(verb, "contact" | "usb-hid" | "usb-msc" | "usb-report" | "usb-key" | "usb-unplug") }
    fn board_command(&mut self, cycle: VirtualCycle, verb: &str, args: &[&str]) -> bool {
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
        format!("[emu] {}: DSI scans={} reads={} on={} sleep={} software I2C transactions={} backlight PWM={:?}",
                self.name(), self.scans, self.reads, self.on, self.sleeping, self.soft.transactions, self.backlight)
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
