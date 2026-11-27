//! M5Stack Dial K130 routing from M5GFX 0.3.0 and M5Dial 1.0.3.
use std::{cell::RefCell, rc::Rc};

use esp_periph::i2c::I2cDevice;
use esp_soc::board::{BoardEdge, BoardModel, SpiPins, VirtualCycle};
use esp_soc::devices::{Bm8563, DcsPanel, Ft5x06, Ft5x06Points, BM8563_ADDRESS, FT5X06_ADDRESS};
use esp_soc::devices::mfrc522::Mfrc522;
use esp_soc::pins::PinRoutes;

use super::i2c_wiring::I2cWiring;
use super::spi_lcd::{LcdPins, SpiLcd};
use crate::periph::CPU_HZ;

pub struct Dial {
    lcd: SpiLcd,
    bus: I2cWiring,
    touch: Ft5x06Points,
    rtc: Rc<RefCell<Bm8563>>,
    rfid: Rc<RefCell<Mfrc522>>,
    held: bool,
    powered: bool,
    wakeups: u64,
    buzzer: Option<(f64, u32)>,
}

impl Dial {
    pub fn new() -> Self {
        let touch = Ft5x06Points::default();
        let rtc = Rc::new(RefCell::new(Bm8563::new(CPU_HZ)));
        let rfid = Rc::new(RefCell::new(Mfrc522::default()));
        let bus = I2cWiring::new((11, 12), vec![
            (FT5X06_ADDRESS, Rc::new(RefCell::new(Ft5x06::new(touch.clone())))),
            (BM8563_ADDRESS, rtc.clone()),
            (Mfrc522::ADDRESS, rfid.clone()),
        ]);
        let pins = LcdPins { clk: 6, data: 5, dc: 4, cs: 7, rst: 8 };
        Self { lcd: SpiLcd::new(pins, DcsPanel::new(240, 240), [0, 0x9a, 1]), bus, touch, rtc, rfid,
            held: false, powered: true, wakeups: 0, buzzer: None }
    }
}
impl Default for Dial { fn default() -> Self { Self::new() } }

impl BoardModel for Dial {
    fn name(&self) -> &'static str { "m5stack-dial" }
    fn observes_gpio_pads(&self) -> bool { true }
    fn gpio_waveform_at(&mut self, cycle: VirtualCycle, routes: PinRoutes<'_>) {
        let was_reset = self.lcd.lcd_reset;
        self.lcd.routes(cycle, &routes);
        if !was_reset && self.lcd.lcd_reset { self.touch.lock().unwrap().fill(None); }
        self.bus.sync(cycle, &routes);
        if let Some(hold) = routes.software_pad_output(46) {
            if self.held && !hold && !self.rtc.borrow().irq_low() { self.powered = false; }
            self.held = hold;
        }
    }
    fn observes_ledc(&self) -> bool { true }
    fn ledc_output_at(&mut self, _: VirtualCycle, pwm: &dyn Fn(u8) -> Option<(f64, u32)>) {
        self.lcd.set_backlight(pwm(9).is_some_and(|(_, duty)| duty != 0));
        self.buzzer = pwm(3);
    }
    fn i2s_buzzer_pin(&self) -> Option<u8> { Some(3) }
    fn uses_spi_pins(&self) -> bool { true }
    fn spi_transfer_pins(&mut self, host: u8, pins: SpiPins, tx: &[u8], rx_len: usize) -> Vec<u8> {
        self.lcd.transfer(host, pins, tx, rx_len)
    }
    fn i2c_devices(&mut self) -> Vec<(u8, u8, Box<dyn I2cDevice>)> { self.bus.ports() }
    fn encoder(&self) -> Option<(u8, u8)> { Some((41, 40)) }
    fn named_pin(&self, name: &str) -> Option<u8> {
        match name { "button" | "a" | "btnA" | "btn_a" | "sw" => Some(42), "boot" | "g0" => Some(0), _ => None }
    }
    fn touch(&mut self, x: u16, y: u16, down: bool) {
        self.touch.lock().unwrap()[0] = (down && !self.lcd.lcd_reset && self.powered).then_some((x.min(239), y.min(239)));
    }
    fn touch_irq(&self) -> Option<(u8, bool)> { Some((14, self.touch.lock().unwrap()[0].is_none())) }
    fn input_levels(&self) -> Vec<(u8, bool)> {
        [(0, true), (40, true), (41, true), (42, true)].into_iter().chain(self.touch_irq())
            .chain(self.bus.levels()).collect()
    }
    fn take_edges(&mut self) -> Vec<BoardEdge> {
        let mut edges = self.lcd.edges();
        edges.extend(self.bus.edges());
        edges
    }
    fn released_inputs(&mut self) -> Vec<u8> { self.lcd.released().into_iter().collect() }
    fn soc_powered(&self) -> Option<bool> { Some(self.powered) }
    fn next_deadline(&self) -> Option<VirtualCycle> { self.rtc.borrow().next_deadline() }
    fn advance_to(&mut self, cycle: VirtualCycle) {
        let mut rtc = self.rtc.borrow_mut();
        rtc.advance_to(cycle);
        if !self.powered && rtc.irq_low() { self.powered = true; self.wakeups += 1; }
    }
    fn accepts_command(&self, verb: &str) -> bool { verb == "rfid" }
    fn board_command(&mut self, _: VirtualCycle, verb: &str, args: &[&str]) -> bool {
        let ("rfid", [value]) = (verb, args) else { return false };
        let uid = if *value == "none" { None } else {
            if value.len() != 8 || !value.bytes().all(|b| b.is_ascii_hexdigit()) { return false; }
            let Ok(uid) = u32::from_str_radix(value, 16) else { return false };
            Some(uid.to_be_bytes())
        };
        self.rfid.borrow_mut().present(uid);
        true
    }
    fn display(&self) -> Option<(u32, u32, Vec<u16>, u64)> {
        let pixels = if self.lcd.visible() && self.powered {
            self.lcd.lcd.gram.iter().map(|&pixel| if self.lcd.lcd.inverted { pixel } else { !pixel }).collect()
        } else { vec![0; 240 * 240] };
        Some((240, 240, pixels, self.display_version()))
    }
    fn display_version(&self) -> u64 { self.lcd.version() }
    fn display_frames(&self) -> u64 { self.lcd.lcd.frames }
    fn report(&self) -> String {
        format!("[dial] RAMWR={} pixels={} rfid_exchanges={} wakeups={} powered={} buzzer_pwm={:?} gpio_i2c={}",
            self.display_frames(), self.lcd.lcd.pixels_written, self.rfid.borrow().exchanges,
            self.wakeups, self.powered, self.buzzer, self.bus.transactions())
    }
}
