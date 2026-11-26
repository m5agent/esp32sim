//! Goodix GT911 I²C touch controller and shared pointer state.
use esp_periph::i2c::I2cDevice;
use std::{cell::Cell, rc::Rc};

/// Touch state shared between a board (UI) and its touch controller.
#[derive(Default, Debug, Clone, Copy)]
pub struct TouchState { pub down: bool, pub x: u16, pub y: u16, pub seen: bool, pub release_pending: bool }
impl TouchState {
    pub fn update(&mut self, x: u16, y: u16, down: bool) {
        self.x = x; self.y = y;
        if down { self.down = true; self.seen = false; self.release_pending = false; }
        else if self.seen { self.down = false; }
        else { self.release_pending = true; }
    }
    /// Keep even a short tap readable until the guest has observed the press once.
    pub fn observe(&mut self) {
        if self.release_pending && self.seen { self.down = false; self.release_pending = false; }
        if self.down { self.seen = true; }
    }
}

/// Goodix GT911 capacitive touch controller: 16-bit register addresses; product ID at 0x8140,
/// config at 0x8047.., status + up to 5 points at 0x814E...
pub struct Gt911 { addr: u16, phase: u8, touch: std::sync::Arc<std::sync::Mutex<TouchState>>, pub reads: u64, w: u16, h: u16, pending: Option<Rc<Cell<bool>>> }
impl Gt911 {
    pub fn new(touch: std::sync::Arc<std::sync::Mutex<TouchState>>, w: u16, h: u16) -> Self { Gt911 { addr: 0, phase: 0, touch, reads: 0, w, h, pending: None } }
    /// The optional pin gate keeps INT asserted through the zero-contact
    /// report. Polling-only boards can leave the signal disconnected.
    pub fn with_pending(mut self, pending: Rc<Cell<bool>>) -> Self { self.pending = Some(pending); self }
    fn reg(&self, a: u16) -> u8 {
        let mut tl = self.touch.lock().unwrap();
        if a == 0x814e {
            tl.observe();
            if let Some(pending) = self.pending.as_ref().filter(|_| !tl.down) { pending.set(false); }
        }
        let t = *tl;
        match a {
            0x8140 => b'9', 0x8141 => b'1', 0x8142 => b'1', 0x8143 => 0, 0x8144 => 0x60, 0x8145 => 0x10,      // "911", firmware 0x1060
            0x8047 => 0x41,                                                                                     // config version
            0x8048 => self.w as u8, 0x8049 => (self.w >> 8) as u8, 0x804a => self.h as u8, 0x804b => (self.h >> 8) as u8,
            0x804c => 5,                                                                                        // touch number
            0x814e => 0x80 | t.down as u8,                                                                      // buffer ready + count
            0x814f => 0, 0x8150 => t.x as u8, 0x8151 => (t.x >> 8) as u8, 0x8152 => t.y as u8, 0x8153 => (t.y >> 8) as u8, 0x8154 => 20, 0x8155 => 0, 0x8156 => 0,
            _ => 0,
        }
    }
}
impl I2cDevice for Gt911 {
    fn start(&mut self, read: bool) -> bool { if !read { self.phase = 0; } true }
    fn write(&mut self, b: u8) -> bool { match self.phase { 0 => { self.addr = (b as u16) << 8; self.phase = 1; } 1 => { self.addr |= b as u16; self.phase = 2; } _ => { self.addr = self.addr.wrapping_add(1); } } true }
    fn read(&mut self) -> u8 { let v = self.reg(self.addr); self.addr = self.addr.wrapping_add(1); self.reads += 1; v }
}
