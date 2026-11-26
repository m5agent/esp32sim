//! M5Stack CoreS3 (K128): ESP32-S3 with 16 MB flash and 8 MB PSRAM, an ILI9342C 320x240 IPS
//! panel on SPI2 and a shared internal I2C bus carrying the power, IO, touch and sensor chips.
//!
//! Wiring, from M5GFX 0.3.0 `board_detect/m5/generated/esp32s3_board_data.hpp` (namespace
//! `cores3`) and M5Unified 0.3.0 `M5Unified.inl`:
//!
//! - LCD: SCLK 36, MOSI 37, CS 3 (software GPIO), D/C 35. GPIO 35 doubles as MISO: M5GFX's
//!   `Panel_M5StackCoreS3_T::cs_control` routes it to the GPIO output (D/C) while CS is low and
//!   releases it to FSPIQ while CS is high. LCD reset is AW9523 P1_1; backlight is AXP2101 DLDO1.
//! - microSD shares SCLK/MOSI/MISO with CS 4.
//! - Internal I2C: SDA 12, SCL 11 (M5GFX probes it on I2C_NUM_1; M5Unified's `In_I2C` may take
//!   either controller, so the devices answer on both when routed to these pins).
//!   0x34 AXP2101, 0x58 AW9523B, 0x38 FT6336U (FT5x06 register set), 0x69 BMI270 (+BMM150 on its
//!   AUX bus), 0x51 BM8563, 0x23 LTR553, 0x36 AW88298, 0x40 ES7210, 0x21 GC0308 (SCCB).
//! - Touch INT -> AW9523 P1_2 -> AW9523 INTN -> GPIO 21 (active low). M5GFX only reads the touch
//!   controller while GPIO 21 is low and clears INTN by reading AW9523 input registers 0x00/0x01.
use std::cell::RefCell;
use std::rc::Rc;
use std::sync::Arc;

use esp_periph::i2c::{I2cDevice, Reg8Device};
use esp_soc::board::{BoardEdge, BoardModel, VirtualCycle};
use esp_soc::devices::{Bmi270, DcsPanel, Ft5x06, Ft5x06Points, FT5X06_ADDRESS};
use esp_soc::devices::imu_motion::ImuMotion;

pub const PIN_CORES3_I2C_SDA: u8 = 12;
pub const PIN_CORES3_I2C_SCL: u8 = 11;
pub const PIN_CORES3_LCD_CS: u8 = 3;
pub const PIN_CORES3_LCD_DC: u8 = 35;
pub const PIN_CORES3_SD_CS: u8 = 4;
pub const PIN_CORES3_I2C_INT: u8 = 21;

pub const CORES3_LCD_W: usize = 320;
pub const CORES3_LCD_H: usize = 240;

/// AW9523B 16-bit IO expander, register map from the AW9523B datasheet (V1.1, section 9):
/// 0x00/0x01 input P0/P1, 0x02/0x03 output, 0x04/0x05 direction (1 = input), 0x06/0x07 interrupt
/// enable (0 = enabled), 0x10 ID (0x23), 0x11 global control, 0x12/0x13 LED mode, 0x7F soft reset.
/// INTN is low while an enabled input differs from the value last read from its input register.
pub struct Aw9523 {
    pub regs: [u8; 256],
    /// levels driven onto pins configured as inputs, by the board (bit 8+n = P1_n)
    pub external: u16,
    /// input values the firmware last read (reading 0x00/0x01 latches a port)
    pub latched: u16,
    ptr: u8,
    first: bool,
}

impl Default for Aw9523 { fn default() -> Self { Self::new() } }

impl Aw9523 {
    pub const ADDRESS: u8 = 0x58;
    pub const P1_LCD_RST: u16 = 1 << 9;
    pub const P1_TOUCH_INT: u16 = 1 << 10;

    pub fn new() -> Self {
        let mut a = Aw9523 { regs: [0; 256], external: 0xffff, latched: 0, ptr: 0, first: true };
        a.reset();
        a
    }
    fn reset(&mut self) {
        self.regs = [0; 256];
        self.regs[0x04] = 0xff;
        self.regs[0x05] = 0xff;
        self.regs[0x10] = 0x23;
        self.regs[0x12] = 0xff;
        self.regs[0x13] = 0xff;
        self.latched = self.inputs();
    }
    fn word(&self, lo: usize) -> u16 { u16::from_le_bytes([self.regs[lo], self.regs[lo + 1]]) }
    /// Output latch (`0x02/0x03`).
    pub fn outputs(&self) -> u16 { self.word(0x02) }
    /// Pin levels: driven outputs, board-driven inputs.
    pub fn inputs(&self) -> u16 {
        let dir = self.word(0x04);
        (self.outputs() & !dir) | (self.external & dir)
    }
    /// INTN asserted (low).
    pub fn intn_low(&self) -> bool {
        let enabled = !self.word(0x06) & self.word(0x04);
        (self.inputs() ^ self.latched) & enabled != 0
    }
    /// Whether the LCD reset line (P1_1, active low) is released.
    pub fn lcd_reset_released(&self) -> bool { self.outputs() & Self::P1_LCD_RST != 0 || self.word(0x04) & Self::P1_LCD_RST != 0 }
}

impl I2cDevice for Aw9523 {
    fn start(&mut self, read: bool) -> bool { if !read { self.first = true; } true }
    fn write(&mut self, b: u8) -> bool {
        if self.first { self.ptr = b; self.first = false; return true; }
        match self.ptr {
            0x00 | 0x01 | 0x10 => {}
            0x7f => if b == 0x00 { self.reset(); },
            p => self.regs[p as usize] = b,
        }
        self.ptr = self.ptr.wrapping_add(1);
        true
    }
    fn read(&mut self) -> u8 {
        let v = match self.ptr {
            0x00 => { let i = self.inputs(); self.latched = (self.latched & 0xff00) | (i & 0x00ff); i as u8 }
            0x01 => { let i = self.inputs(); self.latched = (self.latched & 0x00ff) | (i & 0xff00); (i >> 8) as u8 }
            p => self.regs[p as usize],
        };
        self.ptr = self.ptr.wrapping_add(1);
        v
    }
}

/// Everything on the internal bus plus the panel, behind one lock so that the expander's
/// LCD reset and touch interrupt act on the devices they are wired to.
pub struct Cores3Hw {
    pub panel: DcsPanel,
    pub aw9523: Aw9523,
    pub touch: Ft5x06,
    pub points: Ft5x06Points,
    pub others: Vec<(u8, Box<dyn I2cDevice>)>,
    lcd_in_reset: bool,
    pub panel_version: u64,
}

impl Cores3Hw {
    fn sync(&mut self) {
        let in_reset = !self.aw9523.lcd_reset_released();
        if in_reset && !self.lcd_in_reset { self.panel.reset(); self.panel_version += 1; }
        self.lcd_in_reset = in_reset;
        let touched = self.points.lock().expect("CoreS3 touch mutex poisoned").iter().any(Option::is_some);
        let ext = self.aw9523.external;
        self.aw9523.external = if touched { ext & !Aw9523::P1_TOUCH_INT } else { ext | Aw9523::P1_TOUCH_INT };
    }
    fn device(&mut self, address: u8) -> Option<&mut dyn I2cDevice> {
        match address {
            Aw9523::ADDRESS => Some(&mut self.aw9523),
            FT5X06_ADDRESS => Some(&mut self.touch),
            a => self.others.iter_mut().find(|(o, _)| *o == a).map(|(_, d)| &mut **d as &mut dyn I2cDevice),
        }
    }
}

/// One address on the shared internal bus, attached to a controller with the CoreS3's pins.
struct Port { hw: Rc<RefCell<Cores3Hw>>, address: u8 }

impl Port {
    fn with<R>(&self, f: impl FnOnce(&mut dyn I2cDevice) -> R) -> R {
        let mut hw = self.hw.borrow_mut();
        let r = f(hw.device(self.address).expect("CoreS3 port names an attached device"));
        hw.sync();
        r
    }
}

impl I2cDevice for Port {
    fn pins(&self) -> Option<(u8, u8)> { Some((PIN_CORES3_I2C_SDA, PIN_CORES3_I2C_SCL)) }
    fn start(&mut self, read: bool) -> bool { self.with(|d| d.start(read)) }
    fn write(&mut self, b: u8) -> bool { self.with(|d| d.write(b)) }
    fn read(&mut self) -> u8 { self.with(|d| d.read()) }
    fn stop(&mut self) { self.with(|d| d.stop()) }
}

/// A bit-level slave on the internal bus pins for software (GPIO) I2C masters: M5GFX's board
/// detection bit-bangs SDA 12 / SCL 11 (lgfx `soft_i2c.inl`) before any controller is set up.
/// The lines are open drain with external pull-ups: a master level is low only while the pin is an
/// enabled output with a low latch. The slave pulls SDA low by driving the pad input low and
/// releases it again, so the line the master reads is the wired AND of both. Start and stop are
/// SDA edges while SCL is high; data is sampled on SCL rising edges and changed after falling ones.
#[derive(Default)]
struct SoftI2c { scl: bool, sda: bool, mode: SoftMode, shift: u8, bits: u8, addr: u8, out: u8, master_ack: bool, pull_low: bool, pub transactions: u64 }
#[derive(Default, Clone, Copy, PartialEq, Eq, Debug)]
enum SoftMode { #[default] Idle, Addr, AckSlot { read: bool }, Recv, Send, MasterAck }

pub struct Cores3 {
    pub hw: Rc<RefCell<Cores3Hw>>,
    soft: SoftI2c,
    sda_release: bool,
    pub imu: Arc<ImuMotion>,
    lcd_cs_low: bool,
    sd_cs_low: bool,
    cycle: VirtualCycle,
    int_level: bool,
    edges: Vec<BoardEdge>,
    pub gpio_events: u64,
    pub spi_bytes: u64,
}

impl Default for Cores3 { fn default() -> Self { Self::new() } }

impl Cores3 {
    pub fn new() -> Self {
        let imu: Arc<ImuMotion> = Default::default();
        let points: Ft5x06Points = Default::default();
        let others: Vec<(u8, Box<dyn I2cDevice>)> = vec![
            // AXP2101 chip ID 0x4A at 0x03, as M5GFX's CoreS3 detector requires.
            (0x34, Box::new(Reg8Device::new("axp2101", &[(0x03, 0x4a), (0x00, 0x28)]))),
            (0x69, Box::new(Bmi270::new(imu.clone(), crate::periph::CPU_HZ))),
            (0x51, Box::new(Reg8Device::new("bm8563", &[]))),
            (0x23, Box::new(Reg8Device::new("ltr553", &[(0x86, 0x92), (0x87, 0x05)]))),
            (0x36, Box::new(Reg8Device::new("aw88298", &[]))),
            (0x40, Box::new(Reg8Device::new("es7210", &[(0x3d, 0x72), (0x3e, 0x06)]))),
            (0x21, Box::new(Reg8Device::new("gc0308", &[(0x00, 0x9b)]))),
        ];
        let hw = Cores3Hw {
            panel: DcsPanel::new(CORES3_LCD_W, CORES3_LCD_H), aw9523: Aw9523::new(), touch: Ft5x06::new(points.clone()),
            points, others, lcd_in_reset: true, panel_version: 0,
        };
        Cores3 { hw: Rc::new(RefCell::new(hw)), soft: SoftI2c { scl: true, sda: true, ..Default::default() }, sda_release: false, imu, lcd_cs_low: false, sd_cs_low: false, cycle: 0, int_level: true,
                 edges: Vec::new(), gpio_events: 0, spi_bytes: 0 }
    }

    fn addresses(&self) -> Vec<u8> {
        let hw = self.hw.borrow_mut();
        let mut v = vec![Aw9523::ADDRESS, FT5X06_ADDRESS];
        v.extend(hw.others.iter().map(|(a, _)| *a));
        v
    }

    /// Master levels on the internal bus changed: run the slave and update its SDA drive.
    fn soft_i2c(&mut self, scl: bool, sda: bool) {
        let (old_scl, old_sda) = (self.soft.scl, self.soft.sda);
        self.soft.scl = scl; self.soft.sda = sda;
        let before = self.soft.pull_low;
        if scl && old_scl && sda != old_sda {
            let mut hw = self.hw.borrow_mut();
            if self.soft.mode != SoftMode::Idle { if let Some(d) = hw.device(self.soft.addr) { d.stop(); } }
            hw.sync();
            self.soft.pull_low = false; self.soft.bits = 0; self.soft.shift = 0;
            self.soft.mode = if sda { SoftMode::Idle } else { self.soft.transactions += 1; SoftMode::Addr };
        } else if scl && !old_scl {
            match self.soft.mode {
                SoftMode::Addr | SoftMode::Recv => { self.soft.shift = self.soft.shift << 1 | sda as u8; self.soft.bits += 1; }
                SoftMode::MasterAck => self.soft.master_ack = !sda,
                _ => {}
            }
        } else if !scl && old_scl {
            let mut hw = self.hw.borrow_mut();
            match self.soft.mode {
                SoftMode::Addr if self.soft.bits == 8 => {
                    let (addr, read) = (self.soft.shift >> 1, self.soft.shift & 1 != 0);
                    let ack = hw.device(addr).map(|d| d.start(read)).unwrap_or(false);
                    self.soft.addr = addr; self.soft.pull_low = ack;
                    self.soft.mode = if ack { SoftMode::AckSlot { read } } else { SoftMode::Idle };
                }
                SoftMode::Recv if self.soft.bits == 8 => {
                    let b = self.soft.shift;
                    let ack = hw.device(self.soft.addr).map(|d| d.write(b)).unwrap_or(false);
                    self.soft.pull_low = ack;
                    self.soft.mode = if ack { SoftMode::AckSlot { read: false } } else { SoftMode::Idle };
                }
                SoftMode::AckSlot { read } => {
                    self.soft.bits = 0; self.soft.shift = 0; self.soft.pull_low = false;
                    if read { Self::load_byte(&mut self.soft, &mut hw); } else { self.soft.mode = SoftMode::Recv; }
                }
                SoftMode::Send => {
                    if self.soft.bits < 8 { self.soft.pull_low = self.soft.out & (0x80 >> self.soft.bits) == 0; self.soft.bits += 1; }
                    else { self.soft.pull_low = false; self.soft.mode = SoftMode::MasterAck; }
                }
                SoftMode::MasterAck => {
                    if self.soft.master_ack { Self::load_byte(&mut self.soft, &mut hw); } else { self.soft.pull_low = false; self.soft.mode = SoftMode::Idle; }
                }
                _ => {}
            }
            hw.sync();
        }
        if self.soft.pull_low != before {
            if self.soft.pull_low { self.edges.push(BoardEdge { cycle: self.cycle, pin: PIN_CORES3_I2C_SDA, level: false }); self.sda_release = false; }
            else { self.sda_release = true; }
        }
    }
    fn load_byte(soft: &mut SoftI2c, hw: &mut Cores3Hw) {
        soft.out = hw.device(soft.addr).map(|d| d.read()).unwrap_or(0xff);
        soft.pull_low = soft.out & 0x80 == 0; soft.bits = 1; soft.mode = SoftMode::Send;
    }

    fn update_int(&mut self) {
        let low = self.hw.borrow_mut().aw9523.intn_low();
        if self.int_level == low {
            self.int_level = !low;
            self.edges.push(BoardEdge { cycle: self.cycle, pin: PIN_CORES3_I2C_INT, level: self.int_level });
        }
    }

    /// What the glass shows: the RAM as addressed, red and blue exchanged unless MADCTL.BGR is set
    /// (M5GFX sets it), inverted unless INVON (the IPS glass is normally white; M5GFX's CoreS3
    /// panel configuration has `invert = true`), black while asleep, off or in reset.
    pub fn visible(hw: &Cores3Hw) -> Vec<u16> {
        let p = &hw.panel;
        let blank = !p.on || p.sleeping || hw.lcd_in_reset;
        let exchange = p.madctl & 0x08 == 0;
        p.gram.iter().map(|&px| {
            if blank { return 0; }
            let mut px = if p.inverted { px } else { !px };
            if exchange { px = (px & 0x07e0) | ((px & 0xf800) >> 11) | ((px & 0x001f) << 11); }
            px
        }).collect()
    }
}

impl BoardModel for Cores3 {
    fn name(&self) -> &'static str { "m5stack-cores3" }
    fn gpio_changes(&mut self, changes: &[(u8, bool)]) {
        let mut hw = self.hw.borrow_mut();
        for &(pin, level) in changes {
            self.gpio_events += 1;
            match pin {
                PIN_CORES3_LCD_CS => self.lcd_cs_low = !level,
                PIN_CORES3_LCD_DC => hw.panel.dc = level,
                PIN_CORES3_SD_CS => self.sd_cs_low = !level,
                _ => {}
            }
        }
    }
    fn gpio_output_at(&mut self, cycle: VirtualCycle, changes: &[(u8, bool)], enabled: u64, output: u64) {
        if cycle > self.cycle { self.cycle = cycle; }
        if !changes.is_empty() { self.gpio_changes(changes); }
        let level = |pin: u8| enabled >> pin & 1 == 0 || output >> pin & 1 != 0;
        let (scl, sda) = (level(PIN_CORES3_I2C_SCL), level(PIN_CORES3_I2C_SDA));
        // SDA and SCL never change in the same register write of a bit-banged master; if they do,
        // take SCL first so a data change is not seen as a start or stop.
        if scl != self.soft.scl { let s = self.soft.sda; self.soft_i2c(scl, s); }
        if sda != self.soft.sda { self.soft_i2c(scl, sda); }
    }
    fn released_inputs(&mut self) -> Vec<u8> { if std::mem::take(&mut self.sda_release) { vec![PIN_CORES3_I2C_SDA] } else { Vec::new() } }
    fn gpio_events(&self) -> u64 { self.gpio_events }
    fn spi_transfer(&mut self, host: u8, tx: &[u8], rx_len: usize) -> Vec<u8> {
        if host == 2 && self.lcd_cs_low {
            let mut hw = self.hw.borrow_mut();
            hw.sync();
            if !hw.lcd_in_reset {
                let before = hw.panel.pixels_written;
                for &b in tx { hw.panel.byte(b); }
                if hw.panel.pixels_written != before { hw.panel_version += 1; }
            }
            self.spi_bytes += tx.len() as u64;
        }
        vec![0xff; rx_len]
    }
    fn i2c_devices(&mut self) -> Vec<(u8, u8, Box<dyn I2cDevice>)> {
        let mut v: Vec<(u8, u8, Box<dyn I2cDevice>)> = Vec::new();
        for address in self.addresses() {
            for bus in 0..2 { v.push((bus, address, Box::new(Port { hw: self.hw.clone(), address }))); }
        }
        v
    }
    fn display(&self) -> Option<(u32, u32, Vec<u16>, u64)> {
        let hw = self.hw.borrow_mut();
        Some((CORES3_LCD_W as u32, CORES3_LCD_H as u32, Self::visible(&hw), hw.panel_version))
    }
    fn display_version(&self) -> u64 { self.hw.borrow_mut().panel_version }
    fn display_frames(&self) -> u64 { self.hw.borrow_mut().panel.frames }
    fn gram(&self) -> Option<(Vec<u16>, usize, usize)> {
        let hw = self.hw.borrow_mut();
        Some((hw.panel.gram.clone(), hw.panel.cols, hw.panel.rows))
    }
    fn set_imu_motion(&mut self, mode: u32) -> bool { self.imu.mode.store(mode, std::sync::atomic::Ordering::Relaxed); true }
    fn imu_motion(&self) -> u32 { self.imu.mode.load(std::sync::atomic::Ordering::Relaxed) }
    fn input_levels(&self) -> Vec<(u8, bool)> { vec![(PIN_CORES3_I2C_INT, self.int_level)] }
    fn touch(&mut self, x: u16, y: u16, down: bool) {
        {
            let mut hw = self.hw.borrow_mut();
            hw.points.lock().expect("CoreS3 touch mutex poisoned")[0] =
                down.then_some((x.min(CORES3_LCD_W as u16 - 1), y.min(CORES3_LCD_H as u16 - 1)));
            hw.sync();
        }
        self.update_int();
    }
    fn advance_to(&mut self, cycle: VirtualCycle) {
        if cycle > self.cycle { self.cycle = cycle; }
        self.update_int();
    }
    fn take_edges(&mut self) -> Vec<BoardEdge> { std::mem::take(&mut self.edges) }
    fn named_pin(&self, _name: &str) -> Option<u8> { None }
    fn report(&self) -> String {
        let hw = self.hw.borrow_mut();
        let p = &hw.panel;
        format!("[emu] cores3: soft-i2c {} transactions; {} RAMWR, {} pixels, madctl={:#x} colmod={:#x} inverted={} on={} sleeping={} lcd_reset={} bbox={:?}; spi {} bytes; aw9523 out={:#06x} dir={:#06x}; gpio events {}",
                self.soft.transactions, p.frames, p.pixels_written, p.madctl, p.colmod, p.inverted, p.on, p.sleeping, hw.lcd_in_reset, p.bbox(), self.spi_bytes,
                hw.aw9523.outputs(), u16::from_le_bytes([hw.aw9523.regs[4], hw.aw9523.regs[5]]), self.gpio_events)
    }
}
