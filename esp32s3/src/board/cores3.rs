//! M5Stack CoreS3 (K128): ESP32-S3 with 16 MB flash and 8 MB PSRAM, an ILI9342C 320x240 IPS
//! panel on SPI2 and a shared internal I2C bus carrying the power, IO, touch and sensor chips.
//!
//! Wiring, from M5GFX 0.3.0 `board_detect/m5/generated/esp32s3_board_data.hpp` (namespace
//! `cores3`) and M5Unified 0.3.0 `M5Unified.inl`:
//!
//! - LCD: SCLK 36, MOSI 37, CS 3 (software GPIO), D/C 35. GPIO 35 doubles as MISO: M5GFX's
//!   `Panel_M5StackCoreS3_T::cs_control` routes it to the GPIO output (D/C) while CS is low and
//!   releases it to FSPIQ while CS is high. LCD reset is AW9523 P1_1; backlight is AXP2101 DLDO1.
//! - microSD shares SCLK/MOSI/MISO with CS 4, powered by AXP2101 ALDO4. TF_SW -> AW9523 P0_4.
//! - Internal I2C: SDA 12, SCL 11 (M5GFX probes it on I2C_NUM_1; M5Unified's `In_I2C` may take
//!   either controller, so the devices answer on both when routed to these pins).
//!   0x34 AXP2101, 0x58 AW9523B, 0x38 FT6336U (FT5x06 register set), 0x69 BMI270 (+BMM150 on its
//!   AUX bus), 0x51 BM8563, 0x23 LTR553, 0x36 AW88298, 0x40 ES7210, 0x21 GC0308 (SCCB).
//! - Touch INT -> AW9523 P1_2 -> AW9523 INTN -> GPIO 21 (active low). M5GFX only reads the touch
//!   controller while GPIO 21 is low and clears INTN by reading AW9523 input registers 0x00/0x01.
use super::{soft_i2c::{SoftI2c, SoftOp}, stackchan::{StackChan, IR_RX, IR_TX}};
use std::cell::RefCell;
use std::rc::Rc;
use std::sync::Arc;

use esp_periph::i2c::I2cDevice;
use esp_periph::sdmmc::{SdCard, SpiSdCard};
use esp_soc::board::{BoardEdge, BoardModel, I2sCodecs, SpiPins, VirtualCycle};
use esp_soc::camera::Gc0308;
use esp_soc::picture::Picture;
use esp_soc::devices::aw9523::Aw9523;
use esp_soc::devices::{aw88298::Aw88298, es7210::Es7210};
use esp_soc::devices::{Axp2101, Bm8563, Bmi270, Bmm150, DcsPanel, Ft5x06, Ft5x06Points, Ltr553, AXP2101_ADDRESS, BM8563_ADDRESS, BMI270_ADDRESS, FT5X06_ADDRESS, LTR553_ADDRESS};
use esp_soc::devices::imu_motion::ImuMotion;

pub const PIN_CORES3_I2C_SDA: u8 = 12;
pub const PIN_CORES3_I2C_SCL: u8 = 11;
pub const PIN_CORES3_LCD_CS: u8 = 3;
pub const PIN_CORES3_LCD_DC: u8 = 35;
pub const PIN_CORES3_SD_CS: u8 = 4;
pub const PIN_CORES3_I2C_INT: u8 = 21;
const PIN_SPI_SCLK: u8 = 36;
const PIN_SPI_MOSI: u8 = 37;

pub const CORES3_LCD_W: usize = 320;
pub const CORES3_LCD_H: usize = 240;

// ILI9342C V1.02, §8.2.3 (RDDIDIF). M5GFX 9bcedf4e's CoreS3 probe clocks the
// serial dummy bit separately, then checks ID1 == 0xe3; ID2 and ID3 are unused.
const LCD_RDDID: u8 = 0x04;
const LCD_ID: [u8; 3] = [0xe3, 0, 0];

const AW_TF_SW: u16 = 1 << 4;
const AW_LCD_RST: u16 = 1 << 9;
const AW_TOUCH_INT: u16 = 1 << 10;

/// Everything on the internal bus plus the panel, behind one lock so that the expander's
/// LCD reset and touch interrupt act on the devices they are wired to.
pub struct Cores3Hw {
    pub panel: DcsPanel,
    pub aw9523: Aw9523,
    pub touch: Ft5x06,
    camera: Gc0308,
    pub pmic: Axp2101,
    pub rtc: Bm8563,
    backlight: bool,
    pub points: Ft5x06Points,
    bmi270: Bmi270,
    ltr553: Ltr553,
    amp: Aw88298,
    mic: Es7210,
    pub others: Vec<(u8, Box<dyn I2cDevice>)>,
    sd: Option<Box<SpiSdCard>>,
    lcd_in_reset: bool,
    lcd_id_offset: Option<usize>,
    pub panel_version: u64,
}

impl Cores3Hw {
    fn sync(&mut self) {
        // CoreS3 K128 schematic v1.0: AW_RST is driven by AW9523 P0_2.
        self.amp.set_reset_released(self.aw9523.outputs() & !self.aw9523.directions() & (1 << 2) != 0);
        let backlight = self.pmic.backlight_on();
        if backlight != self.backlight { self.panel_version += 1; self.backlight = backlight; }
        let in_reset = ((self.aw9523.outputs() | self.aw9523.directions()) & AW_LCD_RST) == 0;
        if in_reset && !self.lcd_in_reset {
            self.panel.reset(); self.panel_version += 1; self.lcd_id_offset = None;
        }
        self.lcd_in_reset = in_reset;
        let touched = self.points.lock().expect("CoreS3 touch mutex poisoned").iter().any(Option::is_some);
        let ext = self.aw9523.external;
        self.aw9523.external = if touched { ext & !AW_TOUCH_INT } else { ext | AW_TOUCH_INT };
    }
    // Sch_M5_CoreS3_v1.0 sheets 1 and 5 connect ALDO4 to VDD_3V3_SD.
    // M5Unified 0.3.0 Power_Class.inl enables ALDO4 (0x90 bit 3) and sets it to 3.3 V
    // (0x95 = 28) for the TF socket. The voltage register uses 100 mV steps from 500 mV.
    fn sd_powered(&self) -> bool { self.pmic.powered() && self.pmic.reg(0x90) & 0x08 != 0 && (22..=30).contains(&(self.pmic.reg(0x95) & 0x1f)) }

    fn write_i2c(&mut self, address: u8, data: u8) -> bool {
        if address == AXP2101_ADDRESS && self.sd.is_some() {
            let powered = self.sd_powered();
            let ack = self.pmic.write(data);
            if powered && !self.sd_powered() {
                if let Some(sd) = &mut self.sd { sd.power_off(); }
            }
            return ack;
        }
        self.device(address).is_some_and(|device| device.write(data))
    }

    fn device(&mut self, address: u8) -> Option<&mut dyn I2cDevice> {
        match address {
            Aw9523::ADDRESS => Some(&mut self.aw9523),
            FT5X06_ADDRESS => Some(&mut self.touch),
            BMI270_ADDRESS => Some(&mut self.bmi270),
            LTR553_ADDRESS => Some(&mut self.ltr553),
            Gc0308::ADDRESS => Some(&mut self.camera),
            Aw88298::ADDRESS => Some(&mut self.amp),
            Es7210::ADDRESS => Some(&mut self.mic),
            AXP2101_ADDRESS => Some(&mut self.pmic),
            BM8563_ADDRESS => Some(&mut self.rtc),
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
    fn write(&mut self, b: u8) -> bool {
        let mut hw = self.hw.borrow_mut();
        let ack = hw.write_i2c(self.address, b);
        hw.sync();
        ack
    }
    fn read(&mut self) -> u8 { self.with(|d| d.read()) }
    fn stop(&mut self) { self.with(|d| d.stop()) }
}

pub struct Cores3 {
    pub hw: Rc<RefCell<Cores3Hw>>,
    soft: SoftI2c,
    sda_release: bool,
    pub imu: Arc<ImuMotion>,
    lcd_cs_low: bool,
    sd_cs_low: bool,
    cycle: VirtualCycle,
    int_level: bool,
    powered: bool,
    edges: Vec<BoardEdge>,
    pub gpio_events: u64,
    pub spi_bytes: u64,
    stackchan: Option<StackChan>,
}

impl Default for Cores3 { fn default() -> Self { Self::new() } }

impl Cores3 {
    pub fn new() -> Self {
        let imu: Arc<ImuMotion> = Default::default();
        let points: Ft5x06Points = Default::default();
        let others: Vec<(u8, Box<dyn I2cDevice>)> = vec![
        ];
        let hw = Cores3Hw {
            panel: DcsPanel::new(CORES3_LCD_W, CORES3_LCD_H), aw9523: Aw9523::new(), touch: Ft5x06::new(points.clone()),
            bmi270: Bmi270::new(imu.clone(), crate::periph::CPU_HZ).with_aux_bmm150(Bmm150::new(Default::default())),
            ltr553: Ltr553::new(),
            points, others, camera: Gc0308::default(), amp: Aw88298::new(), mic: Es7210::new(), pmic: Axp2101::new(crate::periph::CPU_HZ), rtc: Bm8563::new(crate::periph::CPU_HZ), backlight: false,
            lcd_in_reset: true, lcd_id_offset: None, panel_version: 0, sd: None,
        };
        Cores3 { hw: Rc::new(RefCell::new(hw)), soft: SoftI2c::idle(), sda_release: false, imu, lcd_cs_low: false, sd_cs_low: false, cycle: 0, int_level: true, powered: true,
                 edges: Vec::new(), gpio_events: 0, spi_bytes: 0, stackchan: None }
    }

    pub fn stackchan() -> Self {
        let mut board = Self::new();
        let base = StackChan::new();
        board.hw.borrow_mut().others.extend(base.ports());
        board.stackchan = Some(base);
        board
    }

    fn addresses(&self) -> Vec<u8> {
        let hw = self.hw.borrow_mut();
        let mut v = vec![Aw9523::ADDRESS, FT5X06_ADDRESS, BMI270_ADDRESS, LTR553_ADDRESS, AXP2101_ADDRESS, BM8563_ADDRESS, Gc0308::ADDRESS, Aw88298::ADDRESS, Es7210::ADDRESS];
        v.extend(hw.others.iter().map(|(a, _)| *a));
        v
    }

    /// Master levels on the internal bus changed: run the slave and update its SDA drive.
    fn soft_i2c(&mut self, scl: bool, sda: bool) {
        let before = self.soft.pull_low;
        self.soft.levels(scl, sda, |op| {
            let mut hw = self.hw.borrow_mut();
            match op {
                SoftOp::Start(addr, read) => hw.device(addr).is_some_and(|d| d.start(read)) as u8,
                SoftOp::Write(addr, byte) => hw.write_i2c(addr, byte) as u8,
                SoftOp::Read(addr) => hw.device(addr).map_or(0xff, |d| d.read()),
                SoftOp::Stop(addr) => { if let Some(d) = hw.device(addr) { d.stop(); } 0 }
                SoftOp::Sync => { hw.sync(); 0 }
            }
        });
        if self.soft.pull_low != before {
            if self.soft.pull_low { self.edges.push(BoardEdge { cycle: self.cycle, pin: PIN_CORES3_I2C_SDA, level: false }); self.sda_release = false; }
            else { self.sda_release = true; }
        }
    }
    fn update_int(&mut self) {
        let low = self.hw.borrow_mut().aw9523.intn_low();
        if self.int_level == low {
            self.int_level = !low;
            self.edges.push(BoardEdge { cycle: self.cycle, pin: PIN_CORES3_I2C_INT, level: self.int_level });
        }
    }

    /// The fixed landscape glass shows GRAM in chassis coordinates. ILI9342C v1.00 §8.2.29
    /// MX/MY/MV transform writes in DcsPanel; do not rotate the framebuffer again for MV.
    /// Red and blue are exchanged unless MADCTL.BGR is set
    /// (M5GFX sets it), inverted unless INVON (the IPS glass is normally white; M5GFX's CoreS3
    /// panel configuration has `invert = true`), black while asleep, off or in reset.
    pub fn visible(hw: &Cores3Hw) -> Vec<u16> {
        let p = &hw.panel;
        let blank = !p.on || p.sleeping || hw.lcd_in_reset || !hw.pmic.backlight_on();
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
    fn name(&self) -> &'static str {
        if self.stackchan.is_some() { "m5stack-stackchan" } else { "m5stack-cores3" }
    }
    fn uses_uart_pins(&self) -> bool { self.stackchan.is_some() }
    fn uart_tx(&mut self, cycle: u64, route: esp_soc::uart::UartRoute, byte: u8) {
        if let Some(base) = &mut self.stackchan { base.uart_tx(cycle, route, byte); }
    }
    fn uart_rx(&mut self, cycle: u64) -> Vec<esp_soc::uart::UartInput> {
        self.stackchan.as_mut().map_or_else(Vec::new, |base| base.uart_rx(cycle))
    }
    fn observes_ledc(&self) -> bool { self.stackchan.is_some() }
    fn ledc_output_at(&mut self, cycle: u64, pwm: &dyn Fn(u8) -> Option<(f64, u32)>) {
        if let Some(level) = self.stackchan.as_mut().and_then(|base| base.pwm_ir(cycle, pwm(IR_TX))) {
            self.edges.push(BoardEdge { cycle, pin: IR_RX, level });
        }
    }
    fn leds(&self) -> Option<(&[[u8; 3]], u64)> { self.stackchan.as_ref().map(StackChan::leds) }
    fn gpio_changes(&mut self, changes: &[(u8, bool)]) {
        let mut hw = self.hw.borrow_mut();
        for &(pin, level) in changes {
            self.gpio_events += 1;
            match pin {
                PIN_CORES3_LCD_CS => {
                    self.lcd_cs_low = !level;
                    if level { hw.lcd_id_offset = None; }
                }
                PIN_CORES3_LCD_DC => hw.panel.dc = level,
                _ => {}
            }
        }
    }
    fn gpio_output_at(&mut self, cycle: VirtualCycle, changes: &[(u8, bool)], enabled: u64, output: u64) {
        if cycle > self.cycle { self.cycle = cycle; }
        if !changes.is_empty() { self.gpio_changes(changes); }
        let level = |pin: u8| enabled >> pin & 1 == 0 || output >> pin & 1 != 0;
        let (scl, sda) = (level(PIN_CORES3_I2C_SCL), level(PIN_CORES3_I2C_SDA));
        let ir = enabled & (1 << IR_TX) != 0 && output & (1 << IR_TX) != 0;
        if let Some(level) = self.stackchan.as_mut().and_then(|base| base.gpio_ir(cycle, ir)) {
            self.edges.push(BoardEdge { cycle, pin: IR_RX, level });
        }
        let sd_cs_low = !level(PIN_CORES3_SD_CS);
        if self.sd_cs_low && !sd_cs_low {
            if let Some(sd) = &mut self.hw.borrow_mut().sd { sd.deselect(); }
        }
        self.sd_cs_low = sd_cs_low;
        // SDA and SCL never change in the same register write of a bit-banged master; if they do,
        // take SCL first so a data change is not seen as a start or stop.
        if scl != self.soft.scl { let s = self.soft.sda; self.soft_i2c(scl, s); }
        if sda != self.soft.sda { self.soft_i2c(scl, sda); }
    }
    fn released_inputs(&mut self) -> Vec<u8> { if std::mem::take(&mut self.sda_release) { vec![PIN_CORES3_I2C_SDA] } else { Vec::new() } }
    fn gpio_events(&self) -> u64 { self.gpio_events }
    fn spi_transfer(&mut self, host: u8, tx: &[u8], rx_len: usize) -> Vec<u8> {
        if host == 2 && self.lcd_cs_low {
            self.spi_bytes += tx.len() as u64;
            let mut hw = self.hw.borrow_mut();
            hw.sync();
            if !hw.lcd_in_reset {
                if !hw.panel.dc {
                    if let Some(&command) = tx.last() { hw.lcd_id_offset = (command == LCD_RDDID).then_some(0); }
                }
                let before = hw.panel.pixels_written;
                for &b in tx { hw.panel.byte(b); }
                if hw.panel.pixels_written != before { hw.panel_version += 1; }
                if hw.panel.dc && rx_len != 0 {
                    if let Some(offset) = &mut hw.lcd_id_offset {
                        let mut rx = vec![0; rx_len];
                        let rest = &LCD_ID[*offset..];
                        let n = rest.len().min(rx_len);
                        rx[..n].copy_from_slice(&rest[..n]);
                        *offset += n;
                        return rx;
                    }
                }
            }
        }
        vec![0xff; rx_len]
    }
    fn uses_spi_pins(&self) -> bool { self.hw.borrow().sd.is_some() }
    fn has_spi_sd_slot(&self) -> bool { true }
    fn reset_spi_sd_observation(&mut self) {
        if let Some(card) = self.hw.borrow_mut().sd.as_deref_mut() { card.reset_observation(); }
    }
    fn spi_sd_state(&self) -> Option<esp_periph::sdmmc::SdMediaState> {
        self.hw.borrow().sd.as_deref().map(SpiSdCard::media_state)
    }
    fn set_spi_sd_present(&mut self, _cycle: VirtualCycle, present: bool) -> bool {
        let mut hw = self.hw.borrow_mut();
        let Some(card) = hw.sd.as_deref_mut() else { return false };
        card.set_present(present);
        if present { hw.aw9523.external &= !AW_TF_SW; }
        else { hw.aw9523.external |= AW_TF_SW; }
        true
    }
    fn insert_spi_sd(&mut self, card: SdCard) {
        let mut hw = self.hw.borrow_mut();
        hw.sd = Some(Box::new(SpiSdCard::new(card)));
        // TF_SW has a pull-up and closes to ground on insertion (schematic sheet 5).
        hw.aw9523.external &= !AW_TF_SW;
    }
    fn spi_transfer_pins(&mut self, host: u8, pins: SpiPins, tx: &[u8], rx_len: usize) -> Vec<u8> {
        if pins.sclk & (1 << PIN_SPI_SCLK) == 0 { return vec![0xff; rx_len]; }
        let has_mosi = pins.mosi & (1 << PIN_SPI_MOSI) != 0;
        let lcd_selected = pins.cs & (1 << PIN_CORES3_LCD_CS) != 0;
        if lcd_selected && has_mosi { self.spi_transfer(host, tx, 0); }
        let mut rx = vec![0xff; rx_len];
        let mut hw = self.hw.borrow_mut();
        let powered = hw.sd_powered();
        if let Some(sd) = &mut hw.sd {
            if pins.cs & (1 << PIN_CORES3_SD_CS) == 0 { sd.deselect(); }
            else if powered {
                let miso = pins.miso == Some(PIN_CORES3_LCD_DC) && !lcd_selected;
                let now_ns = (u128::from(self.cycle) * 1_000_000_000 / u128::from(crate::periph::CPU_HZ)).min(u128::from(u64::MAX)) as u64;
                for i in 0..tx.len().max(rx_len) {
                    let mosi = if has_mosi { tx.get(i).copied().unwrap_or(0xff) } else { 0xff };
                    let out = sd.exchange(mosi, now_ns);
                    if miso {
                        if let Some(b) = rx.get_mut(i) { *b = out; }
                    }
                }
            }
        }
        rx
    }
    fn i2c_devices(&mut self) -> Vec<(u8, u8, Box<dyn I2cDevice>)> {
        let mut v: Vec<(u8, u8, Box<dyn I2cDevice>)> = Vec::new();
        for address in self.addresses() {
            for bus in 0..2 { v.push((bus, address, Box::new(Port { hw: self.hw.clone(), address }))); }
        }
        v
    }
    fn i2s_codecs(&self) -> Option<I2sCodecs> {
        let hw = self.hw.borrow();
        // CoreS3 K128 schematic v1.0 and M5Unified 0.3.0 CoreS3 mic_cfg/spk_cfg.
        Some(I2sCodecs { port: 1, mic_port: 1, bck: 34, ws: 33, mck: Some(0), dout: 13, din: 14,
                         speaker_gain: hw.amp.gain(), mic_lanes: hw.mic.lanes() })
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
    fn input_levels(&self) -> Vec<(u8, bool)> {
        let mut levels = vec![(PIN_CORES3_I2C_INT, self.int_level)];
        if self.stackchan.is_some() { levels.push((IR_RX, true)); }
        levels
    }
    fn touch(&mut self, x: u16, y: u16, down: bool) {
        {
            let mut hw = self.hw.borrow_mut();
            hw.points.lock().expect("CoreS3 touch mutex poisoned")[0] =
                down.then_some((x.min(CORES3_LCD_W as u16 - 1), y.min(CORES3_LCD_H as u16 - 1)));
            hw.sync();
        }
        self.update_int();
    }
    fn soc_powered(&self) -> Option<bool> { Some(self.hw.borrow().pmic.powered()) }
    fn next_deadline(&self) -> Option<VirtualCycle> {
        let hw = self.hw.borrow();
        hw.rtc.next_deadline().into_iter().chain(hw.pmic.next_deadline())
            .chain(self.stackchan.as_ref().and_then(StackChan::deadline)).min()
    }
    fn advance_to(&mut self, cycle: VirtualCycle) {
        self.cycle = self.cycle.max(cycle);
        let mut hw = self.hw.borrow_mut();
        if let Some(base) = &mut self.stackchan { base.advance(self.cycle); }
        hw.rtc.advance_to(self.cycle);
        hw.pmic.advance_to(self.cycle);
        // CoreS3 schematic v1.0: BM8563 INT pulls down the AXP_WAKEUP net at AXP2101 IRQ.
        let rtc_irq = hw.rtc.irq_low();
        hw.pmic.set_irq_input(rtc_irq);
        let powered = hw.pmic.powered();
        if self.powered && !powered {
            hw.aw9523.reset();
            hw.panel.reset();
            hw.panel.gram.fill(0);
            let transactions = self.soft.transactions;
            self.soft = SoftI2c::idle();
            self.soft.transactions = transactions;
            self.sda_release = true;
            hw.panel_version += 1;
            hw.sync();
        }
        self.powered = powered;
        drop(hw);
        self.update_int();
    }
    fn accepts_command(&self, verb: &str) -> bool {
        matches!(verb, "pwrkey" | "battery" | "imu" | "als") || self.stackchan.is_some() && StackChan::accepts_command(verb)
    }
    fn board_command(&mut self, cycle: VirtualCycle, verb: &str, args: &[&str]) -> bool {
        self.advance_to(cycle);
        if StackChan::accepts_command(verb) {
            return self.stackchan.as_mut().is_some_and(|base| base.command(verb, args));
        }
        let mut hw = self.hw.borrow_mut();
        let flag = |s: &str| match s { "0" => Some(false), "1" => Some(true), _ => None };
        match (verb, args) {
            ("pwrkey", [ms]) => ms.parse().is_ok_and(|ms| hw.pmic.press_key(ms)),
            ("battery", [mv, percent, charging] | [mv, percent, charging, _]) => {
                let (Ok(mv), Ok(percent), Some(charging)) = (mv.parse(), percent.parse(), flag(charging)) else { return false; };
                if mv > 0x3fff || percent > 100 { return false; }
                if let Some(vbus) = args.get(3) {
                    let Some(vbus) = flag(vbus) else { return false; };
                    hw.pmic.set_vbus_mv(if vbus { 5000 } else { 0 });
                }
                hw.pmic.set_battery(mv, percent, charging);
                true
            }
            ("imu", args) if matches!(args.len(), 6 | 9) => {
                let mut xyz = [0.0; 9];
                for (dst, arg) in xyz.iter_mut().zip(args) {
                    let Ok(value) = arg.parse::<f64>() else { return false; };
                    if !value.is_finite() { return false; }
                    *dst = value;
                }
                let [ax, ay, az, gx, gy, gz, mx, my, mz] = xyz;
                // Script values use board axes. M5Unified 0.3.0 IMU_Class.inl inverts
                // the BMM150's Y/Z on CoreS3; the accel/gyro axes are already aligned.
                let mag = (args.len() == 9).then_some([mx, -my, -mz]);
                hw.bmi270.set_sample([ax, ay, az], [gx, gy, gz], mag);
                true
            }
            ("als", [ch0, ch1, ps]) => {
                let (Ok(ch0), Ok(ch1), Ok(ps)) = (ch0.parse(), ch1.parse(), ps.parse()) else { return false; };
                hw.ltr553.set_sample(ch0, ch1, ps);
                true
            }
            _ => false,
        }
    }
    fn take_edges(&mut self) -> Vec<BoardEdge> { std::mem::take(&mut self.edges) }
    fn set_camera_picture(&mut self, picture: Picture) -> Result<(), &'static str> { self.hw.borrow_mut().camera.set_camera_picture(picture) }
    fn camera_preview(&self, w: u32, h: u32) -> Option<Vec<u8>> { self.hw.borrow().camera.camera_preview(w, h) }
    fn camera_frame(&mut self) -> Option<(u32, u32, Arc<Vec<u8>>)> { self.hw.borrow_mut().camera.camera_frame() }
    // M5GFX 0.3.0 board data and M5CoreS3 examples/Basic/camera/camera.ino.
    // The board supplies XCLK independently (pin_xclk = -1 in the sketch).
    fn camera_dvp_pins(&self) -> Option<[u8; 11]> { Some([39, 40, 41, 42, 15, 16, 48, 47, 45, 38, 46]) }
    fn named_pin(&self, _name: &str) -> Option<u8> { None }
    fn set_service_property(&mut self, path: &str, property: &str, value: &str) -> bool {
        let mut hw = self.hw.borrow_mut();
        match (path, property) {
            ("/board/imu" | "/board/magnetometer", _) => hw.bmi270.set_service_axis(property, value),
            ("/board/pmic", _) => hw.pmic.set_supply_property(property, value),
            ("/board/rtc", "unix-time") => hw.rtc.set_unix_time(value),
            ("/board/ltr553", _) => hw.ltr553.set_service_property(property, value),
            ("/board/aw9523", "input-bits") => match value.parse() {
                Ok(bits) => { hw.aw9523.external = bits; true }
                Err(_) => false,
            },
            _ => self.stackchan.as_mut().is_some_and(|base| base.set_service_property(path, property, value)),
        }
    }

    fn service_state(&self) -> Vec<(&'static str, esp_soc::json::Json)> {
        use esp_soc::json::Json;
        let hw = self.hw.borrow();
        let mut pmic = hw.pmic.supply_state().to_vec();
        pmic.push(("registers", Json::register_bytes((0..=255).map(|register| hw.pmic.reg(register)))));
        let mut nodes = vec![
            ("pmic", Json::Obj(pmic.into_iter().map(|(key, value)| (key.into(), value)).collect())),
            ("rtc", Json::object([("registers", Json::register_bytes(hw.rtc.registers().iter().copied())),
                                 ("unix-time", Json::Int(hw.rtc.unix_time()))])),
            ("imu", hw.bmi270.service_state()),
            ("touch", Json::object([("report_reads", Json::counter(hw.touch.reads))])),
            ("panel/controller", Json::object([("pixels_written", Json::counter(hw.panel.pixels_written)),
                                              ("backlight", Json::Bool(hw.backlight)), ("reset", Json::Bool(hw.lcd_in_reset))])),
        ];
        if let Some(base) = &self.stackchan {
            nodes.extend(base.service_state());
            if let Some(mag) = hw.bmi270.magnetometer_state() { nodes.push(("magnetometer", mag)); }
            nodes.push(("ltr553", hw.ltr553.service_state()));
            nodes.push(("aw9523", Json::object([("input-bits", Json::Int(hw.aw9523.external.into()))])));
        }
        nodes
    }
    fn report(&self) -> String {
        let hw = self.hw.borrow_mut();
        let p = &hw.panel;
        let mut report = format!("[emu] cores3: soft-i2c {} transactions; {} RAMWR, {} pixels, madctl={:#x} colmod={:#x} inverted={} on={} sleeping={} lcd_reset={} bbox={:?}; spi {} bytes; aw9523 out={:#06x} dir={:#06x}; gpio events {}",
                self.soft.transactions, p.frames, p.pixels_written, p.madctl, p.colmod, p.inverted, p.on, p.sleeping, hw.lcd_in_reset, p.bbox(), self.spi_bytes,
                hw.aw9523.outputs(), u16::from_le_bytes([hw.aw9523.regs[4], hw.aw9523.regs[5]]), self.gpio_events);
        report += &format!("\n[emu] cores3 power: powered={} backlight_mv={} off_cause={:#04x} off_cycle={:?} on_cause={:#04x} on_cycle={:?}; rtc control2={:#04x} timer_expiry_cycle={:?}",
                hw.pmic.powered(), hw.pmic.backlight_mv(), hw.pmic.off_cause(), hw.pmic.last_off, hw.pmic.on_cause(), hw.pmic.last_on,
                hw.rtc.control2(), hw.rtc.last_timer_expiry);
        if let Some(sd) = &hw.sd { report += &format!("\n{}; ALDO4 {}", sd.report(), if hw.sd_powered() { "on" } else { "off" }); }
        if let Some(base) = &self.stackchan { report += &format!("\n{}", base.report()); }
        report
    }
}

#[cfg(test)]
mod sd_tests {
    use super::*;
    use esp_periph::sdmmc::MemStore;

    fn power(pmic: &mut dyn I2cDevice, enabled: bool) {
        assert!(pmic.start(false) && pmic.write(0x95) && pmic.write(28));
        pmic.stop();
        assert!(pmic.start(false) && pmic.write(0x90) && pmic.write(u8::from(enabled) << 3));
        pmic.stop();
    }

    #[test]
    fn tf_socket_needs_power_cs_and_routed_pins() {
        let mut board = Cores3::new();
        let pins = SpiPins { sclk: 1 << PIN_SPI_SCLK, mosi: 1 << PIN_SPI_MOSI, miso: Some(PIN_CORES3_LCD_DC), cs: 1 << PIN_CORES3_SD_CS };
        let cmd0 = [0x40, 0, 0, 0, 0, 0x95];
        let exchange = |b: &mut Cores3, pins, tx: &[u8], rx| b.spi_transfer_pins(2, pins, tx, rx);
        assert_eq!(exchange(&mut board, pins, &[], 2), [0xff; 2]);
        assert_ne!(board.hw.borrow().aw9523.inputs() & AW_TF_SW, 0);

        board.insert_spi_sd(SdCard::new(Box::new(MemStore(vec![0; 512 << 10]))).unwrap());
        assert_eq!(board.hw.borrow().aw9523.inputs() & AW_TF_SW, 0);
        let mut pmic = board.i2c_devices().into_iter().find(|&(bus, addr, _)| bus == 0 && addr == AXP2101_ADDRESS).unwrap().2;
        exchange(&mut board, pins, &cmd0, 0);
        assert_eq!(exchange(&mut board, pins, &[], 2), [0xff; 2]);

        power(&mut *pmic, true);
        for unrouted in [SpiPins { cs: 0, ..pins }, SpiPins { sclk: 0, ..pins }, SpiPins { mosi: 0, ..pins }] {
            exchange(&mut board, unrouted, &cmd0, 0);
            assert_eq!(exchange(&mut board, pins, &[], 2), [0xff; 2]);
        }
        exchange(&mut board, SpiPins { miso: None, ..pins }, &cmd0, 0);
        assert_eq!(exchange(&mut board, pins, &[], 2), [0xff, 1]);
        exchange(&mut board, pins, &cmd0, 0);
        assert_eq!(exchange(&mut board, SpiPins { miso: Some(34), ..pins }, &[], 2), [0xff; 2]);
        assert_eq!(exchange(&mut board, pins, &[], 2), [0xff; 2]);

        board.gpio_output_at(1, &[], pins.cs, 0);
        exchange(&mut board, pins, &cmd0[..3], 0);
        board.gpio_output_at(2, &[], pins.cs, pins.cs);
        board.gpio_output_at(3, &[], pins.cs, 0);
        exchange(&mut board, pins, &cmd0, 0);
        assert_eq!(exchange(&mut board, pins, &[], 2), [0xff, 1]);

        power(&mut *pmic, false);
        power(&mut *pmic, true);
        exchange(&mut board, pins, &[0x7a, 0, 0, 0, 0, 1], 0); // CMD58 before CMD0
        assert_eq!(exchange(&mut board, pins, &[], 2), [0xff; 2]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rddid_is_readable_across_spi_phases_until_deselect() {
        let mut board = Cores3::new();
        board.gpio_changes(&[(PIN_CORES3_LCD_CS, false), (PIN_CORES3_LCD_DC, false)]);
        board.spi_transfer(2, &[0x04], 0);
        board.gpio_changes(&[(PIN_CORES3_LCD_DC, true)]);
        assert_eq!(board.spi_transfer(3, &[], 4), [0xff; 4]);
        assert_eq!(board.spi_transfer(2, &[], 1), [0xe3]);
        assert_eq!(board.spi_transfer(2, &[], 3), [0; 3]);
        board.gpio_changes(&[(PIN_CORES3_LCD_CS, true), (PIN_CORES3_LCD_CS, false)]);
        assert_eq!(board.spi_transfer(2, &[], 4), [0xff; 4]);
    }
}
