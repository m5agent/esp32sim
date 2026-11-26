//! M5Stack Cardputer-ADV (ESP32-S3FN8, 8 MiB flash), with ST7789V2 and TCA8418.
use std::cell::RefCell;
use std::rc::Rc;
use std::sync::{Arc, atomic::Ordering};

use esp_periph::i2c::I2cDevice;
use esp_periph::sdmmc::{SdCard, SpiSdCard};
use esp_soc::board::{BoardEdge, BoardModel, I2sCodecs, SpiPins, VirtualCycle};
use esp_soc::pins::PinRoutes;
use esp_soc::devices::{Bmi270, DcsPanel, SpiBitBang, BMI270_ADDRESS, es8311::Es8311, imu_motion::ImuMotion, tca8418::Tca8418};

use super::cardputer_adv_ir::NecReceiver;
use super::soft_i2c::{SoftI2c, SoftOp};
use crate::periph::CPU_HZ;

// M5GFX 0.3.0 board_detect/m5/generated/esp32s3_board_data.hpp, cardputer_adv.
const LCD_CLK: u8 = 36;
const LCD_DATA: u8 = 35;
const LCD_DC: u8 = 34;
const LCD_CS: u8 = 37;
const LCD_RST: u8 = 33;
const LCD_BL: u8 = 38;
pub(super) const SDA: u8 = 8;
pub(super) const SCL: u8 = 9;
const LCD_X: usize = 52;
const LCD_Y: usize = 40;
const WIDTH: usize = 240;
const HEIGHT: usize = 135;

// M5Cardputer KeyboardReader/TCA8418.cpp; M5Unified 0.3.0 M5Unified.inl.
const KEY_INT: u8 = 11;
const BUTTON: u8 = 0;
const IR: u8 = 44;

// Arduino-ESP32 3.2.5 variants/m5stack_cardputer/pins_arduino.h and the SD docs sketch.
const TF_CS: u8 = 12;
const TF: SpiPins = SpiPins { sclk: 1 << 40, mosi: 1 << 14, miso: Some(39), cs: 1 << TF_CS };

enum InternalPort {
    Keyboard(Rc<RefCell<Tca8418>>),
    Imu(Rc<RefCell<Bmi270>>),
    Codec(Es8311),
}

impl InternalPort {
    fn with<T>(&mut self, f: impl FnOnce(&mut dyn I2cDevice) -> T) -> T {
        match self {
            Self::Keyboard(device) => f(&mut *device.borrow_mut()),
            Self::Imu(device) => f(&mut *device.borrow_mut()),
            Self::Codec(device) => f(device),
        }
    }
}

impl I2cDevice for InternalPort {
    fn pins(&self) -> Option<(u8, u8)> { Some((SDA, SCL)) }
    fn start(&mut self, read: bool) -> bool { self.with(|d| d.start(read)) }
    fn write(&mut self, byte: u8) -> bool { self.with(|d| d.write(byte)) }
    fn read(&mut self) -> u8 { self.with(|d| d.read()) }
    fn stop(&mut self) { self.with(|d| d.stop()); }
}

pub struct CardputerAdv {
    lcd: DcsPanel,
    lcd_gpio: SpiBitBang,
    lcd_cmd: u8,
    lcd_read_bit: usize,
    lcd_reset: bool,
    lcd_selected: bool,
    lcd_clk: bool,
    lcd_bl: bool,
    lcd_version: u64,
    lcd_gpio_reads: u64,
    lcd_spi_bytes: u64,
    lcd_spi2_bytes: u64,
    keyboard: Rc<RefCell<Tca8418>>,
    key_up: Vec<(u8, u8, VirtualCycle)>,
    imu: Rc<RefCell<Bmi270>>,
    motion: Arc<ImuMotion>,
    codec: Es8311,
    soft_codec: Es8311,
    sd: Option<Box<SpiSdCard>>,
    sd_cs_low: bool,
    key_int_high: bool,
    soft: SoftI2c,
    soft_acks: u64,
    sda_drive: Option<bool>,
    scl_drive: Option<bool>,
    lcd_drive: Option<bool>,
    released: u64,
    cycle: VirtualCycle,
    edges: Vec<BoardEdge>,
    ir: NecReceiver,
    ir_pwm: bool,
}

impl Default for CardputerAdv { fn default() -> Self { Self::new() } }

impl CardputerAdv {
    pub fn new() -> Self {
        let motion = Arc::new(ImuMotion::default());
        let codec = Es8311::new();
        let soft_codec = codec.i2c_port();
        Self { lcd: DcsPanel::st7789(), lcd_gpio: SpiBitBang::new(LCD_CLK, LCD_DATA, LCD_CS),
               lcd_cmd: 0, lcd_read_bit: 0, lcd_reset: false, lcd_selected: false, lcd_clk: false,
               lcd_bl: false, lcd_version: 0, lcd_gpio_reads: 0, lcd_spi_bytes: 0, lcd_spi2_bytes: 0,
               keyboard: Default::default(), key_up: Vec::new(), key_int_high: true,
               imu: Rc::new(RefCell::new(Bmi270::new(motion.clone(), CPU_HZ))), motion,
               codec, soft_codec, sd: None, sd_cs_low: false,
               soft: SoftI2c::idle(), soft_acks: 0, sda_drive: Some(true), scl_drive: Some(true),
               lcd_drive: None, released: 0, cycle: 0, edges: Vec::new(), ir: Default::default(), ir_pwm: false }
    }

    fn lcd_byte(&mut self, byte: u8) {
        if self.lcd_reset { return; }
        if !self.lcd.dc { self.lcd_cmd = byte; self.lcd_read_bit = 0; self.lcd_version += 1; }
        self.lcd.byte(byte);
    }

    fn lcd_id(&self) -> &[u8] {
        // Sitronix ST7789V2 v1.0 §9.1.3: RDDID bytes are manufacturer, version, driver.
        match self.lcd_cmd { 0x04 => &[0x85, 0x85, 0x52], 0xda | 0xdb => &[0x85], 0xdc => &[0x52], _ => &[] }
    }

    fn drive(cycle: VirtualCycle, edges: &mut Vec<BoardEdge>, released: &mut u64, pin: u8,
             current: &mut Option<bool>, value: Option<bool>) {
        if *current == value { return; }
        *current = value;
        if let Some(level) = value {
            *released &= !(1 << pin);
            edges.push(BoardEdge { cycle, pin, level });
        } else {
            *released |= 1 << pin;
        }
    }

    fn gpio_lcd(&mut self, enabled: u64, output: u64) {
        let drives = |pin: u8| enabled & (1 << pin) != 0;
        let high = |pin: u8| output & (1 << pin) != 0;
        let selected = drives(LCD_CS) && !high(LCD_CS);
        let reset = drives(LCD_RST) && !high(LCD_RST);
        let clk = drives(LCD_CLK) && high(LCD_CLK);
        if reset && !self.lcd_reset { self.lcd.reset(); self.lcd_cmd = 0; self.lcd_version += 1; }
        self.lcd_reset = reset;
        self.lcd.dc = high(LCD_DC);
        self.lcd_gpio.pin(LCD_CS, !selected);
        self.lcd_gpio.pin(LCD_DATA, high(LCD_DATA));
        let byte = self.lcd_gpio.pin(LCD_CLK, clk);
        if drives(LCD_DATA) {
            if let Some(byte) = byte { self.lcd_byte(byte); }
        }

        // M5GFX 0.3.0 probe_hardware.inl releases the shared data line and clocks one
        // dummy bit. Drive the next bit on each falling edge for its low-phase reads.
        let value = if selected && !reset && self.lcd.dc && !drives(LCD_DATA) && !self.lcd_id().is_empty() {
            if self.lcd_clk && !clk { self.lcd_read_bit = self.lcd_read_bit.saturating_add(1); }
            if self.lcd_read_bit == 1 && self.lcd_clk && !clk { self.lcd_gpio_reads += 1; }
            Some(self.lcd_read_bit.checked_sub(1).is_some_and(|bit| {
                self.lcd_id().get(bit / 8).is_some_and(|byte| byte & (0x80 >> (bit % 8)) != 0)
            }))
        } else { None };
        if self.lcd_selected && !selected { self.lcd_cmd = 0; self.lcd_read_bit = 0; }
        self.lcd_selected = selected;
        self.lcd_clk = clk;
        Self::drive(self.cycle, &mut self.edges, &mut self.released, LCD_DATA, &mut self.lcd_drive, value);
    }

    fn gpio_i2c(&mut self, enabled: u64, output: u64) {
        let level = |pin: u8| enabled & (1 << pin) == 0 || output & (1 << pin) != 0;
        let (scl, sda) = (level(SCL), level(SDA));
        if (scl, sda) != (self.soft.scl, self.soft.sda) {
            let mut keyboard = self.keyboard.borrow_mut();
            let mut imu = self.imu.borrow_mut();
            let mut bus = |op| {
                let addr = match op {
                    SoftOp::Start(addr, _) | SoftOp::Write(addr, _) | SoftOp::Read(addr) | SoftOp::Stop(addr) => addr,
                    SoftOp::Sync => return 0,
                };
                let dev: &mut dyn I2cDevice = match addr {
                    Tca8418::ADDRESS => &mut *keyboard,
                    BMI270_ADDRESS => &mut *imu,
                    Es8311::ADDRESS => &mut self.soft_codec,
                    _ => return if matches!(op, SoftOp::Read(_)) { 0xff } else { 0 },
                };
                match op {
                    SoftOp::Start(_, read) => { self.soft_acks += 1; dev.start(read) as u8 }
                    SoftOp::Write(_, byte) => dev.write(byte) as u8,
                    SoftOp::Read(_) => dev.read(),
                    SoftOp::Stop(_) => { dev.stop(); 0 }
                    SoftOp::Sync => 0,
                }
            };
            if scl != self.soft.scl { self.soft.levels(scl, self.soft.sda, &mut bus); }
            if sda != self.soft.sda { self.soft.levels(scl, sda, &mut bus); }
        }
        let sda = if self.soft.pull_low { Some(false) } else { (enabled & (1 << SDA) == 0).then_some(true) };
        let scl = (enabled & (1 << SCL) == 0).then_some(true);
        Self::drive(self.cycle, &mut self.edges, &mut self.released, SDA, &mut self.sda_drive, sda);
        Self::drive(self.cycle, &mut self.edges, &mut self.released, SCL, &mut self.scl_drive, scl);
    }

    // M5Cardputer Keyboard.h's physical key labels, in the library's remapped grid.
    // KeyboardReader/TCA8418.cpp maps (row, col) to (col % 4, row * 2 + col / 4).
    fn key_position(name: &str) -> Option<(u8, u8)> {
        const ROWS: [[&str; 14]; 4] = [
            ["`", "1", "2", "3", "4", "5", "6", "7", "8", "9", "0", "-", "=", "del"],
            ["tab", "q", "w", "e", "r", "t", "y", "u", "i", "o", "p", "[", "]", "\\"],
            ["fn", "shift", "a", "s", "d", "f", "g", "h", "j", "k", "l", ";", "'", "enter"],
            ["ctrl", "opt", "alt", "z", "x", "c", "v", "b", "n", "m", ",", ".", "/", "space"],
        ];
        let name = match name { "backspace" | "delete" => "del", "return" => "enter", _ => name };
        ROWS.iter().enumerate().find_map(|(y, keys)| keys.iter().position(|key| key.eq_ignore_ascii_case(name))
            .map(|x| ((x / 2) as u8, (y + (x % 2) * 4) as u8)))
    }

    fn visible(&self) -> Vec<u16> {
        if self.lcd_reset || !self.lcd_bl || !self.lcd.on || self.lcd.sleeping { return vec![0; WIDTH * HEIGHT]; }
        // Fixed mounting, with the long side over the keyboard. MADCTL already transformed
        // writes into controller coordinates; the crop never follows the guest's rotation.
        (0..HEIGHT).flat_map(|y| (0..WIDTH).map(move |x| {
            let px = self.lcd.gram[(LCD_Y + x) * self.lcd.cols + LCD_X + HEIGHT - 1 - y];
            if self.lcd.inverted { px } else { !px }
        })).collect()
    }
}

impl BoardModel for CardputerAdv {
    fn name(&self) -> &'static str { "m5stack-cardputer-adv" }
    fn named_pin(&self, name: &str) -> Option<u8> { matches!(name, "btn" | "g0").then_some(BUTTON) }
    fn input_levels(&self) -> Vec<(u8, bool)> { vec![(SDA, !self.soft.pull_low), (SCL, true), (KEY_INT, self.key_int_high)] }
    fn observes_gpio_pads(&self) -> bool { true }
    fn gpio_waveform_at(&mut self, cycle: VirtualCycle, routes: PinRoutes<'_>) {
        self.cycle = cycle;
        let (mut enabled, mut output) = (0, 0);
        for pin in [LCD_CS, LCD_RST, LCD_CLK, LCD_DATA, LCD_DC, SDA, SCL] {
            if let Some(high) = routes.software_pad_output(pin) {
                enabled |= 1 << pin;
                output |= u64::from(high) << pin;
            }
        }
        self.gpio_lcd(enabled, output);
        self.gpio_i2c(enabled, output);
        if let Some(sd) = &mut self.sd {
            let selected = routes.software_pad_output(TF_CS) == Some(false);
            if self.sd_cs_low && !selected { sd.deselect(); }
            self.sd_cs_low = selected;
        }
        // GPIO mode is also usable by an unmodulated NEC transmitter.
        if !self.ir_pwm { self.ir.carrier(cycle, routes.software_pad_output(IR) == Some(true)); }
    }
    fn observes_ledc(&self) -> bool { true }
    fn ledc_output_at(&mut self, cycle: VirtualCycle, pwm: &dyn Fn(u8) -> Option<(f64, u32)>) {
        let bl = pwm(LCD_BL).is_some_and(|(_, duty)| duty != 0);
        if bl != self.lcd_bl { self.lcd_bl = bl; self.lcd_version += 1; }
        let ir = pwm(IR);
        if ir.is_some() || self.ir_pwm {
            self.ir.carrier(cycle, ir.is_some_and(|(hz, duty)| (30_000.0..=45_000.0).contains(&hz) && (1..65535).contains(&duty)));
        }
        self.ir_pwm = ir.is_some();
    }
    fn uses_spi_pins(&self) -> bool { true }
    fn has_spi_sd_slot(&self) -> bool { true }
    fn insert_spi_sd(&mut self, card: SdCard) { self.sd = Some(Box::new(SpiSdCard::new(card))); }
    fn spi_transfer_pins(&mut self, host: u8, pins: SpiPins, tx: &[u8], rx_len: usize) -> Vec<u8> {
        if let Some(sd) = &mut self.sd {
            let now_ns = (u128::from(self.cycle) * 1_000_000_000 / u128::from(CPU_HZ)).min(u128::from(u64::MAX)) as u64;
            if let Some(rx) = pins.transfer_sd(TF, sd, tx, rx_len, now_ns) { return rx; }
        }
        let mut rx = vec![0xff; rx_len];
        // M5GFX 9bcedf4 probes RDDID on SPI2 before moving the LCD to SPI3.
        // Both hosts can reach these same wires, independently of the SD routes.
        if !matches!(host, 2 | 3) || self.lcd_reset || pins.cs & (1 << LCD_CS) == 0 || pins.sclk & (1 << LCD_CLK) == 0 { return rx; }
        if self.lcd.dc && rx_len != 0 && pins.miso == Some(LCD_DATA) && !self.lcd_id().is_empty() {
            for byte in &mut rx {
                *byte = *self.lcd_id().get(self.lcd_read_bit / 8).unwrap_or(&0);
                self.lcd_read_bit = self.lcd_read_bit.saturating_add(8);
            }
        } else if pins.mosi & (1 << LCD_DATA) != 0 {
            if host == 2 { self.lcd_spi2_bytes += tx.len() as u64; }
            else { self.lcd_spi_bytes += tx.len() as u64; }
            for &byte in tx { self.lcd_byte(byte); }
        }
        rx
    }
    fn i2c_devices(&mut self) -> Vec<(u8, u8, Box<dyn I2cDevice>)> {
        let mut devices: Vec<(u8, u8, Box<dyn I2cDevice>)> = Vec::new();
        for bus in 0..2 {
            devices.push((bus, Tca8418::ADDRESS, Box::new(InternalPort::Keyboard(self.keyboard.clone()))));
            devices.push((bus, BMI270_ADDRESS, Box::new(InternalPort::Imu(self.imu.clone()))));
            devices.push((bus, Es8311::ADDRESS, Box::new(InternalPort::Codec(self.codec.i2c_port()))));
        }
        devices
    }
    fn i2s_codecs(&self) -> Option<I2sCodecs> {
        // M5Unified 0.3.0 M5Unified.inl: I2S1 TX and I2S0 RX share BCK and WS.
        Some(self.codec.i2s_codecs(1, 0, 41, 43, 42, 46))
    }
    // M5Unified 0.3.0 Power_Class.inl doubles the calibrated ADC1 GPIO10 voltage.
    fn default_analog_inputs(&self) -> &'static [(u8, f32)] { &[(10, 1.85)] }
    fn set_imu_motion(&mut self, mode: u32) -> bool { self.motion.mode.store(mode, Ordering::Relaxed); true }
    fn imu_motion(&self) -> u32 { self.motion.mode.load(Ordering::Relaxed) }
    fn next_deadline(&self) -> Option<VirtualCycle> { self.key_up.iter().map(|&(_, _, due)| due).min() }
    fn advance_to(&mut self, cycle: VirtualCycle) {
        self.cycle = cycle;
        self.motion.cycle.store(cycle, Ordering::Relaxed);
        if self.next_deadline().is_some_and(|due| due <= cycle) {
            let mut keyboard = self.keyboard.borrow_mut();
            self.key_up.retain(|&(row, col, due)| {
                if due <= cycle { keyboard.key(row, col, false); }
                due > cycle
            });
        }
    }
    fn take_edges(&mut self) -> Vec<BoardEdge> {
        let high = !self.keyboard.borrow().int_low();
        if high != self.key_int_high {
            self.key_int_high = high;
            self.edges.push(BoardEdge { cycle: self.cycle, pin: KEY_INT, level: high });
        }
        std::mem::take(&mut self.edges)
    }
    fn released_inputs(&mut self) -> Vec<u8> {
        let pins = std::mem::take(&mut self.released);
        [SDA, SCL, LCD_DATA].into_iter().filter(|&pin| pins & (1 << pin) != 0).collect()
    }
    fn accepts_command(&self, verb: &str) -> bool { matches!(verb, "key" | "imu") }
    fn board_command(&mut self, cycle: VirtualCycle, verb: &str, args: &[&str]) -> bool {
        if verb == "imu" {
            if args.len() != 6 { return false; }
            let mut values = [0.0; 6];
            for (dst, arg) in values.iter_mut().zip(args) {
                let Ok(value) = arg.parse::<f64>() else { return false; };
                if !value.is_finite() { return false; }
                *dst = value;
            }
            let [ax, ay, az, gx, gy, gz] = values;
            self.imu.borrow_mut().set_sample([ax, ay, az], [gx, gy, gz], None);
            return true;
        }
        if verb != "key" { return false; }
        let (name, ms) = match args {
            [name] => (name, 100u32),
            [name, ms] => { let Ok(ms) = ms.parse::<u32>() else { return false; }; (name, ms.max(1)) }
            _ => return false,
        };
        let Some((row, col)) = Self::key_position(name) else { return false; };
        let release = cycle.saturating_add(u64::from(ms) * (CPU_HZ / 1000));
        if let Some((_, _, due)) = self.key_up.iter_mut().find(|(r, c, _)| (*r, *c) == (row, col)) {
            *due = release;
        } else {
            self.key_up.push((row, col, release));
            self.keyboard.borrow_mut().key(row, col, true);
        }
        true
    }
    fn display(&self) -> Option<(u32, u32, Vec<u16>, u64)> { Some((WIDTH as u32, HEIGHT as u32, self.visible(), self.display_version())) }
    fn display_frames(&self) -> u64 { self.lcd.frames }
    fn display_version(&self) -> u64 { self.lcd_version + self.lcd.pixels_written }
    fn gram(&self) -> Option<(Vec<u16>, usize, usize)> { Some((self.lcd.gram.clone(), self.lcd.cols, self.lcd.rows)) }
    fn report(&self) -> String {
        let mut report = format!("[emu] cardputer-adv: {} GPIO LCD reads, {} SPI3 LCD bytes, {} RAMWR, {} pixels, MADCTL={:#04x}, backlight={}; {} GPIO I2C ACKs, {} keyboard events read",
            self.lcd_gpio_reads, self.lcd_spi_bytes, self.lcd.frames, self.lcd.pixels_written, self.lcd.madctl, self.lcd_bl, self.soft_acks, self.keyboard.borrow().read_count());
        if self.lcd_spi2_bytes != 0 { report += &format!("; {} SPI2 LCD bytes", self.lcd_spi2_bytes); }
        if self.ir.count != 0 {
            report += &format!("\n[emu] cardputer-adv GPIO44 NEC: {} frames", self.ir.count);
            for &(address, command) in &self.ir.recent { report += &format!(" address=0x{address:04x}/command=0x{command:02x}"); }
        }
        if let Some(sd) = &self.sd { report += &format!("\n{}", sd.report()); }
        report
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use emu_core::bus::Bus;

    #[test]
    fn gpio_read_id_accepts_open_drain_turnaround() {
        // ESP-IDF v5.4 gpio_reg.h: OUT1 0x10, ENABLE1_W1TS 0x30, IN1 0x40;
        // GPIO_PINn starts at 0x74. IO_MUX GPIO0 starts at 0x04 (io_mux_reg.h).
        const GPIO: u32 = 0x6000_4000;
        const MUX: u32 = 0x6000_9000;
        let bit = |pin: u8| 1u32 << (pin - 32);
        let mut bus = crate::bus::SocBus::new(8 << 20, 0, [0; 6]);
        bus.board = Box::new(CardputerAdv::new());
        bus.attach_board_devices();
        for pin in [LCD_CS, LCD_RST, LCD_CLK, LCD_DATA, LCD_DC] {
            bus.write32(MUX + 4 + 4 * u32::from(pin), (1 << 12) | (1 << 9)).unwrap();
        }
        let reset_and_cs = bit(LCD_RST) | bit(LCD_CS);
        bus.write32(GPIO + 0x10, reset_and_cs).unwrap();
        bus.write32(GPIO + 0x30, reset_and_cs | bit(LCD_CLK) | bit(LCD_DATA) | bit(LCD_DC)).unwrap();
        let reset = bit(LCD_RST);
        for i in 0..8 {
            let data = if 0x04 & (0x80 >> i) != 0 { bit(LCD_DATA) } else { 0 };
            for clock in [0, bit(LCD_CLK), 0] { bus.write32(GPIO + 0x10, reset | data | clock).unwrap(); }
        }
        bus.write32(GPIO + 0x74 + 4 * u32::from(LCD_DATA), 1 << 2).unwrap();
        let base = reset | bit(LCD_DATA) | bit(LCD_DC);
        bus.write32(GPIO + 0x10, base).unwrap();
        let mut id = 0u32;
        for i in 0..25 {
            let sample = bus.read32(GPIO + 0x40).unwrap() & bit(LCD_DATA) != 0;
            if i != 0 { id = (id << 1) | u32::from(sample); }
            bus.write32(GPIO + 0x10, base | bit(LCD_CLK)).unwrap();
            bus.write32(GPIO + 0x10, base).unwrap();
        }
        assert_eq!(id, 0x85_85_52);
        bus.write32(GPIO + 0x10, base | bit(LCD_CS)).unwrap();
        assert_ne!(bus.read32(GPIO + 0x40).unwrap() & bit(LCD_DATA), 0);
    }

    #[test]
    fn spi2_rddid_reads_the_st7789_on_the_shared_data_pin() {
        // ESP-IDF v5.5.5 esp32s3 spi_reg.h, io_mux_reg.h and gpio_sig_map.h.
        const SPI2: u32 = 0x6002_4000;
        const GPIO: u32 = 0x6000_4000;
        const MUX: u32 = 0x6000_9004;
        const READ: u32 = 1 << 28;
        const SIO: u32 = 1 << 17;
        let transfer = |bus: &mut crate::bus::SocBus, user, bits: u32| {
            bus.write32(SPI2 + 0x10, user).unwrap();
            bus.write32(SPI2 + 0x1c, bits - 1).unwrap();
            bus.write32(SPI2, 1 << 24).unwrap();
            bus.read32(SPI2 + 0x98).unwrap()
        };
        for native in [false, true] {
            let mut bus = crate::bus::SocBus::new(8 << 20, 0, [0; 6]);
            bus.board = Box::new(CardputerAdv::new());
            bus.attach_board_devices();
            let dc = 1 << (LCD_DC - 32);
            let cs = 1 << (LCD_CS - 32);
            for pin in [LCD_CS, LCD_DC] {
                bus.write32(MUX + 4 * u32::from(pin), 1 << 12).unwrap();
            }
            let function = if native { 2 } else { 1 };
            for (pin, signal) in [(LCD_DATA, 103), (LCD_CLK, 101)] {
                bus.write32(MUX + 4 * u32::from(pin), (function << 12) | (1 << 9)).unwrap();
                if !native { bus.write32(GPIO + 0x554 + 4 * u32::from(pin), signal).unwrap(); }
            }
            if !native { bus.write32(GPIO + 0x154 + 4 * 103, 0x80 | u32::from(LCD_DATA)).unwrap(); }
            bus.write32(SPI2 + 0x20, 0x3f).unwrap(); // hardware CS disabled
            bus.write32(GPIO + 0x30, dc | cs).unwrap(); // GPIO CS and D/C low
            bus.write32(SPI2 + 0x98, 0x04).unwrap();
            transfer(&mut bus, 1 << 27, 8);
            bus.write32(GPIO + 0x14, dc).unwrap();
            assert_eq!(transfer(&mut bus, READ, 32), u32::MAX); // separate Q is unwired
            assert_eq!(transfer(&mut bus, READ | SIO, 32), 0x0052_8585);
            bus.write32(GPIO + 0x14, cs).unwrap();
            assert_eq!(transfer(&mut bus, READ | SIO, 32), u32::MAX);
            bus.write32(GPIO + 0x18, cs).unwrap();
            assert_eq!(transfer(&mut bus, READ | SIO, 32), u32::MAX); // new selection
        }
    }

    #[test]
    fn spi3_read_id_follows_physical_select_and_data_routes() {
        let mut board = CardputerAdv::new();
        let route = SpiPins { sclk: 1 << LCD_CLK, mosi: 1 << LCD_DATA, cs: 1 << LCD_CS, miso: Some(LCD_DATA) };
        board.spi_transfer_pins(3, route, &[0x04], 0);
        board.lcd.dc = true;
        assert_eq!(board.spi_transfer_pins(1, route, &[], 1), [0xff]);
        assert_eq!(board.spi_transfer_pins(3, SpiPins { cs: 0, ..route }, &[], 1), [0xff]);
        assert_eq!(board.spi_transfer_pins(3, route, &[], 2), [0x85, 0x85]);
        assert_eq!(board.spi_transfer_pins(3, route, &[], 1), [0x52]);
    }

    #[test]
    fn key_labels_match_the_tca8418_matrix_remap() {
        for (name, row, col) in [("a", 1, 2), ("space", 6, 7), ("enter", 6, 6), ("backspace", 6, 4), ("fn", 0, 2)] {
            let (r, c) = CardputerAdv::key_position(name).unwrap();
            assert_eq!((r, c), (row, col));
        }
    }
}
