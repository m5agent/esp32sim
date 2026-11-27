//! Pin-routed, three-wire DCS displays used by small ESP32-S3 boards.
use esp_soc::board::{BoardEdge, SpiPins, VirtualCycle};
use esp_soc::devices::{DcsPanel, SpiBitBang};
use esp_soc::pins::PinRoutes;

#[derive(Clone, Copy)]
pub(super) struct LcdPins {
    pub clk: u8, pub data: u8, pub dc: u8, pub cs: u8, pub rst: u8,
}

pub(super) struct SpiLcd {
    pins: LcdPins,
    id: [u8; 3],
    pub(super) lcd: DcsPanel,
    lcd_gpio: SpiBitBang,
    lcd_cmd: u8,
    lcd_read_bit: usize,
    pub(super) lcd_reset: bool,
    lcd_selected: bool,
    lcd_clk: bool,
    pub(super) lcd_bl: bool,
    lcd_version: u64,
    pub(super) lcd_gpio_reads: u64,
    pub(super) lcd_spi_bytes: u64,
    pub(super) lcd_spi2_bytes: u64,
    edges: Vec<BoardEdge>,
    released: u64,
    lcd_drive: Option<bool>,
}

impl SpiLcd {
    pub fn new(pins: LcdPins, lcd: DcsPanel, id: [u8; 3]) -> Self {
        Self { pins, id, lcd, lcd_gpio: SpiBitBang::new(pins.clk, pins.data, pins.cs),
            lcd_cmd: 0, lcd_read_bit: 0, lcd_reset: false, lcd_selected: false, lcd_clk: false,
            lcd_bl: false, lcd_version: 0, lcd_gpio_reads: 0, lcd_spi_bytes: 0,
            lcd_spi2_bytes: 0, lcd_drive: None, edges: Vec::new(), released: 0 }
    }
    pub fn routes(&mut self, cycle: VirtualCycle, routes: &PinRoutes<'_>) {
        let mut enabled = 0;
        let mut output = 0;
        for pin in [self.pins.cs, self.pins.rst, self.pins.clk, self.pins.data, self.pins.dc] {
            if let Some(high) = routes.software_pad_output(pin) {
                enabled |= 1 << pin;
                output |= u64::from(high) << pin;
            }
        }
        self.gpio(cycle, enabled, output);
    }
    pub fn power_loss(&mut self) {
        self.lcd.reset();
        self.lcd_cmd = 0;
        self.lcd_read_bit = 0;
        self.lcd_version += 1;
    }
    pub fn set_backlight(&mut self, on: bool) {
        if on != self.lcd_bl { self.lcd_bl = on; self.lcd_version += 1; }
    }
    pub fn visible(&self) -> bool { !self.lcd_reset && self.lcd_bl && self.lcd.on && !self.lcd.sleeping }
    pub fn cardputer_pixels(&self) -> Vec<u16> {
        if !self.visible() { return vec![0; 240 * 135]; }
        // The long side of the glass is mounted over the keyboard.
        (0..135).flat_map(|y| (0..240).map(move |x| {
            let color = self.lcd.gram[(40 + x) * self.lcd.cols + 52 + 134 - y];
            if self.lcd.inverted { color } else { !color }
        })).collect()
    }
    pub fn version(&self) -> u64 { self.lcd_version + self.lcd.pixels_written }
    pub fn edges(&mut self) -> Vec<BoardEdge> { std::mem::take(&mut self.edges) }
    pub fn released(&mut self) -> Option<u8> { (std::mem::take(&mut self.released) != 0).then_some(self.pins.data) }
    fn lcd_byte(&mut self, byte: u8) {
        if self.lcd_reset { return; }
        if !self.lcd.dc { self.lcd_cmd = byte; self.lcd_read_bit = 0; self.lcd_version += 1; }
        self.lcd.byte(byte);
    }

    fn lcd_id(&self) -> &[u8] {
        // RDDID and the three single-byte ID commands use the same ID fields.
        match self.lcd_cmd { 0x04 => &self.id, 0xda..=0xdc => &self.id[(self.lcd_cmd - 0xda) as usize..=(self.lcd_cmd - 0xda) as usize], _ => &[] }
    }

    pub(super) fn drive(cycle: VirtualCycle, edges: &mut Vec<BoardEdge>, released: &mut u64, pin: u8,
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

    pub(super) fn gpio(&mut self, cycle: VirtualCycle, enabled: u64, output: u64) {
        let drives = |pin: u8| enabled & (1 << pin) != 0;
        let high = |pin: u8| output & (1 << pin) != 0;
        let selected = drives(self.pins.cs) && !high(self.pins.cs);
        let reset = drives(self.pins.rst) && !high(self.pins.rst);
        let clk = drives(self.pins.clk) && high(self.pins.clk);
        if reset && !self.lcd_reset { self.lcd.reset(); self.lcd_cmd = 0; self.lcd_version += 1; }
        self.lcd_reset = reset;
        self.lcd.dc = high(self.pins.dc);
        self.lcd_gpio.pin(self.pins.cs, !selected);
        self.lcd_gpio.pin(self.pins.data, high(self.pins.data));
        let byte = self.lcd_gpio.pin(self.pins.clk, clk);
        if drives(self.pins.data) {
            if let Some(byte) = byte { self.lcd_byte(byte); }
        }

        // M5GFX 0.3.0 probe_hardware.inl releases the shared data line and clocks one
        // dummy bit. Drive the next bit on each falling edge for its low-phase reads.
        let value = if selected && !reset && self.lcd.dc && !drives(self.pins.data) && !self.lcd_id().is_empty() {
            if self.lcd_clk && !clk { self.lcd_read_bit = self.lcd_read_bit.saturating_add(1); }
            if self.lcd_read_bit == 1 && self.lcd_clk && !clk { self.lcd_gpio_reads += 1; }
            Some(self.lcd_read_bit.checked_sub(1).is_some_and(|bit| {
                self.lcd_id().get(bit / 8).is_some_and(|byte| byte & (0x80 >> (bit % 8)) != 0)
            }))
        } else { None };
        if self.lcd_selected && !selected { self.lcd_cmd = 0; self.lcd_read_bit = 0; }
        self.lcd_selected = selected;
        self.lcd_clk = clk;
        Self::drive(cycle, &mut self.edges, &mut self.released, self.pins.data, &mut self.lcd_drive, value);
    }

    pub fn transfer(&mut self, host: u8, pins: SpiPins, tx: &[u8], rx_len: usize) -> Vec<u8> {
        let mut rx = vec![0xff; rx_len];
        // M5GFX can probe RDDID on SPI2 before moving the LCD to SPI3.
        // Both hosts can reach these same wires, independently of the SD routes.
        if !matches!(host, 2 | 3) || self.lcd_reset || pins.cs & (1 << self.pins.cs) == 0 || pins.sclk & (1 << self.pins.clk) == 0 { return rx; }
        if self.lcd.dc && rx_len != 0 && pins.miso == Some(self.pins.data) && !self.lcd_id().is_empty() {
            for byte in &mut rx {
                *byte = *self.lcd_id().get(self.lcd_read_bit / 8).unwrap_or(&0);
                self.lcd_read_bit = self.lcd_read_bit.saturating_add(8);
            }
        } else if pins.mosi & (1 << self.pins.data) != 0 {
            if host == 2 { self.lcd_spi2_bytes += tx.len() as u64; }
            else { self.lcd_spi_bytes += tx.len() as u64; }
            for &byte in tx { self.lcd_byte(byte); }
        }
        rx
    }
}

pub(super) const CARDPUTER_LCD: LcdPins = LcdPins { clk: 36, data: 35, dc: 34, cs: 37, rst: 33 };
pub(super) fn cardputer_lcd() -> SpiLcd {
    SpiLcd::new(CARDPUTER_LCD, DcsPanel::st7789(), [0x85, 0x85, 0x52])
}
