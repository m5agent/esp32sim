//! AtomS3 and AtomS3R LCD buses. Wiring, ID bytes and glass viewports follow
//! M5GFX 28d47949 src/board_detect/m5/generated/esp32s3_board_data.hpp.
use esp_soc::{board::SpiPins, devices::{DcsPanel, SpiBitBang}, pins::PinRoutes};

#[derive(Clone, Copy)]
pub(super) enum Controller { St7735s, Gc9107 }

pub(super) struct Panel {
    pub ram: DcsPanel,
    chip: Controller,
    clk: u8,
    cs: u8,
    dc: u8,
    rst: u8,
    decoder: SpiBitBang,
    reset: bool,
    previous_clk: bool,
    previous_cs: bool,
    command: u8,
    bit: usize,
    pub version: u64,
    pub id_reads: u64,
}

const DATA: u8 = 21;

impl Panel {
    pub fn new(r_series: bool, chip: Controller) -> Self {
        let (clk, cs, dc, rst) = if r_series { (15, 14, 42, 48) } else { (17, 15, 33, 34) };
        let ram = match chip { Controller::St7735s => DcsPanel::new(132, 132), Controller::Gc9107 => DcsPanel::new(128, 160) };
        Self { ram, chip, clk, cs, dc, rst, decoder: SpiBitBang::new(clk, DATA, cs), reset: false,
               previous_clk: false, previous_cs: true, command: 0, bit: 0, version: 0, id_reads: 0 }
    }

    fn byte(&mut self, b: u8) {
        if self.reset { return; }
        if !self.ram.dc { self.command = b; self.bit = 0; self.version += 1; }
        self.ram.byte(b);
    }

    fn id(&self) -> &[u8] {
        let id = match self.chip { Controller::St7735s => &[0x83, 0x76, 0x00], Controller::Gc9107 => &[0x00, 0x91, 0x07] };
        match self.command { 0x04 => id, 0xda => &id[..1], 0xdb => &id[1..2], 0xdc => &id[2..], _ => &[] }
    }

    pub fn gpio(&mut self, routes: &PinRoutes<'_>) -> Option<bool> {
        let pin = |gpio| routes.software_pad_output(gpio);
        let reset = pin(self.rst) == Some(false);
        if reset && !self.reset { self.ram.reset(); self.command = 0; self.bit = 0; self.version += 1; }
        self.reset = reset;
        self.ram.dc = pin(self.dc) == Some(true);
        let cs = pin(self.cs) != Some(false);
        let clk = pin(self.clk) == Some(true);
        let data = pin(DATA);
        self.decoder.pin(self.cs, cs);
        self.decoder.pin(DATA, data == Some(true));
        if let Some(byte) = self.decoder.pin(self.clk, clk).filter(|_| data.is_some()) { self.byte(byte); }
        if !self.previous_cs && cs { self.command = 0; self.bit = 0; }
        let result = if !cs && !reset && self.ram.dc && data.is_none() && !self.id().is_empty() {
            if self.previous_clk && !clk {
                self.bit = self.bit.saturating_add(1);
                if self.bit == 1 { self.id_reads += 1; }
            }
            Some(self.bit.checked_sub(1).is_some_and(|bit| {
                self.id().get(bit / 8).is_some_and(|b| b & (0x80 >> (bit % 8)) != 0)
            }))
        } else { None };
        self.previous_clk = clk;
        self.previous_cs = cs;
        result
    }

    pub fn transfer(&mut self, pins: SpiPins, tx: &[u8], rx_len: usize) -> Vec<u8> {
        if self.reset || pins.sclk & (1 << self.clk) == 0 || pins.cs & (1 << self.cs) == 0 { return vec![0xff; rx_len]; }
        if self.ram.dc && pins.miso == Some(DATA) && rx_len != 0 && !self.id().is_empty() {
            if self.bit == 0 { self.id_reads += 1; }
            return (0..rx_len).map(|_| {
                let value = self.id().get(self.bit / 8).copied().unwrap_or(0);
                self.bit = self.bit.saturating_add(8);
                value
            }).collect();
        }
        if pins.mosi & (1 << DATA) != 0 { for &b in tx { self.byte(b); } }
        vec![0xff; rx_len]
    }

    pub fn display(&self, brightness: u32) -> Vec<u16> {
        if self.reset || !self.ram.on || self.ram.sleeping || brightness == 0 { return vec![0; 128 * 128]; }
        (0..128).flat_map(|y| (0..128).map(move |x| {
            let at = match self.chip { Controller::St7735s => (128 - y) * 132 + 129 - x, Controller::Gc9107 => (y + 32) * 128 + x };
            let px = u32::from(self.ram.gram[at]);
            let dim = |mask, shift: u32| ((px >> shift) & mask) * brightness / 255;
            ((dim(31, 11) << 11) | (dim(63, 5) << 5) | dim(31, 0)) as u16
        })).collect()
    }
}
