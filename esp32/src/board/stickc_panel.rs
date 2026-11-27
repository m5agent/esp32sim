//! Shared ST77xx wiring for the StickC family. Panel ID, read latency and
//! viewport follow the board descriptors and soft-SPI probes in M5GFX 28d47949.
use esp_soc::{board::SpiPins, devices::{DcsPanel, SpiBitBang}, pins::PinRoutes};

const CLOCK: u8 = 13;
const DATA: u8 = 15;
const CS: u8 = 5;

pub(super) struct Panel {
    pub ram: DcsPanel,
    small: bool,
    dc: u8,
    rst: u8,
    reset: bool,
    selected: bool,
    prev_clk: bool,
    command: u8,
    read_bit: usize,
    spi: SpiBitBang,
    pub id_reads: u64,
    pub generation: u64,
}

impl Panel {
    pub(super) fn new(small: bool, plus2: bool) -> Self {
        Self {
            ram: if small { DcsPanel::st7735() } else { DcsPanel::st7789() },
            small, dc: if plus2 { 14 } else { 23 }, rst: if plus2 { 12 } else { 18 },
            reset: false, selected: false, prev_clk: false, command: 0, read_bit: 0,
            spi: SpiBitBang::new(CLOCK, DATA, CS), id_reads: 0, generation: 0,
        }
    }

    fn byte(&mut self, value: u8) {
        if !self.ram.dc {
            self.command = value;
            self.read_bit = 0;
            self.generation += 1;
        }
        self.ram.byte(value);
    }

    fn id(&self) -> [u8; 3] {
        if self.small { [0x7c, 0x89, 0xf0] } else { [0x81, 0x00, 0x00] }
    }

    pub(super) fn gpio(&mut self, routes: &PinRoutes<'_>, powered: bool) -> Option<bool> {
        let output = |pin| routes.software_pad_output(pin);
        let reset = !powered || output(self.rst) == Some(false);
        if reset && !self.reset {
            self.ram.reset();
            self.command = 0;
            self.generation += 1;
        }
        self.reset = reset;
        self.ram.dc = output(self.dc) == Some(true);
        let selected = output(CS) == Some(false);
        let clk = output(CLOCK) == Some(true);
        let data = output(DATA);
        self.spi.pin(CS, !selected);
        self.spi.pin(DATA, data == Some(true));
        if let Some(byte) = self.spi.pin(CLOCK, clk).filter(|_| data.is_some() && !reset) {
            self.byte(byte);
        }
        if self.selected && !selected {
            self.command = 0;
            self.read_bit = 0;
        }
        self.selected = selected;
        let falling = self.prev_clk && !clk;
        self.prev_clk = clk;

        // RDDID starts with a dummy bit. M5GFX samples before each rising
        // edge, so a completed falling edge advances to the next bit.
        if selected && !reset && self.ram.dc && data.is_none() && self.command == 4 {
            if falling {
                self.read_bit = self.read_bit.saturating_add(1);
                if self.read_bit == 1 { self.id_reads += 1; }
            }
            return Some(self.read_bit.checked_sub(1).is_some_and(|bit| {
                self.id().get(bit / 8).is_some_and(|byte| byte & (0x80 >> (bit % 8)) != 0)
            }));
        }
        None
    }

    pub(super) fn transfer(&mut self, pins: SpiPins, tx: &[u8], rx_bits: usize, powered: bool) -> Vec<u8> {
        let rx_len = rx_bits.div_ceil(8);
        if !powered || self.reset || pins.sclk & (1 << CLOCK) == 0 || pins.cs & (1 << CS) == 0 {
            return vec![0xff; rx_len];
        }
        if self.ram.dc && self.command == 4 && rx_len != 0 && pins.miso == Some(DATA) {
            if self.read_bit == 0 { self.id_reads += 1; }
            let id = self.id();
            let mut response = vec![0; rx_len];
            for i in 0..rx_bits.min(25usize.saturating_sub(self.read_bit)) {
                if self.read_bit == 0 { self.read_bit += 1; continue; }
                let bit = self.read_bit - 1;
                let value = id[bit / 8] >> (7 - bit % 8) & 1;
                response[i / 8] |= value << (7 - i % 8);
                self.read_bit += 1;
            }
            return response;
        }
        if pins.mosi & (1 << DATA) != 0 {
            for &value in tx { self.byte(value); }
        }
        vec![0xff; rx_len]
    }

    pub(super) fn display(&self, backlight: u16, powered: bool) -> (u32, u32, Vec<u16>, u64) {
        let (w, h, x0, y0) = if self.small { (80, 160, 26, 1) } else { (135, 240, 52, 40) };
        let pixels = if self.reset || !powered || self.ram.sleeping || !self.ram.on || backlight == 0 {
            vec![0; w * h]
        } else {
            (0..h).flat_map(|y| (0..w).map(move |x| {
                // ST7735S is mounted upside down relative to the ST7789 glass.
                let (x, y) = if self.small { (w - 1 - x, h - 1 - y) } else { (x, y) };
                let px = self.ram.gram[(y0 + y) * self.ram.cols + x0 + x];
                let px = u32::from(px);
                let dim = |mask: u32, shift: u32| ((px >> shift) & mask) * u32::from(backlight) / 65535;
                ((dim(31, 11) << 11) | (dim(63, 5) << 5) | dim(31, 0)) as u16
            })).collect()
        };
        (w as u32, h as u32, pixels, self.generation + self.ram.pixels_written)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spi_read_id_clocks_dummy_exactly_once_and_respects_cs() {
        let mut panel = Panel::new(false, false);
        let routes = SpiPins { sclk: 1 << CLOCK, mosi: 1 << DATA, miso: Some(DATA), cs: 1 << CS };
        panel.transfer(routes, &[4], 0, true);
        panel.ram.dc = true;
        let deselected = SpiPins { cs: 0, ..routes };
        assert_eq!(panel.transfer(deselected, &[], 32, true), [0xff; 4]);
        assert_eq!(panel.transfer(routes, &[], 1, true), [0]);
        assert_eq!(panel.transfer(routes, &[], 32, true), [0x81, 0, 0, 0]);
        assert_eq!(panel.id_reads, 1);
        assert_eq!(panel.transfer(routes, &[], 32, false), [0xff; 4]);
    }
}
