//! P4 MIPI-DSI DBI packet transport, DPI bridge, and its DW-GDMA scanout.
//!
//! Register layouts: ESP-IDF v5.4.1 soc/esp32p4/register/soc/{reg_base,
//! mipi_dsi_host_reg,mipi_dsi_bridge_reg,dw_gdma_reg}.h, checked with the
//! Arduino 3.2.5 headers (idf-release_v5.4-858a988d-v1). Both v5.4.1
//! and v5.5.4 esp_lcd/dsi/esp_lcd_panel_dpi.c use DW-GDMA linked lists
//! for scanout and DMA2D only for the optional application blit.
use esp_periph::{Device, RegRam, WriteEffect};
use esp_periph::sdmmc::SdmmcMemory;
use esp_soc::BoardModel;
use std::collections::VecDeque;

pub const HOST_BLOCK: u32 = 0x500a0;
pub const DW_DMA_BLOCK: u32 = 0x50081;
const BRIDGE_FIFO: u32 = 0x5010_5000;
const MAX_FRAME_BYTES: usize = 4 * 1024 * 1024;

#[derive(Default)]
pub struct Dsi {
    host: RegRam,
    bridge: RegRam,
    tx: Vec<u8>,
    rx: VecDeque<u8>,
    header: Option<u32>,
    max_return: [u16; 4],
    bridge_raw: u32,
    debug: bool,
}
impl Dsi {
    pub fn deliver(&mut self, board: &mut dyn BoardModel) {
        let Some(header) = self.header.take() else { return };
        let [id, lo, hi, _] = header.to_le_bytes();
        let channel = id >> 6;
        let dtype = id & 63;
        let arg = [lo, hi];
        let count = u16::from_le_bytes(arg) as usize;
        // MIPI data types as used in IDF v5.4.1 hal/mipi_dsi_hal.c.
        let short = match dtype { 0x03 | 0x04 => 0, 0x13 | 0x14 | 0x05 | 0x06 => 1, _ => 2 };
        match dtype {
            0x37 => self.max_return[channel as usize] = count as u16,
            0x04 | 0x14 | 0x24 | 0x06 => {
                let len = self.max_return[channel as usize].max(1) as usize;
                let data = board.dsi_read(channel, dtype, &arg[..short], len);
                self.rx.clear();
                self.rx.extend(data.into_iter().take(len));
                if self.debug { eprintln!("[dsi] read vc={channel} dt={dtype:#04x} cmd={lo:#04x}: {:02x?}", self.rx); }
            }
            0x03 | 0x13 | 0x23 | 0x05 | 0x15 => board.dsi_write(channel, dtype, &arg[..short]),
            0x29 | 0x39 => {
                if count <= self.tx.len() { board.dsi_write(channel, dtype, &self.tx[..count]); }
                self.tx.clear();
            }
            _ => {}
        }
    }

    pub fn video(&self) -> Option<(u32, u32, usize)> {
        if self.host.read(0x04) & 1 == 0 || self.host.read(0x34) & 1 != 0 ||
            self.bridge.read(0x04) & 1 == 0 || self.bridge.read(0x40) & 1 == 0 { return None; }
        let w = self.bridge.read(0x38) >> 16 & 0xfff;
        let h = self.bridge.read(0x30) >> 16 & 0xfff;
        // mipi_dsi_host_ll.h: DPI color coding (16-bit or 24-bit packed).
        let bpp = match self.host.read(0x10) & 15 { 0..=2 => 2, 5 => 3, _ => return None };
        let bytes = w as usize * h as usize * bpp;
        (bytes > 0 && bytes <= MAX_FRAME_BYTES).then_some((w, h, bpp))
    }

    pub fn vsync(&mut self) { self.bridge_raw |= 1 << 1; }
}
impl Device for Dsi {
    fn read(&mut self, off: u32) -> u32 {
        match off {
            0x70 => {
                let bytes = self.rx.len().min(4);
                let word = u32::from_le_bytes(std::array::from_fn(|_| self.rx.pop_front().unwrap_or(0)));
                if self.debug && bytes != 0 {
                    eprintln!("[dsi] RX FIFO pop: bytes={bytes} word={word:#010x} remaining={}", self.rx.len());
                }
                word
            }
            0x74 => (1 << 16) | 1 | (u32::from(self.tx.is_empty()) << 2) |
                (u32::from(self.tx.len() == 65536) << 3) | (u32::from(self.rx.is_empty()) << 4),
            0xb0 => if self.host.read(0xa0) & 3 == 3 { 0x1bd } else { 0 }, // locked, lanes in LP stop state
            0x858 => self.bridge_raw,
            0x85c => self.bridge_raw & self.bridge.read(0x50),
            0x800.. => self.bridge.read(off - 0x800),
            _ => self.host.read(off),
        }
    }
    fn write(&mut self, off: u32, value: u32) -> WriteEffect {
        match off {
            0x04 => {
                self.host.write(off, value);
                if value & 1 == 0 { self.tx.clear(); self.rx.clear(); self.header = None; self.max_return = [0; 4]; }
            }
            0x6c if self.host.read(0x04) & 1 != 0 => self.header = Some(value),
            0x70 if self.tx.len() < 65536 => self.tx.extend(value.to_le_bytes()),
            0x70 | 0x74 | 0xb0 => {}
            0x854 => self.bridge_raw &= !value,
            0x858 | 0x85c => {}
            0x800.. => self.bridge.write(off - 0x800, value),
            _ => self.host.write(off, value),
        }
        WriteEffect::NONE
    }
    fn irq_sources(&self) -> u64 { u64::from(self.bridge_raw & self.bridge.read(0x50) != 0) }
    fn debug(&mut self, on: bool) { self.debug = on; }
}

pub struct DwDma {
    ram: RegRam,
    enabled: u32,
    raw: [u32; 4],
    pub due: Option<u64>,
    pub bytes: Vec<u8>,
}
impl Default for DwDma {
    fn default() -> Self {
        let mut ram = RegRam::new();
        for channel in 0..4 { ram.write(0x180 + 0x100 * channel, u32::MAX); }
        Self { ram, enabled: 0, raw: [0; 4], due: None, bytes: Vec::new() }
    }
}
impl DwDma {
    pub fn active(&self) -> bool { self.enabled != 0 && self.ram.read(0x10) & 1 != 0 }
    fn status(&self, channel: usize) -> u32 {
        let off = channel as u32 * 0x100 + 0x100;
        // INTSTATUS_ENABLE resets each implemented bit high. When the guest
        // writes the mask, zero is a meaningful value.
        self.raw[channel] & self.ram.read(off + 0x80)
    }
    fn signalled(&self, i: usize) -> bool { self.status(i) & self.ram.read(i as u32 * 0x100 + 0x190) != 0 }

    pub fn scan(&mut self, memory: &mut impl SdmmcMemory, expected: usize) -> bool {
        let mut frame = false;
        for channel in 0..4 {
            if self.enabled & (1 << channel) == 0 { continue; }
            let off = channel as u32 * 0x100 + 0x100;
            // CFG_HI.TT_FC = memory to peripheral, DMA flow control.
            if self.ram.read(off + 0x24) & 7 != 1 { continue; }
            let mut desc = self.ram.read(off + 0x28) & !63;
            if desc == 0 { continue; }
            self.bytes.clear();
            let mut result = 1 << 13; // invalid or unterminated LLI
            for _ in 0..64 {
                let mut data = [0; 40];
                if memory.read(desc, &mut data).is_err() { result = 1 << 9; break; }
                let words: [u32; 10] = std::array::from_fn(|i| u32::from_le_bytes(data.as_chunks::<4>().0[i]));
                let [src, 0, BRIDGE_FIFO, 0, block, _, next, 0, ctl, hi] = words else { break };
                if hi >> 31 == 0 || ctl & (1 << 4) != 0 { break; }
                let size = (((block & 0x3f_ffff) + 1) as usize) << (ctl >> 8 & 7);
                let start = self.bytes.len();
                if size > expected - start { break; }
                self.bytes.resize(start + size, 0);
                if memory.read(src, &mut self.bytes[start..]).is_err() { result = 1 << 5; break; }
                if memory.write(desc.wrapping_add(36), &(hi & !(1 << 31)).to_le_bytes()).is_err() { result = 1 << 10; break; }
                if hi & (1 << 30) != 0 {
                    if self.bytes.len() == expected { result = 1 << 1; frame = true; }
                    break;
                }
                desc = next & !63;
                if desc == 0 { break; }
            }
            self.enabled &= !(1 << channel);
            self.raw[channel] |= result;
            if frame { break; }
        }
        frame
    }
}
impl Device for DwDma {
    fn read(&mut self, off: u32) -> u32 {
        match off {
            0x18 => self.enabled,
            0x30 => (0..4).fold(0, |mask, i| mask | (u32::from(self.status(i) != 0) << i)),
            0x58 => 0, // reset completes immediately
            0x188..=0x488 if off & 255 == 0x88 => self.status(((off - 0x188) / 256) as usize),
            _ => self.ram.read(off),
        }
    }
    fn write(&mut self, off: u32, value: u32) -> WriteEffect {
        match off {
            0x18 => {
                let write_mask = value >> 8 & 15;
                self.enabled = (self.enabled & !write_mask) | (value & write_mask);
            }
            0x58 if value & 1 != 0 => *self = Self::default(),
            0x198..=0x498 if off & 255 == 0x98 => self.raw[((off - 0x198) / 256) as usize] &= !value,
            _ => self.ram.write(off, value),
        }
        WriteEffect::NONE
    }
    fn irq_sources(&self) -> u64 { u64::from(self.ram.read(0x10) & 2 != 0 && (0..4).any(|i| self.signalled(i))) }
}
