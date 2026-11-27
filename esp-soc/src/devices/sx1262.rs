//! Packet-level SX1262: Semtech SX1261/2 datasheet DS.SX1261-2.W.APP,
//! §§8.2, 9 and 13 (SPI framing, packet buffer, registers and commands).
use std::collections::BTreeMap;

const TX_DONE: u16 = 0x0001;
const RX_DONE: u16 = 0x0002;
const TIMEOUT: u16 = 0x0200;

// Common SX1261/2 silicon version exposed by SX1262 modules. RadioLib
// probes it with ReadRegister (hardware SPI transcript: RadioLib #683).
const VERSION_REGISTER: u16 = 0x0320;
const VERSION: &[u8; 16] = b"SX1261 V2D 2D02\0";

pub struct Sx1262 {
    registers: BTreeMap<u16, u8>,
    buffer: [u8; 256],
    command: Vec<u8>,
    clocks: usize,
    mode: u8,
    irq: u16,
    irq_enable: u16,
    dio1_mask: u16,
    tx_base: u8,
    rx_base: u8,
    length: u8,
    rx_length: u8,
    channel: [u8; 10],
    busy_until: u64,
    due: Option<u64>,
    continuous: bool,
}
impl Default for Sx1262 {
    fn default() -> Self {
        Self { registers: BTreeMap::new(), buffer: [0; 256], command: Vec::new(), clocks: 0,
               mode: 2, irq: 0, irq_enable: 0, dio1_mask: 0, tx_base: 0, rx_base: 0,
               length: 0, rx_length: 0, channel: [0; 10], busy_until: 0, due: None, continuous: false }
    }
}
impl Sx1262 {
    pub fn reset(&mut self, now: u64) {
        *self = Self::default();
        self.busy_until = now.saturating_add(1_000);
    }
    pub fn busy(&self, now: u64) -> bool { now < self.busy_until }
    pub fn dio1(&self) -> bool { self.irq & self.irq_enable & self.dio1_mask != 0 }
    pub fn deadline(&self, now: u64) -> Option<u64> {
        self.due.into_iter().chain((self.busy_until > now).then_some(self.busy_until)).min()
    }
    pub fn channel(&self) -> [u8; 10] {
        let mut channel = self.channel;
        channel[8] = self.register(0x0740);
        channel[9] = self.register(0x0741);
        channel
    }
    fn register(&self, addr: u16) -> u8 {
        if let Some(&byte) = addr.checked_sub(VERSION_REGISTER)
            .and_then(|offset| VERSION.get(usize::from(offset)))
        {
            return byte;
        }
        self.registers.get(&addr).copied().unwrap_or(match addr {
            0x0740 => 0x14,
            0x0741 => 0x24,
            0x08ac => 0x94,
            _ => 0,
        })
    }
    pub fn exchange(&mut self, byte: u8) -> u8 {
        let pos = self.clocks;
        self.clocks = self.clocks.saturating_add(1);
        if self.command.len() < 259 { self.command.push(byte); }
        let args = &self.command;
        let status = (self.mode << 4) | if self.irq & TX_DONE != 0 { 0x0c } else if self.irq & RX_DONE != 0 { 0x04 } else { 0 };
        match (args[0], pos) {
            (_, 0) => 0,
            (0x12, 2 | 3) => self.irq.to_be_bytes()[pos - 2],
            (0x13, 2) => self.rx_length,
            (0x13, 3) => self.rx_base,
            (0x14, 2 | 4) => 90, // RSSI -45 dBm
            (0x14, 3) => 32, // SNR +8 dB
            (0x17, 2 | 3) => 0,
            (0x1d, 4..) => self.register(u16::from_be_bytes([args[1], args[2]]).wrapping_add((pos - 4) as u16)),
            (0x1e, 3..) => self.buffer[args[1].wrapping_add((pos - 3) as u8) as usize],
            (0x11, 2) => 1, // LoRa packet type
            _ => status,
        }
    }
    /// NSS rises after the entire command, including any split FIFO data phase.
    pub fn deselect(&mut self, now: u64, cycles_per_us: u64) {
        self.clocks = 0;
        let args = self.command.as_slice();
        match args {
            [0x02, hi, lo, ..] => self.irq &= !u16::from_be_bytes([*hi, *lo]),
            [0x08, hi, lo, dio_hi, dio_lo, ..] => {
                self.irq_enable = u16::from_be_bytes([*hi, *lo]);
                self.dio1_mask = u16::from_be_bytes([*dio_hi, *dio_lo]);
            }
            [0x0d, hi, lo, data @ ..] => {
                let addr = u16::from_be_bytes([*hi, *lo]);
                for (i, &byte) in data.iter().enumerate() {
                    self.registers.insert(addr.wrapping_add(i as u16), byte);
                }
            }
            [0x0e, offset, data @ ..] => {
                for (i, &byte) in data.iter().take(256).enumerate() {
                    self.buffer[offset.wrapping_add(i as u8) as usize] = byte;
                }
            }
            [0x80, osc, ..] => { self.mode = if *osc == 0 { 2 } else { 3 }; self.due = None; }
            [0x82, a, b, c, ..] => {
                self.mode = 5;
                let units = u64::from(u32::from_be_bytes([0, *a, *b, *c]));
                self.continuous = units == 0xff_ffff;
                self.due = (units > 0 && !self.continuous).then(|| now.saturating_add(units * cycles_per_us * 125 / 8));
            }
            [0x83, _, _, _, ..] => { self.mode = 6; self.due = Some(now.saturating_add(2_000 * cycles_per_us)); }
            [0x84, ..] => { self.mode = 0; self.due = None; }
            [0x86, freq @ ..] if freq.len() >= 4 => self.channel[..4].copy_from_slice(&freq[..4]),
            [0x8b, params @ ..] if params.len() >= 4 => self.channel[4..8].copy_from_slice(&params[..4]),
            [0x8c, _, _, _, len, ..] => self.length = *len,
            [0x8f, tx, rx, ..] => { self.tx_base = *tx; self.rx_base = *rx; }
            // Error clearing, calibration, PA, TCXO and RF switch configuration
            // have no analog effects in this packet-level model.
            _ => {}
        }
        if args.first().is_some_and(|cmd| matches!(cmd, 0x07..=0x08 | 0x0d..=0x0e | 0x80..=0x9f | 0x02)) {
            self.busy_until = now.saturating_add(10 * cycles_per_us);
        }
        self.command.clear();
    }
    pub fn advance(&mut self, now: u64) -> Option<Vec<u8>> {
        if !self.due.is_some_and(|due| now >= due) { return None; }
        self.due = None;
        let tx = self.mode == 6;
        self.mode = 2;
        self.irq |= if tx { TX_DONE } else { TIMEOUT };
        tx.then(|| (0..self.length).map(|i| self.buffer[self.tx_base.wrapping_add(i) as usize]).collect())
    }
    pub fn receive(&mut self, data: &[u8]) -> bool {
        if self.mode != 5 || self.irq & RX_DONE != 0 || data.len() > 255 { return false; }
        self.rx_length = data.len() as u8;
        for (i, &byte) in data.iter().enumerate() { self.buffer[self.rx_base.wrapping_add(i as u8) as usize] = byte; }
        self.irq |= RX_DONE;
        if !self.continuous { self.mode = 2; self.due = None; }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn spi(chip: &mut Sx1262, now: u64, data: &[u8]) -> Vec<u8> {
        let reply = data.iter().map(|&b| chip.exchange(b)).collect();
        chip.deselect(now, 1);
        reply
    }

    #[test]
    fn read_version_after_reset() {
        let mut chip = Sx1262::default();
        chip.reset(500);
        let mut request = [0; 20];
        request[..3].copy_from_slice(&[0x1d, 0x03, 0x20]);
        assert_eq!(&spi(&mut chip, 1500, &request)[4..], b"SX1261 V2D 2D02\0");
    }

    #[test]
    fn split_fifo_phase_and_dio1() {
        let mut chip = Sx1262::default();
        spi(&mut chip, 0, &[0x08, 0, 3, 0, 3, 0, 0, 0, 0]);
        spi(&mut chip, 0, &[0x8f, 0xff, 0xfe]);
        spi(&mut chip, 0, &[0x8c, 0, 12, 0, 3, 1, 0]);
        for part in [&[0x0e, 0xff][..], b"one"] {
            for &byte in part { chip.exchange(byte); }
        }
        chip.deselect(0, 1);
        spi(&mut chip, 0, &[0x83, 0, 0, 0]);
        assert!(chip.busy(0));
        assert!(!chip.dio1());
        assert_eq!(chip.advance(1999), None);
        assert_eq!(chip.advance(2000).as_deref(), Some(&b"one"[..]));
        assert!(chip.dio1());
        assert!(!chip.receive(b"idle"));
        spi(&mut chip, 2000, &[0x02, 0, 1]);
        assert!(!chip.dio1());
        spi(&mut chip, 2000, &[0x82, 255, 255, 255]);
        assert!(chip.receive(b"two"));
        assert!(!chip.receive(b"overrun"));
        assert!(chip.dio1());
        assert_eq!(spi(&mut chip, 2000, &[0x13, 0, 0, 0])[2..], [3, 0xfe]);
        assert_eq!(spi(&mut chip, 2000, &[0x1e, 0xfe, 0, 0, 0, 0])[3..], *b"two");
        spi(&mut chip, 2000, &[0x02, 0, 2]);
        assert!(!chip.dio1());
        assert!(chip.receive(b"next"));
    }
}
