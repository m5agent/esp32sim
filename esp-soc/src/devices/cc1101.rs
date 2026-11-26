//! TI CC1101 packet SPI, SWRS061I §§10, 15 and 27–29:
//! burst/status addressing, strobes, MARCSTATE, GDO0 and 64-byte FIFOs.
use std::collections::VecDeque;

const IDLE: u8 = 0x01;
const RX: u8 = 0x0d;
const TX: u8 = 0x13;
const FSTXON: u8 = 0x12;

pub struct Cc1101 {
    registers: [u8; 0x2f],
    pa: [u8; 8],
    tx: Vec<u8>,
    rx: VecDeque<u8>,
    header: Option<u8>,
    offset: usize,
    state: u8,
    tx_due: Option<u64>,
    packet_ready: bool,
}
impl Default for Cc1101 {
    fn default() -> Self {
        let mut registers = [0; 0x2f];
        registers[..9].copy_from_slice(&[0x29, 0x2e, 0x3f, 0x07, 0xd3, 0x91, 0xff, 0x04, 0x45]);
        Self { registers, pa: [0; 8], tx: Vec::new(), rx: VecDeque::new(), header: None,
               offset: 0, state: IDLE, tx_due: None, packet_ready: false }
    }
}
impl Cc1101 {
    pub fn deselect(&mut self) { self.header = None; self.offset = 0; }
    pub fn deadline(&self) -> Option<u64> { self.tx_due }
    pub fn gdo0(&self) -> bool {
        self.packet_ready && !self.rx.is_empty() && self.registers[2] & 0x3f == 1
    }
    pub fn channel(&self) -> [u8; 10] {
        // Frequency, modem format/rate, and sync word.
        let r = &self.registers;
        [r[0x0d], r[0x0e], r[0x0f], r[0x10] & 0xf, r[0x11], r[0x12] & 0x77,
         r[4], r[5], 0, 0]
    }
    fn status(&self) -> u8 {
        match self.state { RX => 0x10, TX => 0x20, FSTXON => 0x30, _ => 0 }
    }
    fn strobe(&mut self, cmd: u8, now: u64, cycles_per_ms: u64) {
        match cmd {
            0x30 => *self = Self::default(),
            0x31 => self.state = FSTXON,
            0x34 => self.state = RX,
            0x35 => { self.state = TX; self.tx_due = Some(now.saturating_add(8 * cycles_per_ms)); }
            0x36 | 0x39 => { self.state = IDLE; self.tx_due = None; }
            0x3a => { self.rx.clear(); self.packet_ready = false; }
            0x3b => self.tx.clear(),
            _ => {}
        }
    }
    pub fn exchange(&mut self, byte: u8, now: u64, cycles_per_ms: u64) -> u8 {
        let Some(header) = self.header else {
            if (0x30..=0x3d).contains(&byte) { self.strobe(byte, now, cycles_per_ms); }
            self.header = Some(byte);
            return self.status();
        };
        let base = header & 0x3f;
        let burst = header & 0x40 != 0;
        let read = header & 0x80 != 0;
        let offset = self.offset;
        self.offset = self.offset.saturating_add(1);
        if offset != 0 && !burst { return self.status(); }
        if base == 0x3f {
            if read { return self.rx.pop_front().unwrap_or(0); }
            if self.tx.len() < 64 { self.tx.push(byte); }
        } else if base == 0x3e {
            let index = offset % 8;
            if read { return self.pa[index]; }
            self.pa[index] = byte;
        } else if read && burst && base >= 0x30 {
            return match base {
                0x30 => 0, 0x31 => 0x14, 0x33 => 0xa8, 0x34 => 0xc4,
                0x35 => self.state, 0x38 => if self.packet_ready { 0x80 } else { 0 },
                0x3a => self.tx.len() as u8, 0x3b => self.rx.len() as u8, _ => 0,
            };
        } else if let Some(register) = self.registers.get_mut(usize::from(base).saturating_add(offset)) {
            if read { return *register; }
            *register = byte;
        }
        self.status()
    }
    pub fn advance(&mut self, now: u64) -> Option<Vec<u8>> {
        if !self.tx_due.is_some_and(|due| now >= due) { return None; }
        self.tx_due = None;
        self.state = mode(self.registers[0x17]);
        let tx = std::mem::take(&mut self.tx);
        match self.registers[8] & 3 {
            0 => tx.get(..self.registers[6] as usize).map(<[u8]>::to_vec),
            1 => tx.first().and_then(|&len| tx.get(1..1 + usize::from(len))).map(<[u8]>::to_vec),
            _ => None,
        }
    }
    pub fn receive(&mut self, data: &[u8]) -> bool {
        if self.state != RX || !self.rx.is_empty() || data.len() > usize::from(self.registers[6]) { return false; }
        let length_mode = self.registers[8] & 3;
        let append = self.registers[7] & 4 != 0;
        if data.len() + usize::from(length_mode == 1) + 2 * usize::from(append) > 64 ||
            !matches!(length_mode, 0..=1) || (length_mode == 0 && data.len() != usize::from(self.registers[6])) {
            return false;
        }
        if length_mode == 1 { self.rx.push_back(data.len() as u8); }
        self.rx.extend(data);
        if append { self.rx.extend([0xc4, 0xa8]); }
        self.packet_ready = true;
        self.state = mode(self.registers[0x17] >> 2);
        true
    }
}
fn mode(value: u8) -> u8 { [IDLE, FSTXON, TX, RX][usize::from(value & 3)] }

#[cfg(test)]
mod tests {
    use super::*;
    fn spi(chip: &mut Cc1101, bytes: &[u8]) -> Vec<u8> {
        let reply = bytes.iter().map(|&b| chip.exchange(b, 0, 1)).collect();
        chip.deselect();
        reply
    }
    #[test]
    fn strobes_fifo_and_status() {
        let mut tx = Cc1101::default();
        let mut rx = Cc1101::default();
        spi(&mut tx, &[0x31]);
        assert_eq!(spi(&mut tx, &[0xf5, 0])[1], FSTXON);
        spi(&mut tx, &[0x3f, 3]);
        spi(&mut tx, &[0x7f, b't', b'x', b'!']);
        spi(&mut tx, &[0x35]);
        assert_eq!(spi(&mut tx, &[0xf5, 0])[1], TX);
        assert_eq!(tx.advance(7), None);
        let data = tx.advance(8).unwrap();
        assert_eq!(spi(&mut tx, &[0xf5, 0])[1], IDLE);
        assert!(!rx.receive(&data));
        spi(&mut rx, &[2, 1]);
        spi(&mut rx, &[0x34]);
        assert!(rx.receive(&data));
        assert!(rx.gdo0());
        assert_eq!(spi(&mut rx, &[0xfb, 0])[1], 6);
        assert_eq!(spi(&mut rx, &[0xff, 0, 0, 0, 0, 0, 0])[1..], [3, b't', b'x', b'!', 0xc4, 0xa8]);
        assert!(!rx.gdo0());
    }
}
