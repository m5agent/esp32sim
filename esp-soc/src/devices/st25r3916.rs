//! ST25R3916 NFC-A reader SPI and ISO/IEC 14443-3 Type A UID selection.
//! ST DS12484, sections 4.2 and 4.4–4.5 (SPI modes, commands and registers).
use std::collections::VecDeque;

fn append_crc_a(frame: &mut Vec<u8>) {
    let crc = frame.iter().fold(0x6363u16, |crc, &byte| {
        let low = (crc as u8) ^ byte;
        let data = u16::from(low ^ (low << 4));
        (crc >> 8) ^ (data << 8) ^ (data << 3) ^ (data >> 4)
    });
    frame.extend(crc.to_le_bytes());
}

#[derive(Default)]
enum Phase {
    #[default]
    Header,
    SpaceB,
    Register { bank: usize, addr: usize, read: bool },
    Load,
    Read,
    Done,
}
pub struct St25r3916 {
    registers: [[u8; 64]; 2],
    fifo: VecDeque<u8>,
    phase: Phase,
    uid: Vec<u8>,
    pending: Option<(u64, Vec<u8>)>,
    halted: bool,
    selected: bool,
    type2: [u8; 80],
    pub uid_reads: u64,
}
impl Default for St25r3916 {
    fn default() -> Self {
        Self { registers: [[0; 64]; 2], fifo: VecDeque::new(), phase: Phase::Header,
               uid: Vec::new(), pending: None, halted: false, selected: false, type2: [0; 80], uid_reads: 0 }
    }
}
impl St25r3916 {
    pub fn reset(&mut self) {
        self.registers = [[0; 64]; 2];
        self.fifo.clear();
        self.pending = None;
        self.halted = false;
        self.selected = false;
        self.deselect();
    }
    pub fn set_tag(&mut self, uid: Vec<u8>) {
        self.type2 = [0; 80];
        if let &[a, b, c, d, e, f, g] = uid.as_slice() {
            self.type2[..12].copy_from_slice(&[a, b, c, 0x88 ^ a ^ b ^ c, d, e, f, g, d ^ e ^ f ^ g, 0x48, 0, 0]);
        }
        self.uid = uid;
        self.halted = false;
        self.selected = false;
        self.fifo.clear();
        self.pending = None;
    }
    pub fn deselect(&mut self) { self.phase = Phase::Header; }
    pub fn irq(&self) -> bool {
        (0..4).any(|i| self.registers[0][0x1a + i] & !self.registers[0][0x16 + i] != 0)
    }
    pub fn deadline(&self) -> Option<u64> { self.pending.as_ref().map(|(due, _)| *due) }
    pub fn advance(&mut self, now: u64) {
        if self.deadline().is_some_and(|due| now >= due) {
            if let Some((_, data)) = self.pending.take() {
                if data.is_empty() {
                    self.registers[0][0x1b] |= 0x40; // Timer expired without a response.
                } else {
                    self.fifo = data.into();
                    self.registers[0][0x1a] |= 0x30; // RX start and RX end.
                }
            }
        }
    }
    fn cascade(&self, command: u8) -> Option<[u8; 5]> {
        let uid = match (command, self.uid.as_slice()) {
            (0x93, &[a, b, c, d]) => [a, b, c, d],
            (0x93, &[a, b, c, _, _, _, _]) => [0x88, a, b, c],
            (0x95, &[_, _, _, a, b, c, d]) => [a, b, c, d],
            _ => return None,
        };
        Some([uid[0], uid[1], uid[2], uid[3], uid.into_iter().fold(0, |a, b| a ^ b)])
    }
    fn command(&mut self, command: u8, now: u64, cycles_per_us: u64) {
        if (0xc4..=0xc7).contains(&command) { self.registers[0][0x1a] |= 0x08; }
        match command {
            0xc1 => self.reset(),
            0xc2 | 0xdb => { self.fifo.clear(); self.pending = None; }
            0xc6 | 0xc7 if !self.uid.is_empty() && (!self.halted || command == 0xc7) => {
                self.halted = false;
                self.selected = false;
                let atqa = if self.uid.len() == 7 { 0x44 } else { 0x04 };
                self.pending = Some((now.saturating_add(250 * cycles_per_us), vec![atqa, 0]));
            }
            0xc6 | 0xc7 => self.pending = Some((now.saturating_add(2500 * cycles_per_us), Vec::new())),
            0xc4 | 0xc5 => {
                let frame: Vec<_> = self.fifo.drain(..).collect();
                if self.halted { return; }
                let response = match (command, frame.as_slice()) {
                    (0xc5, [sel, 0x20]) => self.cascade(*sel).map(|uid| uid.to_vec()),
                    (0xc4, [sel, 0x70, bytes @ ..]) if self.cascade(*sel).is_some_and(|uid| uid == bytes) => {
                        let more = self.uid.len() == 7 && *sel == 0x93;
                        if !more { self.uid_reads += 1; self.selected = true; }
                        Some(vec![if more { 0x04 } else if self.uid.len() == 4 { 0x08 } else { 0 }])
                    }
                    (0xc4, [0x50, 0]) => { self.halted = true; self.selected = false; None }
                    (0xc4, [0x60]) if self.selected && self.uid.len() == 7 =>
                        Some(vec![0, 4, 3, 1, 1, 0, 0x0b, 3]), // Ultralight EV1 MF0UL11.
                    (0xc4, [0x30, page]) if self.selected && self.uid.len() == 7 && *page < 20 => {
                        let bytes = (0..16).map(|i| self.type2[(usize::from(*page) * 4 + i) % self.type2.len()]);
                        Some(bytes.collect())
                    }
                    _ => None,
                };
                self.pending = Some(match response {
                    Some(mut data) => {
                        // DS12484 §2.2.13: the decoder keeps received CRC bytes in the FIFO.
                        if command == 0xc4 { append_crc_a(&mut data); }
                        (now.saturating_add(250 * cycles_per_us), data)
                    }
                    None => (now.saturating_add(2500 * cycles_per_us), Vec::new()),
                });
            }
            _ => {}
        }
    }
    pub fn exchange(&mut self, byte: u8, now: u64, cycles_per_us: u64) -> u8 {
        match self.phase {
            Phase::Header => {
                self.phase = match byte {
                    0x80 => Phase::Load,
                    0x9f => Phase::Read,
                    0xfb => Phase::SpaceB,
                    0x00..=0x7f => Phase::Register { bank: 0, addr: usize::from(byte & 63), read: byte & 64 != 0 },
                    _ => { self.command(byte, now, cycles_per_us); Phase::Done }
                };
            }
            Phase::SpaceB => self.phase = Phase::Register { bank: 1, addr: usize::from(byte & 63), read: byte & 64 != 0 },
            Phase::Register { bank, addr, read } => {
                self.phase = Phase::Register { bank, addr: (addr + 1) & 63, read };
                let old = self.registers[bank][addr];
                if read {
                    return match (bank, addr) {
                        (0, 0x1a..=0x1d) => std::mem::take(&mut self.registers[bank][addr]),
                        (0, 0x1e) => self.fifo.len() as u8,
                        (0, 0x1f) => ((self.fifo.len() >> 8) as u8) << 6,
                        (0, 0x31) => (self.registers[0][2] & 0x80) >> 3,
                        (0, 0x3f) => 0x2a,
                        _ => old,
                    };
                }
                self.registers[bank][addr] = byte;
                if bank == 0 && addr == 2 && (byte & !old & 0x80 != 0) { self.registers[0][0x1a] |= 0x80; }
            }
            Phase::Load => { if self.fifo.len() < 512 { self.fifo.push_back(byte); } }
            Phase::Read => return self.fifo.pop_front().unwrap_or(0),
            Phase::Done => {}
        }
        0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn spi(chip: &mut St25r3916, data: &[u8]) -> Vec<u8> {
        let reply = data.iter().map(|&b| chip.exchange(b, 0, 1)).collect();
        chip.deselect();
        reply
    }

    #[test]
    fn injected_tag_anticollision_and_halt() {
        let mut chip = St25r3916::default();
        spi(&mut chip, &[0xc6]);
        chip.advance(250);
        assert_eq!(spi(&mut chip, &[0x5e, 0])[1], 0);
        chip.set_tag(vec![4, 1, 2, 3, 4, 5, 6]);
        spi(&mut chip, &[0xc6]);
        chip.advance(249);
        assert_eq!(spi(&mut chip, &[0x5e, 0])[1], 0);
        chip.advance(250);
        assert!(chip.irq());
        assert_eq!(spi(&mut chip, &[0x9f, 0, 0])[1..], [0x44, 0]);
        for (select, uid, sak) in [(0x93, [0x88, 4, 1, 2], 4), (0x95, [3, 4, 5, 6], 0)] {
            spi(&mut chip, &[0x80, select, 0x20]);
            spi(&mut chip, &[0xc5]);
            chip.advance(250);
            let bcc = uid.into_iter().fold(0, |a, b| a ^ b);
            assert_eq!(spi(&mut chip, &[0x9f, 0, 0, 0, 0, 0])[1..], [uid[0], uid[1], uid[2], uid[3], bcc]);
            spi(&mut chip, &[0x80, select, 0x70, uid[0], uid[1], uid[2], uid[3], bcc]);
            spi(&mut chip, &[0xc4]);
            chip.advance(250);
            assert_eq!(spi(&mut chip, &[0x5e, 0])[1], 3);
            let expected = if sak == 4 { [4, 0xda, 0x17] } else { [0, 0xfe, 0x51] };
            assert_eq!(spi(&mut chip, &[0x9f, 0, 0, 0])[1..], expected);
        }
        assert_eq!(chip.uid_reads, 1);
        spi(&mut chip, &[0x80, 0x50, 0]);
        spi(&mut chip, &[0xc4]);
        spi(&mut chip, &[0xc6]);
        chip.advance(250);
        assert_eq!(spi(&mut chip, &[0x5e, 0])[1], 0);
        spi(&mut chip, &[0xc7]);
        chip.advance(250);
        assert_eq!(spi(&mut chip, &[0x5e, 0])[1], 2);
    }
}
