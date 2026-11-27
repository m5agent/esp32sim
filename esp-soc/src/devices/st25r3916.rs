//! ST25R3916 NFC-A reader SPI and ISO/IEC 14443-3 Type A UID selection.
//! ST DS12484, sections 4.2 and 4.4–4.5 (SPI modes, commands and registers).
use std::collections::VecDeque;
#[path = "st25r3916/classic.rs"]
mod classic;
use classic::ClassicTag;

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
    LoadTarget(usize),
    Read,
    Done,
}
pub struct St25r3916 {
    registers: [[u8; 64]; 2],
    fifo: VecDeque<u8>,
    phase: Phase,
    uid: Vec<u8>,
    tag_present: bool,
    classic: Option<Box<ClassicTag>>,
    pending: Option<(u64, Vec<u8>, u8)>,
    fifo_tail_bits: u8,
    halted: bool,
    selected: bool,
    type2: [u8; 80],
    passive: [u8; 15],
    target_status: u8,
    external_field: bool,
    target_reply: Option<Vec<u8>>,
    pub uid_reads: u64,
}
impl Default for St25r3916 {
    fn default() -> Self {
        Self { registers: [[0; 64]; 2], fifo: VecDeque::new(), phase: Phase::Header,
               uid: Vec::new(), tag_present: false, classic: None, pending: None, fifo_tail_bits: 0, halted: false, selected: false, type2: [0; 80],
               passive: [0; 15], target_status: 0, external_field: false, target_reply: None, uid_reads: 0 }
    }
}
impl St25r3916 {
    pub fn reset(&mut self) {
        self.registers = [[0; 64]; 2];
        self.fifo.clear();
        self.fifo_tail_bits = 0;
        self.pending = None;
        self.halted = false;
        self.selected = false;
        self.passive = [0; 15];
        self.target_status = 0;
        self.target_reply = None;
        if let Some(tag) = &mut self.classic { tag.reselect(); }
        self.deselect();
    }
    pub fn set_tag(&mut self, uid: Vec<u8>) {
        self.tag_present = !uid.is_empty();
        if self.tag_present && self.uid != uid {
            self.type2 = [0; 80];
            self.classic = if let Ok(bytes) = <[u8; 4]>::try_from(uid.as_slice()) {
                Some(Box::new(ClassicTag::new(bytes)))
            } else { None };
            if let &[a, b, c, d, e, f, g] = uid.as_slice() {
                self.type2[..12].copy_from_slice(&[a, b, c, 0x88 ^ a ^ b ^ c, d, e, f, g,
                                                   d ^ e ^ f ^ g, 0x48, 0, 0]);
            }
            self.uid = uid;
        }
        self.halted = false;
        self.selected = false;
        if let Some(tag) = &mut self.classic { tag.reselect(); }
        self.fifo.clear();
        self.fifo_tail_bits = 0;
        self.pending = None;
    }
    pub fn deselect(&mut self) { self.phase = Phase::Header; }
    pub fn irq(&self) -> bool {
        (0..4).any(|i| self.registers[0][0x1a + i] & !self.registers[0][0x16 + i] != 0)
    }
    pub fn deadline(&self) -> Option<u64> { self.pending.as_ref().map(|(due, _, _)| *due) }
    pub fn advance(&mut self, now: u64) {
        if self.deadline().is_some_and(|due| now >= due) {
            if let Some((_, data, last_bits)) = self.pending.take() {
                if data.is_empty() {
                    self.registers[0][0x1b] |= 0x40; // Timer expired without a response.
                } else {
                    self.fifo = data.into();
                    self.fifo_tail_bits = last_bits;
                    self.registers[0][0x1a] |= 0x30; // RX start and RX end.
                }
            }
        }
    }
    fn cascade(uid: &[u8], command: u8) -> Option<[u8; 5]> {
        let uid = match (command, uid) {
            (0x93, &[a, b, c, d]) => [a, b, c, d],
            (0x93, &[a, b, c, _, _, _, _]) => [0x88, a, b, c],
            (0x95, &[_, _, _, a, b, c, d]) => [a, b, c, d],
            _ => return None,
        };
        Some([uid[0], uid[1], uid[2], uid[3], uid.into_iter().fold(0, |a, b| a ^ b)])
    }
    pub fn set_external_field(&mut self, enabled: bool) {
        if enabled != self.external_field {
            self.external_field = enabled;
            self.registers[0][0x1b] |= if enabled { 0x10 } else { 0x08 };
            self.target_status = u8::from(enabled);
            self.target_reply = None;
        }
    }

    fn target_mode(&self) -> bool { self.registers[0][3] & 0x80 != 0 }

    /// Pass the decoded on-air reader frame to the passive target path.
    /// The IC answers selection from its target config memory before enabling firmware RX.
    pub fn reader_frame(&mut self, now: u64, cycles_per_us: u64, frame: &[u8]) -> Option<Vec<u8>> {
        if !self.external_field || !self.target_mode() || self.registers[0][2] & 0xc0 != 0xc0 {
            return None;
        }
        let auto = self.registers[0][8] & 1 == 0;
        let uid_len = if self.registers[0][0x0a] & 0x30 == 0x10 { 7 } else { 4 };
        let uid = &self.passive[..uid_len];
        match frame {
            [0x26 | 0x52] if auto && self.target_status == 1 => {
                self.registers[0][0x1b] |= 1; // Initiator bitrate recognized.
                self.registers[0][0x1d] |= 0x10; // Passive target RX complete.
                self.target_status = 2;
                Some(self.passive[10..12].to_vec())
            }
            [sel, 0x20] if auto && matches!((self.target_status, sel), (2, 0x93) | (3, 0x95)) =>
                Self::cascade(uid, *sel).map(|data| data.to_vec()),
            [sel, 0x70, data @ ..] if auto && data.len() == 5 &&
                matches!((self.target_status, sel), (2, 0x93) | (3, 0x95)) &&
                Self::cascade(uid, *sel).is_some_and(|bytes| bytes == data) => {
                    let more = uid_len == 7 && *sel == 0x93;
                    self.target_status = if more { 3 } else { 5 };
                    if !more { self.registers[0][0x1d] |= 1; } // Passive target transitioned to Active.
                    let mut response = vec![self.passive[if *sel == 0x93 { 12 } else { 13 }]];
                    append_crc_a(&mut response);
                    Some(response)
                }
            payload if self.target_status == 5 && (1..=256).contains(&payload.len()) => {
                let mut received = payload.to_vec();
                append_crc_a(&mut received);
                self.target_reply = None;
                self.pending = Some((now.saturating_add(250 * cycles_per_us), received, 0));
                None
            }
            _ => None,
        }
    }

    pub fn take_target_reply(&mut self) -> Option<Vec<u8>> { self.target_reply.take() }

    fn command(&mut self, command: u8, now: u64, cycles_per_us: u64) {
        if (0xc4..=0xc7).contains(&command) { self.registers[0][0x1a] |= 0x08; }
        match command {
            0xc1 => self.reset(),
            0xc2 | 0xdb => { self.fifo.clear(); self.fifo_tail_bits = 0; self.pending = None; }
            0xc6 | 0xc7 if self.tag_present && (!self.halted || command == 0xc7) => {
                self.halted = false;
                self.selected = false;
                if let Some(tag) = &mut self.classic { tag.reselect(); }
                let atqa = if self.uid.len() == 7 { 0x44 } else { 0x04 };
                self.pending = Some((now.saturating_add(250 * cycles_per_us), vec![atqa, 0], 0));
            }
            0xc6 | 0xc7 => self.pending = Some((now.saturating_add(2500 * cycles_per_us), Vec::new(), 0)),
            0xcd => self.target_status = u8::from(self.external_field),
            0xce => self.target_status = if self.external_field { 9 } else { 0 },
            0xc4 | 0xc5 if self.target_mode() => {
                let mut data: Vec<_> = self.fifo.drain(..).collect();
                if self.external_field && self.target_status == 5 && !data.is_empty() {
                    if command == 0xc4 { append_crc_a(&mut data); }
                    self.target_reply = Some(data);
                }
            }
            0xc4 | 0xc5 => {
                let frame: Vec<_> = self.fifo.drain(..).collect();
                if self.halted || !self.tag_present { return; }
                if let Some(tag) = self.classic.as_mut().filter(|_| self.selected) {
                    let classic_reply = match (command, frame.as_slice()) {
                        (0xc4, [cmd @ (0x60 | 0x61), block]) =>
                            Some(tag.begin_auth(*block, *cmd == 0x61, false).map(|data| (data, 0))),
                        (0xc5, encrypt) if self.registers[0][5] & 0x80 != 0 => {
                            let bits = u16::from_be_bytes([self.registers[0][0x22], self.registers[0][0x23]]);
                            Some(tag.stream(encrypt, usize::from(bits)))
                        }
                        _ => None,
                    };
                    if let Some(response) = classic_reply {
                        self.halted = tag.halted;
                        self.pending = Some(match response {
                            Some((data, bits)) => (now.saturating_add(250 * cycles_per_us), data, bits),
                            None => (now.saturating_add(2500 * cycles_per_us), Vec::new(), 0),
                        });
                        return;
                    }
                }
                let response = match (command, frame.as_slice()) {
                    (0xc5, [sel, 0x20]) => Self::cascade(&self.uid, *sel).map(|uid| uid.to_vec()),
                    (0xc4, [sel, 0x70, bytes @ ..]) if Self::cascade(&self.uid, *sel).is_some_and(|uid| uid == bytes) => {
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
                    // MF0ULX1 §§8.5, 9.5: fast read is page-bounded and a write returns a
                    // four-bit ACK. The synthetic tag exposes only its OTP and user pages for writes.
                    (0xc4, [0x3a, first, last]) if self.selected && self.uid.len() == 7
                        && first <= last && *last < 20 => {
                        let start = usize::from(*first) * 4;
                        let end = (usize::from(*last) + 1) * 4;
                        Some(self.type2[start..end].to_vec())
                    }
                    (0xc4, [0xa2, page, bytes @ ..]) if self.selected && self.uid.len() == 7 && bytes.len() == 4 => {
                        if !(3..16).contains(page) {
                            Some(vec![0]) // Invalid page NAK.
                        } else {
                            let start = usize::from(*page) * 4;
                            for (old, new) in self.type2[start..start + 4].iter_mut().zip(bytes) {
                                *old = if *page == 3 { *old | *new } else { *new };
                            }
                            Some(vec![0x0a])
                        }
                    }
                    _ => None,
                };
                self.pending = Some(match response {
                    Some(mut data) => {
                        // DS12484 §2.2.13: the decoder keeps received CRC bytes in the FIFO.
                        let last_bits = if frame.first() == Some(&0xa2) { 4 } else { 0 };
                        if command == 0xc4 && last_bits == 0 { append_crc_a(&mut data); }
                        (now.saturating_add(250 * cycles_per_us), data, last_bits)
                    }
                    None => (now.saturating_add(2500 * cycles_per_us), Vec::new(), 0),
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
                    0xa0 => Phase::LoadTarget(0),
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
                        (0, 0x1f) => ((self.fifo.len() >> 8) as u8) << 6 | self.fifo_tail_bits << 1,
                        (0, 0x21) => self.target_status,
                        (0, 0x31) => (self.registers[0][2] & 0x80) >> 3 | u8::from(self.external_field) << 6,
                        (0, 0x3f) => 0x2a,
                        _ => old,
                    };
                }
                self.registers[bank][addr] = byte;
                if bank == 0 && addr == 2 && (byte & !old & 0x80 != 0) { self.registers[0][0x1a] |= 0x80; }
            }
            Phase::LoadTarget(offset) => {
                if let Some(slot) = self.passive.get_mut(offset) {
                    *slot = byte;
                    self.phase = Phase::LoadTarget(offset + 1);
                }
            }
            Phase::Load => { if self.fifo.len() < 512 { self.fifo.push_back(byte); } }
            Phase::Read => {
                let byte = self.fifo.pop_front().unwrap_or(0);
                if self.fifo.is_empty() { self.fifo_tail_bits = 0; }
                return byte;
            }
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
    fn target_selection_uses_config_and_delivers_read_frames() {
        let mut chip = St25r3916::default();
        spi(&mut chip, &[2, 0xc0, 0xc8]);
        spi(&mut chip, &[0xa0, 4, 1, 2, 3, 4, 5, 6, 0, 0, 0, 0x44, 0, 4, 0, 0]);
        spi(&mut chip, &[0x0a, 0x10]);
        assert!(chip.reader_frame(0, 1, &[0x26]).is_none());
        chip.set_external_field(true);
        assert_eq!(spi(&mut chip, &[0x5b, 0])[1], 0x10);
        assert_eq!(chip.reader_frame(0, 1, &[0x26]).unwrap(), [0x44, 0]);
        assert_eq!(spi(&mut chip, &[0x5d, 0])[1], 0x10);
        for (select, expected_sak) in [(0x93, 4), (0x95, 0)] {
            let data = chip.reader_frame(0, 1, &[select, 0x20]).unwrap();
            let frame = [vec![select, 0x70], data].concat();
            assert_eq!(chip.reader_frame(0, 1, &frame).unwrap()[0], expected_sak);
        }
        assert_eq!(spi(&mut chip, &[0x5d, 0])[1], 1);
        assert!(chip.reader_frame(0, 1, &[0x30, 0]).is_none());
        chip.advance(250);
        assert_eq!(spi(&mut chip, &[0x5a, 0])[1] & 0x30, 0x30);
        assert_eq!(spi(&mut chip, &[0x9f, 0, 0, 0, 0])[1..], [0x30, 0, 2, 0xa8]);
        spi(&mut chip, &[0x80, 1, 2, 3]);
        spi(&mut chip, &[0xc4]);
        assert_eq!(chip.take_target_reply().unwrap()[..3], [1, 2, 3]);
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
        for (page, data, ack) in [(3, [0xe1, 0x10, 6, 0], 0x0a), (3, [0; 4], 0x0a),
                                  (4, [1, 2, 3, 4], 0x0a), (0, [0; 4], 0)] {
            spi(&mut chip, &[0x80, 0xa2, page, data[0], data[1], data[2], data[3]]);
            spi(&mut chip, &[0xc4]);
            chip.advance(250);
            assert_eq!(spi(&mut chip, &[0x5e, 0, 0])[1..], [1, 8]);
            assert_eq!(spi(&mut chip, &[0x9f, 0])[1], ack);
        }
        spi(&mut chip, &[0x80, 0x3a, 3, 4]);
        spi(&mut chip, &[0xc4]);
        chip.advance(250);
        assert_eq!(spi(&mut chip, &[0x5e, 0, 0])[1..], [10, 0]);
        assert_eq!(spi(&mut chip, &[0x9f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])[1..9],
                   [0xe1, 0x10, 6, 0, 1, 2, 3, 4]);
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
