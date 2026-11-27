//! MFRC522-compatible I²C frontend, NXP MFRC522 rev. 3.9 §§8–10.
//! One synthetic ISO14443-A Classic 1K tag. Authentication checks the sector's
//! configured key and UID directly. RF signaling and Crypto1 are outside this model.
use std::collections::VecDeque;
use esp_periph::i2c::I2cDevice;

const COMMAND: usize = 1;
const IRQ: usize = 4;
const DIV_IRQ: usize = 5;
const ERROR: usize = 6;
const STATUS2: usize = 8;
const FIFO: usize = 9;
const FIFO_LEVEL: usize = 10;
const CONTROL: usize = 12;
const FRAMING: usize = 13;
const TX_CONTROL: usize = 0x14;

struct ClassicTag {
    uid: [u8; 4],
    blocks: [[u8; 16]; 64],
    ready: bool,
    selected: bool,
    halted: bool,
    sector: Option<u8>,
    write: Option<u8>,
}

impl ClassicTag {
    fn new(uid: [u8; 4]) -> Self {
        let mut blocks = [[0; 16]; 64];
        for trailer in blocks.iter_mut().skip(3).step_by(4) {
            *trailer = [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x07, 0x80, 0x69, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff];
        }
        blocks[0][..4].copy_from_slice(&uid);
        blocks[0][4] = Self::bcc(uid);
        blocks[0][5..8].copy_from_slice(&[0x08, 0x04, 0x00]);
        Self { uid, blocks, ready: false, selected: false, halted: false, sector: None, write: None }
    }
    fn bcc(uid: [u8; 4]) -> u8 { uid.into_iter().reduce(|a, b| a ^ b).unwrap_or(0) }
    fn authenticate(&mut self, frame: &[u8]) -> bool {
        self.sector = None;
        let [key_type @ (0x60 | 0x61), block, rest @ ..] = frame else { return false };
        if !self.selected || *block >= 64 || rest.len() != 10 || rest[6..] != self.uid { return false; }
        let key_at = if *key_type == 0x60 { 0 } else { 10 };
        let trailer = &self.blocks[usize::from(*block | 3)];
        if rest[..6] != trailer[key_at..key_at + 6] { return false; }
        self.sector = Some(*block / 4);
        true
    }
    fn allowed(&self, block: u8) -> bool {
        self.selected && block < 64 && self.sector == Some(block / 4)
    }
    fn exchange(&mut self, frame: &[u8], partial_bits: u8) -> Option<(Vec<u8>, u8)> {
        if let Some(block) = self.write.take() {
            if frame.len() == 18 && crc_valid(frame) {
                self.blocks[usize::from(block)].copy_from_slice(&frame[..16]);
                return Some((vec![0x0a], 4));
            }
        }
        let append_crc = |mut bytes: Vec<u8>| { bytes.extend(crc_a(&bytes)); (bytes, 0) };
        match frame {
            [0x26 | 0x52] if partial_bits == 7 && (!self.halted || frame[0] == 0x52) => {
                self.ready = true; self.halted = false; self.selected = false; self.sector = None;
                Some((vec![0x04, 0x00], 0))
            }
            [0x93, 0x20] if self.ready => {
                let mut bytes = self.uid.to_vec();
                bytes.push(Self::bcc(self.uid));
                Some((bytes, 0))
            }
            [0x93, 0x70, rest @ ..] if self.ready && rest.len() == 7
                && rest[..4] == self.uid && rest[4] == Self::bcc(self.uid) && crc_valid(frame) => {
                self.selected = true; self.ready = false;
                Some(append_crc(vec![0x08]))
            }
            [0x30, block, _, _] if self.allowed(*block) && crc_valid(frame) => {
                let mut data = self.blocks[usize::from(*block)];
                if *block % 4 == 3 { data[..6].fill(0); }
                Some(append_crc(data.to_vec()))
            }
            [0xa0, block, _, _] if *block != 0 && *block % 4 != 3 && self.allowed(*block) && crc_valid(frame) => {
                self.write = Some(*block);
                Some((vec![0x0a], 4))
            }
            [0x50, 0, _, _] if self.selected && crc_valid(frame) => {
                self.halted = true; self.selected = false; self.sector = None;
                None
            }
            _ => None,
        }
    }
}

fn crc_a(bytes: &[u8]) -> [u8; 2] {
    let mut crc = 0x6363u16;
    for byte in bytes {
        crc ^= u16::from(*byte);
        for _ in 0..8 { crc = (crc >> 1) ^ if crc & 1 != 0 { 0x8408 } else { 0 }; }
    }
    crc.to_le_bytes()
}
fn crc_valid(bytes: &[u8]) -> bool {
    let (data, crc) = bytes.split_at(bytes.len() - 2);
    crc_a(data) == crc
}

pub struct Mfrc522 {
    regs: [u8; 64],
    fifo: VecDeque<u8>,
    tag: Option<ClassicTag>,
    ptr: usize,
    first: bool,
    pub exchanges: u64,
}

impl Default for Mfrc522 {
    fn default() -> Self {
        let mut reader = Self { regs: [0; 64], fifo: VecDeque::new(), tag: None,
            ptr: 0, first: true, exchanges: 0 };
        reader.reset();
        reader
    }
}

impl Mfrc522 {
    pub const ADDRESS: u8 = 0x28;
    pub fn present(&mut self, uid: Option<[u8; 4]>) {
        self.tag = uid.map(ClassicTag::new);
        self.regs[STATUS2] &= !8;
    }
    fn reset(&mut self) {
        self.regs = [0; 64];
        self.regs[0x37] = 0x92;
        self.regs[0x11] = 0x3f;
        self.fifo.clear();
    }
    fn transfer(&mut self, authenticate: bool) {
        let frame: Vec<_> = self.fifo.drain(..).collect();
        self.regs[ERROR] = 0;
        self.regs[CONTROL] &= !7;
        // With TX1/TX2 disabled the tag receives no field.
        let mut response = None;
        if self.regs[TX_CONTROL] & 3 == 3 {
            if let Some(tag) = &mut self.tag {
                if authenticate {
                    self.regs[STATUS2] &= !8;
                    if tag.authenticate(&frame) {
                        self.regs[STATUS2] |= 8;
                        response = Some((Vec::new(), 0));
                    }
                } else {
                    if self.regs[STATUS2] & 8 == 0 { tag.sector = None; }
                    response = tag.exchange(&frame, self.regs[FRAMING] & 7);
                }
            }
        }
        if let Some((bytes, bits)) = response {
            self.fifo.extend(bytes);
            self.regs[CONTROL] |= bits;
            self.regs[IRQ] |= if authenticate { 0x10 } else { 0x30 };
            self.exchanges += 1;
        } else {
            self.regs[IRQ] |= 1; // TimerIRq completion for no response.
        }
        self.regs[COMMAND] = 0;
        self.regs[FRAMING] &= !0x80;
    }
    fn write_register(&mut self, reg: usize, value: u8) {
        match reg {
            IRQ | DIV_IRQ => {
                if value & 0x80 != 0 { self.regs[reg] |= value & 0x7f; }
                else { self.regs[reg] &= !value; }
            }
            FIFO if self.fifo.len() < 64 => self.fifo.push_back(value),
            FIFO => self.regs[ERROR] |= 0x10,
            FIFO_LEVEL => { if value & 0x80 != 0 { self.fifo.clear(); } }
            COMMAND => {
                self.regs[COMMAND] = value & 0x3f;
                match value & 15 {
                    3 => {
                        let [lo, hi] = crc_a(self.fifo.make_contiguous());
                        self.regs[0x21] = hi; self.regs[0x22] = lo;
                        self.regs[DIV_IRQ] |= 4;
                    }
                    0x0e => self.transfer(true),
                    0x0f => self.reset(),
                    _ => {}
                }
            }
            FRAMING => {
                self.regs[FRAMING] = value;
                if value & 0x80 != 0 && self.regs[COMMAND] & 15 == 0x0c { self.transfer(false); }
            }
            0x37 => {}
            _ => self.regs[reg] = value,
        }
    }
}

impl I2cDevice for Mfrc522 {
    fn start(&mut self, read: bool) -> bool { self.first = !read; true }
    fn write(&mut self, byte: u8) -> bool {
        if self.first { self.ptr = usize::from(byte & 0x3f); self.first = false; }
        else { self.write_register(self.ptr, byte); }
        true
    }
    fn read(&mut self) -> u8 {
        match self.ptr {
            FIFO => self.fifo.pop_front().unwrap_or(0),
            FIFO_LEVEL => self.fifo.len() as u8,
            _ => self.regs[self.ptr],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn transceive(reader: &mut Mfrc522, bytes: &[u8], bits: u8) -> Vec<u8> {
        reader.write_register(IRQ, 0x7f);
        reader.write_register(FIFO_LEVEL, 0x80);
        for &byte in bytes { reader.write_register(FIFO, byte); }
        reader.write_register(COMMAND, 0x0c);
        reader.write_register(FRAMING, 0x80 | bits);
        reader.fifo.iter().copied().collect()
    }

    #[test]
    fn no_card_and_two_card_uids_through_fifo() {
        let mut reader = Mfrc522::default();
        reader.write_register(TX_CONTROL, 3);
        assert!(transceive(&mut reader, &[0x26], 7).is_empty());
        assert_eq!(reader.regs[IRQ], 1);
        for uid in [[0x04, 1, 2, 3], [0x04, 9, 8, 7]] {
            reader.present(Some(uid));
            assert_eq!(transceive(&mut reader, &[0x26], 7), [4, 0]);
            let uid_response = transceive(&mut reader, &[0x93, 0x20], 0);
            assert_eq!(uid_response[..4], uid);
            let mut select = vec![0x93, 0x70];
            select.extend(uid_response);
            select.extend(crc_a(&select));
            let sak = transceive(&mut reader, &select, 0);
            assert_eq!(sak[0], 0x08);
            assert!(crc_valid(&sak));
        }
    }
}
