//! In-memory MIFARE Classic 1K tag for the virtual NFC-A air link.
//! Card state and access conditions follow NXP MF1S50YYX_V1 §§8.6–8.10.
//! The cipher matches M5Unit-NFC 0.1.2's MIT-licensed MIFARE Classic implementation.
//! SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
//! SPDX-License-Identifier: MIT
use super::append_crc_a;

const MASK48: u64 = (1 << 48) - 1;
const TAPS: u64 = 0x846b_50d4_1170;
const DEFAULT_TRAILER: [u8; 16] = [255, 255, 255, 255, 255, 255, 0xff, 0x07, 0x80, 0x69,
                                   255, 255, 255, 255, 255, 255];

struct Cipher(u64);

impl Cipher {
    fn new(key: [u8; 6]) -> Self {
        let mut bytes = [0; 8];
        for (dest, src) in bytes[2..].iter_mut().zip(key) { *dest = src.reverse_bits(); }
        Self(u64::from_be_bytes(bytes))
    }

    fn filter(&self) -> bool {
        fn fa([a, b, c, d]: [bool; 4]) -> bool { ((a || b) ^ (a && d)) ^ (c && ((a ^ b) || d)) }
        fn fb([a, b, c, d]: [bool; 4]) -> bool { ((a && b) || c) ^ ((a ^ b) && (c || d)) }
        let group = |base: usize| [6, 4, 2, 0].map(|offset| (self.0 >> (base + offset)) & 1 != 0);
        let [a, b, c, d, e] = [fa(group(32)), fb(group(24)), fb(group(16)), fa(group(8)), fb(group(0))];
        (a || ((b || e) && (d ^ e))) ^ ((a ^ (b && d)) && ((c ^ d) || (b && e)))
    }

    fn bit(&mut self, input: bool, encrypted: bool) -> u8 {
        let stream = self.filter();
        let feedback = ((self.0 & TAPS).count_ones() & 1 != 0) ^ input ^ (encrypted && stream);
        self.0 = (self.0 << 1 | u64::from(feedback)) & MASK48;
        u8::from(stream)
    }

    fn scramble(&mut self, byte: u8, bits: usize, feedback: bool) -> u8 {
        let mut out = 0;
        for i in 0..bits {
            let input = byte >> i & 1;
            out |= (input ^ self.bit(feedback && input != 0, feedback)) << i;
        }
        out
    }

    fn inject(&mut self, input: u32) -> u32 {
        let mut out = 0;
        for (i, byte) in input.to_be_bytes().into_iter().enumerate() {
            for b in 0..8 {
                out |= u32::from(self.bit(byte >> b & 1 != 0, false)) << ((3 - i) * 8 + b);
            }
        }
        out
    }

    fn decode(&mut self, stream: &[u8], bits: usize, feed_first: bool) -> Option<Vec<u8>> {
        if bits == 0 || !bits.is_multiple_of(9) || bits.div_ceil(8) != stream.len() { return None; }
        let bit = |i: usize| stream[i / 8] >> (i % 8) & 1;
        let mut decoded = Vec::new();
        for start in (0..bits).step_by(9) {
            let byte = (0..8).fold(0, |byte, offset| byte | bit(start + offset) << offset);
            let plain = self.scramble(byte, 8, feed_first && start < 36);
            let parity = u8::from(self.filter()) ^ ((plain.count_ones() & 1) as u8 ^ 1);
            if bit(start + 8) != parity { return None; }
            decoded.push(plain);
        }
        Some(decoded)
    }
}

fn shift_nonce(mut state: u32, bits: usize) -> u32 {
    let mut output = 0;
    for i in 0..bits {
        let feedback = (state >> 16 ^ state >> 18 ^ state >> 19 ^ state >> 21) & 1;
        if i >= bits - 32 { output |= (state & 1) << (i - (bits - 32)); }
        state = state >> 1 | feedback << 31;
    }
    output
}

enum Pending {
    Write(u8),
    Value(u8, u8),
}

struct Session {
    cipher: Cipher,
    sector: u8,
    key_b: bool,
    nonce: Option<u32>,
}

pub(super) struct ClassicTag {
    uid: [u8; 4],
    memory: [[u8; 16]; 64],
    next_nonce: u32,
    session: Option<Session>,
    pending: Option<Pending>,
    transfer: Option<i32>,
    pub halted: bool,
}

impl ClassicTag {
    pub(super) fn new(uid: [u8; 4]) -> Self {
        let mut memory = [[0; 16]; 64];
        for trailer in memory[3..].iter_mut().step_by(4) { *trailer = DEFAULT_TRAILER; }
        memory[0][..4].copy_from_slice(&uid);
        memory[0][4] = uid.iter().fold(0, |bcc, byte| bcc ^ byte);
        memory[0][5..8].copy_from_slice(&[8, 4, 0]);
        Self { uid, memory, next_nonce: 0x2816_dc6e, session: None, pending: None,
            transfer: None, halted: false }
    }

    pub(super) fn reselect(&mut self) {
        self.session = None;
        self.pending = None;
        self.transfer = None;
        self.halted = false;
    }

    pub(super) fn begin_auth(&mut self, block: u8, key_b: bool, nested: bool) -> Option<Vec<u8>> {
        let trailer = self.memory.get(usize::from(block | 3))?;
        let key = if key_b { &trailer[10..] } else { &trailer[..6] };
        let mut cipher = Cipher::new(key.try_into().ok()?);
        let nt = self.next_nonce;
        self.next_nonce = shift_nonce(nt.swap_bytes(), 64).swap_bytes();
        let output = cipher.inject(u32::from_be_bytes(self.uid) ^ nt);
        self.session = Some(Session { cipher, sector: block / 4, key_b, nonce: Some(nt) });
        self.pending = None;
        let nonce = if nested { nt ^ output } else { nt };
        Some(nonce.to_be_bytes().to_vec())
    }

    fn access(&self, block: u8) -> Option<usize> {
        let data = self.memory.get(usize::from(block | 3))?;
        let c1 = data[7] >> 4;
        let c2 = data[8] & 15;
        let c3 = data[8] >> 4;
        if data[6] & 15 != (!c1 & 15) || data[6] >> 4 != (!c2 & 15) || data[7] & 15 != (!c3 & 15) {
            return None;
        }
        let shift = block & 3;
        Some(usize::from(((c1 >> shift & 1) << 2) | ((c2 >> shift & 1) << 1) | (c3 >> shift & 1)))
    }

    fn allowed(&self, session: &Session, cmd: u8, block: u8) -> bool {
        if block / 4 != session.sector || block == 0 && cmd != 0x30 { return false; }
        let Some(access) = self.access(block) else { return false };
        if session.key_b && self.access(block | 3).is_some_and(|mode| matches!(mode, 0..=2)) {
            return false; // Key B is readable data in the transport configuration.
        }
        if block & 3 == 3 {
            let required = match cmd {
                0x30 => [1, 1, 1, 3, 3, 3, 3, 3][access],
                0xa0 => [0, 1, 0, 2, 0, 0, 0, 0][access],
                _ => 0,
            };
            return required & if session.key_b { 2 } else { 1 } != 0;
        }
        let keys = match cmd {
            0x30 => [3, 3, 3, 2, 3, 2, 3, 0][access],
            0xa0 => [3, 0, 0, 2, 2, 0, 2, 0][access],
            0xc1 => [3, 0, 0, 0, 0, 0, 2, 0][access],
            0xc0 | 0xc2 | 0xb0 => [3, 3, 0, 0, 0, 0, 3, 0][access],
            _ => 0,
        };
        keys & if session.key_b { 2 } else { 1 } != 0
    }

    fn nack(&mut self, session: &mut Session) -> (Vec<u8>, u8) {
        self.halted = true;
        self.pending = None;
        (vec![session.cipher.scramble(0, 4, false)], 4)
    }

    pub(super) fn stream(&mut self, frame: &[u8], bits: usize) -> Option<(Vec<u8>, u8)> {
        let mut session = self.session.take()?;
        if let Some(nt) = session.nonce.take() {
            let payload = session.cipher.decode(frame, bits, true)?;
            if payload.len() != 8 || payload[4..] != shift_nonce(nt.swap_bytes(), 96).to_le_bytes() {
                self.halted = true;
                return None;
            }
            let response = shift_nonce(nt.swap_bytes(), 128).to_le_bytes()
                .map(|byte| session.cipher.scramble(byte, 8, false));
            self.session = Some(session);
            return Some((response.to_vec(), 0));
        }
        let mut data = session.cipher.decode(frame, bits, false)?;
        if data.len() < 3 { return None; }
        let checksum = data.split_off(data.len() - 2);
        let mut checked = data.clone();
        append_crc_a(&mut checked);
        if checked[data.len()..] != checksum { return Some(self.nack(&mut session)); }
        if let [cmd @ (0x60 | 0x61), block] = data.as_slice() {
            return self.begin_auth(*block, *cmd == 0x61, true).map(|bytes| (bytes, 0));
        }
        if data == [0x50, 0] { self.halted = true; return None; }

        let answer = match (self.pending.take(), data.as_slice()) {
            (Some(Pending::Write(block)), bytes) if bytes.len() == 16 => {
                self.memory[usize::from(block)].copy_from_slice(bytes);
                Some((vec![0x0a], 4))
            }
            (Some(Pending::Value(cmd, block)), bytes) if bytes.len() == 4 => {
                let arg = i32::from_le_bytes(bytes.try_into().ok()?);
                let original = i32::from_le_bytes(self.memory[usize::from(block)][..4].try_into().ok()?);
                self.transfer = Some(match cmd {
                    0xc0 => original.wrapping_sub(arg),
                    0xc1 => original.wrapping_add(arg),
                    _ => original,
                });
                None
            }
            (None, [cmd, block]) if self.allowed(&session, *cmd, *block) => match cmd {
                0x30 => {
                    let mut bytes = self.memory[usize::from(*block)].to_vec();
                    if block & 3 == 3 {
                        bytes[..6].fill(0);
                        if !matches!(self.access(*block), Some(0..=2)) { bytes[10..].fill(0); }
                    }
                    append_crc_a(&mut bytes);
                    Some((bytes, 0))
                }
                0xa0 => { self.pending = Some(Pending::Write(*block)); Some((vec![0x0a], 4)) }
                0xc0..=0xc2 => { self.pending = Some(Pending::Value(*cmd, *block)); Some((vec![0x0a], 4)) }
                0xb0 if self.transfer.is_some() => {
                    let value = self.transfer.take()?.to_le_bytes();
                    let mut block_data = [0; 16];
                    block_data[12..].copy_from_slice(&[*block, !block, *block, !block]);
                    block_data[..4].copy_from_slice(&value);
                    block_data[4..8].copy_from_slice(&value.map(|byte| !byte));
                    block_data[8..12].copy_from_slice(&value);
                    self.memory[usize::from(*block)].copy_from_slice(&block_data);
                    Some((vec![0x0a], 4))
                }
                _ => return Some(self.nack(&mut session)),
            }
            _ => return Some(self.nack(&mut session)),
        };
        let answer = answer.map(|(mut bytes, bits)| {
            for byte in &mut bytes { *byte = session.cipher.scramble(*byte, if bits == 0 { 8 } else { bits }, false); }
            (bytes, bits as u8)
        });
        self.session = Some(session);
        answer
    }
}
