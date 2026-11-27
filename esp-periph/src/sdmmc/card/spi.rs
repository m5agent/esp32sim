//! SPI transport for the image-backed SDHC/SDXC card, per SD Physical Layer Simplified
//! Specification v9.10, chapter 7. Arduino-ESP32 3.2.5 libraries/SD/src/sd_diskio.cpp
//! enables CRC with CMD59, toggles CS across CMD55, and discards a stuff byte after CMD12.
use super::{crc7, CardState, SdCard, BLOCK, OCR_CCS, OCR_READY, OCR_VOLTAGE};
use std::collections::VecDeque;

const IDLE: u8 = 1;
const ILLEGAL: u8 = 1 << 2;
const CRC_ERROR: u8 = 1 << 3;
const ADDRESS_ERROR: u8 = 1 << 5;
const PARAM_ERROR: u8 = 1 << 6;
const DATA_START: u8 = 0xfe;
const WRITE_START: u8 = 0xfc;
const WRITE_STOP: u8 = 0xfd;

enum Phase {
    Command,
    Read { next: u64 },
    Write { lba: u64, multi: bool, receiving: bool },
}

pub struct SpiSdCard {
    card: SdCard,
    spi_mode: bool,
    crc_enabled: bool,
    command: [u8; 6],
    command_len: usize,
    response: VecDeque<u8>,
    phase: Phase,
    write_data: Vec<u8>,
    write_status: u8,
}

impl SpiSdCard {
    pub fn new(card: SdCard) -> Self {
        Self { card, spi_mode: false, crc_enabled: false, command: [0; 6], command_len: 0,
               response: VecDeque::with_capacity(BLOCK + 5), phase: Phase::Command,
               write_data: Vec::with_capacity(BLOCK + 2), write_status: 0 }
    }

    /// CS high releases MISO and discards partial packets. APP_CMD and programming
    /// survive deselection, as does the initialized state.
    pub fn deselect(&mut self) {
        self.command_len = 0;
        self.response.clear();
        self.phase = Phase::Command;
        self.write_data.clear();
    }

    pub fn power_off(&mut self) {
        self.deselect();
        self.card.reset();
        self.spi_mode = false;
        self.crc_enabled = false;
        self.write_status = 0;
    }

    pub fn report(&self) -> String {
        format!("[sd-spi] {} commands, {} blocks read, {} blocks written",
                self.card.commands, self.card.blocks_read, self.card.blocks_written)
    }

    fn r1(&self) -> u8 { if self.card.state == CardState::Idle { IDLE } else { 0 } }

    fn execute(&mut self) {
        let cmd = self.command[0] & 0x3f;
        let arg = u32::from_be_bytes([self.command[1], self.command[2], self.command[3], self.command[4]]);
        let check_crc = self.crc_enabled || matches!(cmd, 0 | 8);
        let bad_crc = check_crc && self.command[5] != (crc7(&self.command[..5]) << 1 | 1);
        if !self.spi_mode && (cmd != 0 || bad_crc) { return; }
        let r1 = self.r1();
        self.card.commands += 1;
        self.response.clear();
        // N_CR gap, or the stuff byte for CMD12 when interrupting a data transfer.
        self.response.push_back(0xff);
        if bad_crc {
            self.response.push_back(r1 | CRC_ERROR);
            return;
        }
        let app = std::mem::take(&mut self.card.app_cmd);
        self.phase = Phase::Command;
        match (app, cmd) {
            (_, 0) => {
                self.card.reset();
                self.spi_mode = true;
                self.crc_enabled = false;
                self.write_status = 0;
                self.response.push_back(IDLE);
            }
            (_, 8) if r1 == IDLE => {
                if arg >> 8 & 0xf != 1 { self.response.push_back(r1 | PARAM_ERROR); }
                else {
                    self.card.if_cond = true;
                    self.response.push_back(r1);
                    self.response.extend((arg & 0xfff).to_be_bytes());
                }
            }
            (_, 55) => {
                self.card.app_cmd = true;
                self.response.push_back(r1);
            }
            (true, 41) => {
                // SPI ACMD41 carries HCS, with no SD-mode voltage window.
                if r1 == IDLE && self.card.if_cond && arg & OCR_CCS != 0 {
                    if self.card.init_left == 0 { self.card.state = CardState::Tran; }
                    else { self.card.init_left -= 1; }
                }
                self.response.push_back(self.r1());
            }
            (_, 58) => {
                self.response.push_back(r1);
                let ocr = OCR_VOLTAGE | if r1 == 0 { OCR_CCS | OCR_READY } else { 0 };
                self.response.extend(ocr.to_be_bytes());
            }
            (_, 59) => {
                self.crc_enabled = arg & 1 != 0;
                self.response.push_back(r1);
            }
            _ if r1 == IDLE => self.response.push_back(r1 | ILLEGAL),
            (true, 23 | 42) => self.response.push_back(0),
            (true, 22) => {
                self.response.push_back(0);
                self.queue_data(&self.card.written.to_be_bytes());
            }
            (false, 9 | 10) => {
                self.response.push_back(0);
                let reg = if cmd == 9 { self.card.csd() } else { self.card.cid() };
                self.queue_data(&reg.to_be_bytes());
            }
            (false, 12) => self.response.push_back(0),
            (false, 13) => self.response.extend([0, std::mem::take(&mut self.write_status)]),
            (false, 16) => self.response.push_back(if arg == BLOCK as u32 { 0 } else { PARAM_ERROR }),
            (false, 17 | 18 | 24 | 25) if u64::from(arg) >= self.card.blocks => self.response.push_back(ADDRESS_ERROR),
            (false, 17 | 18) => {
                self.response.push_back(0);
                self.queue_block(u64::from(arg), cmd == 18);
            }
            (false, 24 | 25) => {
                self.response.push_back(0);
                self.card.written = 0;
                self.phase = Phase::Write { lba: u64::from(arg), multi: cmd == 25, receiving: false };
            }
            _ => self.response.push_back(ILLEGAL),
        }
    }

    fn queue_data(&mut self, bytes: &[u8]) {
        self.response.extend([0xff, DATA_START]);
        self.response.extend(bytes);
        self.response.extend(crc16(bytes).to_be_bytes());
    }

    fn queue_block(&mut self, lba: u64, multi: bool) {
        let mut block = [0; BLOCK];
        if lba >= self.card.blocks {
            self.response.extend([0xff, 0x08]); // out-of-range data error token
            self.phase = Phase::Command;
        } else if self.card.store.read(lba, &mut block).is_err() {
            self.response.extend([0xff, 0x01]); // general data error token
            self.phase = Phase::Command;
        } else {
            self.card.blocks_read += 1;
            self.queue_data(&block);
            self.phase = if multi { Phase::Read { next: lba + 1 } } else { Phase::Command };
        }
    }

    fn program_block(&mut self, lba: u64, multi: bool, now_ns: u64) {
        let data = <&[u8; BLOCK]>::try_from(&self.write_data[..BLOCK]).expect("one complete SD block");
        let expected = u16::from_be_bytes([self.write_data[BLOCK], self.write_data[BLOCK + 1]]);
        let result = if self.crc_enabled && crc16(data) != expected { 0x0b }
        else if lba >= self.card.blocks { self.write_status |= 0x80; 0x0d }
        else if self.card.store.write(lba, data).is_err() { self.write_status |= 0x04; 0x0d }
        else {
            self.card.blocks_written += 1;
            self.card.written += 1;
            self.card.busy_until = now_ns.saturating_add(self.card.config.prg_ns);
            0x05
        };
        self.response.push_back(result);
        self.write_data.clear();
        self.phase = if multi { Phase::Write { lba: lba + u64::from(result == 0x05), multi, receiving: false } }
                     else { Phase::Command };
    }

    /// First byte already queued for the next clock. A GPIO SPI master
    /// samples individual bits before the byte of MOSI has arrived.
    pub fn pending_miso(&self, now_ns: u64) -> u8 {
        self.response.front().copied().unwrap_or(if now_ns < self.card.busy_until { 0 } else { 0xff })
    }

    /// Shift one byte with CS low. Clocks advance the card even when the host discards MISO.
    pub fn exchange(&mut self, mosi: u8, now_ns: u64) -> u8 {
        let had_response = !self.response.is_empty();
        let out = self.response.pop_front().unwrap_or(0xff);
        if !had_response && now_ns < self.card.busy_until { return 0; }
        if let Phase::Write { lba, multi, receiving } = self.phase {
            if had_response { return out; }
            if receiving {
                self.write_data.push(mosi);
                if self.write_data.len() == BLOCK + 2 { self.program_block(lba, multi, now_ns); }
                return out;
            }
            if self.command_len == 0 {
                if mosi == if multi { WRITE_START } else { DATA_START } {
                    self.phase = Phase::Write { lba, multi, receiving: true };
                    return out;
                }
                if multi && mosi == WRITE_STOP { self.phase = Phase::Command; return out; }
                if mosi != 0x4c { return out; } // CMD12 can also end write-error recovery.
            }
        }
        if self.command_len != 0 || mosi & 0xc0 == 0x40 {
            self.command[self.command_len] = mosi;
            self.command_len += 1;
            if self.command_len == self.command.len() {
                self.command_len = 0;
                self.execute();
            }
        } else if !had_response {
            if let Phase::Read { next } = self.phase {
                self.queue_block(next, true);
                return self.response.pop_front().unwrap_or(0xff);
            }
        }
        out
    }
}

/// CRC-16/CCITT for SD data packets (SD spec §4.5 and §7.3.3).
fn crc16(bytes: &[u8]) -> u16 {
    let mut crc = 0u16;
    for &byte in bytes {
        crc ^= u16::from(byte) << 8;
        for _ in 0..8 { crc = (crc << 1) ^ if crc & 0x8000 != 0 { 0x1021 } else { 0 }; }
    }
    crc
}

#[cfg(test)]
mod tests;
