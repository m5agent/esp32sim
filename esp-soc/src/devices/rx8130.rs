//! RX8130 calendar, register and countdown subset. Register addresses and
//! countdown clocks follow M5Unified 96576a14 RX8130_Class.inl.
use std::{cell::Cell, rc::Rc};
use esp_periph::i2c::I2cDevice;
use super::bm8563::Bm8563;

pub struct Rx8130 {
    time: Rc<Cell<u64>>,
    hz: u64,
    calendar: Bm8563,
    regs: [u8; 64],
    snapshot: [u8; 7],
    pointer: u8,
    address_next: bool,
    timer_start: u64,
}

impl Rx8130 {
    pub fn new(time: Rc<Cell<u64>>, hz: u64) -> Self {
        let mut regs = [0; 64];
        regs[0x17..0x1a].fill(0x80);
        Self {
            time, hz, calendar: Bm8563::new(hz), regs, snapshot: [0; 7],
            pointer: 0, address_next: true, timer_start: 0,
        }
    }

    fn tick(&mut self) {
        let now = self.time.get();
        self.calendar.advance_to(now);
        let count = u64::from(u16::from_le_bytes([self.regs[0x1a], self.regs[0x1b]]));
        let select = usize::from(self.regs[0x1c] & 7);
        if let Some(&cycles) = [self.hz / 4096, self.hz / 64, self.hz, self.hz * 60, self.hz * 3600].get(select) {
            if self.regs[0x1c] & 0x10 != 0 && count != 0 && now - self.timer_start >= cycles.saturating_mul(count) {
                self.regs[0x1d] |= 0x10;
            }
        }
    }

    fn read_calendar(&mut self) {
        self.calendar.start(false);
        self.calendar.write(2);
        self.calendar.start(true);
        let [s, m, h, day, week, month, year] = std::array::from_fn(|_| self.calendar.read());
        self.snapshot = [s, m, h, 1 << (week % 7), day, month & 0x1f, year];
        let alarm = &self.regs[0x17..0x1a];
        let date_matches = alarm[2] & 0x80 != 0
            || if self.regs[0x1c] & 8 == 0 { alarm[2] & self.snapshot[3] != 0 } else { alarm[2] == day };
        if self.regs[0x1e] & 8 != 0 && date_matches
            && (alarm[0] & 0x80 != 0 || alarm[0] == m)
            && (alarm[1] & 0x80 != 0 || alarm[1] == h)
        {
            self.regs[0x1d] |= 8;
        }
    }
}

impl I2cDevice for Rx8130 {
    fn start(&mut self, read: bool) -> bool {
        self.tick();
        if read { self.read_calendar(); } else { self.address_next = true; }
        true
    }

    fn write(&mut self, byte: u8) -> bool {
        if std::mem::take(&mut self.address_next) {
            self.pointer = byte & 63;
            return true;
        }
        let reg = usize::from(self.pointer);
        match reg {
            0x10..=0x16 => {
                let map = [2, 3, 4, 6, 5, 7, 8];
                self.calendar.start(false);
                self.calendar.write(map[reg - 0x10]);
                self.calendar.write(if reg == 0x13 { (byte & 0x7f).trailing_zeros().min(6) as u8 } else { byte });
            }
            0x1c => {
                if self.regs[reg] & 0x10 == 0 && byte & 0x10 != 0 { self.timer_start = self.time.get(); }
                self.regs[reg] = byte;
            }
            0x1d => self.regs[reg] &= byte,
            _ => self.regs[reg] = byte,
        }
        self.pointer = (self.pointer + 1) & 63;
        true
    }

    fn read(&mut self) -> u8 {
        let reg = self.pointer;
        self.pointer = (reg + 1) & 63;
        match reg {
            0x10..=0x16 => self.snapshot[usize::from(reg - 0x10)],
            _ => self.regs[usize::from(reg)],
        }
    }
}
