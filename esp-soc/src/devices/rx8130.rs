//! Epson RX8130CE RTC, shared by Tab5 and StopWatch.
use esp_periph::i2c::I2cDevice;

/// RX8130CE ETM50E-10 §§13.2, 14.2, 14.5. Calendar advances from virtual cycles,
/// with coherent burst reads, one-hot weekdays, alarm and timer flags.
pub struct Rx8130 {
    hz: u64,
    regs: [u8; 0x32],
    snapshot: [u8; 7],
    ptr: u8,
    first: bool,
    cycle: u64,
    anchor: u64,
    next_timer: Option<u64>,
}

fn bcd(value: u8) -> u8 { (value / 10) << 4 | (value % 10) }
fn decimal(value: u8) -> u8 { (value >> 4) * 10 + (value & 15) }

impl Rx8130 {
    pub const ADDRESS: u8 = 0x32;
    pub fn new(hz: u64) -> Self {
        assert!(hz > 0);
        let mut regs = [0; 0x32];
        regs[0x10..0x17].copy_from_slice(&[0, 0, 0, 2, 1, 1, 0x24]); // 2024-01-01, Monday
        regs[0x17..=0x19].fill(0x80); // all alarm fields disabled
        regs[0x1c] = 4;
        Self { hz, regs, snapshot: [0; 7], ptr: 0, first: true, cycle: 0, anchor: 0, next_timer: None }
    }
    fn new_day(&mut self) {
        let [week, day, month, year] = [0x13, 0x14, 0x15, 0x16].map(|i| self.regs[i]);
        let (mut day, mut month, mut year) = (decimal(day).clamp(1, 31), decimal(month).clamp(1, 12), decimal(year).min(99));
        let last = match month { 2 => 28 + u8::from(year.is_multiple_of(4)), 4 | 6 | 9 | 11 => 30, _ => 31 };
        day += 1;
        if day > last { day = 1; month += 1; }
        if month == 13 { month = 1; year = (year + 1) % 100; }
        self.regs[0x13..0x17].copy_from_slice(&[((week << 1) | (week >> 6)) & 0x7f, bcd(day), bcd(month), bcd(year)]);
    }
    fn check_alarm(&mut self) {
        let [min, hour, date] = [0x17, 0x18, 0x19].map(|i| self.regs[i]);
        if min & hour & date & 0x80 != 0 { return; }
        let day_matches = date & 0x80 != 0 || if self.regs[0x1c] & 8 != 0 { date == self.regs[0x14] }
            else { date & self.regs[0x13] != 0 };
        if (min & 0x80 != 0 || min == self.regs[0x11]) && (hour & 0x80 != 0 || hour == self.regs[0x12]) && day_matches {
            self.regs[0x1d] |= 8;
        }
    }
    /// RX8130CE ETM50E-10: AF/AIE at bit 3, TF/TIE at bit 4 drive the active-low IRQ.
    pub fn irq(&self) -> bool { self.regs[0x1d] & self.regs[0x1e] & 0x18 != 0 }
    pub fn next_irq_check(&self) -> Option<u64> {
        let alarm = (self.regs[0x1e] & 8 != 0 && self.regs[0x1e] & 0x40 == 0)
            .then(|| self.anchor.saturating_add(self.hz));
        let timer = self.next_timer.filter(|_| self.regs[0x1e] & 0x10 != 0);
        alarm.into_iter().chain(timer).min()
    }
    fn timer_period(&self) -> u64 {
        let count = u64::from(u16::from_le_bytes([self.regs[0x1a], self.regs[0x1b]]) & 0x0fff).max(1);
        match self.regs[0x1c] & 7 {
            0 => (self.hz * count).div_ceil(4096),
            1 => (self.hz * count).div_ceil(64),
            2 => self.hz * count,
            3 => self.hz * count * 60,
            _ => self.hz * count * 3600,
        }
    }
    pub fn advance(&mut self, cycle: u64) {
        self.cycle = self.cycle.max(cycle);
        let mut seconds = (self.cycle - self.anchor) / self.hz;
        self.anchor += seconds * self.hz;
        // STOP in Control0 freezes calendar counting.
        if self.regs[0x1e] & 0x40 != 0 { seconds = 0; }
        while seconds != 0 {
            let [s, m, h] = [0x10, 0x11, 0x12].map(|i| u64::from(decimal(self.regs[i] & 0x7f)));
            let (s, m, h) = (s.min(59), m.min(59), h.min(23));
            let step = seconds.min(60 - s);
            let now = (h * 3600 + m * 60 + s + step) % 86400;
            self.regs[0x10..0x13].copy_from_slice(&[bcd((now % 60) as u8), bcd((now / 60 % 60) as u8), bcd((now / 3600) as u8)]);
            if now == 0 { self.new_day(); }
            if now.is_multiple_of(60) { self.check_alarm(); }
            seconds -= step;
        }
        if let Some(next) = self.next_timer.filter(|n| *n <= self.cycle) {
            self.regs[0x1d] |= 0x10;
            let period = self.timer_period();
            self.next_timer = Some(self.cycle.saturating_add(period - (self.cycle - next) % period));
        }
    }
}

impl I2cDevice for Rx8130 {
    fn start(&mut self, read: bool) -> bool {
        if read { self.snapshot.copy_from_slice(&self.regs[0x10..0x17]); }
        else { self.first = true; }
        true
    }
    fn read(&mut self) -> u8 {
        let reg = usize::from(self.ptr);
        self.ptr = self.ptr.wrapping_add(1);
        if (0x10..0x17).contains(&reg) { self.snapshot[reg - 0x10] } else { self.regs.get(reg).copied().unwrap_or(0) }
    }
    fn write(&mut self, byte: u8) -> bool {
        if self.first { self.ptr = byte; self.first = false; return true; }
        let reg = usize::from(self.ptr);
        self.ptr = self.ptr.wrapping_add(1);
        let Some(slot) = self.regs.get_mut(reg) else { return true };
        let old = *slot;
        *slot = if reg == 0x1d { old & (byte | 2) } else { byte }; // W0C, except read-only VBFF
        if reg == 0x10 || reg == 0x1e && (old ^ byte) & 0x40 != 0 { self.anchor = self.cycle; }
        if matches!(reg, 0x1a..=0x1c) {
            self.next_timer = (self.regs[0x1c] & 0x10 != 0).then(|| self.cycle.saturating_add(self.timer_period()));
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    const HZ: u64 = 240_000_000;

    fn write(device: &mut impl I2cDevice, addr: u8, bytes: &[u8]) {
        assert!(device.start(false) && device.write(addr));
        for &byte in bytes { assert!(device.write(byte)); }
        device.stop();
    }
    fn read(device: &mut impl I2cDevice, addr: u8, count: usize) -> Vec<u8> {
        assert!(device.start(false) && device.write(addr) && device.start(true));
        let bytes = (0..count).map(|_| device.read()).collect();
        device.stop();
        bytes
    }

    #[test]
    fn rx8130_calendar_rollover_and_timer_flags_use_virtual_time() {
        let mut rtc = Rx8130::new(HZ);
        write(&mut rtc, 0x10, &[0x59, 0x59, 0x23, 0x08, 0x29, 0x02, 0x24]);
        write(&mut rtc, 0x1a, &[5, 0, 0x12]); // TE, 1 Hz
        rtc.advance(HZ);
        assert_eq!(read(&mut rtc, 0x10, 7), [0, 0, 0, 0x10, 1, 3, 0x24]);
        rtc.advance(5 * HZ);
        assert_eq!(read(&mut rtc, 0x1d, 1), [0x10]);
        write(&mut rtc, 0x1d, &[0xef]);
        assert_eq!(read(&mut rtc, 0x1d, 1), [0]);
    }

}
