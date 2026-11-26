//! Belling BM8563 RTC. BCD calendar, alarm, countdown and INT follow datasheet
//! v1.1 pp. 6–12 (tables 1–5 and 22–23) and M5Unified 0.3.0 `PCF8563_Class.inl`.
use esp_periph::i2c::I2cDevice;
use crate::board::VirtualCycle;

pub const BM8563_ADDRESS: u8 = 0x51;
const STOP: u8 = 0x20;
const PULSE: u8 = 0x10;
const AF: u8 = 8;
const TF: u8 = 4;
const AIE: u8 = 2;
const TIE: u8 = 1;
const TE: u8 = 0x80;

pub struct Bm8563 {
    regs: [u8; 16],
    clock_read: [u8; 7],
    ptr: u8,
    first: bool,
    hz: u64,
    cycle: VirtualCycle,
    clock_anchor: VirtualCycle,
    timer_anchor: VirtualCycle,
    timer_periods: u64,
    pulse_end: VirtualCycle,
    pub last_timer_expiry: Option<VirtualCycle>,
}

fn from_bcd(b: u8) -> u8 { (b >> 4) * 10 + (b & 15) }
fn bcd(n: u8) -> u8 { ((n / 10) << 4) | (n % 10) }

impl Bm8563 {
    pub fn new(hz: u64) -> Self {
        // A running, battery-backed RTC with a deterministic valid initial calendar (2024-01-01).
        let regs = [0, 0, 0, 0, 0, 1, 1, 1, 0x24, 0x80, 0x80, 0x80, 0x80, 0x80, 3, 0];
        Self {
            regs, clock_read: [0; 7], ptr: 0, first: true, hz: hz.max(4096), cycle: 0,
            clock_anchor: 0, timer_anchor: 0, timer_periods: 0, pulse_end: 0,
            last_timer_expiry: None,
        }
    }

    fn alarm_enabled(&self) -> bool { self.regs[9..=12].iter().any(|&field| field & 0x80 == 0) }
    fn check_alarm(&mut self) {
        let clock = [self.regs[3], self.regs[4], self.regs[5], self.regs[6]];
        if self.regs[9..=12].iter().zip(clock).all(|(&field, now)| field & 0x80 != 0 || field == now) {
            self.regs[1] |= AF;
        }
    }
    fn advance_clock(&mut self, mut secs: u64) {
        while secs != 0 {
            let s = u64::from(from_bcd(self.regs[2] & 0x7f).min(59));
            let m = u64::from(from_bcd(self.regs[3]).min(59));
            let h = u64::from(from_bcd(self.regs[4]).min(23));
            let time = h * 3600 + m * 60 + s;
            let alarm = self.regs[1] & AF == 0 && self.alarm_enabled();
            let step = secs.min(if alarm { 60 - s } else { 86400 - time });
            let next = (time + step) % 86400;
            self.regs[2] = (self.regs[2] & 0x80) | bcd((next % 60) as u8);
            self.regs[3] = bcd((next / 60 % 60) as u8);
            self.regs[4] = bcd((next / 3600) as u8);
            if next == 0 { self.advance_day(); }
            if alarm && next.is_multiple_of(60) { self.check_alarm(); }
            secs -= step;
        }
    }
    fn advance_day(&mut self) {
        let century = self.regs[7] & 0x80;
        let mut month = from_bcd(self.regs[7] & 0x1f).clamp(1, 12);
        let mut year = from_bcd(self.regs[8]).min(99);
        let days = match month {
            2 => 28 + u8::from(year.is_multiple_of(4) && (year != 0 || century == 0)),
            4 | 6 | 9 | 11 => 30,
            _ => 31,
        };
        let mut day = from_bcd(self.regs[5]).clamp(1, days) + 1;
        if day > days {
            day = 1;
            month += 1;
            if month == 13 { month = 1; year = (year + 1) % 100; }
        }
        self.regs[5] = bcd(day);
        self.regs[6] = (self.regs[6] + 1) % 7;
        self.regs[7] = bcd(month) | if year == 0 && month == 1 && day == 1 { century ^ 0x80 } else { century };
        self.regs[8] = bcd(year);
    }

    fn timer_period(&self) -> Option<u64> {
        let reload = self.regs[15];
        (self.regs[14] & TE != 0 && reload != 0).then_some(u64::from(reload))
    }
    // Number of source ticks per 60 seconds, including the 1/60 Hz mode.
    fn timer_rate(&self) -> u128 { [245760, 3840, 60, 1][usize::from(self.regs[14] & 3)] }
    fn timer_ticks(&self) -> u64 {
        (u128::from(self.cycle - self.timer_anchor) * self.timer_rate() / (u128::from(self.hz) * 60)) as u64
    }
    fn timer_cycle(&self, ticks: u64) -> VirtualCycle {
        let delta = (u128::from(ticks) * u128::from(self.hz) * 60).div_ceil(self.timer_rate());
        self.timer_anchor.saturating_add(u64::try_from(delta).unwrap_or(u64::MAX))
    }
    fn advance_timer(&mut self, reload: u64) {
        let periods = self.timer_ticks() / reload;
        if periods == self.timer_periods { return; }
        self.timer_periods = periods;
        self.regs[1] |= TF;
        let expiry = self.timer_cycle(periods * reload);
        self.last_timer_expiry = Some(expiry);
        if self.regs[1] & (PULSE | TIE) == (PULSE | TIE) {
            let denominator = match (self.regs[14] & 3, reload) {
                (0, 1) => 8192, (0, _) => 4096, (1, 1) => 128, _ => 64,
            };
            self.pulse_end = expiry.saturating_add(self.hz.div_ceil(denominator));
        }
    }
    pub fn advance_to(&mut self, cycle: VirtualCycle) {
        self.cycle = self.cycle.max(cycle);
        if self.regs[0] & STOP == 0 && self.cycle - self.clock_anchor >= self.hz {
            let seconds = (self.cycle - self.clock_anchor) / self.hz;
            self.clock_anchor += seconds * self.hz;
            self.advance_clock(seconds);
        }
        if let Some(reload) = self.timer_period() { self.advance_timer(reload); }
    }
    pub fn irq_low(&self) -> bool {
        let c = self.regs[1];
        c & (AF | AIE) == (AF | AIE)
            || (c & TIE != 0 && if c & PULSE != 0 { self.cycle < self.pulse_end } else { c & TF != 0 })
    }
    pub fn control2(&self) -> u8 { self.regs[1] }
    pub fn next_deadline(&self) -> Option<VirtualCycle> {
        let alarm = (self.regs[0] & STOP == 0 && self.regs[1] & (AF | AIE) == AIE && self.alarm_enabled())
            .then(|| self.clock_anchor.saturating_add(self.hz.saturating_mul(60 - u64::from(from_bcd(self.regs[2] & 0x7f).min(59)))));
        let timer = self.timer_period().filter(|_| self.regs[1] & TIE != 0 && (self.regs[1] & TF == 0 || self.regs[1] & PULSE != 0))
            .map(|n| self.timer_cycle(self.timer_periods.saturating_add(1).saturating_mul(n)));
        let pulse = (self.pulse_end > self.cycle).then_some(self.pulse_end);
        alarm.into_iter().chain(timer).chain(pulse).filter(|&at| at > self.cycle).min()
    }

    fn write_reg(&mut self, reg: u8, byte: u8) {
        let i = usize::from(reg);
        self.regs[i] = match reg {
            0 => {
                if (self.regs[0] ^ byte) & STOP != 0 { self.clock_anchor = self.cycle; }
                byte & 0xa8
            }
            1 => {
                if byte & (PULSE | TIE) != (PULSE | TIE) { self.pulse_end = 0; }
                byte & (PULSE | AIE | TIE | (self.regs[i] & (AF | TF)))
            }
            2 => {
                self.clock_anchor = self.cycle;
                byte
            }
            0x0e | 0x0f => {
                if reg == 0x0f || (self.regs[i] ^ byte) & (TE | 3) != 0 {
                    self.timer_anchor = self.cycle;
                    self.timer_periods = 0;
                }
                if reg == 0x0e { byte & (TE | 3) } else { byte }
            }
            3 => byte & 0x7f,
            4 | 5 => byte & 0x3f,
            6 => byte & 7,
            7 => byte & 0x9f,
            0x0a | 0x0b => byte & 0xbf,
            0x0c => byte & 0x87,
            0x0d => byte & 0x83,
            _ => byte,
        };
    }
}

impl I2cDevice for Bm8563 {
    fn start(&mut self, read: bool) -> bool {
        if read { self.clock_read.copy_from_slice(&self.regs[2..=8]); }
        else { self.first = true; }
        true
    }
    fn write(&mut self, byte: u8) -> bool {
        if self.first { self.ptr = byte & 0xf; self.first = false; }
        else { self.write_reg(self.ptr, byte); self.ptr = (self.ptr + 1) & 0xf; }
        true
    }
    fn read(&mut self) -> u8 {
        let value = match self.ptr {
            2..=8 => self.clock_read[usize::from(self.ptr - 2)],
            15 => self.timer_period().map_or(self.regs[15], |reload| (reload - self.timer_ticks() % reload) as u8),
            reg => self.regs[usize::from(reg)],
        };
        self.ptr = (self.ptr + 1) & 0xf;
        value
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const HZ: u64 = 32768;
    fn write(r: &mut Bm8563, reg: u8, bytes: &[u8]) {
        r.start(false); r.write(reg);
        for &byte in bytes { r.write(byte); }
        r.stop();
    }

    #[test]
    fn calendar_set_snapshot_leap_day_and_stop() {
        let mut r = Bm8563::new(HZ);
        write(&mut r, 2, &[0x59, 0x59, 0x23, 0x28, 3, 2, 0x24]);
        write(&mut r, 2, &[]);
        r.start(true);
        assert_eq!(r.read(), 0x59);
        r.advance_to(HZ);
        assert_eq!(r.read(), 0x59); // one coherent read across the tick
        write(&mut r, 2, &[]);
        r.start(true);
        assert_eq!(std::array::from_fn::<_, 7, _>(|_| r.read()), [0, 0, 0, 0x29, 4, 2, 0x24]);
        write(&mut r, 0, &[STOP]);
        r.advance_to(10 * HZ);
        assert_eq!(r.regs[2], 0);
        write(&mut r, 0, &[0]);
        r.advance_to(12 * HZ);
        assert_eq!(r.regs[2], 2);
    }

    #[test]
    fn countdown_reloads_and_asserts_level_irq_until_cleared() {
        let mut r = Bm8563::new(HZ);
        write(&mut r, 14, &[0x82, 5]);
        write(&mut r, 1, &[TIE]);
        assert_eq!(r.next_deadline(), Some(5 * HZ));
        r.advance_to(5 * HZ - 1);
        write(&mut r, 15, &[]);
        r.start(true);
        assert_eq!(r.read(), 1);
        assert!(!r.irq_low());
        r.advance_to(5 * HZ);
        assert!(r.irq_low());
        assert_eq!(r.control2(), TF | TIE);
        write(&mut r, 1, &[TF | TIE]);
        assert!(r.irq_low());
        write(&mut r, 1, &[TIE]);
        assert!(!r.irq_low());
        assert_eq!(r.next_deadline(), Some(10 * HZ));
    }

    #[test]
    fn alarm_compares_enabled_bcd_fields_at_minute_boundaries() {
        let mut r = Bm8563::new(HZ);
        write(&mut r, 2, &[0x59, 0x09, 0x12]);
        write(&mut r, 9, &[0x10, 0x12, 0x80, 0x80]);
        write(&mut r, 1, &[AIE]);
        r.advance_to(HZ - 1);
        assert!(!r.irq_low());
        r.advance_to(HZ + 7);
        assert_eq!(r.control2(), AF | AIE);
        write(&mut r, 1, &[AIE]);
        r.advance_to(4 * HZ);
        assert!(!r.irq_low());
    }
}
