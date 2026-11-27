use esp_periph::i2c::I2cDevice;

const DEFAULT_REGS: [u16; 8] = [0x4127, 0, 0, 0, 0, 0, 0, 0];
const CONVERSION_US: [u64; 8] = [140, 204, 332, 588, 1100, 2116, 4156, 8244];
const AVERAGES: [u64; 8] = [1, 4, 16, 64, 128, 256, 512, 1024];

/// TI INA226, SBOS547C sections 6.5 and 7.1. The board supplies its shunt,
/// polarity and a steady bus-voltage/load stimulus for the conversion window.
pub struct Ina226 {
    regs: [u16; 8],
    ptr: u8,
    phase: u8,
    upper: u8,
    latch: u16,
    mv: u16,
    ma: i16,
    shunt_microohms: u32,
    reversed: bool,
    hz: u64,
    cycle: u64,
    next: Option<u64>,
}

impl Ina226 {
    pub fn new(hz: u64, shunt_microohms: u32, reversed: bool, mv: u16, ma: i16) -> Self {
        let mut chip = Self { regs: DEFAULT_REGS, ptr: 0, phase: 0, upper: 0, latch: 0,
                              mv, ma, shunt_microohms, reversed, hz, cycle: 0, next: None };
        chip.begin_conversion();
        chip
    }
    pub fn supply(&mut self, mv: u16, ma: i16) { self.mv = mv; self.ma = ma; }

    fn period(&self) -> Option<u64> {
        let config = self.regs[0];
        let shunt = if config & 1 != 0 { CONVERSION_US[usize::from((config >> 3) & 7)] } else { 0 };
        let bus = if config & 2 != 0 { CONVERSION_US[usize::from((config >> 6) & 7)] } else { 0 };
        (shunt + bus != 0).then(|| {
            let micros = (shunt + bus) * AVERAGES[usize::from((config >> 9) & 7)];
            micros.saturating_mul(self.hz).div_ceil(1_000_000).max(1)
        })
    }

    pub fn alert_low(&self) -> bool {
        let asserted = self.regs[6] & 0x10 != 0 || self.regs[6] & 0x408 == 0x408;
        asserted ^ (self.regs[6] & 2 != 0)
    }

    fn begin_conversion(&mut self) {
        if self.period().is_some() { self.regs[6] &= !0x18; }
        self.next = self.period().map(|period| self.cycle.saturating_add(period));
    }

    pub fn advance(&mut self, cycle: u64) {
        self.cycle = cycle.max(self.cycle);
        let Some(next) = self.next.filter(|&next| next <= self.cycle) else { return };
        let mode = self.regs[0] & 7;
        if mode & 1 != 0 {
            let polarity = if self.reversed { -1 } else { 1 };
            let shunt = polarity * i64::from(self.ma) * i64::from(self.shunt_microohms) / 2500;
            self.regs[1] = shunt.clamp(i16::MIN.into(), i16::MAX.into()) as u16;
        }
        if mode & 2 != 0 {
            self.regs[2] = (u32::from(self.mv) * 4 / 5).min(0x7fff) as u16;
        }
        let current = i64::from(self.regs[1] as i16) * i64::from(self.regs[5]) / 2048;
        let power = current.unsigned_abs() * u64::from(self.regs[2]) / 20000;
        let overflow = !(i64::from(i16::MIN)..=i64::from(i16::MAX)).contains(&current) || power > u64::from(u16::MAX);
        self.regs[3] = power.min(u64::from(u16::MAX)) as u16;
        self.regs[4] = current.clamp(i16::MIN.into(), i16::MAX.into()) as u16;
        self.regs[6] = (self.regs[6] & !4) | 8 | (u16::from(overflow) << 2);
        let limit = self.regs[7];
        let alert = match (self.regs[6] & 0xf800).leading_zeros() {
            0 => (self.regs[1] as i16) > limit as i16,
            1 => (self.regs[1] as i16) < limit as i16,
            2 => self.regs[2] > limit,
            3 => self.regs[2] < limit,
            4 => self.regs[3] > limit,
            _ => false,
        };
        if alert { self.regs[6] |= 0x10; }
        else if self.regs[6] & 1 == 0 { self.regs[6] &= !0x10; }
        self.next = self.period().filter(|_| mode & 4 != 0)
            .map(|period| self.cycle.saturating_add(period - (self.cycle - next) % period));
    }
}

impl I2cDevice for Ina226 {
    fn start(&mut self, read: bool) -> bool {
        self.phase = 0;
        if read {
            self.latch = match self.ptr {
                0xfe => 0x5449, 0xff => 0x2260,
                reg => self.regs.get(usize::from(reg)).copied().unwrap_or(0),
            };
        }
        true
    }
    fn write(&mut self, byte: u8) -> bool {
        match self.phase {
            0 => self.ptr = byte,
            1 => self.upper = byte,
            2 => {
                let value = u16::from_be_bytes([self.upper, byte]);
                match self.ptr {
                    0 => {
                        if value & 0x8000 != 0 { self.regs = DEFAULT_REGS; }
                        else { self.regs[0] = 0x4000 | (value & 0x0fff); }
                        self.begin_conversion();
                    }
                    5 | 7 => self.regs[usize::from(self.ptr)] = value,
                    6 => self.regs[6] = (self.regs[6] & 0x1c) | (value & 0xfc03),
                    _ => {}
                }
            }
            _ => {}
        }
        self.phase = self.phase.saturating_add(1);
        true
    }
    fn read(&mut self) -> u8 {
        let byte = self.latch.to_be_bytes()[usize::from(self.phase & 1)];
        self.phase ^= 1;
        if self.phase == 0 && self.ptr == 6 {
            self.regs[6] &= if self.regs[6] & 1 != 0 { !0x18 } else { !8 };
        }
        byte
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn conversion_scaling_one_shot_and_read_to_clear() {
        let mut ina = Ina226::new(1_000_000, 5000, true, 7980, 410);
        let write = |ina: &mut Ina226, ptr, value: u16| {
            let [upper, lower] = value.to_be_bytes();
            assert!(ina.start(false) && ina.write(ptr) && ina.write(upper) && ina.write(lower));
        };
        let read = |ina: &mut Ina226, ptr| {
            assert!(ina.start(false) && ina.write(ptr) && ina.start(true));
            u16::from_be_bytes([ina.read(), ina.read()])
        };
        write(&mut ina, 5, 2048);
        assert_eq!(read(&mut ina, 2), 0);
        ina.advance(2200);
        assert_eq!(read(&mut ina, 2), 6384);
        assert_eq!(read(&mut ina, 4), (-820i16) as u16);
        assert_eq!(read(&mut ina, 6), 8);
        assert_eq!(read(&mut ina, 6), 0);
        assert_eq!(read(&mut ina, 0xff), 0x2260);
        write(&mut ina, 0, 0x0123);
        ina.supply(5000, -100);
        ina.advance(4400);
        assert_eq!(read(&mut ina, 2), 4000);
        assert_eq!(read(&mut ina, 4), 200);
        ina.supply(3300, 200);
        ina.advance(100_000);
        assert_eq!(read(&mut ina, 2), 4000);
    }
}
