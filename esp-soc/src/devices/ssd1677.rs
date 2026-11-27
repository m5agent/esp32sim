//! SSD1677 command and RAM subset, datasheet Rev. 1.0 §6.6 and M5GFX
//! 28d47949 Panel_SSD1677.inl. Refresh BUSY is active high. Timing below
//! approximates the digital busy interval by update mode.

pub struct Ssd1677 {
    pub frame: Vec<u16>,
    pub refreshes: u64,
    pub bytes_written: u64,
    banks: [Vec<u8>; 2],
    command: u8,
    params: [u8; 105],
    count: usize,
    lut: Option<[u8; 105]>,
    x_bounds: [usize; 2],
    y_bounds: [usize; 2],
    x: usize,
    y: usize,
    entry: u8,
    ctrl1: u8,
    ctrl2: u8,
    mode2_odd: bool,
    ping_pong: bool,
    powered: bool,
    asleep: bool,
    update: bool,
    busy_until: Option<u64>,
    cycles_per_ms: u64,
}

impl Ssd1677 {
    pub const WIDTH: usize = 480;
    pub const HEIGHT: usize = 800;
    const SOURCE: usize = 800;
    const GATE: usize = 480;
    const STRIDE: usize = Self::SOURCE / 8;

    pub fn new(cycles_per_ms: u64) -> Self {
        Self {
            frame: vec![0xffff; Self::WIDTH * Self::HEIGHT], refreshes: 0, bytes_written: 0,
            banks: std::array::from_fn(|_| vec![0xff; Self::STRIDE * Self::GATE]),
            command: 0, params: [0; 105], count: 0, lut: None,
            x_bounds: [0, Self::STRIDE - 1], y_bounds: [0, Self::GATE - 1], x: 0, y: 0,
            entry: 3, ctrl1: 0, ctrl2: 0xff, mode2_odd: false, ping_pong: true,
            powered: false, asleep: false, update: false, busy_until: None, cycles_per_ms,
        }
    }

    pub fn control(&self) -> u8 { self.ctrl2 }
    pub fn waveform_timing(&self) -> Option<&[u8]> { self.lut.as_ref().map(|lut| &lut[50..]) }
    pub fn deadline(&self) -> Option<u64> { self.busy_until }
    fn busy_for(&mut self, now: u64, ms: u64) {
        self.busy_until = Some(now.saturating_add(ms.saturating_mul(self.cycles_per_ms)));
    }

    fn reset(&mut self) {
        self.command = 0;
        self.count = 0;
        self.x_bounds = [0, Self::STRIDE - 1];
        self.y_bounds = [0, Self::GATE - 1];
        self.x = 0;
        self.y = 0;
        self.entry = 3;
        self.ctrl1 = 0;
        self.ctrl2 = 0xff;
        self.lut = None;
        self.mode2_odd = false;
        self.ping_pong = true;
        self.powered = false;
        self.asleep = false;
        self.update = false;
        self.busy_until = None;
    }

    pub fn power_loss(&mut self) {
        self.reset();
        for bank in &mut self.banks { bank.fill(0xff); }
    }

    pub fn advance_to(&mut self, now: u64) {
        if self.busy_until.is_none_or(|due| now < due) { return; }
        self.busy_until = None;
        if std::mem::take(&mut self.update) { self.publish(); }
        if self.ctrl2 & 2 != 0 { self.powered = false; }
    }

    pub fn write(&mut self, now: u64, data: bool, bytes: &[u8]) {
        self.advance_to(now);
        if self.asleep || self.busy_until.is_some() { return; }
        for &byte in bytes {
            if data {
                self.write_data(now, byte);
            } else {
                self.command = byte;
                self.count = 0;
                match byte {
                    0x12 => { self.reset(); self.busy_for(now, 5); }
                    0x20 => {
                        self.powered |= self.ctrl2 & 0x40 != 0;
                        self.update = self.powered && self.ctrl2 & 4 != 0;
                        self.busy_for(now, if !self.update { 8 } else if self.ctrl2 & 8 != 0 { 300 } else { 1000 });
                    }
                    _ => {}
                }
            }
        }
    }

    fn write_data(&mut self, now: u64, byte: u8) {
        if matches!(self.command, 0x24 | 0x26) {
            self.write_ram(usize::from(self.command == 0x26), byte);
            return;
        }
        if let Some(param) = self.params.get_mut(self.count) { *param = byte; }
        self.count = self.count.saturating_add(1);
        let word = |i| u16::from_le_bytes([self.params[i], self.params[i + 1]]) as usize & 0x03ff;
        match (self.command, self.count) {
            (0x10, 1) => self.asleep = byte & 1 != 0,
            (0x11, 1) => self.entry = byte & 7,
            (0x21, 1) => self.ctrl1 = byte,
            (0x22, 1) => self.ctrl2 = byte,
            (0x32, 105) => self.lut = Some(self.params),
            (0x37, 10) => self.ping_pong = self.params[5] & 0x40 != 0,
            (0x44, 4) => self.x_bounds = [word(0) / 8, word(2) / 8],
            (0x45, 4) => self.y_bounds = [word(0), word(2)],
            (0x4e, 2) => self.x = word(0) / 8,
            (0x4f, 2) => self.y = word(0),
            (0x46 | 0x47, 1) => {
                let bank = usize::from(self.command == 0x46);
                let x_period = if byte & 7 == 7 { Self::STRIDE } else { 1 << (byte & 7) };
                let y_period = if (byte >> 4) & 7 == 7 { Self::GATE } else { 8 << ((byte >> 4) & 7) };
                for (i, dst) in self.banks[bank].iter_mut().enumerate() {
                    let stripes = ((i % Self::STRIDE) / x_period + (i / Self::STRIDE) / y_period) % 2;
                    *dst = if (stripes == 0) == (byte & 0x80 != 0) { 0xff } else { 0 };
                }
                self.busy_for(now, 8);
            }
            _ => {}
        }
    }

    fn write_ram(&mut self, bank: usize, byte: u8) {
        if self.x < Self::STRIDE && self.y < Self::GATE {
            self.banks[bank][self.y * Self::STRIDE + self.x] = byte;
            self.bytes_written += 1;
        }
        fn step(current: &mut usize, [start, end]: [usize; 2], increment: bool) -> bool {
            let wrapped = if increment { *current >= end } else { *current <= end };
            *current = if wrapped { start } else if increment { *current + 1 } else { *current - 1 };
            wrapped
        }
        let x_first = self.entry & 4 == 0;
        let (dx, dy) = (self.entry & 1 != 0, self.entry & 2 != 0);
        if x_first {
            if step(&mut self.x, self.x_bounds, dx) { step(&mut self.y, self.y_bounds, dy); }
        } else if step(&mut self.y, self.y_bounds, dy) {
            step(&mut self.x, self.x_bounds, dx);
        }
    }

    fn publish(&mut self) {
        let mode2 = self.ctrl2 & 8 != 0;
        let shades = if let Some(lut) = &self.lut {
            let differential = lut[..10].iter().chain(&lut[30..40]).all(|&v| v == 0);
            if differential { [None, Some(0xffff), Some(0), None] }
            else if lut[..10] == lut[20..30] && lut[10..20] == lut[30..40] {
                [Some(0), Some(0xffff), Some(0), Some(0xffff)]
            } else { [Some(0xffff), Some(0xad55), Some(0x52aa), Some(0)] }
        } else {
            [Some(0), Some(0xffff), Some(0), Some(0xffff)]
        };
        let ctrl_bit = |bit: usize, ctrl: u8| (bit ^ usize::from(ctrl & 8 != 0)) * usize::from(ctrl & 4 == 0);
        for source in 0..Self::SOURCE {
            for gate in 0..Self::GATE {
                let index = gate * Self::STRIDE + source / 8;
                let mask = 0x80 >> (source % 8);
                let mut bw = ctrl_bit(usize::from(self.banks[0][index] & mask != 0), self.ctrl1);
                let mut red = ctrl_bit(usize::from(self.banks[1][index] & mask != 0), self.ctrl1 >> 4);
                if !mode2 && self.mode2_odd { std::mem::swap(&mut bw, &mut red); }
                if let Some(shade) = shades[bw | (red << 1)] {
                    self.frame[source * Self::WIDTH + gate] = shade;
                }
            }
        }
        if mode2 && self.ping_pong { self.banks.swap(0, 1); }
        self.mode2_odd = mode2 && self.ping_pong && !self.mode2_odd;
        self.refreshes += 1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn refresh_resolves_four_levels_and_publishes_only_at_busy_end() {
        let mut panel = Ssd1677::new(1);
        let mut send = |command, bytes: &[u8]| {
            panel.write(0, false, &[command]);
            panel.write(0, true, bytes);
        };
        send(0x24, &[0x50]);
        send(0x4e, &[0, 0]);
        send(0x26, &[0x30]);
        let mut lut = [0; 105];
        lut[0] = 1;
        lut[30] = 2;
        send(0x32, &lut);
        send(0x22, &[0xc7]);
        send(0x20, &[]);
        panel.advance_to(999);
        assert_eq!(panel.refreshes, 0);
        panel.advance_to(1000);
        assert_eq!(panel.refreshes, 1);
        assert_eq!((0..4).map(|x| panel.frame[x * 480]).collect::<Vec<_>>(),
                   [0xffff, 0xad55, 0x52aa, 0]);
        panel.power_loss();
        assert_eq!(panel.frame[1440], 0);
    }
}
