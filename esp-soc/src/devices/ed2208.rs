//! ED2208 digital RAM and update commands from M5GFX 28d47949 Panel_ED2208.inl.
//! The Spectra 6 indices use the driver's nominal palette. BUSY is active low;
//! the driver documents 15–30 s for both full and partial pigment refreshes.

pub struct Ed2208 {
    pub frame: Vec<u16>,
    pub refreshes: u64,
    pub bytes_written: u64,
    ram: Vec<u8>,
    window: [usize; 4], // inclusive xs, xe, ys, ye
    written: Option<[usize; 4]>,
    cursor: usize,
    command: u8,
    params: [u8; 9],
    count: usize,
    powered: bool,
    asleep: bool,
    busy_until: Option<u64>,
    refresh: bool,
    sleep_after: bool,
    cycles_per_ms: u64,
}

impl Ed2208 {
    pub const WIDTH: usize = 400;
    pub const HEIGHT: usize = 600;
    const FULL: [usize; 4] = [0, Self::WIDTH - 1, 0, Self::HEIGHT - 1];

    pub fn new(cycles_per_ms: u64) -> Self {
        Self {
            frame: vec![0xffff; Self::WIDTH * Self::HEIGHT],
            ram: vec![0x11; Self::WIDTH * Self::HEIGHT / 2],
            refreshes: 0, bytes_written: 0, window: Self::FULL, written: None, cursor: 0,
            command: 0, params: [0; 9], count: 0, powered: false, asleep: false,
            busy_until: None, refresh: false, sleep_after: false, cycles_per_ms,
        }
    }

    pub fn deadline(&self) -> Option<u64> { self.busy_until }

    pub fn reset(&mut self) {
        self.ram.fill(0x11);
        self.window = Self::FULL;
        self.written = None;
        self.command = 0;
        self.count = 0;
        self.powered = false;
        self.asleep = false;
        self.busy_until = None;
        self.refresh = false;
        self.sleep_after = false;
    }

    pub fn advance_to(&mut self, now: u64) {
        if self.busy_until.is_none_or(|due| now < due) { return; }
        self.busy_until = None;
        if std::mem::take(&mut self.refresh) {
            if let Some([xs, xe, ys, ye]) = self.written.take() {
                const PALETTE: [Option<u16>; 7] = [
                    Some(0), Some(0xffff), Some(0xff87), Some(0xb800),
                    None, Some(0x621f), Some(0x4443),
                ];
                for y in ys..=ye {
                    for x in xs..=xe {
                        let pixel = y * Self::WIDTH + x;
                        let byte = self.ram[pixel / 2];
                        let color = if x % 2 == 0 { byte >> 4 } else { byte & 15 };
                        if let Some(Some(rgb)) = PALETTE.get(usize::from(color)) {
                            self.frame[pixel] = *rgb;
                        }
                    }
                }
                self.refreshes += 1;
            }
            if self.sleep_after { self.asleep = true; }
        }
    }

    fn busy_for(&mut self, now: u64, ms: u64) {
        self.busy_until = Some(now.saturating_add(ms.saturating_mul(self.cycles_per_ms)));
    }

    fn start_refresh(&mut self, now: u64, sleep_after: bool) {
        self.refresh = true;
        self.sleep_after = sleep_after;
        self.busy_for(now, 15_000);
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
                    0x10 => self.cursor = 0,
                    0x04 => { self.powered = true; self.busy_for(now, 10); }
                    _ => {}
                }
            }
        }
    }

    fn write_data(&mut self, now: u64, byte: u8) {
        if self.command == 0x10 {
            let [xs, xe, ys, ye] = self.window;
            let row_bytes = (xe - xs + 1).div_ceil(2);
            let y = ys + self.cursor / row_bytes;
            if y <= ye {
                let x = xs + 2 * (self.cursor % row_bytes);
                self.ram[(y * Self::WIDTH + x) / 2] = byte;
                self.cursor += 1;
                self.bytes_written += 1;
                let [x0, x1, y0, y1] = self.written.unwrap_or([x, x, y, y]);
                self.written = Some([x0.min(x), x1.max((x + 1).min(xe)), y0.min(y), y1.max(y)]);
            }
            return;
        }
        if let Some(param) = self.params.get_mut(self.count) { *param = byte; }
        self.count = self.count.saturating_add(1);
        match (self.command, self.count) {
            (0x83, 9) => {
                let word = |i| u16::from_be_bytes([self.params[i], self.params[i + 1]]) as usize;
                let xs = (word(0) & !7).min(Self::WIDTH - 8);
                let xe = word(2).clamp(xs, Self::WIDTH - 1);
                let ys = word(4).min(Self::HEIGHT - 1);
                let ye = word(6).clamp(ys, Self::HEIGHT - 1);
                self.window = if byte & 1 != 0 { [xs, xe, ys, ye] } else { Self::FULL };
            }
            (0x17, 1) if matches!(byte, 0xa5 | 0xa7) => {
                self.powered = false;
                self.start_refresh(now, byte == 0xa7);
            }
            (0x12, 1) if byte == 0 && self.powered => self.start_refresh(now, false),
            (0x02, 1) if byte == 0 => { self.powered = false; self.busy_for(now, 10); }
            (0x07, 1) if byte == 0xa5 => { self.asleep = true; self.powered = false; }
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn auto_refresh_publishes_only_its_window_after_busy() {
        let mut panel = Ed2208::new(1);
        panel.write(0, false, &[0x83]);
        panel.write(0, true, &[0, 8, 0, 15, 0, 1, 0, 1, 1]);
        panel.write(0, false, &[0x10]);
        panel.write(0, true, &[0x23, 0x56, 0x01, 0x11]);
        panel.write(0, false, &[0x17]);
        panel.write(0, true, &[0xa7]);
        panel.advance_to(14_999);
        assert_eq!(panel.deadline(), Some(15_000));
        assert!(panel.frame.iter().all(|&p| p == 0xffff));
        panel.advance_to(15_000);
        assert_eq!(panel.deadline(), None);
        assert_eq!(&panel.frame[407..415], &[0xffff, 0xff87, 0xb800, 0x621f, 0x4443, 0, 0xffff, 0xffff]);
        panel.reset();
        assert_eq!(panel.frame[408], 0xff87);
    }
}
