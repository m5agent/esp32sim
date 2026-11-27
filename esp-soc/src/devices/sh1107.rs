//! Sino Wealth SH1107 OLED controller, page-address mode and scan controls.
//! GDDRAM is 128 columns by 16 pages of 8 rows (SH1107 v2.1).
pub struct Sh1107 {
    ram: [u8; 2048],
    column: u8,
    page: u8,
    args: u8,
    start_line: u8,
    offset: u8,
    remap: bool,
    reverse_com: bool,
    inverse: bool,
    entire_on: bool,
    on: bool,
    vertical: bool,
    command: u8,
    pub writes: u64,
    pub generation: u64,
}

impl Default for Sh1107 {
    fn default() -> Self {
        Self { ram: [0; 2048], column: 0, page: 0, args: 0, start_line: 0,
            offset: 0, remap: false, reverse_com: false, inverse: false,
            entire_on: false, on: false, vertical: false, command: 0, writes: 0, generation: 0 }
    }
}

impl Sh1107 {
    pub fn reset(&mut self) {
        *self = Self { ram: self.ram, writes: self.writes, generation: self.generation + 1, ..Self::default() };
    }

    pub fn write(&mut self, dc: bool, byte: u8) {
        self.generation += 1;
        if dc {
            self.ram[usize::from(self.page) * 128 + usize::from(self.column)] = byte;
            if self.vertical { self.page = (self.page + 1) & 15; }
            else { self.column = (self.column + 1) & 127; }
            self.writes += 1;
            return;
        }
        if self.args != 0 {
            match self.command {
                0xd3 => self.offset = byte & 127,
                0xdc => self.start_line = byte & 127,
                _ => {}
            }
            self.args -= 1;
            return;
        }
        self.command = byte;
        match byte {
            0x00..=0x0f => self.column = (self.column & 0x70) | byte,
            0x10..=0x17 => self.column = (self.column & 15) | ((byte & 7) << 4),
            0x20 | 0x21 => self.vertical = byte & 1 != 0,
            0x81 | 0xa8 | 0xad | 0xd3 | 0xd5 | 0xd9 | 0xdb | 0xdc => self.args = 1,
            0xa0 | 0xa1 => self.remap = byte & 1 != 0,
            0xa4 | 0xa5 => self.entire_on = byte & 1 != 0,
            0xa6 | 0xa7 => self.inverse = byte & 1 != 0,
            0xae | 0xaf => self.on = byte & 1 != 0,
            0xb0..=0xbf => self.page = byte & 15,
            0xc0 | 0xc8 => self.reverse_com = byte & 8 != 0,
            _ => {}
        }
    }

    pub fn pixel(&self, segment: usize, common: usize) -> u16 {
        if !self.on { return 0; }
        let col = if self.remap { 127 - (segment & 127) } else { segment & 127 };
        let row = if self.reverse_com { 127 - (common & 127) } else { common & 127 };
        let row = (row + usize::from(self.start_line) + 128 - usize::from(self.offset)) & 127;
        let lit = self.entire_on || ((self.ram[(row / 8) * 128 + col] >> (row % 8) & 1 != 0) ^ self.inverse);
        if lit { 0xffff } else { 0 }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn page_write_and_scan_commands() {
        let mut panel = Sh1107::default();
        for cmd in [0xb2, 0x11, 0x04, 0xaf] { panel.write(false, cmd); }
        panel.write(true, 1 << 3);
        panel.write(true, 1 << 4);
        assert_eq!((panel.pixel(20, 19), panel.pixel(21, 20)), (0xffff, 0xffff));
        panel.write(false, 0xa1);
        assert_eq!((panel.pixel(107, 19), panel.pixel(20, 19)), (0xffff, 0));
        panel.reset();
        assert_eq!(panel.pixel(20, 19), 0);
        panel.write(false, 0xaf);
        assert_eq!(panel.pixel(20, 19), 0xffff);
    }
}
