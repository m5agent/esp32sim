//! TI TCA8418 I²C keypad scanner, with a ten-event FIFO and active-low interrupt.
use esp_periph::i2c::I2cDevice;
use std::collections::VecDeque;

// TI SCPS215G §§8.3.1 and 8.6.2. The keypad uses physical row/column numbers;
// independent GPIO inputs and the keypad unlock sequence are outside this model.
#[derive(Default)]
pub struct Tca8418 {
    cfg: u8, status: u8, lock: u8, ptr: u8, select_register: bool,
    gpio_regs: [u8; 0x1e],
    fifo: VecDeque<u8>,
    read_count: u64,
}

impl Tca8418 {
    pub const ADDRESS: u8 = 0x34;

    pub fn key(&mut self, row: u8, col: u8, down: bool) {
        if row >= 8 || col >= 10 || self.lock & 0x40 != 0 { return; }
        let rows = self.gpio_regs[0x1d - 0x11];
        let cols = u16::from_le_bytes([self.gpio_regs[0x1e - 0x11], self.gpio_regs[0x1f - 0x11]]);
        if rows & (1 << row) == 0 || cols & (1 << col) == 0 { return; }
        if self.fifo.len() == 10 {
            self.status |= 0x08;
            if self.cfg & 0x20 == 0 { return; }
            self.fifo.pop_front();
        }
        self.fifo.push_back((row * 10 + col + 1) | (u8::from(down) << 7));
        self.status |= 0x01;
    }

    pub fn int_low(&self) -> bool { self.cfg & self.status & 0x09 != 0 }
    pub fn read_count(&self) -> u64 { self.read_count }

    fn increment(&mut self) {
        if self.cfg & 0x80 != 0 { self.ptr = if self.ptr >= 0x2e { 0 } else { self.ptr + 1 }; }
    }
}

impl I2cDevice for Tca8418 {
    fn start(&mut self, read: bool) -> bool { self.select_register = !read; true }
    fn write(&mut self, byte: u8) -> bool {
        if self.select_register {
            self.select_register = false;
            self.ptr = byte;
            return (1..=0x2e).contains(&byte);
        }
        match self.ptr {
            0x01 => self.cfg = byte,
            0x02 => self.status &= !(byte & if self.fifo.is_empty() { 0xff } else { 0xfe }),
            0x03 => self.lock = byte & 0x40,
            0x11..=0x2e => self.gpio_regs[usize::from(self.ptr - 0x11)] = byte,
            _ => {}
        }
        self.increment();
        true
    }
    fn read(&mut self) -> u8 {
        let byte = match self.ptr {
            0x01 => self.cfg,
            0x02 => self.status,
            0x03 => self.lock | self.fifo.len() as u8,
            0x04 => {
                if let Some(event) = self.fifo.pop_front() { self.read_count += 1; event } else { 0 }
            }
            0x05..=0x0d => self.fifo.get(usize::from(self.ptr - 4)).copied().unwrap_or(0),
            0x11..=0x2e => self.gpio_regs[usize::from(self.ptr - 0x11)],
            _ => 0,
        };
        self.increment();
        byte
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn write(dev: &mut Tca8418, reg: u8, data: u8) { dev.start(false); assert!(dev.write(reg)); assert!(dev.write(data)); }
    fn read(dev: &mut Tca8418, reg: u8) -> u8 { dev.start(false); assert!(dev.write(reg)); dev.start(true); dev.read() }

    #[test]
    fn matrix_press_release_and_interrupt_clear_after_drain() {
        let mut dev = Tca8418::default();
        dev.key(1, 2, true);
        assert_eq!(read(&mut dev, 3), 0);
        write(&mut dev, 0x1d, 0x7f);
        write(&mut dev, 0x1e, 0xff);
        dev.key(1, 2, true);
        dev.key(1, 2, false);
        assert!(!dev.int_low());
        write(&mut dev, 1, 1);
        assert!(dev.int_low());
        assert_eq!(read(&mut dev, 3), 2);
        assert_eq!(read(&mut dev, 4), 0x8d);
        write(&mut dev, 2, 1);
        assert!(dev.int_low());
        assert_eq!(read(&mut dev, 4), 0x0d);
        assert_eq!(read(&mut dev, 4), 0);
        assert_eq!(dev.read_count(), 2);
        write(&mut dev, 2, 1);
        assert!(!dev.int_low());
    }

    #[test]
    fn overflow_retains_oldest_by_default_and_wraps_when_configured() {
        for (cfg, first) in [(0x09, 0x81), (0x29, 0x82)] {
            let mut dev = Tca8418::default();
            write(&mut dev, 1, cfg);
            write(&mut dev, 0x1d, 3);
            write(&mut dev, 0x1e, 0xff);
            for key in 0..11 { dev.key(key / 8, key % 8, true); }
            assert_eq!(read(&mut dev, 3), 10);
            assert_eq!(read(&mut dev, 2), 9);
            assert_eq!(read(&mut dev, 4), first);
        }
    }
}
