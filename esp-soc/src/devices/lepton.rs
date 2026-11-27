//! FLIR Lepton 3.x CCI (IDD 110-0144-04) and Raw14 VoSPI (Engineering
//! Datasheet 500-0659-00). Synthetic scene inputs yield four 60-packet
//! segments per frame, each packet containing 80 big-endian pixels.
use esp_periph::i2c::I2cDevice;

const PACKET: usize = 164;
const SEGMENT_PACKETS: usize = 60;

pub struct Lepton {
    cart: [u16; 20],
    pointer: u16,
    select_bytes: u8,
    low_byte: bool,
    write_high: u8,
    clock_hz: u64,
    cycle: u64,
    busy_until: u64,
    reset: bool,
    synced: bool,
    vsync_delay: u16,
    packet: [u8; PACKET],
    byte: usize,
    line: usize,
    segment: usize,
    pub packets: u64,
    pub frames: u64,
    scene: [u16; 2],
    pending_scene: [u16; 2],
    last_transfer: Option<u64>,
}

impl Lepton {
    pub const ADDRESS: u8 = 0x2a;

    pub fn new(clock_hz: u64) -> Self {
        Self { cart: [0; 20], pointer: 0, select_bytes: 2, low_byte: false, write_high: 0,
            clock_hz, cycle: 0, busy_until: 0, reset: false, synced: false, vsync_delay: 0,
            packet: [0; PACKET], byte: 0, line: 0, segment: 0, packets: 0, frames: 0,
            scene: [7300, 8300], pending_scene: [7300, 8300], last_transfer: None }
    }

    pub fn set_scene(&mut self, low: u16, high: u16) -> bool {
        if low == 0 || high < low || high > 0x3fff { return false; }
        self.pending_scene = [low, high];
        true
    }

    pub fn reset_at(&mut self, cycle: u64, reset: bool) {
        if reset && !self.reset {
            let mut next = Self::new(self.clock_hz);
            next.packets = self.packets;
            next.frames = self.frames;
            next.pending_scene = self.pending_scene;
            *self = next;
        }
        self.reset = reset;
        self.advance_to(cycle);
    }

    pub fn advance_to(&mut self, cycle: u64) { self.cycle = self.cycle.max(cycle); }
    fn pulse_period(&self) -> u64 { (self.clock_hz / 106).max(2) }
    fn pulse_width(&self) -> u64 { (self.clock_hz / 10_000).max(1) }
    pub fn vsync(&self) -> bool {
        self.synced && !self.reset && self.cycle % self.pulse_period() < self.pulse_width()
    }
    pub fn next_edge(&self) -> Option<u64> {
        if !self.synced || self.reset { return None; }
        let phase = self.cycle % self.pulse_period();
        let edge = if self.vsync() { self.pulse_width() } else { self.pulse_period() };
        Some(self.cycle.saturating_add(edge - phase))
    }

    fn new_packet(&mut self) {
        if self.line == 0 && self.segment == 0 { self.scene = self.pending_scene; }
        let segment_id = if self.line == 20 { ((self.segment + 1) as u8) << 4 } else { 0 };
        self.packet[..4].copy_from_slice(&[segment_id, self.line as u8, 0, 0]);
        let row = self.segment * 30 + self.line / 2;
        let column = (self.line & 1) * 80;
        for (x, bytes) in self.packet[4..].as_chunks_mut::<2>().0.iter_mut().enumerate() {
            let hot = (30..90).contains(&row) && (40..120).contains(&(column + x));
            *bytes = self.scene[usize::from(hot)].to_be_bytes();
        }
        // Feed the packet ID with its segment nibble masked, and the
        // two zeroed CRC bytes, to CCITT polynomial x^16+x^12+x^5+1.
        let mut crc = 0u16;
        for &byte in &self.packet[1..] {
            crc ^= u16::from(byte) << 8;
            for _ in 0..8 { crc = (crc << 1) ^ if crc & 0x8000 != 0 { 0x1021 } else { 0 }; }
        }
        self.packet[2..4].copy_from_slice(&crc.to_be_bytes());
    }

    pub fn transfer(&mut self, bytes: usize) -> Vec<u8> {
        if self.reset { return vec![0xff; bytes]; }
        if self.last_transfer.is_some_and(|last| self.cycle.saturating_sub(last) >= self.clock_hz * 185 / 1000) {
            (self.byte, self.line, self.segment) = (0, 0, 0);
        }
        self.last_transfer = Some(self.cycle);
        (0..bytes).map(|_| {
            if self.byte == 0 { self.new_packet(); }
            let value = self.packet[self.byte];
            self.byte += 1;
            if self.byte == PACKET {
                self.byte = 0;
                self.line += 1;
                self.packets += 1;
                if self.line == SEGMENT_PACKETS {
                    self.line = 0;
                    self.segment = (self.segment + 1) % 4;
                    if self.segment == 0 { self.frames += 1; }
                }
            }
            value
        }).collect()
    }

    fn run_command(&mut self, command: u16) {
        self.cart[1] = 0;
        self.busy_until = self.cycle.saturating_add(self.clock_hz / 2000);
        match command {
            0x0200 | 0x0202 | 0x0242 => self.cart[3] = 0, // ping / FFC
            0x0208 => {
                // Unassigned synthetic module identifier.
                self.cart[3] = 4;
                self.cart[4..8].copy_from_slice(&[0, 0, 0, 1]);
            }
            0x0210 | 0x0214 => {
                self.cart[3] = 1;
                self.cart[4] = if command == 0x0210 { 29715 } else { 30115 };
            }
            0x4854 => { self.cart[3] = 1; self.cart[4] = if self.synced { 5 } else { 0 }; }
            0x4855 => self.synced = self.cart[3] == 1 && self.cart[4] == 5,
            0x4858 => { self.cart[3] = 1; self.cart[4] = self.vsync_delay; }
            0x4859 => self.vsync_delay = self.cart[4],
            _ => { self.cart[1] = 0xf900; self.cart[3] = 0; } // LEP_UNDEFINED_FUNCTION_ERROR
        }
    }
}

impl I2cDevice for Lepton {
    fn start(&mut self, read: bool) -> bool {
        self.select_bytes = if read { 0 } else { 2 };
        self.low_byte = false;
        !self.reset
    }

    fn write(&mut self, value: u8) -> bool {
        match self.select_bytes {
            2 => { self.pointer = u16::from(value) << 8; self.select_bytes = 1; }
            1 => { self.pointer |= u16::from(value) & !1; self.select_bytes = 0; }
            _ if !self.low_byte => { self.write_high = value; self.low_byte = true; }
            _ => {
                let word = u16::from_be_bytes([self.write_high, value]);
                if let Some(reg) = self.cart.get_mut(usize::from(self.pointer / 2)) {
                    *reg = word;
                }
                if self.pointer == 4 { self.run_command(word); }
                self.pointer = self.pointer.wrapping_add(2);
                self.low_byte = false;
            }
        }
        true
    }

    fn read(&mut self) -> u8 {
        let mut word = self.cart.get(usize::from(self.pointer / 2)).copied().unwrap_or(0);
        if self.pointer == 2 { word |= 6 | u16::from(self.cycle < self.busy_until); }
        let value = word.to_be_bytes()[usize::from(self.low_byte)];
        if self.low_byte { self.pointer = self.pointer.wrapping_add(2); }
        self.low_byte = !self.low_byte;
        value
    }
}

#[derive(Default)]
pub struct LeptonEncoder { increment: i16, pressed: bool, pointer: u8, select: bool }

impl LeptonEncoder {
    pub const ADDRESS: u8 = 0x30;
    pub fn turn(&mut self, delta: i16) { self.increment = self.increment.saturating_add(delta); }
    pub fn press(&mut self, pressed: bool) { self.pressed = pressed; }
}

impl I2cDevice for LeptonEncoder {
    fn start(&mut self, read: bool) -> bool { self.select = !read; true }
    fn write(&mut self, data: u8) -> bool {
        if self.select {
            self.pointer = data;
            self.select = false;
        } else {
            if self.pointer == 0x20 && data == 0xff { self.increment = 0; self.pressed = false; }
            self.pointer = self.pointer.wrapping_add(1);
        }
        true
    }
    fn read(&mut self) -> u8 {
        let value = match self.pointer {
            0..=1 => self.increment.to_be_bytes()[usize::from(self.pointer)],
            0x10 => u8::from(self.pressed),
            _ => 0,
        };
        self.pointer = self.pointer.wrapping_add(1);
        value
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cci_sync_and_two_vospi_scenes() {
        let mut dev = Lepton::new(1_060_000);
        for (register, data) in [(6, [0, 1, 0, 5].as_slice()), (4, &[0x48, 0x55])] {
            assert!(dev.start(false));
            dev.write(0);
            dev.write(register);
            for &byte in data { dev.write(byte); }
        }
        assert!(dev.vsync());
        dev.advance_to(dev.next_edge().unwrap());
        assert!(!dev.vsync());
        for (low, high) in [(7000u16, 8000u16), (8000, 10000)] {
            assert!(dev.set_scene(low, high));
            let frame = dev.transfer(4 * 60 * PACKET);
            assert_eq!(&frame[4..6], &low.to_be_bytes());
            let region = &frame[(60 * PACKET + 4 + 80)..];
            assert_eq!(&region[..2], &high.to_be_bytes());
            for segment in 0..4 {
                let id = (segment * 60 + 20) * PACKET;
                assert_eq!(frame[id..id + 2], [((segment + 1) as u8) << 4, 20]);
            }
        }
        assert_eq!(dev.frames, 2);
    }
}
