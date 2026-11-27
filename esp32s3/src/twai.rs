//! ESP32-S3 TWAI frame-level controller. Register layout and bit names follow
//! ESP-IDF v5.4.2 components/soc/esp32s3/register/soc/twai_struct.h.
use std::collections::VecDeque;
use esp_periph::{Device, WriteEffect};

const RESET: u8 = 1;
const LISTEN_ONLY: u8 = 2;
const SELF_TEST: u8 = 4;
const SINGLE_FILTER: u8 = 8;
const RX_IRQ: u8 = 1;
const TX_IRQ: u8 = 2;
const OVERRUN_IRQ: u8 = 8;
const BUS_ERROR_IRQ: u8 = 128;

pub struct Twai {
    regs: [u32; 32],
    filter: [u8; 8],
    tx: [u8; 13],
    rx: VecDeque<[u8; 13]>,
    irq: u8,
    complete: bool,
    overrun: bool,
    log: bool,
    pub peer: bool,
    pub wired: bool,
}

impl Default for Twai {
    fn default() -> Self {
        let mut regs = [0; 32];
        regs[0] = 1;
        regs[0x34 / 4] = 96;
        Self { regs, filter: [0; 8], tx: [0; 13], rx: VecDeque::new(), irq: 0,
               complete: false, overrun: false, log: false, peer: false, wired: false }
    }
}

impl Twai {
    fn mode(&self) -> u8 { self.regs[0] as u8 }

    fn receive(&mut self, raw: [u8; 13]) {
        let extended = raw[0] & 0x80 != 0;
        let remote = raw[0] & 0x40 != 0;
        let key = if extended {
            [raw[1], raw[2], raw[3], (raw[4] & 0xf8) | u8::from(remote) << 2]
        } else {
            [raw[1], (raw[2] & 0xe0) | u8::from(remote) << 4, raw[3], raw[4]]
        };
        let matches = |start: usize, bytes: &[u8]| {
            bytes.iter().enumerate().all(|(i, &value)| {
                (value ^ self.filter[start + i]) & !self.filter[4 + start + i] == 0
            })
        };
        let accepted = if self.mode() & SINGLE_FILTER != 0 { matches(0, &key) }
            else { matches(0, &key[..2]) || matches(2, &key[..2]) };
        if !accepted { return; }
        if self.rx.len() < 8 {
            self.rx.push_back(raw);
        } else {
            self.overrun = true;
            self.irq |= OVERRUN_IRQ;
        }
    }

    fn send(&mut self, self_receive: bool) {
        if self.mode() & (RESET | LISTEN_ONLY) != 0 { return; }
        let valid = self.tx[0] & 15 <= 8;
        let ack = valid && (self.wired || self.mode() & SELF_TEST != 0);
        self.complete = ack;
        self.irq |= TX_IRQ;
        if !ack {
            self.regs[0x3c / 4] = (self.regs[0x3c / 4] + 8).min(255);
            self.regs[0x30 / 4] = 0x19; // Error code capture: TX ACK slot.
            self.irq |= BUS_ERROR_IRQ;
            return;
        }
        self.regs[0x3c / 4] = self.regs[0x3c / 4].saturating_sub(1);
        if self_receive { self.receive(self.tx); }
        if self.wired && !self_receive {
            // A deterministic peer acknowledges and returns the payload under the next CAN ID.
            let mut reply = self.tx;
            if reply[0] & 0x80 != 0 {
                let id = u32::from_be_bytes([reply[1], reply[2], reply[3], reply[4]]) >> 3;
                reply[1..5].copy_from_slice(&(((id + 1) & 0x1fff_ffff) << 3).to_be_bytes());
            } else {
                let id = u16::from_be_bytes([reply[1], reply[2]]) >> 5;
                reply[1..3].copy_from_slice(&(((id + 1) & 0x07ff) << 5).to_be_bytes());
            }
            if self.log { eprintln!("[TWAI] peer ACK and response to frame {:02x?}", &self.tx[..]); }
            self.receive(reply);
        }
    }
}

impl Device for Twai {
    fn read(&mut self, off: u32) -> u32 {
        match off {
            4 => 0,
            8 => 4 | u32::from(!self.rx.is_empty()) | (u32::from(self.overrun) << 1) |
                (u32::from(self.complete) << 3) | (u32::from(self.regs[15] >= self.regs[13]) << 6),
            12 => u32::from(std::mem::take(&mut self.irq) | if self.rx.is_empty() { 0 } else { RX_IRQ }),
            0x40..=0x70 => {
                let index = ((off - 0x40) / 4) as usize;
                let value = if self.mode() & RESET != 0 { self.filter.get(index).copied().unwrap_or(0) }
                    else { self.rx.front().map_or(0, |frame| frame[index]) };
                u32::from(value)
            }
            0x74 => self.rx.len() as u32,
            _ => self.regs.get((off / 4) as usize).copied().unwrap_or(0),
        }
    }

    fn write(&mut self, off: u32, value: u32) -> WriteEffect {
        match off {
            0 => {
                self.regs[0] = value & 15;
                if value & 1 != 0 { self.rx.clear(); self.irq = 0; self.complete = false; self.overrun = false; }
            }
            4 => {
                if value & 4 != 0 { self.rx.pop_front(); }
                if value & 8 != 0 { self.overrun = false; }
                if value & 0x11 != 0 { self.send(value & 0x10 != 0); }
            }
            0x40..=0x70 => {
                let index = ((off - 0x40) / 4) as usize;
                if self.mode() & RESET != 0 {
                    if let Some(byte) = self.filter.get_mut(index) { *byte = value as u8; }
                } else { self.tx[index] = value as u8; }
            }
            0x10 | 0x18 | 0x1c | 0x34 | 0x38 | 0x3c | 0x7c => self.regs[(off / 4) as usize] = value & 0xffff,
            _ => {}
        }
        WriteEffect::NONE
    }

    fn irq_sources(&self) -> u64 {
        let pending = self.irq | if self.rx.is_empty() { 0 } else { RX_IRQ };
        u64::from(u32::from(pending) & self.regs[4] != 0)
    }
    fn debug(&mut self, enabled: bool) { self.log = enabled; }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn controller_ack_requires_a_peer_and_rx_interrupt_releases_with_fifo() {
        let mut dev = Twai::default();
        for offset in (0x50..=0x5c).step_by(4) { dev.write(offset, 255); }
        dev.write(0, 8);
        dev.write(0x10, 0xff);
        dev.write(0x40, 2);
        dev.write(0x44, 0x10);
        dev.write(0x48, 0);
        dev.write(0x4c, 0x81);
        dev.write(0x50, 0xcd);
        dev.write(4, 1);
        assert_eq!(dev.read(8) & 8, 0);
        assert_eq!(dev.read(0x74), 0);
        assert_eq!(dev.read(12), TX_IRQ as u32 | BUS_ERROR_IRQ as u32);
        dev.wired = true;
        dev.write(4, 1);
        assert_eq!(dev.read(8) & 9, 9);
        assert_eq!(dev.read(12), RX_IRQ as u32 | TX_IRQ as u32);
        assert_eq!(dev.irq_sources(), 1);
        assert_eq!([dev.read(0x48), dev.read(0x4c), dev.read(0x50)], [32, 0x81, 0xcd]);
        dev.write(4, 4);
        assert_eq!(dev.irq_sources(), 0);
    }
}
