//! The test fixture is a second Modbus RTU station on the K141 RS485 connector.
//! Register mapping follows M5StamPLC 1.2.1 and its library examples.
use esp_soc::uart::{UartInput, UartRoute};

use crate::periph::CPU_HZ;

pub const RX: u8 = 39;
pub const TX: u8 = 0;
pub const DIR: u8 = 46;
const ADDRESS: u8 = 1;
const BAUD: u32 = 115_200;
const GAP: u64 = CPU_HZ * 1750 / 1_000_000;

fn crc(data: &[u8]) -> u16 {
    data.iter().fold(0xffff, |mut crc, &byte| {
        crc ^= u16::from(byte);
        for _ in 0..8 { crc = (crc >> 1) ^ if crc & 1 == 0 { 0 } else { 0xa001 }; }
        crc
    })
}

fn frame(data: &[u8]) -> Vec<u8> {
    let mut bytes = data.to_vec();
    bytes.extend(crc(data).to_le_bytes());
    bytes
}

#[derive(Default)]
pub struct Peer {
    pub de: bool,
    pub hardware_rts: bool,
    guest: Vec<u8>,
    stop: Option<u64>,
    outbound: Option<Vec<u8>>,
    coils: u8,
    pub requests: u64,
    pub acknowledgments: u64,
    expected_echo: Option<[u8; 6]>,
}

impl Peer {
    pub fn deadline(&self) -> Option<u64> { self.stop }
    pub fn transmit(&mut self, cycle: u64, route: UartRoute, byte: u8) {
        if !route.transmits_on(TX) || !route.matches_baud(BAUD) || !(self.de || self.hardware_rts) { return; }
        if self.guest.len() < 256 { self.guest.push(byte); }
        self.stop = Some(cycle.saturating_add(GAP));
    }

    pub fn master_request(&mut self, register: u16, on: bool) -> bool {
        if self.outbound.is_some() || self.de { return false; }
        let [hi, lo] = register.to_be_bytes();
        let data = [ADDRESS, 5, hi, lo, if on { 255 } else { 0 }, 0];
        self.outbound = Some(frame(&data));
        self.expected_echo = Some(data);
        true
    }

    pub fn receive(&mut self, cycle: u64) -> Vec<UartInput> {
        if self.stop.is_some_and(|due| due <= cycle) && !self.de {
            self.stop = None;
            let bytes = std::mem::take(&mut self.guest);
            if bytes.len() == 8 && bytes[0] == ADDRESS && crc(&bytes) == 0 {
                let data = &bytes[..6];
                if self.expected_echo.as_ref().is_some_and(|request| data == request) {
                    self.expected_echo = None;
                    self.acknowledgments += 1;
                } else if self.expected_echo.is_none() {
                    self.outbound = self.respond(data);
                }
            }
        }
        if self.de { return Vec::new(); }
        self.outbound.take().map_or_else(Vec::new, |data| vec![UartInput::new(RX, BAUD, data)])
    }

    fn respond(&mut self, data: &[u8]) -> Option<Vec<u8>> {
        let addr = u16::from_be_bytes([data[2], data[3]]);
        let count = u16::from_be_bytes([data[4], data[5]]);
        self.requests += 1;
        let response = match data[1] {
            1 if addr < 4 && (1..=4 - addr).contains(&count) =>
                vec![ADDRESS, 1, 1, (self.coils >> addr) & ((1 << count) - 1)],
            5 if addr < 4 && matches!(count, 0 | 0xff00) => {
                self.coils = (self.coils & !(1 << addr)) | (u8::from(count != 0) << addr);
                data.to_vec()
            }
            4 if addr < 14 && (1..=14 - addr).contains(&count) => {
                let mut registers = [0; 14];
                for (offset, value) in [(8, 27.5_f32), (10, 12.5_f32), (12, 0.25_f32)] {
                    let bits = value.to_bits();
                    registers[offset] = (bits >> 16) as u16;
                    registers[offset + 1] = bits as u16;
                }
                let mut reply = vec![ADDRESS, 4, (2 * count) as u8];
                for value in &registers[usize::from(addr)..usize::from(addr + count)] {
                    reply.extend(value.to_be_bytes());
                }
                reply
            }
            1 | 4 | 5 => vec![ADDRESS, data[1] | 128, 2],
            _ => vec![ADDRESS, data[1] | 128, 1],
        };
        Some(frame(&response))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn half_duplex_crc_and_coil_round_trip() {
        let mut peer = Peer::default();
        let route = UartRoute::new(2, 1 << TX, Some(RX), Some(BAUD));
        for byte in frame(&[1, 5, 0, 2, 255, 0]) { peer.transmit(10, route, byte); }
        assert!(peer.receive(10 + GAP).is_empty());
        peer.de = true;
        for byte in frame(&[1, 5, 0, 2, 255, 0]) { peer.transmit(20, route, byte); }
        assert!(peer.receive(20 + GAP).is_empty());
        peer.de = false;
        assert_eq!(peer.receive(20 + GAP)[0].data, frame(&[1, 5, 0, 2, 255, 0]));
        peer.de = true;
        for byte in frame(&[1, 1, 0, 2, 0, 1]) { peer.transmit(30 + GAP, route, byte); }
        peer.de = false;
        assert_eq!(peer.receive(30 + GAP * 2)[0].data, frame(&[1, 1, 1, 1]));
    }
}
