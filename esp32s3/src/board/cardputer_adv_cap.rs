//! Cardputer-ADV EXT bus pinout from UIFlow2 release/2.5.3
//! m5stack/libs/cap/{lora868,cc1101}.py.
use std::io;
use std::net::{Ipv4Addr, UdpSocket};

use esp_soc::board::{BoardEdge, SpiPins};
use esp_soc::devices::{cc1101::Cc1101, st25r3916::St25r3916, sx1262::Sx1262};
use esp_soc::pins::PinRoutes;
use esp_soc::uart::UartInput;

use crate::periph::CPU_HZ;

const SPI_CLK: u64 = 1 << 40;
const SPI_MOSI: u64 = 1 << 14;
const SPI_MISO: u8 = 39;
const RADIO_CS: u8 = 5;
const NFC_CS: u8 = 6;
const RESET_OR_POWER: u8 = 3;
const LORA_BUSY: u8 = 6;
const LORA_DIO1: u8 = 4;
const NFC_IRQ: u8 = 4;
const CC1101_GDO0: u8 = 15;
const GPS_RX: u8 = 15;
const GPS_BAUD: u32 = 115_200;
const GPS_BYTE_CYCLES: u64 = (CPU_HZ * 10).div_ceil(GPS_BAUD as u64);
const US: u64 = CPU_HZ / 1_000_000;
const MS: u64 = CPU_HZ / 1_000;
const AIR_HEADER: usize = 11;

enum Radio {
    LoRa(Box<Sx1262>),
    SubGhz(Cc1101),
}

pub(super) struct Cap {
    name: &'static str,
    radio: Radio,
    nfc: Option<St25r3916>,
    peer: Option<UdpSocket>,
    poll: u64,
    gps: Option<(u64, std::vec::IntoIter<u8>)>,
    selected: u64,
    powered: bool,
    reset: bool,
    driven: u64,
    spi_bytes: u64,
    tx: u64,
    linked_tx: u64,
    rx: u64,
    gps_bytes: u64,
}

impl Cap {
    pub fn new(name: &str, ports: Option<(u16, u16)>) -> Result<Self, String> {
        let (name, radio, nfc) = match name {
            "lora1262" => ("lora1262", Radio::LoRa(Box::default()), None),
            "lora868" => ("lora868", Radio::LoRa(Box::default()), None),
            "cc1101" => ("cc1101", Radio::SubGhz(Cc1101::default()), Some(St25r3916::default())),
            _ => return Err(format!("--cap {name}: expected lora1262, lora868 or cc1101")),
        };
        let peer = ports.map(|(local, remote)| -> io::Result<_> {
            let socket = UdpSocket::bind((Ipv4Addr::LOCALHOST, local))?;
            socket.connect((Ipv4Addr::LOCALHOST, remote))?;
            socket.set_nonblocking(true)?;
            Ok(socket)
        }).transpose().map_err(|err| format!("--cap-peer: {err}"))?;
        Ok(Self { name, radio, nfc, peer, poll: 0, gps: None, selected: 0, powered: false,
                  reset: false, driven: 0, spi_bytes: 0, tx: 0, linked_tx: 0, rx: 0, gps_bytes: 0 })
    }
    pub fn uses_uart(&self) -> bool { matches!(self.radio, Radio::LoRa(_)) }
    fn active(&self) -> bool { if self.uses_uart() { !self.reset } else { self.powered } }
    pub fn gpio(&mut self, now: u64, routes: &PinRoutes<'_>) {
        let reset_or_power = routes.software_pad_output(RESET_OR_POWER);
        let selected = [RADIO_CS, NFC_CS].into_iter().filter(|&pin| routes.software_pad_output(pin) == Some(false))
            .fold(0, |mask, pin| mask | 1 << pin);
        let deselected = self.selected & !selected;
        self.selected = selected;
        match &mut self.radio {
            Radio::LoRa(radio) => {
                let reset = reset_or_power == Some(false);
                if reset != self.reset { radio.reset(now); }
                self.reset = reset;
                if !reset && deselected & (1 << RADIO_CS) != 0 { radio.deselect(now, US); }
            }
            Radio::SubGhz(radio) => {
                let power = reset_or_power == Some(true);
                if power != self.powered {
                    *radio = Cc1101::default();
                    if let Some(nfc) = &mut self.nfc { nfc.reset(); }
                }
                self.powered = power;
                if deselected & (1 << RADIO_CS) != 0 { radio.deselect(); }
                if deselected & (1 << NFC_CS) != 0 {
                    if let Some(nfc) = &mut self.nfc { nfc.deselect(); }
                }
            }
        }
    }
    pub fn spi(&mut self, now: u64, pins: SpiPins, tx: &[u8], rx_len: usize) -> Option<Vec<u8>> {
        let nfc_mask = if self.nfc.is_some() { 1 << NFC_CS } else { 0 };
        let sel = pins.cs & ((1 << RADIO_CS) | nfc_mask);
        if pins.sclk & SPI_CLK == 0 || sel == 0 { return None; }
        let mut rx = vec![0xff; rx_len];
        if !self.active() || sel.count_ones() != 1 || pins.cs & (1 << 12) != 0 { return Some(rx); }
        let clocked = tx.len().max(rx_len);
        self.spi_bytes += clocked as u64;
        for i in 0..clocked {
            let input = if pins.mosi & SPI_MOSI != 0 { tx.get(i).copied().unwrap_or(0xff) } else { 0xff };
            let out = if sel == 1 << NFC_CS {
                self.nfc.as_mut().map_or(0xff, |nfc| nfc.exchange(input, now, US))
            } else {
                match &mut self.radio {
                    Radio::LoRa(radio) => radio.exchange(input),
                    Radio::SubGhz(radio) => radio.exchange(input, now, MS),
                }
            };
            if pins.miso == Some(SPI_MISO) {
                if let Some(byte) = rx.get_mut(i) { *byte = out; }
            }
        }
        // Hardware CS is released at the end of the hardware transfer. The
        // UIFlow2 drivers use software CS and can split commands across transfers.
        if self.selected & sel == 0 {
            match &mut self.radio {
                Radio::LoRa(radio) => radio.deselect(now, US),
                Radio::SubGhz(radio) if sel == 1 << RADIO_CS => radio.deselect(),
                _ => if let Some(nfc) = &mut self.nfc { nfc.deselect(); },
            }
        }
        Some(rx)
    }
    fn air_header(&self) -> [u8; AIR_HEADER] {
        let (kind, channel) = match &self.radio {
            Radio::LoRa(radio) => (1, radio.channel()),
            Radio::SubGhz(radio) => (2, radio.channel()),
        };
        let mut header = [kind; AIR_HEADER];
        header[1..].copy_from_slice(&channel);
        header
    }
    pub fn deadline(&self, now: u64) -> Option<u64> {
        let radio = match &self.radio {
            Radio::LoRa(radio) => radio.deadline(now), Radio::SubGhz(radio) => radio.deadline(),
        };
        radio.into_iter().chain(self.nfc.as_ref().and_then(St25r3916::deadline))
            .chain(self.peer.as_ref().map(|_| self.poll)).chain(self.gps.as_ref().map(|(due, _)| *due)).min()
    }
    pub fn advance(&mut self, now: u64) {
        if let Some(nfc) = &mut self.nfc { nfc.advance(now); }
        let tx = match &mut self.radio {
            Radio::LoRa(radio) => radio.advance(now), Radio::SubGhz(radio) => radio.advance(now),
        };
        if let Some(payload) = tx {
            self.tx += 1;
            if let Some(peer) = &self.peer {
                let mut datagram = self.air_header().to_vec();
                datagram.extend(payload);
                if peer.send(&datagram).is_ok() { self.linked_tx += 1; }
            }
        }
        let Some(peer) = &self.peer else { return; };
        if now < self.poll { return; }
        self.poll = now.saturating_add(2 * MS);
        let header = self.air_header();
        let active = self.active();
        let mut datagram = [0; AIR_HEADER + 256];
        for _ in 0..16 {
            let Ok(len) = peer.recv(&mut datagram) else { break; };
            if !active || !(AIR_HEADER..AIR_HEADER + 256).contains(&len) || datagram[..AIR_HEADER] != header { continue; }
            let data = &datagram[AIR_HEADER..len];
            if match &mut self.radio {
                Radio::LoRa(radio) => radio.receive(data), Radio::SubGhz(radio) => radio.receive(data),
            } { self.rx += 1; }
        }
    }
    pub fn levels(&self, now: u64) -> [(u8, bool); 2] {
        match &self.radio {
            Radio::LoRa(radio) => [(LORA_BUSY, self.reset || radio.busy(now)), (LORA_DIO1, radio.dio1())],
            Radio::SubGhz(radio) => [(CC1101_GDO0, self.powered && radio.gdo0()),
                                    (NFC_IRQ, self.powered && self.nfc.as_ref().is_some_and(St25r3916::irq))],
        }
    }
    pub fn edges(&mut self, now: u64, edges: &mut Vec<BoardEdge>) {
        for (pin, level) in self.levels(now) {
            let mask = 1 << pin;
            if (self.driven & mask != 0) != level {
                self.driven ^= mask;
                edges.push(BoardEdge { cycle: now, pin, level });
            }
        }
    }
    pub fn uart_rx(&mut self, now: u64) -> Vec<UartInput> {
        let Some((due, bytes)) = &mut self.gps else { return Vec::new(); };
        if now < *due { return Vec::new(); }
        let count = ((now - *due) / GPS_BYTE_CYCLES + 1).min(64) as usize;
        let data: Vec<_> = bytes.take(count).collect();
        *due = due.saturating_add(data.len() as u64 * GPS_BYTE_CYCLES);
        self.gps_bytes += data.len() as u64;
        if bytes.as_slice().is_empty() { self.gps = None; }
        vec![UartInput::new(GPS_RX, GPS_BAUD, data)]
    }
    pub fn command(&mut self, now: u64, verb: &str, args: &[&str]) -> bool {
        match (verb, args) {
            ("cap-gps", [sentence]) if self.uses_uart() && self.gps.is_none() &&
                sentence.starts_with('$') && sentence.is_ascii() && sentence.len() <= 128 => {
                let mut data = sentence.as_bytes().to_vec();
                data.extend(b"\r\n");
                self.gps = Some((now.saturating_add(GPS_BYTE_CYCLES), data.into_iter()));
                true
            }
            ("cap-tag", [uid]) => {
                let Some(nfc) = &mut self.nfc else { return false; };
                let mut bytes = Vec::new();
                if *uid != "none" {
                    if !uid.is_ascii() || !matches!(uid.len(), 8 | 14) || uid.starts_with("88") { return false; }
                    for i in (0..uid.len()).step_by(2) {
                        let Ok(byte) = u8::from_str_radix(&uid[i..i + 2], 16) else { return false; };
                        bytes.push(byte);
                    }
                }
                nfc.set_tag(bytes);
                true
            }
            _ => false,
        }
    }
    pub fn report(&self) -> String {
        format!("[emu] CAP {}: {} SPI bytes, {} TX ({} UDP), {} RX, {} GPS UART bytes, {} NFC selections",
                self.name, self.spi_bytes, self.tx, self.linked_tx, self.rx, self.gps_bytes,
                self.nfc.as_ref().map_or(0, |nfc| nfc.uid_reads))
    }
}
