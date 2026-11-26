//! DWC2 device Slave-mode packet storage, allocated on first FIFO traffic.
//!
//! ESP-IDF v5.5.5 components/soc/esp32s3/register/soc/{usb_reg,usb_struct}.h:
//! GRXSTSR/GRXSTSP, GAHBCFG, GNPTXFSIZ, DIEPTXF, DTXFSTS and GRSTCTL.
//! TinyUSB aa0fc2e08f1c2dd6f026a431e8989357fbb4c5bf dcd_dwc2.c:
//! handle_rxflvl_irq, handle_epin_slave, and dfifo_alloc.

use std::collections::VecDeque;

pub(super) const GLOBAL_OUT_NAK: u32 = 1;
pub(super) const OUT_DATA: u32 = 2;
pub(super) const OUT_DONE: u32 = 3;
pub(super) const SETUP_DONE: u32 = 4;
pub(super) const SETUP_DATA: u32 = 6;

pub(super) const TX_FIFOS: usize = 5;
pub(super) const EMPTY_LEVEL: u32 = 1 << 7;

#[derive(Default)]
pub(super) struct Fifo {
    rx_status: VecDeque<u32>,
    rx_data: VecDeque<u32>,
    pub tx: [TxFifo; TX_FIFOS],
}

impl Fifo {
    pub fn rx_status(&self) -> u32 { self.rx_status.front().copied().unwrap_or(0) }
    pub fn pop_rx_status(&mut self) -> u32 { self.rx_status.pop_front().unwrap_or(0) }
    pub fn pop_rx_data(&mut self) -> u32 { self.rx_data.pop_front().unwrap_or(0) }
    pub fn flush_rx(&mut self) { self.rx_status.clear(); self.rx_data.clear(); }

    // Reserve room for the payload and all its statuses before ACKing it.
    pub fn receive(&mut self, capacity: usize, status: u32, data: &[u8], completion: u32) -> bool {
        if self.rx_status.len() + self.rx_data.len() + 1 + data.len().div_ceil(4) + usize::from(completion != 0) > capacity {
            return false;
        }
        self.rx_status.push_back(status);
        self.rx_data.extend(data.chunks(4).map(|bytes| {
            let mut word = [0; 4];
            word[..bytes.len()].copy_from_slice(bytes);
            u32::from_le_bytes(word)
        }));
        if completion != 0 { self.rx_status.push_back(completion); }
        true
    }

    pub fn flush_tx(&mut self, n: usize) {
        if n == 0x10 { for tx in &mut self.tx { *tx = TxFifo::default(); } }
        else if let Some(tx) = self.tx.get_mut(n) { *tx = TxFifo::default(); }
    }
}

#[derive(Default)]
pub(super) struct TxFifo {
    words: VecDeque<u32>,
    pub bytes: usize,
    /// Bytes written into the current packet. Each packet's last word is padded separately.
    partial: usize,
}

impl TxFifo {
    pub fn words(&self) -> usize { self.words.len() }

    pub fn push(&mut self, word: u32, mps: usize, remaining: usize) -> usize {
        let count = remaining.min(mps.saturating_sub(self.partial)).min(4);
        if count != 0 {
            self.words.push_back(word);
            self.bytes += count;
            self.partial = if self.partial + count == mps || count == remaining { 0 } else { self.partial + count };
        }
        count
    }

    pub fn take_packet(&mut self, len: usize) -> Option<Vec<u8>> {
        let words = len.div_ceil(4);
        if self.bytes < len || self.words.len() < words { return None; }
        let mut data = Vec::with_capacity(words * 4);
        for word in self.words.drain(..words) { data.extend_from_slice(&word.to_le_bytes()); }
        data.truncate(len);
        self.bytes -= len;
        Some(data)
    }
}

pub(super) fn rx_status(ep: usize, data1: bool, kind: u32, len: usize) -> u32 {
    ep as u32 | (len as u32 & 0x7ff) << 4 | (data1 as u32) << 16 | kind << 17
}
