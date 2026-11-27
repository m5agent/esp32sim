//! ESP32-S3 non-DMA RMT receive channels, ESP-IDF v5.4.2 rmt_reg.h and rmt_ll.h.
//! The input is the GPIO-matrix level, already demodulated by the board receiver.
use crate::rmt::RMT_MEM_WORDS;

#[derive(Default)]
struct Channel {
    conf0: u32,
    conf1: u32,
    armed: bool,
    level: Option<bool>,
    start: Option<u64>,
    half: Option<u16>,
    written: usize,
}

#[derive(Default)]
pub(crate) struct RxChannels([Channel; 4]);

impl RxChannels {
    pub fn handles(off: u32) -> bool { matches!(off, 0x30..=0x4c | 0x60..=0x6c) }
    pub fn read(&self, off: u32) -> u32 {
        if off >= 0x60 {
            let index = ((off - 0x60) / 4) as usize;
            let channel = &self.0[index];
            return ((index + 4) * RMT_MEM_WORDS + channel.written) as u32
                | if channel.armed { 2 << 22 } else { 0 };
        }
        let channel = &self.0[((off - 0x30) / 8) as usize];
        if off & 4 == 0 { channel.conf0 } else { channel.conf1 }
    }

    pub fn write(&mut self, off: u32, value: u32) {
        if off >= 0x60 { return; }
        let channel = &mut self.0[((off - 0x30) / 8) as usize];
        if off & 4 == 0 { channel.conf0 = value; return; }
        channel.conf1 = value & !0xc006; // reset/update strobes read as zero
        if value & 2 != 0 {
            channel.written = 0;
            channel.half = None;
        }
        if value & (1 << 15) != 0 {
            let enabled = value & 1 != 0;
            if enabled && !channel.armed {
                channel.start = None;
                channel.level = None;
                channel.half = None;
            }
            channel.armed = enabled;
        }
    }

    pub fn active(&self) -> bool { self.0.iter().any(|channel| channel.armed) }

    pub fn next_deadline(&self, ratio: u64) -> Option<u64> {
        self.0.iter().filter(|channel| channel.armed).filter_map(|channel| channel.deadline(ratio)).min()
    }

    pub fn sample(&mut self, mem: &mut [u32], irq: &mut u32, cycle: u64, ratio: u64, level: impl Fn(usize) -> Option<bool>) -> bool {
        let irq_before = *irq;
        for (index, channel) in self.0.iter_mut().enumerate() {
            if !channel.armed { continue; }
            let Some(level) = level(index) else { continue };
            let old = channel.level.replace(level);
            let Some(old) = old else { continue };
            let step = channel.step(ratio);
            let base = (index + 4) * RMT_MEM_WORDS;
            let capacity = (((channel.conf0 >> 24 & 15) as usize).max(1) * RMT_MEM_WORDS)
                .min(mem.len() - base);
            if let Some(deadline) = channel.deadline(ratio).filter(|&at| at <= cycle) {
                let elapsed = deadline - channel.start.unwrap();
                channel.push(&mut mem[base..base + capacity], old, elapsed / step);
                if let Some(half) = channel.half.take() {
                    if channel.written < capacity {
                        mem[base + channel.written] = u32::from(half);
                        channel.written += 1;
                    }
                }
                channel.armed = false;
                channel.start = None;
                *irq |= 1 << (16 + index);
                continue;
            }
            if old == level { continue; }
            if let Some(start) = channel.start {
                let elapsed = cycle.saturating_sub(start);
                // Filter thresholds run on the undivided source clock.
                let filter = u64::from(channel.conf1 >> 5 & 0xff) * ratio;
                if channel.conf1 & 0x10 != 0 && elapsed < filter {
                    channel.level = Some(old);
                    continue;
                }
                channel.push(&mut mem[base..base + capacity], old, elapsed / step);
                if channel.written == capacity {
                    channel.armed = false;
                    *irq |= 1 << (16 + index) | 1 << (20 + index);
                    continue;
                }
            }
            channel.start = Some(cycle);
        }
        *irq != irq_before
    }
}

impl Channel {
    fn step(&self, ratio: u64) -> u64 { ratio * u64::from((self.conf0 & 0xff).max(1)) }
    fn deadline(&self, ratio: u64) -> Option<u64> {
        let idle = u64::from((self.conf0 >> 8 & 0x7fff).max(1));
        self.start.map(|start| start.saturating_add(idle * self.step(ratio)))
    }
    fn push(&mut self, mem: &mut [u32], level: bool, ticks: u64) {
        let symbol = ticks.clamp(1, 0x7fff) as u16 | (u16::from(level) << 15);
        if let Some(low) = self.half.take() {
            if let Some(word) = mem.get_mut(self.written) {
                *word = u32::from(low) | (u32::from(symbol) << 16);
                self.written += 1;
            }
        } else {
            self.half = Some(symbol);
        }
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn rx_channel_stores_pulses_and_stops_at_idle_timeout() {
        let mut dma = crate::rmt::RmtDma::new(240_000_000);
        dma.write(0x30, (1 << 24) | (20_000 << 8) | 80);
        dma.write(0x34, 0x800b);
        let route = |level| move |channel| (channel == 0).then_some(level);
        dma.sample_rx(0, route(true));
        dma.sample_rx(240, route(false));
        dma.sample_rx(2_160_240, route(true)); // 9000 us low
        dma.sample_rx(3_240_240, route(false)); // 4500 us high
        dma.sample_rx(3_374_640, route(true)); // 560 us low
        assert_eq!(dma.rx_deadline(), Some(8_174_640));
        assert!(dma.sample_rx(8_174_640, route(true)));
        assert_eq!(dma.mem[4 * 48], (1 << 31) | (4500 << 16) | 9000);
        assert_eq!(dma.mem[4 * 48 + 1], (1 << 31) | (20_000 << 16) | 560);
        assert_eq!(dma.read(0x60) & 0x3ff, 4 * 48 + 2);
        assert_eq!(dma.int_raw, 1 << 16);
        assert!(!dma.rx_active());
    }
}
