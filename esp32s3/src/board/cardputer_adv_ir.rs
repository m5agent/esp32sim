//! NEC receiver observing the envelope of the Cardputer-ADV IR LED's carrier.
use crate::periph::CPU_HZ;
use esp_soc::board::VirtualCycle;

#[derive(Default)]
pub(super) struct NecReceiver {
    active: bool,
    edge: VirtualCycle,
    // A leader starts at phase 1, data at phase 3, and the closing mark at 67.
    phase: u8,
    bits: u32,
    pub(super) count: u64,
    pub(super) recent: std::collections::VecDeque<(u16, u8)>,
}

impl NecReceiver {
    pub(super) fn carrier(&mut self, cycle: VirtualCycle, active: bool) {
        if self.active == active { return; }
        let us = cycle.saturating_sub(self.edge) / (CPU_HZ / 1_000_000);
        self.active = active;
        self.edge = cycle;
        // Arduino-IRremote ir_NEC.hpp: a 16-unit leader, 8-unit gap, then 32 LSB-first
        // mark/space pairs. One unit is 560 us. Allow for PWM period quantization and ISR jitter.
        let valid_mark = (350..=800).contains(&us);
        let zero = (350..=800).contains(&us);
        let one = (1300..=2050).contains(&us);
        match (self.phase, active) {
            (_, false) if (7800..=10200).contains(&us) => { self.phase = 2; self.bits = 0; }
            (2, true) if (3800..=5200).contains(&us) => self.phase = 3,
            (3..=66, false) if valid_mark => self.phase += 1,
            (4..=66, true) if zero || one => {
                self.bits |= u32::from(one) << ((self.phase - 4) / 2);
                self.phase += 1;
            }
            (67, false) if valid_mark => {
                self.phase = 0;
                let [low, high, command, inverse] = self.bits.to_le_bytes();
                if command != !inverse { return; }
                let address = if low == !high { u16::from(low) } else { u16::from_le_bytes([low, high]) };
                self.count += 1;
                if self.recent.len() == 16 { self.recent.pop_front(); }
                self.recent.push_back((address, command));
            }
            _ => self.phase = 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn frame(rx: &mut NecReceiver, bits: u32, leader: u64) {
        let mut us = 100_000 + rx.edge / (CPU_HZ / 1_000_000);
        let mut edge = |duration, level| { us += duration; rx.carrier(us * (CPU_HZ / 1_000_000), level); };
        edge(0, true); edge(leader, false); edge(4480, true);
        for i in 0..32 {
            edge(560, false); edge(if bits & (1 << i) == 0 { 560 } else { 1680 }, true);
        }
        edge(560, false);
    }

    #[test]
    fn decodes_extended_address_and_rejects_bad_leader_or_command_checksum() {
        let mut rx = NecReceiver::default();
        frame(&mut rx, 0xcb34_1111, 8960);
        frame(&mut rx, 0xcc34_1111, 8960);
        frame(&mut rx, 0xca35_1111, 7500);
        frame(&mut rx, 0xca35_5aa5, 8960);
        assert_eq!(rx.count, 2);
        assert_eq!(rx.recent, [(0x1111, 0x34), (0xa5, 0x35)]);
    }
}
