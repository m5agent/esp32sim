//! Demodulate GPIO-generated NEC envelopes. Carrier pulses are grouped into marks
//! when separated by less than 200 µs. The checks allow the NEC timing tolerances.

#[derive(Default)]
pub struct NecTrace {
    high: bool,
    mark: u64,
    last_fall: u64,
    bit: Option<u32>,
    word: u32,
    pub frames: u64,
    last: [Option<u32>; 2],
}

const CYCLES_PER_US: u64 = crate::periph::CPU_HZ / 1_000_000;

impl NecTrace {
    pub fn drive(&mut self, cycle: u64, high: bool) {
        if self.high == high { return; }
        self.high = high;
        if !high {
            self.last_fall = cycle;
            return;
        }
        let space = (cycle - self.last_fall) / CYCLES_PER_US;
        if space < 200 && self.last_fall != 0 { return; }
        let mark = (self.last_fall - self.mark.min(self.last_fall)) / CYCLES_PER_US;
        self.mark = cycle;
        if (8000..10000).contains(&mark) && (3500..5200).contains(&space) {
            self.bit = Some(0);
            self.word = 0;
            return;
        }
        let Some(bit) = self.bit.take() else { return };
        if !(360..750).contains(&mark) { return; }
        match space {
            350..=750 => {}
            1300..=2000 => self.word |= 1 << bit,
            _ => return,
        }
        if bit < 31 {
            self.bit = Some(bit + 1);
        } else if self.word.to_le_bytes()[2] == !self.word.to_le_bytes()[3] {
            self.frames += 1;
            self.last.rotate_left(1);
            self.last[1] = Some(self.word);
        }
    }

    pub fn recent(&self) -> String {
        self.last.iter().flatten().map(|word| format!("{word:08x}")).collect::<Vec<_>>().join(",")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn send(trace: &mut NecTrace, base: u64, word: u32) {
        let mut t = base;
        let mut pulse = |mark: u64, space: u64| {
            trace.drive(t * CYCLES_PER_US, true);
            t += mark;
            trace.drive(t * CYCLES_PER_US, false);
            t += space;
        };
        pulse(9000, 4500);
        for bit in 0..32 {
            pulse(560, if word & (1 << bit) != 0 { 1690 } else { 560 });
        }
        pulse(560, 0);
    }

    #[test]
    fn nec_payload_and_complement_come_from_pad_intervals() {
        let mut trace = NecTrace::default();
        send(&mut trace, 10, 0xcb341111);
        send(&mut trace, 100_000, 0xca351111);
        send(&mut trace, 200_000, 0xca371111);
        assert_eq!(trace.frames, 2);
        assert_eq!(trace.recent(), "cb341111,ca351111");
    }
}
