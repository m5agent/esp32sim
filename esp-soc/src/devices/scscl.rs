//! Feetech SCSCL half-duplex packet and register subset used by StackChan.
//! Addresses and word order follow M5StackChan 1.1.1's FTServo_Arduino SCSCL driver.
struct Servo {
    regs: [u8; 71],
    pending: Option<(u8, Vec<u8>)>,
    movement: Option<(u64, u64, u16, u16)>,
    rotation: Option<(u64, u16, i16)>,
}

impl Servo {
    fn new(id: u8, zero: u16) -> Self {
        let mut regs = [0; 71];
        regs[5] = id;
        regs[11..13].copy_from_slice(&1000u16.to_be_bytes());
        regs[40] = 1;
        regs[42..44].copy_from_slice(&zero.to_be_bytes());
        regs[56..58].copy_from_slice(&zero.to_be_bytes());
        regs[62] = 74;
        regs[63] = 25;
        Self { regs, pending: None, movement: None, rotation: None }
    }

    fn position(&self) -> u16 { u16::from_be_bytes([self.regs[56], self.regs[57]]) }
    fn set_position(&mut self, position: u16) {
        self.regs[56..58].copy_from_slice(&position.to_be_bytes());
    }
    fn advance(&mut self, cycle: u64, hz: u64) {
        if let Some((start, initial, effort)) = self.rotation {
            // Virtual speed scale: full effort is one 1024-count revolution per second.
            let elapsed = i128::from(cycle.saturating_sub(start));
            let offset = elapsed * i128::from(effort) * 1024 / (i128::from(hz.max(1)) * 1000);
            self.set_position((i128::from(initial) + offset).rem_euclid(1024) as u16);
            return;
        }
        let Some((start, end, from, to)) = self.movement else { return };
        let elapsed = cycle.saturating_sub(start).min(end - start);
        let pos = i64::from(from) + (i64::from(to) - i64::from(from)) * elapsed as i64 / (end - start) as i64;
        self.set_position(pos as u16);
        if cycle >= end {
            self.movement = None;
            self.regs[66] = 0;
        }
    }
    fn write(&mut self, address: u8, data: &[u8], cycle: u64, hz: u64) -> bool {
        let start = usize::from(address);
        let end = start + data.len();
        let Some(target) = self.regs.get_mut(start..end).filter(|_| start >= 5 && end <= 56) else { return false };
        target.copy_from_slice(data);
        let wheel_mode = self.regs[9..13] == [0; 4];
        if self.regs[40] == 0 || (start < 13 && end > 9) {
            self.movement = None;
            self.rotation = None;
            self.regs[66] = 0;
        }
        if self.regs[40] == 0 { return true; }
        if wheel_mode && start <= 44 && end >= 46 {
            let raw = u16::from_be_bytes([self.regs[44], self.regs[45]]);
            let effort = (raw & 1023).min(1000) as i16 * if raw & 1024 == 0 { 1 } else { -1 };
            self.movement = None;
            self.rotation = (effort != 0).then(|| (cycle, self.position(), effort));
            self.regs[66] = u8::from(effort != 0);
        } else if !wheel_mode && start <= 42 && end >= 44 {
            let target = u16::from_be_bytes([self.regs[42], self.regs[43]]).min(1000);
            let time_ms = u16::from_be_bytes([self.regs[44], self.regs[45]]).max(1);
            let due = cycle.saturating_add(u64::from(time_ms) * (hz / 1000).max(1));
            if due > cycle {
                self.movement = Some((cycle, due, self.position(), target));
                self.regs[66] = 1;
            }
        }
        true
    }
}

pub struct ScsclPair {
    devices: [Servo; 2],
    frame: Vec<u8>,
    response: Option<(u64, Vec<u8>)>,
    hz: u64,
    last_byte: u64,
    pub reads: u64,
    pub writes: u64,
}

impl ScsclPair {
    pub fn new(hz: u64, zeros: [u16; 2]) -> Self {
        Self { devices: [Servo::new(1, zeros[0]), Servo::new(2, zeros[1])], frame: Vec::with_capacity(32),
               response: None, hz, last_byte: 0, reads: 0, writes: 0 }
    }
    /// Apply a manual displacement in encoder counts for a home calibration fixture.
    pub fn pose(&mut self, positions: [u16; 2]) {
        for (servo, position) in self.devices.iter_mut().zip(positions) {
            servo.set_position(position.min(1000));
            servo.movement = None;
            servo.rotation = None;
            servo.regs[66] = 0;
        }
    }
    pub fn positions(&self) -> [u16; 2] { self.devices.each_ref().map(Servo::position) }
    pub fn deadline(&self) -> Option<u64> { self.response.as_ref().map(|reply| reply.0) }
    pub fn receive(&mut self, cycle: u64) -> Option<Vec<u8>> {
        self.deadline().filter(|due| *due <= cycle)?;
        self.response.take().map(|reply| reply.1)
    }
    pub fn advance(&mut self, cycle: u64) {
        for servo in &mut self.devices { servo.advance(cycle, self.hz); }
    }
    pub fn transmit(&mut self, cycle: u64, byte: u8) {
        if cycle.saturating_sub(self.last_byte) > self.hz / 1000 { self.frame.clear(); }
        self.last_byte = cycle;
        if self.frame.len() < 2 && byte != 0xff {
            self.frame.clear();
            return;
        }
        if self.frame.len() == 2 && byte == 0xff { return; }
        self.frame.push(byte);
        let Some(&length) = self.frame.get(3) else { return };
        if length < 2 {
            self.frame.clear();
        } else if self.frame.len() == usize::from(length) + 4 {
            let mut frame = std::mem::take(&mut self.frame);
            if frame[2..].iter().fold(0u8, |sum, b| sum.wrapping_add(*b)) == 0xff {
                self.advance(cycle);
                self.dispatch(cycle, frame[2], frame[4], &frame[5..frame.len() - 1]);
            }
            frame.clear();
            self.frame = frame;
        }
    }
    fn dispatch(&mut self, cycle: u64, id: u8, op: u8, data: &[u8]) {
        if id == 0xfe {
            if let (0x83, [reg, width, payload @ ..]) = (op, data) {
                let width = usize::from(*width);
                if width > 0 && payload.len().is_multiple_of(width + 1) {
                    for row in payload.chunks_exact(width + 1) {
                        if let Some(servo) = self.devices.iter_mut().find(|s| s.regs[5] == row[0]) {
                            self.writes += u64::from(servo.write(*reg, &row[1..], cycle, self.hz));
                        }
                    }
                }
            } else if op == 5 {
                for servo in &mut self.devices {
                    if let Some((reg, bytes)) = servo.pending.take() {
                        self.writes += u64::from(servo.write(reg, &bytes, cycle, self.hz));
                    }
                }
            }
            return;
        }
        let Some(servo) = self.devices.iter_mut().find(|s| s.regs[5] == id) else { return };
        let mut status = 0;
        let mut reply = vec![0xff, 0xff, id, 2, 0];
        match (op, data) {
            (1, []) => {}
            (2, [address, count]) if *count > 0 => {
                let start = usize::from(*address);
                if let Some(bytes) = servo.regs.get(start..start + usize::from(*count)) {
                    self.reads += 1;
                    reply.extend_from_slice(bytes);
                } else { status = 0x20; }
            }
            (3, [reg, bytes @ ..]) if !bytes.is_empty() => {
                if servo.write(*reg, bytes, cycle, self.hz) { self.writes += 1; } else { status = 0x20; }
            }
            (4, [reg, bytes @ ..]) if !bytes.is_empty() => servo.pending = Some((*reg, bytes.to_vec())),
            (5, []) => {
                if let Some((reg, bytes)) = servo.pending.take() {
                    self.writes += u64::from(servo.write(reg, &bytes, cycle, self.hz));
                }
            }
            _ => status = 0x40,
        }
        reply[3] = (reply.len() - 3) as u8;
        reply[4] = status;
        reply.push(!reply[2..].iter().fold(0u8, |sum, b| sum.wrapping_add(*b)));
        self.response = Some((cycle.saturating_add(self.hz / 20_000), reply));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn addressed_reads_motion_and_checksum_rejection() {
        let mut pair = ScsclPair::new(1_000_000, [460, 620]);
        let packet = |pair: &mut ScsclPair, cycle, body: &[u8], good: bool| {
            for byte in [0xff, 0xff].into_iter().chain(body.iter().copied())
                .chain([!body.iter().fold(0u8, |sum, b| sum.wrapping_add(*b)) ^ u8::from(!good)]) {
                pair.transmit(cycle, byte);
            }
        };
        packet(&mut pair, 0, &[1, 4, 2, 56, 2], false);
        assert!(pair.receive(200).is_none());
        packet(&mut pair, 200, &[1, 9, 3, 42, 2, 224, 0, 20, 0, 0], true);
        assert!(pair.receive(240).is_none());
        assert_eq!(pair.receive(250).unwrap()[4], 0);
        pair.advance(10_200);
        assert_eq!(pair.positions(), [598, 620]);
        pair.advance(20_200);
        packet(&mut pair, 21_000, &[1, 4, 2, 56, 2], true);
        assert_eq!(pair.receive(21_050).unwrap()[5..7], [2, 224]);
        assert_eq!(pair.positions(), [736, 620]);
    }
}
