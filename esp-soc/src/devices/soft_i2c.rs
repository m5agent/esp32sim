//! GPIO open-drain I²C bridge for board-detection probes.

/// A bit-level slave on the internal bus pins for software (GPIO) I2C masters: M5GFX's board
/// detection bit-bangs I²C before any controller is set up.
/// The lines are open drain with external pull-ups: a master level is low only while the pin is an
/// enabled output with a low latch. The slave pulls SDA low by driving the pad input low and
/// releases it again, so the line the master reads is the wired AND of both. Start and stop are
/// SDA edges while SCL is high; data is sampled on SCL rising edges and changed after falling ones.
#[derive(Default)]
pub struct SoftI2c { pub scl: bool, pub sda: bool, mode: SoftMode, shift: u8, bits: u8, addr: u8, out: u8, master_ack: bool, pub pull_low: bool, pub transactions: u64 }
#[derive(Default, Clone, Copy, PartialEq, Eq, Debug)]
enum SoftMode { #[default] Idle, Addr, AckSlot { read: bool }, Recv, Send, MasterAck }

pub enum SoftOp { Start(u8, bool), Write(u8, u8), Read(u8), Stop(u8), Sync }

impl SoftI2c {
    pub fn levels(&mut self, scl: bool, sda: bool, mut bus: impl FnMut(SoftOp) -> u8) {
        let (old_scl, old_sda) = (self.scl, self.sda);
        self.scl = scl; self.sda = sda;
        if scl && old_scl && sda != old_sda {
            if self.mode != SoftMode::Idle { bus(SoftOp::Stop(self.addr)); }
            bus(SoftOp::Sync);
            self.pull_low = false; self.bits = 0; self.shift = 0;
            self.mode = if sda { SoftMode::Idle } else { self.transactions += 1; SoftMode::Addr };
        } else if scl && !old_scl {
            match self.mode {
                SoftMode::Addr | SoftMode::Recv => { self.shift = self.shift << 1 | sda as u8; self.bits += 1; }
                SoftMode::MasterAck => self.master_ack = !sda,
                _ => {}
            }
        } else if !scl && old_scl {
            match self.mode {
                SoftMode::Addr if self.bits == 8 => {
                    let (addr, read) = (self.shift >> 1, self.shift & 1 != 0);
                    let ack = bus(SoftOp::Start(addr, read)) != 0;
                    self.addr = addr; self.pull_low = ack;
                    self.mode = if ack { SoftMode::AckSlot { read } } else { SoftMode::Idle };
                }
                SoftMode::Recv if self.bits == 8 => {
                    let b = self.shift;
                    let ack = bus(SoftOp::Write(self.addr, b)) != 0;
                    self.pull_low = ack;
                    self.mode = if ack { SoftMode::AckSlot { read: false } } else { SoftMode::Idle };
                }
                SoftMode::AckSlot { read } => {
                    self.bits = 0; self.shift = 0; self.pull_low = false;
                    if read { self.load_byte(&mut bus); } else { self.mode = SoftMode::Recv; }
                }
                SoftMode::Send => {
                    if self.bits < 8 { self.pull_low = self.out & (0x80 >> self.bits) == 0; self.bits += 1; }
                    else { self.pull_low = false; self.mode = SoftMode::MasterAck; }
                }
                SoftMode::MasterAck => {
                    if self.master_ack { self.load_byte(&mut bus); } else { self.pull_low = false; self.mode = SoftMode::Idle; }
                }
                _ => {}
            }
            bus(SoftOp::Sync);
        }
    }
    fn load_byte(&mut self, bus: &mut impl FnMut(SoftOp) -> u8) {
        self.out = bus(SoftOp::Read(self.addr));
        self.pull_low = self.out & 0x80 == 0; self.bits = 1; self.mode = SoftMode::Send;
    }
    pub fn idle() -> Self { Self { scl: true, sda: true, ..Default::default() } }
}
