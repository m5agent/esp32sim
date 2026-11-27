use esp_periph::i2c::I2cDevice;

/// AW9523B 16-bit IO expander, register map from the AW9523B datasheet (V1.1, section 9):
/// 0x00/0x01 input P0/P1, 0x02/0x03 output, 0x04/0x05 direction (1 = input), 0x06/0x07 interrupt
/// enable (0 = enabled), 0x10 ID (0x23), 0x11 global control, 0x12/0x13 LED mode, 0x7F soft reset.
/// INTN is low while an enabled input differs from the value last read from its input register.
pub struct Aw9523 {
    pub regs: [u8; 256],
    /// levels driven onto pins configured as inputs, by the board (bit 8+n = P1_n)
    pub external: u16,
    /// input values the firmware last read (reading 0x00/0x01 latches a port)
    pub latched: u16,
    ptr: u8,
    first: bool,
}

impl Default for Aw9523 { fn default() -> Self { Self::new() } }

impl Aw9523 {
    pub const ADDRESS: u8 = 0x58;

    pub fn new() -> Self {
        let mut a = Aw9523 { regs: [0; 256], external: 0xffff, latched: 0, ptr: 0, first: true };
        a.reset();
        a
    }
    pub fn reset(&mut self) {
        self.regs = [0; 256];
        self.regs[0x04] = 0xff;
        self.regs[0x05] = 0xff;
        self.regs[0x10] = 0x23;
        self.regs[0x12] = 0xff;
        self.regs[0x13] = 0xff;
        self.latched = self.inputs();
    }
    fn word(&self, lo: usize) -> u16 { u16::from_le_bytes([self.regs[lo], self.regs[lo + 1]]) }
    /// Output latch (`0x02/0x03`).
    pub fn outputs(&self) -> u16 { self.word(0x02) }
    /// Pin levels: driven outputs, board-driven inputs.
    pub fn inputs(&self) -> u16 {
        let dir = self.word(0x04);
        (self.outputs() & !dir) | (self.external & dir)
    }
    /// INTN asserted (low).
    pub fn intn_low(&self) -> bool {
        let enabled = !self.word(0x06) & self.word(0x04);
        (self.inputs() ^ self.latched) & enabled != 0
    }
    pub fn directions(&self) -> u16 { self.word(0x04) }
    /// Logic-high pins configured as GPIO outputs. LED-current outputs are separate.
    pub fn driven_high(&self) -> u16 { self.outputs() & !self.directions() & self.word(0x12) }
}

impl I2cDevice for Aw9523 {
    fn start(&mut self, read: bool) -> bool { if !read { self.first = true; } true }
    fn write(&mut self, b: u8) -> bool {
        if self.first { self.ptr = b; self.first = false; return true; }
        match self.ptr {
            0x00 | 0x01 | 0x10 => {}
            0x7f => if b == 0x00 { self.reset(); },
            p => self.regs[p as usize] = b,
        }
        self.ptr = self.ptr.wrapping_add(1);
        true
    }
    fn read(&mut self) -> u8 {
        let v = match self.ptr {
            0x00 => { let i = self.inputs(); self.latched = (self.latched & 0xff00) | (i & 0x00ff); i as u8 }
            0x01 => { let i = self.inputs(); self.latched = (self.latched & 0x00ff) | (i & 0xff00); (i >> 8) as u8 }
            p => self.regs[p as usize],
        };
        self.ptr = self.ptr.wrapping_add(1);
        v
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn direction_output_interrupt_and_software_reset() {
        let mut chip = Aw9523::new();
        let write = |chip: &mut Aw9523, register, bytes: &[u8]| {
            assert!(chip.start(false) && chip.write(register));
            for &byte in bytes { assert!(chip.write(byte)); }
        };
        write(&mut chip, 2, &[3, 0]);
        assert_eq!(chip.driven_high(), 0);
        chip.external = 0;
        write(&mut chip, 4, &[0xfe, 0xff]);
        assert_eq!(chip.driven_high(), 1);
        assert!(chip.intn_low());
        assert!(chip.start(false) && chip.write(0) && chip.start(true));
        assert_eq!([chip.read(), chip.read()], [1, 0]);
        assert!(!chip.intn_low());
        write(&mut chip, 0x7f, &[0]);
        assert_eq!(chip.directions(), 0xffff);
    }
}
