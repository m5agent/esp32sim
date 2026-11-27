//! Tab5's I²C battery monitor and SD backing store.
use esp_periph::i2c::I2cDevice;

/// TI INA226 SBOS547B. Tab5 M5Unified 0.3.0 uses a 0.005-ohm shunt at 0x41.
pub struct Ina226 {
    regs: [u16; 8], ptr: u8, phase: u8, upper: u8, mv: u16, ma: i16, latch: u16,
}
impl Ina226 {
    pub const ADDRESS: u8 = 0x41;
    pub fn new() -> Self { Self { regs: [0x4127, 0, 0, 0, 0, 0, 0, 0], ptr: 0, phase: 0, upper: 0, mv: 7400, ma: -120, latch: 0 } }
    pub fn supply(&mut self, mv: u16, ma: i16) { self.mv = mv; self.ma = ma; }
    fn reg(&self) -> u16 {
        // Shunt polarity is reversed on Tab5. mA * 5 µV per mA, then 2.5 µV per LSB.
        let shunt = (-2 * i32::from(self.ma)).clamp(i16::MIN as i32, i16::MAX as i32);
        let bus = u32::from(self.mv) * 4 / 5;
        let current = shunt * i32::from(self.regs[5]) / 2048;
        match self.ptr {
            0x01 => shunt as u16,
            0x02 => bus as u16,
            0x03 => (current.unsigned_abs().min(32767) * bus / 20000) as u16,
            0x04 => current.clamp(i16::MIN as i32, i16::MAX as i32) as u16,
            0xfe => 0x5449, 0xff => 0x2260,
            r => self.regs.get(usize::from(r)).copied().unwrap_or(0),
        }
    }
}
impl I2cDevice for Ina226 {
    fn start(&mut self, read: bool) -> bool { self.phase = 0; if read { self.latch = self.reg(); } true }
    fn write(&mut self, byte: u8) -> bool {
        match self.phase {
            0 => self.ptr = byte,
            1 => self.upper = byte,
            2 if matches!(self.ptr, 0 | 5..=7) => {
                let value = u16::from_be_bytes([self.upper, byte]);
                if self.ptr == 0 && value & 0x8000 != 0 { self.regs = [0x4127, 0, 0, 0, 0, 0, 0, 0]; }
                else { self.regs[self.ptr as usize] = value; }
            }
            _ => {}
        }
        self.phase = self.phase.saturating_add(1);
        true
    }
    fn read(&mut self) -> u8 {
        let byte = self.latch.to_be_bytes()[usize::from(self.phase & 1)];
        self.phase ^= 1;
        byte
    }
}


/// Two protocol front ends of the same physical socket use one backing store.
#[derive(Clone)]
pub struct SocketStore(pub std::rc::Rc<std::cell::RefCell<Box<dyn esp_periph::sdmmc::SdStore>>>);
impl esp_periph::sdmmc::SdStore for SocketStore {
    fn len(&self) -> u64 { self.0.borrow().len() }
    fn read(&mut self, lba: u64, buf: &mut [u8; 512]) -> std::io::Result<()> { self.0.borrow_mut().read(lba, buf) }
    fn write(&mut self, lba: u64, buf: &[u8; 512]) -> std::io::Result<()> { self.0.borrow_mut().write(lba, buf) }
    fn erase(&mut self, lo: u64, hi: u64) -> std::io::Result<()> { self.0.borrow_mut().erase(lo, hi) }
    fn describe(&self) -> String { self.0.borrow().describe() }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn write(device: &mut impl I2cDevice, addr: u8, bytes: &[u8]) {
        assert!(device.start(false) && device.write(addr));
        for &byte in bytes { assert!(device.write(byte)); }
        device.stop();
    }
    fn read(device: &mut impl I2cDevice, addr: u8, count: usize) -> Vec<u8> {
        assert!(device.start(false) && device.write(addr) && device.start(true));
        let bytes = (0..count).map(|_| device.read()).collect();
        device.stop();
        bytes
    }

    #[test]
    fn ina226_reports_big_endian_bus_voltage_and_reversed_charge_current() {
        let mut ina = Ina226::new();
        ina.supply(7980, 410);
        write(&mut ina, 5, &2048u16.to_be_bytes());
        assert_eq!(read(&mut ina, 2, 2), 6384u16.to_be_bytes());
        assert_eq!(read(&mut ina, 4, 2), (-820i16).to_be_bytes());
        assert_eq!(read(&mut ina, 0xff, 2), [0x22, 0x60]);
    }
}
