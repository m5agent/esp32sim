//! Injoinic IP5306 I2C variant, register document v1.21.
//! Charger flags and gauge LEDs are supplied independently of the boost controls.
use esp_periph::i2c::I2cDevice;

pub const ADDRESS: u8 = 0x75;

pub struct Ip5306 {
    regs: [u8; 0x25],
    pointer: Option<u8>,
    selecting: bool,
    gauge: u8,
    charging: bool,
    full: bool,
    supply: Option<(u16, bool)>,
    power_off_requests: u64,
    charger_present: bool,
}

impl Default for Ip5306 {
    fn default() -> Self {
        let mut regs = [0; 0x25];
        regs[0] = 0xbe;
        regs[1] = 0x05;
        regs[0x20..=0x23].copy_from_slice(&[0x02, 0x54, 0x01, 0x20]);
        Self { regs, pointer: None, selecting: true, gauge: 0x0f, charging: false, full: false, supply: None, power_off_requests: 0, charger_present: true }
    }
}

impl Ip5306 {
    pub fn m5stick() -> Self {
        let mut pmic = Self::default();
        pmic.supply(3900, true);
        pmic
    }

    fn charge_voltage_mv(&self) -> u16 {
        let charge = self.regs[0x22];
        [4200, 4300, 4350, 4400][usize::from((charge >> 2) & 3)] + 14 * u16::from(charge & 3)
    }

    pub fn service_state(&self) -> crate::json::Json {
        use crate::json::Json;
        Json::object([
            ("battery-mv", Json::Int(self.supply.map_or(i64::from(u32::MAX), |(mv, _)| i64::from(mv)))),
            ("charger-present", Json::Bool(self.charger_present)),
            ("boost-enabled", Json::Bool(self.regs[0] & 0x20 != 0)),
            ("boost-keep-on", Json::Bool(self.regs[0] & 2 != 0)),
            ("charge-voltage-mv", Json::Int(self.charge_voltage_mv().into())),
            ("charge-current-ma", Json::Int(50 + i64::from(self.regs[0x24] & 0x1f) * 100)),
            ("battery-code", Json::Int(if self.read_gauge() == 0xff && self.supply.is_none() { 0xff } else { self.gauge.into() })),
            ("power-off-requests", Json::counter(self.power_off_requests)),
        ])
    }

    fn read_gauge(&self) -> u8 { (self.gauge << 4) | 0x0f }

    pub fn set_supply_fixture(&mut self, property: &str, value: &str) -> bool {
        match property {
            "charger-present" => {
                let Ok(present) = value.parse() else { return false };
                self.charger_present = present;
                if let Some((mv, _)) = self.supply { self.supply(mv, present); }
            }
            "battery-mv" => {
                if let Ok(mv) = value.parse::<u16>() {
                    self.supply(mv, self.charger_present);
                } else if value == "4294967295" {
                    self.supply = None;
                    self.set_battery(0, false, false);
                } else {
                    return false;
                }
            }
            _ => return false,
        }
        true
    }

    pub fn supply(&mut self, millivolts: u16, usb: bool) {
        self.charger_present = usb;
        let full = millivolts >= self.charge_voltage_mv();
        let bars = match millivolts { 4000.. => 4, 3800.. => 3, 3650.. => 2, 3450.. => 1, _ => 0 };
        self.set_battery(bars, usb && millivolts != 0 && !full, full);
        self.supply = Some((millivolts, usb));
    }

    pub fn output_on(&self) -> bool {
        self.supply.is_none_or(|(mv, usb)| usb || mv >= 3000 && self.regs[0] & 0x20 != 0)
    }

    /// The gauge exposes four cumulative LED bits, with a set bit meaning off.
    pub fn set_battery(&mut self, bars: u8, charging: bool, full: bool) {
        self.gauge = (0x0f << bars.min(4)) & 0x0f;
        self.charging = charging;
        self.full = full;
    }
}

impl I2cDevice for Ip5306 {
    fn start(&mut self, read: bool) -> bool {
        self.selecting = !read;
        true
    }

    fn write(&mut self, byte: u8) -> bool {
        if self.selecting {
            self.pointer = Some(byte);
            self.selecting = false;
        } else if let Some(reg @ (0..=2 | 6 | 0x20..=0x24)) = self.pointer {
            // Reserved fields keep their reset values through guest writes.
            let mask = match reg {
                0 => 0x37,
                1 => 0xe5,
                2 => 0x1c,
                0x20 => 0x03,
                0x21 => 0xdc,
                0x22 => 0x0f,
                0x23 => 0x20,
                0x24 => 0x1f,
                _ => 0xff,
            };
            let value = &mut self.regs[usize::from(reg)];
            let old = *value;
            *value = (old & !mask) | (byte & mask);
            if reg == 0 && old & 0x22 != 0 && *value & 0x22 == 0 {
                self.power_off_requests += 1;
            }
            if let Some((mv, usb)) = self.supply.filter(|_| reg == 0x22) {
                self.supply(mv, usb);
            }
        }
        true
    }

    fn read(&mut self) -> u8 {
        match self.pointer {
            Some(0x70) => u8::from(self.charging && self.regs[0] & 0x10 != 0) << 3,
            Some(0x71) => u8::from(self.full) << 3,
            Some(0x78) => self.read_gauge(),
            Some(0x72) => u8::from(!self.output_on()) << 2,
            Some(0x77) => 0,
            Some(reg) => self.regs.get(usize::from(reg)).copied().unwrap_or(0xff),
            None => 0xff,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn read_at(dev: &mut Ip5306, reg: u8) -> u8 {
        dev.start(false); dev.write(reg); dev.start(true); dev.read()
    }

    #[test]
    fn stick_supply_updates_the_gauge_and_boost_with_the_same_registers() {
        let mut dev = Ip5306::m5stick();
        assert_eq!(read_at(&mut dev, 0x78), 0x8f);
        assert_eq!(read_at(&mut dev, 0x70), 8);
        dev.supply(3500, false);
        assert_eq!(read_at(&mut dev, 0x78), 0xef);
        assert_eq!(read_at(&mut dev, 0x70), 0);
        dev.start(false); dev.write(0); dev.write(0);
        assert!(!dev.output_on());
        assert_eq!(read_at(&mut dev, 0x72), 4);
    }

    #[test]
    fn basic_firmware_writes_preserve_reserved_bits_and_no_cell_is_not_charging() {
        let mut dev = Ip5306::default();
        assert_eq!(read_at(&mut dev, 0x22), 1);
        for (reg, write, expected) in [(0, 0x31, 0xb9), (0x24, 0xc1, 1), (0x22, 9, 9)] {
            dev.start(false);
            dev.write(reg);
            dev.write(write);
            assert_eq!(read_at(&mut dev, reg), expected);
        }
        assert_eq!(read_at(&mut dev, 0x70), 0);
        assert_eq!(read_at(&mut dev, 0x71), 0);
        assert_eq!(read_at(&mut dev, 0x78), 0xff);
    }

    #[test]
    fn gauge_and_charge_flags_follow_inputs_and_charge_enable() {
        let mut dev = Ip5306::default();
        assert_eq!(read_at(&mut dev, 0x78), 0xff);
        assert_eq!(read_at(&mut dev, 0x70), 0);
        dev.set_battery(2, true, false);
        assert_eq!(read_at(&mut dev, 0x78) >> 4, 0x0c);
        assert_eq!(read_at(&mut dev, 0x70), 8);
        dev.start(false); dev.write(0); dev.write(0xae);
        assert_eq!(read_at(&mut dev, 0x00), 0xae);
        assert_eq!(read_at(&mut dev, 0x70), 0);
        dev.set_battery(4, true, true);
        assert_eq!(read_at(&mut dev, 0x78) >> 4, 0);
        assert_eq!(read_at(&mut dev, 0x71), 8);
    }
}
