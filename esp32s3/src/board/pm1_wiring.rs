use std::{cell::RefCell, rc::Rc, sync::{Arc, Mutex}};

use esp_periph::i2c::m5::{M5pm1, M5pm1State};
use esp_soc::devices::Bmi270;
use crate::periph::CPU_HZ;

pub(super) struct Pm1Wiring {
    pub pmic: Rc<RefCell<M5pm1>>,
    state: Arc<Mutex<M5pm1State>>,
}

impl Pm1Wiring {
    pub fn new(charging_pin: u8) -> Self {
        let state = Arc::new(Mutex::new(M5pm1State::default()));
        let pmic = Rc::new(RefCell::new(M5pm1::new(state.clone()).with_wiring(CPU_HZ, charging_pin)));
        Self { pmic, state }
    }
    pub fn powered(&self) -> bool { !self.state.lock().unwrap().power_off_requested }
    pub fn command(&self, verb: &str, args: &[&str]) -> bool {
        match (verb, args) {
            ("battery", [millivolts, charging @ ("0" | "1")]) => {
                let Ok(millivolts) = millivolts.parse::<u16>() else { return false };
                let mut state = self.state.lock().unwrap();
                state.battery_mv = millivolts;
                state.charging = *charging == "1";
            }
            ("pwrkey", []) => self.pmic.borrow_mut().power_key(),
            ("pm1-gpio", [pin, high @ ("0" | "1")]) => {
                let Ok(pin @ 0..=4) = pin.parse::<u8>() else { return false };
                self.pmic.borrow_mut().gpio_input(pin, *high == "1");
            }
            _ => return false,
        }
        true
    }
    pub fn accepts(verb: &str) -> bool { matches!(verb, "battery" | "pwrkey" | "pm1-gpio") }
}

pub(super) fn imu_sample(imu: &mut Bmi270, args: &[&str]) -> bool {
    if args.len() != 6 { return false; }
    let mut values = [0.0; 6];
    for (value, arg) in values.iter_mut().zip(args) {
        let Ok(parsed) = arg.parse::<f64>() else { return false; };
        if !parsed.is_finite() { return false; }
        *value = parsed;
    }
    let [ax, ay, az, gx, gy, gz] = values;
    imu.set_sample([ax, ay, az], [gx, gy, gz], None);
    true
}
