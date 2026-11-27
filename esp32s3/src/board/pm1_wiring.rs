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
    pub fn service_state(&self, pa: Option<u8>) -> esp_soc::json::Json {
        use esp_soc::json::Json;
        let pmic = self.pmic.borrow();
        let gpio_inputs = pmic.gpio_inputs();
        let state = self.state.lock().unwrap();
        let Json::Obj(mut fields) = Json::object([
            ("battery_mv", Json::Int(i64::from(state.battery_mv))),
            ("charging", Json::Bool(state.charging)),
            ("vbus_mv", Json::Int(i64::from(state.vbus_mv))),
            ("vbat-mv", Json::Int(state.battery_mv.into())),
            ("vin-mv", Json::Int(state.vbus_mv.into())),
            ("vinout-mv", Json::Int(state.vinout_mv.into())),
            ("temperature-c", Json::Int(state.temperature_c.into())),
            ("adc1-mv", Json::Int(state.adc_mv[0].into())),
            ("adc2-mv", Json::Int(state.adc_mv[1].into())),
            ("wake-source", Json::Int(pmic.registers()[0x05].into())),
            ("driven_high", Json::Int(i64::from(state.driven_high))),
            ("registers", Json::register_bytes(pmic.registers().iter().copied())),
            ("gpio-input-bits", Json::Int(gpio_inputs.into())),
        ]) else { unreachable!() };
        if let Some(pin) = pa {
            fields.push(("pa-enabled".into(), Json::Bool(pmic.driven_high() & (1 << pin) != 0)));
        }
        Json::Obj(fields)
    }
    pub fn set_service_property(&self, property: &str, value: &str) -> bool {
        if property == "gpio-input-bits" {
            let Ok(bits @ 0..=31) = value.parse::<u8>() else { return false };
            let mut pmic = self.pmic.borrow_mut();
            for pin in 0..5 { pmic.gpio_input(pin, bits & (1 << pin) != 0); }
            return true;
        }
        if property == "wake-source" {
            let Ok(bits @ 0..=127) = value.parse::<u8>() else { return false };
            self.pmic.borrow_mut().set_wake_source(bits);
            return true;
        }
        self.state.lock().unwrap().set_supply_fixture(property, value)
    }

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
