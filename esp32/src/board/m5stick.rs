//! Original M5Stick K016, ESP32-WROOM32 and 64x128 SH1107 OLED.
//! Wiring follows the K016 schematic and factory pin assignments.
use super::stickc::InternalBus;
use esp_periph::i2c::I2cDevice;
use esp_soc::{
    board::{BoardModel, SpiPins, VirtualCycle},
    devices::{ip5306::{ADDRESS as IP5306_ADDRESS, Ip5306}, mpu9250::{Ak8963, Mpu9250}, sh1107::Sh1107, SpiBitBang},
    pins::PinRoutes,
};
use std::{cell::RefCell, rc::Rc};

const CLOCK: u8 = 18;
const DATA: u8 = 23;
const CS: u8 = 14;
const DC: u8 = 27;
const RESET: u8 = 33;

pub(super) struct M5Stick {
    panel: Sh1107,
    imu: Rc<RefCell<Mpu9250>>,
    mag: Rc<RefCell<Ak8963>>,
    pmic: Rc<RefCell<Ip5306>>,
    dc: bool,
    reset: bool,
    spi: SpiBitBang,
    battery: u16,
    usb: bool,
    buzzer: Option<(f64, u32)>,
    buzzer_updates: u64,
    ir_frames: u64,
}

impl M5Stick {
    pub(super) fn new() -> Self {
        let imu = Rc::new(RefCell::new(Mpu9250::default()));
        Self { panel: Sh1107::default(), mag: Rc::new(RefCell::new(Ak8963::new(imu.clone()))),
            imu, pmic: Rc::new(RefCell::new(Ip5306::m5stick())), dc: false, reset: false,
            spi: SpiBitBang::new(CLOCK, DATA, CS), battery: 3900, usb: true,
            buzzer: None, buzzer_updates: 0, ir_frames: 0 }
    }
}

impl BoardModel for M5Stick {
    fn name(&self) -> &'static str { "m5stack-m5stick" }
    fn observes_gpio_pads(&self) -> bool { true }
    fn soc_powered(&self) -> Option<bool> { Some(self.pmic.borrow().output_on()) }

    fn gpio_waveform_at(&mut self, _cycle: VirtualCycle, routes: PinRoutes<'_>) {
        let output = |pin| routes.software_pad_output(pin);
        let reset = output(RESET) == Some(false);
        if reset && !self.reset { self.panel.reset(); }
        self.reset = reset;
        self.dc = output(DC) == Some(true);
        self.spi.pin(CS, output(CS) != Some(false));
        self.spi.pin(DATA, output(DATA) == Some(true));
        let clock = output(CLOCK);
        if let Some(byte) = self.spi.pin(CLOCK, clock == Some(true)).filter(|_| clock.is_some() && !reset) {
            self.panel.write(self.dc, byte);
        }
    }
    fn input_levels(&self) -> Vec<(u8, bool)> { vec![(21, true), (22, true), (35, true)] }
    fn named_pin(&self, name: &str) -> Option<u8> { matches!(name, "a" | "btn1").then_some(35) }

    fn i2c_devices(&mut self) -> Vec<(u8, u8, Box<dyn I2cDevice>)> {
        let devices = [
            (Mpu9250::ADDRESS, InternalBus(self.imu.clone())),
            (Ak8963::ADDRESS, InternalBus(self.mag.clone())),
            (IP5306_ADDRESS, InternalBus(self.pmic.clone())),
        ];
        (0..2).flat_map(|bus| devices.clone().into_iter().map(move |(a, dev)|
            (bus, a, Box::new(dev) as Box<dyn I2cDevice>))).collect()
    }
    fn uses_spi_pins(&self) -> bool { true }
    fn spi_transfer_pins(&mut self, _host: u8, pins: SpiPins, tx: &[u8], rx_len: usize) -> Vec<u8> {
        if !self.reset && self.pmic.borrow().output_on()
            && pins.sclk & (1 << CLOCK) != 0 && pins.mosi & (1 << DATA) != 0 && pins.cs & (1 << CS) != 0 {
            for &value in tx { self.panel.write(self.dc, value); }
        }
        vec![0xff; rx_len]
    }
    fn display(&self) -> Option<(u32, u32, Vec<u16>, u64)> {
        let lit = !self.reset && self.pmic.borrow().output_on();
        let pixels = (0..128).flat_map(|y| (0..64).map(move |x|
            if lit { self.panel.pixel(x, y) } else { 0 })).collect();
        Some((64, 128, pixels, self.panel.generation))
    }
    fn display_version(&self) -> u64 { self.panel.generation }
    fn rmt_frame(&mut self, pin: u8, _bits: &[bool]) {
        if pin == 17 { self.ir_frames += 1; }
    }
    fn advance_inputs_with_clock(&mut self, cycle: VirtualCycle, clock: &dyn Fn(u8) -> Option<(f64, u32)>) {
        self.imu.borrow_mut().advance_to(cycle, crate::periph::CPU_HZ);
        let pwm = clock(26).filter(|&(_, duty)| duty > 0 && duty < 65535);
        if pwm != self.buzzer {
            self.buzzer = pwm;
            self.buzzer_updates += 1;
        }
    }
    fn accepts_command(&self, verb: &str) -> bool { matches!(verb, "imu" | "mag" | "battery-mv" | "vbus-mv") }
    fn board_command(&mut self, _cycle: VirtualCycle, verb: &str, args: &[&str]) -> bool {
        let values = || args.iter().map(|arg| arg.parse::<f64>().ok().filter(|value| value.is_finite())).collect::<Option<Vec<_>>>();
        match (verb, args) {
            ("imu", [_, _, _, _, _, _]) => {
                let Some(sample) = values().and_then(|v| v.try_into().ok()) else { return false; };
                self.imu.borrow_mut().set_sample(sample);
            }
            ("mag", [_, _, _]) => {
                let Some(sample) = values().and_then(|v| v.try_into().ok()) else { return false; };
                self.mag.borrow_mut().set_sample(sample);
            }
            ("battery-mv" | "vbus-mv", [mv]) => {
                let Ok(mv) = mv.parse::<u16>() else { return false; };
                if verb == "battery-mv" { self.battery = mv; } else { self.usb = mv >= 4000; }
                self.pmic.borrow_mut().supply(self.battery, self.usb);
            }
            _ => return false,
        }
        true
    }
    fn report(&self) -> String {
        format!("[m5stick] oled_writes={} buzzer_updates={} ir_frames={}",
            self.panel.writes, self.buzzer_updates, self.ir_frames)
    }
}
