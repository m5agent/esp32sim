use std::{cell::{Cell, RefCell}, rc::Rc, sync::{Arc, Mutex}, collections::BTreeSet};

use esp_periph::i2c::m5::{M5ioe1, M5ioe1State, M5pm1};
use esp_soc::board::{BoardEdge, BoardModel, I2sCodecs, SpiPins, VirtualCycle};
use esp_soc::devices::{Bmi270, es8311::Es8311, imu_motion::ImuMotion, rx8130::Rx8130};
use esp_soc::pins::PinRoutes;

use super::{i2c_wiring::I2cWiring, pm1_wiring::{imu_sample, Pm1Wiring}};
use crate::periph::CPU_HZ;
use esp_soc::devices::co5300::{Co5300, Co5300Scan};

use crate::i2c::{Cst820, I2cDevice, TouchState};

pub const PIN_STOPWATCH_TE: u8 = 38;
pub const PIN_STOPWATCH_BUTTON_A: u8 = 2;
pub const PIN_STOPWATCH_BUTTON_B: u8 = 1;

struct PanelState {
    panel: Co5300,
    active: bool,
    version: u64,
    power_transitions: u64,
}

struct PowerWiring {
    panel: Mutex<PanelState>,
    power: Pm1Wiring,
    expander: Arc<Mutex<M5ioe1State>>,
    touch: Arc<Mutex<TouchState>>,
    codec: Es8311,
    audio_powered: Cell<bool>,
}

impl PowerWiring {
    fn sync(&self) {
        let off = !self.power.powered();
        let expander = *self.expander.lock().expect("StopWatch expander state mutex poisoned");
        let audio_powered = !off && expander.audio_rail;
        if self.audio_powered.replace(audio_powered) && !audio_powered { self.codec.power_loss(); }
        let active = !off && expander.oled_rail && expander.oled_reset_high;
        let mut panel = self.panel.lock().expect("StopWatch panel mutex poisoned");
        if panel.active && !active {
            panel.panel.reset();
            panel.version += 1;
        }
        if panel.active != active { panel.power_transitions += 1; }
        panel.active = active;
        if off || !expander.touch_reset_high {
            *self.touch.lock().expect("StopWatch touch mutex poisoned") = TouchState::default();
        }
    }
}

// The C152's Q5/Q6 isolate the ES8311 I2C branch when AU_L3B is off.
struct AudioPort {
    codec: Es8311,
    wiring: Rc<PowerWiring>,
}

impl I2cDevice for AudioPort {
    fn start(&mut self, read: bool) -> bool { self.wiring.audio_powered.get() && self.codec.start(read) }
    fn write(&mut self, byte: u8) -> bool { self.wiring.audio_powered.get() && self.codec.write(byte) }
    fn read(&mut self) -> u8 { if self.wiring.audio_powered.get() { self.codec.read() } else { 0xff } }
}

struct PowerDevice {
    device: Rc<RefCell<dyn I2cDevice>>,
    wiring: Rc<PowerWiring>,
}

impl I2cDevice for PowerDevice {
    fn start(&mut self, read: bool) -> bool { self.device.borrow_mut().start(read) }
    fn write(&mut self, byte: u8) -> bool {
        let ack = self.device.borrow_mut().write(byte);
        self.wiring.sync();
        ack
    }
    fn read(&mut self) -> u8 { self.device.borrow_mut().read() }
    fn stop(&mut self) { self.device.borrow_mut().stop(); }
}

pub struct StopWatch {
    wiring: Rc<PowerWiring>,
    bus: I2cWiring,
    motion: Arc<ImuMotion>,
    imu: Rc<RefCell<Bmi270>>,
    rtc: Rc<RefCell<Rx8130>>,
    pm1_irq_high: bool,
    cycle: VirtualCycle,
    gpio_events: u64,
    vibration: BTreeSet<u16>,
    motor_duty_since: (u16, VirtualCycle),
    external_port_routes: [bool; 2],
    lcd_selected: bool,
}

impl StopWatch {
    pub const WIDTH: usize = 468;
    pub const HEIGHT: usize = 466;
    pub const X_OFFSET: u16 = 6;

    pub fn new() -> Self {
        let us = CPU_HZ / 1_000_000;
        let wiring = Rc::new(PowerWiring {
            panel: Mutex::new(PanelState {
                panel: Co5300::new(Self::WIDTH, Self::HEIGHT, Self::X_OFFSET).with_scan(Co5300Scan {
                    lines: 500, period: 15_000 * us, pulse_width: 120 * us, pin: PIN_STOPWATCH_TE,
                }),
                active: false,
                version: 0,
                power_transitions: 0,
            }),
            power: Pm1Wiring::new(2),
            expander: Default::default(),
            touch: Default::default(),
            codec: Es8311::new(),
            audio_powered: Cell::new(false),
        });
        let motion = Arc::new(ImuMotion::default());
        let imu = Rc::new(RefCell::new(Bmi270::new(motion.clone(), CPU_HZ).with_any_motion()));
        let rtc = Rc::new(RefCell::new(Rx8130::new(CPU_HZ)));
        wiring.power.pmic.borrow_mut().gpio_input(0, true);
        let wrap = |device| Rc::new(RefCell::new(PowerDevice { device, wiring: wiring.clone() }));
        let bus = I2cWiring::new((47, 48), vec![
            (M5pm1::ADDRESS, wrap(wiring.power.pmic.clone())),
            // M5IOE1 1.0.9 selects the StopWatch mapping for revision W.
            (M5ioe1::ADDRESS, wrap(Rc::new(RefCell::new(M5ioe1::new(wiring.expander.clone(), 0).with_revision(b'W'))))),
            (0x15, Rc::new(RefCell::new(Cst820::new(wiring.touch.clone())))),
            (Rx8130::ADDRESS, rtc.clone()),
            (0x68, imu.clone()),
            (Es8311::ADDRESS, Rc::new(RefCell::new(AudioPort { codec: wiring.codec.i2c_port(), wiring: wiring.clone() }))),
        ]);
        Self { wiring, bus, motion, imu, rtc, pm1_irq_high: true,
            cycle: 0, gpio_events: 0, vibration: BTreeSet::new(), motor_duty_since: (0, 0),
            external_port_routes: [false; 2], lcd_selected: false }
    }
}

impl Default for StopWatch { fn default() -> Self { Self::new() } }

impl BoardModel for StopWatch {
    fn name(&self) -> &'static str { "stopwatch" }
    fn gpio_changes(&mut self, changes: &[(u8, bool)]) { self.gpio_events += changes.len() as u64; }
    fn gpio_events(&self) -> u64 { self.gpio_events }
    fn spi_transfer(&mut self, host: u8, tx: &[u8], rx_len: usize) -> Vec<u8> {
        if host == 2 {
            self.wiring.sync();
            let mut state = self.wiring.panel.lock().expect("StopWatch panel mutex poisoned");
            if state.active {
                let before = state.panel.pixels_written;
                state.panel.transaction(tx);
                if state.panel.pixels_written != before || tx.get(..4) == Some(&[0x02, 0, 0x01, 0]) {
                    state.version += 1;
                }
            }
        }
        vec![0xff; rx_len]
    }
    fn observes_gpio_pads(&self) -> bool { true }
    fn gpio_waveform_at(&mut self, cycle: VirtualCycle, routes: PinRoutes<'_>) {
        self.bus.sync(cycle, &routes);
        let selected = routes.software_pad_output(39) == Some(false);
        if selected && !self.lcd_selected { self.wiring.panel.lock().unwrap().panel.select(); }
        self.lcd_selected = selected;
    }
    fn uses_spi_pins(&self) -> bool { true }
    fn spi_transfer_pins(&mut self, host: u8, pins: SpiPins, tx: &[u8], rx_len: usize) -> Vec<u8> {
        if pins.sclk & (1 << 40) == 0 || pins.mosi & (1 << 41) == 0 || pins.cs & (1 << 39) == 0 {
            return vec![0xff; rx_len];
        }
        if !self.lcd_selected { return self.spi_transfer(host, tx, rx_len); }
        self.wiring.sync();
        let mut state = self.wiring.panel.lock().unwrap();
        if host == 2 && state.active {
            let pixels = state.panel.pixels_written;
            state.panel.selected_bytes(tx);
            if state.panel.pixels_written != pixels { state.version += 1; }
        }
        vec![0xff; rx_len]
    }
    fn i2c_devices(&mut self) -> Vec<(u8, u8, Box<dyn I2cDevice>)> { self.bus.ports() }
    fn i2s_codecs(&self) -> Option<I2sCodecs> {
        let expander = self.wiring.expander.lock().unwrap();
        let mut codecs = self.wiring.codec.i2s_codecs(0, 1, 17, 15, 21, 16);
        codecs.mck = Some(18);
        // AU_L3B supplies the codec. The AW8737A uses VBUS_L0 and its own
        // PYG10 shutdown input. Both stages must be active to emit PCM.
        let audio_powered = self.wiring.audio_powered.get();
        if !audio_powered { codecs.mic_lanes = [false; 2]; }
        if !audio_powered || !expander.amp_enabled { codecs.speaker_gain = 0.0; }
        Some(codecs)
    }
    fn display(&self) -> Option<(u32, u32, Vec<u16>, u64)> {
        let state = self.wiring.panel.lock().expect("StopWatch panel mutex poisoned");
        Some((Self::WIDTH as u32, Self::HEIGHT as u32, state.panel.frame.clone(), state.version))
    }
    fn display_version(&self) -> u64 { self.wiring.panel.lock().expect("StopWatch panel mutex poisoned").version }
    fn display_frames(&self) -> u64 { self.wiring.panel.lock().expect("StopWatch panel mutex poisoned").panel.frames }
    fn input_levels(&self) -> Vec<(u8, bool)> {
        let state = self.wiring.panel.lock().expect("StopWatch panel mutex poisoned");
        [state.panel.tear_input().expect("StopWatch scan must be configured"),
            (PIN_STOPWATCH_BUTTON_A, true), (PIN_STOPWATCH_BUTTON_B, true), (12, self.pm1_irq_high)]
            .into_iter().chain(self.touch_irq()).chain(self.bus.levels()).collect()
    }
    fn touch(&mut self, x: u16, y: u16, down: bool) {
        self.wiring.sync();
        if !self.wiring.power.powered()
            || !self.wiring.expander.lock().expect("StopWatch expander state mutex poisoned").touch_reset_high { return; }
        self.wiring.touch.lock().expect("StopWatch touch mutex poisoned")
            .update(x.min(Self::WIDTH as u16 - 1), y.min(Self::HEIGHT as u16 - 1), down);
    }
    fn touch_irq(&self) -> Option<(u8, bool)> { Some((13, !self.wiring.touch.lock().unwrap().down)) }
    fn soc_powered(&self) -> Option<bool> { Some(self.wiring.power.powered()) }
    fn next_deadline(&self) -> Option<VirtualCycle> {
        self.wiring.panel.lock().unwrap().panel.next_deadline().into_iter()
            .chain(self.rtc.borrow().next_irq_check())
            .chain(self.imu.borrow().motion_deadline())
            .chain(self.wiring.power.pmic.borrow().next_deadline()).min()
    }
    fn advance_to(&mut self, cycle: VirtualCycle) {
        self.cycle = cycle;
        self.motion.cycle.store(cycle, std::sync::atomic::Ordering::Relaxed);
        let mut imu = self.imu.borrow_mut();
        imu.advance_motion(cycle);
        let mut rtc = self.rtc.borrow_mut();
        rtc.advance(cycle);
        {
            let mut pmic = self.wiring.power.pmic.borrow_mut();
            pmic.advance(cycle);
            // C152 sheet 4: RX8130 nIRQ shares PMG0 with Q7, which pulls low on high INT1.
            pmic.gpio_input(0, !rtc.irq() && !imu.int1_drive().unwrap_or(false));
        }
        self.wiring.sync();
        let expander = self.wiring.expander.lock().unwrap();
        if self.wiring.power.powered() {
            let duty = expander.vibration_duty;
            if self.motor_duty_since.0 != duty { self.motor_duty_since = (duty, cycle); }
            // Report PWM states that lasted a full period without counting intermediate I2C bytes.
            if duty > 0 && expander.vibration_hz > 0
                && cycle.saturating_sub(self.motor_duty_since.1) >= CPU_HZ / u64::from(expander.vibration_hz) {
                self.vibration.insert(duty);
            }
            // C152 schematic sheet 4: PYG1 selects UART0 or USB on J6 pins 2/3.
            self.external_port_routes[usize::from(expander.external_usb)] = true;
        } else {
            self.motor_duty_since = (0, cycle);
        }
        self.wiring.panel.lock().expect("StopWatch panel mutex poisoned").panel.advance_to(cycle);
    }
    fn take_edges(&mut self) -> Vec<BoardEdge> {
        let mut edges = self.wiring.panel.lock().unwrap().panel.take_edges();
        let high = !self.wiring.power.pmic.borrow().drives_irq(1);
        if high != self.pm1_irq_high {
            self.pm1_irq_high = high;
            edges.push(BoardEdge { cycle: self.cycle, pin: 12, level: high });
        }
        edges.extend(self.bus.edges());
        edges
    }
    fn accepts_command(&self, verb: &str) -> bool { verb == "imu" || Pm1Wiring::accepts(verb) }
    fn board_command(&mut self, _: VirtualCycle, verb: &str, args: &[&str]) -> bool {
        if verb == "imu" { imu_sample(&mut self.imu.borrow_mut(), args) }
        else { self.wiring.power.command(verb, args) }
    }
    fn named_pin(&self, name: &str) -> Option<u8> {
        match name {
            "a" | "btn_a" => Some(PIN_STOPWATCH_BUTTON_A),
            "b" | "btn_b" => Some(PIN_STOPWATCH_BUTTON_B),
            _ => None,
        }
    }
    fn report(&self) -> String {
        let off = !self.wiring.power.powered();
        format!("[stopwatch] power_off_requested={off} timer_wakes={} oled_transitions={} vibration_duties={:?} ext_uart={} ext_usb={} gpio_i2c={}",
            self.wiring.power.pmic.borrow().timer_wakes(),
            self.wiring.panel.lock().unwrap().power_transitions, self.vibration, self.external_port_routes[0],
            self.external_port_routes[1], self.bus.transactions())
    }
}
