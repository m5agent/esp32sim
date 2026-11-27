//! PaperDIY K159-B and PaperS3 C139. Wiring from the schematics and the
//! M5GFX 28d47949 board descriptions. Display source: M5GFX Bus_EPD.inl.

use std::{cell::{Cell, RefCell}, rc::Rc, sync::{Arc, Mutex}};
use esp_periph::{i2c::{I2cDevice, m5::{M5pm1, M5pm1State}}, sdmmc::{SdCard, SpiSdCard}};
use esp_soc::{
    board::{BoardEdge, BoardModel, LcdI80Pins, SpiPins, VirtualCycle},
    devices::{bm8563::Bm8563, bmi270::Bmi270, et047tc1::Et047tc1,
              gt911::{Gt911, TouchState}, imu_motion::ImuMotion},
};
use super::paper_i2c::PaperI2c;
use crate::periph::CPU_HZ;

const DATA: [u8; 8] = [6, 14, 7, 12, 9, 11, 8, 10];
const BOOST: u64 = 1 << 46;
const OE: u64 = 1 << 45;
const SPV: u64 = 1 << 17;
const CKV: u64 = 1 << 18;
const LE: u64 = 1 << 15;
const SD: SpiPins = SpiPins { sclk: 1 << 39, mosi: 1 << 38, miso: Some(40), cs: 1 << 47 };

struct Clock(Rc<RefCell<Bm8563>>);
impl I2cDevice for Clock {
    fn start(&mut self, read: bool) -> bool { self.0.borrow_mut().start(read) }
    fn write(&mut self, byte: u8) -> bool { self.0.borrow_mut().write(byte) }
    fn read(&mut self) -> u8 { self.0.borrow_mut().read() }
}

pub struct PaperParallel {
    diy: bool,
    i2c: PaperI2c,
    panel: Et047tc1,
    pmic: Arc<Mutex<M5pm1State>>,
    touch: Arc<Mutex<TouchState>>,
    touch_pending: Rc<Cell<bool>>,
    touch_high: bool,
    rtc: Rc<RefCell<Bm8563>>,
    imu: Arc<ImuMotion>,
    sd: Option<Box<SpiSdCard>>,
    cycle: VirtualCycle,
    output: u64,
    sd_selected: bool,
    panel_on: bool,
    edges: Vec<BoardEdge>,
}

impl PaperParallel {
    pub fn new(diy: bool) -> Self {
        Self {
            diy, i2c: PaperI2c::new((41, 42)), panel: Default::default(),
            pmic: Default::default(), touch: Default::default(),
            touch_pending: Default::default(), touch_high: true,
            rtc: Rc::new(RefCell::new(Bm8563::new(CPU_HZ))),
            imu: Default::default(), sd: None, cycle: 0, output: 0,
            sd_selected: false, panel_on: false, edges: Vec::new(),
        }
    }

    fn sync_panel_power(&mut self) {
        let power = if self.diy {
            let pmic = self.pmic.lock().unwrap();
            !pmic.power_off_requested && pmic.driven_high & 4 != 0
        } else {
            self.output & OE != 0
        };
        let on = power && self.output & BOOST != 0;
        if self.panel_on && !on { self.panel.power_off(); }
        self.panel_on = on;
    }

    fn sync_touch_irq(&mut self) {
        let high = !self.touch_pending.get();
        if high != self.touch_high {
            self.touch_high = high;
            self.edges.push(BoardEdge { cycle: self.cycle, pin: 48, level: high });
        }
    }
}

impl BoardModel for PaperParallel {
    fn name(&self) -> &'static str { if self.diy { "m5stack-paperdiy" } else { "m5stack-papers3" } }

    fn i2c_devices(&mut self) -> Vec<(u8, u8, Box<dyn I2cDevice>)> {
        self.i2c.reset();
        if self.diy {
            self.output = 0;
            self.panel_on = false;
            self.i2c.attach(M5pm1::ADDRESS, M5pm1::with_power_timer(self.pmic.clone(), CPU_HZ));
        } else {
            self.i2c.attach(0x51, Clock(self.rtc.clone()));
            self.i2c.attach(0x5d, Gt911::new(self.touch.clone(), 540, 960).with_pending(self.touch_pending.clone()));
            self.i2c.attach(0x68, Bmi270::new(self.imu.clone(), CPU_HZ));
        }
        self.i2c.controller_devices()
    }

    fn gpio_output_at(&mut self, cycle: VirtualCycle, _changes: &[(u8, bool)], enabled: u64, output: u64) {
        self.cycle = cycle;
        let previous = std::mem::replace(&mut self.output, enabled & output);
        self.i2c.drive(cycle, enabled, output, &mut self.edges);
        let selected = enabled & !output & SD.cs != 0;
        if self.sd_selected && !selected {
            if let Some(card) = &mut self.sd { card.deselect(); }
        }
        self.sd_selected = selected;
        self.sync_panel_power();
        if self.panel_on && previous & SPV != 0 && self.output & SPV == 0 {
            self.panel.start_scan();
        }
        if self.panel_on && previous & LE == 0 && self.output & LE != 0 {
            self.panel.latch();
        }
    }

    fn lcd_i80_data(&mut self, pins: LcdI80Pins, bytes: &[u8]) {
        if self.panel_on && self.output & (LE | CKV) == CKV && pins.width == 8
            && pins.write & (1 << 16) != 0 && pins.select & (1 << 13) != 0
            && DATA.iter().enumerate().all(|(signal, pin)| pins.data[signal] & (1 << pin) != 0)
        {
            self.panel.shift(bytes);
        }
    }

    fn uses_spi_pins(&self) -> bool { true }
    fn has_spi_sd_slot(&self) -> bool { true }
    fn insert_spi_sd(&mut self, card: SdCard) {
        self.sd = Some(Box::new(SpiSdCard::new(card)));
        if self.diy { self.edges.push(BoardEdge { cycle: self.cycle, pin: 48, level: false }); }
    }
    fn spi_transfer_pins(&mut self, _host: u8, pins: SpiPins, tx: &[u8], rx_len: usize) -> Vec<u8> {
        let now_ns = (u128::from(self.cycle) * 1_000_000_000 / u128::from(CPU_HZ)) as u64;
        self.sd.as_mut().and_then(|card| pins.transfer_sd(SD, card, tx, rx_len, now_ns))
            .unwrap_or_else(|| vec![0xff; rx_len])
    }

    fn display(&self) -> Option<(u32, u32, Vec<u16>, u64)> {
        Some((Et047tc1::WIDTH, Et047tc1::HEIGHT, self.panel.frame.clone(), self.panel.version))
    }
    fn display_version(&self) -> u64 { self.panel.version }
    fn display_frames(&self) -> u64 { self.panel.frames }

    fn input_levels(&self) -> Vec<(u8, bool)> {
        let mut levels = self.i2c.levels().to_vec();
        if self.diy { levels.extend([(3, true), (4, true), (48, self.sd.is_none())]); }
        else { levels.extend([(4, true), (48, self.touch_high)]); }
        levels
    }
    fn default_analog_inputs(&self) -> &'static [(u8, f32)] {
        if self.diy { &[] } else { &[(3, 2.0)] }
    }
    fn named_pin(&self, name: &str) -> Option<u8> {
        match (self.diy, name) {
            (true, "a" | "btn_a" | "button-a") => Some(4),
            (true, "b" | "btn_b" | "button-b") => Some(3),
            _ => None,
        }
    }
    fn touch(&mut self, x: u16, y: u16, down: bool) {
        if !self.diy {
            self.touch.lock().unwrap().update(x.min(539), y.min(959), down);
            self.touch_pending.set(true);
            self.sync_touch_irq();
        }
    }
    fn touch_irq(&self) -> Option<(u8, bool)> { (!self.diy).then_some((48, self.touch_high)) }
    fn set_imu_motion(&mut self, mode: u32) -> bool {
        if self.diy { return false; }
        self.imu.mode.store(mode, std::sync::atomic::Ordering::Relaxed);
        true
    }
    fn take_edges(&mut self) -> Vec<BoardEdge> { std::mem::take(&mut self.edges) }
    fn soc_powered(&self) -> Option<bool> {
        self.diy.then(|| !self.pmic.lock().unwrap().power_off_requested)
    }
    fn next_deadline(&self) -> Option<VirtualCycle> {
        if !self.diy { return None; }
        self.pmic.lock().unwrap().power_deadline()
    }
    fn advance_to(&mut self, cycle: VirtualCycle) {
        self.cycle = cycle;
        if self.diy {
            self.pmic.lock().unwrap().advance_power_timer(cycle);
            self.sync_panel_power();
        } else {
            self.imu.cycle.store(cycle, std::sync::atomic::Ordering::Relaxed);
            self.rtc.borrow_mut().advance_to(cycle);
            self.sync_touch_irq();
        }
    }
    fn report(&self) -> String {
        format!("[{}] ET047TC1 scans={} refreshes={} powered={} soft-i2c={}",
            self.name(), self.panel.scans, self.panel.frames, self.panel_on, self.i2c.transactions())
    }
}
