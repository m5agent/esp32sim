//! Share a physical I2C bus between GPIO probing and either S3 I2C controller.

use std::{cell::RefCell, rc::Rc};
use esp_periph::i2c::I2cDevice;
use esp_soc::board::{BoardEdge, VirtualCycle};
use esp_soc::devices::soft_i2c::{SoftI2c, SoftOp};

struct Device {
    pins: (u8, u8),
    inner: Rc<RefCell<Box<dyn I2cDevice>>>,
}

impl I2cDevice for Device {
    fn pins(&self) -> Option<(u8, u8)> { Some(self.pins) }
    fn start(&mut self, read: bool) -> bool { self.inner.borrow_mut().start(read) }
    fn write(&mut self, value: u8) -> bool { self.inner.borrow_mut().write(value) }
    fn read(&mut self) -> u8 { self.inner.borrow_mut().read() }
    fn stop(&mut self) { self.inner.borrow_mut().stop(); }
}

type I2cAttachment = (u8, u8, Box<dyn I2cDevice>);

pub(super) struct PaperI2c {
    pins: (u8, u8),
    devices: Vec<(u8, Device)>,
    soft: SoftI2c,
    levels: [bool; 2],
}

impl PaperI2c {
    pub fn new(pins: (u8, u8)) -> Self {
        Self { pins, devices: Vec::new(), soft: SoftI2c::idle(), levels: [true; 2] }
    }

    pub fn reset(&mut self) {
        self.devices.clear();
        self.soft = SoftI2c::idle();
        self.levels = [true; 2];
    }

    pub fn attach(&mut self, address: u8, device: impl I2cDevice + 'static) {
        self.devices.push((address, Device { pins: self.pins, inner: Rc::new(RefCell::new(Box::new(device))) }));
    }

    pub fn controller_devices(&self) -> Vec<I2cAttachment> {
        (0..2).flat_map(|bus| self.devices.iter().map(move |(address, device)| {
            let device: Box<dyn I2cDevice> = Box::new(Device { pins: self.pins, inner: device.inner.clone() });
            (bus, *address, device)
        })).collect()
    }

    pub fn levels(&self) -> [(u8, bool); 2] {
        [(self.pins.0, self.levels[0]), (self.pins.1, self.levels[1])]
    }

    pub fn drive(&mut self, cycle: VirtualCycle, enabled: u64, output: u64, edges: &mut Vec<BoardEdge>) {
        let released = |pin| enabled & (1u64 << pin) == 0 || output & (1u64 << pin) != 0;
        let (sda, scl) = (released(self.pins.0), released(self.pins.1));
        if self.soft.sda == sda && self.soft.scl == scl { return; }
        if scl != self.soft.scl { self.step(scl, self.soft.sda); }
        if sda != self.soft.sda { self.step(scl, sda); }
        let levels = [sda && !self.soft.pull_low, scl];
        for (index, (pin, _)) in self.levels().into_iter().enumerate() {
            if self.levels[index] != levels[index] {
                edges.push(BoardEdge { cycle, pin, level: levels[index] });
            }
        }
        self.levels = levels;
    }

    fn step(&mut self, scl: bool, sda: bool) {
        self.soft.levels(scl, sda, |op| {
            let address = match op {
                SoftOp::Start(a, _) | SoftOp::Write(a, _) | SoftOp::Read(a) | SoftOp::Stop(a) => a,
                SoftOp::Sync => return 0,
            };
            let device = self.devices.iter_mut().find(|(a, _)| *a == address).map(|(_, d)| d);
            let Some(device) = device else { return if matches!(op, SoftOp::Read(_)) { 0xff } else { 0 }; };
            match op {
                SoftOp::Start(_, read) => u8::from(device.start(read)),
                SoftOp::Write(_, byte) => u8::from(device.write(byte)),
                SoftOp::Read(_) => device.read(),
                SoftOp::Stop(_) => { device.stop(); 0 }
                SoftOp::Sync => 0,
            }
        });
    }

    pub fn transactions(&self) -> u64 { self.soft.transactions }
}
