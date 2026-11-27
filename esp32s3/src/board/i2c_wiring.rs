//! One physical internal bus, reachable by either I²C controller or GPIO.
use std::{cell::RefCell, rc::Rc};

use esp_periph::i2c::I2cDevice;
use esp_soc::board::{BoardEdge, VirtualCycle};
use esp_soc::pins::PinRoutes;
use super::soft_i2c::{SoftI2c, SoftOp};
use super::spi_lcd::SpiLcd;

type Device = Rc<RefCell<dyn I2cDevice>>;

struct Port { pins: (u8, u8), device: Device }
impl I2cDevice for Port {
    fn pins(&self) -> Option<(u8, u8)> { Some(self.pins) }
    fn start(&mut self, read: bool) -> bool { self.device.borrow_mut().start(read) }
    fn write(&mut self, byte: u8) -> bool { self.device.borrow_mut().write(byte) }
    fn read(&mut self) -> u8 { self.device.borrow_mut().read() }
    fn stop(&mut self) { self.device.borrow_mut().stop(); }
}

pub(super) struct I2cWiring {
    pins: (u8, u8),
    devices: Vec<(u8, Device)>,
    soft: SoftI2c,
    drives: [Option<bool>; 2],
    edges: Vec<BoardEdge>,
    released: u64,
}

impl I2cWiring {
    pub fn new(pins: (u8, u8), devices: Vec<(u8, Device)>) -> Self {
        Self { pins, devices, soft: SoftI2c::idle(), drives: [Some(true); 2],
            edges: Vec::new(), released: 0 }
    }
    pub fn sync(&mut self, cycle: VirtualCycle, routes: &PinRoutes<'_>) {
        let levels = [self.pins.0, self.pins.1].map(|pin| routes.software_pad_output(pin));
        let [sda, scl] = levels.map(|level| level != Some(false));
        let mut bus = |op| {
            let address = match op {
                SoftOp::Start(addr, _) | SoftOp::Write(addr, _) | SoftOp::Read(addr) | SoftOp::Stop(addr) => addr,
                SoftOp::Sync => return 0,
            };
            let Some((_, device)) = self.devices.iter().find(|(addr, _)| *addr == address) else {
                return if matches!(op, SoftOp::Read(_)) { 0xff } else { 0 };
            };
            let mut device = device.borrow_mut();
            match op {
                SoftOp::Start(_, read) => device.start(read) as u8,
                SoftOp::Write(_, byte) => device.write(byte) as u8,
                SoftOp::Read(_) => device.read(),
                SoftOp::Stop(_) => { device.stop(); 0 }
                SoftOp::Sync => 0,
            }
        };
        if scl != self.soft.scl { self.soft.levels(scl, self.soft.sda, &mut bus); }
        if sda != self.soft.sda { self.soft.levels(scl, sda, &mut bus); }
        let mut drives = levels.map(|level| level.is_none().then_some(true));
        if self.soft.pull_low { drives[0] = Some(false); }
        for (i, pin) in [self.pins.0, self.pins.1].into_iter().enumerate() {
            SpiLcd::drive(cycle, &mut self.edges, &mut self.released, pin, &mut self.drives[i], drives[i]);
        }
    }
    pub fn ports(&self) -> Vec<(u8, u8, Box<dyn I2cDevice>)> {
        (0..2).flat_map(|bus| self.devices.iter().map(move |(address, device)| {
            (bus, *address, Box::new(Port { pins: self.pins, device: device.clone() }) as Box<dyn I2cDevice>)
        })).collect()
    }
    pub fn levels(&self) -> impl Iterator<Item = (u8, bool)> {
        [self.pins.0, self.pins.1].into_iter().zip(self.drives).filter_map(|(pin, value)| value.map(|level| (pin, level)))
    }
    pub fn edges(&mut self) -> Vec<BoardEdge> { std::mem::take(&mut self.edges) }
    pub fn released(&mut self) -> impl Iterator<Item = u8> {
        let released = std::mem::take(&mut self.released);
        [self.pins.0, self.pins.1].into_iter().filter(move |&pin| released & (1 << pin) != 0)
    }
}
