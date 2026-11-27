//! Arduino Nesso N1 DK001 (TPX00227 schematic), M5GFX 0.3.0 C6 board data,
//! and the M5Unified Nesso N1 power, GPIO expander, and IMU setup.

use std::{cell::RefCell, net::UdpSocket, rc::Rc, sync::{Arc, atomic::Ordering}};
use esp_periph::i2c::I2cDevice;
use esp_soc::{BoardModel, board::{BoardEdge, SpiPins}, pins::PinRoutes};
use esp_soc::devices::{
    Bmi270, DcsPanel, Ft5x06, Ft5x06Points, BMI270_ADDRESS, FT5X06_ADDRESS,
    aw32001::{self, Aw32001}, bq27220::{self, BatterySample, Bq27220},
    imu_motion::ImuMotion, pi4ioe5v6408::Pi4ioe5v6408,
    soft_i2c::{SoftI2c, SoftOp}, sx1262::Sx1262,
};

const SDA: u8 = 10;
const SCL: u8 = 8;
const SYS_IRQ: u8 = 3;
const LCD_DC: u8 = 16;
const LCD_CS: u64 = 1 << 17;
const SPI_SCK: u64 = 1 << 20;
const SPI_MOSI: u64 = 1 << 21;
const SPI_MISO: u8 = 22;
const LORA_CS_PIN: u8 = 23;
const LORA_CS: u64 = 1 << LORA_CS_PIN;
const LORA_BUSY: u8 = 19;
const LORA_IRQ: u8 = 15;
const CPU_HZ: u64 = crate::periph::CPU_HZ;
const US: u64 = CPU_HZ / 1_000_000;
const W: usize = 135;
const H: usize = 240;

// One device state for each address, shared by the GPIO master and the I2C controller.
type Shared<T> = Rc<RefCell<T>>;
type Device = Shared<dyn I2cDevice>;

struct I2cProxy(Device);

impl I2cDevice for I2cProxy {
    fn pins(&self) -> Option<(u8, u8)> { Some((SDA, SCL)) }
    fn start(&mut self, read: bool) -> bool { self.0.borrow_mut().start(read) }
    fn write(&mut self, byte: u8) -> bool { self.0.borrow_mut().write(byte) }
    fn read(&mut self) -> u8 { self.0.borrow_mut().read() }
    fn stop(&mut self) { self.0.borrow_mut().stop(); }
}

pub struct NessoN1 {
    panel: DcsPanel,
    panel_cmd: u8,
    panel_read: usize,
    panel_resets: u64,
    lcd_ready: bool,
    backlight: bool,
    version: u64,
    soft: SoftI2c,
    i2c: Vec<(u8, Device)>,
    ioe: [Shared<Pi4ioe5v6408>; 2],
    gauge: Shared<Bq27220>,
    charger: Shared<Aw32001>,
    touch: Ft5x06Points,
    imu: Arc<ImuMotion>,
    cycle: u64,
    edges: Vec<BoardEdge>,
    levels: u64,
    leds: [[u8; 3]; 1],
    led_changes: u64,
    grove_power: bool,
    grove_changes: u64,
    received_touches: u64,
    radio: Sx1262,
    radio_ready: bool,
    radio_selected: bool,
    peer: Option<UdpSocket>,
    poll_at: u64,
    tx: u64,
    rx: u64,
    ir_rmt_frames: u64,
    ir: super::ir::NecTrace,
}

impl Default for NessoN1 {
    fn default() -> Self { Self::new() }
}

impl NessoN1 {
    pub fn new() -> Self {
        let touch = Ft5x06Points::default();
        let imu = Arc::new(ImuMotion::default());
        let ioe: [Shared<Pi4ioe5v6408>; 2] = Default::default();
        let gauge = Rc::new(RefCell::new(Bq27220::new(BatterySample {
            millivolts: 3800, milliamps: 96, percent: 70,
        })));
        let charger = Rc::new(RefCell::new(Aw32001::new(true, 3800)));
        let touch_dev = Rc::new(RefCell::new(Ft5x06::new(touch.clone())));
        let imu_dev = Rc::new(RefCell::new(Bmi270::new(imu.clone(), CPU_HZ)));
        let i2c: Vec<(u8, Device)> = vec![
            (0x43, ioe[0].clone()), (0x44, ioe[1].clone()),
            (FT5X06_ADDRESS, touch_dev), (BMI270_ADDRESS, imu_dev),
            (bq27220::ADDRESS, gauge.clone()),
            (aw32001::ADDRESS, charger.clone()),
        ];
        for pin in 0..2 { ioe[0].borrow_mut().set_input(pin, Some(true)); }
        ioe[1].borrow_mut().set_input(5, Some(true)); // VIN_DET
        Self {
            panel: DcsPanel::st7789(), panel_cmd: 0, panel_read: 0, panel_resets: 0,
            lcd_ready: false, backlight: false, version: 0, soft: SoftI2c::idle(),
            i2c, ioe, gauge, charger, touch, imu, cycle: 0, edges: Vec::new(),
            levels: 1 << SYS_IRQ | 1 << SDA | 1 << SCL, leds: [[0; 3]], led_changes: 0,
            grove_power: false, grove_changes: 0, received_touches: 0,
            radio: Sx1262::default(), radio_ready: false, radio_selected: false,
            peer: None, poll_at: 0, tx: 0, rx: 0, ir_rmt_frames: 0, ir: Default::default(),
        }
    }

    pub fn connect_lora(&mut self, local: u16, remote: u16) -> std::io::Result<()> {
        let socket = UdpSocket::bind((std::net::Ipv4Addr::LOCALHOST, local))?;
        socket.connect((std::net::Ipv4Addr::LOCALHOST, remote))?;
        socket.set_nonblocking(true)?;
        self.peer = Some(socket);
        Ok(())
    }

    fn drive(&mut self, pin: u8, level: bool) {
        let bit = 1 << pin;
        if (self.levels & bit != 0) != level {
            self.levels ^= bit;
            self.edges.push(BoardEdge { cycle: self.cycle, pin, level });
        }
    }

    fn sync(&mut self) {
        let (lcd_ready, backlight, led, grove) = {
            let ioe = self.ioe[1].borrow();
            let high = ioe.driven_high();
            (high & 2 != 0, high & (1 << 6) != 0,
             (ioe.driven() & !high) & (1 << 7) != 0, high & 4 != 0)
        };
        if self.lcd_ready && !lcd_ready {
            self.panel.reset();
            self.panel_cmd = 0;
            self.panel_resets += 1;
        }
        if (lcd_ready, backlight) != (self.lcd_ready, self.backlight) { self.version += 1; }
        self.lcd_ready = lcd_ready;
        self.backlight = backlight;
        let color = if led { [0, 0, 255] } else { [0; 3] };
        if self.leds[0] != color {
            self.led_changes += 1;
            self.leds[0] = color;
        }
        if self.grove_power != grove {
            self.grove_changes += 1;
            self.grove_power = grove;
        }
        let (irq, radio_ready) = {
            let ioe = self.ioe[0].borrow();
            (ioe.irq(), ioe.driven_high() & (1 << 7) != 0)
        };
        let touch = self.touch.lock().unwrap()[0].is_some();
        self.drive(SYS_IRQ, !(irq || touch));
        if self.radio_ready != radio_ready {
            self.radio.reset(self.cycle);
            self.radio_ready = radio_ready;
        }
        self.drive(LORA_BUSY, !radio_ready || self.radio.busy(self.cycle));
        self.drive(LORA_IRQ, radio_ready && self.radio.dio1());
    }

    fn poll_radio(&mut self) {
        // Same packet contract as the Cardputer ADV LoRa CAP, with no RF claims.
        let mut header = [1; 11];
        header[1..].copy_from_slice(&self.radio.channel());
        if let Some(payload) = self.radio.advance(self.cycle) {
            self.tx += 1;
            if let Some(peer) = &self.peer {
                let mut packet = header.to_vec();
                packet.extend(payload);
                let _ = peer.send(&packet);
            }
        }
        let Some(peer) = &self.peer else { return };
        if self.cycle < self.poll_at { return; }
        self.poll_at = self.cycle + 2_000 * US;
        let mut data = [0; 267];
        for _ in 0..16 {
            let Ok(n) = peer.recv(&mut data) else { break };
            if self.radio_ready && (11..267).contains(&n) && data[..11] == header
                && self.radio.receive(&data[11..n]) {
                self.rx += 1;
            }
        }
    }
}

impl BoardModel for NessoN1 {
    fn name(&self) -> &'static str { "arduino-nesso-n1" }

    fn input_levels(&self) -> Vec<(u8, bool)> {
        let mut input = vec![(18, false)];
        for pin in [SYS_IRQ, SDA, SCL, LORA_BUSY, LORA_IRQ] {
            input.push((pin, self.levels & (1 << pin) != 0));
        }
        input
    }

    fn i2c_devices(&mut self) -> Vec<(u8, u8, Box<dyn I2cDevice>)> {
        // Bridge both controllers to the same external register state.
        self.i2c.iter().map(|(addr, device)| {
            (0, *addr, Box::new(I2cProxy(device.clone())) as Box<dyn I2cDevice>)
        }).collect()
    }

    fn observes_gpio_pads(&self) -> bool { true }
    fn observes_ledc(&self) -> bool { true }

    fn ledc_output_at(&mut self, cycle: u64, pwm: &dyn Fn(u8) -> Option<(f64, u32)>) {
        if let Some((hz, duty)) = pwm(9) {
            self.ir.drive(cycle, (30_000.0..=60_000.0).contains(&hz) && duty != 0);
        }
    }

    fn gpio_waveform_at(&mut self, cycle: u64, routes: PinRoutes<'_>) {
        self.cycle = cycle;
        if let Some(level) = routes.software_output(9) { self.ir.drive(cycle, level); }
        self.panel.dc = routes.software_output(LCD_DC) == Some(true);
        let selected = routes.software_output(LORA_CS_PIN) == Some(false);
        if self.radio_selected && !selected { self.radio.deselect(cycle, US); }
        self.radio_selected = selected;
        let scl = routes.software_output(SCL) != Some(false);
        let sda = routes.software_output(SDA) != Some(false);
        self.soft.levels(scl, sda, |op| {
            let addr = match op {
                SoftOp::Start(addr, _) | SoftOp::Write(addr, _) | SoftOp::Read(addr)
                    | SoftOp::Stop(addr) => addr,
                SoftOp::Sync => return 0,
            };
            let Some((_, device)) = self.i2c.iter().find(|(a, _)| *a == addr) else { return 0; };
            let mut device = device.borrow_mut();
            match op {
                SoftOp::Start(_, read) => u8::from(device.start(read)),
                SoftOp::Write(_, byte) => u8::from(device.write(byte)),
                SoftOp::Read(_) => device.read(),
                SoftOp::Stop(_) => { device.stop(); 0 }
                SoftOp::Sync => 0,
            }
        });
        self.drive(SDA, sda && !self.soft.pull_low);
        self.drive(SCL, scl);
        self.sync();
    }

    fn advance_to(&mut self, cycle: u64) {
        self.cycle = cycle;
        self.imu.cycle.store(cycle, Ordering::Relaxed);
        self.poll_radio();
        self.sync();
    }

    fn next_deadline(&self) -> Option<u64> {
        self.radio.deadline(self.cycle).into_iter()
            .chain(self.peer.as_ref().map(|_| self.poll_at)).min()
    }

    fn take_edges(&mut self) -> Vec<BoardEdge> { std::mem::take(&mut self.edges) }

    fn uses_spi_pins(&self) -> bool { true }

    fn spi_transfer_pins(&mut self, host: u8, pins: SpiPins, tx: &[u8], rx_len: usize) -> Vec<u8> {
        let mut rx = vec![0xff; rx_len];
        self.sync();
        if host != 2 || pins.sclk & SPI_SCK == 0 { return rx; }
        match pins.cs & (LCD_CS | LORA_CS) {
            LCD_CS if self.lcd_ready => {
                if pins.mosi & SPI_MOSI != 0 {
                    for &byte in tx {
                        if !self.panel.dc { self.panel_cmd = byte; self.panel_read = 0; }
                        self.panel.byte(byte);
                    }
                    if !tx.is_empty() { self.version += 1; }
                }
                if matches!(pins.miso, Some(21 | SPI_MISO)) && self.panel_cmd == 4 {
                    // ST7789V2 RDDID, including the leading turnaround bit on the three-wire bus.
                    let id = [0x81, 0x85, 0x52];
                    for byte in &mut rx {
                        *byte = id.get(self.panel_read).copied().unwrap_or(0);
                        self.panel_read = self.panel_read.saturating_add(1);
                    }
                }
            }
            LORA_CS if self.radio_ready => {
                for index in 0..tx.len().max(rx_len) {
                    let mosi = if pins.mosi & SPI_MOSI != 0 { tx.get(index).copied().unwrap_or(0xff) } else { 0xff };
                    let miso = self.radio.exchange(mosi);
                    if pins.miso == Some(SPI_MISO) {
                        if let Some(byte) = rx.get_mut(index) { *byte = miso; }
                    }
                }
                if !self.radio_selected { self.radio.deselect(self.cycle, US); }
            }
            _ => {}
        }
        rx
    }

    fn touch_at(&mut self, cycle: u64, x: u16, y: u16, down: bool) {
        self.cycle = cycle;
        self.touch.lock().unwrap()[0] = down.then_some((x.min(W as u16 - 1), y.min(H as u16 - 1)));
        self.received_touches += u64::from(down);
        self.sync();
    }

    fn touch(&mut self, x: u16, y: u16, down: bool) { self.touch_at(self.cycle, x, y, down); }

    fn accepts_command(&self, verb: &str) -> bool { matches!(verb, "key" | "battery") }

    fn board_command(&mut self, cycle: u64, verb: &str, args: &[&str]) -> bool {
        self.cycle = cycle;
        match (verb, args) {
            ("key", [name, state]) => {
                let pin = match *name { "A" => 0, "B" => 1, _ => return false };
                let level = match *state { "down" => false, "up" => true, _ => return false };
                self.ioe[0].borrow_mut().set_input(pin, Some(level));
            }
            ("battery", [mv, ma, pct, vin]) => {
                let (Ok(millivolts), Ok(milliamps), Ok(percent @ 0..=100)) =
                    (mv.parse(), ma.parse(), pct.parse()) else { return false };
                let input_present = match *vin { "0" => false, "1" => true, _ => return false };
                self.gauge.borrow_mut().sample = BatterySample { millivolts, milliamps, percent };
                let mut charger = self.charger.borrow_mut();
                charger.cell_mv = millivolts;
                charger.input_present = input_present;
                self.ioe[1].borrow_mut().set_input(5, Some(input_present));
            }
            _ => return false,
        }
        self.sync();
        true
    }

    fn set_imu_motion(&mut self, mode: u32) -> bool {
        self.imu.mode.store(mode, Ordering::Relaxed);
        true
    }

    fn imu_motion(&self) -> u32 { self.imu.mode.load(Ordering::Relaxed) }

    fn display(&self) -> Option<(u32, u32, Vec<u16>, u64)> {
        let mut frame = vec![0; W * H];
        if self.backlight && self.lcd_ready && self.panel.on && !self.panel.sleeping {
            // ST7789V2 135x240 glass starts at RAM column 52, row 40.
            for (y, row) in frame.as_chunks_mut::<W>().0.iter_mut().enumerate() {
                let from = (y + 40) * self.panel.cols + 52;
                row.copy_from_slice(&self.panel.gram[from..from + W]);
            }
        }
        Some((W as u32, H as u32, frame, self.version))
    }

    fn display_version(&self) -> u64 { self.version }
    fn display_frames(&self) -> u64 { self.panel.frames }
    fn leds(&self) -> Option<(&[[u8; 3]], u64)> { Some((&self.leds, self.led_changes)) }

    fn rmt_frame(&mut self, pin: u8, _: &[bool]) {
        if pin == 9 { self.ir_rmt_frames += 1; }
    }

    fn report(&self) -> String {
        format!("[nesso-n1] frames={} pixels={} madctl={:#04x} lit={} lcd_resets={} soft_i2c={} led_changes={} grove_changes={} touches={} lora_tx={} lora_rx={} ir_rmt_frames={} nec_frames={} nec_recent={}",
            self.panel.frames, self.panel.pixels_written, self.panel.madctl, self.backlight && self.lcd_ready,
            self.panel_resets, self.soft.transactions, self.led_changes, self.grove_changes,
            self.received_touches, self.tx, self.rx, self.ir_rmt_frames, self.ir.frames, self.ir.recent())
    }
}
