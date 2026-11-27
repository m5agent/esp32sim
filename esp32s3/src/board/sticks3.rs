//! StickS3 K150: M5GFX 0.3.0 display wiring, M5Unified 0.3.0 audio and keys.
use std::{cell::RefCell, rc::Rc, sync::Arc};
use esp_periph::i2c::{m5::M5pm1, I2cDevice};
use esp_soc::board::{BoardEdge, BoardModel, I2sCodecs, SpiPins, VirtualCycle};
use esp_soc::devices::{Bmi270, DcsPanel, es8311::Es8311, imu_motion::ImuMotion};
use esp_soc::pins::PinRoutes;
use super::{i2c_wiring::I2cWiring, pm1_wiring::{imu_sample, Pm1Wiring}, spi_lcd::{SpiLcd, LcdPins}};
use crate::periph::CPU_HZ;

const LCD: LcdPins = LcdPins { clk: 40, data: 39, dc: 45, cs: 41, rst: 21 };
const IR_RX: u8 = 42;
const PM1_IRQ: u8 = 13;

pub struct StickS3 {
    lcd: SpiLcd,
    bus: I2cWiring,
    power: Pm1Wiring,
    codec: Es8311,
    motion: Arc<ImuMotion>,
    imu: Rc<RefCell<Bmi270>>,
    lcd_powered: bool,
    irq_high: bool,
    cycle: VirtualCycle,
    ir_edges: Vec<BoardEdge>,
    edges: Vec<BoardEdge>,
    ir_frames: u64,
    ir_active_low_frames: u64,
    last_nec: Option<(u16, u8)>,
}

impl Default for StickS3 { fn default() -> Self { Self::new() } }

impl StickS3 {
    pub fn new() -> Self {
        let power = Pm1Wiring::new(0);
        power.pmic.borrow_mut().gpio_input(4, true);
        let codec = Es8311::new();
        let motion = Arc::new(ImuMotion::default());
        let imu = Rc::new(RefCell::new(Bmi270::new(motion.clone(), CPU_HZ).with_any_motion()));
        let bus = I2cWiring::new((47, 48), vec![
            (M5pm1::ADDRESS, power.pmic.clone()),
            (0x68, imu.clone()),
            (Es8311::ADDRESS, Rc::new(RefCell::new(codec.i2c_port()))),
        ]);
        Self { lcd: SpiLcd::new(LCD, DcsPanel::st7789(), [0x85, 0x85, 0x52]), bus,
            power, codec, motion, imu, lcd_powered: false, irq_high: true, cycle: 0,
            ir_edges: Vec::new(), edges: Vec::new(), ir_frames: 0, ir_active_low_frames: 0, last_nec: None }
    }
    fn sync_lcd(&mut self) {
        let enabled = self.power.powered() && self.power.pmic.borrow().driven_high() & 4 != 0;
        if self.lcd_powered && !enabled { self.lcd.power_loss(); }
        self.lcd_powered = enabled;
    }
    fn inject_nec(&mut self, cycle: VirtualCycle, address: u16, command: u8) -> bool {
        if !self.ir_edges.is_empty() { return false; }
        let [low, high] = address.to_le_bytes();
        let raw = u32::from_le_bytes([low, if high == 0 { !low } else { high }, command, !command]);
        let mut next = cycle.saturating_add(1);
        let mut emit = |level, duration| {
            self.ir_edges.push(BoardEdge { cycle: next, pin: IR_RX, level });
            next = next.saturating_add(duration * (CPU_HZ / 1_000_000));
        };
        emit(false, 9000); emit(true, 4500);
        for n in 0..32 { emit(false, 560); emit(true, if raw & (1 << n) != 0 { 1690 } else { 560 }); }
        emit(false, 560); emit(true, 0);
        true
    }
}

impl BoardModel for StickS3 {
    fn name(&self) -> &'static str { "m5stack-sticks3" }
    fn observes_gpio_pads(&self) -> bool { true }
    fn gpio_waveform_at(&mut self, cycle: VirtualCycle, routes: PinRoutes<'_>) {
        self.bus.sync(cycle, &routes);
        self.sync_lcd();
        self.lcd.routes(cycle, &routes);
    }
    fn observes_ledc(&self) -> bool { true }
    fn ledc_output_at(&mut self, _: VirtualCycle, pwm: &dyn Fn(u8) -> Option<(f64, u32)>) {
        self.lcd.set_backlight(pwm(38).is_some_and(|(_, duty)| duty != 0));
    }
    fn uses_spi_pins(&self) -> bool { true }
    fn spi_transfer_pins(&mut self, host: u8, pins: SpiPins, tx: &[u8], rx_len: usize) -> Vec<u8> {
        self.sync_lcd();
        if self.lcd_powered { self.lcd.transfer(host, pins, tx, rx_len) } else { vec![0xff; rx_len] }
    }
    fn i2c_devices(&mut self) -> Vec<(u8, u8, Box<dyn I2cDevice>)> { self.bus.ports() }
    fn i2s_codecs(&self) -> Option<I2sCodecs> {
        let mut codec = self.codec.i2s_codecs(0, 1, 17, 15, 14, 16);
        codec.mck = Some(18);
        if !self.power.powered() || self.power.pmic.borrow().driven_high() & 8 == 0 { codec.speaker_gain = 0.0; }
        Some(codec)
    }
    fn input_levels(&self) -> Vec<(u8, bool)> {
        [(11, true), (12, true), (PM1_IRQ, self.irq_high), (IR_RX, true)].into_iter().chain(self.bus.levels()).collect()
    }
    fn named_pin(&self, name: &str) -> Option<u8> {
        match name { "a" | "btn_a" | "btnA" => Some(11), "b" | "btn_b" | "btnB" => Some(12), _ => None }
    }
    fn soc_powered(&self) -> Option<bool> { Some(self.power.powered()) }
    fn next_deadline(&self) -> Option<VirtualCycle> {
        self.power.pmic.borrow().next_deadline().into_iter().chain(self.imu.borrow().motion_deadline())
            .chain(self.ir_edges.first().map(|edge| edge.cycle)).min()
    }
    fn advance_to(&mut self, cycle: VirtualCycle) {
        self.cycle = cycle;
        self.motion.cycle.store(cycle, std::sync::atomic::Ordering::Relaxed);
        let mut imu = self.imu.borrow_mut();
        imu.advance_motion(cycle);
        let mut pmic = self.power.pmic.borrow_mut();
        pmic.advance(cycle);
        pmic.gpio_input(4, imu.int1_drive().unwrap_or(true));
        drop(pmic);
        drop(imu);
        self.sync_lcd();
        let due = self.ir_edges.partition_point(|edge| edge.cycle <= cycle);
        self.edges.extend(self.ir_edges.drain(..due));
    }
    fn rmt_symbols(&mut self, pin: u8, hz: u32, symbols: &[u32]) {
        if pin != 46 { return; }
        if let Some((frame, active_low)) = decode_nec(hz, symbols) {
            self.ir_frames += 1;
            self.ir_active_low_frames += u64::from(active_low);
            self.last_nec = Some(frame);
        }
    }
    fn take_edges(&mut self) -> Vec<BoardEdge> {
        let high = !self.power.pmic.borrow().drives_irq(1);
        if self.irq_high != high {
            self.irq_high = high;
            self.edges.push(BoardEdge { cycle: self.cycle, pin: PM1_IRQ, level: high });
        }
        self.edges.extend(self.lcd.edges());
        self.edges.extend(self.bus.edges());
        std::mem::take(&mut self.edges)
    }
    fn released_inputs(&mut self) -> Vec<u8> { self.lcd.released().into_iter().collect() }
    fn accepts_command(&self, verb: &str) -> bool { matches!(verb, "imu" | "ir-nec") || Pm1Wiring::accepts(verb) }
    fn board_command(&mut self, cycle: VirtualCycle, verb: &str, args: &[&str]) -> bool {
        match (verb, args) {
            ("imu", _) => imu_sample(&mut self.imu.borrow_mut(), args),
            ("ir-nec", [address, command]) => {
                let (Ok(address), Ok(command)) = (u16::from_str_radix(address, 16), u8::from_str_radix(command, 16)) else { return false; };
                self.inject_nec(cycle, address, command)
            }
            _ => self.power.command(verb, args),
        }
    }
    fn display(&self) -> Option<(u32, u32, Vec<u16>, u64)> {
        let frame = if self.lcd.visible() && self.lcd_powered {
            (40..280).flat_map(|y| self.lcd.lcd.gram[y * 240 + 52..y * 240 + 187].iter()
                .map(|&pixel| if self.lcd.lcd.inverted { pixel } else { !pixel })).collect()
        } else { vec![0; 135 * 240] };
        Some((135, 240, frame, self.display_version()))
    }
    fn display_version(&self) -> u64 { self.lcd.version() }
    fn display_frames(&self) -> u64 { self.lcd.lcd.frames }
    fn service_state(&self) -> Vec<(&'static str, esp_soc::json::Json)> {
        vec![("pmic", self.power.service_state()), ("imu", self.imu.borrow().service_state())]
    }
    fn report(&self) -> String {
        format!("[sticks3] RAMWR={} pixels={} lcd_rail={} powered={} timer_wakes={} RMT46_frames={} RMT46_active_low={} last_nec={:?} gpio_i2c={}",
            self.display_frames(), self.lcd.lcd.pixels_written, self.lcd_powered, self.power.powered(),
            self.power.pmic.borrow().timer_wakes(), self.ir_frames, self.ir_active_low_frames, self.last_nec, self.bus.transactions())
    }
}

/// Decode the envelope seen by the IR LED. Carrier phase and optical travel are omitted.
fn decode_nec(hz: u32, symbols: &[u32]) -> Option<((u16, u8), bool)> {
    let [header, data @ .., trailer] = symbols else { return None };
    if data.len() != 32 { return None; }
    let durations = |word: u32| {
        let micros = |ticks| u64::from(ticks) * 1_000_000 / u64::from(hz);
        (micros(word & 0x7fff), micros(word >> 16 & 0x7fff))
    };
    // The raw echo example sends the demodulator's active-low samples verbatim.
    // Preserve their polarity in the report, while reading the durations of both forms.
    let levels = *header & 0x8000_8000;
    if levels != 0x0000_8000 && levels != 0x8000_0000 { return None; }
    let envelope = |word: u32| word & 0x8000_8000 == levels;
    let (mark, space) = durations(*header);
    if !(8_000..=10_000).contains(&mark) || !(4_000..=5_000).contains(&space)
        || *trailer & 0x8000 != levels & 0x8000 || !(350..=800).contains(&durations(*trailer).0) { return None; }
    let mut raw = 0u32;
    for (bit, &word) in data.iter().enumerate() {
        let (mark, space) = durations(word);
        if !envelope(word) || !(350..=800).contains(&mark) { return None; }
        match space {
            350..=800 => {}
            1350..=2000 => raw |= 1 << bit,
            _ => return None,
        }
    }
    let [lo, hi, command, complement] = raw.to_le_bytes();
    if complement != !command { return None; }
    let address = if hi == !lo { u16::from(lo) } else { u16::from_le_bytes([lo, hi]) };
    Some(((address, command), levels == 0x8000_0000))
}
