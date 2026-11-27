//! Original Cardputer K132. The 74HC138 is wired as in M5Cardputer 1.1.1 IOMatrix.
use esp_periph::sdmmc::{SdCard, SpiSdCard};
use esp_soc::board::{BoardEdge, BoardModel, I2sCodecs, SpiPins, VirtualCycle};
use esp_soc::devices::ws2812::Ws2812Chain;
use esp_soc::pins::PinRoutes;

use super::cardputer_adv_ir::NecReceiver;
use super::cardputer_keys;
use super::spi_lcd::{cardputer_lcd, SpiLcd};
use crate::periph::CPU_HZ;

const ROW_SELECT: [u8; 3] = [8, 9, 11];
const COLUMNS: [u8; 7] = [13, 15, 3, 4, 5, 6, 7];
const SD: SpiPins = SpiPins { sclk: 1 << 40, mosi: 1 << 14, miso: Some(39), cs: 1 << 12 };
const IR: u8 = 44;
const RGB: u8 = 21;

#[derive(Default)]
struct Keyboard {
    keys: [[bool; 14]; 4],
    selector: Option<u8>,
    drives: [Option<bool>; 7],
    releases: [[Option<VirtualCycle>; 14]; 4],
}

impl Keyboard {
    fn level(&self, column: usize) -> Option<bool> {
        let select = self.selector?;
        // Even switches are wired to Y4..Y7; odd switches use Y0..Y3.
        // A selected low dominates simultaneous drives on the same input.
        let mut level = None;
        for (row, keys) in self.keys.iter().enumerate() {
            for parity in 0..2 {
                if keys[2 * column + parity] {
                    let output = (3 - row + (1 - parity) * 4) as u8;
                    level = Some(level.unwrap_or(true) && output != select);
                }
            }
        }
        level
    }
}

pub struct Cardputer {
    display: SpiLcd,
    keyboard: Keyboard,
    rgb: Ws2812Chain,
    sd: Option<Box<SpiSdCard>>,
    sd_selected: bool,
    ir: NecReceiver,
    ir_pwm: bool,
    ir_drive: Option<bool>,
    cycle: VirtualCycle,
    edges: Vec<BoardEdge>,
    released: u64,
    battery_switch: bool,
    usb_power: bool,
}

impl Cardputer {
    pub fn new() -> Self {
        Self { display: cardputer_lcd(), keyboard: Keyboard::default(),
            rgb: Ws2812Chain::new(1), sd: None, sd_selected: false, ir: Default::default(),
            ir_pwm: false, ir_drive: Some(false), cycle: 0, edges: Vec::new(), released: 0,
            battery_switch: true, usb_power: false }
    }

    fn scan(&mut self) {
        for (index, &pin) in COLUMNS.iter().enumerate() {
            let level = self.keyboard.level(index);
            SpiLcd::drive(self.cycle, &mut self.edges, &mut self.released, pin,
                &mut self.keyboard.drives[index], level);
        }
    }
}

impl Default for Cardputer { fn default() -> Self { Self::new() } }

impl BoardModel for Cardputer {
    fn name(&self) -> &'static str { "m5stack-cardputer" }
    fn named_pin(&self, name: &str) -> Option<u8> {
        matches!(name, "a" | "btn" | "boot" | "g0").then_some(0)
    }
    fn input_levels(&self) -> Vec<(u8, bool)> {
        [(0, true)].into_iter().chain(self.ir_drive.map(|level| (IR, level)))
            .chain(COLUMNS.into_iter().zip(self.keyboard.drives).filter_map(|(pin, level)| level.map(|level| (pin, level))))
            .collect()
    }
    fn observes_gpio_pads(&self) -> bool { true }
    fn gpio_waveform_at(&mut self, cycle: VirtualCycle, routes: PinRoutes<'_>) {
        self.cycle = cycle;
        self.display.routes(cycle, &routes);
        let [a, b, c] = ROW_SELECT.map(|pin| routes.software_pad_output(pin));
        let selected = match (a, b, c) {
            (Some(a), Some(b), Some(c)) => Some(u8::from(a) | u8::from(b) << 1 | u8::from(c) << 2),
            _ => None,
        };
        if selected != self.keyboard.selector {
            self.keyboard.selector = selected;
            self.scan();
        }

        if let Some(sd) = &mut self.sd {
            let selected = routes.software_pad_output(12) == Some(false);
            if self.sd_selected && !selected { sd.deselect(); }
            self.sd_selected = selected;
        }

        // K132 schematic: the IR LED to ground pulls an undriven GPIO44 low.
        let ir = routes.software_pad_output(IR);
        SpiLcd::drive(cycle, &mut self.edges, &mut self.released, IR,
            &mut self.ir_drive, ir.is_none().then_some(false));
        if !self.ir_pwm { self.ir.carrier(cycle, ir == Some(true)); }
    }
    fn observes_ledc(&self) -> bool { true }
    fn ledc_output_at(&mut self, cycle: VirtualCycle, pwm: &dyn Fn(u8) -> Option<(f64, u32)>) {
        self.display.set_backlight(pwm(38).is_some_and(|(_, duty)| duty != 0));
        let ir = pwm(IR);
        if ir.is_some() || self.ir_pwm {
            self.ir.carrier(cycle, ir.is_some_and(|(hz, duty)| (30_000.0..=45_000.0).contains(&hz) && (1..65535).contains(&duty)));
        }
        self.ir_pwm = ir.is_some();
    }
    fn rmt_frame(&mut self, pin: u8, bits: &[bool]) {
        if pin == RGB { self.rgb.from_bits(bits); }
    }
    fn leds(&self) -> Option<(&[[u8; 3]], u64)> { Some((&self.rgb.leds, self.rgb.updates)) }
    fn uses_spi_pins(&self) -> bool { true }
    fn has_spi_sd_slot(&self) -> bool { true }
    fn insert_spi_sd(&mut self, card: SdCard) { self.sd = Some(Box::new(SpiSdCard::new(card))); }
    fn spi_transfer_pins(&mut self, host: u8, pins: SpiPins, tx: &[u8], rx_len: usize) -> Vec<u8> {
        if let Some(sd) = &mut self.sd {
            let ns = (u128::from(self.cycle) * 1_000_000_000 / u128::from(CPU_HZ)) as u64;
            if let Some(rx) = pins.transfer_sd(SD, sd, tx, rx_len, ns) { return rx; }
        }
        self.display.transfer(host, pins, tx, rx_len)
    }
    fn default_analog_inputs(&self) -> &'static [(u8, f32)] { &[(10, 1.85)] }
    fn i2s_codecs(&self) -> Option<I2sCodecs> {
        // M5Unified 0.3.0: SPM1423 on I2S0 PDM; NS4168 on I2S1 PCM.
        Some(I2sCodecs { port: 1, mic_port: 0, bck: 41, ws: 43, mck: None,
            dout: 42, din: 46, speaker_gain: 1.0, mic_lanes: [true; 2] })
    }
    fn pdm_mic(&self) -> Option<(u8, u8)> { Some((43, 46)) }
    fn next_deadline(&self) -> Option<VirtualCycle> {
        self.keyboard.releases.iter().flatten().flatten().copied().min()
    }
    fn advance_to(&mut self, cycle: VirtualCycle) {
        self.cycle = cycle;
        let mut changed = false;
        for (keys, releases) in self.keyboard.keys.iter_mut().zip(&mut self.keyboard.releases) {
            for (key, due) in keys.iter_mut().zip(releases) {
                if due.is_some_and(|due| due <= cycle) { *key = false; *due = None; changed = true; }
            }
        }
        if changed { self.scan(); }
    }
    fn soc_powered(&self) -> Option<bool> { Some(self.battery_switch || self.usb_power) }
    fn accepts_command(&self, verb: &str) -> bool {
        matches!(verb, "key" | "key-edge" | "power-switch" | "vbus")
    }
    fn board_command(&mut self, cycle: VirtualCycle, verb: &str, args: &[&str]) -> bool {
        if matches!(verb, "power-switch" | "vbus") {
            let [value @ ("0" | "1")] = args else { return false };
            let powered = self.soc_powered();
            if verb == "power-switch" { self.battery_switch = *value == "1"; }
            else { self.usb_power = *value == "1"; }
            if powered == Some(true) && self.soc_powered() == Some(false) {
                self.display.power_loss();
                self.display.set_backlight(false);
                if let Some(sd) = &mut self.sd { sd.deselect(); }
            }
            return true;
        }
        let (name, down, release) = match (verb, args) {
            ("key-edge", [name, "0"]) => (*name, false, None),
            ("key-edge", [name, "1"]) => (*name, true, None),
            ("key", [name, remainder @ ..]) if remainder.len() <= 1 => {
                let ms = match remainder.first().copied().unwrap_or("100").parse::<u32>() {
                    Ok(ms) => ms.max(1), Err(_) => return false,
                };
                (*name, true, Some(cycle.saturating_add(u64::from(ms) * (CPU_HZ / 1000))))
            }
            _ => return false,
        };
        let Some((row, col)) = cardputer_keys::position(name) else { return false };
        self.cycle = cycle;
        self.keyboard.keys[row as usize][col as usize] = down;
        self.keyboard.releases[row as usize][col as usize] = release;
        self.scan();
        true
    }
    fn take_edges(&mut self) -> Vec<BoardEdge> {
        self.edges.extend(self.display.edges());
        std::mem::take(&mut self.edges)
    }
    fn released_inputs(&mut self) -> Vec<u8> {
        let released = std::mem::take(&mut self.released);
        COLUMNS.into_iter().chain([IR]).filter(|pin| released & (1 << pin) != 0)
            .chain(self.display.released()).collect()
    }
    fn display(&self) -> Option<(u32, u32, Vec<u16>, u64)> {
        Some((240, 135, self.display.cardputer_pixels(), self.display_version()))
    }
    fn display_version(&self) -> u64 { self.display.version() }
    fn display_rotation(&self) -> i8 {
        if !self.display.lcd.on || self.display.lcd.madctl & 0x20 != 0 { 0 }
        else if self.display.lcd.madctl & 0x40 != 0 { 90 }
        else { -90 }
    }
    fn display_frames(&self) -> u64 { self.display.lcd.frames }
    fn report(&self) -> String {
        let mut report = format!("[cardputer] RAMWR={} pixels={} RDDID={} backlight={}",
            self.display_frames(), self.display.lcd.pixels_written, self.display.lcd_gpio_reads,
            self.display.lcd_bl);
        if self.ir.count != 0 {
            report += &format!("\n[cardputer] GPIO44 NEC: {} frames", self.ir.count);
            for &(address, command) in &self.ir.recent { report += &format!(" address=0x{address:04x}/command=0x{command:02x}"); }
        }
        if let Some(sd) = &self.sd { report += &format!("\n{}", sd.report()); }
        report
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matrix_decodes_selected_output_and_releases_the_column() {
        let mut board = Cardputer::new();
        board.keyboard.selector = Some(5); // row 2, even column: 'a'
        assert!(board.board_command(10, "key-edge", &["a", "1"]));
        assert_eq!(board.take_edges(), [BoardEdge { cycle: 10, pin: 15, level: false }]);
        board.keyboard.selector = Some(1);
        board.scan();
        assert_eq!(board.take_edges(), [BoardEdge { cycle: 10, pin: 15, level: true }]);
        assert!(board.board_command(20, "key-edge", &["a", "0"]));
        assert_eq!(board.released_inputs(), [15]);
    }
}
