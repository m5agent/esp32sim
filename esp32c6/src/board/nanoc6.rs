//! M5Stack NanoC6 (C125), Sch_M5NanoC6_v0.0.1 and the m5stack_nano_c6 Arduino variant.
//! The IR diode on GPIO3 also provides the signature used by M5GFX's C6 detector.

use esp_soc::{BoardModel, devices::Ws2812Chain, pins::PinRoutes};

#[derive(Default)]
pub struct NanoC6 {
    powered: bool,
    leds: [[u8; 3]; 2],
    changes: u64,
    rgb_frames: u64,
    ir_rmt_frames: u64,
    ir: super::ir::NecTrace,
    rgb: Option<Ws2812Chain>,
}

impl NanoC6 {
    fn set_led(&mut self, index: usize, color: [u8; 3]) {
        if self.leds[index] != color {
            self.leds[index] = color;
            self.changes += 1;
        }
    }
}

impl BoardModel for NanoC6 {
    fn name(&self) -> &'static str { "m5stack-nanoc6" }

    fn input_levels(&self) -> Vec<(u8, bool)> {
        vec![(3, false), (8, true), (9, true)]
    }

    fn named_pin(&self, name: &str) -> Option<u8> {
        match name {
            "btnA" | "button" | "boot" => Some(9),
            _ => None,
        }
    }

    fn uses_gpio_edges(&self) -> bool { false }
    fn observes_gpio_pads(&self) -> bool { true }
    fn observes_ledc(&self) -> bool { true }

    fn ledc_output_at(&mut self, cycle: u64, pwm: &dyn Fn(u8) -> Option<(f64, u32)>) {
        self.ir.drive(cycle, pwm(3).is_some_and(|(hz, duty)|
            (30_000.0..=60_000.0).contains(&hz) && duty != 0));
    }

    fn gpio_waveform_at(&mut self, _cycle: u64, routes: PinRoutes<'_>) {
        self.powered = routes.software_output(19) == Some(true);
        if !self.powered {
            self.rgb = None;
            self.set_led(0, [0; 3]);
        }
        self.set_led(1, if routes.software_output(7) == Some(true) { [0, 0, 255] } else { [0; 3] });
    }

    fn rmt_frame(&mut self, pin: u8, bits: &[bool]) {
        match pin {
            3 => self.ir_rmt_frames += 1,
            20 if self.powered && bits.len() >= 24 => {
                let rgb = self.rgb.get_or_insert_with(|| Ws2812Chain::new(1));
                rgb.from_bits(bits);
                let color = rgb.leds[0];
                self.rgb_frames += 1;
                self.set_led(0, color);
            }
            _ => {}
        }
    }

    fn leds(&self) -> Option<(&[[u8; 3]], u64)> { Some((&self.leds, self.changes)) }

    fn report(&self) -> String {
        format!("[nanoc6] rgb_power={} rgb={:?} blue={:?} led_changes={} rgb_frames={} ir_rmt_frames={} nec_frames={} nec_recent={}",
            self.powered, self.leds[0], self.leds[1], self.changes, self.rgb_frames, self.ir_rmt_frames, self.ir.frames, self.ir.recent())
    }
}
