//! Boards around the ESP32-S3. The SoC model emits generic events (GPIO edges, RMT symbol
//! streams, SPI bytes, LCD_CAM frames, I2S samples); a `BoardModel` interprets them as the
//! devices wired to the pins. One file per board; the device models they share (DCS panels,
//! WS2812 chains, bit-banged SPI) live in `esp_soc::devices`.
//!
//! - `Atech14` — the Atech 14-port motherboard with the Knob, Light Grid and TFT modules
//! - `WaveshareCam` — ESP32-S3-CAM-OV5640
//! - `WaveshareLcd4b` — ESP32-S3-Touch-LCD-4B
//! - `WaveshareAmoled18V2` — ESP32-S3-Touch-AMOLED-1.8 V2
//! - `StopWatch` — M5Stack StopWatch
//! - `Cores3` — M5Stack CoreS3
//! - `NoBoard` — a bare module: nothing on the pins (any ESP32-S3 firmware, console only)
pub use esp_soc::board::{Board, BoardEdge, BoardModel, NoBoard, VirtualCycle};

mod atom_s3;
mod atom_s3_panel;
mod atom_s3_camera;
pub mod atech14;
mod cardputer;
mod cardputer_keys;
pub mod cardputer_adv;
mod cardputer_adv_ir;
mod cardputer_adv_cap;
pub mod cores3;
mod dial;
mod i2c_wiring;
mod pm1_wiring;
mod sticks3;
use esp_soc::devices::soft_i2c;
mod spi_lcd;

pub mod stopwatch;
mod stamplc;
mod stamplc_modbus;
mod stackchan;
pub mod waveshare_amoled18;
pub mod waveshare_cam;
pub mod waveshare_lcd4b;

pub use atech14::*;
pub use cardputer_adv::*;
pub use cores3::*;
pub use stopwatch::*;
pub use waveshare_amoled18::*;
pub use waveshare_cam::*;
pub use waveshare_lcd4b::*;

/// Board by name: `atech14` (default), `none`, `stopwatch` or a supported Waveshare board.
pub fn make_board(name: &str) -> Option<Board> {
    use atom_s3::{AtomBoard, AtomS3};
    let atoms = [AtomBoard::Lite, AtomBoard::S3, AtomBoard::S3Gc9107, AtomBoard::U, AtomBoard::R,
                 AtomBoard::RGc9107, AtomBoard::Voice, AtomBoard::Cam, AtomBoard::M12];
    if let Some(board) = atoms.into_iter().find(|board| board.name() == name) {
        return Some(Box::new(AtomS3::new(board)));
    }
    match name {
        "atech14" | "atech" => Some(Box::new(Atech14::new())),
        "none" | "bare" => Some(Box::new(NoBoard)),
        "sticks3" | "m5stack-sticks3" => Some(Box::new(sticks3::StickS3::new())),
        "stopwatch" | "m5stack-stopwatch" => Some(Box::new(StopWatch::new())),
        "cardputer" | "m5stack-cardputer" => Some(Box::new(cardputer::Cardputer::new())),
        "cardputer-adv" | "m5stack-cardputer-adv" => Some(Box::new(CardputerAdv::new())),
        "dial" | "m5stack-dial" => Some(Box::new(dial::Dial::new())),
        "stackchan" | "m5stack-stackchan" => Some(Box::new(Cores3::stackchan())),
        "stamplc" | "m5stack-stamplc" => Some(Box::new(stamplc::StamPlc::new())),
        "cores3" | "m5stack-cores3" => Some(Box::new(Cores3::new())),
        "waveshare-cam" | "waveshare" => Some(Box::new(WaveshareCam::new())),
        "waveshare-lcd4b" | "lcd4b" => Some(Box::new(WaveshareLcd4b::new())),
        "waveshare-amoled18-v2" | "amoled18-v2" => Some(Box::new(WaveshareAmoled18V2::new())),
        _ => None,
    }
}
