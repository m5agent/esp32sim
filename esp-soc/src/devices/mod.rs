//! Device models that are not tied to a chip: the things a board wires to the pins, fed with
//! what the SoC model already decoded (GPIO levels, SPI bytes, RMT bit streams). A board owns
//! them and adds what is module-specific — pin numbers, the visible window of a glass, where
//! each LED of a chain sits physically.
pub mod aw88298;
pub mod aw9523;
pub mod axp2101;
pub mod bm8563;
pub mod bmi270;
pub mod bmm150;
pub mod bmp280;
pub mod cc1101;
pub mod co5300;
pub mod dcs_panel;
pub mod es7210;
pub mod es8311;
pub mod ed2208;
pub mod et047tc1;
pub mod ft5x06;
pub mod gt911;
pub mod imu_motion;
pub mod ina226;
pub mod lm75b;
pub mod mfrc522;
pub mod ltr553;
pub mod mpu6886;
pub mod pi4ioe5v6408;
pub mod rx8130;
pub mod scscl;
pub mod soft_i2c;
pub mod sht40;
pub mod spi_bitbang;
pub mod ssd1677;
pub mod st25r3916;
pub mod sx1262;
pub mod tca8418;
pub mod ws2812;

pub use axp2101::{Axp2101, AXP2101_ADDRESS};
pub use bm8563::{Bm8563, BM8563_ADDRESS};
pub use bmi270::{Bmi270, BMI270_ADDRESS};
pub use bmm150::{Bmm150, Bmm150Trim};
pub use bmp280::{Bmp280, BMP280_ADDRESS};
pub use dcs_panel::DcsPanel;
pub use ft5x06::{Ft5x06, Ft5x06Points, FT5X06_ADDRESS};
pub use ltr553::{Ltr553, LTR553_ADDRESS};
pub use spi_bitbang::SpiBitBang;
pub use ws2812::Ws2812Chain;

pub mod lp5562;
