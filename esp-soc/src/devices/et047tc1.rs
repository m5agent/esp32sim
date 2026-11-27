//! Digital endpoint of the ED047TC1/ET047TC1 source and gate driver.
//! M5GFX 28d47949 Panel_EPD.inl sends two bits per pixel, most significant first.
//! We decode completed waveform sequences and publish only when the boost rail drops.
//! Reflectance and temperature dependence are outside this digital model.

const COLUMNS: usize = 960;
const ROWS: usize = 540;
const LINE_BYTES: usize = COLUMNS / 4;

// Quality and text signatures, chronological symbols packed MSB first. Trailing
// no-drive frames are trimmed before matching. Derived from M5GFX's v0.3.0 LUTs.
const GRAY_SIGNATURES: [[u64; 16]; 2] = [
    [0x1a555f5, 0x6955695, 0x6959e95, 0x6959a95, 0x6955ab5, 0x69566a5, 0x6955aa5, 0xa959aa5, 0x6966aa5, 0xa95aaa5, 0xa955aad, 0x6956aad, 0x1a555aa6, 0x1a559aa6, 0x1a55aaa6, 0x1a57aaa],
    [0xaa95d5, 0xaaaf55, 0xaaafd5, 0xaa95a5, 0xaa56a5, 0xaa96a5, 0xaa9569, 0xaa9d69, 0xaabd69, 0xaab5a9, 0xaa96a9, 0x2a555a, 0x2aad5a, 0x2aaf5a, 0x2aa56a, 0x556aea],
];

pub struct Et047tc1 {
    pub frame: Vec<u16>,
    pending: Vec<u16>,
    scan: Vec<u8>,
    line: Option<[u8; LINE_BYTES]>,
    wave: Vec<u64>,
    endpoint: Vec<u8>,
    scanning: bool,
    pub scans: u64,
    pub latched_lines: u64,
    pub frames: u64,
    pub version: u64,
}

impl Default for Et047tc1 {
    fn default() -> Self {
        Self {
            frame: vec![0xffff; COLUMNS * ROWS], pending: vec![0xffff; COLUMNS * ROWS],
            scan: Vec::with_capacity(LINE_BYTES * ROWS), line: None,
            wave: vec![0; COLUMNS * ROWS], endpoint: vec![0; COLUMNS * ROWS],
            scanning: false, scans: 0, latched_lines: 0, frames: 0, version: 0,
        }
    }
}

impl Et047tc1 {
    pub const WIDTH: u32 = ROWS as u32;
    pub const HEIGHT: u32 = COLUMNS as u32;

    pub fn start_scan(&mut self) {
        self.scan.clear();
        self.line = None;
        self.scanning = true;
    }

    pub fn shift(&mut self, bytes: &[u8]) {
        self.line = bytes.first_chunk().copied().filter(|_| self.scanning);
    }

    pub fn latch(&mut self) {
        let Some(line) = self.line.take() else { return; };
        self.scan.extend_from_slice(&line);
        self.latched_lines += 1;
        if self.scan.len() == LINE_BYTES * ROWS {
            self.scanning = false;
            self.scans += 1;
            for index in 0..self.scan.len() {
                let byte = self.scan[index];
                for offset in 0..4 {
                    let pixel = index * 4 + offset;
                    let symbol = (byte >> (6 - 2 * offset)) & 3;
                    if symbol == 0 {
                        self.finish_pixel(pixel);
                    } else if symbol != 3 || self.wave[pixel] != 0 {
                        self.wave[pixel] = (self.wave[pixel] << 2) | u64::from(symbol);
                        if symbol != 3 { self.endpoint[pixel] = symbol; }
                    }
                }
            }
        }
    }

    fn finish_pixel(&mut self, pixel: usize) {
        let mut signature = std::mem::take(&mut self.wave[pixel]);
        let endpoint = std::mem::take(&mut self.endpoint[pixel]);
        if endpoint == 0 { return; }
        while signature & 3 == 3 { signature >>= 2; }
        let level = GRAY_SIGNATURES.iter().find_map(|mode| {
            mode.iter().position(|&pattern| pattern == signature)
        }).unwrap_or(if endpoint == 1 { 0 } else { 15 });
        let gray = (level * 17) as u16;
        let portrait = (pixel % COLUMNS) * ROWS + ROWS - 1 - pixel / COLUMNS;
        self.pending[portrait] = (gray >> 3) << 11 | (gray >> 2) << 5 | (gray >> 3);
    }

    pub fn power_off(&mut self) {
        self.scanning = false;
        self.line = None;
        for pixel in 0..self.wave.len() { self.finish_pixel(pixel); }
        if self.pending != self.frame {
            self.frame.copy_from_slice(&self.pending);
            self.version += 1;
            self.frames += 1;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn complete_scan_is_invisible_until_rail_off() {
        let mut panel = Et047tc1::default();
        panel.start_scan();
        panel.shift(&[0x55; LINE_BYTES]);
        assert_eq!(panel.latched_lines, 0);
        panel.latch();
        assert_eq!(panel.latched_lines, 1);
        panel.power_off();
        assert_eq!(panel.version, 0, "incomplete gate scan is discarded");
        panel.start_scan();
        for _ in 0..ROWS {
            panel.shift(&[0x55; LINE_BYTES]);
            panel.latch();
        }
        assert_eq!(panel.frame[0], 0xffff);
        panel.power_off();
        assert_eq!(panel.version, 1);
        assert_eq!(panel.frame[0], 0);
    }
    #[test]
    fn gray_signature_survives_sixteen_trailing_idle_scans() {
        let mut panel = Et047tc1::default();
        let signature = GRAY_SIGNATURES[0][11];
        panel.wave[0] = (signature << 32) | u64::from(u32::MAX);
        panel.endpoint[0] = 1;
        panel.power_off();
        assert_eq!(panel.frame[539], 0xbdd7);
    }

}
