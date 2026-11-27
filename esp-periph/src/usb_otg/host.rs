//! P4 HS host port and scatter/gather host channels. Register and QTD layouts are from
//! ESP-IDF v5.5.4 hal/esp32p4/include/hal/usb_dwc_ll.h and hal/usb_dwc_hal.c.
use super::{DmaMemory, host_msc::Msc};

const HCFG: u32 = 0x400;
const HFNUM: u32 = 0x408;
const HAINT: u32 = 0x414;
const HAINTMSK: u32 = 0x418;
const HPRT: u32 = 0x440;
const CH0: u32 = 0x400 + 0x100;
const CHEN: u32 = 1 << 31;
const CHDIS: u32 = 1 << 30;
const PORT_POWER: u32 = 1 << 12;
const PORT_RESET: u32 = 1 << 8;
const PORT_EN: u32 = 1 << 2;
const PORT_EN_CHG: u32 = 1 << 3;
const PORT_CONN_CHG: u32 = 1 << 1;
const PORT_CONN: u32 = 1;

#[derive(Default)]
struct Channel { regs: [u32; 8], left: usize }

#[derive(Default)]
pub(super) struct Host {
    channels: [Channel; 16],
    regs: [u32; 8],
    port: u32,
    disconnected: bool,
    device: Option<UsbDevice>,
    pub transfers: u64,
    pub bytes: u64,
    pub setups: u64,
}

impl Host {
    pub(super) fn reset(&mut self) {
        let device = self.device.take();
        let (transfers, bytes, setups) = (self.transfers, self.bytes, self.setups);
        *self = Self { device, transfers, bytes, setups, ..Self::default() };
    }
    pub(super) fn attach(&mut self, gamepad: bool) {
        self.connect(UsbDevice { gamepad, ..UsbDevice::default() });
    }
    pub(super) fn attach_msc(&mut self, full_speed: bool, image: Option<std::sync::Arc<[u8]>>) {
        self.connect(UsbDevice { msc: Some(Msc::new(image)), full_speed, ..UsbDevice::default() });
    }
    fn connect(&mut self, device: UsbDevice) {
        self.device = Some(device);
        self.disconnected = false;
        if self.port & PORT_POWER != 0 { self.port |= PORT_CONN | PORT_CONN_CHG; }
    }
    pub(super) fn report(&mut self, report: &[u8]) -> bool {
        let Some(device) = &mut self.device else { return false };
        if device.msc.is_some() || report.len() != device.report_len() { return false; }
        device.report = Some(report.to_vec());
        true
    }
    pub(super) fn detach(&mut self) {
        self.device = None;
        if self.port & PORT_CONN != 0 {
            self.port &= !(PORT_CONN | PORT_EN | PORT_CONN_CHG);
            self.disconnected = true;
        }
    }
    pub(super) fn has_device(&self) -> bool { self.device.is_some() }

    fn irqs(&self) -> u32 {
        self.channels.iter().enumerate().fold(0, |m, (i, c)| m | u32::from(c.regs[2] & c.regs[3] != 0) << i)
    }
    pub(super) fn clear_gint(&mut self, value: u32) { if value & (1 << 29) != 0 { self.disconnected = false; } }
    pub(super) fn gint(&self) -> u32 {
        u32::from(self.port & (PORT_EN_CHG | PORT_CONN_CHG) != 0) << 24 |
        u32::from(self.irqs() & self.regs[((HAINTMSK - HCFG) / 4) as usize] != 0) << 25 |
        u32::from(self.disconnected) << 29
    }
    fn channel(off: u32) -> Option<(usize, usize)> {
        (CH0..CH0 + 16 * 0x20).contains(&off).then(|| (((off - CH0) / 0x20) as usize, ((off & 31) / 4) as usize))
    }
    pub(super) fn read(&self, off: u32, microframe: u64) -> u32 {
        if let Some((i, r)) = Self::channel(off) { return self.channels[i].regs[r]; }
        match off {
            HPRT => self.port,
            HFNUM => microframe as u32 & 0xffff,
            HAINT => self.irqs(),
            HCFG..=0x41c => self.regs[((off - HCFG) / 4) as usize],
            _ => 0,
        }
    }
    pub(super) fn write(&mut self, off: u32, v: u32) -> bool {
        if let Some((i, r)) = Self::channel(off) {
            let c = &mut self.channels[i];
            match r {
                0 => {
                    c.regs[0] = v & !CHDIS;
                    if v & CHDIS != 0 {
                        c.regs[0] &= !CHEN;
                        c.regs[2] |= 2; // channel halted
                    } else if v & CHEN != 0 {
                        c.left = (c.regs[4] >> 8 & 255) as usize + 1;
                        return true;
                    }
                }
                2 => c.regs[2] &= !v,
                7 => {}
                _ => c.regs[r] = v,
            }
        } else {
            match off {
                HPRT => {
                    let old = self.port;
                    self.port &= !(v & (PORT_EN_CHG | PORT_CONN_CHG | PORT_EN));
                    self.port = (self.port & !(PORT_POWER | PORT_RESET)) | (v & (PORT_POWER | PORT_RESET));
                    if v & PORT_POWER == 0 { self.port &= !(PORT_EN | PORT_CONN); }
                    else if old & PORT_POWER == 0 && self.has_device() { self.port |= PORT_CONN | PORT_CONN_CHG; }
                    if self.port & PORT_CONN != 0 && old & PORT_RESET != 0 && v & PORT_RESET == 0 {
                        if let Some(device) = &mut self.device {
                            device.bus_reset();
                            // DWC2 HPRT.PRTSPD (ESP-IDF v5.4.1 usb_dwc_struct.h), 0 HS, 1 FS.
                            self.port = (self.port & !(3 << 17)) | (u32::from(device.full_speed) << 17);
                        }
                        self.port |= PORT_EN | PORT_EN_CHG;
                    }
                }
                HCFG | 0x404 | HAINTMSK | 0x41c => self.regs[((off - HCFG) / 4) as usize] = v,
                _ => {}
            }
        }
        false
    }
    pub(super) fn service(&mut self, mem: &mut dyn DmaMemory, dma: bool, log: bool) {
        if self.port & PORT_EN == 0 || !dma || self.regs[0] & (1 << 23) == 0 { return; }
        let Some(dev) = &mut self.device else { return };
        for c in &mut self.channels {
            if c.regs[0] & CHEN == 0 { continue; }
            let addr = (c.regs[0] >> 22 & 127) as u8;
            let ep = (c.regs[0] >> 11 & 15) as u8;
            let input = c.regs[0] & (1 << 15) != 0;
            let kind = c.regs[0] >> 18 & 3;
            let fail = |c: &mut Channel, bits| { c.regs[2] |= 2 | bits; c.regs[0] &= !CHEN; };
            if addr != dev.address || kind == 1 { fail(c, 1 << 7); continue; }
            for _ in 0..64 {
                if c.left == 0 { fail(c, 1); break; }
                let qtd = c.regs[5] & !7;
                let mut b = [0; 8];
                if mem.read(qtd, &mut b).is_err() { fail(c, 1 << 2); break; }
                let status = u32::from_le_bytes([b[0], b[1], b[2], b[3]]);
                let buf = u32::from_le_bytes([b[4], b[5], b[6], b[7]]);
                if status & (1 << 31) == 0 {
                    // A periodic list leaves inactive entries between scheduled frames.
                    if kind != 3 { fail(c, 1 << 11); break; }
                    c.advance(); continue;
                }
                let len = (status & 0x1ffff) as usize;
                let is_setup = !input && status & (1 << 24) != 0;
                let outcome = if input {
                    dev.input(ep, len).and_then(|data| {
                        if mem.write(buf, &data).is_ok() { Ok(data.len()) } else { Err(1 << 2) }
                    })
                } else {
                    let mut data = vec![0; len];
                    if mem.read(buf, &mut data).is_err() { Err(1 << 2) }
                    else { dev.output(ep, is_setup, &data).map(|()| len) }
                };
                let transferred = match outcome { Ok(n) => n, Err(0) => break, Err(bits) => { fail(c, bits); break; } };
                let writeback = (len - transferred) as u32;
                if mem.write(qtd, &writeback.to_le_bytes()).is_err() { fail(c, 1 << 2); break; }
                self.transfers += 1; self.bytes += transferred as u64; self.setups += u64::from(is_setup);
                if log { eprintln!("[usb-hcd] addr={addr} ep={ep} {}{} {transferred}/{len} bytes", if input { "in" } else { "out" }, if is_setup { " setup" } else { "" }); }
                c.advance();
                if status & (3 << 25) != 0 || transferred < len || c.left == 0 {
                    // IDF distinguishes a normal periodic list end (index wraps to zero)
                    // from a shortened IN transfer, which retains the completed QTD index.
                    if kind == 3 && transferred == len { c.regs[5] &= !0x1ff; }
                    fail(c, 1);
                    break;
                }
            }
        }
    }
}
impl Channel {
    fn advance(&mut self) {
        let dma = self.regs[5];
        self.regs[5] = (dma & !0x1ff) | (dma.wrapping_add(8) & 0x1f8);
        self.left -= 1;
    }
}

const KEYBOARD: &[u8] = &[
    0x05,1,0x09,6,0xa1,1,0x05,7,0x19,0xe0,0x29,0xe7,0x15,0,0x25,1,0x75,1,0x95,8,0x81,2,
    0x75,8,0x95,1,0x81,1,0x19,0,0x29,0x65,0x15,0,0x25,0x65,0x75,8,0x95,6,0x81,0,0xc0,
];
const GAMEPAD: &[u8] = &[
    0x05,1,0x09,5,0xa1,1,0x09,0x30,0x09,0x31,0x15,0x81,0x25,0x7f,0x75,8,0x95,2,0x81,2,
    0x05,9,0x19,1,0x29,8,0x15,0,0x25,1,0x75,1,0x95,8,0x81,2,0xc0,
];

#[derive(Default)]
struct UsbDevice {
    msc: Option<Msc>,
    gamepad: bool,
    full_speed: bool,
    address: u8,
    pending_address: Option<u8>,
    configuration: u8,
    response: Vec<u8>,
    report: Option<Vec<u8>>,
}
impl UsbDevice {
    fn report_len(&self) -> usize { if self.gamepad { 3 } else { 8 } }
    fn bus_reset(&mut self) {
        let mut msc = self.msc.take();
        if let Some(device) = &mut msc { device.reset(); }
        *self = Self { msc, gamepad: self.gamepad, full_speed: self.full_speed, ..Self::default() };
    }
    fn input(&mut self, ep: u8, len: usize) -> Result<Vec<u8>, u32> {
        let data = match ep {
            0 if len == 0 => {
                if let Some(addr) = self.pending_address.take() { self.address = addr; }
                Vec::new()
            }
            0 => self.response.drain(..self.response.len().min(len)).collect(),
            1 if self.configuration != 0 => match &mut self.msc {
                Some(msc) => return msc.input(len),
                None => self.report.take().ok_or(0u32)?,
            },
            _ => return Err(1 << 3), // STALL
        };
        Ok(data.into_iter().take(len).collect())
    }
    fn output(&mut self, ep: u8, setup: bool, data: &[u8]) -> Result<(), u32> {
        if !setup {
            if ep == 0 && data.is_empty() { return Ok(()); }
            if ep == 2 && self.configuration != 0 {
                if let Some(msc) = &mut self.msc { return msc.output(data); }
            }
            return Err(1 << 3);
        }
        let &[typ, request, low, high, _, _, len0, len1] = data else { return Err(1 << 3) };
        let [mps0, mps1] = (if self.full_speed { 64u16 } else { 512u16 }).to_le_bytes();
        let report = if self.gamepad { GAMEPAD } else { KEYBOARD };
        self.response.clear();
        match (typ, request, high) {
            (0x80, 6, 1) => self.response.extend([18,1,0,2,0,0,0,64,0x09,0x12,0x01,0x00,0,1,0,0,0,1]),
            (0x80, 6, 2) if self.msc.is_some() => self.response.extend([
                9,2,32,0,1,1,0,0x80,25, 9,4,0,0,2,8,6,0x50,0,
                7,5,0x81,2,mps0,mps1,0, 7,5,0x02,2,mps0,mps1,0,
            ]),
            (0x80, 6, 2) => self.response.extend([
                9,2,34,0,1,1,0,0x80,25, 9,4,0,0,1,3,if self.gamepad { 0 } else { 1 },if self.gamepad { 0 } else { 1 },0,
                9,0x21,0x11,1,0,1,0x22,report.len() as u8,0, 7,5,0x81,3,self.report_len() as u8,0,4,
            ]),
            (0x81, 6, 0x22) => self.response.extend_from_slice(report),
            (0, 5, 0) if low < 128 => self.pending_address = Some(low),
            (0, 9, 0) if low <= 1 => self.configuration = low,
            (0x80, 8, 0) => self.response.push(self.configuration),
            (0x80..=0x82, 0, 0) => self.response.extend([0,0]),
            (0xa1, 0xfe, 0) if self.msc.is_some() => self.response.push(0), // GET_MAX_LUN
            (0x21, 0xff, 0) if self.msc.is_some() => self.msc.as_mut().unwrap().reset(),
            (0x02, 1, 0) | (0x21, 10 | 11, _) => {}
            _ => return Err(1 << 3),
        }
        self.response.truncate(u16::from_le_bytes([len0, len1]) as usize);
        Ok(())
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use emu_core::Fault;

    struct Ram([u8; 2048]);
    impl DmaMemory for Ram {
        fn read(&mut self, at: u32, data: &mut [u8]) -> Result<(), Fault> {
            let at = at as usize;
            data.copy_from_slice(self.0.get(at..at + data.len()).ok_or(Fault::Unmapped)?);
            Ok(())
        }
        fn write(&mut self, at: u32, data: &[u8]) -> Result<(), Fault> {
            let at = at as usize;
            self.0.get_mut(at..at + data.len()).ok_or(Fault::Unmapped)?.copy_from_slice(data);
            Ok(())
        }
    }

    #[test]
    fn host_channel_setup_and_in_write_back_actual_length() {
        let mut h = Host::default();
        let mut m = Ram([0; 2048]);
        h.attach(true);
        h.write(HPRT, PORT_POWER);
        assert_eq!(h.gint(), 1 << 24);
        h.write(HPRT, PORT_POWER | PORT_CONN_CHG | PORT_RESET);
        h.write(HPRT, PORT_POWER);
        assert_ne!(h.read(HPRT, 0) & PORT_EN, 0);
        h.write(HPRT, PORT_POWER | PORT_EN_CHG);
        h.write(HCFG, 1 << 23);
        h.write(HAINTMSK, 1);
        h.write(CH0 + 0xc, 3);
        m.write(0x700, &[0x80, 6, 0, 1, 0, 0, 18, 0]).unwrap();
        for (control, status, buf) in [(64, 0x8700_0008u32, 0x700u32), (64 | 1 << 15, 0x8600_0040, 0x740)] {
            m.write(0x200, &status.to_le_bytes()).unwrap();
            m.write(0x204, &buf.to_le_bytes()).unwrap();
            h.write(CH0 + 0x14, 0x200);
            h.write(CH0 + 0x10, 0); // one QTD
            assert!(h.write(CH0, CHEN | control));
            h.service(&mut m, true, false);
            assert_eq!(h.read(CH0 + 8, 0), 3);
            assert_eq!(h.gint(), 1 << 25);
            h.write(CH0 + 8, 3);
            assert_eq!(h.gint(), 0);
        }
        assert_eq!(&m.0[0x740..0x744], &[18, 1, 0, 2]);
        assert_eq!(m.0[0x200], 46, "short IN packet leaves a residual length");
        h.detach();
        assert_eq!(h.gint(), 1 << 29);
        h.clear_gint(1 << 29);
        assert_eq!(h.gint(), 0);
    }
}
