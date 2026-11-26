//! 2D-DMA (DMA2D): four TX ("out") and three RX ("in") channels with 2D-capable descriptors,
//! memory-to-memory and the PPA's data paths. Same IP on the ESP32-P4 (0x5008_8000) and the
//! ESP32-S31 (0x2034_6000).
//!
//! Sources: ESP32-P4 TRM v0.5 chapter "2D Direct Memory Access Controller (2D-DMA)" (descriptor
//! format, 2D block mode, mem-to-mem, peripheral selection); register layout from IDF master
//! components/soc/esp32s31/register/soc/dma2d_{reg,struct}.h, descriptor from
//! components/esp_hal_dma/include/hal/dma2d_types.h (`dma2d_descriptor_align8_t`), peri_sel ids
//! from components/soc/esp32s31/include/soc/dma2d_channel.h, channel reset sequence from
//! components/esp_hal_dma/dma2d_hal.c. P4 rev < 3 fields and IDs checked against ESP-IDF
//! v5.4.1/v5.5.4 soc/esp32p4 dma2d_reg.h, include/soc/dma2d_channel.h, and hal/esp32p4
//! include/hal/dma2d_ll.h and dma2d_types.h.
//!
//! Model: a transfer completes synchronously when everything it needs has been started (both
//! channels of an M2M pair; the PPA plus its channels). The FIFOs, burst lengths, arbitration,
//! reorder and macro-block sizes only shape timing and are kept as storage. Packed three-byte
//! colour conversion supports the programmable matrix, RGB565 expansion and quantization.
//! Unsupported subsampling modes are refused.
use crate::device::{Device, WriteEffect};
use crate::ppa::{Ppa, PpaOp};
use crate::sdmmc::SdmmcMemory;
use emu_core::Fault;

pub const DMA2D_OUT_CHANNELS: usize = 4;
pub const DMA2D_IN_CHANNELS: usize = 3;
const IN_BASE: u32 = 0x500;
const STRIDE: u32 = 0x100;
const COMMON: u32 = 0xa00;
/// Descriptors walked per transfer before the chain is treated as looping.
const STEP_BUDGET: usize = 1 << 16;
const STREAM_LIMIT: usize = 64 * 1024 * 1024;

/// dma2d_channel.h: PPA SRM uses peri_sel 1 on both sides; blending reads background on TX 2,
/// foreground on TX 3, writes on RX 2. M2M uses any of TX 4-7 / RX 3-7 (dma2d_ll.h masks 0xF0/0xF8).
pub const PERI_PPA_SRM: u32 = 1;
pub const PERI_PPA_BLEND_RX: u32 = 2;
pub const PERI_PPA_BLEND_BG_TX: u32 = 2;
pub const PERI_PPA_BLEND_FG_TX: u32 = 3;

// Interrupt bits (dma2d_struct.h out/in_int_raw).
const OUT_DONE: u32 = 1 << 0; const OUT_EOF: u32 = 1 << 1; const OUT_DSCR_ERR: u32 = 1 << 2; const OUT_TOTAL_EOF: u32 = 1 << 3;
const IN_DONE: u32 = 1 << 0; const IN_SUC_EOF: u32 = 1 << 1; const IN_DSCR_ERR: u32 = 1 << 3; const IN_DSCR_EMPTY: u32 = 1 << 10;

#[derive(Clone, Copy)]
pub struct Dma2dCh {
    /// Register words of the channel's 0x100 window (storage for what is not interpreted).
    pub regs: [u32; 64],
    pub running: bool,
    /// Descriptor the channel will fetch next.
    pub desc: u32,
}
impl Dma2dCh {
    fn new(input: bool) -> Self {
        let mut regs = [0; 64];
        // COLOR_CONVERT reset: INPUT_SEL 7 (disabled); OUTPUT_SEL default per direction.
        regs[if input { 0x4c } else { 0x48 } / 4] = 7 << 3;
        Dma2dCh { regs, running: false, desc: 0 }
    }
    fn conf0(&self) -> u32 { self.regs[0] }
    pub fn int_raw(&self) -> u32 { self.regs[1] }
    fn int_ena(&self) -> u32 { self.regs[2] }
    pub fn irq(&self) -> bool { self.int_raw() & self.int_ena() != 0 }
    fn raise(&mut self, bits: u32) { self.regs[1] |= bits; }
}

/// What a service pass refused: register offset (DMA2D block, or PPA block when `ppa`) that asked for it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Refusal { pub ppa: bool, pub off: u32 }

pub struct Dma2d {
    pub out: [Dma2dCh; DMA2D_OUT_CHANNELS],
    pub inp: [Dma2dCh; DMA2D_IN_CHANNELS],
    common: [u32; 12],
    pub dbg: bool,
    /// Completed transfers, for tests and the stop report: (m2m, ppa).
    pub transfers: (u64, u64),
}
impl Default for Dma2d { fn default() -> Self { Self::new() } }

/// One `dma2d_descriptor_align8_t` (24 bytes, 8-byte aligned).
#[derive(Clone, Copy, Debug)]
struct Desc { w0: u32, w1: u32, w2: u32, buf: u32, next: u32 }
impl Desc {
    fn read(mem: &mut impl SdmmcMemory, addr: u32) -> Result<Self, Fault> {
        let mut b = [0u8; 20];
        mem.read(addr, &mut b)?;
        let w = |i: usize| u32::from_le_bytes([b[i], b[i + 1], b[i + 2], b[i + 3]]);
        Ok(Desc { w0: w(0), w1: w(4), w2: w(8), buf: w(12), next: w(16) })
    }
    fn vb(&self) -> usize { (self.w0 & 0x3fff) as usize }
    fn hb(&self) -> usize { (self.w0 >> 14 & 0x3fff) as usize }
    fn d2(&self) -> bool { self.w0 >> 29 & 1 != 0 }
    fn suc_eof(&self) -> bool { self.w0 >> 30 & 1 != 0 }
    fn owner(&self) -> bool { self.w0 >> 31 != 0 }
    fn va(&self) -> usize { (self.w1 & 0x3fff) as usize }
    fn ha(&self) -> usize { (self.w1 >> 14 & 0x3fff) as usize }
    fn pbyte(&self) -> u32 { self.w1 >> 28 }
    fn y(&self) -> usize { (self.w2 & 0x3fff) as usize }
    fn x(&self) -> usize { (self.w2 >> 14 & 0x3fff) as usize }
    fn multiple(&self) -> bool { self.w2 >> 28 & 1 != 0 }
    /// 1D: buffer length (TX bytes to send / RX bytes received) = hb_length | ha_length << 14.
    fn length(&self) -> usize { self.hb() | self.ha() << 14 }
    /// 1D: buffer size (RX capacity) = vb_size | va_size << 14.
    fn size(&self) -> usize { self.vb() | self.va() << 14 }
    /// The rows a 2D descriptor covers: (first byte address, bytes per row, row stride, rows).
    /// Single mode is the hb x vb block at (x, y) of the ha-wide picture. Multiple mode walks
    /// hb x vb blocks until the whole ha x va picture is done; the result in memory is the whole
    /// picture, which this model moves in raster order.
    fn rows(&self) -> Option<(u32, usize, usize, usize)> {
        // pbyte: 0 = 0.5 B/px, 1 = 1, 2 = 1.5, 3 = 2, 4 = 3, 5 = 4 (dma2d_types.h).
        let half = match self.pbyte() { 0 => 1, 1 => 2, 2 => 3, 3 => 4, 4 => 6, 5 => 8, _ => return None };
        let stride = self.ha() * half / 2;
        let (x, y, w, h) = if self.multiple() { (0, 0, self.ha(), self.va()) } else { (self.x(), self.y(), self.hb(), self.vb()) };
        if w == 0 || h == 0 || x + w > self.ha() || y + h > self.va() || (x * half) & 1 != 0 { return None; }
        let start = self.buf.wrapping_add(((y * self.ha() + x) * half / 2) as u32);
        Some((start, (w * half).div_ceil(2), stride, h))
    }
}

/// How a TX chain ended.
struct Gathered { data: Vec<u8>, eof_desc: u32, next: u32 }
#[derive(Clone, Copy)]
struct Block { width: usize, height: usize, pbyte: u32 }

impl Dma2d {
    pub fn new() -> Self {
        Dma2d { out: [Dma2dCh::new(false); DMA2D_OUT_CHANNELS], inp: [Dma2dCh::new(true); DMA2D_IN_CHANNELS], common: [0; 12], dbg: false, transfers: (0, 0) }
    }
    /// Interrupt sources, bit i = the chip table's order: IN ch0-2 then OUT ch0-3.
    pub fn irq_bits(&self) -> u64 {
        let mut v = 0;
        for (i, c) in self.inp.iter().enumerate() { if c.irq() { v |= 1 << i; } }
        for (i, c) in self.out.iter().enumerate() { if c.irq() { v |= 1 << (DMA2D_IN_CHANNELS + i); } }
        v
    }
    fn channel(&mut self, off: u32) -> Option<(bool, usize, u32)> {
        if off < STRIDE * DMA2D_OUT_CHANNELS as u32 { return Some((false, (off / STRIDE) as usize, off % STRIDE)); }
        if (IN_BASE..IN_BASE + STRIDE * DMA2D_IN_CHANNELS as u32).contains(&off) { return Some((true, ((off - IN_BASE) / STRIDE) as usize, off % STRIDE)); }
        None
    }

    /// Read a TX chain into one byte stream, as the channel would push it into its FIFO.
    fn gather(&mut self, ch: usize, mem: &mut impl SdmmcMemory) -> Result<Gathered, Fault> {
        let c = self.out[ch];
        let (wrback, check_owner) = (c.conf0() & 1 != 0, c.conf0() >> 4 & 1 != 0);
        let mut data = Vec::new();
        let mut addr = c.desc;
        for _ in 0..STEP_BUDGET {
            if addr == 0 || addr & 7 != 0 { return Err(Fault::Unmapped); }
            let d = Desc::read(mem, addr)?;
            if check_owner && !d.owner() { return Err(Fault::Prohibited); }
            if d.d2() {
                let (start, row, stride, rows) = d.rows().ok_or(Fault::Unmapped)?;
                if data.len() + row * rows > STREAM_LIMIT { return Err(Fault::Prohibited); }
                for r in 0..rows {
                    let at = data.len(); data.resize(at + row, 0);
                    mem.read(start.wrapping_add((r * stride) as u32), &mut data[at..])?;
                }
            } else {
                let at = data.len();
                if at + d.length() > STREAM_LIMIT { return Err(Fault::Prohibited); }
                data.resize(at + d.length(), 0);
                mem.read(d.buf, &mut data[at..])?;
            }
            if wrback { mem.write(addr, &(d.w0 & !(1 << 31)).to_le_bytes())?; }
            if self.dbg { eprintln!("[dma2d] out{ch} desc {addr:#010x} w0={:#010x} w1={:#010x} w2={:#010x} buf={:#010x} -> {} bytes", d.w0, d.w1, d.w2, d.buf, data.len()); }
            if d.suc_eof() || d.next == 0 { return Ok(Gathered { data, eof_desc: addr, next: d.next }); }
            addr = d.next;
        }
        Err(Fault::Unmapped)
    }

    /// Write a byte stream down an RX chain. The descriptor holding the last byte gets SUC_EOF
    /// and (1D) the received length; owner is handed back to the CPU on each one used.
    fn scatter(&mut self, ch: usize, data: &[u8], block: Option<Block>, mem: &mut impl SdmmcMemory) -> Result<(), Fault> {
        let mut addr = self.inp[ch].desc;
        let mut pos = 0;
        for _ in 0..STEP_BUDGET {
            if addr == 0 { break; }
            if addr & 7 != 0 { return Err(Fault::Unmapped); }
            let mut d = Desc::read(mem, addr)?;
            if let Some(Block { width, height, pbyte }) = block {
                if !d.d2() || d.multiple() || d.hb() == 0 || d.vb() == 0 { return Err(Fault::Prohibited); }
                d.w0 = (d.w0 & 0xf000_0000) | (width as u32) << 14 | height as u32;
                d.w1 = (d.w1 & 0x0fff_ffff) | pbyte << 28;
            }
            if self.inp[ch].conf0() >> 4 & 1 != 0 && !d.owner() { return Err(Fault::Prohibited); }
            let mut w0 = d.w0 & !(1 << 31);
            if d.d2() {
                let (start, row, stride, rows) = d.rows().ok_or(Fault::Unmapped)?;
                for r in 0..rows {
                    if pos >= data.len() { break; }
                    let n = row.min(data.len() - pos);
                    mem.write(start.wrapping_add((r * stride) as u32), &data[pos..pos + n])?;
                    pos += n;
                }
                if block.is_some() { mem.write(addr.wrapping_add(4), &d.w1.to_le_bytes())?; }
                mem.write(addr, &(w0 | if pos >= data.len() { 1 << 30 } else { 0 }).to_le_bytes())?;
            } else {
                let n = d.size().min(data.len() - pos);
                mem.write(d.buf, &data[pos..pos + n])?;
                pos += n;
                w0 = (w0 & !(0x3fff << 14)) | ((n as u32 & 0x3fff) << 14);
                if pos >= data.len() { w0 |= 1 << 30; }
                mem.write(addr, &w0.to_le_bytes())?;
                mem.write(addr.wrapping_add(4), &((d.w1 & !(0x3fff << 14)) | ((n as u32 >> 14 & 0x3fff) << 14)).to_le_bytes())?;
            }
            if self.dbg { eprintln!("[dma2d] in{ch} desc {addr:#010x} w0={:#010x} w1={:#010x} w2={:#010x} buf={:#010x} <- {pos}/{} bytes", d.w0, d.w1, d.w2, d.buf, data.len()); }
            if pos >= data.len() {
                let c = &mut self.inp[ch];
                c.regs[0x28 / 4] = addr; c.desc = d.next; c.running = false;
                c.raise(IN_DONE | IN_SUC_EOF);
                return Ok(());
            }
            addr = d.next;
        }
        // The chain ran out before the data did: descriptor-empty, the rest is lost.
        let c = &mut self.inp[ch];
        c.running = false; c.raise(IN_DSCR_EMPTY);
        Ok(())
    }

    fn finish_out(&mut self, ch: usize, g: &Gathered) {
        let c = &mut self.out[ch];
        c.regs[0x28 / 4] = g.eof_desc; c.desc = g.next; c.running = false;
        c.raise(OUT_DONE | OUT_EOF | OUT_TOTAL_EOF);
    }
    fn out_error(&mut self, ch: usize) { let c = &mut self.out[ch]; c.running = false; c.raise(OUT_DSCR_ERR); }
    fn in_error(&mut self, ch: usize) { let c = &mut self.inp[ch]; c.running = false; c.raise(IN_DSCR_ERR); }

    /// DMA2D_{OUT,IN}_COLOR_CONVERT/SCRAMBLE and COLOR_PARAM0..5 (IDF v5.4.1
    /// dma2d_struct.h and dma2d_ll_{tx,rx}_configure_color_space_conv). The matrix
    /// consumes high/middle/low bytes and uses signed fixed-point coefficients.
    fn convert(&self, input: bool, ch: usize, data: Vec<u8>) -> Result<Vec<u8>, Refusal> {
        let (c, base, off) = if input { (&self.inp[ch], IN_BASE + STRIDE * ch as u32, 0x4c) }
                             else { (&self.out[ch], STRIDE * ch as u32, 0x48) };
        let ctrl = c.regs[off as usize / 4];
        let (src, dst) = (ctrl >> 3 & 7, ctrl & 3);
        if src == 7 { return Ok(data); }
        let refuse = Refusal { ppa: false, off: base + off };
        let src_bytes = match (input, src) { (false, 0 | 2) | (true, 1) => 2, (false, 3) | (true, 2) => 3, _ => return Err(refuse) };
        let dst_bytes = match (input, dst) { (_, 0) => 2, (false, 2) | (true, 1) => 3, _ => return Err(refuse) };
        let expand = !input && src == 0;
        let proc = ctrl & 4 != 0;
        let scramble = c.regs[off as usize / 4 + 1];
        let orders = [[0, 1, 2], [1, 0, 2], [2, 0, 1], [0, 2, 1], [1, 2, 0], [2, 1, 0]];
        let pre = orders.get((scramble & 7) as usize).ok_or(refuse)?;
        let post = orders.get(if input { (scramble >> 3 & 7) as usize } else { 0 }).ok_or(refuse)?;
        if !data.len().is_multiple_of(src_bytes) || data.len() / src_bytes * dst_bytes > STREAM_LIMIT { return Err(refuse); }
        let sign = |v: u32, bits: u32| ((v << (32 - bits)) as i32) >> (32 - bits);
        let coeff: [[i32; 4]; 3] = std::array::from_fn(|i| {
            let index = off as usize / 4 + 2 + i * 2;
            let (a, b) = (c.regs[index], c.regs[index + 1]);
            [sign(a, 10), sign(a >> 10, 11), sign(b, 10), sign(b >> 10, 18)]
        });
        if src_bytes == 2 && !expand {
            // Opaque two-byte pixels can pass through the scrambler without reinterpretation.
            if dst == 0 || proc || pre[2] != 2 || post[2] != 2 { return Err(refuse); }
            if pre[0] != post[0] {
                let mut data = data;
                for pixel in data.as_chunks_mut::<2>().0 { pixel.swap(0, 1); }
                return Ok(data);
            }
            return Ok(data);
        }
        let mut out = Vec::with_capacity(data.len() / src_bytes * dst_bytes);
        for bytes in data.chunks_exact(src_bytes) {
            let p = if expand {
                let word = u16::from_le_bytes([bytes[0], bytes[1]]);
                let (r, g, b) = (word >> 11, word >> 5 & 63, word & 31);
                [(b << 3 | b >> 2) as u8, (g << 2 | g >> 4) as u8, (r << 3 | r >> 2) as u8]
            } else { [bytes[0], bytes[1], bytes[2]] };
            let p = pre.map(|i| p[i]);
            let p = if proc {
                let [lo, mid, hi] = p.map(i32::from);
                let convert = |[a, b, c, d]: [i32; 4]| ((a * hi + b * mid + c * lo + d) >> 8).clamp(0, 255) as u8;
                [convert(coeff[2]), convert(coeff[1]), convert(coeff[0])]
            } else { p };
            let [lo, mid, hi] = post.map(|i| p[i]);
            if dst_bytes == 2 {
                let pixel = (u16::from(hi) >> 3) << 11 | (u16::from(mid) >> 2) << 5 | u16::from(lo) >> 3;
                out.extend_from_slice(&pixel.to_le_bytes());
            } else { out.extend_from_slice(&[lo, mid, hi]); }
        }
        Ok(out)
    }

    fn running_out(&self, peri: impl Fn(u32) -> bool) -> Option<usize> {
        // Arbitration between simultaneously armed channels is by priority, then channel number.
        // Token weights and timeouts do not affect a synchronous transfer.
        (0..DMA2D_OUT_CHANNELS).rev().filter(|&i| self.out[i].running && peri(self.out[i].regs[0x38 / 4] & 7))
            .max_by_key(|&i| self.out[i].regs[0x3c / 4] >> 4 & 3)
    }
    fn running_in(&self, m2m: bool, peri: u32) -> Option<usize> {
        (0..DMA2D_IN_CHANNELS).rev()
            .filter(|&i| { let c = &self.inp[i]; c.running && (c.conf0() & 1 != 0) == m2m && (m2m || c.regs[0x3c / 4] & 7 == peri) })
            .max_by_key(|&i| self.inp[i].regs[0x40 / 4] >> 4 & 1)
    }

    /// Run every transfer whose channels (and PPA engine) are all started. Returns what was refused.
    pub fn service(&mut self, ppa: &mut Ppa, mem: &mut impl SdmmcMemory) -> Vec<Refusal> {
        let mut refused = Vec::new();
        // Memory-to-memory: RX channel with IN_MEM_TRANS_EN takes the stream of a started M2M TX
        // channel (peri_sel 4-7). TX and RX selection IDs are independent (dma2d_channel.h).
        while let Some(rx) = self.running_in(true, 0) {
            let Some(tx) = self.running_out(|p| p >= 4) else { break };
            match self.gather(tx, mem) {
                Ok(g) => {
                    self.finish_out(tx, &g);
                    match self.convert(false, tx, g.data).and_then(|data| self.convert(true, rx, data)) {
                        Ok(data) => {
                            if self.scatter(rx, &data, None, mem).is_err() { self.in_error(rx); }
                            self.transfers.0 += 1;
                        }
                        Err(r) => { refused.push(r); self.in_error(rx); }
                    }
                }
                Err(_) => { self.out_error(tx); self.in_error(rx); }
            }
        }
        // PPA operations wait for their channels.
        for op in ppa.pending() {
            let rx = self.running_in(false, if op == PpaOp::Srm { PERI_PPA_SRM } else { PERI_PPA_BLEND_RX });
            let need: &[u32] = match op { PpaOp::Fill => &[], PpaOp::Bypass => &[PERI_PPA_BLEND_BG_TX], PpaOp::Blend => &[PERI_PPA_BLEND_BG_TX, PERI_PPA_BLEND_FG_TX], PpaOp::Srm => &[PERI_PPA_SRM] };
            let txs: Vec<Option<usize>> = need.iter().map(|&p| self.running_out(|q| q == p)).collect();
            if let (Some(rx), true) = (rx, txs.iter().all(Option::is_some)) {
                let txs: Vec<usize> = txs.into_iter().flatten().collect();
                let shape = txs.first().and_then(|&tx| Desc::read(mem, self.out[tx].desc).ok())
                    .filter(|d| d.d2() && !d.multiple()).map(|d| (d.hb(), d.vb()));
                let pixels = shape.map_or(0, |(w, h)| w * h);
                let mut inputs = Vec::new();
                let mut ok = true;
                for &tx in &txs {
                    match self.gather(tx, mem) {
                        Ok(g) => {
                            self.finish_out(tx, &g);
                            match self.convert(false, tx, g.data) {
                                Ok(data) => inputs.push(data),
                                Err(r) => { refused.push(r); ok = false; break; }
                            }
                        }
                        Err(_) => { self.out_error(tx); ok = false; break; }
                    }
                }
                if ok {
                    let result = if op == PpaOp::Srm {
                        match shape {
                            Some((w, h)) => ppa.srm(&inputs[0], w, h)
                                .map(|(data, width, height, pbyte)| (data, Some(Block { width, height, pbyte }))),
                            None => { ppa.cancel(op); Err(0x64) }
                        }
                    } else { ppa.run(op, &inputs, pixels).map(|data| (data, None)) };
                    match result {
                        Ok((out, block)) => match self.convert(true, rx, out) {
                            Ok(out) => {
                                if self.scatter(rx, &out, block, mem).is_err() { self.in_error(rx); }
                                self.transfers.1 += 1;
                            }
                            Err(r) => { refused.push(r); self.in_error(rx); }
                        },
                        Err(off) => { refused.push(Refusal { ppa: true, off }); self.in_error(rx); }
                    }
                } else {
                    for tx in txs { self.out[tx].running = false; }
                    self.in_error(rx);
                    ppa.cancel(op);
                }
            }
        }
        refused
    }
}

impl Device for Dma2d {
    fn read(&mut self, off: u32) -> u32 {
        if let Some((input, ch, o)) = self.channel(off) {
            let c = if input { &self.inp[ch] } else { &self.out[ch] };
            return match o {
                0x0c => c.int_raw() & c.int_ena(),
                0x10 => 0,
                // FIFO status: L1/L2/L3 empty (bits 1, 18, 25), nothing in flight in this model.
                0x14 => 1 << 1 | 1 << 18 | 1 << 25,
                // LINK_CONF: start/restart self-clear, PARK reads 1 while the channel is idle.
                0x1c => { let park = if input { 1 << 24 } else { 1 << 23 }; let pulse = if input { 3 << 22 } else { 3 << 21 }; (c.regs[7] & !pulse & !park) | if c.running { 0 } else { park } }
                // STATE: descriptor address low 18 bits, state, RESET_AVAIL (out bit 24, in bit 23):
                // transfers complete synchronously, so a channel is always safe to reset.
                0x24 => (c.desc & 0x3ffff) | if input { 1 << 23 } else { 1 << 24 },
                0x2c if !input => c.desc, 0x30 if input => c.desc,
                _ => c.regs[(o / 4) as usize],
            };
        }
        if (COMMON..COMMON + 0x30).contains(&off) {
            // DATE (IDF master dma2d_reg.h reset value).
            if off == 0xa2c { return self.common[11].max(0x2306_0100); }
            return self.common[((off - COMMON) / 4) as usize];
        }
        0
    }
    fn write(&mut self, off: u32, v: u32) -> WriteEffect {
        if let Some((input, ch, o)) = self.channel(off) {
            let dbg = self.dbg;
            let c = if input { &mut self.inp[ch] } else { &mut self.out[ch] };
            if dbg { eprintln!("[dma2d] {}{ch} +{o:#04x} <- {v:#010x}", if input { "in" } else { "out" }); }
            match o {
                0x00 => {
                    c.regs[0] = v;
                    // RST (bit 24) resets the channel's state machine and FIFO pointers;
                    // CMD_DISABLE (bit 25) aborts the running transfer (dma2d_hal.c reset sequence).
                    if v & (1 << 24 | 1 << 25) != 0 { c.running = false; }
                }
                0x04 | 0x0c | 0x14 | 0x24 => {}
                0x10 => c.regs[1] &= !v,
                0x1c => {
                    let (stop, start, restart) = if input { (21, 22, 23) } else { (20, 21, 22) };
                    c.regs[7] = v;
                    if v >> stop & 1 != 0 { c.running = false; }
                    if v >> start & 1 != 0 { c.desc = c.regs[8]; c.running = true; }
                    if v >> restart & 1 != 0 { c.running = true; }
                }
                _ => c.regs[(o / 4) as usize] = v,
            }
            return WriteEffect::NONE;
        }
        if (COMMON..COMMON + 0x2c).contains(&off) { self.common[((off - COMMON) / 4) as usize] = v; }
        WriteEffect::NONE
    }
    fn irq_sources(&self) -> u64 { self.irq_bits() }
    fn debug(&mut self, on: bool) { self.dbg = on; }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ppa::Ppa;
    struct Ram(Vec<u8>);
    impl SdmmcMemory for Ram {
        fn read(&mut self, addr: u32, buf: &mut [u8]) -> Result<(), Fault> { let a = addr as usize; buf.copy_from_slice(self.0.get(a..a + buf.len()).ok_or(Fault::Unmapped)?); Ok(()) }
        fn write(&mut self, addr: u32, data: &[u8]) -> Result<(), Fault> { let a = addr as usize; self.0.get_mut(a..a + data.len()).ok_or(Fault::Unmapped)?.copy_from_slice(data); Ok(()) }
    }
    fn desc(m: &mut Ram, at: u32, w0: u32, w1: u32, w2: u32, buf: u32, next: u32) {
        for (i, w) in [w0, w1, w2, buf, next].into_iter().enumerate() { m.write(at + 4 * i as u32, &w.to_le_bytes()).unwrap(); }
    }
    fn d2(vb: u32, hb: u32, va: u32, ha: u32, pbyte: u32, x: u32, y: u32) -> (u32, u32, u32) {
        (vb | hb << 14 | 1 << 29 | 1 << 30 | 1 << 31, va | ha << 14 | pbyte << 28, y | x << 14)
    }
    fn start(d: &mut Dma2d, input: bool, ch: u32, desc: u32, peri: u32, m2m: bool) {
        let base = if input { IN_BASE } else { 0 } + STRIDE * ch;
        if input && m2m { d.write(base, 1); }
        d.write(base + 0x08, 0x3);
        d.write(base + if input { 0x3c } else { 0x38 }, peri);
        d.write(base + 0x20, desc);
        d.write(base + 0x1c, 1 << if input { 22 } else { 21 });
    }

    #[test]
    fn reset_sequence_sees_reset_avail() {
        let mut d = Dma2d::new();
        d.write(0x100, 1 << 25);
        assert_ne!(d.read(0x124) & (1 << 24), 0);
        d.write(0x100, 1 << 24 | 1 << 25); d.write(0x100, 1 << 25); d.write(0x100, 0);
        assert_ne!(d.read(0x624) & (1 << 23), 0);
    }

    #[test]
    fn m2m_1d_copy_sets_eof_and_length() {
        let mut m = Ram(vec![0; 0x4000]);
        for i in 0..100u32 { m.0[0x1000 + i as usize] = i as u8 ^ 0x5a; }
        desc(&mut m, 0x100, 100 << 14 | 1 << 30 | 1 << 31, 0, 0, 0x1000, 0);
        desc(&mut m, 0x200, 128 | 1 << 31, 0, 0, 0x2000, 0);
        let (mut d, mut p) = (Dma2d::new(), Ppa::new());
        start(&mut d, false, 1, 0x100, 4, false);
        assert!(d.service(&mut p, &mut m).is_empty());
        assert!(d.out[1].running, "TX waits for its RX partner");
        start(&mut d, true, 1, 0x200, 3, true);
        assert!(d.service(&mut p, &mut m).is_empty());
        assert_eq!(&m.0[0x2000..0x2064], &m.0[0x1000..0x1064].to_vec()[..]);
        assert_eq!(m.0[0x2064], 0);
        assert_eq!(d.out[1].int_raw() & OUT_EOF, OUT_EOF);
        assert_eq!(d.inp[1].int_raw() & IN_SUC_EOF, IN_SUC_EOF);
        assert_eq!(d.read(0x628), 0x200);
        let w0 = u32::from_le_bytes(m.0[0x200..0x204].try_into().unwrap());
        assert_eq!((w0 >> 14 & 0x3fff, w0 >> 30), (100, 1), "received length and SUC_EOF, owner back to CPU");
        assert_eq!(d.irq_bits(), 1 << 1 | 1 << 4);
        d.write(0x110, 0xff); d.write(0x610, 0xff); assert_eq!(d.irq_bits(), 0);
    }

    #[test]
    fn m2m_2d_block_copies_a_window_between_pictures() {
        // 4x3 RGB565 block at (2,1) of an 8x4 source into (1,2) of a 6x6 destination.
        let mut m = Ram(vec![0; 0x4000]);
        for i in 0..64u32 { m.0[0x1000 + i as usize] = i as u8; }
        let (w0, w1, w2) = d2(3, 4, 4, 8, 3, 2, 1); desc(&mut m, 0x100, w0, w1, w2, 0x1000, 0);
        let (w0, w1, w2) = d2(3, 4, 6, 6, 3, 1, 2); desc(&mut m, 0x200, w0, w1, w2, 0x2000, 0);
        let (mut d, mut p) = (Dma2d::new(), Ppa::new());
        start(&mut d, false, 0, 0x100, 4, false); start(&mut d, true, 0, 0x200, 3, true);
        d.service(&mut p, &mut m);
        for r in 0..3 {
            let src = 0x1000 + ((1 + r) * 8 + 2) * 2; let dst = 0x2000 + ((2 + r) * 6 + 1) * 2;
            let want = m.0[src..src + 8].to_vec(); assert_eq!(&m.0[dst..dst + 8], &want[..]);
            assert_eq!(m.0[dst - 1], 0); assert_eq!(m.0[dst + 8], 0);
        }
    }

    #[test]
    fn ppa_fill_writes_the_block_through_rx_channel() {
        let mut m = Ram(vec![0; 0x4000]);
        // 2x2 RGB565 block at (1,1) of a 4x3 picture.
        let (w0, w1, w2) = d2(2, 2, 3, 4, 3, 1, 1); desc(&mut m, 0x200, w0, w1, w2, 0x2000, 0);
        let (mut d, mut p) = (Dma2d::new(), Ppa::new());
        start(&mut d, true, 2, 0x200, PERI_PPA_BLEND_RX, false);
        p.write(0x24, 2 << 8); p.write(0x3c, 2 | 2 << 14); p.write(0x4c, 0xf800);
        p.write(0x34, 1 | 1 << 2 | 1 << 3);
        assert!(d.service(&mut p, &mut m).is_empty());
        let px = |x: usize, y: usize| u16::from_le_bytes([m.0[0x2000 + (y * 4 + x) * 2], m.0[0x2001 + (y * 4 + x) * 2]]);
        assert_eq!([px(1, 1), px(2, 1), px(1, 2), px(2, 2)], [0xf800; 4]);
        assert_eq!([px(0, 1), px(3, 1), px(1, 0)], [0; 3]);
        assert_eq!(p.read(0x10) & 2, 2);
        assert_eq!(d.inp[2].int_raw() & IN_SUC_EOF, IN_SUC_EOF);
    }

    #[test]
    fn ppa_blend_argb_over_rgb565_into_rgb565() {
        let mut m = Ram(vec![0; 0x4000]);
        // bg: 2 RGB565 px (pure blue, pure red); fg: ARGB8888 white @ alpha 0x80, then alpha 0.
        m.0[0x1000..0x1004].copy_from_slice(&[0x1f, 0x00, 0x00, 0xf8]);
        m.0[0x1100..0x1108].copy_from_slice(&[0xff, 0xff, 0xff, 0x80, 0xff, 0xff, 0xff, 0x00]);
        let (w0, w1, w2) = d2(1, 2, 1, 2, 3, 0, 0); desc(&mut m, 0x100, w0, w1, w2, 0x1000, 0);
        let (w0, w1, w2) = d2(1, 2, 1, 2, 5, 0, 0); desc(&mut m, 0x140, w0, w1, w2, 0x1100, 0);
        let (w0, w1, w2) = d2(1, 2, 1, 2, 3, 0, 0); desc(&mut m, 0x200, w0, w1, w2, 0x2000, 0);
        let (mut d, mut p) = (Dma2d::new(), Ppa::new());
        start(&mut d, false, 0, 0x100, PERI_PPA_BLEND_BG_TX, false);
        start(&mut d, false, 1, 0x140, PERI_PPA_BLEND_FG_TX, false);
        start(&mut d, true, 0, 0x200, PERI_PPA_BLEND_RX, false);
        p.write(0x24, 2 | 2 << 8); // bg RGB565, fg ARGB8888 (0), out RGB565
        p.write(0x50, 0xffffff); p.write(0x54, 0); p.write(0x58, 0xffffff); p.write(0x5c, 0);
        p.write(0x34, 1 | 1 << 3);
        assert!(d.service(&mut p, &mut m).is_empty());
        let px = |i: usize| u16::from_le_bytes([m.0[0x2000 + 2 * i], m.0[0x2001 + 2 * i]]);
        // 0x80/255 white over blue: r = g = 128, b = 255 -> RGB565 (16, 32, 31).
        assert_eq!(px(0), 16 << 11 | 32 << 5 | 31);
        assert_eq!(px(1), 0xf800, "transparent foreground leaves the background");
        assert_eq!(d.transfers.1, 1);
    }

    #[test]
    fn ppa_srm_rotates_scales_and_writes_back_the_output_geometry() {
        let mut m = Ram(vec![0; 0x4000]);
        // Two RGB565 pixels: red, then blue. Scale X by 2 and rotate 90° CCW.
        m.0[0x1000..0x1004].copy_from_slice(&[0, 0xf8, 0x1f, 0]);
        let (w0, w1, w2) = d2(1, 2, 1, 2, 3, 0, 0); desc(&mut m, 0x100, w0, w1, w2, 0x1000, 0);
        // RX descriptor uses the IDF's placeholder 2x2 fields and no pbyte value.
        let (w0, w1, w2) = d2(2, 2, 5, 4, 0, 1, 1); desc(&mut m, 0x200, w0, w1, w2, 0x2000, 0);
        let (mut d, mut p) = (Dma2d::new(), Ppa::new());
        start(&mut d, false, 2, 0x100, PERI_PPA_SRM, false);
        start(&mut d, true, 0, 0x200, PERI_PPA_SRM, false);
        p.write(0x20, 2 | 1 << 4); // RGB565 -> RGB888
        p.write(0x64, 2 | 1 << 12 | 1 << 24 | 1 << 27);
        assert!(d.service(&mut p, &mut m).is_empty());
        for (row, color) in [[255, 0, 0], [255, 0, 0], [128, 0, 128], [0, 0, 255]].into_iter().enumerate() {
            let addr = 0x2000 + (row + 1) * 12 + 3;
            assert_eq!(&m.0[addr..addr + 3], &color);
            assert_eq!((m.0[addr - 1], m.0[addr + 3]), (0, 0));
        }
        let rx = Desc::read(&mut m, 0x200).unwrap();
        assert_eq!((rx.hb(), rx.vb(), rx.pbyte()), (1, 4, 4));
        assert_eq!(p.read(0x10) & 1, 1);
        assert_eq!(d.inp[0].int_raw() & IN_SUC_EOF, IN_SUC_EOF);
    }

    #[test]
    fn dma2d_csc_applies_rgb_packing_and_the_programmable_matrix() {
        let mut d = Dma2d::new();
        d.write(0x48, 2); // expand RGB565 and output packed RGB888
        assert_eq!(d.convert(false, 0, vec![0x1f, 0xf8]).unwrap(), [255, 0, 255]);
        d.write(0x48, 3 << 3); // RGB888 -> RGB565
        assert_eq!(d.convert(false, 0, vec![0, 0, 255]).unwrap(), [0, 0xf8]);
        d.write(0x48, 3 << 3 | 4 | 2);
        // A simple programmed red-to-luma matrix with neutral chroma offsets.
        d.write(0x50, 256);
        d.write(0x5c, 128 << 18);
        d.write(0x64, 128 << 18);
        assert_eq!(d.convert(false, 0, vec![14, 10, 200]).unwrap(), [128, 128, 200]);
    }

    #[test]
    fn unsupported_subsampling_is_refused() {
        let mut m = Ram(vec![0; 0x4000]);
        m.0[0x1000] = 9;
        desc(&mut m, 0x100, 4 << 14 | 1 << 30 | 1 << 31, 0, 0, 0x1000, 0);
        desc(&mut m, 0x200, 4 | 1 << 31, 0, 0, 0x2000, 0);
        let (mut d, mut p) = (Dma2d::new(), Ppa::new());
        d.write(0x48, 1 << 3); // input_sel 1: expand subsampled YUV422
        start(&mut d, false, 0, 0x100, 4, false); start(&mut d, true, 0, 0x200, 3, true);
        assert_eq!(d.service(&mut p, &mut m), vec![Refusal { ppa: false, off: 0x48 }]);
        assert_eq!(m.0[0x2000], 0);
    }
}
