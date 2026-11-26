//! A USB Mass Storage Bulk-Only Transport target with a small read-only FAT12 volume.
use std::collections::VecDeque;

const SECTOR: usize = 512;
const BLOCKS: usize = 128;
const STALL: u32 = 1 << 3;

#[derive(Default)]
pub(super) struct Msc {
    data: VecDeque<u8>,
    status: Option<[u8; 13]>,
    sense: (u8, u8),
}

impl Msc {
    pub(super) fn input(&mut self, len: usize) -> Result<Vec<u8>, u32> {
        if !self.data.is_empty() { return Ok(self.data.drain(..len.min(self.data.len())).collect()); }
        let status = self.status.take().ok_or(0u32)?;
        if len < status.len() { return Err(STALL); }
        Ok(status.to_vec())
    }
    pub(super) fn output(&mut self, cmd: &[u8]) -> Result<(), u32> {
        // USB-IF Bulk-Only Transport 1.0 §§5.1–5.2: command/status wrappers.
        if self.status.is_some() || cmd.len() != 31 || &cmd[..4] != b"USBC" || cmd[13] != 0 || !(1..=16).contains(&cmd[14]) {
            return Err(STALL);
        }
        let want = u32::from_le_bytes([cmd[8], cmd[9], cmd[10], cmd[11]]);
        let input = cmd[12] & 0x80 != 0;
        let scsi = &cmd[15..];
        let mut fail = false;
        // SCSI SPC/SBC commands used to identify and mount a removable block device.
        let mut payload = match scsi[0] {
            0x00 | 0x1b | 0x1e => Vec::new(), // TEST UNIT READY, START STOP UNIT, PREVENT/ALLOW
            0x03 => {
                let mut data = vec![0; 18];
                let (key, asc) = std::mem::take(&mut self.sense);
                data[0] = 0x70; data[2] = key; data[7] = 10; data[12] = asc;
                data.truncate(scsi[4] as usize);
                data
            }
            0x12 => {
                let mut data = vec![0, 0x80, 4, 2, 31, 0, 0, 0];
                data.extend_from_slice(b"ESP32SIMVIRTUAL DISK    0001");
                data.truncate(scsi[4] as usize);
                data
            }
            0x1a => vec![3, 0, 0x80, 0], // MODE SENSE(6): write protected
            0x25 => [(BLOCKS as u32 - 1).to_be_bytes(), (SECTOR as u32).to_be_bytes()].concat(),
            0x28 => {
                let lba = u32::from_be_bytes([scsi[2], scsi[3], scsi[4], scsi[5]]) as usize;
                let count = u16::from_be_bytes([scsi[7], scsi[8]]) as usize;
                if lba >= BLOCKS || lba.saturating_add(count) > BLOCKS {
                    self.sense = (5, 0x21);
                    fail = true;
                    Vec::new()
                } else { (lba..lba + count).flat_map(sector).collect() }
            }
            _ => { self.sense = (5, 0x20); fail = true; Vec::new() }
        };
        if !input && want != 0 { self.sense = (7, 0x27); return Err(STALL); }
        payload.truncate(want as usize);
        let mut status = [0; 13];
        status[..4].copy_from_slice(b"USBS");
        status[4..8].copy_from_slice(&cmd[4..8]);
        status[8..12].copy_from_slice(&(want - payload.len() as u32).to_le_bytes());
        status[12] = u8::from(fail);
        self.status = Some(status);
        self.data = payload.into();
        Ok(())
    }
}

fn sector(lba: usize) -> [u8; SECTOR] {
    let mut data = [0; SECTOR];
    const HELLO: &[u8] = b"ESP32-P4 virtual USB mass storage\r\n";
    match lba {
        0 => {
            data[..11].copy_from_slice(b"\xeb\x3c\x90ESP32SIM");
            data[11..13].copy_from_slice(&(SECTOR as u16).to_le_bytes());
            data[13..17].copy_from_slice(&[1, 1, 0, 2]); // clusters, reserved sectors, FAT copies
            data[17..19].copy_from_slice(&16u16.to_le_bytes());
            data[19..21].copy_from_slice(&(BLOCKS as u16).to_le_bytes());
            data[21..28].copy_from_slice(&[0xf8, 1, 0, 1, 0, 1, 0]); // media, FAT size, geometry
            data[38] = 0x29;
            data[43..62].copy_from_slice(b"ESP32SIM   FAT12   ");
            data[510..].copy_from_slice(&[0x55, 0xaa]);
        }
        1 | 2 => data[..5].copy_from_slice(&[0xf8, 0xff, 0xff, 0xff, 0x0f]),
        3 => {
            data[..11].copy_from_slice(b"HELLO   TXT");
            data[11] = 0x20;
            data[26..28].copy_from_slice(&2u16.to_le_bytes());
            data[28..32].copy_from_slice(&(HELLO.len() as u32).to_le_bytes());
        }
        4 => data[..HELLO.len()].copy_from_slice(HELLO),
        _ => {}
    }
    data
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn read10_returns_data_and_tagged_status() {
        let mut msc = Msc::default();
        let mut cbw = [0; 31];
        cbw[..8].copy_from_slice(b"USBCtag!");
        cbw[8..12].copy_from_slice(&512u32.to_le_bytes());
        cbw[12] = 0x80;
        cbw[14] = 10;
        cbw[15] = 0x28;
        cbw[20] = 4;
        cbw[23] = 1;
        msc.output(&cbw).unwrap();
        assert!(msc.input(512).unwrap().starts_with(b"ESP32-P4 virtual USB mass storage"));
        assert_eq!(msc.input(13).unwrap(), b"USBStag!\0\0\0\0\0");
    }
}
