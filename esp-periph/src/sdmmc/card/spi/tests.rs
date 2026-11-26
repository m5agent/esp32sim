use super::*;
use crate::sdmmc::{MemStore, SdCardConfig};

fn make_card() -> SpiSdCard {
    let config = SdCardConfig { prg_ns: 100, ..Default::default() };
    SpiSdCard::new(SdCard::with_config(Box::new(MemStore(vec![0; 512 << 10])), config).unwrap())
}

fn send(card: &mut SpiSdCard, index: u8, arg: u32, good_crc: bool) {
    let mut cmd = [0; 6];
    cmd[0] = 0x40 | index;
    cmd[1..5].copy_from_slice(&arg.to_be_bytes());
    cmd[5] = if good_crc { crc7(&cmd[..5]) << 1 | 1 } else { 0 };
    for byte in cmd { card.exchange(byte, 100); }
}

fn clock(card: &mut SpiSdCard) -> u8 { card.exchange(0xff, 100) }

fn command(card: &mut SpiSdCard, index: u8, arg: u32) -> u8 {
    send(card, index, arg, true);
    assert_eq!(clock(card), 0xff);
    clock(card)
}

fn transaction(card: &mut SpiSdCard, index: u8, arg: u32) -> u8 {
    let r1 = command(card, index, arg);
    card.deselect();
    r1
}

fn initialize(card: &mut SpiSdCard) {
    assert_eq!(transaction(card, 0, 0), IDLE);
    assert_eq!(transaction(card, 59, 1), IDLE);
    assert_eq!(command(card, 8, 0x1aa), IDLE);
    assert_eq!(std::array::from_fn::<_, 4, _>(|_| clock(card)), [0, 0, 1, 0xaa]);
    card.deselect();
    assert_eq!(transaction(card, 41, OCR_CCS), IDLE | ILLEGAL);
    for r1 in [IDLE, 0] {
        assert_eq!(transaction(card, 55, 0), IDLE);
        assert_eq!(transaction(card, 41, OCR_CCS), r1);
    }
}

fn read_packet(card: &mut SpiSdCard, len: usize) -> Vec<u8> {
    assert_eq!(clock(card), 0xff);
    assert_eq!(clock(card), DATA_START);
    let data = (0..len).map(|_| clock(card)).collect::<Vec<_>>();
    assert_eq!(u16::from_be_bytes([clock(card), clock(card)]), crc16(&data));
    data
}

fn write_packet(card: &mut SpiSdCard, token: u8, block: &[u8; BLOCK], crc: u16) -> u8 {
    for byte in [token].iter().chain(block).chain(&crc.to_be_bytes()) {
        assert_eq!(card.exchange(*byte, 100), 0xff);
    }
    clock(card)
}

#[test]
fn idle_crc_and_card_registers() {
    let mut card = make_card();
    send(&mut card, 0, 0, false);
    assert_eq!([clock(&mut card), clock(&mut card)], [0xff; 2]);
    assert_eq!(transaction(&mut card, 0, 0), IDLE);
    assert_eq!(command(&mut card, 58, 0), IDLE);
    assert_eq!(u32::from_be_bytes(std::array::from_fn(|_| clock(&mut card))), OCR_VOLTAGE);
    card.deselect();
    initialize(&mut card);
    assert_eq!(command(&mut card, 58, 0), 0);
    assert_eq!(u32::from_be_bytes(std::array::from_fn(|_| clock(&mut card))), OCR_READY | OCR_CCS | OCR_VOLTAGE);
    for (index, register) in [(9, card.card.csd()), (10, card.card.cid())] {
        assert_eq!(command(&mut card, index, 0), 0);
        assert_eq!(read_packet(&mut card, 16), register.to_be_bytes());
    }
    send(&mut card, 16, 512, false);
    assert_eq!([clock(&mut card), clock(&mut card)], [0xff, CRC_ERROR]);
    assert_eq!(command(&mut card, 16, 256), PARAM_ERROR);
    assert_eq!(command(&mut card, 59, 0), 0);
    send(&mut card, 16, 512, false);
    assert_eq!([clock(&mut card), clock(&mut card)], [0xff, 0]);
    card.power_off();
    assert_eq!(command(&mut card, 58, 0), 0xff);
}

#[test]
fn single_write_checks_crc_and_keeps_programming_across_cs() {
    let mut card = make_card();
    initialize(&mut card);
    let block = [0x23; BLOCK];
    assert_eq!(command(&mut card, 24, 3), 0);
    assert_eq!(write_packet(&mut card, DATA_START, &block, crc16(&block) ^ 1), 0x0b);
    assert_eq!(command(&mut card, 17, 3), 0);
    assert_eq!(read_packet(&mut card, BLOCK), [0; BLOCK]);
    assert_eq!(command(&mut card, 24, 3), 0);
    assert_eq!(write_packet(&mut card, DATA_START, &block, crc16(&block)), 0x05);
    card.deselect();
    assert_eq!(card.exchange(0xff, 199), 0);
    assert_eq!(card.exchange(0xff, 200), 0xff);
    card.card.busy_until = 0;
    assert_eq!(command(&mut card, 13, 0), 0);
    assert_eq!(clock(&mut card), 0);
    assert_eq!(command(&mut card, 17, 3), 0);
    assert_eq!(read_packet(&mut card, BLOCK), block);
    assert_eq!(command(&mut card, 17, 1024), ADDRESS_ERROR);
}

#[test]
fn multiple_blocks_stop_at_token_or_cmd12() {
    let mut card = make_card();
    card.card.config.prg_ns = 0;
    initialize(&mut card);
    let blocks = [[0x57; BLOCK], [0xa4; BLOCK]];
    assert_eq!(command(&mut card, 25, 12), 0);
    for block in &blocks {
        assert_eq!(write_packet(&mut card, WRITE_START, block, crc16(block)), 0x05);
    }
    assert_eq!(card.exchange(WRITE_STOP, 100), 0xff);
    assert_eq!(command(&mut card, 18, 12), 0);
    for block in blocks { assert_eq!(read_packet(&mut card, BLOCK), block); }
    assert_eq!(command(&mut card, 12, 0), 0);
    assert_eq!(clock(&mut card), 0xff);
    assert_eq!(card.card.blocks_read, 2);

    assert_eq!(command(&mut card, 24, 15), 0);
    card.exchange(DATA_START, 100);
    for _ in 0..16 { card.exchange(0x43, 100); }
    card.deselect();
    assert_eq!(command(&mut card, 17, 15), 0);
    assert_eq!(read_packet(&mut card, BLOCK), [0; BLOCK]);
}
