// SPDX-License-Identifier: MIT
// Include at the end of esp32/src/i2s.rs in a detached LX6-C checkout.
#[cfg(test)]
mod dac_contract {
    use super::*;

    const I2S0: u32 = 0x3ff4_f000;
    const DESC: u32 = 0x3ffb_0000;
    const PAYLOAD: u32 = DESC + 0x80;
    const DAC_FORCE: u32 = 0x3ff4_8898;
    const DAC0_PAD: u32 = 0x3ff4_8484;
    const DAC1_PAD: u32 = DAC0_PAD + 4;
    const POWER: u32 = (1 << 18) | (1 << 10);
    const FORCE: u32 = 1 << 22;
    const CYCLES_PER_SAMPLE: u64 = CPU_HZ / 32_000;

    fn setup(stereo: bool, pad: u32) -> SocBus {
        let mut bus = SocBus::new(0, [0; 6]);
        // Ring two 16-bit DMA words containing unsigned DAC samples in the high byte.
        for (addr, value) in [
            (0x3ff0_00c0, 1 << 4), (0x3ff0_00c4, 0),
            (DESC, 0xc000_4004), (DESC + 4, PAYLOAD), (DESC + 8, DESC),
            (PAYLOAD, 0xe0f7_208c),
            (pad, POWER), (DAC_FORCE, FORCE),
            (I2S0 + 0xa8, 1 << 5), (I2S0 + 0xac, 50), (I2S0 + 0xb0, (16 << 12) | 50),
            (I2S0 + 0x2c, u32::from(!stereo)), (I2S0 + 0x20, 1 << 12),
            (I2S0 + 0x30, (1 << 29) | (DESC & 0xfffff)),
            (I2S0 + 8, 1 << 4),
        ] {
            bus.write32(addr, value).unwrap();
        }
        bus
    }

    #[test]
    fn cyclic_dac_dma_clocks_two_bcks_consumes_high_bytes_and_restarts_ring() {
        let mut bus = setup(false, DAC0_PAD);
        bus.i2s_step(CYCLES_PER_SAMPLE - 1);
        assert!(bus.periph.i2s[0].inner.pcm.is_empty());
        bus.i2s_step(1);
        assert_eq!(bus.periph.i2s[0].inner.pcm, [-24576]);
        assert_eq!(bus.read32(I2S0 + 0x38), Ok(0));
        bus.i2s_step(CYCLES_PER_SAMPLE * 2);
        assert_eq!(bus.periph.i2s[0].inner.pcm, [-24576, 24576, -24576]);
        assert_eq!(bus.read32(I2S0 + 0x38), Ok(DESC));
        assert_ne!(bus.read32(I2S0 + 0xc).unwrap() & (1 << 12), 0);
    }

    #[test]
    fn stereo_dac_selects_second_slot_and_dac_gate_mutes_dma() {
        let mut bus = setup(true, DAC1_PAD);
        bus.i2s_step(CYCLES_PER_SAMPLE);
        assert_eq!(bus.periph.i2s[0].inner.pcm, [24576]);
        bus.write32(DAC_FORCE, 0).unwrap();
        bus.i2s_step(CYCLES_PER_SAMPLE);
        assert_eq!(bus.periph.i2s[0].inner.pcm, [24576, 0]);
        bus.write32(DAC_FORCE, FORCE).unwrap();
        bus.write32(DAC1_PAD, 0).unwrap();
        bus.i2s_step(CYCLES_PER_SAMPLE);
        assert_eq!(bus.periph.i2s[0].inner.pcm, [24576, 0, 0]);
    }
}
