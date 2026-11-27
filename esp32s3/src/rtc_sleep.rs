//! ESP32-S3 RTC timer light sleep. See ESP-IDF v5.4.2 rtc_sleep.c and
//! soc/esp32s3/register/soc/rtc_cntl_reg.h.
use crate::bus::SocBus;
use crate::periph::CPU_HZ;
use esp_soc::SocBus as _;

const TIMER_WAKE: u32 = 1 << 3;
const TIMER_MASK: u64 = (1 << 48) - 1;

impl SocBus {
    pub(crate) fn rtc_sleeping(&self) -> bool {
        self.rtc_sleep_target.is_some()
    }

    pub(crate) fn enter_rtc_timer_sleep(&mut self) {
        let rtc = &self.periph.rtc.ram;
        // DG_WRAP_PD_EN selects deep sleep. This path retains digital state.
        if rtc.read(0x90) & (1 << 31) != 0 || rtc.read(0x3c) & (TIMER_WAKE << 15) == 0 {
            return;
        }
        let target = u64::from(rtc.read(4)) | (u64::from(rtc.read(8) & 0xffff) << 32);
        self.rtc_sleep_target = Some(target);
        self.irq_dirty = true;
        if self.debug.has("rtc-sleep") {
            eprintln!("[rtc-sleep] enter timer light sleep t={:.6}s", self.cycles() as f64 / CPU_HZ as f64);
        }
        if self.rtc_timer_remaining() == Some(0) {
            self.finish_rtc_timer_sleep();
        }
    }

    pub(crate) fn rtc_timer_remaining(&self) -> Option<u64> {
        let relative = self.rtc_sleep_target?.wrapping_sub(self.periph.rtc.slow_ticks) & TIMER_MASK;
        Some(if relative & (1 << 47) == 0 { relative } else { 0 })
    }

    pub(crate) fn finish_rtc_timer_sleep(&mut self) {
        self.rtc_sleep_target = None;
        let rtc = &mut self.periph.rtc.ram;
        rtc.write(0x18, rtc.read(0x18) & !(1 << 31));
        rtc.write(0x130, TIMER_WAKE);
        rtc.write(0x44, rtc.read(0x44) | 1);
        self.irq_dirty = true;
        if self.debug.has("rtc-sleep") {
            eprintln!("[rtc-sleep] timer wake t={:.6}s", self.cycles() as f64 / CPU_HZ as f64);
        }
    }
}
