//! NEC envelope decoder for S3 boards at the S3 bus-cycle frequency.
pub(super) type NecReceiver = esp_soc::devices::nec::NecReceiver<{ crate::periph::CPU_HZ }>;
