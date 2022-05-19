#![no_std]
#![cfg_attr(feature = "nightly", feature(core_intrinsics))]

pub const RX_INDEX: usize = 9;
pub const TX_INDEX: usize = 8;

pub mod hardware;
pub mod net;

/// Set the pin state of USART3-TX
///
/// # Args
/// * `enabled` - True if the GPIO should be asserted.
pub fn set_tx(enabled: bool) {
    let regs = unsafe { &*stm32h7xx_hal::stm32::GPIOD::ptr() };

    let offset = if enabled { TX_INDEX } else { TX_INDEX + 16 };
    regs.bsrr.write(|w| unsafe { w.bits(1 << offset) });
}

/// Set the pin state of USART3-RX
///
/// # Args
/// * `enabled` - True if the GPIO should be asserted.
pub fn set_rx(enabled: bool) {
    let regs = unsafe { &*stm32h7xx_hal::stm32::GPIOD::ptr() };

    let offset = if enabled { RX_INDEX } else { RX_INDEX + 16 };
    regs.bsrr.write(|w| unsafe { w.bits(1 << offset) });
}
