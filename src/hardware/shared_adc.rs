/// Shared Internal ADC Support
///
/// # Description
/// This module provides an abstraction to share ownership of a single ADC peripheral with multiple
/// ADC channels attached to it.
///
/// The design of this module mimics that of [`shared-bus`].
///
/// First, the shared ADC is created with the use of a macro, which places the ADC peripheral into
/// a mutable, static (singleton) location. Then, individual channels are created by passing in the
/// associated ADC input pin to the [SharedAdc::create_channel()] function to generate an
/// [AdcChannel]. The [AdcChannel]'s ownership can then be moved to any required drivers.
///
/// ## Synchronization
/// Currently, the sharing of the ADC peripheral across multiple priority levels is managed by the
/// use of a critical section.
use embedded_hal::adc::{Channel, OneShot};
use stm32h7xx_hal as hal;

#[derive(Debug, Copy, Clone)]
pub enum AdcError {
    /// Indicates that the ADC channel has already been allocated.
    Allocated,
}

/// A single channel on an ADC peripheral.
pub struct AdcChannel<'a, Adc, PIN> {
    pin: PIN,
    slope: f32,
    adc: &'a cortex_m::interrupt::Mutex<
        core::cell::RefCell<hal::adc::Adc<Adc, hal::adc::Enabled>>,
    >,
}

impl<'a, Adc, PIN> AdcChannel<'a, Adc, PIN>
where
    PIN: Channel<Adc, ID = u8>,
    hal::adc::Adc<Adc, hal::adc::Enabled>: OneShot<Adc, u32, PIN>,
    <hal::adc::Adc<Adc, hal::adc::Enabled> as OneShot<Adc, u32, PIN>>::Error:
        core::fmt::Debug,
{
    /// Read the ADC channel.
    ///
    /// # Returns
    /// The normalized ADC measurement as a ratio of full-scale.
    pub fn read(&mut self) -> f32 {
        cortex_m::interrupt::free(|cs| {
            let adc = self.adc.borrow(cs);
            adc.borrow_mut().read(&mut self.pin).unwrap() as f32 / self.slope
        })
    }
}

/// An ADC peripheral that can provide ownership of individual channels for sharing between
/// drivers.
pub struct SharedAdc<Adc> {
    mutex: cortex_m::interrupt::Mutex<
        core::cell::RefCell<hal::adc::Adc<Adc, hal::adc::Enabled>>,
    >,
    allocated_channels: core::cell::RefCell<[bool; 20]>,
    slope: f32,
}

impl<Adc> SharedAdc<Adc> {
    /// Construct a new shared ADC driver.
    ///
    /// # Args
    /// * `slope` - The slope of the ADC conversion transfer function.
    /// * `adc` - The ADC peripheral to share.
    pub fn new(slope: f32, adc: hal::adc::Adc<Adc, hal::adc::Enabled>) -> Self {
        Self {
            slope,
            mutex: cortex_m::interrupt::Mutex::new(core::cell::RefCell::new(
                adc,
            )),
            allocated_channels: core::cell::RefCell::new([false; 20]),
        }
    }

    /// Allocate an ADC channel for usage.
    ///
    /// # Args
    /// * `pin` - The ADC input associated with the desired ADC channel. Often, this is a GPIO pin.
    ///
    /// # Returns
    /// An instantiated [AdcChannel] whose ownership can be transferred to other drivers.
    pub fn create_channel<PIN: Channel<Adc, ID = u8>>(
        &self,
        pin: PIN,
    ) -> Result<AdcChannel<'_, Adc, PIN>, AdcError> {
        let mut channels = self.allocated_channels.borrow_mut();
        if channels[PIN::channel() as usize] {
            return Err(AdcError::Allocated);
        }

        channels[PIN::channel() as usize] = true;

        Ok(AdcChannel {
            pin,
            slope: self.slope,
            adc: &self.mutex,
        })
    }
}

macro_rules! new_shared_adc {
    ($adc_type:ty = $adc:expr) => {{
        let m: Option<&'static mut _> = cortex_m::singleton!(
            : $crate::hardware::shared_adc::SharedAdc<$adc_type> = $crate::hardware::shared_adc::SharedAdc::new($adc.slope() as f32, $adc)
        );

        m
    }};
}

/// Construct a shared ADC driver into a global, mutable singleton.
pub(crate) use new_shared_adc;
