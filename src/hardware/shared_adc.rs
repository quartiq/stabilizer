use super::hal;
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
/// If the multiple priorities utilize the ADC that results in resource pre-emption, pre-emption is
/// protected against through the use of an atomic bool. Attempting to utilize the ADC from a
/// higher priority level while it is in use at a lower level will result in a [AdcError::InUse].
use embedded_hal_02::adc::{Channel, OneShot};

#[derive(Debug, Copy, Clone)]
pub enum AdcError {
    /// Indicates that the ADC is already in use
    InUse,
}

/// A single channel on an ADC peripheral.
pub struct AdcChannel<'a, Adc, PIN> {
    pin: PIN,
    slope: f32,
    mutex: &'a spin::Mutex<hal::adc::Adc<Adc, hal::adc::Enabled>>,
}

impl<Adc, PIN> AdcChannel<'_, Adc, PIN>
where
    PIN: Channel<Adc, ID = u8>,
    hal::adc::Adc<Adc, hal::adc::Enabled>: OneShot<Adc, u32, PIN>,
    <hal::adc::Adc<Adc, hal::adc::Enabled> as OneShot<Adc, u32, PIN>>::Error:
        core::fmt::Debug,
{
    /// Read the ADC channel and normalize the result.
    ///
    /// # Returns
    /// The normalized ADC measurement as a ratio of full-scale.
    pub fn read_normalized(&mut self) -> Result<f32, AdcError> {
        self.read_raw().map(|code| code as f32 / self.slope)
    }

    /// Read the raw ADC sample for the channel.
    ///
    /// # Returns
    /// The raw ADC code measured on the channel.
    pub fn read_raw(&mut self) -> Result<u32, AdcError> {
        let mut adc = self.mutex.try_lock().ok_or(AdcError::InUse)?;
        Ok(adc.read(&mut self.pin).unwrap())
    }
}

/// An ADC peripheral that can provide ownership of individual channels for sharing between
/// drivers.
pub struct SharedAdc<Adc> {
    mutex: spin::Mutex<hal::adc::Adc<Adc, hal::adc::Enabled>>,
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
            mutex: spin::Mutex::new(adc),
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
    ) -> AdcChannel<'_, Adc, PIN> {
        AdcChannel {
            pin,
            slope: self.slope,
            mutex: &self.mutex,
        }
    }
}
