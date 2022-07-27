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
use embedded_hal::adc::{Channel, OneShot};
use stm32h7xx_hal as hal;

struct AdcMutex<Adc> {
    in_use: core::sync::atomic::AtomicBool,
    adc: core::cell::RefCell<hal::adc::Adc<Adc, hal::adc::Enabled>>,
}

impl<Adc> AdcMutex<Adc> {
    pub fn new(adc: hal::adc::Adc<Adc, hal::adc::Enabled>) -> Self {
        Self {
            adc: core::cell::RefCell::new(adc),
            in_use: core::sync::atomic::AtomicBool::new(false),
        }
    }

    pub fn lock<
        R,
        F: FnOnce(&mut hal::adc::Adc<Adc, hal::adc::Enabled>) -> R,
    >(
        &self,
        f: F,
    ) -> Result<R, AdcError> {
        self.in_use
            .compare_exchange(
                false,
                true,
                core::sync::atomic::Ordering::SeqCst,
                core::sync::atomic::Ordering::SeqCst,
            )
            .map_err(|_| AdcError::InUse)?;

        let result = f(&mut self.adc.borrow_mut());

        self.in_use
            .store(false, core::sync::atomic::Ordering::SeqCst);

        Ok(result)
    }
}

#[derive(Debug, Copy, Clone)]
pub enum AdcError {
    /// Indicates that the ADC channel has already been allocated.
    Allocated,

    /// Indicates that the ADC is already in use
    InUse,
}

/// A single channel on an ADC peripheral.
pub struct AdcChannel<'a, Adc, PIN> {
    pin: PIN,
    slope: f32,
    mutex: &'a AdcMutex<Adc>,
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
    pub fn read(&mut self) -> Result<f32, AdcError> {
        self.mutex
            .lock(|adc| adc.read(&mut self.pin).unwrap() as f32 / self.slope)
    }
}

/// An ADC peripheral that can provide ownership of individual channels for sharing between
/// drivers.
pub struct SharedAdc<Adc> {
    mutex: AdcMutex<Adc>,
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
            mutex: AdcMutex::new(adc),
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
            mutex: &self.mutex,
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
