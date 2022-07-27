use embedded_hal::adc::{Channel, OneShot};
use stm32h7xx_hal as hal;

#[derive(Debug, Copy, Clone)]
pub enum AdcError {
    Allocated,
}

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
    pub fn read(&mut self) -> f32 {
        cortex_m::interrupt::free(|cs| {
            let adc = self.adc.borrow(cs);
            adc.borrow_mut().read(&mut self.pin).unwrap() as f32 / self.slope
        })
    }
}

pub struct SharedAdc<Adc> {
    mutex: cortex_m::interrupt::Mutex<
        core::cell::RefCell<hal::adc::Adc<Adc, hal::adc::Enabled>>,
    >,
    allocated_channels: core::cell::RefCell<[bool; 20]>,
    slope: f32,
}

impl<Adc> SharedAdc<Adc> {
    pub fn new(slope: f32, adc: hal::adc::Adc<Adc, hal::adc::Enabled>) -> Self {
        Self {
            slope,
            mutex: cortex_m::interrupt::Mutex::new(core::cell::RefCell::new(
                adc,
            )),
            allocated_channels: core::cell::RefCell::new([false; 20]),
        }
    }

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

pub(crate) use new_shared_adc;
