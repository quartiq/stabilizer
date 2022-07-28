use enum_iterator::IntoEnumIterator;
use num_enum::TryFromPrimitive;
pub mod adc_internal;
pub mod ltc2320;

pub struct DriverDevices {
    pub ltc2320: ltc2320::Ltc2320,
    pub adc_internal: adc_internal::AdcInternal,
}

#[derive(Clone, Copy, TryFromPrimitive, IntoEnumIterator, Debug)]
#[repr(usize)]
pub enum OutputChannelIdx {
    Zero = 0,
    One = 1,
}
