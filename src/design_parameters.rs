/// The ADC setup time is the number of seconds after the CSn line goes low before the serial clock
/// may begin. This is used for performing the internal ADC conversion.
pub const ADC_SETUP_TIME: f32 = 220e-9;

/// The maximum DAC/ADC serial clock line frequency. This is a hardware limit.
pub const ADC_DAC_SCK_MHZ_MAX: u32 = 50;

/// The optimal counting frequency of the hardware timers used for timestamping and sampling.
pub const TIMER_FREQUENCY_MHZ: u32 = 100;
