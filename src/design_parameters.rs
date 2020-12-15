/// The ADC setup time is the number of seconds after the CSn line goes low before the serial clock
/// may begin. This is used for performing the internal ADC conversion.
pub const ADC_SETUP_TIME: f32 = 220e-9;

/// The maximum DAC/ADC serial clock line frequency. This is a hardware limit.
pub const ADC_DAC_SCK_MHZ_MAX: u32 = 50;

/// The optimal counting frequency of the hardware timers used for timestamping and sampling.
pub const TIMER_FREQUENCY_MHZ: u32 = 100;

/// The DDS reference clock frequency in MHz.
pub const DDS_REF_CLK_MHZ: u32 = 100;

/// The multiplier used for the DDS reference clock PLL.
pub const DDS_MULTIPLIER: u8 = 5;

/// The rate of the DDS SYNC_CLK in MHz is always 1/4 that of the internal PLL clock.
pub const DDS_SYNC_CLK_MHZ: u32 = DDS_REF_CLK_MHZ * DDS_MULTIPLIER as u32 / 4;
