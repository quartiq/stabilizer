use arbitrary_int::u5;
use fugit::MegahertzU32 as MegaHertz;

/// The system clock, used in various timer calculations
pub const SYSCLK: MegaHertz = MegaHertz::MHz(400);

/// The ADC setup time is the number of seconds after the CSn line goes low before the serial clock
/// may begin. This is used for performing the internal ADC conversion.
pub const ADC_SETUP_TIME: f32 = 220e-9;

/// The maximum DAC/ADC serial clock line frequency. This is a hardware limit.
pub const ADC_DAC_SCK_MAX: MegaHertz = MegaHertz::MHz(50);

/// The optimal counting frequency of the hardware timers used for timestamping and sampling.
pub const TIMER_FREQUENCY: MegaHertz = MegaHertz::MHz(100);
pub const TIMER_PERIOD: f32 = 1. / (TIMER_FREQUENCY.to_Hz() as f32);

/// The QSPI frequency for communicating with the pounder DDS.
pub const POUNDER_QSPI_FREQUENCY: MegaHertz = MegaHertz::MHz(50);

/// The delay after initiating a QSPI transfer before asserting the IO_Update for the pounder DDS.
// Pending Pounder Profile writes are up to 32 bytes (QSPI FIFO depth),
// with 2 cycles required per byte, coming out to a total of 64 QSPI clock cycles.
// The QSPI is configured for 50MHz, so this comes out to an offset
// of 1280 ns. We use 1300 ns to be safe.
pub const POUNDER_IO_UPDATE_DELAY: f32 = 1_300e-9;

/// The duration to assert IO_Update for the pounder DDS.
// IO_Update should be latched for 4 SYNC_CLK cycles after the QSPI profile write. With pounder
// SYNC_CLK running at 100MHz (1/4 of the pounder reference clock of 500MHz), this corresponds to
// 32ns. To accomodate rounding errors, we use 50ns instead.
pub const POUNDER_IO_UPDATE_DURATION: f32 = 50e-9;

/// The DDS reference clock frequency in MHz.
pub const DDS_REF_CLK: MegaHertz = MegaHertz::MHz(100);

/// The multiplier used for the DDS reference clock PLL.
pub const DDS_MULTIPLIER: u5 = u5::new(5);

/// The DDS system clock frequency after the internal PLL multiplication.
#[allow(dead_code)]
pub const DDS_SYSTEM_CLK: MegaHertz =
    MegaHertz::MHz(DDS_REF_CLK.to_MHz() * DDS_MULTIPLIER.value() as u32);

/// The divider from the DDS system clock to the SYNC_CLK output (sync-clk is always 1/4 of sysclk).
#[allow(dead_code)]
pub const DDS_SYNC_CLK_DIV: u8 = 4;

/// The maximum ADC/DAC sample processing buffer size.
pub const MAX_SAMPLE_BUFFER_SIZE: usize = 32;

pub type SampleBuffer = [u16; MAX_SAMPLE_BUFFER_SIZE];
