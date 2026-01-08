#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

//! Patent pending: DE102021112017A1
//!
//! # Algorithm description
//!
//! This application can be understood as a universal phase (frequency)
//! signal processor. It determines the phase (we will drop frequency
//! from now on as in a phase-aware system frequency is merely the
//! difference between successive phases) of an RF input signal and
//! emits an RF output signal with a phase that depends on the input
//! phase. The transfer function between input and output phase is a
//! sequence of various types of filters (analog RC, digital FIR, IIR,
//! unwrapping, scaling, clipping) designed to implement either
//! high-quality phase measurements or a certain constrained and
//! somewhat exotic phase locked loop that is highly applicable
//! to the task of stabilizing the arm length of an optical Michelson
//! interferometer which in turn occurs when stabilizing the effective
//! path length of an optical frequency transmission system.
//!
//! The sequence of processing steps is as follows. Analyzing it's
//! application in the context of optical path length stabilization including
//! laser sources, optical modulators, and photodetectors optical is left as
//! an exercise for the user.
//!
//! ## PLL path
//!
//! * DDS locks its sysclk (500 MHz) to XO or external ref
//! * DDS emits SYNC signal at sysclk/4
//! * Prescaler 1/4 (in CPU)
//! * Drives CPU timer counter
//! * Counter is captured once per batch (based on CPU clock).
//!   See [stabilizer::hardware::pounder::timestamp].
//! * Digital PLL reconstructs SYNC frequency and phase (thus sysclk)
//!   w.r.t. batch and sample frequency and phase.
//!   This determines the relation of the CPU 8 MHz crystal (thus CPU
//!   clock and timers) to the DDS clock (derived from an external reference
//!   frequency or internal XCO). See [idsp::PLL].
//!
//! ## Signal path
//!
//! * RF signal enters Pounder at Pounder IN0
//! * Adjustable attenuation `demod_att`.
//! * 30 dB gain block
//! * Mixing with DDS at `demod_freq`
//! * RC lowpass and amplification to reject unwanted demodulation products and
//!   harmonics
//! * IF signal enters Stabilizer and is available at ADC0 for analog monitoring
//! * 2x PGIA and AA filter on Stabilizer
//! * ADC digitization at 1/1.28 µs interval
//! * Data processing in batches of 8 samples
//! * Digital mixing with the reconstructed sample phase (PLL path). See [idsp::Lockin].
//! * Lowpass filtering with a second order (12 dB/octave)
//!   IIR lowpass with an additional double zero at Nyquist. Adjustable corner frequency.
//!   See [idsp::Lowpass]
//! * Full rate baseband demodulated data (quadrature only) on DAC0
//! * Lowpass filtering with a batch-size boxcar FIR filter (zeros at n/4 Nyquist)
//! * Computation of signal power and phase. See [idsp::ComplexExt].
//! * Fractional rescaling (`phase_scale`) and unwrapping of the phase with 32 bit turn range.
//! * Scaling and clamping.
//! * Filtering by a second order (biquad) IIR filter (supporting e.g. II, I, P
//!   action). See [idsp::iir].
//! * Clamping, output offset, and anti-windup. See [idsp::iir].
//! * Feedback onto a frequency offset of the modulation DDS at `mod_freq`
//! * Additional feedback path from the phase before unwrapping onto the
//!   modulation DDS phase offset with an adjustable gain `pow_gain`
//! * Adjustable DDS output amplitude and blanking on digital input
//! * Adjustable modulation attenuation `mod_att`
//! * Modulation output at Pounder OUT0
//!
//! # Telemetry
//! Data is regularly published via MQTT. See [Telemetry].
//!
//! # Streaming
//! Full-rate ADC and DAC data is available via configurable UDP data streaming.
//! See [stream]. To view and analyze noise spectra the graphical application
//! [`stabilizer-stream`](https://github.com/quartiq/stabilizer-stream) can be used.

use ad9959::Acr;
use arbitrary_int::{u14, u24};
use dsp_fixedpoint::{Const, Q32};
use dsp_process::{Process, SplitProcess};
use idsp::{
    Accu, Complex, ComplexExt, Lockin, Lowpass, LowpassState, PLL, Unwrapper,
    iir::{
        Biquad, BiquadClamp, BiquadRepr, DirectForm1, DirectForm1Dither, Pid,
    },
};
use miniconf::Tree;
use num_traits::AsPrimitive;
use platform::NetSettings;
use serde::{Deserialize, Serialize};
use stabilizer::{
    convert::{DacCode, Gain},
    statistics,
};

/// Sample and batch period configuration.
/// Note that both `SAMPLE_TICKS_LOG2` and `BATCH_SIZE_LOG2` are implicitly used in the
/// lockin harmonic computation below. Do not change them without accounting for that.
const SAMPLE_TICKS_LOG2: u32 = 7;
const BATCH_SIZE_LOG2: usize = 3;

/// ADC and DAC sample rate in timer cycles. One timer cycle at 100 MHz is 10 ns.
const SAMPLE_TICKS: u32 = 1 << SAMPLE_TICKS_LOG2; // 1.28 µs

/// ADC/DAC Samples per batch. The [app::process] routine is invoked once per batch period
/// and has access to the two (both channels) filled buffers of ADC samples from the
/// previous batch period and to the two to-be-filled buffers of DAC samples that will
/// be emitted in the next batch period.
const BATCH_SIZE: usize = 1 << BATCH_SIZE_LOG2;

// Delta FTW between the two DDS: DF
// Timestamp counter wrap period in DDS clock cycles:
// 1 << (2 (dds SYNC prescaler) + 2 (timer prescaler) + 16 (timer counter width))
// Lockin demodulation period in DDS clock cycles: (1 << 32) / DF
// Counter capture period in samples (also batch size): 1 << 3
//
// DDS clock interval t_dds = 2 ns
// Lockin period
// t_lo = 1/(f_b - f_a) = (1 << 32)*t_dds/DF
// SYNC interval
// t_sync = t_dds*psc_dds*psc_tim
// CPU timer clock interval:
// t_cpu = 10 ns
// Batch interval:
// t_batch = t_cpu*128*8
// Timestamper increment:
// dt_sync = t_batch/t_sync = t_cpu*128*8/(t_dds*4*4) = t_cpu/t_dds*64
// Sample interval
// t_sample = t_batch/n_batch = dt_sync*t_sync/n_batch = dt_sync*t_dds*2
// Sample phase increment
// dp_sample = t_sample/t_lo*(1 << 32) = dt_sync*2*DF
// Ratio between sample phase increment and timestamper increment
// harmonic_sample = dp_sample/dt_sync = DF << 1

// Scaling factor (harmonic) to convert PLL frequency to lockin LO frequency.
const MULT_SHIFT: u32 = 2 + 2 + 14 - BATCH_SIZE_LOG2 as u32;

// Phase scale for fine phase offset, such that 1 is one DDS LSB.
const PHASE_SCALE_SHIFT: u32 = 12;

// Default modulation/demodulation frequency for characterization.
// High CTZ has fewest DDS phase truncation spurs. Near 160 MHz.
const F_DEMOD: u32 = 0x5200_0000;

#[derive(Clone, Debug, Tree)]
pub struct BiquadReprTree<T, Y>
where
    T: 'static + Const + Copy,
    Y: 'static + Copy,
    f32: AsPrimitive<T> + AsPrimitive<Y>,
    BiquadClamp<T, Y>: Default,
    BiquadRepr<f32, T, Y>: Default,
    Pid<f32>: Default + Into<BiquadClamp<T, Y>>,
{
    // Order matters
    /// Biquad representation type
    #[tree(rename="typ", typ="&str", with=miniconf::str_leaf, defer=self.repr)]
    _typ: (),
    /// Biquad parameters
    /// Biquad representation subtree access
    repr: BiquadRepr<f32, T, Y>,
    /// Update trigger. TODO: Needs explicit trigger for serial-settings
    #[tree(rename="update", with=biquad_update, defer=*self)]
    _update: (),
    /// Built raw IIR
    #[tree(skip)]
    iir: BiquadClamp<T, Y>,
    #[tree(skip)]
    period: f32,
    #[tree(skip)]
    b_scale: f32,
    #[tree(skip)]
    y_scale: f32,
}

mod biquad_update {
    use super::BiquadReprTree;
    use dsp_fixedpoint::Const;
    use idsp::iir::{BiquadClamp, BiquadRepr, Pid};
    use miniconf::{Keys, SerdeError, leaf};
    pub use miniconf::{
        deny::{mut_any_by_key, ref_any_by_key},
        leaf::SCHEMA,
    };
    use num_traits::AsPrimitive;
    use serde::{Deserialize, Deserializer, Serializer};

    pub fn serialize_by_key<S, T, Y>(
        _value: &BiquadReprTree<T, Y>,
        keys: impl Keys,
        ser: S,
    ) -> Result<S::Ok, SerdeError<S::Error>>
    where
        S: Serializer,
        T: 'static + Const + Copy,
        Y: 'static + Copy,
        f32: AsPrimitive<T> + AsPrimitive<Y>,
        BiquadRepr<f32, T, Y>: Default,
        BiquadClamp<T, Y>: Default,
        Pid<f32>: Into<BiquadClamp<T, Y>>,
    {
        leaf::serialize_by_key(&(), keys, ser)
    }

    pub fn deserialize_by_key<'de, D, T, Y>(
        value: &mut BiquadReprTree<T, Y>,
        keys: impl Keys,
        de: D,
    ) -> Result<(), SerdeError<D::Error>>
    where
        D: Deserializer<'de>,
        T: 'static + Const + Copy,
        Y: 'static + Copy,
        f32: AsPrimitive<T> + AsPrimitive<Y>,
        BiquadRepr<f32, T, Y>: Default,
        BiquadClamp<T, Y>: Default,
        Pid<f32>: Into<BiquadClamp<T, Y>>,
    {
        leaf::deserialize_by_key(&mut (), keys, de)?;
        value.iir =
            value.repr.build(value.period, value.b_scale, value.y_scale);
        Ok(())
    }

    #[allow(clippy::extra_unused_type_parameters)]
    pub fn probe_by_key<'de, T, D>(
        keys: impl Keys,
        de: D,
    ) -> Result<(), SerdeError<D::Error>>
    where
        T: Deserialize<'de>,
        D: Deserializer<'de>,
    {
        leaf::probe_by_key::<'_, T, _>(keys, de)
    }
}

impl<T, Y> Default for BiquadReprTree<T, Y>
where
    T: 'static + Const + Copy,
    Y: 'static + Copy,
    f32: AsPrimitive<T> + AsPrimitive<Y>,
    BiquadClamp<T, Y>: Default,
    Pid<f32>: Into<BiquadClamp<T, Y>>,
    BiquadRepr<f32, T, Y>: Default,
{
    fn default() -> Self {
        Self {
            _typ: (),
            repr: BiquadRepr::Raw(Biquad::IDENTITY.into()),
            _update: (),
            iir: Biquad::IDENTITY.into(),
            period: 1.0,
            b_scale: 1.0,
            y_scale: 1.0,
        }
    }
}

#[derive(Clone, Debug, Deserialize, Serialize, Tree)]
struct DdsSettings {
    /// RF output (modulation) or input (demodulation) offset frequency tuning word.
    /// The DDS sample clock is nominally 500 MHz.
    ///
    /// # Value
    /// Modulation/demodulation frequency tuning word (32 bit).
    /// Range [0, 0xffff_ffff]
    ///
    /// # Default
    /// A `0x5200_0000` tuning word corresponds to close to 160 MHz.
    freq: u32,
    /// Modulation/demodulation RF attenuation.
    ///
    /// # Value
    /// Attenuation in dB, Range [0, 31.5]
    ///
    /// # Default
    /// 6 dB output attenuation, 31.5 dB input attenuation
    #[tree(with=validate_att)]
    att: f32,
    /// Modulation/demodulation phase offset.
    ///
    /// # Value
    /// Phase offset in machine units (16 bit).
    ///
    /// # Default
    /// 0
    #[tree(with=miniconf::leaf)]
    phase: u14,
}

mod validate_att {
    use miniconf::ValueError;
    pub use miniconf::{
        Keys, SerdeError,
        deny::mut_any_by_key,
        leaf::{self, SCHEMA, probe_by_key, ref_any_by_key, serialize_by_key},
    };
    use serde::Deserializer;

    pub fn deserialize_by_key<'de, D: Deserializer<'de>>(
        value: &mut f32,
        keys: impl Keys,
        de: D,
    ) -> Result<(), SerdeError<D::Error>> {
        let mut att = *value;
        leaf::deserialize_by_key(&mut att, keys, de)?;
        if stabilizer::convert::att_is_valid(att) {
            *value = att;
            Ok(())
        } else {
            Err(ValueError::Access("Attenuation out of range (0..=31.5 dB)")
                .into())
        }
    }
}

#[derive(Clone, Debug, Tree)]
struct ChannelSettings {
    /// Input (demodulation) DDS settings
    /// Feedback to stabilize the RF input phase is applied to the RF output
    /// on top of the output frequency and phase.
    ///
    /// For the demodulation this is the total (DDS **minus** Lockin, i.e. lower sideband)
    /// demodulation frequency. If the modulation AOM is passed twice at +1 order,
    /// `input/freq` should be twice `output/freq`.
    input: DdsSettings,
    /// Output (modulation) DDS settings
    output: DdsSettings,
    /// Demodulation amplitude control register.
    ///
    /// # Value
    /// AD9959 amplitude control register (24 bits, see datasheet)
    ///
    /// # Default
    /// 0 for full scale amplitude and multiplier disable
    #[tree(with=miniconf::leaf)]
    amp: u24,
    /// Lockin lowpass time constant. The lowpass is a cascade of one second order IIR
    /// filters, 12 dB/octave.
    /// This needs to be high enough to suppress the unwanted demodulation components
    /// and harmonics but as low as possible to maximize bandwidth. Many demodulation
    /// components and harmonics are also suppressed by the zeros of the batch size
    /// moving average FIR filter and judicious choice of `lockin_freq`.
    ///
    /// TODO: settle pll and lockin settings into design after confirming optimal choice
    ///
    /// # Default
    /// `lockin_k = [0x200_0000, -0x2000_0000]`
    #[tree(skip)] // TODO
    lockin_k: Lockin<Lowpass<2>>,
    /// Minimum demodulated signal power to enable feedback.
    /// Note that this is RMS and that the signal peak must not clip.
    ///
    /// # Value
    /// `log2` of the signal power relative to full scale. Range: `[-63..0]`
    ///
    /// # Default
    /// `min_power = -24` corresponding to about -69 dBFS.
    min_power: i32,
    /// Clear the phase unwrap tracking counters once.
    /// To make this setting edge-sensitive, after setting it to `true`,
    /// it must be reset to `false` by the user before setting any other settings.
    clear: bool,
    /// Scaling factor of the unwrapped phase and fine rational offset.
    /// The phase scaling is located after the phase unwrapping before the feedback
    /// IIR filter.
    ///
    /// FIXME: doc rational offset
    ///
    /// # Value
    /// `[[phase_factor, phase_shr], [time_factor, time_shr]]`
    ///
    /// # Default
    /// `phase_scale = [[1, 16], [0, 0]]`:
    /// clamped range: ±33 k turn (tracked range is ±2 G turn)
    /// quantization: 1.5 µ turn, 0.1 Hz
    #[tree(with=phase_scale, defer=*self)]
    phase_scale: [[i32; 2]; 2],
    /// Feedback IIR filter settings. The filter input is phase, the output is frequency.
    ///
    /// # Default
    /// A proportional gain=-1 filter.
    iir: BiquadReprTree<Q32<29>, i32>,
    /// Phase offset feedback gain.
    /// Phase feedback is a proportional bypass of the unwrapper, the IIR
    /// (including its input and output scaling) and the frequency feedback path.
    /// The phase offset gain is `pow_gain/(1 << 13) rad/rad`.
    ///
    /// # Value
    /// Integer scaled phase feedback gain. Range: `[-0x2000, 0x2000]`
    ///
    /// # Default
    /// 0 for no phase feedback
    pow_gain: i16,
    /// Allow digital input to hold
    hold_en: bool,
    /// Amplitude IIR filter. The filter input is squared magnitude, the output is DDS amplitude.
    ///
    /// # Default
    /// No feedback
    iir_amp: BiquadReprTree<f32, f32>,
}

const DDS_LSB_PER_HZ: f32 = (1i64 << 32) as f32
    / stabilizer::design_parameters::DDS_SYSTEM_CLK.to_Hz() as f32;

impl Default for ChannelSettings {
    fn default() -> Self {
        let mut iir_prop = BiquadClamp::from(Biquad::IDENTITY);
        iir_prop.coeff.ba[0] *= -1;
        iir_prop.min = -0x4_0000;
        iir_prop.max = 0x4_0000;
        let mut iir_amp = BiquadClamp::default();
        iir_amp.u = 0x3ff as _;
        iir_amp.min = 0.0;
        iir_amp.max = 0x3ff as _;
        let mut s = Self {
            input: DdsSettings {
                freq: F_DEMOD,
                att: 31.5,
                phase: u14::new(0),
            },
            output: DdsSettings {
                freq: F_DEMOD,
                att: 6.0,
                phase: u14::new(0),
            },
            lockin_k: Lockin(Lowpass([-(i32::MIN >> 6), i32::MIN >> 2])),
            amp: u24::new(0),
            min_power: -24,
            clear: true,
            phase_scale: [[1, 16], [0, 0]],
            iir: BiquadReprTree {
                repr: BiquadRepr::Raw(iir_prop.clone()),
                iir: iir_prop.clone(),
                period: stabilizer::design_parameters::TIMER_PERIOD
                    * (SAMPLE_TICKS * BATCH_SIZE as u32) as f32,
                y_scale: DDS_LSB_PER_HZ,
                ..Default::default()
            },
            pow_gain: 0,
            hold_en: false,
            iir_amp: BiquadReprTree {
                repr: BiquadRepr::Raw(iir_amp.clone()),
                iir: iir_amp.clone(),
                period: 10e-3,
                b_scale: iir_amp.max,
                y_scale: iir_amp.max,
                ..Default::default()
            },
        };
        s.update_phase_scale();
        s
    }
}

mod phase_scale {
    use super::ChannelSettings;
    pub use miniconf::{
        Keys, SerdeError,
        deny::{mut_any_by_key, ref_any_by_key},
        leaf::{self, SCHEMA},
    };
    use serde::{Deserialize, Deserializer, Serializer};

    pub fn serialize_by_key<S: Serializer>(
        value: &ChannelSettings,
        keys: impl Keys,
        ser: S,
    ) -> Result<S::Ok, SerdeError<S::Error>> {
        leaf::serialize_by_key(&value.phase_scale, keys, ser)
    }

    pub fn deserialize_by_key<'de, D: Deserializer<'de>>(
        value: &mut ChannelSettings,
        keys: impl Keys,
        de: D,
    ) -> Result<(), SerdeError<D::Error>> {
        leaf::deserialize_by_key(&mut value.phase_scale, keys, de)?;
        value.update_phase_scale();
        Ok(())
    }

    pub fn probe_by_key<'de, T: Deserialize<'de>, D: Deserializer<'de>>(
        keys: impl Keys,
        de: D,
    ) -> Result<(), SerdeError<D::Error>> {
        leaf::probe_by_key::<'de, T, _>(keys, de)
    }
}

impl ChannelSettings {
    fn update_phase_scale(&mut self) {
        // Units: [x] = turns, [y] = Hz
        // TODO: verify
        let phase_lsb_per_turn =
            (self.phase_scale[0][0] << (32 - self.phase_scale[0][1])) as f32;
        self.iir.b_scale = DDS_LSB_PER_HZ / phase_lsb_per_turn;
    }
}

/// Settings structure for the application.
/// All fields in this structure are available through MQTT and can be configured at runtime.
#[derive(Clone, Debug, Tree)]
pub struct Fls {
    /// Channel-specific settings.
    ch: [ChannelSettings; 2],
    /// External reference
    ///
    /// # Value
    /// `true` for external 100 MHz reference input selected,
    /// `false` for internal 100 MHz XO enabled and selected
    ///
    /// # Default
    /// `false`
    ext_clk: bool,
    /// Lockin local oscillator frequency tuning word. Common to both demodulation/input
    /// channels.
    ///
    /// The demodulation DDS frequency tuning word
    /// is `/ch/+/input/freq + lockin_freq*0x8000` (lower sideband).
    ///
    /// TODO: settle pll and lockin settings into design after confirming optimal choice
    ///
    /// # Default
    /// `0x40` corresponding to 244 kHz. 5/8 Nyquist.
    lockin_freq: u32,
    /// Lockin demodulation oscillator PLL bandwidth.
    /// This PLL reconstructs the DDS SYNC clock output on the CPU clock timescale.
    ///
    /// TODO: settle pll and lockin settings into design after confirming optimal choice
    ///
    /// # Default
    /// `/pll_k = 0x4_0000` corresponds to to a time constant of about 0.4 s.
    pll_k: i32,
    /// Telemetry output period in seconds
    ///
    /// # Default
    /// 2 second interval
    telemetry_period: u16,
    /// Target for data streaming
    ///
    /// # Default
    /// Streaming disabled
    #[tree(with=miniconf::leaf)]
    stream: stream::Target,
}

impl Default for Fls {
    fn default() -> Self {
        Self {
            ch: Default::default(),
            ext_clk: false,
            lockin_freq: 0x40,
            pll_k: 0x4_0000,
            telemetry_period: 10,
            stream: Default::default(),
        }
    }
}

#[derive(Clone, Debug, Tree, Default)]
pub struct Settings {
    pub fls: Fls,

    pub net: NetSettings,
}

impl platform::AppSettings for Settings {
    fn new(net: NetSettings) -> Self {
        Self {
            net,
            fls: Default::default(),
        }
    }

    fn net(&self) -> &NetSettings {
        &self.net
    }
}

impl serial_settings::Settings for Settings {
    fn reset(&mut self) {
        *self = Self {
            fls: Default::default(),
            net: NetSettings::new(self.net.mac),
        }
    }
}

/// Stream data format.
#[derive(
    Clone, Copy, Debug, Default, Serialize, bytemuck::Zeroable, bytemuck::Pod,
)]
#[repr(C)]
struct Stream {
    /// Demodulated signal.  `-1 << 31` corresponds to negative full scale.
    demod: Complex<i32>,
    /// Current number of phase wraps. In units of turns.
    phase: [i32; 2],
    /// Current frequency tuning word added to the configured modulation
    /// offset `mod_freq`.
    delta_ftw: i32,
    /// Current phase offset word applied to the modulation DDS.
    delta_pow: i16,
    /// Modulation DDS amplitude word
    mod_amp: u16,
    /// PLL time
    pll: u32,
}

/// Channel Telemetry
#[derive(Default, Clone, Serialize)]
struct ChannelTelemetry {
    /// Current phase. Offset and scaled.
    phase: i64,
    /// Power estimate, `|demod|²` re full scale.
    power_log: i32,
    ///
    // power: i32,
    /// Auxiliary front panel ADC input values, undersmpled
    aux_adc: f32,
    mod_amp: u16,
    /// Number of sampler where digital input signal was high.
    holds: u32,
    /// Number of potential phase slips where the absolute
    /// phase difference between successive samples is larger than π/2.
    slips: u32,
    /// Counter for the number of samples with low power.
    blanks: u32,
}

#[derive(Default, Clone)]
pub struct Telemetry {
    pll_time: i64,
    ch: [ChannelTelemetry; 2],
    stats: [statistics::State; 2],
}

/// Telemetry structure.
/// This structure is published via MQTT at the `telemetry_interval` configured in
/// [Settings].
/// There is no dedicated AA filtering for telemetry data (except for `stats`),
/// it is just decimated by the telemetry interval. Use streaming for full
/// bandwidth data.
#[derive(Default, Clone, Serialize)]
pub struct CookedTelemetry {
    /// PLL time
    /// DDS PLL time as seen by CPU (sample) clock.
    /// Settles increments of approximately `0x140_0000`.
    pll_time: i64,
    /// Statistics of scaled (settings.phase_scale) phase including wraps.
    /// Phase statistics state. Each message corresponds to the statistics of the
    /// phase data since the last message.
    phase: [statistics::ScaledStatistics; 2],
    /// RF power in dBm as reported by the RF detector and ADC. Functionality
    /// limited. <https://github.com/sinara-hw/Pounder/issues/95>
    rf_power: [f32; 2],
    /// Raw (binary) channel telemetry, mostly "stateful"
    raw: [ChannelTelemetry; 2],
    /// Channel frequency estimate (PI counter between telemetry messages)
    /// TODO: deprecate
    ch_freq: [f64; 2],
    /// Pounder board temperature
    temp: f32,
}

#[derive(Clone, Default)]
pub struct ChannelState {
    lockin: [LowpassState<2>; 2],
    x0: i32,
    t0: i32,
    t: i64,
    y: i64,
    unwrapper: Unwrapper<i64>,
    iir: DirectForm1Dither,
    iir_amp: DirectForm1<f32>,
    hold: bool,
}

#[cfg(not(target_os = "none"))]
fn main() {
    use miniconf::{json::to_json_value, json_schema::TreeJsonSchema};
    let s = Settings::default();
    println!(
        "{}",
        serde_json::to_string_pretty(&to_json_value(&s).unwrap()).unwrap()
    );
    let mut schema = TreeJsonSchema::new(Some(&s)).unwrap();
    schema
        .root
        .insert("title".to_string(), "Stabilizer fls".into());
    println!("{}", serde_json::to_string_pretty(&schema.root).unwrap());
}

#[cfg(target_os = "none")]
#[cfg_attr(target_os = "none", rtic::app(device = stabilizer::hardware::hal::stm32, peripherals = true, dispatchers=[DCMI, JPEG, LTDC, SDMMC]))]
mod app {
    use arbitrary_int::u10;
    use core::sync::atomic::{Ordering, fence};
    use fugit::ExtU32 as _;
    use rtic_monotonics::Monotonic;

    use stabilizer::hardware::{
        self,
        DigitalInput0,
        DigitalInput1,
        SerialTerminal,
        SystemTimer,
        Systick,
        UsbDevice,
        adc::{Adc0Input, Adc1Input},
        dac::{Dac0Output, Dac1Output},
        hal,
        net::{NetworkState, NetworkUsers},
        // afe::Gain,
        pounder::{
            Channel, PounderDevices, dds_output::DdsOutput,
            timestamp::Timestamper,
        },
        timers::SamplingTimer,
    };

    use stream::FrameGenerator;

    use super::*;

    #[shared]
    struct Shared {
        usb: UsbDevice,
        network: NetworkUsers<Fls>,
        active_settings: Fls,
        settings: Settings,
        telemetry: Telemetry,
        dds_output: DdsOutput,
        pounder: PounderDevices,
        state: [ChannelState; 2],
    }

    #[local]
    struct Local {
        usb_terminal: SerialTerminal<Settings>,
        sampling_timer: SamplingTimer,
        digital_inputs: (DigitalInput0, DigitalInput1),
        adcs: (Adc0Input, Adc1Input),
        dacs: (Dac0Output, Dac1Output),
        generator: FrameGenerator,
        timestamper: Timestamper,
        stream: [Stream; 2],
        tele_state: [i64; 3],
        pll: PLL,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local) {
        let clock = SystemTimer::new(|| Systick::now().ticks());

        // Configure the microcontroller
        let (mut carrier, mezzanine, _eem) = hardware::setup::setup::<Settings>(
            c.core,
            c.device,
            clock,
            BATCH_SIZE,
            SAMPLE_TICKS,
        );

        let mut network = NetworkUsers::new(
            carrier.network_devices.stack,
            carrier.network_devices.phy,
            clock,
            env!("CARGO_BIN_NAME"),
            &carrier.settings.net,
            carrier.metadata,
        );

        let generator = network.configure_streaming(stream::Format::Fls);

        // ADC0 full scale 5V
        carrier.afes[0].set_gain(Gain::G2);
        carrier.afes[1].set_gain(Gain::G2);

        let hardware::setup::Mezzanine::Pounder(mut pounder) = mezzanine else {
            panic!("Missing Pounder Mezzanine");
        };
        pounder.timestamper.start();

        // Enable ADC/DAC events
        carrier.adcs.0.start();
        carrier.adcs.1.start();
        carrier.dacs.0.start();
        carrier.dacs.1.start();

        let shared = Shared {
            usb: carrier.usb,
            network,
            telemetry: Telemetry::default(),
            active_settings: carrier.settings.fls.clone(),
            settings: carrier.settings,
            dds_output: pounder.dds_output,
            pounder: pounder.pounder,
            state: Default::default(),
        };

        let local = Local {
            usb_terminal: carrier.usb_serial,
            sampling_timer: carrier.sampling_timer,
            digital_inputs: carrier.digital_inputs,
            adcs: carrier.adcs,
            dacs: carrier.dacs,
            generator,
            timestamper: pounder.timestamper,
            stream: Default::default(),
            tele_state: [0; 3],
            pll: PLL::default(),
        };

        settings_update::spawn().unwrap();
        telemetry::spawn().unwrap();
        aux_adc::spawn().unwrap();
        usb::spawn().unwrap();
        ethernet_link::spawn().unwrap();
        start::spawn().unwrap();

        (shared, local)
    }

    #[task(priority = 1, local = [sampling_timer])]
    async fn start(c: start::Context) {
        Systick::delay(200.millis()).await;
        c.local.sampling_timer.start();
    }

    /// Main DSP processing routine.
    ///
    /// See `dual-iir` for general notes on processing time and timing.
    ///
    /// This is an implementation of fiber length stabilization using super-heterodyne
    /// (pounder + lockin) and digital feedback to a DDS.
    #[task(binds = DMA1_STR4, local=[timestamper, adcs, dacs, generator, digital_inputs, stream, pll], shared = [active_settings, state, telemetry, dds_output], priority = 3)]
    #[unsafe(link_section = ".itcm.process")]
    fn process(c: process::Context) {
        let process::LocalResources {
            adcs: (adc0, adc1),
            dacs: (dac0, dac1),
            digital_inputs,
            timestamper,
            generator,
            stream,
            pll,
            ..
        } = c.local;

        // A counter running at a fourth of the DDS SYNC interval is captured by
        // the overflow of a timer synchronized to the sampling timer (locked to the
        // CPU clock and the other CPU timer clocks).
        // Captured timestamps are about 0x140 counts apart between batches.
        // They determine the phase and period of the DDS clock (driving the counter)
        // in terms of the CPU clock (driving the capture).
        // Discard double captures (overcaptures) and extrapolate.
        // Extrapolate on no capture (undercapture).
        let timestamp = timestamper
            .latest_timestamp()
            .unwrap_or(None)
            .map(|t| ((t as u32) << 16) as i32);

        (
            c.shared.state,
            c.shared.active_settings,
            c.shared.dds_output,
            c.shared.telemetry,
        )
            .lock(|state, settings, dds_output, telemetry| {
                // Reconstruct frequency and phase using a lowpass that is aware of phase and frequency
                // wraps.
                settings.pll_k.process(pll, timestamp);
                // TODO: implement clear
                stream[0].pll = pll.frequency() as _;
                stream[1].pll = pll.phase() as _;
                telemetry.pll_time =
                    telemetry.pll_time.wrapping_add(pll.frequency() as _);

                let mut demod = [Complex::<i32>::default(); BATCH_SIZE];
                // TODO: fixed lockin_freq, const 5/16 frequency table (80 entries), then rotate each by pll phase
                for (d, p) in demod.iter_mut().zip(Accu::new(
                    (pll.phase() << BATCH_SIZE_LOG2)
                        .wrapping_mul(settings.lockin_freq as _),
                    pll.frequency().wrapping_mul(settings.lockin_freq as _),
                )) {
                    *d = Complex::from_angle(p);
                }

                (adc0, adc1, dac0, dac1).lock(|adc0, adc1, dac0, dac1| {
                    fence(Ordering::SeqCst);
                    let adc: [&[u16; BATCH_SIZE]; 2] = [
                        (**adc0).try_into().unwrap(),
                        (**adc1).try_into().unwrap(),
                    ];
                    let dac: [&mut [u16; BATCH_SIZE]; 2] = [
                        (*dac0).try_into().unwrap(),
                        (*dac1).try_into().unwrap(),
                    ];
                    // Perform lockin demodulation of the ADC samples in the batch.
                    for ((((adc, dac), state), settings), stream) in adc
                        .into_iter()
                        .zip(dac.into_iter())
                        .zip(state.iter_mut())
                        .zip(settings.ch.iter())
                        .zip(stream.iter_mut())
                    {
                        stream.demod = adc
                            .iter()
                            .zip(dac.iter_mut())
                            .zip(demod.iter())
                            .map(|((a, d), p)| {
                                // Demodulate the ADC sample `a0` with the sample's phase `p` and
                                // filter it with the lowpass.
                                // zero(s) at fs/2 (Nyquist) by lowpass
                                let y = settings.lockin_k.process(
                                    &mut state.lockin,
                                    // 3 bit headroom for coeff sum minus one bit gain for filter
                                    ((*a as i16 as i32) << 14, *p),
                                );
                                // Convert quadrature demodulated output to DAC data for monitoring
                                *d = DacCode::from((y.im() >> 13) as i16).0;
                                y
                            })
                            // Add more zeros at fs/2, fs/4, and fs/8 by rectangular window.
                            // Sum up all demodulated samples in the batch. Corresponds to a boxcar
                            // averager with sinc frequency response. The first 15 lockin harmonics end up
                            // in zeros of the filter.
                            .sum();
                    }
                    fence(Ordering::SeqCst);
                });
                let di =
                    [digital_inputs.0.is_high(), digital_inputs.1.is_high()];
                // TODO: pll.frequency()?
                let time = pll.phase() & (-1 << PHASE_SCALE_SHIFT);
                let dtime = time.wrapping_sub(state[0].t0) >> PHASE_SCALE_SHIFT;
                state[0].t0 = time;
                state[1].t0 = time;

                let mut builder = dds_output.builder();
                for (
                    (((((idx, di), settings), state), telemetry), stream),
                    stats,
                ) in [Channel::Out0, Channel::Out1]
                    .into_iter()
                    .zip(di)
                    .zip(settings.ch.iter_mut())
                    .zip(state.iter_mut())
                    .zip(telemetry.ch.iter_mut())
                    .zip(stream.iter_mut())
                    .zip(telemetry.stats.iter_mut())
                {
                    state.hold = settings.hold_en && di;
                    if state.hold {
                        telemetry.holds = telemetry.holds.wrapping_add(1);
                    }

                    if settings.clear {
                        state.unwrapper = Unwrapper::default();
                        state.t = 0;
                        state.y = 0;
                        settings.clear = false;
                    }

                    let power = stream.demod.log2();
                    telemetry.power_log = power;
                    let blank = power < settings.min_power;

                    if blank {
                        telemetry.blanks = telemetry.blanks.wrapping_add(1);
                    }

                    // Perform unwrapping, phase scaling, IIR filtering and FTW scaling.
                    let (delta_ftw, delta_pow) = if blank || state.hold {
                        // TODO: Unclear whether feeding zero error into the IIR or holding its output
                        // is more correct. Also unclear what the frequency and phase should do.
                        // telemetry.ch[0].dphase = 0;
                        (0, stream.delta_pow)
                    } else {
                        let phase = stream.demod.arg();
                        let dphase = phase.wrapping_sub(state.x0);
                        state.x0 = phase;

                        // |dphi| > pi/2 indicates a possible undetected phase slip.
                        // Neither a necessary nor a sufficient condition though.
                        if dphase.wrapping_add(1 << 30) < 0 {
                            telemetry.slips = telemetry.slips.wrapping_add(1);
                        }

                        // Scale, offset and unwrap phase
                        state.t = state.t.wrapping_add(
                            dtime as i64 * settings.phase_scale[1][0] as i64,
                        );
                        state.y = state.y.wrapping_add(
                            dphase as i64 * settings.phase_scale[0][0] as i64,
                        );
                        state.unwrapper.process(
                            ((state.y >> settings.phase_scale[0][1]) as i32)
                                .wrapping_add(
                                    (state.t >> settings.phase_scale[1][1])
                                        as i32,
                                ),
                        );

                        stream.phase = bytemuck::cast(state.unwrapper.y);
                        telemetry.phase = state.unwrapper.y;
                        let phase_err = state
                            .unwrapper
                            .y
                            .clamp(-i32::MAX as _, i32::MAX as _)
                            as _;

                        stats.update(phase_err);
                        // TODO; unchecked_shr
                        let delta_ftw =
                            settings.iir.iir.process(&mut state.iir, phase_err);
                        let delta_pow = ((phase_err >> 16)
                            .wrapping_mul(settings.pow_gain as _)
                            >> 16) as _;
                        (delta_ftw, delta_pow)
                    };
                    stream.delta_ftw = delta_ftw;
                    stream.delta_pow = delta_pow; // note the u14 wrap below

                    // let power = (((stream.demod.re as i64).pow(2)
                    //     + (stream.demod.im as i64).pow(2))
                    //     >> 32) as i32;

                    builder.push(
                        idx.into(),
                        Some(settings.output.freq.wrapping_add(delta_ftw as _)),
                        Some(u14::new(
                            settings
                                .output
                                .phase
                                .value()
                                .wrapping_add(delta_pow as _)
                                & u14::MASK,
                        )),
                        Some(
                            Acr::DEFAULT
                                .with_asf(u10::new(
                                    telemetry.mod_amp.clamp(0, 0x3ff),
                                ))
                                .with_multiplier(true),
                        ),
                    );
                }
                dds_output.write(builder);

                const N: usize = core::mem::size_of::<[Stream; 2]>();
                generator.add(|buf| {
                    buf[..N].copy_from_slice(bytemuck::cast_slice(stream));
                    N
                });
            });
    }

    #[idle(shared=[network, usb, settings])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            match (&mut c.shared.network, &mut c.shared.settings)
                .lock(|net, settings| net.update(&mut settings.fls))
            {
                NetworkState::SettingsChanged => {
                    settings_update::spawn().unwrap()
                }
                NetworkState::Updated => {}
                NetworkState::NoChange => {
                    // We can't sleep if USB is not in suspend.
                    if c.shared.usb.lock(|usb| {
                        usb.state()
                            == usb_device::device::UsbDeviceState::Suspend
                    }) {
                        cortex_m::asm::wfi();
                    }
                }
            }
        }
    }

    #[task(priority = 1, shared=[network, settings, dds_output, pounder, active_settings])]
    async fn settings_update(mut c: settings_update::Context) {
        c.shared.settings.lock(|settings| {
            c.shared.pounder.lock(|p| {
                for (ch, att) in [
                    (Channel::In0, settings.fls.ch[0].input.att),
                    (Channel::Out0, settings.fls.ch[0].output.att),
                    (Channel::In1, settings.fls.ch[1].input.att),
                    (Channel::Out1, settings.fls.ch[1].output.att),
                ]
                .into_iter()
                {
                    p.set_attenuation(ch, att).unwrap();
                }
                p.set_ext_clk(settings.fls.ext_clk).unwrap();
            });

            c.shared.dds_output.lock(|dds_output| {
                let mut builder = dds_output.builder();
                builder.push(
                    Channel::In0.into(),
                    Some(
                        settings.fls.ch[0].input.freq.wrapping_add(
                            settings.fls.lockin_freq << MULT_SHIFT,
                        ),
                    ),
                    Some(settings.fls.ch[0].input.phase),
                    Some(Acr::new_with_raw_value(settings.fls.ch[0].amp)),
                );
                builder.push(
                    Channel::In1.into(),
                    Some(
                        settings.fls.ch[1].input.freq.wrapping_add(
                            settings.fls.lockin_freq << MULT_SHIFT,
                        ),
                    ),
                    Some(settings.fls.ch[1].input.phase),
                    Some(Acr::new_with_raw_value(settings.fls.ch[1].amp)),
                );
                dds_output.write(builder);
            });
            c.shared
                .network
                .lock(|net| net.direct_stream(settings.fls.stream));
            c.shared
                .active_settings
                .lock(|current| *current = settings.fls.clone());
        });
    }

    #[task(priority = 1, shared=[pounder, telemetry, settings, state])]
    async fn aux_adc(mut c: aux_adc::Context) -> ! {
        loop {
            let aux_adc::SharedResources {
                settings,
                state,
                telemetry,
                pounder,
                ..
            } = &mut c.shared;
            let x = pounder.lock(|p| {
                [
                    p.sample_aux_adc(Channel::In0).unwrap(),
                    p.sample_aux_adc(Channel::In1).unwrap(),
                ]
            });
            let mut y = [0; 2];
            (settings, state).lock(|s, c| {
                for (((s, c), x), y) in s
                    .fls
                    .ch
                    .iter()
                    .zip(c.iter_mut())
                    .zip(x.iter())
                    .zip(y.iter_mut())
                {
                    *y = if c.hold {
                        Biquad::<f32>::HOLD.process(&mut c.iir_amp, *x);
                        0
                    } else {
                        s.iir_amp.iir.process(&mut c.iir_amp, *x) as u16
                    };
                }
            });
            telemetry.lock(|t| {
                t.ch[0].aux_adc = x[0];
                t.ch[0].mod_amp = y[0];
                t.ch[1].aux_adc = x[1];
                t.ch[1].mod_amp = y[1];
            });
            Systick::delay(10.millis()).await;
        }
    }

    #[task(priority = 1, local=[tele_state], shared=[network, settings, telemetry, pounder])]
    async fn telemetry(mut c: telemetry::Context) -> ! {
        loop {
            let (raw, stats) = c
                .shared
                .telemetry
                .lock(|t| (t.clone(), core::mem::take(&mut t.stats)));

            let (phase_scale, freq) = c.shared.settings.lock(|s| {
                (
                    [s.fls.ch[0].phase_scale, s.fls.ch[1].phase_scale],
                    [s.fls.ch[0].input.freq, s.fls.ch[1].input.freq],
                )
            });
            let scales = [
                [
                    (1i64 << phase_scale[0][0][1]) as f64
                        / phase_scale[0][0][0] as f64,
                    phase_scale[0][1][0] as f64
                        / ((1 << PHASE_SCALE_SHIFT) as f64
                            * (1i64 << phase_scale[0][1][1]) as f64),
                ],
                [
                    (1i64 << phase_scale[1][0][1]) as f64
                        / phase_scale[1][0][0] as f64,
                    phase_scale[1][1][0] as f64
                        / ((1 << PHASE_SCALE_SHIFT) as f64
                            * (1i64 << phase_scale[1][1][1]) as f64),
                ],
            ];
            const FDDS: f64 = 1.0 / DDS_LSB_PER_HZ as f64;
            const FLI: f64 = 0x1000 as f64 * FDDS;
            let pll_tau =
                1.0 / raw.pll_time.wrapping_sub(c.local.tele_state[2]) as f64;
            *c.local.tele_state =
                [raw.ch[0].phase, raw.ch[1].phase, raw.pll_time];

            let mut tele = CookedTelemetry {
                pll_time: raw.pll_time,
                phase: [
                    stats[0]
                        .get_scaled(scales[0][0] as f32 / (1i64 << 32) as f32),
                    stats[1]
                        .get_scaled(scales[1][0] as f32 / (1i64 << 32) as f32),
                ],
                ch_freq: [
                    freq[0] as f64 * FDDS
                        + (raw.ch[0].phase.wrapping_sub(c.local.tele_state[0])
                            as f64
                            * pll_tau
                            - scales[0][1])
                            * scales[0][0]
                            * FLI,
                    freq[1] as f64 * FDDS
                        + (raw.ch[1].phase.wrapping_sub(c.local.tele_state[1])
                            as f64
                            * pll_tau
                            - scales[1][1])
                            * scales[1][0]
                            * FLI,
                ],
                raw: raw.ch,
                ..Default::default()
            };

            c.shared.pounder.lock(|p| {
                tele.rf_power = [
                    p.measure_power(Channel::In0).unwrap(),
                    p.measure_power(Channel::In1).unwrap(),
                ];
                tele.temp = p.temperature().unwrap();
            });
            c.shared.network.lock(|net| {
                net.telemetry.publish_telemetry("/telemetry", &tele)
            });

            let telemetry_period =
                c.shared.settings.lock(|s| s.fls.telemetry_period);
            // Schedule the telemetry task in the future.
            Systick::delay((telemetry_period as u32).secs()).await;
        }
    }

    #[task(priority = 1, shared=[usb, settings], local=[usb_terminal])]
    async fn usb(mut c: usb::Context) -> ! {
        loop {
            // Handle the USB serial terminal.
            c.shared.usb.lock(|usb| {
                usb.poll(&mut [c
                    .local
                    .usb_terminal
                    .interface_mut()
                    .inner_mut()]);
            });

            c.shared
                .settings
                .lock(|settings| c.local.usb_terminal.poll(settings).unwrap());

            // Schedule to run this task every 10 milliseconds.
            Systick::delay(10.millis()).await;
        }
    }

    #[task(priority = 1, shared=[network])]
    async fn ethernet_link(mut c: ethernet_link::Context) -> ! {
        loop {
            c.shared.network.lock(|net| net.processor.handle_link());
            Systick::delay(1.secs()).await;
        }
    }

    #[task(binds = ETH, priority = 1)]
    fn eth(_: eth::Context) {
        unsafe { hal::ethernet::interrupt_handler() }
    }
}
