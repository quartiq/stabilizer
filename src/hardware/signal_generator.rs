use crate::{
    configuration::ADC_SAMPLE_TICKS_LOG2, hardware::dac::DacCode,
    hardware::design_parameters::TIMER_FREQUENCY,
};
use core::convert::{TryFrom, TryInto};
use miniconf::Miniconf;
use serde::Deserialize;

/// Types of signals that can be generated.
#[derive(Copy, Clone, Debug, Deserialize, Miniconf)]
pub enum Signal {
    Cosine,
    Square,
    Triangle,
}

/// Basic configuration for a generated signal.
///
/// # Miniconf
/// `{"signal": <signal>, "frequency", 1000.0, "symmetry": 0.5, "amplitude": 1.0}`
///
/// Where `<signal>` may be any of [Signal] variants, `frequency` specifies the signal frequency
/// in Hertz, `symmetry` specifies the normalized signal symmetry which ranges from 0 - 1.0, and
/// `amplitude` specifies the signal amplitude in Volts.
#[derive(Copy, Clone, Debug, Miniconf, Deserialize)]
pub struct BasicConfig {
    /// The signal type that should be generated. See [Signal] variants.
    pub signal: Signal,

    /// The frequency of the generated signal in Hertz.
    pub frequency: f32,

    /// The normalized symmetry of the signal. At 0% symmetry, the duration of the first half oscillation is minimal.
    /// At 25% symmetry, the first half oscillation lasts for 25% of the signal period. For square wave output this
    //// symmetry is the duty cycle.
    pub symmetry: f32,

    /// The amplitude of the output signal in volts.
    pub amplitude: f32,
}

impl Default for BasicConfig {
    fn default() -> Self {
        Self {
            frequency: 1.0e3,
            symmetry: 0.5,
            signal: Signal::Cosine,
            amplitude: 0.0,
        }
    }
}

/// Represents the errors that can occur when attempting to configure the signal generator.
#[derive(Copy, Clone, Debug)]
pub enum Error {
    /// The provided amplitude is out-of-range.
    InvalidAmplitude,
}

impl TryFrom<BasicConfig> for Config {
    type Error = Error;

    fn try_from(config: BasicConfig) -> Result<Config, Error> {
        // Calculate the frequency tuning words
        let frequency_tuning_word: [u32; 2] = {
            const LSB_PER_HERTZ: f32 = (1u64 << (31 + ADC_SAMPLE_TICKS_LOG2))
                as f32
                / (TIMER_FREQUENCY.0 * 1_000_000) as f32;
            let ftw = config.frequency * LSB_PER_HERTZ;

            if config.symmetry <= ftw / u32::MAX as f32 {
                [1u32 << 31, ftw as u32]
            } else if 1. - config.symmetry <= ftw / u32::MAX as f32 {
                [ftw as u32, 1u32 << 31]
            } else {
                [
                    (ftw / config.symmetry) as u32,
                    (ftw / (1.0 - config.symmetry)) as u32,
                ]
            }
        };

        Ok(Config {
            amplitude: DacCode::try_from(config.amplitude)
                .or(Err(Error::InvalidAmplitude))?
                .into(),
            signal: config.signal,
            frequency_tuning_word,
        })
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Config {
    /// The type of signal being generated
    pub signal: Signal,

    /// The full-scale output code of the signal
    pub amplitude: i16,

    /// The frequency tuning word of the signal. Phase is incremented by this amount
    pub frequency_tuning_word: [u32; 2],
}

#[derive(Debug)]
pub struct SignalGenerator {
    phase_accumulator: u32,
    config: Config,
}

impl Default for SignalGenerator {
    fn default() -> Self {
        Self {
            config: BasicConfig::default().try_into().unwrap(),
            phase_accumulator: 0,
        }
    }
}

impl SignalGenerator {
    /// Construct a new signal generator with some specific config.
    ///
    /// # Args
    /// * `config` - The config to use for generating signals.
    ///
    /// # Returns
    /// The generator
    pub fn new(config: Config) -> Self {
        Self {
            config,
            phase_accumulator: 0,
        }
    }

    /// Update waveform generation settings.
    pub fn update_waveform(&mut self, new_config: Config) {
        self.config = new_config;
    }
}

impl core::iter::Iterator for SignalGenerator {
    type Item = i16;

    /// Get the next value in the generator sequence.
    fn next(&mut self) -> Option<i16> {
        self.phase_accumulator = self.phase_accumulator.wrapping_add(
            if (self.phase_accumulator as i32).is_negative() {
                self.config.frequency_tuning_word[0]
            } else {
                self.config.frequency_tuning_word[1]
            },
        );

        let phase = self.phase_accumulator as i32;

        let amplitude: i16 = match self.config.signal {
            Signal::Cosine => (dsp::cossin(phase).0 >> 16) as i16,
            Signal::Square => {
                if phase.is_negative() {
                    i16::MAX
                } else {
                    -i16::MAX
                }
            }
            Signal::Triangle => i16::MAX - (phase.abs() >> 15) as i16,
        };

        // Calculate the final output result as an i16.
        let result = amplitude as i32 * self.config.amplitude as i32;

        // Note: We downshift by 15-bits to preserve only one of the sign bits.
        Some(((result + (1 << 14)) >> 15) as i16)
    }
}
