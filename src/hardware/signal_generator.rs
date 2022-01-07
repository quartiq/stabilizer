use crate::{
    hardware::dac::DacCode, hardware::design_parameters::TIMER_FREQUENCY,
};
use core::convert::TryFrom;
use miniconf::Miniconf;
use serde::{Deserialize, Serialize};

/// Types of signals that can be generated.
#[derive(Copy, Clone, Debug, Deserialize, Serialize, Miniconf)]
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
    /// symmetry is the duty cycle.
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
    /// The provided symmetry is out of range.
    InvalidSymmetry,
    /// The provided frequency is out of range.
    InvalidFrequency,
}

impl BasicConfig {
    /// Convert configuration into signal generator values.
    ///
    /// # Args
    /// * `sample_ticks_log2` - The logarithm of the number of timer sample ticks between each
    /// sample.
    pub fn try_into_config(
        self,
        sample_ticks_log2: u8,
    ) -> Result<Config, Error> {
        let symmetry_complement = 1.0 - self.symmetry;
        // Validate symmetry
        if self.symmetry < 0.0 || symmetry_complement < 0.0 {
            return Err(Error::InvalidSymmetry);
        }

        let lsb_per_hertz: f32 = (1u64 << (31 + sample_ticks_log2)) as f32
            / (TIMER_FREQUENCY.0 * 1_000_000) as f32;
        let ftw = self.frequency * lsb_per_hertz;

        // Validate base frequency tuning word to be below Nyquist.
        const NYQUIST: f32 = (1u32 << 31) as _;
        if ftw < 0.0 || 2.0 * ftw > NYQUIST {
            return Err(Error::InvalidFrequency);
        }

        // Calculate the frequency tuning words.
        // Clip both frequency tuning words to within Nyquist before rounding.
        let frequency_tuning_word = [
            if self.symmetry * NYQUIST > ftw {
                ftw / self.symmetry
            } else {
                NYQUIST
            } as i32,
            if symmetry_complement * NYQUIST > ftw {
                ftw / symmetry_complement
            } else {
                NYQUIST
            } as i32,
        ];

        Ok(Config {
            amplitude: DacCode::try_from(self.amplitude)
                .or(Err(Error::InvalidAmplitude))?
                .into(),
            signal: self.signal,
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
    pub frequency_tuning_word: [i32; 2],
}

impl Default for Config {
    fn default() -> Self {
        Self {
            signal: Signal::Cosine,
            amplitude: 0,
            frequency_tuning_word: [0, 0],
        }
    }
}

#[derive(Debug, Default)]
pub struct SignalGenerator {
    phase_accumulator: i32,
    config: Config,
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
        let sign = self.phase_accumulator.is_negative();
        self.phase_accumulator = self
            .phase_accumulator
            .wrapping_add(self.config.frequency_tuning_word[sign as usize]);

        let scale = match self.config.signal {
            Signal::Cosine => (idsp::cossin(self.phase_accumulator).0 >> 16),
            Signal::Square => {
                if sign {
                    -1 << 15
                } else {
                    1 << 15
                }
            }
            Signal::Triangle => {
                (self.phase_accumulator >> 15).abs() - (1 << 15)
            }
        };

        // Calculate the final output result as an i16.
        Some(((self.config.amplitude as i32 * scale) >> 15) as _)
    }
}
