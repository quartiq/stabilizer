use miniconf::Miniconf;
use rand_core::{RngCore, SeedableRng};
use rand_xorshift::XorShiftRng;
use serde::{Deserialize, Serialize};

/// Types of signals that can be generated.
#[derive(Copy, Clone, Debug, Deserialize, Serialize, Miniconf)]
pub enum Signal {
    Cosine,
    Square,
    Triangle,
    WhiteNoise,
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

    /// The phase of the output signal in turns.
    pub phase: f32,
}

impl Default for BasicConfig {
    fn default() -> Self {
        Self {
            frequency: 1.0e3,
            symmetry: 0.5,
            signal: Signal::Cosine,
            amplitude: 0.0,
            phase: 0.0,
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
    /// * `sample_period` - The time in seconds between samples.
    /// * `full_scale` - The full scale output voltage.
    pub fn try_into_config(
        self,
        sample_period: f32,
        full_scale: f32,
    ) -> Result<Config, Error> {
        let symmetry_complement = 1.0 - self.symmetry;
        // Validate symmetry
        if self.symmetry < 0.0 || symmetry_complement < 0.0 {
            return Err(Error::InvalidSymmetry);
        }

        const NYQUIST: f32 = (1u32 << 31) as _;
        let ftw = self.frequency * sample_period * NYQUIST;

        // Validate base frequency tuning word to be below Nyquist.
        if ftw < 0.0 || 2.0 * ftw > NYQUIST {
            return Err(Error::InvalidFrequency);
        }

        // Calculate the frequency tuning words.
        // Clip both frequency tuning words to within Nyquist before rounding.
        let phase_increment = [
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

        let amplitude = self.amplitude * (i16::MIN as f32 / -full_scale);
        if !(i16::MIN as f32..=i16::MAX as f32).contains(&amplitude) {
            return Err(Error::InvalidAmplitude);
        }

        let phase = self.phase * (1u64 << 32) as f32;

        Ok(Config {
            amplitude: amplitude as i16,
            signal: self.signal,
            phase_increment,
            phase_offset: phase as i32,
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
    pub phase_increment: [i32; 2],

    /// The phase offset
    pub phase_offset: i32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            signal: Signal::Cosine,
            amplitude: 0,
            phase_increment: [0, 0],
            phase_offset: 0,
        }
    }
}

#[derive(Debug)]
pub struct SignalGenerator {
    phase_accumulator: i32,
    config: Config,
    rng: XorShiftRng,
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
            rng: XorShiftRng::from_seed([0; 16]), // zeros will initialize with XorShiftRng internal seed
        }
    }

    /// Update waveform generation settings.
    pub fn update_waveform(&mut self, new_config: Config) {
        self.config = new_config;
    }

    /// Clear the phase accumulator.
    pub fn clear_phase_accumulator(&mut self) {
        self.phase_accumulator = 0;
    }
}

impl core::iter::Iterator for SignalGenerator {
    type Item = i16;

    /// Get the next value in the generator sequence.
    fn next(&mut self) -> Option<i16> {
        let phase = self
            .phase_accumulator
            .wrapping_add(self.config.phase_offset);
        let sign = phase.is_negative();
        self.phase_accumulator = self
            .phase_accumulator
            .wrapping_add(self.config.phase_increment[sign as usize]);

        let scale = match self.config.signal {
            Signal::Cosine => (idsp::cossin(phase).0 >> 16),
            Signal::Square => {
                if sign {
                    i16::MIN as i32
                } else {
                    -(i16::MIN as i32)
                }
            }
            Signal::Triangle => i16::MIN as i32 + (phase >> 15).abs(),
            Signal::WhiteNoise => self.rng.next_u32() as i32 >> 16,
        };

        // Calculate the final output result as an i16.
        Some(((self.config.amplitude as i32 * scale) >> 15) as _)
    }
}
