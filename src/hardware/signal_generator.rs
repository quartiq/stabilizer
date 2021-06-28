use crate::hardware::{dac::DacCode, design_parameters::ADC_SAMPLE_TICKS};
use miniconf::Miniconf;
use serde::Deserialize;

#[derive(Copy, Clone, Debug, Deserialize, Miniconf)]
pub enum Signal {
    Sine,
    Square,
    Triangle,
}

#[derive(Copy, Clone, Debug, Miniconf, Deserialize)]
pub struct BasicConfig {
    pub frequency: f32,
    pub symmetry: f32,
    pub signal: Signal,
    pub amplitude: f32,
}

impl Default for BasicConfig {
    fn default() -> Self {
        Self {
            frequency: 1.0e3,
            symmetry: 0.5,
            signal: Signal::Sine,
            amplitude: 1.0,
        }
    }
}

impl Into<Config> for BasicConfig {
    fn into(self) -> Config {
        // Calculate the number of output codes in the signal period.
        let period =
            (100.0_e6 / (self.frequency * ADC_SAMPLE_TICKS as f32)) as u32;

        // Clamp amplitude and symmetry.
        let amplitude = if self.amplitude > 10.24 {
            10.24
        } else {
            self.amplitude
        };

        let symmetry = if self.symmetry < 0.0 {
            0.0
        } else if self.symmetry > 1.0 {
            1.0
        } else {
            self.symmetry
        };

        Config {
            signal: self.signal,
            amplitude: ((amplitude / 10.24) * i16::MAX as f32) as i16,
            phase_symmetry: (2.0 * (symmetry - 0.5) * i32::MAX as f32) as i32,
            frequency_tuning_word: ((u32::MAX as u64 + 1u64) / period as u64)
                as u32,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Config {
    // The type of signal being generated
    pub signal: Signal,

    // The full-scale output code of the signal
    pub amplitude: i16,

    // The 32-bit representation of the phase symmetry. That is, with a 50% symmetry, this is equal
    // to 0.
    pub phase_symmetry: i32,

    // The frequency tuning word of the signal. Phase is incremented by this amount
    pub frequency_tuning_word: u32,
}

#[derive(Debug)]
pub struct SignalGenerator {
    phase_accumulator: u32,
    config: Config,
    pending_config: Option<Config>,
}

impl Default for SignalGenerator {
    fn default() -> Self {
        Self {
            config: BasicConfig::default().into(),
            phase_accumulator: 0,
            pending_config: None,
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
    pub fn new(config: impl Into<Config>) -> Self {
        Self {
            config: config.into(),
            pending_config: None,
            phase_accumulator: 0,
        }
    }

    /// Generate a sequence of new values.
    ///
    /// # Args
    /// * `samples` - The location to store generated values into.
    pub fn generate(&mut self, samples: &mut [u16]) {
        for sample in samples.iter_mut() {
            *sample = DacCode::from(self.next()).0;
        }
    }

    // Increment the phase of the signal.
    //
    // # Note
    // This handles automatically applying pending configurations on phase wrap.
    //
    // # Returns
    // The new phase to use
    fn increment(&mut self) -> i32 {
        let (phase, overflow) = self
            .phase_accumulator
            .overflowing_add(self.config.frequency_tuning_word);

        self.phase_accumulator = phase;

        // Special case: If the FTW is specified as zero, we would otherwise never update the
        // settings. Perform a check here for this corner case.
        if overflow || self.config.frequency_tuning_word == 0 {
            if let Some(config) = self.pending_config.take() {
                self.config = config;
                self.phase_accumulator = 0;
            }
        }

        self.phase_accumulator as i32
    }

    /// Skip `count` elements of the generator
    pub fn skip(&mut self, count: u32) {
        for _ in 0..count {
            self.increment();
        }
    }

    /// Get the next value in the generator sequence.
    pub fn next(&mut self) -> i16 {
        let phase = self.increment();

        let amplitude = match self.config.signal {
            Signal::Sine => (dsp::cossin(phase).1 >> 16) as i16,
            Signal::Square => {
                if phase < self.config.phase_symmetry {
                    i16::MAX
                } else {
                    i16::MIN
                }
            }
            Signal::Triangle => {
                if phase < self.config.phase_symmetry {
                    let duration_of_phase =
                        (self.config.phase_symmetry.wrapping_sub(i32::MIN)
                            >> 16) as u16;
                    let phase_progress =
                        (phase.wrapping_sub(i32::MIN) >> 16) as u16;

                    if duration_of_phase == 0 {
                        i16::MIN
                    } else {
                        i16::MIN.wrapping_add(
                            (u16::MAX as u32 * phase_progress as u32
                                / duration_of_phase as u32)
                                as i16,
                        )
                    }
                } else {
                    let duration_of_phase =
                        (i32::MAX.wrapping_sub(self.config.phase_symmetry)
                            >> 16) as u16;
                    let phase_progress = (phase
                        .wrapping_sub(self.config.phase_symmetry)
                        >> 16) as u16;

                    if duration_of_phase == 0 {
                        i16::MAX
                    } else {
                        i16::MAX.wrapping_sub(
                            (u16::MAX as u32 * phase_progress as u32
                                / duration_of_phase as u32)
                                as i16,
                        )
                    }
                }
            }
        };

        // Calculate the final output result as an i16.
        let result = amplitude as i32 * self.config.amplitude as i32;

        // Note: We downshift by 15-bits to preserve only one of the sign bits.
        (result >> 15) as i16
    }

    /// Update waveform generation settings.
    ///
    /// # Note
    /// Changes will not take effect until the current waveform period elapses.
    pub fn update_waveform(&mut self, new_config: impl Into<Config>) {
        self.pending_config = Some(new_config.into());
    }
}
