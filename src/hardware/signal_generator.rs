#[derive(Copy, Clone, Debug)]
pub enum Signal {
    Sine,
    Square,
    Triangle,
}

#[derive(Copy, Clone, Debug)]
pub struct Config {
    // TODO: Should period be specified in Hz?
    pub period: u32,
    pub symmetry: f32,
    pub signal: Signal,
    pub amplitude: f32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            period: 10,
            symmetry: 0.5,
            signal: Signal::Sine,
            amplitude: 1.0,
        }
    }
}

impl Into<InternalConf> for Config {
    fn into(self) -> InternalConf {
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

        InternalConf {
            signal: self.signal,
            period: self.period,
            amplitude: ((amplitude / 10.24) * i16::MAX as f32) as i16,
            phase_symmetry: (2.0 * (symmetry - 0.5) * i32::MAX as f32) as i32,
            phase_step: ((u32::MAX as u64 + 1u64) / self.period as u64) as u32,
        }
    }
}

#[derive(Copy, Clone, Debug)]
struct InternalConf {
    period: u32,
    signal: Signal,
    amplitude: i16,

    // The 32-bit representation of the phase symmetry. That is, with a 50% symmetry, this is equal
    // to 0.
    phase_symmetry: i32,

    phase_step: u32,
}

#[derive(Debug)]
pub struct SignalGenerator {
    index: u32,
    config: InternalConf,
    pending_config: Option<InternalConf>,
}

impl Default for SignalGenerator {
    fn default() -> Self {
        Self {
            config: Config::default().into(),
            index: 0,
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
    pub fn new(config: Config) -> Self {
        Self {
            config: config.into(),
            pending_config: None,
            index: 0,
        }
    }

    /// Generate a sequence of new values.
    ///
    /// # Args
    /// * `samples` - The location to store generated values into.
    pub fn generate(&mut self, samples: &mut [i16]) {
        for sample in samples.iter_mut() {
            *sample = self.next();
        }
    }

    /// Skip `count` elements of the generator
    pub fn skip(&mut self, count: usize) {
        let index = self.index.wrapping_add(count);

        // If we skip past the period of the signal, apply any pending config.
        if index > self.config.period && let Some(config) = self.pendig_config.take() {
            self.config = config;
        }

        self.index = index % self.config.period;
    }

    /// Get the next value in the generator sequence.
    pub fn next(&mut self) -> i16 {
        // When phase wraps, apply any new settings.
        if self.pending_config.is_some() && self.index == 0 {
            self.config = self.pending_config.take().unwrap();
        }

        let phase = (self.index * self.config.phase_step) as i32;

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
                    let duration_of_phase = (self.config.phase_symmetry.wrapping_sub(i32::MIN) >> 16) as u16;
                    let phase_progress = (phase.wrapping_sub(i32::MIN) >> 16) as u16;

                    if duration_of_phase == 0 {
                        i16::MIN
                    } else {
                        i16::MIN.wrapping_add((u16::MAX as u32 * phase_progress as u32 /
                                duration_of_phase as u32) as i16)
                    }
                } else {

                    let duration_of_phase = (i32::MAX.wrapping_sub(self.config.phase_symmetry) >> 16) as u16;
                    let phase_progress = (phase.wrapping_sub(self.config.phase_symmetry) >> 16) as
                        u16;

                    if duration_of_phase == 0 {
                        i16::MAX
                    } else {
                        i16::MAX.wrapping_sub((u16::MAX as u32 * phase_progress as u32 / duration_of_phase as u32) as i16)
                    }
                }
            }
        };

        // Update the current index.
        self.index = self.index.wrapping_add(1) % self.config.period;

        // Calculate the final output result as an i16.
        let result = amplitude as i32 * self.config.amplitude as i32;

        // Note: We downshift by 15-bits to preserve only one of the sign bits.
        (result >> 15) as i16
    }

    /// Update waveform generation settings.
    ///
    /// # Note
    /// Changes will not take effect until the current waveform period elapses.
    pub fn update_waveform(&mut self, new_config: Config) {
        self.pending_config = Some(new_config.into());
    }
}
