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
            phase_symmetry: ((symmetry - 0.5) * i32::MAX as f32) as i32,
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
}

#[derive(Debug)]
pub struct Generator {
    index: u32,
    config: InternalConf,
    pending_config: Option<InternalConf>,
}

impl Default for Generator {
    fn default() -> Self {
        Self {
            config: Config::default().into(),
            index: 0,
            pending_config: None,
        }
    }
}

impl Generator {
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

    /// Get the next value in the generator sequence.
    pub fn next(&mut self) -> i16 {
        // When phase wraps, apply any new settings.
        if self.pending_config.is_some() && self.index == 0 {
            self.config = self.pending_config.take().unwrap();
        }

        let phase_step = u32::MAX / self.config.period;

        // Note: We allow phase to silently wrap here intentionally, as it will wrap to negative.
        // This is acceptable with phase, since it is perfectly periodic.
        let phase = (self.index * phase_step) as i32;

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
                    let rise_period: u32 =
                        (self.config.phase_symmetry - i32::MIN) as u32
                            / phase_step;

                    if rise_period == 0 {
                        i16::MIN
                    } else {
                        i16::MIN
                            + (self.index * u16::MAX as u32 / rise_period)
                                as i16
                    }
                } else {
                    let fall_period: u32 = (i32::MAX as u32
                        - self.config.phase_symmetry as u32)
                        / phase_step;
                    let index: u32 = (phase - self.config.phase_symmetry)
                        as u32
                        / phase_step;

                    if fall_period == 0 {
                        i16::MAX
                    } else {
                        i16::MAX
                            - (index * u16::MAX as u32 / fall_period) as i16
                    }
                }
            }
        };

        // Update the current index.
        self.index = (self.index + 1) % self.config.period;

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
