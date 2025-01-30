use core::iter::Take;

use idsp::{AccuOsc, Sweep};
use miniconf::{Leaf, Tree};
use rand_core::{RngCore, SeedableRng};
use rand_xorshift::XorShiftRng;
use serde::{Deserialize, Serialize};

/// Types of signals that can be generated.
#[derive(Copy, Clone, Debug, Deserialize, Serialize)]
pub enum Signal {
    Cosine,
    Square,
    Triangle,
    WhiteNoise,
    SweptSine,
}

impl Signal {
    #[inline]
    fn map(&self, x: i32) -> i32 {
        match self {
            Self::Cosine => idsp::cossin(x).0,
            Self::Square => {
                if x.is_negative() {
                    -i32::MAX
                } else {
                    i32::MAX
                }
            }
            Self::Triangle => i32::MIN + (x.saturating_abs() << 1),
            _ => unimplemented!(),
        }
    }
}

/// Basic configuration for a generated signal.
#[derive(Clone, Debug, Tree, Serialize, Deserialize)]
pub struct Config {
    /// The signal type that should be generated. See [Signal] variants.
    signal: Leaf<Signal>,

    /// The frequency of the generated signal in Hertz.
    frequency: Leaf<f32>,

    /// The normalized symmetry of the signal. At 0% symmetry, the duration of the first half oscillation is minimal.
    /// At 25% symmetry, the first half oscillation lasts for 25% of the signal period. For square wave output this
    /// symmetry is the duty cycle.
    symmetry: Leaf<f32>,

    /// The amplitude of the output signal
    amplitude: Leaf<f32>,

    /// Output offset
    offset: Leaf<f32>,

    /// The initial phase of the period output signal in turns
    phase: Leaf<f32>,

    /// Number of half periods (periodic) or samples (sweep and noise), 0 for infinte
    length: Leaf<u32>,

    /// Sweep: Number of cycles for the first octave
    cycles: Leaf<i32>,

    /// Sweep: Sweep rate
    rate: Leaf<i32>,

    /// Sample period
    #[tree(skip)]
    pub period: f32,
    /// Output full scale
    #[tree(skip)]
    pub scale: f32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            frequency: Leaf(1.0e3),
            symmetry: Leaf(0.5),
            signal: Leaf(Signal::Cosine),
            amplitude: Leaf(0.0),
            phase: Leaf(0.0),
            offset: Leaf(0.0),
            cycles: Leaf(1),
            rate: Leaf(0),
            length: Leaf(0),
            period: 1.0,
            scale: 1.0,
        }
    }
}

#[derive(Clone, Debug)]
pub struct AsymmetricAccu {
    ftw: [i32; 2],
    pow: i32,
    accu: i32,
    count: u32,
}

impl Iterator for AsymmetricAccu {
    type Item = i32;
    fn next(&mut self) -> Option<Self::Item> {
        let sign = self.accu.is_negative();
        self.accu = self.accu.wrapping_add(self.ftw[sign as usize]);
        self.count
            .checked_sub(sign as u32 ^ self.accu.is_negative() as u32)
            .map(|c| {
                self.count = c;
                self.accu.wrapping_add(self.pow)
            })
    }
}

#[derive(Clone, Debug)]
pub struct Scaler {
    amp: i32,
    offset: i32,
}

impl Scaler {
    fn map(&self, x: i32) -> i32 {
        ((x as i64 * self.amp as i64) >> 31) as i32 + self.offset
    }
}

/// Represents the errors that can occur when attempting to configure the signal generator.
#[derive(Copy, Clone, Debug)]
pub enum Error {
    /// The provided amplitude is out-of-range.
    Amplitude,
    /// The provided symmetry is out of range.
    Symmetry,
    /// The provided frequency is out of range.
    Frequency,
    /// Sweep would wrap/invalid
    Wrap,
}

#[derive(Clone, Debug)]
pub enum Source {
    SweptSine {
        sweep: Take<AccuOsc<Sweep>>,
        amp: Scaler,
    },
    Periodic {
        accu: AsymmetricAccu,
        signal: Signal,
        amp: Scaler,
    },
    WhiteNoise {
        rng: XorShiftRng,
        count: u32,
        amp: Scaler,
    },
}

impl Iterator for Source {
    type Item = i32;
    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        let (s, a) = match self {
            Self::SweptSine { sweep, amp } => (sweep.next().map(|c| c.im), amp),
            Self::Periodic { accu, signal, amp } => {
                (accu.next().map(|p| signal.map(p)), amp)
            }
            Self::WhiteNoise { rng, count, amp } => (
                count.checked_sub(1).map(|m| {
                    *count = m;
                    rng.next_u32() as i32
                }),
                amp,
            ),
        };
        Some(a.map(s.unwrap_or_default()))
    }
}

impl Source {
    /// Convert from SI config
    pub fn try_from_config(value: &Config) -> Result<Source, Error> {
        if !(0.0..1.0).contains(&*value.symmetry) {
            return Err(Error::Symmetry);
        }

        const NYQUIST: f32 = (1u32 << 31) as _;
        let ftw0 = *value.frequency * value.period * NYQUIST;
        if !(0.0..2.0 * NYQUIST).contains(&ftw0) {
            return Err(Error::Frequency);
        }

        // Clip both frequency tuning words to within Nyquist before rounding.
        let ftw = [
            if *value.symmetry * NYQUIST > ftw0 {
                ftw0 / *value.symmetry
            } else {
                NYQUIST
            } as i32,
            if (1.0 - *value.symmetry) * NYQUIST > ftw0 {
                ftw0 / (1.0 - *value.symmetry)
            } else {
                NYQUIST
            } as i32,
        ];

        let offset = *value.offset / value.scale;
        let amplitude = *value.amplitude / value.scale;
        fn abs(x: f32) -> f32 {
            if x.is_sign_negative() {
                -x
            } else {
                x
            }
        }
        if abs(offset) + abs(amplitude) >= 1.0 {
            return Err(Error::Amplitude);
        }
        let amp = Scaler {
            amp: (amplitude * NYQUIST) as _,
            offset: (offset * NYQUIST) as _,
        };

        Ok(match *value.signal {
            signal @ (Signal::Cosine | Signal::Square | Signal::Triangle) => {
                Self::Periodic {
                    accu: AsymmetricAccu {
                        ftw,
                        pow: (*value.phase * NYQUIST) as i32,
                        accu: 0,
                        count: *value.length,
                    },
                    signal,
                    amp,
                }
            }
            Signal::SweptSine => Self::SweptSine {
                sweep: AccuOsc::new(Sweep::new(
                    *value.rate,
                    ((*value.rate * *value.cycles) as i64) << 32,
                ))
                .take(*value.length as _),
                amp,
            },
            Signal::WhiteNoise => Self::WhiteNoise {
                rng: XorShiftRng::from_seed(Default::default()),
                count: *value.length,
                amp,
            },
        })
    }
}
