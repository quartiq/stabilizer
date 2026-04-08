#![no_std]

use core::iter::Take;

use idsp::{AccuOsc, Sweep};
use miniconf::Tree;
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
#[tree(meta(doc, typename))]
pub struct Config {
    /// The signal type that should be generated. See [Signal] variants.
    #[tree(with=miniconf::leaf)]
    signal: Signal,

    /// The frequency of the generated signal in Hertz.
    frequency: f32,

    /// The normalized symmetry of the signal. At 0% symmetry, the duration of the first half oscillation is minimal.
    /// At 25% symmetry, the first half oscillation lasts for 25% of the signal period. For square wave output this
    /// symmetry is the duty cycle.
    symmetry: f32,

    /// The amplitude of the output signal
    amplitude: f32,

    /// Output offset
    offset: f32,

    /// The initial phase of the period output signal in turns
    phase: f32,

    /// Number of half periods (periodic) or samples (sweep and noise), 0 for infinte
    length: u32,

    /// Sweep: initial state
    state: i64,

    /// Sweep: Sweep rate
    rate: i32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            frequency: 1.0e3,
            symmetry: 0.5,
            signal: Signal::Cosine,
            amplitude: 0.0,
            phase: 0.0,
            offset: 0.0,
            state: 0,
            rate: 0,
            length: 0,
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
        (((x as i64 * self.amp as i64) >> 31) as i32)
            .saturating_add(self.offset)
    }
}

/// Represents the errors that can occur when attempting to configure the signal generator.
#[derive(Copy, Clone, Debug, thiserror::Error)]
pub enum Error {
    /// The provided amplitude is out-of-range.
    #[error("Invalid amplitude")]
    Amplitude,
    /// The provided symmetry is out of range.
    #[error("Invalid symmetry")]
    Symmetry,
    /// The provided frequency is out of range.
    #[error("Invalid frequency")]
    Frequency,
    /// Sweep would wrap/invalid
    #[error("Sweep would wrap")]
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
            Self::SweptSine { sweep, amp } => {
                (sweep.next().map(|c| c.im()), amp)
            }
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

impl Config {
    /// Convert from SI config
    pub fn build(&self, period: f32, scale: f32) -> Result<Source, Error> {
        if !(0.0..1.0).contains(&self.symmetry) {
            return Err(Error::Symmetry);
        }

        const NYQUIST: f32 = (1u32 << 31) as _;
        let ftw0 = self.frequency * period * NYQUIST;
        if !(0.0..2.0 * NYQUIST).contains(&ftw0) {
            return Err(Error::Frequency);
        }

        // Clip both frequency tuning words to within Nyquist before rounding.
        let ftw = [
            if self.symmetry * NYQUIST > ftw0 {
                ftw0 / self.symmetry
            } else {
                NYQUIST
            } as i32,
            if (1.0 - self.symmetry) * NYQUIST > ftw0 {
                ftw0 / (1.0 - self.symmetry)
            } else {
                NYQUIST
            } as i32,
        ];

        let offset = self.offset * scale;
        let amplitude = self.amplitude * scale;
        fn abs(x: f32) -> f32 {
            if x.is_sign_negative() { -x } else { x }
        }
        if abs(offset) + abs(amplitude) >= 1.0 {
            return Err(Error::Amplitude);
        }
        let amp = Scaler {
            amp: (amplitude * NYQUIST) as _,
            offset: (offset * NYQUIST) as _,
        };

        Ok(match self.signal {
            signal @ (Signal::Cosine | Signal::Square | Signal::Triangle) => {
                Source::Periodic {
                    accu: AsymmetricAccu {
                        ftw,
                        pow: (self.phase * NYQUIST) as i32,
                        accu: 0,
                        count: self.length,
                    },
                    signal,
                    amp,
                }
            }
            Signal::SweptSine => Source::SweptSine {
                sweep: AccuOsc::new(Sweep::new(self.rate, self.state))
                    .take(self.length as _),
                amp,
            },
            Signal::WhiteNoise => Source::WhiteNoise {
                rng: XorShiftRng::from_seed(Default::default()),
                count: self.length,
                amp,
            },
        })
    }
}
