use core::num::Wrapping;

use arbitrary_int::{Number, u4};
use dsp_fixedpoint::{P32, Q32};
use dsp_process::{Process, SplitProcess};
use idsp::{
    ClampWrap, Complex, Unwrapper,
    iir::{
        Biquad, BiquadClamp, Cascade, DirectForm1, DirectForm1Dither,
        pid::{Gains, Order, Pid, Units},
        repr::BiquadRepr,
    },
};
use num_traits::Float;

pub const BATCH_SIZE: usize = 8;
pub const SAMPLE_TICKS_E: usize = 7;

// signal: p
// demod: t'f/N
// t': fixstate
// -> p + t'f/N
// -> lp(p + t'f/N) + (t - t')*f/N
// q: pll time
// t: true sample time
// t = q * (PLL_M << PLL_E)
const PLL_M: Wrapping<i32> = Wrapping(5); // 5 ns
const PLL_E: usize =
    16 // PLL input align ASL
    + SAMPLE_TICKS_E // sample timer divider
    + 1 // 10ns timer clock: 5 ns M, 1 E
    - 1 // 2 ns DDS sample clock
    - 2 // SYNC divider
    - 2 // ETR prescaler
;
// in q32: (t - t')*PLL_M/N << 32 = (q << (32 - PLL_E - N_LOG2)) - (PLL_M*t' << (32 - N_LOG2))

const _: () = {
    assert!(0x140_0000 / BATCH_SIZE as i32 == PLL_M.0 << PLL_E);
};

const N_E: usize = 4;

/// Fixed LO LUT mixer
#[derive(Debug, Clone)]
struct FixMix<const N: usize> {
    lo: [Q32<32>; N],
    f: usize,
    pi_half: usize,
}

#[derive(Debug, Clone, Default)]
struct FixMixState {
    i: usize,
}

impl<const N: usize> FixMix<N> {
    pub fn new(f: usize, p: f32) -> Self {
        Self {
            f,
            lo: core::array::from_fn(|i| {
                Q32::from_f64(
                    (0.5 - 1e-5)
                        * ((p as f64 + (i * f) as f64)
                            * const { core::f64::consts::TAU / N as f64 })
                        .cos(),
                )
            }),
            pi_half: (0..N).find(|i| (i * f) % N == N / 4).unwrap_or_default(),
        }
    }
}

impl<const N: usize> SplitProcess<i32, Complex<i32>, FixMixState>
    for FixMix<N>
{
    fn process(&self, state: &mut FixMixState, x: i32) -> Complex<i32> {
        state.i = state.i.wrapping_add(1);
        Complex([
            x * self.lo[state.i % N],
            x * self.lo[state.i.wrapping_sub(self.pi_half) % N],
        ])
    }
}

const WRAP_BITS: u32 = 13;

pub const PHASE_UNITS: Units<f32> = Units {
    // 100 MHz timer, 128 divider, BATCH_SIZE 8: period of 10.24 µs, ~98 KHz
    t: (BATCH_SIZE << SAMPLE_TICKS_E) as f32 * 10e-9,
    x: 1.0 / (PLL_M.0 << (32 - WRAP_BITS)) as f32,
    // 500 MHz DDS clock
    y: 500e6 / (1u64 << 32) as f32,
};

pub const AMPLITUDE_UNITS: Units<f32> = Units {
    t: 10e-3,
    x: 2.048 / (1 << 16) as f32, // 2.048 V per 16 bit
    y: 1.0 / i32::MAX as f32,
};

#[derive(Debug, Clone, miniconf::Tree)]
#[tree(meta(doc, typename))]
pub struct Channel {
    /// LO frequency tuning word in 1/16 Fs
    #[tree(with=miniconf::leaf)]
    lo_frequency: u4,

    /// LO phase offsets in turns
    lo_phase: f32,

    /// Lowpass filter (6th order biquad)
    #[tree(with=miniconf::leaf)]
    lowpass: [[f64; 5]; 3],

    /// Minimum demodulated signal power to enable feedback (re full scale)
    min_power: f32,

    /// Active filter representation
    #[tree(rename="phase_repr", typ="&str", with=miniconf::str_leaf, defer=self.phase)]
    _phase_repr: (), // before repr for setup from iter
    /// Phase-to-frequency filter (units: Hz/turn, s)
    phase: BiquadRepr<f32, Q32<29>, i32>,

    /// Active filter representation
    #[tree(rename="amplitude_repr", typ="&str", with=miniconf::str_leaf, defer=self.amplitude)]
    _amplitude_repr: (), // before repr
    /// Amplitude IIR filter (units: full scale amplitude/Volt, s)
    amplitude: BiquadRepr<f32, Q32<29>, i32>,
}

impl Default for Channel {
    fn default() -> Self {
        Self {
            lo_frequency: u4::new(5),
            lo_phase: 0.0,
            // 6th order transitional Guassian to 6 dB, zeros at 1,2,3 * 5/16
            // 3dB+13τ @22kHz, 36 dB @100kHz, gain 2
            lowpass: [
                [
                    0.020708473,
                    0.029286202,
                    0.020708473,
                    1.672374985,
                    -0.707726415,
                ],
                [
                    0.037786974,
                    0.028920897,
                    0.037786974,
                    1.643345831,
                    -0.747840887,
                ],
                [
                    1.366993065,
                    -2.525873828,
                    1.366993065,
                    1.6675967,
                    -0.875708817,
                ],
            ],
            min_power: 1e-6,
            _phase_repr: (),
            phase: BiquadRepr::Pid(Pid {
                min: -20e3,
                max: 20e3,
                setpoint: 0.0,
                order: Order::I,
                gain: Gains::new([0.0, -1e6, -1e3, 0.0, 0.0]),
                ..Default::default()
            }),
            _amplitude_repr: (),
            amplitude: BiquadRepr::Pid(Pid {
                min: 1.0,
                max: 1.0,
                ..Default::default()
            }),
        }
    }
}

#[derive(Debug, Clone)]
pub struct ChannelConfig {
    lo: FixMix<16>,
    lowpass: Cascade<[Biquad<Q32<29>>; 3]>,
    phase: BiquadClamp<Q32<29>, i32>,
    amplitude: BiquadClamp<Q32<29>, i32>,
    min_power: P32<28>,
}

impl Channel {
    pub fn build(&self) -> ChannelConfig {
        const { assert!(u4::MAX.value() == (1 << N_E) as u8 - 1) };
        ChannelConfig {
            lo: FixMix::new(self.lo_frequency.value() as _, self.lo_phase),
            lowpass: Cascade(self.lowpass.each_ref().map(|c| Biquad::from(*c))),
            min_power: P32::from_f32(self.min_power),
            phase: self.phase.build(&PHASE_UNITS),
            amplitude: self.amplitude.build(&AMPLITUDE_UNITS),
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct ChannelState {
    lo: FixMixState,
    lowpass: Complex<[DirectForm1<i32>; 3]>,
    pub demod: Complex<i32>,
    pub unwrap: Unwrapper<i64>,
    pub clamp: ClampWrap<Wrapping<i32>>,
    pub phase: DirectForm1Dither,
    pub amplitude: DirectForm1<i32>,
    pub hold: bool,
    pub holds: Wrapping<u32>,
    pub blanks: Wrapping<u32>,
    pub slips: Wrapping<u32>,
    pub power: P32<28>,
}

impl ChannelConfig {
    #[inline(always)]
    pub fn demodulate<const B: usize>(
        &self,
        state: &mut ChannelState,
        x: &[u16; B],
        y: &mut [u16; B],
    ) {
        // bit are peak amplitude
        // x: 15 bit, 16 asl
        // lo: 31 bit, 32 asr: 1 bit loss
        let mut m =
            x.map(|x| self.lo.process(&mut state.lo, (x as i16 as i32) << 16));
        // m: 29 bit 0f plus 29 bit 2f
        // lp: 1 bit DC gain
        let [si, sq] = state.lowpass.each_mut();
        for (c, s) in self.lowpass.0.iter().zip(si.iter_mut().zip(sq)) {
            for m in m.iter_mut() {
                m[0] = c.process(s.0, m[0]);
                m[1] = c.process(s.1, m[1]);
            }
            // demod: 30 bit
        }
        state.demod = m[B - 1];
        for (y, m) in y.iter_mut().zip(m) {
            // output in phase (before CPU clock phase correction from PLL)
            *y = ((m.re() >> 15) as i16).wrapping_add(i16::MIN) as u16;
        }
    }

    #[inline(always)]
    pub fn update(&self, state: &mut ChannelState, t: Wrapping<i32>) {
        let d2 = state.demod.0.map(|x| x as i64 * x as i64);
        // d2: 30 + 30 bit
        state.power = P32::new(((d2[0] + d2[1]) >> 32) as _);
        // power: 28 bit
        let blank = state.power < self.min_power;
        if blank {
            state.blanks += Wrapping(1);
        }
        if state.hold {
            state.holds += Wrapping(1);
        }
        if blank | state.hold {
            state.unwrap = Default::default();
            state.clamp = Default::default();
            state.phase = Default::default();
        } else {
            let phase = state.demod.arg();
            let phase = PLL_M * phase
                + Wrapping(self.lo.f as i32)
                    * ((t << (32 - PLL_E - N_E))
                        - (PLL_M << (32 - N_E)) * Wrapping(state.lo.i as i32));
            let dphase = Wrapping(state.unwrap.process(phase.0));
            // |delta phase| > pi/2 indicates a possible undetected phase slip
            // Neither a necessary nor a sufficient condition though
            if dphase + Wrapping(1 << 30) < Wrapping(0) {
                state.slips += Wrapping(1);
            }
            let phase = Wrapping((state.unwrap.y >> WRAP_BITS) as i32);
            let phase = state.clamp.process(phase);
            self.phase.process(&mut state.phase, phase.0);
        }
    }

    pub fn power(&self, state: &mut ChannelState, x: u32) {
        if state.hold {
            Biquad::<Q32<29>>::HOLD.process(&mut state.amplitude, x as _)
        } else {
            self.amplitude.process(&mut state.amplitude, x as _)
        };
    }
}
