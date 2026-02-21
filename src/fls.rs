use core::num::Wrapping;

use arbitrary_int::{Number, u4, u10};
use dsp_fixedpoint::{P32, Q32};
use dsp_process::{
    Add, Identity, Pair, Parallel, Process, SplitProcess, Unsplit,
};
use idsp::{
    ClampWrap, Complex, Unwrapper,
    iir::{
        Biquad, BiquadClamp, DirectForm1, DirectForm1Dither,
        pid::{Gains, Order, Pid, Units},
        repr::BiquadRepr,
        wdf::{Wdf, WdfState},
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
    16 // PLL input align
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
struct FixLo<const N: usize> {
    lo: [Q32<32>; N],
    f: usize,
    pi_half: usize,
}

#[derive(Debug, Clone, Default)]
pub struct FixState {
    i: usize,
}

impl<const N: usize> FixLo<N> {
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
            pi_half: (0..N).find(|i| i * f % N == N / 4).unwrap(),
        }
    }
}

impl<const N: usize> SplitProcess<i32, Complex<i32>, FixState> for FixLo<N> {
    fn process(&self, state: &mut FixState, x: i32) -> Complex<i32> {
        state.i = state.i.wrapping_add(1);
        let y = Complex([
            x * self.lo[state.i % N],
            x * self.lo[state.i.wrapping_sub(self.pi_half) % N],
        ]);
        y
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
    x: 2.048 / (1 << 16) as f32,
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

    /// Lowpass filter poles (7th order wave digital filter allpass pair)
    #[tree(with=miniconf::leaf)]
    lowpass: (([f64; 2], [f64; 2]), ([f64; 2], [f64; 1])),

    /// Minimum demodulated signal power to enable feedback (re full scale)
    min_power: f32,

    /// Active filter representation
    #[tree(rename="phase_repr", typ="&str", with=miniconf::str_leaf, defer=self.phase)]
    _phase_repr: (), // before repr
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
            lowpass: (
                (
                    [-0.982905335393602, 0.9991878840149784],
                    [-0.9283527198560211, 0.9991355621411774],
                ),
                (
                    [-0.9515541301687671, 0.9991654023997083],
                    [0.9589369885243411],
                ),
            ),
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
    lo: FixLo<16>,
    lowpass: Pair<[Wdf<2, 0xad>; 2], (Wdf<2, 0xad>, Wdf<1, 0xa>), i32>,
    phase: BiquadClamp<Q32<29>, i32>,
    amplitude: BiquadClamp<Q32<29>, i32>,
    min_power: P32<28>,
}

impl Channel {
    pub fn build(&self) -> ChannelConfig {
        const { assert!(u4::MAX.value() == (1 << N_E) as u8 - 1) };
        ChannelConfig {
            lo: FixLo::new(self.lo_frequency.value() as _, self.lo_phase),
            lowpass: Pair::new((
                (
                    (),
                    Parallel((
                        [
                            Wdf::quantize(&self.lowpass.0.0).unwrap(),
                            Wdf::quantize(&self.lowpass.0.1).unwrap(),
                        ],
                        (
                            Wdf::quantize(&self.lowpass.1.0).unwrap(),
                            Wdf::quantize(&self.lowpass.1.1).unwrap(),
                        ),
                    )),
                ),
                (),
            )),
            min_power: P32::from_f32(self.min_power),
            phase: self.phase.build(&PHASE_UNITS),
            amplitude: self.amplitude.build(&AMPLITUDE_UNITS),
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct ChannelState {
    lo: FixState,
    lowpass: Complex<(
        (
            Unsplit<Identity>,
            ([WdfState<2>; 2], (WdfState<2>, WdfState<1>)),
        ),
        Unsplit<Add>,
    )>,
    pub unwrap: Unwrapper<i64>,
    pub clamp: ClampWrap<Wrapping<i32>>,
    pub phase: DirectForm1Dither,
    pub amplitude: DirectForm1<i32>,
    pub hold: bool,
    pub holds: Wrapping<u32>,
    pub blanks: Wrapping<u32>,
    pub slips: Wrapping<u32>,
    pub mod_amp: u10,
    pub power: P32<28>,
}

impl ChannelConfig {
    #[inline(always)]
    pub fn demodulate<const B: usize>(
        &self,
        state: &mut ChannelState,
        x: &[u16; B],
        y: &mut [u16; B],
    ) -> Complex<i32> {
        x.iter()
            .zip(y.iter_mut())
            .map(|(x, y)| {
                // bit are peak amplitude
                // x: 15 bit, 16 asl
                // lo: 31 bit, 32 asr: 1 bit loss
                let m =
                    self.lo.process(&mut state.lo, (*x as i16 as i32) << 16);
                // m: 29 bit 0f plus 29 bit 2f
                // lp: 1 bit DC gain
                let demod = Complex([
                    self.lowpass.process(&mut state.lowpass[0], m[0]),
                    self.lowpass.process(&mut state.lowpass[1], m[1]),
                ]);
                // demod: 30 bit
                // output in phase (before CPU clock phase correction)
                *y = ((demod.re() >> 15) as i16).wrapping_add(i16::MIN) as u16;
                demod
            })
            .last()
            .unwrap()
    }

    #[inline(always)]
    pub fn update(
        &self,
        state: &mut ChannelState,
        t: Wrapping<i32>,
        demod: Complex<i32>,
    ) {
        let d2 = demod.0.map(|x| x as i64 * x as i64);
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
            let phase = demod.arg();
            let phase = PLL_M * phase
                + Wrapping(self.lo.f as i32)
                    * ((t << (32 - PLL_E - N_E))
                        - (PLL_M << (32 - N_E)) * Wrapping(state.lo.i as i32));
            let dphase = Wrapping(state.unwrap.process(phase.0));
            // |delta phase| > pi/2 indicates a possible undetected phase slip.
            // Neither a necessary nor a sufficient condition though.
            if dphase + Wrapping(1 << 30) < Wrapping(0) {
                state.slips += Wrapping(1);
            }
            let phase = Wrapping((state.unwrap.y >> WRAP_BITS) as i32);
            let phase = state.clamp.process(phase);
            self.phase.process(&mut state.phase, phase.0);
        }
    }

    pub fn power(&self, state: &mut ChannelState, x: u32) {
        let y0 = if state.hold {
            Biquad::<Q32<29>>::HOLD.process(&mut state.amplitude, x as _)
        } else {
            self.amplitude.process(&mut state.amplitude, x as _)
        };
        state.mod_amp = u10::new((y0 >> 21) as u16);
    }
}
