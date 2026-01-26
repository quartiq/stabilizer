use core::num::Wrapping;

use dsp_fixedpoint::Q32;
use dsp_process::{
    Add, Identity, Pair, Parallel, Process, SplitInplace, SplitProcess, Unsplit,
};
use idsp::iir::{
    BiquadClamp, DirectForm1, pid,
    repr::BiquadRepr,
    wdf::{Wdf, WdfState},
};

pub const BATCH_SIZE: usize = 8;
// 100 MHz timer, 128 divider: period of 1.28 µs, ~781.25 KHz.
pub const SAMPLE_TICKS: u32 = 1 << 7;
const S_PER_SAMPLE: f32 = SAMPLE_TICKS as f32 / 100e6;
const TURN_PER_LSB: f32 = ((1u64 << 32) as f32).recip();
const HZ_PER_LSB: f32 = ((1u64 << 32) as f32 * S_PER_SAMPLE).recip();

pub const UNITS: pid::Units<f32> = pid::Units {
    t: BATCH_SIZE as f32 * S_PER_SAMPLE,
    x: TURN_PER_LSB,
    y: HZ_PER_LSB,
};

#[derive(Debug, Clone, Default)]
pub struct MpllState {
    /// Lowpass state
    lp: [(((), ([WdfState<2>; 2], (WdfState<2>, WdfState<1>))), ()); 4],
    /// Phase clamp
    clamp: idsp::ClampWrap<i32>,
    /// PID state
    iir: DirectForm1<i32>,
    /// Current modulation phase
    phase: Wrapping<i32>,
    /// LO samples for downconverting the next batch
    lo: [[i32; BATCH_SIZE]; 2],
}

impl MpllState {
    pub fn set_frequency(&mut self, f: f32) {
        self.iir.xy[2] = (f * UNITS.y.recip()) as _;
        self.iir.xy[3] = self.iir.xy[2];
    }
}

/// Runtime active settings
#[derive(Debug, Clone)]
pub struct Mpll {
    lp: Lowpass,
    offset: Option<Wrapping<i32>>,
    iir: BiquadClamp<Q32<30>, i32>,
    amplitude: [Q32<16>; 2],
}

#[derive(Debug, Clone)]
struct Lowpass(Pair<[Wdf<2, 0xad>; 2], (Wdf<2, 0xad>, Wdf<1, 0xa>), i32>);

#[derive(Debug, Clone, miniconf::Tree)]
#[tree(meta(doc, typename))]
pub struct MpllConfig {
    /// Lowpass filter poles (7th order wave digital filter allpass pair)
    #[tree(with=miniconf::leaf)]
    lp: (([f64; 2], [f64; 2]), ([f64; 2], [f64; 1])),
    /// Input phase offset (turns) and clamp
    ///
    /// Makes the phase wrap monotonic by clamping it to aid capture/pull-in with external modulation.
    /// Disable for modulation drive and use biquad offset/pid setpoint
    offset: Option<f32>,
    /// Filter representation
    #[tree(rename="repr", typ="&str", with=miniconf::str_leaf, defer=self.iir)]
    _repr: (), // before iir
    /// Phase-to-frequency filter (units Hz/turn, s)
    iir: BiquadRepr<f32, Q32<30>, i32>,
    /// Output amplitude (Volt)
    ///
    /// DAC0: in-phase modulation amplitude in V
    /// DAC1: in-loop phase error in V/turn
    amplitude: [f32; 2],
}

impl Default for MpllConfig {
    fn default() -> Self {
        // 7th order Cheby2 as WDF-CA, -3 dB @ 0.005*1.28, -112 dB @ 0.018*1.28
        let _lp = (
            (
                [-0.982905335393602, 0.9991878840149784],
                [-0.9283527198560211, 0.9991355621411774],
            ),
            (
                [-0.9515541301687671, 0.9991654023997083],
                [0.9589369885243411],
            ),
        );
        // 7th order Cheby2 as WDF-CA, -3 dB @ 0.002*1.28, -112 dB @ 0.008*1.28
        let lp = (
            (
                [-0.9931253245194313, 0.9998700465308578],
                [-0.9707043427084593, 0.9998616710293764],
            ),
            (
                [-0.9803312351720791, 0.9998664477981001],
                [0.9833717980454499],
            ),
        );

        let mut pid = pid::Pid::default();
        pid.order = pid::Order::I;
        pid.gains.value[pid::Action::P as usize] = -1e3; // Hz/turn
        pid.gains.value[pid::Action::I as usize] = -5e5; // Hz/sturn
        // pid.gains.value[pid::Action::P as usize] = -1.2e3; // Hz/turn
        // pid.gains.value[pid::Action::I as usize] = -5e5; // Hz/sturn
        // pid.gains.value[pid::Action::D as usize] = -0.15; // sHz/turn
        // pid.limits.value[pid::Action::D as usize] = -20e3; // Hz/turn
        pid.min = 5e3; // Hz
        pid.max = 300e3; // Hz

        Self {
            lp,
            offset: Some(0.0),
            iir: BiquadRepr::Pid(pid),
            _repr: (),
            amplitude: [1.0, 0.0], // V
        }
    }
}

impl MpllConfig {
    pub fn build(&self) -> Mpll {
        Mpll {
            lp: Lowpass(Pair::new((
                (
                    Unsplit(&Identity),
                    Parallel((
                        [
                            Wdf::quantize(&self.lp.0.0).unwrap(),
                            Wdf::quantize(&self.lp.0.1).unwrap(),
                        ],
                        (
                            Wdf::quantize(&self.lp.1.0).unwrap(),
                            Wdf::quantize(&self.lp.1.1).unwrap(),
                        ),
                    )),
                ),
                Unsplit(&Add),
            ))),
            offset: self.offset.map(|p| Wrapping((p * UNITS.x.recip()) as _)),
            iir: self.iir.build(UNITS),
            amplitude: self
                .amplitude
                .map(|a| Q32::from_f32(a * 10.24f32.recip())),
        }
    }
}

impl Mpll {
    /// Ingest and process a batch of input samples and output new modulation
    pub fn process(
        &self,
        state: &mut MpllState,
        x: [&[u16; BATCH_SIZE]; 2],
        y: [&mut [u16; BATCH_SIZE]; 2],
    ) -> Stream {
        // scratch
        let mut mix = [[0; BATCH_SIZE]; 4];
        let [m00, m01, m10, m11] = mix.each_mut();
        // mix
        for (x, (mix, lo)) in x[0].iter().zip(x[1].iter()).zip(
            m00.iter_mut()
                .zip(m01)
                .zip(m10.iter_mut().zip(m11))
                .zip(state.lo[0].iter().zip(state.lo[1].iter())),
        ) {
            // ADC encoding, mix, 1 bit loss (headroom)
            *mix.0.0 = ((*x.0 as i16 as i64 * *lo.0 as i64) >> 16) as i32;
            *mix.0.1 = ((*x.0 as i16 as i64 * *lo.1 as i64) >> 16) as i32;
            *mix.1.0 = ((*x.1 as i16 as i64 * *lo.0 as i64) >> 16) as i32;
            *mix.1.1 = ((*x.1 as i16 as i64 * *lo.1 as i64) >> 16) as i32;
        }
        // lowpass, 1 bit gain
        for (mix, state) in mix.iter_mut().zip(state.lp.iter_mut()) {
            self.lp.0.inplace(state, mix);
        }
        // decimate
        let demod = [
            mix[0][BATCH_SIZE - 1],
            mix[1][BATCH_SIZE - 1],
            mix[2][BATCH_SIZE - 1],
            mix[3][BATCH_SIZE - 1],
        ];
        // phase
        // need full atan2, rotation, or addtl osc to support any phase offset
        let mut phase = Wrapping(idsp::atan2(demod[1], demod[0]));
        // Delay correction
        // TODO: consider delayed LO. This might add some frequency noise.
        phase -= Wrapping(state.iir.xy[2]) * Wrapping(10);
        // Offset and clamp
        if let Some(offset) = self.offset {
            phase.0 = state.clamp.process((phase + offset).0);
        }
        // PID
        let frequency = Wrapping(self.iir.process(&mut state.iir, phase.0));
        // modulate
        let [y0, y1] = state.lo.each_mut();
        let [yo0, yo1] = y;
        for (y, yo) in y0.iter_mut().zip(y1).zip(yo0.iter_mut().zip(yo1)) {
            state.phase += frequency;
            (*y.0, *y.1) = idsp::cossin(state.phase.0);
            *yo.0 = (((*y.0 as i64 * self.amplitude[0].inner as i64) >> 32)
                as i16)
                .wrapping_add(i16::MIN) as u16; // DAC encoding
            *yo.1 = (((phase.0 as i64 * self.amplitude[1].inner as i64) >> 32)
                as i16)
                .wrapping_add(i16::MIN) as u16; // DAC encoding
        }
        Stream {
            demod,
            phase,
            frequency,
        }
    }
}

/// Stream data format.
#[derive(Clone, Copy, Debug, Default, bytemuck::Zeroable, bytemuck::Pod)]
#[repr(C)]
pub struct Stream {
    /// Demodulated inputs `[0.i, 0.q, 1.i, 1.q]`
    pub demod: [i32; 4],
    /// Input phase after delay compensation and offset
    pub phase: Wrapping<i32>,
    /// Output frequency
    pub frequency: Wrapping<i32>,
}
