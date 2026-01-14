use core::num::Wrapping;

use dsp_fixedpoint::Q32;
use dsp_process::{
    Add, Identity, Inplace, Pair, Parallel, Process, Split, Unsplit,
};
use idsp::iir::{
    BiquadClamp, DirectForm1, pid,
    repr::BiquadRepr,
    wdf::{Wdf, WdfState},
};
use miniconf::Tree;

pub const BATCH_SIZE: usize = 8;
pub const S_PER_SAMPLE: f32 = 128.0 / 100e6;
pub const TURN_PER_LSB: f32 = ((1u64 << 32) as f32).recip();
pub const HZ_PER_LSB: f32 = ((1u64 << 32) as f32 * S_PER_SAMPLE).recip();

#[derive(Debug, Clone, Default)]
pub struct MpllState {
    /// Lowpass state
    lp: [(((), ([WdfState<2>; 2], (WdfState<2>, WdfState<1>))), ()); 4],
    /// Phase clamp
    clamp: idsp::ClampWrap<i32>,
    /// PID state
    iir: DirectForm1<i32>,
    /// Current output phase
    phase: Wrapping<i32>,
    /// Current LO samples for downconverting the next batch
    lo: [[i32; BATCH_SIZE]; 2],
    // TODO: investigate matched LO delay: 20 samples
}

impl MpllState {
    pub fn set_frequency(&mut self, f: f32) {
        self.iir.xy[2] = (f * HZ_PER_LSB.recip()) as _;
        self.iir.xy[3] = self.iir.xy[2];
    }
}

#[derive(Debug, Clone)]
pub struct Mpll {
    lp: Lowpass,
    phase: Option<Wrapping<i32>>,
    iir: BiquadClamp<Q32<30>, i32>,
    amplitude: [Q32<16>; 2],
}

#[derive(Debug, Clone)]
struct Lowpass(Pair<[Wdf<2, 0xad>; 2], (Wdf<2, 0xad>, Wdf<1, 0xa>), i32>);

#[derive(Debug, Clone, Tree)]
#[tree(meta(doc, typename))]
pub struct MpllConfig {
    /// Lowpass filter configuration
    #[tree(skip)]
    lp: Lowpass,
    /// Input phase offset (turns) and clamp
    ///
    /// Makes the phase wrap monotonic by clamping it to aid capture/pull-in with external modulation.
    /// Disable for modulation drive and use biquad offset/pid setpoint
    phase: Option<f32>,
    /// Filter representation
    #[tree(rename="repr", typ="&str", with=miniconf::str_leaf, defer=self.iir)]
    _repr: (), // before iir
    /// Phase-to-frequency filter (units Hz/turn, s)
    ///
    /// Do not use the IIR offset as phase offset with clamp=true
    iir: BiquadRepr<f32, Q32<30>, i32>,
    /// Output amplitude (Volt)
    amplitude: [f32; 2],
}

impl Default for MpllConfig {
    fn default() -> Self {
        // 7th order Cheby2 as WDF-CA, -3 dB @ 0.005*1.28, -112 dB @ 0.018*1.28
        let lp = Pair::new((
            (
                Unsplit(&Identity),
                Parallel((
                    [
                        Wdf::quantize(&[
                            -0.9866183703960676,
                            0.9995042973541263,
                        ])
                        .unwrap(),
                        Wdf::quantize(&[-0.94357710177202, 0.9994723555364557])
                            .unwrap(),
                    ],
                    (
                        Wdf::quantize(&[
                            -0.9619459355859967,
                            0.9994905727027024,
                        ])
                        .unwrap(),
                        Wdf::quantize(&[0.9677764552414969]).unwrap(),
                    ),
                )),
            ),
            Unsplit(&Add),
        ));

        let mut pid = pid::Pid::default();
        pid.order = pid::Order::I;
        pid.gains.value[pid::Action::P as usize] = -3.9e3; // Hz/turn
        pid.gains.value[pid::Action::I as usize] = -3.8e6; // Hz/sturn
        pid.gains.value[pid::Action::D as usize] = -0.25; // sHz/turn
        pid.limits.value[pid::Action::D as usize] = -150e3; // Hz/turn
        pid.min = 5e3; // Hz
        pid.max = 300e3; // Hz

        Self {
            lp: Lowpass(lp),
            phase: Some(0.0),
            iir: BiquadRepr::Pid(pid),
            _repr: (),
            amplitude: [1.0, 0.0], // V
        }
    }
}

impl MpllConfig {
    pub fn build(&self) -> Mpll {
        Mpll {
            lp: self.lp.clone(),
            phase: self
                .phase
                .map(|p| Wrapping((p * TURN_PER_LSB.recip()) as _)),
            iir: self.iir.build(pid::Units {
                t: BATCH_SIZE as f32 * S_PER_SAMPLE,
                x: TURN_PER_LSB,
                y: HZ_PER_LSB,
            }),
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
                .zip(m01.iter_mut())
                .zip(m10.iter_mut().zip(m11.iter_mut()))
                .zip(state.lo[0].iter().zip(state.lo[1].iter())),
        ) {
            // ADC encoding, mix
            *mix.0.0 = ((*x.0 as i16 as i64 * *lo.0 as i64) >> 16) as i32;
            *mix.0.1 = ((*x.0 as i16 as i64 * *lo.1 as i64) >> 16) as i32;
            *mix.1.0 = ((*x.1 as i16 as i64 * *lo.0 as i64) >> 16) as i32;
            *mix.1.1 = ((*x.1 as i16 as i64 * *lo.1 as i64) >> 16) as i32;
        }
        // lowpass
        for (mix, state) in mix.iter_mut().zip(state.lp.iter_mut()) {
            Split::new(&self.lp.0, state).inplace(mix);
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
        let mut p = Wrapping(idsp::atan2(demod[1], demod[0]));
        // Delay correction
        p -= Wrapping(state.iir.xy[2]) * Wrapping(10);
        // Offset and clamp
        if let Some(offset) = self.phase {
            p.0 = state.clamp.process((p + offset).0);
        }
        // PID
        let f = Wrapping(Split::new(&self.iir, &mut state.iir).process(p.0));
        // modulate
        let [y0, y1] = state.lo.each_mut();
        let [yo0, yo1] = y;
        state.phase = y0
            .iter_mut()
            .zip(y1)
            .zip((*yo0).iter_mut().zip((*yo1).iter_mut()))
            .fold(state.phase, |mut p, (y, yo)| {
                p += f;
                (*y.0, *y.1) = idsp::cossin(p.0);
                *yo.0 = ((((*y.0 >> 16) * self.amplitude[0].inner) >> 16)
                    as i16)
                    .wrapping_add(i16::MIN) as u16; // DAC encoding
                *yo.1 = ((((*y.1 >> 16) * self.amplitude[1].inner) >> 16)
                    as i16)
                    .wrapping_add(i16::MIN) as u16; // DAC encoding
                p
            });
        Stream {
            demod,
            phase: p,
            frequency: f,
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
