use dsp_fixedpoint::Q32;
use dsp_process::{
    Add, Identity, Inplace, Pair, Parallel, Process, Split, Unsplit,
};
use idsp::iir::{
    BiquadClamp, DirectForm1,
    wdf::{Wdf, WdfState},
};
use miniconf::Tree;

pub const BATCH_SIZE: usize = 8;

#[derive(Debug, Clone, Default)]
pub struct MpllState {
    /// Lowpass state
    lp: [(((), ([WdfState<2>; 2], (WdfState<2>, WdfState<1>))), ()); 4],
    /// Phase clamp
    ///
    /// Makes the phase wrap monotonic by clamping it.
    /// Aids capture/pull-in with external modulation.
    ///
    /// TODO: Remove this for modulation drive.
    clamp: idsp::ClampWrap<i32>,
    /// PID state
    pub iir: DirectForm1<i32>,
    /// Current output phase
    phase: i32,
    /// Current LO samples for downconverting the next batch
    lo: [[i32; BATCH_SIZE]; 2],
    // TODO: investigate matched LO delay: 20 samples
}

#[derive(Debug, Clone, Tree)]
#[tree(meta(doc, typename))]
pub struct Mpll {
    /// Lowpass
    #[tree(skip)]
    lp: Pair<[Wdf<2, 0xad>; 2], (Wdf<2, 0xad>, Wdf<1, 0xa>), i32>,
    /// Input phase offset
    phase: i32,
    /// PID IIR filter
    ///
    /// Do not use the iir offset as phase offset.
    /// Includes frequency limits.
    #[tree(skip)]
    iir: BiquadClamp<Q32<30>, i32>,
    /// Output amplitude scale
    amp: [i32; 2],
}

impl Mpll {
    pub fn new(config: &Self) -> Self {
        config.clone()
    }
}

impl Default for Mpll {
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

        let mut iir: BiquadClamp<_, _> = idsp::iir::pid::Builder::default()
            .order(idsp::iir::pid::Order::I)
            .period(1.0 / BATCH_SIZE as f32)
            .gain(idsp::iir::pid::Action::P, -5e-3) // fs/turn
            .gain(idsp::iir::pid::Action::I, -4e-4) // fs/turn/ts = 1/turn
            .gain(idsp::iir::pid::Action::D, -4e-3) // fs/turn*ts = turn
            .limit(idsp::iir::pid::Action::D, -0.2)
            .into();
        iir.max = (0.3 * (1u64 << 32) as f32) as _;
        iir.min = (0.005 * (1u64 << 32) as f32) as _;

        Self {
            lp,
            phase: (0.0 * (1u64 << 32) as f32) as _,
            iir,
            amp: [(0.09 * (1u64 << 31) as f32) as _; 2], // ~0.9 V
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
    ) -> (Stream, i32) {
        // scratch
        let mut mix = [[0; BATCH_SIZE]; 4];
        let [m00, m01, m10, m11] = mix.each_mut();
        // mix
        for (x, (mix, lo)) in x[0].iter().zip(x[1].iter()).zip(
            m00.iter_mut()
                .zip(m01.iter_mut())
                .zip(m10.iter_mut().zip(m11.iter_mut()))
                .zip(state.lo[0].iter().zip(&state.lo[1])),
        ) {
            // ADC encoding, mix
            *mix.0.0 = ((*x.0 as i16 as i64 * *lo.0 as i64) >> 16) as i32;
            *mix.0.1 = ((*x.0 as i16 as i64 * *lo.1 as i64) >> 16) as i32;
            *mix.1.0 = ((*x.1 as i16 as i64 * *lo.0 as i64) >> 16) as i32;
            *mix.1.1 = ((*x.1 as i16 as i64 * *lo.1 as i64) >> 16) as i32;
        }
        // lowpass
        for (mix, state) in mix.iter_mut().zip(state.lp.iter_mut()) {
            Split::new(&self.lp, state).inplace(mix);
        }
        // decimate
        let demod = [
            mix[0][BATCH_SIZE - 1],
            mix[1][BATCH_SIZE - 1],
            mix[2][BATCH_SIZE - 1],
            mix[3][BATCH_SIZE - 1],
        ];
        // phase
        // need full atan2 or addtl osc to support any phase offset
        let p = idsp::atan2(demod[1], demod[0]);
        // Delay correction
        let p = p
            .wrapping_add(self.phase)
            .wrapping_sub(state.iir.xy[2].wrapping_mul(10));
        let p = state.clamp.process(p);
        // pid
        let f = Split::new(&self.iir, &mut state.iir).process(p);
        // modulate
        let [y0, y1] = state.lo.each_mut();
        let [yo0, yo1] = y;
        state.phase = y0
            .iter_mut()
            .zip(y1)
            .zip((*yo0).iter_mut().zip((*yo1).iter_mut()))
            .fold(state.phase, |mut p, (y, yo)| {
                p = p.wrapping_add(f);
                (*y.0, *y.1) = idsp::cossin(p);
                *yo.0 = (((*y.0 as i64 * self.amp[0] as i64) >> (31 + 31 - 15))
                    as i16)
                    .wrapping_add(i16::MIN) as u16; // DAC encoding
                *yo.1 = (((*y.1 as i64 * self.amp[1] as i64) >> (31 + 31 - 15))
                    as i16)
                    .wrapping_add(i16::MIN) as u16; // DAC encoding
                p
            });
        (
            Stream {
                demod,
                phase_in: p,
                phase_out: state.phase,
            },
            f,
        )
    }
}

/// Stream data format.
#[derive(Clone, Copy, Debug, Default, bytemuck::Zeroable, bytemuck::Pod)]
#[repr(C)]
pub struct Stream {
    pub demod: [i32; 4],
    pub phase_in: i32,  // after clamping and offset
    pub phase_out: i32, // derivative is output frequency
}
