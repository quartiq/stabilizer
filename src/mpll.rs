use core::num::Wrapping;

use dsp_fixedpoint::Q32;
use dsp_process::{SplitInplace, SplitProcess};
use idsp::iir::{
    Biquad, BiquadClamp, Cascade, DirectForm, DirectForm1, pid,
    repr::BiquadRepr,
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
    lp: [DirectForm<i32, 4>; 2],
    /// PID state
    iir: DirectForm1<i32>,
    /// Current modulation phase
    phase: Wrapping<i32>,
    /// LO samples for downconverting the next batch
    lo: [[i32; BATCH_SIZE]; 2],
}

impl MpllState {
    pub fn set_frequency(&mut self, f: f32) {
        self.iir.set_y((f * UNITS.y.recip()) as _);
    }
}

/// Runtime active settings
#[derive(Debug, Clone)]
pub struct Mpll {
    lp: Cascade<[Biquad<Q32<29>>; 4]>,
    iir: BiquadClamp<Q32<30>, i32>,
    amplitude: [Q32<16>; 2],
}

#[derive(Debug, Clone, miniconf::Tree)]
#[tree(meta(doc, typename))]
pub struct MpllConfig {
    /// Lowpass filter coefficients (8th order biquad)
    #[tree(with=miniconf::leaf)]
    lp: [[f64; 5]; 4],
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
        // 8th order Gaussian to 12dB, 3dB+153τ @3.3kHz, 108dB @20kHz, gain 2
        let lp = [
            [
                7.092673331e-05 * 2.0,
                1.418534666e-04 * 2.0,
                7.092673331e-05 * 2.0,
                1.971263763e+00,
                -9.715472944e-01,
            ],
            [
                2.185730264e-04,
                4.371460527e-04,
                2.185730264e-04,
                1.972048940e+00,
                -9.729234576e-01,
            ],
            [
                4.830453545e-04,
                9.660916403e-04,
                4.830453545e-04,
                1.975378300e+00,
                -9.773108205e-01,
            ],
            [
                7.561957464e-04,
                1.512391493e-03,
                7.561957464e-04,
                1.986543883e+00,
                -9.895690493e-01,
            ],
        ];
        let mut pid = pid::Pid::default();
        pid.order = pid::Order::I;
        pid.gain.value[pid::Action::P as usize] = -3e3; // Hz/turn
        pid.gain.value[pid::Action::I as usize] = -7e6; // Hz/sturn
        // pid.gain.value[pid::Action::P as usize] = -1.2e3; // Hz/turn
        // pid.gain.value[pid::Action::I as usize] = -5e5; // Hz/sturn
        // pid.gain.value[pid::Action::D as usize] = -0.15; // sHz/turn
        // pid.limit.value[pid::Action::D as usize] = -20e3; // Hz/turn
        pid.min = 5e3; // Hz
        pid.max = 300e3; // Hz

        Self {
            lp,
            iir: BiquadRepr::Pid(pid),
            _repr: (),
            amplitude: [1.0, 1.0],
        }
    }
}

impl MpllConfig {
    pub fn build(&self) -> Mpll {
        Mpll {
            lp: Cascade(self.lp.each_ref().map(|c| Biquad::from(*c))),
            iir: self.iir.build(&UNITS),
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
            // ADC encoding, mix, 1 bit loss, 30 bit amplitude
            *mix.0.0 = ((*x.0 as i16 as i64 * *lo.0 as i64) >> 16) as i32;
            *mix.0.1 = ((*x.0 as i16 as i64 * *lo.1 as i64) >> 16) as i32;
            *mix.1.0 = ((*x.1 as i16 as i64 * *lo.0 as i64) >> 16) as i32;
            *mix.1.1 = ((*x.1 as i16 as i64 * *lo.1 as i64) >> 16) as i32;
        }
        // lowpass, 1 bit DC gain, 1 bit loss to 2f
        for (mix, state) in mix.iter_mut().zip(state.lp.iter_mut()) {
            self.lp.inplace(state, mix);
        }
        // decimate
        let demod = [
            [mix[0][BATCH_SIZE - 1], mix[1][BATCH_SIZE - 1]],
            [mix[2][BATCH_SIZE - 1], mix[3][BATCH_SIZE - 1]],
        ];
        // phase
        // need full atan2, rotation, or addtl osc to support any phase offset
        let mut phase = Wrapping(idsp::atan2(demod[0][1], demod[0][0]));
        // Delay correction
        phase += Wrapping(-10) * Wrapping(state.iir.y0());
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
    pub demod: [[i32; 2]; 2],
    /// Input phase after delay compensation and offset
    pub phase: Wrapping<i32>,
    /// Output frequency
    pub frequency: Wrapping<i32>,
}
