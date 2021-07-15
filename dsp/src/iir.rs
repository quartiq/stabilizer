use miniconf::MiniconfAtomic;
use serde::Deserialize;

use super::{abs, copysign, macc, max, min};
use core::f32;

/// IIR state and coefficients type.
///
/// To represent the IIR state (input and output memory) during the filter update
/// this contains the three inputs (x0, x1, x2) and the two outputs (y1, y2)
/// concatenated. Lower indices correspond to more recent samples.
/// To represent the IIR coefficients, this contains the feed-forward
/// coefficients (b0, b1, b2) followd by the negated feed-back coefficients
/// (-a1, -a2), all five normalized such that a0 = 1.
pub type Vec5 = [f32; 5];

/// IIR configuration.
///
/// Contains the coeeficients `ba`, the output offset `y_offset`, and the
/// output limits `y_min` and `y_max`.
///
/// This implementation achieves several important properties:
///
/// * Its transfer function is universal in the sense that any biquadratic
///   transfer function can be implemented (high-passes, gain limits, second
///   order integrators with inherent anti-windup, notches etc) without code
///   changes preserving all features.
/// * It inherits a universal implementation of "integrator anti-windup", also
///   and especially in the presence of set-point changes and in the presence
///   of proportional or derivative gain without any back-off that would reduce
///   steady-state output range.
/// * It has universal derivative-kick (undesired, unlimited, and un-physical
///   amplification of set-point changes by the derivative term) avoidance.
/// * An offset at the input of an IIR filter (a.k.a. "set-point") is
///   equivalent to an offset at the output. They are related by the
///   overall (DC feed-forward) gain of the filter.
/// * It stores only previous outputs and inputs. These have direct and
///   invariant interpretation (independent of gains and offsets).
///   Therefore it can trivially implement bump-less transfer.
/// * Cascading multiple IIR filters allows stable and robust
///   implementation of transfer functions beyond bequadratic terms.
///
/// # Miniconf
///
/// `{"y_offset": y0, "y_min": ym, "y_max": yM, "ba": [b0, b1, b2, -a1, -a2]}`
///
/// * `y0` is the output offset code
/// * `ym` is the lower saturation limit
/// * `yM` is the upper saturation limit
///
/// IIR filter tap gains (`ba`) are an array `[b0, b1, b2, a1, a2]` such that the
/// new output is computed as `y0 = a1*y1 + a2*y2 + b0*x0 + b1*x1 + b2*x2`.
/// The IIR coefficients can be mapped to other transfer function
/// representations, for example as described in <https://arxiv.org/abs/1508.06319>
///
///
/// ## Notes
/// The units of the IIR utilize 16-bit signed integers for full-scale. saturation and offset
/// parameter are given in this scale, where full-scale represents an output amplitude of 10.24
/// V.
#[derive(Copy, Clone, Debug, Default, Deserialize, MiniconfAtomic)]
pub struct IIR {
    pub ba: Vec5,
    pub y_offset: f32,
    pub y_min: f32,
    pub y_max: f32,
}

impl IIR {
    pub const fn new(gain: f32, y_min: f32, y_max: f32) -> Self {
        Self {
            ba: [gain, 0., 0., 0., 0.],
            y_offset: 0.,
            y_min,
            y_max,
        }
    }

    /// Configures IIR filter coefficients for proportional-integral behavior
    /// with gain limit.
    ///
    /// # Arguments
    ///
    /// * `kp` - Proportional gain. Also defines gain sign.
    /// * `ki` - Integral gain at Nyquist. Sign taken from `kp`.
    /// * `g` - Gain limit.
    pub fn set_pi(&mut self, kp: f32, ki: f32, g: f32) -> Result<(), &str> {
        let ki = copysign(ki, kp);
        let g = copysign(g, kp);
        let (a1, b0, b1) = if abs(ki) < f32::EPSILON {
            (0., kp, 0.)
        } else {
            let c = if abs(g) < f32::EPSILON {
                1.
            } else {
                1. / (1. + ki / g)
            };
            let a1 = 2. * c - 1.;
            let b0 = ki * c + kp;
            let b1 = ki * c - a1 * kp;
            if abs(b0 + b1) < f32::EPSILON {
                return Err("low integrator gain and/or gain limit");
            }
            (a1, b0, b1)
        };
        self.ba.copy_from_slice(&[b0, b1, 0., a1, 0.]);
        Ok(())
    }

    /// Compute the overall (DC feed-forward) gain.
    pub fn get_k(&self) -> f32 {
        self.ba[..3].iter().sum()
    }

    /// Compute input-referred (`x`) offset from output (`y`) offset.
    pub fn get_x_offset(&self) -> Result<f32, &str> {
        let k = self.get_k();
        if abs(k) < f32::EPSILON {
            Err("k is zero")
        } else {
            Ok(self.y_offset / k)
        }
    }

    /// Convert input (`x`) offset to equivalent output (`y`) offset and apply.
    ///
    /// # Arguments
    /// * `xo`: Input (`x`) offset.
    pub fn set_x_offset(&mut self, xo: f32) {
        self.y_offset = xo * self.get_k();
    }

    /// Feed a new input value into the filter, update the filter state, and
    /// return the new output. Only the state `xy` is modified.
    ///
    /// # Arguments
    /// * `xy` - Current filter state.
    /// * `x0` - New input.
    pub fn update(&self, xy: &mut Vec5, x0: f32, hold: bool) -> f32 {
        let n = self.ba.len();
        debug_assert!(xy.len() == n);
        // `xy` contains       x0 x1 y0 y1 y2
        // Increment time      x1 x2 y1 y2 y3
        // Shift               x1 x1 x2 y1 y2
        // This unrolls better than xy.rotate_right(1)
        xy.copy_within(0..n - 1, 1);
        // Store x0            x0 x1 x2 y1 y2
        xy[0] = x0;
        // Compute y0 by multiply-accumulate
        let y0 = if hold {
            xy[n / 2 + 1]
        } else {
            macc(self.y_offset, xy, &self.ba)
        };
        // Limit y0
        let y0 = max(self.y_min, min(self.y_max, y0));
        // Store y0            x0 x1 y0 y1 y2
        xy[n / 2] = y0;
        y0
    }
}
