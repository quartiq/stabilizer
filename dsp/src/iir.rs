use core::ops::{Add, Mul, Neg};
use serde::{Deserialize, Serialize};

use core::f32;

// These are implemented here because core::f32 doesn't have them (yet).
// They are naive and don't handle inf/nan.
// `compiler-intrinsics`/llvm should have better (robust, universal, and
// faster) implementations.

fn abs<T>(x: T) -> T
where
    T: PartialOrd + Default + Neg<Output = T>,
{
    if x >= T::default() {
        x
    } else {
        -x
    }
}

fn copysign<T>(x: T, y: T) -> T
where
    T: PartialOrd + Default + Neg<Output = T>,
{
    if (x >= T::default() && y >= T::default())
        || (x <= T::default() && y <= T::default())
    {
        x
    } else {
        -x
    }
}

#[cfg(not(feature = "nightly"))]
fn max<T>(x: T, y: T) -> T
where
    T: PartialOrd,
{
    if x > y {
        x
    } else {
        y
    }
}

#[cfg(not(feature = "nightly"))]
fn min<T>(x: T, y: T) -> T
where
    T: PartialOrd,
{
    if x < y {
        x
    } else {
        y
    }
}

#[cfg(feature = "nightly")]
fn max(x: f32, y: f32) -> f32 {
    core::intrinsics::maxnumf32(x, y)
}

#[cfg(feature = "nightly")]
fn min(x: f32, y: f32) -> f32 {
    core::intrinsics::minnumf32(x, y)
}

// Multiply-accumulate vectors `x` and `a`.
//
// A.k.a. dot product.
// Rust/LLVM optimize this nicely.
fn macc<T>(y0: T, x: &[T], a: &[T]) -> T
where
    T: Add<Output = T> + Mul<Output = T> + Copy,
{
    x.iter()
        .zip(a)
        .map(|(x, a)| *x * *a)
        .fold(y0, |y, xa| y + xa)
}

/// IIR state and coefficients type.
///
/// To represent the IIR state (input and output memory) during the filter update
/// this contains the three inputs (x0, x1, x2) and the two outputs (y1, y2)
/// concatenated. Lower indices correspond to more recent samples.
/// To represent the IIR coefficients, this contains the feed-forward
/// coefficients (b0, b1, b2) followd by the negated feed-back coefficients
/// (-a1, -a2), all five normalized such that a0 = 1.
pub type IIRState = [f32; 5];

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
#[derive(Copy, Clone, Deserialize, Serialize)]
pub struct IIR {
    pub ba: IIRState,
    pub y_offset: f32,
    pub y_min: f32,
    pub y_max: f32,
}

impl IIR {
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
    pub fn update(&self, xy: &mut IIRState, x0: f32) -> f32 {
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
        let y0 = macc(self.y_offset, xy, &self.ba);
        // Limit y0
        let y0 = max(self.y_min, min(self.y_max, y0));
        // Store y0            x0 x1 y0 y1 y2
        xy[n / 2] = y0;
        y0
    }
}
