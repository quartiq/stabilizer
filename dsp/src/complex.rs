pub use num::Complex;

use super::{atan2, cossin};

pub trait Map<F> {
    fn map(&self, func: F) -> Self;
}

impl<F: Fn(T) -> T, T: Copy> Map<F> for Complex<T> {
    fn map(&self, func: F) -> Self {
        Complex {
            re: func(self.re),
            im: func(self.im),
        }
    }
}

pub trait FastInt<T, U> {
    fn from_angle(angle: T) -> Self;
    fn abs_sqr(&self) -> U;
    fn log2(&self) -> T;
    fn arg(&self) -> T;
}

impl FastInt<i32, u32> for Complex<i32> {
    /// Return a Complex on the unit circle given an angle.
    ///
    /// Example:
    ///
    /// ```
    /// use dsp::{Complex, FastInt};
    /// Complex::<i32>::from_angle(0);
    /// Complex::<i32>::from_angle(1 << 30);  // pi/2
    /// Complex::<i32>::from_angle(-1 << 30);  // -pi/2
    /// ```
    fn from_angle(angle: i32) -> Self {
        let (c, s) = cossin(angle);
        Self { re: c, im: s }
    }

    /// Return the absolute square (the squared magnitude).
    ///
    /// Note: Normalization is `1 << 32`, i.e. U0.32.
    ///
    /// Note(panic): This will panic for `Complex(i32::MIN, i32::MIN)`
    ///
    /// Example:
    ///
    /// ```
    /// use dsp::{Complex, FastInt};
    /// assert_eq!(Complex::new(i32::MIN, 0).abs_sqr(), 1 << 31);
    /// assert_eq!(Complex::new(i32::MAX, i32::MAX).abs_sqr(), u32::MAX - 3);
    /// ```
    fn abs_sqr(&self) -> u32 {
        (((self.re as i64) * (self.re as i64)
            + (self.im as i64) * (self.im as i64))
            >> 31) as u32
    }

    /// log2(power) re full scale approximation
    ///
    /// TODO: scale up, interpolate
    ///
    /// Panic:
    /// This will panic for `Complex(i32::MIN, i32::MIN)`
    ///
    /// Example:
    ///
    /// ```
    /// use dsp::{Complex, FastInt};
    /// assert_eq!(Complex::new(i32::MAX, i32::MAX).log2(), -1);
    /// assert_eq!(Complex::new(i32::MAX, 0).log2(), -2);
    /// assert_eq!(Complex::new(1, 0).log2(), -63);
    /// assert_eq!(Complex::new(0, 0).log2(), -64);
    /// ```
    fn log2(&self) -> i32 {
        let a = (self.re as i64) * (self.re as i64)
            + (self.im as i64) * (self.im as i64);
        -(a.leading_zeros() as i32)
    }

    /// Return the angle.
    ///
    /// Note: Normalization is `1 << 31 == pi`.
    ///
    /// Example:
    ///
    /// ```
    /// use dsp::{Complex, FastInt};
    /// assert_eq!(Complex::new(1, 0).arg(), 0);
    /// assert_eq!(Complex::new(-i32::MAX, 1).arg(), i32::MAX);
    /// assert_eq!(Complex::new(-i32::MAX, -1).arg(), -i32::MAX);
    /// assert_eq!(Complex::new(0, -1).arg(), -i32::MAX >> 1);
    /// assert_eq!(Complex::new(0, 1).arg(), (i32::MAX >> 1) + 1);
    /// assert_eq!(Complex::new(1, 1).arg(), (i32::MAX >> 2) + 1);
    /// ```
    fn arg(&self) -> i32 {
        atan2(self.im, self.re)
    }
}

pub trait MulScaled<T> {
    fn mul_scaled(self, other: T) -> Self;
}

impl MulScaled<Complex<i32>> for Complex<i32> {
    fn mul_scaled(self, other: Self) -> Self {
        let a = self.re as i64;
        let b = self.im as i64;
        let c = other.re as i64;
        let d = other.im as i64;
        Complex {
            re: ((a * c - b * d + (1 << 31)) >> 32) as i32,
            im: ((b * c + a * d + (1 << 31)) >> 32) as i32,
        }
    }
}

impl MulScaled<i32> for Complex<i32> {
    fn mul_scaled(self, other: i32) -> Self {
        Complex {
            re: ((other as i64 * self.re as i64 + (1 << 31)) >> 32) as i32,
            im: ((other as i64 * self.im as i64 + (1 << 31)) >> 32) as i32,
        }
    }
}

impl MulScaled<i16> for Complex<i32> {
    fn mul_scaled(self, other: i16) -> Self {
        Complex {
            re: (other as i32 * (self.re >> 16) + (1 << 15)) >> 16,
            im: (other as i32 * (self.im >> 16) + (1 << 15)) >> 16,
        }
    }
}
