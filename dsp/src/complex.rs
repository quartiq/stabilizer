use super::{atan2, cossin};

#[derive(Copy, Clone, Default, PartialEq, Debug)]
pub struct Complex<T>(pub T, pub T);

impl Complex<i32> {
    /// Return a Complex on the unit circle given an angle.
    ///
    /// Example:
    ///
    /// ```
    /// use dsp::Complex;
    /// Complex::<i32>::from_angle(0);
    /// Complex::<i32>::from_angle(1 << 30);  // pi/2
    /// Complex::<i32>::from_angle(-1 << 30);  // -pi/2
    /// ```
    pub fn from_angle(angle: i32) -> Self {
        let (c, s) = cossin(angle);
        Self(c, s)
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
    /// use dsp::Complex;
    /// assert_eq!(Complex(i32::MIN, 0).abs_sqr(), 1 << 31);
    /// assert_eq!(Complex(i32::MAX, i32::MAX).abs_sqr(), u32::MAX - 3);
    /// ```
    pub fn abs_sqr(&self) -> u32 {
        (((self.0 as i64) * (self.0 as i64)
            + (self.1 as i64) * (self.1 as i64))
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
    /// use dsp::Complex;
    /// assert_eq!(Complex(i32::MAX, i32::MAX).log2(), -1);
    /// assert_eq!(Complex(i32::MAX, 0).log2(), -2);
    /// assert_eq!(Complex(1, 0).log2(), -63);
    /// assert_eq!(Complex(0, 0).log2(), -64);
    /// ```
    pub fn log2(&self) -> i32 {
        let a = (self.0 as i64) * (self.0 as i64)
            + (self.1 as i64) * (self.1 as i64);
        -(a.leading_zeros() as i32)
    }

    /// Return the angle.
    ///
    /// Note: Normalization is `1 << 31 == pi`.
    ///
    /// Example:
    ///
    /// ```
    /// use dsp::Complex;
    /// assert_eq!(Complex(1, 0).arg(), 0);
    /// assert_eq!(Complex(-i32::MAX, 1).arg(), i32::MAX);
    /// assert_eq!(Complex(-i32::MAX, -1).arg(), -i32::MAX);
    /// assert_eq!(Complex(0, -1).arg(), -i32::MAX >> 1);
    /// assert_eq!(Complex(0, 1).arg(), (i32::MAX >> 1) + 1);
    /// assert_eq!(Complex(1, 1).arg(), (i32::MAX >> 2) + 1);
    /// ```
    pub fn arg(&self) -> i32 {
        atan2(self.1, self.0)
    }
}
