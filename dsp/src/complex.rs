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
    #[inline(always)]
    pub fn from_angle(angle: i32) -> Self {
        let (c, s) = cossin(angle);
        Self(c, s)
    }

    /// Return the absolute square (the squared magnitude).
    ///
    /// Note: Normalization is `1 << 31`, i.e. Q0.31.
    ///
    /// Example:
    ///
    /// ```
    /// use dsp::Complex;
    /// assert_eq!(Complex(i32::MAX, 0).abs_sqr(), i32::MAX - 1);
    /// assert_eq!(Complex(i32::MIN + 1, 0).abs_sqr(), i32::MAX - 1);
    /// ```
    pub fn abs_sqr(&self) -> i32 {
        (((self.0 as i64) * (self.0 as i64)
            + (self.1 as i64) * (self.1 as i64))
            >> 31) as i32
    }

    /// Return the angle.
    ///
    /// Note: Normalization is `1 << 31 == pi`.
    ///
    /// Example:
    ///
    /// ```
    /// use dsp::Complex;
    /// assert_eq!(Complex(i32::MAX, 0).arg(), 0);
    /// assert_eq!(Complex(-i32::MAX, 1).arg(), i32::MAX);
    /// assert_eq!(Complex(-i32::MAX, -1).arg(), -i32::MAX);
    /// assert_eq!(Complex(0, -i32::MAX).arg(), -i32::MAX >> 1);
    /// assert_eq!(Complex(0, i32::MAX).arg(), (i32::MAX >> 1) + 1);
    /// assert_eq!(Complex(i32::MAX, i32::MAX).arg(), (i32::MAX >> 2) + 1);
    /// ```
    pub fn arg(&self) -> i32 {
        atan2(self.1, self.0)
    }
}
