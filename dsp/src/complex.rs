use core::cmp::PartialEq;

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Complex {
    pub re: f32,
    pub im: f32,
}

impl Complex {
    pub fn new(re: f32, im: f32) -> Self {
        Complex { re: re, im: im }
    }

    pub fn arg(&self) -> f32 {
        libm::atan2f(self.im, self.re)
    }

    pub fn abs(&self) -> f32 {
        libm::sqrtf([self.re, self.im].iter().map(|i| i * i).sum())
    }
}
