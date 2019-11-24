use core::ops::{Add, Mul};
use serde::{Deserialize, Serialize};

use core::f32;

pub type IIRState = [f32; 5];

#[derive(Copy, Clone, Deserialize, Serialize)]
pub struct IIR {
    pub ba: IIRState,
    pub y_offset: f32,
    pub y_min: f32,
    pub y_max: f32,
}

fn abs(x: f32) -> f32 {
    if x >= 0. {
        x
    } else {
        -x
    }
}

fn copysign(x: f32, y: f32) -> f32 {
    if (x >= 0. && y >= 0.) || (x <= 0. && y <= 0.) {
        x
    } else {
        -x
    }
}

fn max(x: f32, y: f32) -> f32 {
    if x > y {
        x
    } else {
        y
    }
}

fn min(x: f32, y: f32) -> f32 {
    if x < y {
        x
    } else {
        y
    }
}

fn macc<T>(y0: T, x: &[T], a: &[T]) -> T
where
    T: Add<Output = T> + Mul<Output = T> + Copy,
{
    x.iter()
        .zip(a.iter())
        .map(|(&i, &j)| i * j)
        .fold(y0, |y, xa| y + xa)
}

impl IIR {
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
        self.ba[0] = b0;
        self.ba[1] = b1;
        self.ba[2] = 0.;
        self.ba[3] = a1;
        self.ba[4] = 0.;
        Ok(())
    }

    pub fn get_x_offset(&self) -> Result<f32, &str> {
        let b: f32 = self.ba[..3].iter().sum();
        if abs(b) < f32::EPSILON {
            Err("b is zero")
        } else {
            Ok(self.y_offset / b)
        }
    }

    pub fn set_x_offset(&mut self, xo: f32) {
        let b: f32 = self.ba[..3].iter().sum();
        self.y_offset = xo * b;
    }

    pub fn update(&self, xy: &mut IIRState, x0: f32) -> f32 {
        xy.rotate_right(1);
        xy[0] = x0;
        let y0 = macc(self.y_offset, xy, &self.ba);
        let y0 = max(self.y_min, min(self.y_max, y0));
        xy[xy.len() / 2] = y0;
        y0
    }
}
