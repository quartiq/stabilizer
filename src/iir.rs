use core::f32;

pub type IIRState = [f32; 5];

#[derive(Default,Copy,Clone,Debug)]
pub struct IIR {
    pub x_offset: f32,
    pub y_offset: f32,
    pub ba: IIRState,
    pub scale: f32,
}

impl IIR {
    pub fn pi(&mut self, kp: f32, ki: f32, g: f32) -> Result<(), &str> {
        if ki < 0. {
            return Err("ki < 0");
        }
        let (a1, b1, b0) = match () {
            _ if ki < 0. => return Err("ki < 0"),
            _ if ki < f32::EPSILON => (0., 0., kp),
            _ => {
                let (c, a1) = match () {
                    _ if g < 0. => return Err("g < 0"),
                    _ if g < f32::EPSILON => (1., 1.),
                    _ => {
                        let c = 1./(1. + ki/g);
                        (c, 2.*c - 1.)
                    }
                };
                let b0 = kp + ki*c;
                let b1 = b0 - 2.*kp*c;
                if b0 + b1 > -f32::EPSILON && b0 + b1 < f32::EPSILON {
                    return Err("low integrator gain and/or gain limit");
                }
                (a1, b1, b0)
            }
        };
        if b0 > 1. || b0 < -1. || b1 > 1. || b1 < -1. {
            return Err("high gains");
        }
        self.ba[0] = b0;
        self.ba[1] = b1;
        self.ba[2] = 0.;
        self.ba[3] = a1;
        self.ba[4] = 0.;
        Ok(())
    }

    pub fn update(&self, xy: &mut IIRState, x0: f32) -> f32 {
        xy.rotate_right(1);
        xy[0] = x0 + self.x_offset;
        let y0 = macc(self.y_offset, xy, &self.ba)
            .min(self.scale).max(-self.scale);
        xy[xy.len()/2] = y0;
        y0
    }
}

fn macc(y0: f32, x: &[f32], a: &[f32]) -> f32 {
    y0 + x.iter().zip(a.iter())
        .map(|(&i, &j)| i * j).sum::<f32>()
}
