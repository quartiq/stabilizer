pub type IIRState = [f32; 5];

pub struct IIR {
    pub y0: f32,
    pub ba: IIRState,
    pub scale: f32,
}

impl IIR {
    pub fn update(&self, xy: &mut IIRState, x0: f32) -> f32 {
        xy.rotate_right(1);
        xy[0] = x0;
        let y0 = macc(self.y0, xy, &self.ba, self.scale);
        xy[xy.len()/2] = y0;
        y0
    }
}

fn macc(y0: f32, x: &[f32], a: &[f32], scale: f32) -> f32 {
    y0 + x.iter().zip(a.iter())
        .map(|(&i, &j)| i * j).sum::<f32>()
        .min(scale).max(-scale)
}
