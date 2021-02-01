#[derive(Copy, Clone, Default, PartialEq, Debug)]
pub struct Accu {
    state: i32,
    step: i32,
}

impl Accu {
    pub fn new(state: i32, step: i32) -> Self {
        Self { state, step }
    }
}

impl Iterator for Accu {
    type Item = i32;
    #[inline]
    fn next(&mut self) -> Option<i32> {
        let s = self.state;
        self.state = self.state.wrapping_add(self.step);
        Some(s)
    }
}
