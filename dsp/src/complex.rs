use super::atan2;
use serde::{Deserialize, Serialize};

#[derive(Copy, Clone, Default, Deserialize, Serialize)]
pub struct Complex<T>(pub T, pub T);

impl Complex<i32> {
    pub fn power(&self) -> i32 {
        (((self.0 as i64) * (self.0 as i64)
            + (self.1 as i64) * (self.1 as i64))
            >> 32) as i32
    }

    pub fn phase(&self) -> i32 {
        atan2(self.1, self.0)
    }
}
