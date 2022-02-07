///! System timer used for non-RTIC compatibility
///!
///! # Design
///!  `Clock` is implemented using the RTIC `app::monotonics::now()` default `Monotonic`.
///!  That `Monotonic` must tick at 1 kHz.
use minimq::embedded_time::{clock::Error, fraction::Fraction, Clock, Instant};

#[derive(Copy, Clone, Debug)]
pub struct SystemTimer(fn() -> u64);

impl SystemTimer {
    pub fn new(now: fn() -> u64) -> Self {
        Self(now)
    }
}

impl Clock for SystemTimer {
    type T = u32;

    // The duration of each tick in seconds.
    const SCALING_FACTOR: Fraction = Fraction::new(1, 1_000);

    fn try_now(&self) -> Result<Instant<Self>, Error> {
        Ok(Instant::new((self.0)() as _))
    }
}
