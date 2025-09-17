use crate::convert::Gain;

/// A programmable gain amplifier that allows for setting the gain via GPIO.
pub struct ProgrammableGainAmplifier<P> {
    a: [P; 2],
}

impl<P> ProgrammableGainAmplifier<P>
where
    P: embedded_hal_1::digital::OutputPin,
{
    /// Construct a new programmable gain driver.
    ///
    /// Args:
    /// * `a0` - An output connected to the A0 input of the amplifier.
    /// * `a1` - An output connected to the A1 input of the amplifier.
    pub fn new(a: [P; 2]) -> Self {
        let mut afe = Self { a };
        afe.set_gain(Gain::default());
        afe
    }

    /// Set the gain of the front-end.
    pub fn set_gain(&mut self, gain: Gain) {
        let mut gain = gain as u8;
        for a in self.a.iter_mut() {
            a.set_state(((gain & 1) != 0).into()).unwrap();
            gain >>= 1;
        }
    }
}
