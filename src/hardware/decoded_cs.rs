use arbitrary_int::UInt;
use core::cell::RefCell;
use embedded_hal_1::digital::{ErrorType, OutputPin};

pub struct DecodedCs<'a, CS, const N: usize> {
    cs: &'a RefCell<[CS; N]>,
    sel: UInt<u8, N>,
}

impl<'a, CS, const N: usize> DecodedCs<'a, CS, N> {
    pub fn new(cs: &'a RefCell<[CS; N]>, sel: UInt<u8, N>) -> Self {
        assert!(sel.value() != 0);
        Self { cs, sel }
    }
}

impl<'a, CS: ErrorType, const N: usize> ErrorType for DecodedCs<'a, CS, N> {
    type Error = CS::Error;
}

impl<'a, CS: OutputPin, const N: usize> OutputPin for DecodedCs<'a, CS, N> {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        // assert
        for (i, cs) in self.cs.borrow_mut().iter_mut().enumerate() {
            if self.sel.value() & (1 << i) != 0 {
                cs.set_high()?;
            }
        }
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        // deassert
        for (i, cs) in self.cs.borrow_mut().iter_mut().enumerate() {
            if self.sel.value() & (1 << i) != 0 {
                cs.set_low()?;
            }
        }
        Ok(())
    }
}
