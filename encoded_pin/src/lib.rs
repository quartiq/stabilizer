#![no_std]

use arbitrary_int::UInt;
use core::cell::RefCell;
use embedded_hal::digital::{ErrorType, OutputPin};

pub struct EncodedPin<'a, P, const N: usize> {
    cs: &'a RefCell<[P; N]>,
    sel: UInt<u8, N>,
}

impl<'a, P, const N: usize> EncodedPin<'a, P, N> {
    pub fn new(cs: &'a RefCell<[P; N]>, sel: UInt<u8, N>) -> Self {
        assert!(sel.value() != 0);
        Self { cs, sel }
    }
}

impl<'a, P: ErrorType, const N: usize> ErrorType for EncodedPin<'a, P, N> {
    type Error = P::Error;
}

impl<'a, P: OutputPin, const N: usize> OutputPin for EncodedPin<'a, P, N> {
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
