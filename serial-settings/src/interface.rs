/// Wrapper type for a "best effort" serial interface.
///
/// # Note
/// Overflows of the output are silently ignored.
pub struct BestEffortInterface<T>(T);

impl<T> BestEffortInterface<T>
where
    T: embedded_io::Write
        + embedded_io::WriteReady
        + embedded_io::Read
        + embedded_io::ReadReady,
{
    /// Construct an interface where overflows and errors when writing on the output are silently
    /// ignored.
    pub fn new(interface: T) -> Self {
        Self(interface)
    }

    /// Get access to the inner (wrapped) interface
    pub fn inner(&self) -> &T {
        &self.0
    }

    /// Get mutable access to the inner (wrapped) interface
    pub fn inner_mut(&mut self) -> &mut T {
        &mut self.0
    }
}

impl<T> embedded_io::Write for BestEffortInterface<T>
where
    T: embedded_io::Write + embedded_io::WriteReady,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        if let Ok(true) = self.0.write_ready() {
            self.0.write(buf).ok();
        }
        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.0.flush()
    }
}

impl<T> embedded_io::ErrorType for BestEffortInterface<T>
where
    T: embedded_io::ErrorType,
{
    type Error = <T as embedded_io::ErrorType>::Error;
}

impl<T> embedded_io::Read for BestEffortInterface<T>
where
    T: embedded_io::Read,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.0.read(buf)
    }
}

impl<T> embedded_io::ReadReady for BestEffortInterface<T>
where
    T: embedded_io::ReadReady,
{
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        self.0.read_ready()
    }
}
