use super::UsbBus;

pub struct SerialInterface {
    usb_device: usb_device::device::UsbDevice<'static, UsbBus>,
    usb_serial: usbd_serial::SerialPort<'static, UsbBus>,
}

#[derive(Debug)]
pub struct Error(usbd_serial::UsbError);

impl From<usbd_serial::UsbError> for Error {
    fn from(e: usbd_serial::UsbError) -> Self {
        Self(e)
    }
}

impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

impl embedded_io::ErrorType for SerialInterface {
    type Error = Error;
}

impl embedded_io::Read for SerialInterface {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.usb_serial.read(buf).map_err(From::from)
    }
}

impl embedded_io::ReadReady for SerialInterface {
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(true)
    }
}

impl embedded_io::Write for SerialInterface {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.usb_serial.write(buf).map_err(From::from)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        unimplemented!()
    }
}

impl serial_settings::Interface for SerialInterface {}

impl SerialInterface {
    pub fn new(
        usb_device: usb_device::device::UsbDevice<'static, UsbBus>,
        usb_serial: usbd_serial::SerialPort<'static, UsbBus>,
    ) -> Self {
        Self {
            usb_device,
            usb_serial,
        }
    }

    pub fn usb_is_suspended(&self) -> bool {
        self.usb_device.state() == usb_device::device::UsbDeviceState::Suspend
    }

    pub fn process(&mut self) {
        self.usb_device.poll(&mut [&mut self.usb_serial]);
    }
}
