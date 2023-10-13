use super::UsbBus;

static OUTPUT_BUFFER: bbqueue::BBBuffer<1024> = bbqueue::BBBuffer::new();

pub struct SerialTerminal {
    usb_device: usb_device::device::UsbDevice<'static, UsbBus>,
    usb_serial: usbd_serial::SerialPort<'static, UsbBus>,
    output: bbqueue::Consumer<'static, 1024>,
}

impl SerialTerminal {
    pub fn new(
        usb_device: usb_device::device::UsbDevice<'static, UsbBus>,
        usb_serial: usbd_serial::SerialPort<'static, UsbBus>,
    ) -> Self {
        let (_producer, consumer) = OUTPUT_BUFFER.try_split().unwrap();

        Self {
            usb_device,
            usb_serial,
            output: consumer,
        }
    }

    fn flush(&mut self) {
        let read = match self.output.read() {
            Ok(grant) => grant,
            Err(bbqueue::Error::InsufficientSize) => return,
            err => err.unwrap(),
        };

        match self.usb_serial.write(read.buf()) {
            Ok(count) => read.release(count),
            Err(usbd_serial::UsbError::WouldBlock) => read.release(0),
            Err(_) => {
                let len = read.buf().len();
                read.release(len);
            }
        }
    }

    pub fn process(&mut self) {
        self.flush();

        if !self.usb_device.poll(&mut [&mut self.usb_serial]) {
            return;
        }

        let mut buffer = [0u8; 64];
        match self.usb_serial.read(&mut buffer) {
            Ok(count) => {
                for &value in &buffer[..count] {
                    // Currently, just echo it back.
                    self.usb_serial.write(&[value]).ok();
                }
            }

            Err(usbd_serial::UsbError::WouldBlock) => {}
            Err(_) => {
                // Flush the output buffer if USB is not connected.
                self.output
                    .read()
                    .map(|grant| {
                        let len = grant.buf().len();
                        grant.release(len);
                    })
                    .ok();
            }
        }
    }
}
