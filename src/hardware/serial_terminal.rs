use super::UsbBus;
use core::fmt::Write;

static OUTPUT_BUFFER: bbqueue::BBBuffer<512> = bbqueue::BBBuffer::new();

pub struct OutputBuffer {
    producer: bbqueue::Producer<'static, 512>,
}

impl Write for OutputBuffer {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let data = s.as_bytes();

        // Write as much data as possible to the output buffer.
        let Ok(mut grant) = self.producer.grant_max_remaining(data.len()) else {
            // Output buffer is full, silently drop the data.
            return Ok(());
        };

        let len = grant.buf().len();
        grant.buf().copy_from_slice(&data[..len]);
        grant.commit(len);
        Ok(())
    }
}

pub struct SerialTerminal {
    usb_device: usb_device::device::UsbDevice<'static, UsbBus>,
    usb_serial: usbd_serial::SerialPort<'static, UsbBus>,
    output: bbqueue::Consumer<'static, 512>,
    buffer: OutputBuffer,
}

impl SerialTerminal {
    pub fn new(
        usb_device: usb_device::device::UsbDevice<'static, UsbBus>,
        usb_serial: usbd_serial::SerialPort<'static, UsbBus>,
    ) -> Self {
        let (producer, consumer) = OUTPUT_BUFFER.try_split().unwrap();

        Self {
            buffer: OutputBuffer { producer },
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

    pub fn usb_is_suspended(&self) -> bool {
        self.usb_device.state() == usb_device::device::UsbDeviceState::Suspend
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
                    writeln!(self.buffer, "echo: {}", value as char).unwrap();
                }
            }

            Err(usbd_serial::UsbError::WouldBlock) => {}
            Err(_) => {
                // Clear the output buffer if USB is not connected.
                while let Ok(grant) = self.output.read() {
                    let len = grant.buf().len();
                    grant.release(len);
                }
            }
        }
    }
}
