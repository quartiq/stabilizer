use super::SerialTerminal;
use super::UsbBus;

pub struct UsbDevice {
    usb_device: usb_device::device::UsbDevice<'static, UsbBus>,
}

impl UsbDevice {
    pub fn new(
        usb_device: usb_device::device::UsbDevice<'static, UsbBus>,
    ) -> Self {
        Self { usb_device }
    }

    pub fn usb_is_suspended(&self) -> bool {
        self.usb_device.state() == usb_device::device::UsbDeviceState::Suspend
    }

    pub fn process(&mut self, terminal: &mut SerialTerminal) {
        self.usb_device.poll(&mut [terminal.interface_mut()]);
    }
}
