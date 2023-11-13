use super::UsbBus;
use crate::hardware::flash::FlashSettings;
use core::fmt::Write;
use miniconf::JsonCoreSlash;

struct Context {
    output: OutputBuffer,
    flash: FlashSettings,
}

const ROOT_MENU: menu::Menu<Context> = menu::Menu {
    label: "root",
    items: &[
        &menu::Item {
            command: "reset",
            help: Some("Reset Stabilizer to force new settings to take effect."),
            item_type: menu::ItemType::Callback {
                function: handle_reset,
                parameters: &[]
            },
        },
        &menu::Item {
            command: "read",
            help: Some("Read a property from the device:

Available Properties:
* /id: The MQTT ID of the device
* /broker: The MQTT broker address"),
            item_type: menu::ItemType::Callback {
                function: handle_property_read,
                parameters: &[menu::Parameter::Optional {
                    parameter_name: "property",
                    help: Some("The name of the property to read. If not specified, all properties are read"),
                }]
            },
        },
        &menu::Item {
            command: "write",
            help: Some("Write a property to the device:

Available Properties:
* /id: The MQTT ID of the device
* /broker: The MQTT broker address"),
            item_type: menu::ItemType::Callback {
                function: handle_property_write,
                parameters: &[
                    menu::Parameter::Mandatory {
                        parameter_name: "property",
                        help: Some("The name of the property to write: [id, broker]"),
                    },
                    menu::Parameter::Mandatory {
                        parameter_name: "value",
                        help: Some("Specifies the value to be written to the property."),
                    },
                ]
            },
        },
    ],
    entry: None,
    exit: None,
};

static OUTPUT_BUFFER: bbqueue::BBBuffer<512> = bbqueue::BBBuffer::new();

pub struct OutputBuffer {
    producer: bbqueue::Producer<'static, 512>,
}

impl Write for OutputBuffer {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let data = s.as_bytes();

        // Write as much data as possible to the output buffer.
        let Ok(mut grant) = self.producer.grant_max_remaining(data.len())
        else {
            // Output buffer is full, silently drop the data.
            return Ok(());
        };

        let len = grant.buf().len();
        grant.buf().copy_from_slice(&data[..len]);
        grant.commit(len);
        Ok(())
    }
}

impl core::fmt::Write for Context {
    /// Write data to the serial terminal.
    ///
    /// # Note
    /// The terminal uses an internal buffer. Overflows of the output buffer are silently ignored.
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.output.write_str(s)
    }
}

fn handle_reset(
    _menu: &menu::Menu<Context>,
    _item: &menu::Item<Context>,
    _args: &[&str],
    _context: &mut Context,
) {
    cortex_m::peripheral::SCB::sys_reset();
}

fn handle_property_read(
    _menu: &menu::Menu<Context>,
    item: &menu::Item<Context>,
    args: &[&str],
    context: &mut Context,
) {
    let props: heapless::Vec<&'_ str, 2> = if let Some(prop) =
        menu::argument_finder(item, args, "property").unwrap()
    {
        heapless::Vec::from_slice(&[prop]).unwrap()
    } else {
        heapless::Vec::from_slice(&["/id", "/broker"]).unwrap()
    };

    let mut buf = [0u8; 256];
    for path in props {
        let len = context.flash.settings.get_json(path, &mut buf).unwrap();
        let stringified = core::str::from_utf8(&buf[..len]).unwrap();
        writeln!(&mut context.output, "{path}: {stringified}").unwrap();
    }
}

fn handle_property_write(
    _menu: &menu::Menu<Context>,
    item: &menu::Item<Context>,
    args: &[&str],
    context: &mut Context,
) {
    let property = menu::argument_finder(item, args, "property")
        .unwrap()
        .unwrap();
    let value = menu::argument_finder(item, args, "value").unwrap().unwrap();

    // Now, write the new value into memory.
    // TODO: Validate it first?
    match context.flash.settings.set_json(property, value.as_bytes()) {
        Ok(_) => {
            context.flash.save();
            writeln!(
                &mut context.output,
                "Settings in memory may differ from currently operating settings. \
        Reset device to apply settings."
            )
    .unwrap();
        }
        Err(e) => {
            writeln!(&mut context.output, "Failed to update {property}: {e:?}").unwrap();
        }
    }
}

pub struct SerialTerminal {
    usb_device: usb_device::device::UsbDevice<'static, UsbBus>,
    usb_serial: usbd_serial::SerialPort<'static, UsbBus>,
    menu: menu::Runner<'static, Context>,
    output: bbqueue::Consumer<'static, 512>,
}

impl SerialTerminal {
    pub fn new(
        usb_device: usb_device::device::UsbDevice<'static, UsbBus>,
        usb_serial: usbd_serial::SerialPort<'static, UsbBus>,
        flash: FlashSettings,
    ) -> Self {
        let (producer, consumer) = OUTPUT_BUFFER.try_split().unwrap();

        let input_buffer =
            cortex_m::singleton!(: [u8; 255] = [0; 255]).unwrap();
        let context = Context {
            output: OutputBuffer { producer },
            flash,
        };
        Self {
            menu: menu::Runner::new(&ROOT_MENU, input_buffer, context),
            usb_device,
            usb_serial,
            output: consumer,
        }
    }

    pub fn flash(&mut self) -> &mut FlashSettings {
        &mut self.menu.context.flash
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
                    self.menu.input_byte(value);
                }
            }

            Err(usbd_serial::UsbError::WouldBlock) => {}
            Err(_) => {
                self.menu.prompt(true);
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
