use super::UsbBus;
use crate::hardware::flash::FlashSettings;
use crate::hardware::flash::Settings;
use core::fmt::Write;
use miniconf::{JsonCoreSlash, TreeKey};

struct Context {
    output: OutputBuffer,
    flash: FlashSettings,
}

const ROOT_MENU: menu::Menu<Context> = menu::Menu {
    label: "root",
    items: &[
        &menu::Item {
            command: "reboot",
            help: Some("Reboot the device to force new settings to take effect."),
            item_type: menu::ItemType::Callback {
                function: handle_device_reboot,
                parameters: &[]
            },
        },
        &menu::Item {
            command: "factory-reset",
            help: Some("Reset the device settings to default values."),
            item_type: menu::ItemType::Callback {
                function: handle_settings_reset,
                parameters: &[]
            },
        },
        &menu::Item {
            command: "list",
            help: Some("List all available settings and their current values."),
            item_type: menu::ItemType::Callback {
                function: handle_list,
                parameters: &[],
            },
        },
        &menu::Item {
            command: "read",
            help: Some("Read a setting_from the device."),
            item_type: menu::ItemType::Callback {
                function: handle_setting_read,
                parameters: &[menu::Parameter::Mandatory {
                    parameter_name: "item",
                    help: Some("The name of the setting to read."),
                }]
            },
        },
        &menu::Item {
            command: "write",
            help: Some("Update a a setting in the device."),
            item_type: menu::ItemType::Callback {
                function: handle_setting_write,
                parameters: &[
                    menu::Parameter::Mandatory {
                        parameter_name: "item",
                        help: Some("The name of the setting to write."),
                    },
                    menu::Parameter::Mandatory {
                        parameter_name: "value",
                        help: Some("Specifies the value to be written. Values must be JSON-encoded"),
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

fn handle_list(
    _menu: &menu::Menu<Context>,
    _item: &menu::Item<Context>,
    _args: &[&str],
    context: &mut Context,
) {
    writeln!(context, "Available properties:").unwrap();

    let mut defaults = context.flash.settings.clone();
    defaults.reset();

    let mut buf = [0; 256];
    let mut default_buf = [0; 256];
    for path in Settings::iter_paths::<heapless::String<32>>("/") {
        let path = path.unwrap();
        let current_value = {
            let len = context.flash.settings.get_json(&path, &mut buf).unwrap();
            core::str::from_utf8(&buf[..len]).unwrap()
        };
        let default_value = {
            let len = defaults.get_json(&path, &mut default_buf).unwrap();
            core::str::from_utf8(&default_buf[..len]).unwrap()
        };
        writeln!(
            context,
            "{path}: {current_value} [default: {default_value}]"
        )
        .unwrap();
    }
}

fn handle_device_reboot(
    _menu: &menu::Menu<Context>,
    _item: &menu::Item<Context>,
    _args: &[&str],
    _context: &mut Context,
) {
    cortex_m::peripheral::SCB::sys_reset();
}

fn handle_settings_reset(
    _menu: &menu::Menu<Context>,
    _item: &menu::Item<Context>,
    _args: &[&str],
    context: &mut Context,
) {
    context.flash.settings.reset();
    context.flash.save();
}

fn handle_setting_read(
    _menu: &menu::Menu<Context>,
    item: &menu::Item<Context>,
    args: &[&str],
    context: &mut Context,
) {
    let mut buf = [0u8; 256];
    let key = menu::argument_finder(item, args, "item").unwrap().unwrap();
    let len = match context.flash.settings.get_json(key, &mut buf) {
        Err(e) => {
            writeln!(context, "Failed to read {key}: {e}").unwrap();
            return;
        }
        Ok(len) => len,
    };

    let stringified = core::str::from_utf8(&buf[..len]).unwrap();
    writeln!(context, "{key}: {stringified}").unwrap();
}

fn handle_setting_write(
    _menu: &menu::Menu<Context>,
    item: &menu::Item<Context>,
    args: &[&str],
    context: &mut Context,
) {
    let key = menu::argument_finder(item, args, "item").unwrap().unwrap();
    let value = menu::argument_finder(item, args, "value").unwrap().unwrap();

    // Now, write the new value into memory.
    // TODO: Validate it first?
    match context.flash.settings.set_json(key, value.as_bytes()) {
        Ok(_) => {
            context.flash.save();
            writeln!(
                context,
                "Settings in memory may differ from currently operating settings. \
        Reset device to apply settings."
            )
    .unwrap();
        }
        Err(e) => {
            writeln!(context, "Failed to update {key}: {e:?}").unwrap();
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
            cortex_m::singleton!(: [u8; 256] = [0; 256]).unwrap();
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
