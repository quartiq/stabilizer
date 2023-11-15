#![no_std]

use core::fmt::Write;
use miniconf::JsonCoreSlash;
use embedded_storage::nor_flash::NorFlash;
use serde::Serialize;

static OUTPUT_BUFFER: bbqueue::BBBuffer<512> = bbqueue::BBBuffer::new();

struct Context<Settings: SerialSettings, Flash: NorFlash + 'static> {
    output: OutputBuffer,
    flash: Flash,
    settings: Settings,
}

impl<Settings: SerialSettings, Flash: NorFlash + 'static> Context<Settings, Flash> {
    fn save(&mut self) {
        self.flash.erase(0, self.flash.capacity() as u32).unwrap();
        // TODO: Use a user provided buffer.
        let mut buf = [0; 512];
        let serialized = postcard::to_slice(&self.settings, &mut buf).unwrap();
        self.flash.write(0, serialized).unwrap();
    }
}

pub trait SerialSettings: for<'a> JsonCoreSlash<'a> + Serialize + Clone + 'static {
    fn reset(&mut self);
}

fn make_menu<Settings: SerialSettings, Flash: NorFlash + 'static>() -> menu::Menu<'static, Context<Settings, Flash>> {
    menu::Menu {
        label: "root",
        items: &[
            &menu::Item {
                command: "reboot",
                help: Some("Reboot the device to force new settings to take effect."),
                item_type: menu::ItemType::Callback {
                    function: handle_reboot,
                    parameters: &[]
                },
            },
            &menu::Item {
                command: "factory-reset",
                help: Some("Reset the device settings to default values."),
                item_type: menu::ItemType::Callback {
                    function: handle_reset,
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
                command: "get",
                help: Some("Read a setting_from the device."),
                item_type: menu::ItemType::Callback {
                    function: handle_get,
                    parameters: &[menu::Parameter::Mandatory {
                        parameter_name: "item",
                        help: Some("The name of the setting to read."),
                    }]
                },
            },
            &menu::Item {
                command: "set",
                help: Some("Update a a setting in the device."),
                item_type: menu::ItemType::Callback {
                    function: handle_set,
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
    }
}

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

impl<Settings: SerialSettings, Flash: NorFlash + 'static> core::fmt::Write for Context<Settings, Flash> {
    /// Write data to the serial terminal.
    ///
    /// # Note
    /// The terminal uses an internal buffer. Overflows of the output buffer are silently ignored.
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.output.write_str(s)
    }
}

fn handle_list<Settings: SerialSettings, Flash: NorFlash + 'static>(
    _menu: &menu::Menu<Context<Settings, Flash>>,
    _item: &menu::Item<Context<Settings, Flash>>,
    _args: &[&str],
    context: &mut Context<Settings, Flash>,
) {
    writeln!(context, "Available items:").unwrap();

    let mut defaults = context.settings.clone();
    defaults.reset();

    let mut buf = [0; 256];
    let mut default_buf = [0; 256];
    for path in Settings::iter_paths::<heapless::String<32>>("/") {
        let path = path.unwrap();
        let current_value = {
            let len = context.settings.get_json(&path, &mut buf).unwrap();
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

fn handle_reboot<Settings: SerialSettings, Flash: NorFlash + 'static>(
    _menu: &menu::Menu<Context<Settings, Flash>>,
    _item: &menu::Item<Context<Settings, Flash>>,
    _args: &[&str],
    _context: &mut Context<Settings, Flash>,
) {
    cortex_m::peripheral::SCB::sys_reset();
}

fn handle_reset<Settings: SerialSettings, Flash: NorFlash + 'static>(
    _menu: &menu::Menu<Context<Settings, Flash>>,
    _item: &menu::Item<Context<Settings, Flash>>,
    _args: &[&str],
    context: &mut Context<Settings, Flash>,
) {
    context.settings.reset();
    context.save();
}

fn handle_get<Settings: SerialSettings, Flash: NorFlash + 'static>(
    _menu: &menu::Menu<Context<Settings, Flash>>,
    item: &menu::Item<Context<Settings, Flash>>,
    args: &[&str],
    context: &mut Context<Settings, Flash>,
) {
    let mut buf = [0u8; 256];
    let key = menu::argument_finder(item, args, "item").unwrap().unwrap();
    let len = match context.settings.get_json(key, &mut buf) {
        Err(e) => {
            writeln!(context, "Failed to read {key}: {e}").unwrap();
            return;
        }
        Ok(len) => len,
    };

    let stringified = core::str::from_utf8(&buf[..len]).unwrap();
    writeln!(context, "{key}: {stringified}").unwrap();
}

fn handle_set<Settings: SerialSettings, Flash: NorFlash + 'static>(
    _menu: &menu::Menu<Context<Settings, Flash>>,
    item: &menu::Item<Context<Settings, Flash>>,
    args: &[&str],
    context: &mut Context<Settings, Flash>,
) {
    let key = menu::argument_finder(item, args, "item").unwrap().unwrap();
    let value = menu::argument_finder(item, args, "value").unwrap().unwrap();

    // Now, write the new value into memory.
    // TODO: Validate it first?
    match context.settings.set_json(key, value.as_bytes()) {
        Ok(_) => {
            context.save();
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

pub struct SerialTerminal<'a, UsbBus: usb_device::bus::UsbBus, Settings: SerialSettings, Flash: NorFlash + 'static> {
    usb_device: usb_device::device::UsbDevice<'a, UsbBus>,
    usb_serial: usbd_serial::SerialPort<'a, UsbBus>,
    menu: menu::Runner<'a, Context<Settings, Flash>>,
    output: bbqueue::Consumer<'static, 512>,
}

impl<'a, UsbBus: usb_device::bus::UsbBus, Settings: SerialSettings, Flash: NorFlash + 'static> SerialTerminal<'a, UsbBus, Settings, Flash> {
    pub fn new(
        usb_device: usb_device::device::UsbDevice<'static, UsbBus>,
        usb_serial: usbd_serial::SerialPort<'static, UsbBus>,
        input_buffer: &'a mut [u8],
        settings: Settings,
        flash: Flash,
    ) -> Self {
        let (producer, consumer) = OUTPUT_BUFFER.try_split().unwrap();

        // TODO: Attempt to load from flash first.

        let context = Context {
            settings,
            output: OutputBuffer { producer },
            flash,
        };
        Self {
            menu: menu::Runner::new(make_menu(), input_buffer, context),
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

    pub fn settings(&self) -> &Settings {
        &self.menu.context.settings
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
