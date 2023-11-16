//! Persistent Settings Management Serial Interface
//!
//! # Description
//! This crate provides a simple means to load, configure, and save device settings over a serial
//! (i.e. text-based) interface. It is ideal to be used with serial ports and terminal emulators,
//! and exposes a simple way to allow users to configure device operation.
//!
//! # Example
//! Let's assume that your settings structure looks as follows:
//! ```rust
//! #[derive(miniconf::Tree, ...)]
//! struct Settings {
//!     broker: String,
//!     id: String,
//! }
//! ```
//!
//! A user would be displayed the following terminal interface:
//! ```
//!> help
//! AVAILABLE ITEMS:
//!   platform <cmd>
//!   factory-reset
//!   list
//!   get <item>
//!   set <item> <value>
//!   help [ <command> ]
//!
//! > plaform dfu
//! Reset to DFU is not supported
//!
//! > plaform service
//! Service data not available
//!
//! > list
//! Available settings:
//! /broker: "test" [default: "mqtt"]
//! /id: "04-91-62-d2-a8-6f" [default: "04-91-62-d2-a8-6f"]
//! ```
//!
//! # Design
//! Settings are specified in a [`Miniconf::Tree`] settings tree and are transferred over the
//! serial interface using JSON encoding. This means that things like strings must be encased in
//! qutoes.
//!
//! # Limitations
//! Currently, there is a hardcoded limit of 32-bytes on the settings path. This is arbitrary and
//! can be changed if needed.
#![no_std]

use core::fmt::Write;
use embedded_io::{Read, ReadReady, Write as EioWrite, WriteReady};
use miniconf::{JsonCoreSlash, TreeKey};

/// Specifies the API required for objects that are used as settings with the serial terminal
/// interface.
pub trait Settings: for<'a> JsonCoreSlash<'a> + Clone {
    /// Reset the settings to their default values.
    fn reset(&mut self) {}
}

pub trait Platform: Sized {
    /// This type specifies the interface to the user, for example, a USB CDC-ACM serial port.
    type Interface: embedded_io::Read
        + embedded_io::ReadReady
        + embedded_io::Write
        + embedded_io::WriteReady;

    /// Specifies the settings that are used on the device.
    type Settings: Settings;

    type Error: core::fmt::Debug;

    /// Execute a platform specific command.
    fn cmd(&mut self, cmd: &str);
    /// Return a mutable reference to the `Interface`.
    fn interface_mut(&mut self) -> &mut Self::Interface;
    /// Return a reference to the `Settings`
    fn settings(&self) -> &Self::Settings;
    /// Return a mutable reference to the `Settings`.
    fn settings_mut(&mut self) -> &mut Self::Settings;
    /// Load the settings from storage
    fn load(&mut self, buffer: &mut [u8]) -> Result<(), Self::Error>;
    /// Save the settings to storage
    fn save(&mut self, buffer: &mut [u8]) -> Result<(), Self::Error>;
}

struct Context<'a, P: Platform> {
    platform: P,
    buffer: &'a mut [u8],
}

impl<'a, P: Platform> Context<'a, P> {
    fn handle_platform(
        _menu: &menu::Menu<Self>,
        item: &menu::Item<Self>,
        args: &[&str],
        context: &mut Self,
    ) {
        let key = menu::argument_finder(item, args, "cmd").unwrap().unwrap();
        context.platform.cmd(key);
    }

    fn handle_list(
        _menu: &menu::Menu<Self>,
        _item: &menu::Item<Self>,
        _args: &[&str],
        context: &mut Self,
    ) {
        writeln!(context, "Available settings:").unwrap();

        let mut defaults = context.platform.settings().clone();
        defaults.reset();

        for path in P::Settings::iter_paths::<heapless::String<32>>("/") {
            let path = path.unwrap();
            let current_value = {
                let len = context
                    .platform
                    .settings()
                    .get_json(&path, context.buffer)
                    .unwrap();
                core::str::from_utf8(&context.buffer[..len]).unwrap()
            };
            write!(context.platform.interface_mut(), "{path}: {current_value}")
                .unwrap();

            let default_value = {
                let len = defaults.get_json(&path, context.buffer).unwrap();
                core::str::from_utf8(&context.buffer[..len]).unwrap()
            };
            writeln!(
                context.platform.interface_mut(),
                " [default: {default_value}]"
            )
            .unwrap()
        }
    }

    fn handle_factory_reset(
        _menu: &menu::Menu<Self>,
        _item: &menu::Item<Self>,
        _args: &[&str],
        context: &mut Self,
    ) {
        context.platform.settings_mut().reset();
        match context.platform.save(context.buffer) {
            Ok(_) => {
                writeln!(context, "Settings reset to default").unwrap();
            }
            Err(e) => {
                writeln!(context, "Failed to reset settings: {e:?}").unwrap();
            }
        }
    }

    fn handle_get(
        _menu: &menu::Menu<Self>,
        item: &menu::Item<Self>,
        args: &[&str],
        context: &mut Self,
    ) {
        let key = menu::argument_finder(item, args, "item").unwrap().unwrap();
        let len =
            match context.platform.settings().get_json(key, context.buffer) {
                Err(e) => {
                    writeln!(context, "Failed to read {key}: {e}").unwrap();
                    return;
                }
                Ok(len) => len,
            };

        let stringified = core::str::from_utf8(&context.buffer[..len]).unwrap();
        writeln!(context.platform.interface_mut(), "{key}: {stringified}")
            .unwrap();
    }

    fn handle_set(
        _menu: &menu::Menu<Self>,
        item: &menu::Item<Self>,
        args: &[&str],
        context: &mut Self,
    ) {
        let key = menu::argument_finder(item, args, "item").unwrap().unwrap();
        let value =
            menu::argument_finder(item, args, "value").unwrap().unwrap();

        // Now, write the new value into memory.
        // TODO: Validate it first?
        match context
            .platform
            .settings_mut()
            .set_json(key, value.as_bytes())
        {
            Ok(_) => match context.platform.save(context.buffer) {
                Ok(_) => {
                    writeln!(
                            context,
                            "Settings in memory may differ from currently operating settings. \
    Reset device to apply settings."
                        )
                        .unwrap();
                }
                Err(e) => {
                    writeln!(context, "Failed to save settings: {e:?}")
                        .unwrap();
                }
            },
            Err(e) => {
                writeln!(context, "Failed to update {key}: {e:?}").unwrap();
            }
        }
    }

    fn menu() -> menu::Menu<'a, Self> {
        menu::Menu {
        label: "settings",
        items: &[
            &menu::Item {
                command: "platform",
                help: Some("Platform specific command"),
                item_type: menu::ItemType::Callback {
                    function: Self::handle_platform,
                    parameters: &[menu::Parameter::Mandatory {
                        parameter_name: "cmd",
                        help: Some("The name of the command (e.g. `reboot`, `service`, `dfu`)."),
                    }]
                },
            },
            &menu::Item {
                command: "factory-reset",
                help: Some("Reset the device settings to default values."),
                item_type: menu::ItemType::Callback {
                    function: Self::handle_factory_reset,
                    parameters: &[]
                },
            },
            &menu::Item {
                command: "list",
                help: Some("List all available settings and their current values."),
                item_type: menu::ItemType::Callback {
                    function: Self::handle_list,
                    parameters: &[],
                },
            },
            &menu::Item {
                command: "get",
                help: Some("Read a setting_from the device."),
                item_type: menu::ItemType::Callback {
                    function: Self::handle_get,
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
                    function: Self::handle_set,
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
}

impl<'a, P: Platform> core::fmt::Write for Context<'a, P> {
    /// Write data to the serial terminal.
    ///
    /// # Note
    /// The terminal uses an internal buffer. Overflows of the output buffer are silently ignored.
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        if let Ok(true) = self.platform.interface_mut().write_ready() {
            self.platform.interface_mut().write_all(s.as_bytes()).ok();
        }
        Ok(())
    }
}

/// The serial settings management object.
pub struct Runner<'a, P: Platform>(menu::Runner<'a, Context<'a, P>>);

impl<'a, P: Platform> Runner<'a, P> {
    /// Constructor
    ///
    /// # Args
    /// * `platform` - The platform associated with the serial settings, providing the necessary
    /// context and API to manage device settings.
    /// * `line_buf` - A buffer used for maintaining the serial menu input line. It should be at
    /// least as long as the longest user input.
    /// * `serialize_buf` - A buffer used for serializing and deserializing settings. This buffer
    /// needs to be at least as big as the entire serialized settings structure.
    pub fn new(
        mut platform: P,
        line_buf: &'a mut [u8],
        serialize_buf: &'a mut [u8],
    ) -> Result<Self, P::Error> {
        platform.load(serialize_buf)?;
        Ok(Self(menu::Runner::new(
            Context::menu(),
            line_buf,
            Context {
                platform,
                buffer: serialize_buf,
            },
        )))
    }

    /// Get the current device settings.
    pub fn settings(&self) -> &P::Settings {
        self.0.context.platform.settings()
    }

    /// Get the device communication interface
    pub fn interface_mut(&mut self) -> &mut P::Interface {
        self.0.context.platform.interface_mut()
    }

    /// Must be called periodically to process user input.
    pub fn process(
        &mut self,
    ) -> Result<(), <P::Interface as embedded_io::ErrorType>::Error> {
        while self.interface_mut().read_ready()? {
            let mut buffer = [0u8; 64];
            let count = self.interface_mut().read(&mut buffer)?;
            for &value in &buffer[..count] {
                self.0.input_byte(value);
            }
        }

        Ok(())
    }
}
