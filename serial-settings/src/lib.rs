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
//!   list
//!   get <item>
//!   set <item> <value>
//!   clear
//!   platform <cmd>
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

    /// `load()`/`save()` Error type
    type Error: core::fmt::Debug;

    /// Load the settings from storage
    fn load(&mut self, buffer: &mut [u8]) -> Result<(), Self::Error>;

    /// Save the settings to storage
    fn save(&mut self, buffer: &mut [u8]) -> Result<(), Self::Error>;

    /// Execute a platform specific command.
    fn cmd(&mut self, cmd: &str);

    /// Return a mutable reference to the `Interface`.
    fn interface_mut(&mut self) -> &mut Self::Interface;

    /// Return a reference to the `Settings`
    fn settings(&self) -> &Self::Settings;

    /// Return a mutable reference to the `Settings`.
    fn settings_mut(&mut self) -> &mut Self::Settings;
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
        context.platform.cmd(key)
    }

    fn handle_list(
        _menu: &menu::Menu<Self>,
        _item: &menu::Item<Self>,
        _args: &[&str],
        context: &mut Self,
    ) {
        let mut defaults = context.platform.settings().clone();
        defaults.reset();

        for path in P::Settings::iter_paths::<heapless::String<32>>("/") {
            match path {
                Err(e) => writeln!(
                    context.platform.interface_mut(),
                    "Failed to get path: {e}"
                ),
                Ok(path) => {
                    match context
                        .platform
                        .settings()
                        .get_json(&path, context.buffer)
                    {
                        Err(e) => {
                            writeln!(
                                context.platform.interface_mut(),
                                "Failed to read {path}: {e}"
                            )
                            .unwrap();
                            continue;
                        }
                        Ok(len) => write!(
                            context.platform.interface_mut(),
                            "{path}: {}",
                            core::str::from_utf8(&context.buffer[..len])
                                .unwrap()
                        )
                        .unwrap(),
                    }

                    match defaults.get_json(&path, context.buffer) {
                        Err(e) => writeln!(
                            context.platform.interface_mut(),
                            "[default serialization error: {e}]"
                        ),
                        Ok(len) => writeln!(
                            context.platform.interface_mut(),
                            " [default: {}]",
                            core::str::from_utf8(&context.buffer[..len])
                                .unwrap()
                        ),
                    }
                }
            }
            .unwrap()
        }
    }

    fn handle_clear(
        _menu: &menu::Menu<Self>,
        _item: &menu::Item<Self>,
        _args: &[&str],
        context: &mut Self,
    ) {
        context.platform.settings_mut().reset();
        match context.platform.save(context.buffer) {
            Ok(_) => {
                writeln!(context, "Settings cleared to defaults and saved.")
            }
            Err(e) => {
                writeln!(context, "Failed to clear settings: {e:?}")
            }
        }
        .unwrap();
    }

    fn handle_get(
        _menu: &menu::Menu<Self>,
        item: &menu::Item<Self>,
        args: &[&str],
        context: &mut Self,
    ) {
        let key = menu::argument_finder(item, args, "item").unwrap().unwrap();
        match context.platform.settings().get_json(key, context.buffer) {
            Err(e) => {
                writeln!(
                    context.platform.interface_mut(),
                    "Failed to read {key}: {e}"
                )
            }
            Ok(len) => {
                writeln!(
                    context.platform.interface_mut(),
                    "{key}: {}",
                    core::str::from_utf8(&context.buffer[..len]).unwrap()
                )
            }
        }
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
                            "Settings saved. Reboot device (`platform reboot`) to apply."
                        )
                }
                Err(e) => {
                    writeln!(context, "Failed to save settings: {e:?}")
                }
            },
            Err(e) => {
                writeln!(context, "Failed to update {key}: {e:?}")
            }
        }.unwrap();
    }

    fn menu() -> menu::Menu<'a, Self> {
        menu::Menu {
        label: "settings",
        items: &[
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
            &menu::Item {
                command: "clear",
                help: Some("Clear the device settings to default values."),
                item_type: menu::ItemType::Callback {
                    function: Self::handle_clear,
                    parameters: &[]
                },
            },
            &menu::Item {
                command: "platform",
                help: Some("Platform specific commands"),
                item_type: menu::ItemType::Callback {
                    function: Self::handle_platform,
                    parameters: &[menu::Parameter::Mandatory {
                        parameter_name: "cmd",
                        help: Some("The name of the command (e.g. `reboot`, `service`, `dfu`)."),
                    }]
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
