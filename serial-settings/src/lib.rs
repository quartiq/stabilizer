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
//!   save
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
//! Settings are specified in a [`miniconf::Tree`] settings tree and are transferred over the
//! serial interface using JSON encoding. This means that things like strings must be encased in
//! qutoes.
//!
//! # Limitations
//! Currently, there is a hardcoded limit of 64-bytes on the settings path. This is arbitrary and
//! can be changed if needed.
#![no_std]

use core::fmt::Write;
use core::hash::Hasher;
use embedded_io::{Read, ReadReady};
use heapless::String;
use miniconf::{JsonCoreSlash, TreeKey};

mod interface;

pub use interface::BestEffortInterface;

/// Specifies the API required for objects that are used as settings with the serial terminal
/// interface.
pub trait Settings<const Y: usize>:
    for<'a> JsonCoreSlash<'a, Y> + Clone
{
    /// Reset the settings to their default values.
    fn reset(&mut self) {}
}

pub trait Platform<const Y: usize>: Sized {
    /// This type specifies the interface to the user, for example, a USB CDC-ACM serial port.
    type Interface: embedded_io::Read
        + embedded_io::ReadReady
        + core::fmt::Write;

    /// Specifies the settings that are used on the device.
    type Settings: Settings<Y>;

    /// `save()` Error type
    type Error: core::fmt::Debug;

    /// Save the settings to storage
    fn save(
        &mut self,
        buffer: &mut [u8],
        key: Option<&str>,
        settings: &Self::Settings,
    ) -> Result<(), Self::Error>;

    /// Execute a platform specific command.
    fn cmd(&mut self, cmd: &str);

    /// Return a mutable reference to the `Interface`.
    fn interface_mut(&mut self) -> &mut Self::Interface;
}

struct Interface<'a, P: Platform<Y>, const Y: usize> {
    platform: P,
    buffer: &'a mut [u8],
    updated: bool,
}

impl<'a, P: Platform<Y>, const Y: usize> Interface<'a, P, Y> {
    fn handle_platform(
        _menu: &menu::Menu<Self, P::Settings>,
        item: &menu::Item<Self, P::Settings>,
        args: &[&str],
        interface: &mut Self,
        _settings: &mut P::Settings,
    ) {
        let key = menu::argument_finder(item, args, "cmd").unwrap().unwrap();
        interface.platform.cmd(key)
    }

    fn handle_list(
        _menu: &menu::Menu<Self, P::Settings>,
        _item: &menu::Item<Self, P::Settings>,
        _args: &[&str],
        interface: &mut Self,
        settings: &mut P::Settings,
    ) {
        let mut defaults = settings.clone();
        defaults.reset();

        for path in P::Settings::iter_paths::<String<64>>("/") {
            match path {
                Err(e) => writeln!(interface, "Failed to get path: {e}"),
                Ok(path) => {
                    let value = match settings.get_json(&path, interface.buffer)
                    {
                        Err(e) => {
                            writeln!(interface, "Failed to read {path}: {e}")
                                .unwrap();
                            continue;
                        }
                        Ok(len) => {
                            core::str::from_utf8(&interface.buffer[..len])
                                .unwrap()
                        }
                    };

                    write!(
                        &mut interface.platform.interface_mut(),
                        "{path}: {value}"
                    )
                    .unwrap();

                    let value_hash = {
                        let mut hasher = yafnv::Fnv1aHasher::default();
                        hasher.write(value.as_bytes());
                        hasher.finish()
                    };

                    let default_value =
                        match defaults.get_json(&path, interface.buffer) {
                            Err(e) => {
                                writeln!(
                                    interface,
                                    "[default serialization error: {e}]"
                                )
                                .unwrap();
                                continue;
                            }
                            Ok(len) => {
                                core::str::from_utf8(&interface.buffer[..len])
                                    .unwrap()
                            }
                        };

                    let default_hash = {
                        let mut hasher = yafnv::Fnv1aHasher::default();
                        hasher.write(default_value.as_bytes());
                        hasher.finish()
                    };
                    if default_hash != value_hash {
                        writeln!(
                            &mut interface.platform.interface_mut(),
                            " [default: {default_value}]"
                        )
                    } else {
                        writeln!(
                            &mut interface.platform.interface_mut(),
                            " [default]"
                        )
                    }
                }
            }
            .unwrap()
        }
    }

    fn handle_clear(
        _menu: &menu::Menu<Self, P::Settings>,
        item: &menu::Item<Self, P::Settings>,
        args: &[&str],
        interface: &mut Self,
        settings: &mut P::Settings,
    ) {
        let maybe_key = menu::argument_finder(item, args, "item").unwrap();
        if let Some(key) = maybe_key {
            let mut defaults = settings.clone();
            defaults.reset();

            let len = match defaults.get_json(key, interface.buffer) {
                Err(e) => {
                    writeln!(interface, "Failed to clear `{key}`: {e:?}")
                        .unwrap();
                    return;
                }

                Ok(len) => len,
            };

            if let Err(e) = settings.set_json(key, &interface.buffer[..len]) {
                writeln!(interface, "Failed to update {key}: {e:?}").unwrap();
                return;
            }

            interface.updated = true;
            writeln!(interface, "{key} cleared to default").unwrap();
        } else {
            settings.reset();
            interface.updated = true;
            writeln!(interface, "All settings cleared").unwrap();
        }

        match interface.platform.save(interface.buffer, maybe_key, settings) {
            Ok(_) => {
                writeln!(interface, "Settings saved. You may need to reboot for the settings to be applied")
            }
            Err(e) => {
                writeln!(interface, "Failed to clear settings: {e:?}")
            }
        }
        .unwrap();
    }

    fn handle_get(
        _menu: &menu::Menu<Self, P::Settings>,
        item: &menu::Item<Self, P::Settings>,
        args: &[&str],
        interface: &mut Self,
        settings: &mut P::Settings,
    ) {
        let key = menu::argument_finder(item, args, "item").unwrap().unwrap();
        match settings.get_json(key, interface.buffer) {
            Err(e) => {
                writeln!(interface, "Failed to read {key}: {e}")
            }
            Ok(len) => {
                writeln!(
                    &mut interface.platform.interface_mut(),
                    "{key}: {}",
                    core::str::from_utf8(&interface.buffer[..len]).unwrap()
                )
            }
        }
        .unwrap();
    }

    fn handle_save(
        _menu: &menu::Menu<Self, P::Settings>,
        _item: &menu::Item<Self, P::Settings>,
        _args: &[&str],
        interface: &mut Self,
        settings: &mut P::Settings,
    ) {
        match interface.platform.save(interface.buffer, None, settings) {
            Ok(_) => writeln!(
                interface,
                "Settings saved. You may need to reboot for the settings to be applied"
            )
            .unwrap(),
            Err(e) => {
                writeln!(interface, "Failed to save settings: {e:?}").unwrap()
            }
        }
    }

    fn handle_set(
        _menu: &menu::Menu<Self, P::Settings>,
        item: &menu::Item<Self, P::Settings>,
        args: &[&str],
        interface: &mut Self,
        settings: &mut P::Settings,
    ) {
        let key = menu::argument_finder(item, args, "item").unwrap().unwrap();
        let value =
            menu::argument_finder(item, args, "value").unwrap().unwrap();

        // Now, write the new value into memory.
        match settings.set_json(key, value.as_bytes())
        {
            Ok(_) => {
                interface.updated = true;
                writeln!(
                        interface,
                        "Settings updated. You may need to reboot for the setting to be applied"
                    )
            },
            Err(e) => {
                writeln!(interface, "Failed to update {key}: {e:?}")
            }
        }.unwrap();
    }

    fn menu() -> menu::Menu<'a, Self, P::Settings> {
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
                command: "save",
                help: Some("Save all current settings to the device."),
                item_type: menu::ItemType::Callback {
                    function: Self::handle_save,
                    parameters: &[],
                },
            },
            &menu::Item {
                command: "clear",
                help: Some("Clear the device settings to default values."),
                item_type: menu::ItemType::Callback {
                    function: Self::handle_clear,
                    parameters: &[
                        menu::Parameter::Optional {
                            parameter_name: "item",
                            help: Some("The name of the setting to clear."),
                        },
                    ]
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

impl<'a, P: Platform<Y>, const Y: usize> core::fmt::Write
    for Interface<'a, P, Y>
{
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.platform.interface_mut().write_str(s)
    }
}

/// The serial settings management object.
pub struct Runner<'a, P: Platform<Y>, const Y: usize>(
    menu::Runner<'a, Interface<'a, P, Y>, P::Settings>,
);

impl<'a, P: Platform<Y>, const Y: usize> Runner<'a, P, Y> {
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
        platform: P,
        line_buf: &'a mut [u8],
        serialize_buf: &'a mut [u8],
        settings: &mut P::Settings,
    ) -> Result<Self, P::Error> {
        Ok(Self(menu::Runner::new(
            Interface::menu(),
            line_buf,
            Interface {
                platform,
                buffer: serialize_buf,
                updated: false,
            },
            settings,
        )))
    }

    /// Get the device communication interface
    pub fn interface_mut(&mut self) -> &mut P::Interface {
        self.0.interface.platform.interface_mut()
    }

    /// Must be called periodically to process user input.
    ///
    /// # Returns
    /// A boolean indicating true if the settings were modified.
    pub fn process(
        &mut self,
        settings: &mut P::Settings,
    ) -> Result<bool, <P::Interface as embedded_io::ErrorType>::Error> {
        self.0.interface.updated = false;

        while self.interface_mut().read_ready()? {
            let mut buffer = [0u8; 64];
            let count = self.interface_mut().read(&mut buffer)?;
            for &value in &buffer[..count] {
                self.0.input_byte(value, settings);
            }
        }

        Ok(self.0.interface.updated)
    }
}
