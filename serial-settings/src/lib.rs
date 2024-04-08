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
//! Settings are specified in a [`miniconf::Tree`] settings tree and are transferred over the
//! serial interface using JSON encoding. This means that things like strings must be encased in
//! qutoes.
//!
//! # Limitations
//! Currently, there is a hardcoded limit of 64-bytes on the settings path. This is arbitrary and
//! can be changed if needed.
#![no_std]

use core::hash::Hasher;
use embedded_io::Write;
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
        + embedded_io::Write;

    /// Specifies the settings that are used on the device.
    type Settings: Settings<Y>;

    /// `save()` Error type
    type Error: core::fmt::Debug;

    /// Save the settings to storage
    fn save(&mut self, buffer: &mut [u8]) -> Result<(), Self::Error>;

    /// Execute a platform specific command.
    fn cmd(&mut self, cmd: &str);

    /// Return a mutable reference to the `Interface`.
    fn interface_mut(&mut self) -> &mut Self::Interface;

    fn settings(&self) -> &Self::Settings;
    fn settings_mut(&mut self) -> &mut Self::Settings;
}

pub struct Context<'a, P: Platform<Y>, const Y: usize> {
    platform: P,
    buffer: &'a mut [u8],
    updated: bool,
}

impl<'a, P: Platform<Y>, const Y: usize> Context<'a, P, Y> {
    pub fn new(platform: P, buffer: &'a mut [u8]) -> Self {
        Self {
            platform,
            buffer,
            updated: false,
        }
    }

    pub fn platform_mut(&mut self) -> &mut P {
        &mut self.platform
    }

    pub fn platform(&self) -> &P {
        &self.platform
    }
}

impl<'a, P: Platform<Y>, const Y: usize> Context<'a, P, Y> {
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

        for path in P::Settings::iter_paths::<heapless::String<64>>("/") {
            match path {
                Err(e) => writeln!(context, "Failed to get path: {e}").unwrap(),
                Ok(path) => {
                    let value = match context
                        .platform
                        .settings()
                        .get_json(&path, context.buffer)
                    {
                        Err(e) => {
                            writeln!(context, "Failed to read {path}: {e}")
                                .unwrap();
                            continue;
                        }
                        Ok(len) => core::str::from_utf8(&context.buffer[..len])
                            .unwrap(),
                    };

                    write!(
                        &mut context.platform.interface_mut(),
                        "{path}: {value}"
                    )
                    .unwrap();

                    let value_hash = {
                        let mut hasher = yafnv::Fnv1aHasher::default();
                        hasher.write(value.as_bytes());
                        hasher.finish()
                    };

                    let default_value = match defaults
                        .get_json(&path, context.buffer)
                    {
                        Err(e) => {
                            writeln!(
                                context,
                                "[default serialization error: {e}]"
                            )
                            .unwrap();
                            continue;
                        }
                        Ok(len) => core::str::from_utf8(&context.buffer[..len])
                            .unwrap(),
                    };

                    let default_hash = {
                        let mut hasher = yafnv::Fnv1aHasher::default();
                        hasher.write(default_value.as_bytes());
                        hasher.finish()
                    };
                    if default_hash != value_hash {
                        writeln!(
                            &mut context.platform.interface_mut(),
                            " [default: {default_value}]"
                        )
                        .unwrap()
                    } else {
                        writeln!(
                            &mut context.platform.interface_mut(),
                            " [default]"
                        )
                        .unwrap()
                    }
                }
            }
        }
    }

    fn handle_clear(
        _menu: &menu::Menu<Self>,
        item: &menu::Item<Self>,
        args: &[&str],
        context: &mut Self,
    ) {
        if let Some(key) = menu::argument_finder(item, args, "item").unwrap() {
            let mut defaults = context.platform.settings().clone();
            defaults.reset();

            let len = match defaults.get_json(key, context.buffer) {
                Err(e) => {
                    writeln!(context, "Failed to clear `{key}`: {e:?}")
                        .unwrap();
                    return;
                }

                Ok(len) => len,
            };

            if let Err(e) = context
                .platform
                .settings_mut()
                .set_json(key, &context.buffer[..len])
            {
                writeln!(context, "Failed to update {key}: {e:?}").unwrap();
                return;
            }

            context.updated = true;
            writeln!(context, "{key} cleared to default").unwrap();
        } else {
            context.platform.settings_mut().reset();
            context.updated = true;
            writeln!(context, "All settings cleared").unwrap();
        }

        match context.platform.save(context.buffer) {
            Ok(_) => {
                writeln!(context, "Settings saved. Reboot device (`platform reboot`) to apply.")
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
            Err(e) => writeln!(context, "Failed to read {key}: {e}").unwrap(),
            Ok(len) => writeln!(
                &mut context.platform.interface_mut(),
                "{key}: {}",
                core::str::from_utf8(&context.buffer[..len]).unwrap()
            )
            .unwrap(),
        }
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
        match context.platform.settings_mut().set_json(key, value.as_bytes())
        {
            Ok(_) => {
                context.updated = true;
                match context.platform.save(context.buffer) {
                    Ok(_) => {
                        writeln!(
                                context,
                                "Settings saved. Reboot device (`platform reboot`) to apply."
                            )
                    }
                    Err(e) => {
                        writeln!(context, "Failed to save settings: {e:?}")
                    }
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

impl<'a, P: Platform<Y>, const Y: usize> embedded_io::ReadReady
    for Context<'a, P, Y>
{
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        self.platform.interface_mut().read_ready()
    }
}

impl<'a, P: Platform<Y>, const Y: usize> embedded_io::ErrorType
    for Context<'a, P, Y>
{
    type Error =
        <<P as Platform<Y>>::Interface as embedded_io::ErrorType>::Error;
}

impl<'a, P: Platform<Y>, const Y: usize> embedded_io::Write
    for Context<'a, P, Y>
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.platform.interface_mut().write(buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.platform.interface_mut().flush()
    }
}

impl<'a, P: Platform<Y>, const Y: usize> embedded_io::Read
    for Context<'a, P, Y>
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.platform.interface_mut().read(buf)
    }
}

/// The serial settings management object.
pub struct Runner<'a, P: Platform<Y>, const Y: usize>(
    menu::Runner<'a, Context<'a, P, Y>>,
);

impl<'a, P: Platform<Y>, const Y: usize> Runner<'a, P, Y> {
    /// Constructor
    ///
    /// # Args
    /// * `context` - The [Context] associated with the serial settings, providing the necessary
    /// context and API to manage device settings.
    /// * `line_buf` - A buffer used for maintaining the serial menu input line. It should be at
    /// least as long as the longest user input.
    pub fn new(
        line_buf: &'a mut [u8],
        context: &mut Context<'a, P, Y>,
    ) -> Result<Self, P::Error> {
        Ok(Self(menu::Runner::new(Context::menu(), line_buf, context)))
    }

    /// Must be called periodically to process user input.
    ///
    /// # Returns
    /// A boolean indicating true if the settings were modified.
    pub fn process(
        &mut self,
        context: &mut Context<'a, P, Y>,
    ) -> Result<bool, <P::Interface as embedded_io::ErrorType>::Error> {
        context.updated = false;
        self.0.process(context);
        Ok(context.updated)
    }
}
