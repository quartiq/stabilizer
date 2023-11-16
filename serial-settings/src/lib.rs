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
//!   dfu
//!   service
//!   reboot
//!   factory-reset
//!   list
//!   get <item>
//!   set <item> <value>
//!   help [ <command> ]
//!
//! > dfu
//! Reset to DFU is not supported on this device
//!
//! > service
//! Service data not available
//!
//! > list
//! Available items:
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
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use miniconf::{JsonCoreSlash, TreeKey};
use serde::{Deserialize, Serialize};

/// Specifies the API required for objects that are used as settings with the serial terminal
/// interface.
pub trait Settings:
    for<'a> JsonCoreSlash<'a> + Serialize + Clone + for<'a> Deserialize<'a>
{
    /// Reset the settings to their default values.
    fn reset(&mut self) {}
}

pub trait Platform {
    /// This type specifies the interface to the user, for example, a USB CDC-ACM serial port.
    type Interface: embedded_io::Read
        + embedded_io::ReadReady
        + embedded_io::Write
        + embedded_io::WriteReady;

    /// Specifies the settings that are used on the device.
    type Settings: Settings;

    /// Specifies the type of storage used for persisting settings between device boots.
    type Storage: NorFlash;

    /// This function is called immediately before a device reset is initiated.
    ///
    /// # Args
    /// * `interface` - An object to write device informational messages to.
    fn reset(&mut self, _interface: impl embedded_io::Write) {}

    /// This device is called when a reboot to DFU (device firmware update) mode is requested.
    ///
    /// # Args
    /// * `interface` - An object to write device informational messages to.
    fn dfu(&mut self, mut interface: impl embedded_io::Write) {
        self.reset(&mut interface);
        writeln!(
            &mut interface,
            "Reset to DFU is not supported on this device"
        )
        .ok();
    }

    /// This function is called to provide information about the operating device state to the
    /// user.
    ///
    /// # Args
    /// * `interface` - An object to write device status to.
    fn service(&mut self, mut interface: impl embedded_io::Write) {
        writeln!(&mut interface, "Service data not available").ok();
    }
}

struct Context<'a, P: Platform> {
    flash: P::Storage,
    platform: P,
    settings: P::Settings,
    interface: P::Interface,
    buffer: &'a mut [u8],
}

#[derive(Debug)]
enum Error<F> {
    Postcard(postcard::Error),
    Flash(F),
}

impl<F> From<postcard::Error> for Error<F> {
    fn from(e: postcard::Error) -> Self {
        Self::Postcard(e)
    }
}

impl<'a, P: Platform> Context<'a, P> {
    fn save(
        &mut self,
    ) -> Result<
        (),
        Error<<P::Storage as embedded_storage::nor_flash::ErrorType>::Error>,
    > {
        let serialized = postcard::to_slice(&self.settings, self.buffer)?;
        self.flash
            .erase(0, serialized.len() as u32)
            .map_err(Error::Flash)?;
        self.flash.write(0, serialized).map_err(Error::Flash)?;
        Ok(())
    }

    fn handle_reboot(
        _menu: &menu::Menu<Self>,
        _item: &menu::Item<Self>,
        _args: &[&str],
        context: &mut Self,
    ) {
        context.platform.reset(&mut context.interface);
        cortex_m::peripheral::SCB::sys_reset();
    }

    fn handle_list(
        _menu: &menu::Menu<Self>,
        _item: &menu::Item<Self>,
        _args: &[&str],
        context: &mut Self,
    ) {
        writeln!(context, "Available items:").unwrap();

        let mut defaults = context.settings.clone();
        defaults.reset();

        for path in P::Settings::iter_paths::<heapless::String<32>>("/") {
            let path = path.unwrap();
            let current_value = {
                let len =
                    context.settings.get_json(&path, context.buffer).unwrap();
                core::str::from_utf8(&context.buffer[..len]).unwrap()
            };
            write!(context.interface, "{path}: {current_value}").unwrap();

            let default_value = {
                let len = defaults.get_json(&path, context.buffer).unwrap();
                core::str::from_utf8(&context.buffer[..len]).unwrap()
            };
            writeln!(context.interface, " [default: {default_value}]").unwrap()
        }
    }

    fn handle_dfu(
        _menu: &menu::Menu<Self>,
        _item: &menu::Item<Self>,
        _args: &[&str],
        context: &mut Self,
    ) {
        context.platform.dfu(&mut context.interface);
    }

    fn handle_service(
        _menu: &menu::Menu<Self>,
        _item: &menu::Item<Self>,
        _args: &[&str],
        context: &mut Self,
    ) {
        context.platform.service(&mut context.interface);
    }

    fn handle_factory_reset(
        _menu: &menu::Menu<Self>,
        _item: &menu::Item<Self>,
        _args: &[&str],
        context: &mut Self,
    ) {
        context.settings.reset();
        match context.save() {
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
        let len = match context.settings.get_json(key, context.buffer) {
            Err(e) => {
                writeln!(context, "Failed to read {key}: {e}").unwrap();
                return;
            }
            Ok(len) => len,
        };

        let stringified = core::str::from_utf8(&context.buffer[..len]).unwrap();
        writeln!(context.interface, "{key}: {stringified}").unwrap();
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
        match context.settings.set_json(key, value.as_bytes()) {
            Ok(_) => match context.save() {
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
                command: "dfu",
                help: Some("Reboot the device to DFU mode"),
                item_type: menu::ItemType::Callback {
                    function: Self::handle_dfu,
                    parameters: &[]
                },
            },
            &menu::Item {
                command: "service",
                help: Some("Read device service information"),
                item_type: menu::ItemType::Callback {
                    function: Self::handle_service,
                    parameters: &[]
                },
            },
            &menu::Item {
                command: "reboot",
                help: Some("Reboot the device to force new settings to take effect."),
                item_type: menu::ItemType::Callback {
                    function: Self::handle_reboot,
                    parameters: &[]
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
        if let Ok(true) = self.interface.write_ready() {
            self.interface.write_all(s.as_bytes()).ok();
        }
        Ok(())
    }
}

/// The serial settings management object.
pub struct SerialSettings<'a, P: Platform> {
    menu: menu::Runner<'a, Context<'a, P>>,
}

impl<'a, P: Platform> SerialSettings<'a, P> {
    /// Constructor
    ///
    /// # Args
    /// * `platform` - The platform associated with the serial settings, providing the necessary
    /// context and API to manage device settings.
    /// * `interface` - The interface to read/write data to/from serially (via text) to the user.
    /// * `flash` - The storage mechanism used to persist settings to between boots.
    /// * `settings_callback` - A function called after the settings are loaded from memory. If no
    /// settings were found, `None` is provided. This function should provide the initial device
    /// settings.
    /// * `line_buf` - A buffer used for maintaining the serial menu input line. It should be at
    /// least as long as the longest user input.
    /// * `serialize_buf` - A buffer used for serializing and deserializing settings. This buffer
    /// needs to be at least as big as the entire serialized settings structure.
    pub fn new(
        platform: P,
        interface: P::Interface,
        mut flash: P::Storage,
        settings_callback: impl FnOnce(Option<P::Settings>) -> P::Settings,
        line_buf: &'a mut [u8],
        serialize_buf: &'a mut [u8],
    ) -> Result<
        Self,
        <P::Storage as embedded_storage::nor_flash::ErrorType>::Error,
    > {
        flash.read(0, &mut serialize_buf[..])?;

        let settings = settings_callback(
            postcard::from_bytes::<P::Settings>(serialize_buf).ok(),
        );

        let context = Context {
            settings,
            interface,
            flash,
            platform,
            buffer: serialize_buf,
        };

        let menu = menu::Runner::new(Context::menu(), line_buf, context);
        Ok(Self { menu })
    }

    /// Get the current device settings.
    pub fn settings(&self) -> &P::Settings {
        &self.menu.context.settings
    }

    /// Get the device communication interface
    pub fn interface_mut(&mut self) -> &mut P::Interface {
        &mut self.menu.context.interface
    }

    /// Get the device communication interface
    pub fn interface(&self) -> &P::Interface {
        &self.menu.context.interface
    }

    /// Must be called periodically to process user input.
    pub fn process(
        &mut self,
    ) -> Result<(), <P::Interface as embedded_io::ErrorType>::Error> {
        while self.menu.context.interface.read_ready()? {
            let mut buffer = [0u8; 64];
            let count = self.menu.context.interface.read(&mut buffer)?;
            for &value in &buffer[..count] {
                self.menu.input_byte(value);
            }
        }

        Ok(())
    }
}
