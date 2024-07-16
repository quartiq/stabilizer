//! Persistent Settings Management Serial Interface
//!
//! # Description
//! This crate provides a simple means to load, configure, and store device settings over a serial
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
//!   get [path]
//!   set <path> <value>
//!   store [path]
//!   clear [path]
//!   platform <cmd>
//!   help [ <command> ]
//!
//! > plaform dfu
//! Reset to DFU is not supported
//!
//! > plaform service
//! Service data not available
//!
//! > get
//! Available settings:
//! /broker: "test" [default: "mqtt"] [not stored]
//! /id: "04-91-62-d2-a8-6f" [default: "04-91-62-d2-a8-6f"] [not stored]
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

use embedded_io::{ErrorType, Read, ReadReady, Write};
use heapless::String;
use miniconf::{JsonCoreSlash, Path, Postcard, Traversal, TreeKey};

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

pub trait Platform<const Y: usize> {
    /// This type specifies the interface to the user, for example, a USB CDC-ACM serial port.
    type Interface: embedded_io::Read
        + embedded_io::ReadReady
        + embedded_io::Write;

    type Error: core::fmt::Debug;

    type Settings: Settings<Y>;

    fn fetch<'a>(
        &mut self,
        buf: &'a mut [u8],
        key: &[u8],
    ) -> Result<Option<&'a [u8]>, Self::Error>;

    /// Store the setting
    fn store(
        &mut self,
        buf: &mut [u8],
        key: &[u8],
        value: &[u8],
    ) -> Result<(), Self::Error>;

    fn clear(&mut self, buf: &mut [u8], key: &[u8]) -> Result<(), Self::Error>;

    /// Execute a platform specific command.
    fn cmd(&mut self, cmd: &str);

    /// Return a mutable reference to the `Interface`.
    fn interface_mut(&mut self) -> &mut Self::Interface;
}

struct Interface<'a, P, const Y: usize> {
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

    fn iter_root<F>(
        key: Option<&str>,
        interface: &mut Self,
        settings: &mut P::Settings,
        mut func: F,
    ) where
        F: FnMut(
            &Path<String<64>, '/'>,
            &mut Self,
            &mut P::Settings,
            &mut P::Settings,
        ),
    {
        let mut iter = P::Settings::nodes::<Path<String<64>, '/'>>();
        if let Some(key) = key {
            match iter.root(&Path::<_, '/'>::from(key)) {
                Ok(it) => iter = it,
                Err(e) => {
                    writeln!(interface, "Failed to locate `{key}`: {e}")
                        .unwrap();
                    return;
                }
            };
        }

        let mut defaults = settings.clone();
        defaults.reset();
        for key in iter {
            match key {
                Ok((key, node)) => {
                    debug_assert!(node.is_leaf());
                    func(&key, interface, settings, &mut defaults)
                }
                Err(depth) => {
                    writeln!(
                        interface,
                        "Failed to build path: no space at depth {depth}"
                    )
                    .unwrap();
                }
            }
        }
    }

    fn handle_get(
        _menu: &menu::Menu<Self, P::Settings>,
        item: &menu::Item<Self, P::Settings>,
        args: &[&str],
        interface: &mut Self,
        settings: &mut P::Settings,
    ) {
        let key = menu::argument_finder(item, args, "path").unwrap();
        Self::iter_root(
            key,
            interface,
            settings,
            |key, interface, settings, defaults| {
                // Get current
                let check = match settings
                    .get_json_by_key(key, interface.buffer)
                {
                    Err(miniconf::Error::Traversal(Traversal::Absent(_))) => {
                        return;
                    }
                    Err(e) => {
                        writeln!(
                            interface,
                            "Failed to get `{}`: {e}",
                            key.as_str()
                        )
                        .unwrap();
                        return;
                    }
                    Ok(len) => {
                        write!(
                            interface.platform.interface_mut(),
                            "{}: {}",
                            key.as_str(),
                            core::str::from_utf8(&interface.buffer[..len])
                                .unwrap()
                        )
                        .unwrap();
                        yafnv::fnv1a::<u32>(&interface.buffer[..len])
                    }
                };

                // Get default and compare
                match defaults.get_json_by_key(key, interface.buffer) {
                    Err(miniconf::Error::Traversal(Traversal::Absent(_))) => {
                        write!(interface, " [default: absent]")
                    }
                    Err(e) => {
                        write!(interface, " [default serialization error: {e}]")
                    }
                    Ok(len) => {
                        if yafnv::fnv1a::<u32>(&interface.buffer[..len])
                            != check
                        {
                            write!(
                                interface.platform.interface_mut(),
                                " [default: {}]",
                                core::str::from_utf8(&interface.buffer[..len])
                                    .unwrap()
                            )
                        } else {
                            write!(interface, " [default]")
                        }
                    }
                }
                .unwrap();

                // Get stored and compare
                match interface.platform.fetch(interface.buffer, key.as_bytes())
                {
                    Err(e) => write!(
                        interface,
                        " [fetch error: {e:?}]"
                    ),
                    Ok(None) =>
                        write!(interface, " [not stored]"),
                    Ok(Some(stored)) => {
                        let slic = postcard::de_flavors::Slice::new(stored);
                        // Temporary reload into default for conversion
                        match defaults.set_postcard_by_key(key, slic) {
                            Err(e) => write!(
                                interface,
                                " [stored deserialize error: {e}]"
                            ),
                            Ok(_rest) =>
                                match defaults.get_json_by_key(key, interface.buffer) {
                                    Err(e) => write!(
                                        interface,
                                        " [stored serialization error: {e}]"
                                    ),
                                    Ok(len) => {
                                        if yafnv::fnv1a::<u32>(&interface.buffer[..len]) != check {
                                        write!(
                                            interface.platform.interface_mut(),
                                            " [stored: {}]",
                                            core::str::from_utf8(&interface.buffer[..len]).unwrap())
                                        } else {
                                            write!(
                                                interface,
                                                " [stored]"
                                            )
                                        }
                                    },
                                }
                            }
                    }
                }.unwrap();
                writeln!(interface).unwrap();
            },
        );
    }

    fn handle_clear(
        _menu: &menu::Menu<Self, P::Settings>,
        item: &menu::Item<Self, P::Settings>,
        args: &[&str],
        interface: &mut Self,
        settings: &mut P::Settings,
    ) {
        let key = menu::argument_finder(item, args, "path").unwrap();
        Self::iter_root(
            key,
            interface,
            settings,
            |key, interface, settings, defaults| {
                let slic = postcard::ser_flavors::Slice::new(interface.buffer);
                let check = match settings.get_postcard_by_key(key, slic) {
                    Err(miniconf::Error::Traversal(Traversal::Absent(_))) => {
                        return;
                    }
                    Err(e) => {
                        writeln!(
                            interface,
                            "Failed to get {}: {e:?}",
                            key.as_str()
                        )
                        .unwrap();
                        return;
                    }
                    Ok(slic) => yafnv::fnv1a::<u32>(slic),
                };

                // Get default
                let slic = postcard::ser_flavors::Slice::new(interface.buffer);
                let slic = match defaults.get_postcard_by_key(key, slic) {
                    Err(miniconf::Error::Traversal(Traversal::Absent(_))) => {
                        log::warn!(
                            "Can't clear. Default is absent: `{}`",
                            key.as_str()
                        );
                        None
                    }
                    Err(e) => {
                        writeln!(
                            interface,
                            "Failed to get default `{}`: {e}",
                            key.as_str()
                        )
                        .unwrap();
                        return;
                    }
                    Ok(slic) => {
                        if yafnv::fnv1a::<u32>(slic) != check {
                            Some(slic)
                        } else {
                            None
                        }
                    }
                };

                // Set default
                if let Some(slic) = slic {
                    let slic = postcard::de_flavors::Slice::new(slic);
                    match settings.set_postcard_by_key(key, slic) {
                        Err(miniconf::Error::Traversal(Traversal::Absent(
                            _,
                        ))) => {
                            return;
                        }
                        Err(e) => {
                            writeln!(
                                interface,
                                "Failed to set {}: {e:?}",
                                key.as_str()
                            )
                            .unwrap();
                            return;
                        }
                        Ok(_rest) => {
                            interface.updated = true;
                            writeln!(
                                interface,
                                "Cleared current `{}`",
                                key.as_str()
                            )
                            .unwrap()
                        }
                    }
                }

                // Check for stored
                match interface.platform.fetch(interface.buffer, key.as_bytes())
                {
                    Err(e) => {
                        writeln!(
                            interface,
                            "Failed to fetch `{}`: {e:?}",
                            key.as_str()
                        )
                        .unwrap();
                    }
                    Ok(None) => {}
                    // Clear stored
                    Ok(Some(_stored)) => match interface
                        .platform
                        .clear(interface.buffer, key.as_bytes())
                    {
                        Ok(()) => {
                            writeln!(
                                interface,
                                "Clear stored `{}`",
                                key.as_str()
                            )
                        }
                        Err(e) => {
                            writeln!(
                                interface,
                                "Failed to clear `{}` from storage: {e:?}",
                                key.as_str()
                            )
                        }
                    }
                    .unwrap(),
                }
            },
        );
        interface.updated = true;
        writeln!(interface, "Some values may require reboot to become active")
            .unwrap();
    }

    fn handle_store(
        _menu: &menu::Menu<Self, P::Settings>,
        item: &menu::Item<Self, P::Settings>,
        args: &[&str],
        interface: &mut Self,
        settings: &mut P::Settings,
    ) {
        let key = menu::argument_finder(item, args, "path").unwrap();
        let force = menu::argument_finder(item, args, "force")
            .unwrap()
            .is_some();
        Self::iter_root(
            key,
            interface,
            settings,
            |key, interface, settings, defaults| {
                // Get default value checksum
                let slic = postcard::ser_flavors::Slice::new(interface.buffer);
                // Could in theory serialize directly into the hasher
                let mut check = match defaults.get_postcard_by_key(key, slic) {
                    Ok(slic) => yafnv::fnv1a::<u32>(slic),
                    Err(miniconf::Error::Traversal(Traversal::Absent(
                        _depth,
                    ))) => {
                        log::warn!("Default absent: `{}`", key.as_str());
                        return;
                    }
                    Err(e) => {
                        writeln!(
                            interface,
                            "Failed to get `{}` default: {e:?}",
                            key.as_str()
                        )
                        .unwrap();
                        return;
                    }
                };

                // Get stored value checksum
                match interface.platform.fetch(interface.buffer, key.as_bytes())
                {
                    Ok(None) => {}
                    Ok(Some(stored)) => {
                        let stored = yafnv::fnv1a::<u32>(stored);
                        if stored != check {
                            log::debug!(
                                "Stored differs from default: `{}`",
                                key.as_str()
                            );
                        } else {
                            log::debug!(
                                "Stored matches default: `{}`",
                                key.as_str()
                            );
                        }
                        check = stored;
                    }
                    Err(e) => {
                        writeln!(
                            interface,
                            "Failed to fetch `{}`: {e:?}",
                            key.as_str()
                        )
                        .unwrap();
                    }
                }

                // Get value
                let slic = postcard::ser_flavors::Slice::new(interface.buffer);
                let value = match settings.get_postcard_by_key(key, slic) {
                    Ok(value) => value,
                    Err(miniconf::Error::Traversal(Traversal::Absent(
                        _depth,
                    ))) => {
                        return;
                    }
                    Err(e) => {
                        writeln!(
                            interface,
                            "Could not get `{}`: {e}",
                            key.as_str()
                        )
                        .unwrap();
                        return;
                    }
                };

                // Check for mismatch
                if yafnv::fnv1a::<u32>(value) == check && !force {
                    log::debug!(
                        "Not saving matching default/stored `{}`",
                        key.as_str()
                    );
                    return;
                }
                let len = value.len();
                let (value, rest) = interface.buffer.split_at_mut(len);

                // Store
                match interface.platform.store(rest, key.as_bytes(), value) {
                    Ok(_) => writeln!(interface, "`{}` stored", key.as_str()),
                    Err(e) => writeln!(
                        interface,
                        "Failed to store `{}`: {e:?}",
                        key.as_str()
                    ),
                }
                .unwrap();
            },
        );
        writeln!(interface, "Some values may require reboot to become active")
            .unwrap();
    }

    fn handle_set(
        _menu: &menu::Menu<Self, P::Settings>,
        item: &menu::Item<Self, P::Settings>,
        args: &[&str],
        interface: &mut Self,
        settings: &mut P::Settings,
    ) {
        let key = menu::argument_finder(item, args, "path").unwrap().unwrap();
        let value =
            menu::argument_finder(item, args, "value").unwrap().unwrap();

        // Now, write the new value into memory.
        match settings.set_json(key, value.as_bytes()) {
            Ok(_) => {
                interface.updated = true;
                writeln!(
                    interface,
                    "Set but not stored. May require store and reboot to activate."
                )
            }
            Err(e) => {
                writeln!(interface, "Failed to set `{key}`: {e:?}")
            }
        }
        .unwrap();
    }

    fn menu() -> menu::Menu<'a, Self, P::Settings> {
        menu::Menu {
        label: "settings",
        items: &[
            &menu::Item {
                command: "get",
                help: Some("List paths and read current, default, and stored values"),
                item_type: menu::ItemType::Callback {
                    function: Self::handle_get,
                    parameters: &[menu::Parameter::Optional {
                        parameter_name: "path",
                        help: Some("The path of the value or subtree to list/read."),
                    }]
                },
            },
            &menu::Item {
                command: "set",
                help: Some("Update a value"),
                item_type: menu::ItemType::Callback {
                    function: Self::handle_set,
                    parameters: &[
                        menu::Parameter::Mandatory {
                            parameter_name: "path",
                            help: Some("The path to set"),
                        },
                        menu::Parameter::Mandatory {
                            parameter_name: "value",
                            help: Some("The value to be written, JSON-encoded"),
                        },
                    ]
                },
            },
            &menu::Item {
                command: "store",
                help: Some("Store values that differ from defaults"),
                item_type: menu::ItemType::Callback {
                    function: Self::handle_store,
                    parameters: &[
                        menu::Parameter::Named {
                            parameter_name: "force",
                            help: Some("Also store values that match defaults"),
                        },
                        menu::Parameter::Optional {
                            parameter_name: "path",
                            help: Some("The path of the value or subtree to store."),
                        },

                    ]
                },
            },
            &menu::Item {
                command: "clear",
                help: Some("Clear active to defaults and remove all stored values"),
                item_type: menu::ItemType::Callback {
                    function: Self::handle_clear,
                    parameters: &[
                        menu::Parameter::Optional {
                            parameter_name: "path",
                            help: Some("The path of the value or subtree to clear"),
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
        self.platform
            .interface_mut()
            .write_all(s.as_bytes())
            .or(Err(core::fmt::Error))
    }
}

impl<'a, P: Platform<Y>, const Y: usize> ErrorType for Interface<'a, P, Y> {
    type Error = <P::Interface as ErrorType>::Error;
}

impl<'a, P: Platform<Y>, const Y: usize> Write for Interface<'a, P, Y> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.platform.interface_mut().write(buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.platform.interface_mut().flush()
    }
}

// The Menu runner
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
    /// needs to be at least as big as twice the biggest serialized setting plus its path.
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

    pub fn platform_mut(&mut self) -> &mut P {
        &mut self.0.interface.platform
    }

    pub fn platform(&mut self) -> &P {
        &self.0.interface.platform
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
