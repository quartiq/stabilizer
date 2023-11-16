#![no_std]

use core::fmt::Write;
use embedded_storage::nor_flash::NorFlash;
use miniconf::JsonCoreSlash;
use serde::{Deserialize, Serialize};

pub use menu;

pub trait Settings:
    for<'a> JsonCoreSlash<'a>
    + Serialize
    + Clone
    + 'static
    + for<'a> Deserialize<'a>
{
    fn reset(&mut self);
}

pub trait Interface:
    embedded_io::Read
    + embedded_io::ReadReady
    + embedded_io::Write
    + embedded_io::WriteReady
{
}

impl<T> Interface for T where
    T: embedded_io::Read
        + embedded_io::ReadReady
        + embedded_io::Write
        + embedded_io::WriteReady
{
}

pub trait Platform {
    fn reset(&mut self, _interface: impl embedded_io::Write) {}

    fn dfu(&mut self, mut interface: impl embedded_io::Write) {
        self.reset(&mut interface);
        writeln!(
            &mut interface,
            "Reset to DFU is not supported on this device"
        )
        .ok();
    }

    fn service(&mut self, mut interface: impl embedded_io::Write) {
        writeln!(&mut interface, "Service data not available").ok();
    }
}

impl<T> Platform for T {}

pub struct Context<
    'a,
    I: Interface,
    S: Settings,
    P: Platform,
    Flash: NorFlash + 'static,
> {
    flash: Flash,
    platform: P,
    settings: S,
    interface: I,
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

impl<'a, I: Interface, S: Settings, P: Platform, Flash: NorFlash + 'static>
    Context<'a, I, S, P, Flash>
{
    fn save(&mut self) -> Result<(), Error<Flash::Error>> {
        let serialized = postcard::to_slice(&self.settings, self.buffer)?;
        self.flash
            .erase(0, serialized.len() as u32)
            .map_err(Error::Flash)?;
        self.flash.write(0, serialized).map_err(Error::Flash)?;
        Ok(())
    }
}

fn settings_menu<
    'a,
    I: Interface,
    S: Settings,
    P: Platform,
    Flash: NorFlash + 'static,
>() -> menu::Menu<'a, Context<'a, I, S, P, Flash>> {
    menu::Menu {
        label: "settings",
        items: &[
            &menu::Item {
                command: "dfu",
                help: Some("Reboot the device to DFU mode"),
                item_type: menu::ItemType::Callback {
                    function: handle_dfu_reboot,
                    parameters: &[]
                },
            },
            &menu::Item {
                command: "service",
                help: Some("Read device service information"),
                item_type: menu::ItemType::Callback {
                    function: handle_service,
                    parameters: &[]
                },
            },
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

impl<'a, I: Interface, S: Settings, P: Platform, Flash: NorFlash + 'static>
    core::fmt::Write for Context<'a, I, S, P, Flash>
{
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

fn handle_list<
    'a,
    I: Interface,
    S: Settings,
    P: Platform,
    Flash: NorFlash + 'static,
>(
    _menu: &menu::Menu<Context<'a, I, S, P, Flash>>,
    _item: &menu::Item<Context<'a, I, S, P, Flash>>,
    _args: &[&str],
    context: &mut Context<'a, I, S, P, Flash>,
) {
    writeln!(context, "Available items:").unwrap();

    let mut defaults = context.settings.clone();
    defaults.reset();

    for path in S::iter_paths::<heapless::String<32>>("/") {
        let path = path.unwrap();
        let current_value = {
            let len = context.settings.get_json(&path, context.buffer).unwrap();
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

fn handle_reboot<
    'a,
    I: Interface,
    S: Settings,
    P: Platform,
    Flash: NorFlash + 'static,
>(
    _menu: &menu::Menu<Context<'a, I, S, P, Flash>>,
    _item: &menu::Item<Context<'a, I, S, P, Flash>>,
    _args: &[&str],
    context: &mut Context<'a, I, S, P, Flash>,
) {
    context.platform.reset(&mut context.interface);
    cortex_m::peripheral::SCB::sys_reset();
}

fn handle_dfu_reboot<
    'a,
    I: Interface,
    S: Settings,
    P: Platform,
    Flash: NorFlash + 'static,
>(
    _menu: &menu::Menu<Context<'a, I, S, P, Flash>>,
    _item: &menu::Item<Context<'a, I, S, P, Flash>>,
    _args: &[&str],
    context: &mut Context<'a, I, S, P, Flash>,
) {
    context.platform.dfu(&mut context.interface);
}

fn handle_service<
    'a,
    I: Interface,
    S: Settings,
    P: Platform,
    Flash: NorFlash + 'static,
>(
    _menu: &menu::Menu<Context<'a, I, S, P, Flash>>,
    _item: &menu::Item<Context<'a, I, S, P, Flash>>,
    _args: &[&str],
    context: &mut Context<'a, I, S, P, Flash>,
) {
    context.platform.service(&mut context.interface);
}

fn handle_reset<
    'a,
    I: Interface,
    S: Settings,
    P: Platform,
    Flash: NorFlash + 'static,
>(
    _menu: &menu::Menu<Context<'a, I, S, P, Flash>>,
    _item: &menu::Item<Context<'a, I, S, P, Flash>>,
    _args: &[&str],
    context: &mut Context<'a, I, S, P, Flash>,
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

fn handle_get<
    'a,
    I: Interface,
    S: Settings,
    P: Platform,
    Flash: NorFlash + 'static,
>(
    _menu: &menu::Menu<Context<'a, I, S, P, Flash>>,
    item: &menu::Item<Context<'a, I, S, P, Flash>>,
    args: &[&str],
    context: &mut Context<'a, I, S, P, Flash>,
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

fn handle_set<
    'a,
    I: Interface,
    S: Settings,
    P: Platform,
    Flash: NorFlash + 'static,
>(
    _menu: &menu::Menu<Context<'a, I, S, P, Flash>>,
    item: &menu::Item<Context<'a, I, S, P, Flash>>,
    args: &[&str],
    context: &mut Context<'a, I, S, P, Flash>,
) {
    let key = menu::argument_finder(item, args, "item").unwrap().unwrap();
    let value = menu::argument_finder(item, args, "value").unwrap().unwrap();

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
                writeln!(context, "Failed to save settings: {e:?}").unwrap();
            }
        },
        Err(e) => {
            writeln!(context, "Failed to update {key}: {e:?}").unwrap();
        }
    }
}

pub struct SerialSettings<
    'a,
    I: Interface,
    S: Settings,
    P: Platform,
    Flash: NorFlash + 'static,
> {
    menu: menu::Runner<'a, Context<'a, I, S, P, Flash>>,
}

impl<'a, I: Interface, S: Settings, P: Platform, Flash: NorFlash + 'static>
    SerialSettings<'a, I, S, P, Flash>
{
    pub fn new(
        interface: I,
        mut flash: Flash,
        platform: P,
        settings_callback: impl FnOnce(Option<S>) -> S,
        line_buf: &'a mut [u8],
        serialize_buf: &'a mut [u8],
    ) -> Result<Self, Flash::Error> {
        flash.read(0, &mut serialize_buf[..])?;

        let settings =
            settings_callback(postcard::from_bytes::<S>(serialize_buf).ok());

        let context = Context {
            settings,
            interface,
            flash,
            platform,
            buffer: serialize_buf,
        };

        let menu = menu::Runner::new(settings_menu(), line_buf, context);
        Ok(Self { menu })
    }

    pub fn settings(&self) -> &S {
        &self.menu.context.settings
    }

    pub fn interface_mut(&mut self) -> &mut I {
        &mut self.menu.context.interface
    }

    pub fn interface(&self) -> &I {
        &self.menu.context.interface
    }

    pub fn process(&mut self) -> Result<(), I::Error> {
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
