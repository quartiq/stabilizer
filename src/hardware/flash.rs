use core::fmt::Write;
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use stm32h7xx_hal::flash::LockedFlashBank;

#[derive(Clone, serde::Serialize, serde::Deserialize, miniconf::Tree)]
pub struct Settings {
    pub broker: heapless::String<255>,
    pub id: heapless::String<23>,
    #[serde(skip)]
    #[tree(skip)]
    pub mac: smoltcp_nal::smoltcp::wire::EthernetAddress,
}

impl serial_settings::Settings for Settings {
    fn reset(&mut self) {
        *self = Self::new(self.mac)
    }
}

impl Settings {
    pub fn reload(&mut self, storage: &mut Flash) {
        let mut buffer = [0u8; 512];
        storage.read(0, &mut buffer).unwrap();
        let Ok(mut settings) = postcard::from_bytes::<Self>(&buffer) else {
            return;
        };

        settings.mac = self.mac;
        *self = settings;
    }

    pub fn new(mac: smoltcp_nal::smoltcp::wire::EthernetAddress) -> Self {
        let mut id = heapless::String::new();
        write!(&mut id, "{mac}").unwrap();

        Self {
            broker: "mqtt".into(),
            id,
            mac,
        }
    }
}

pub struct Flash(pub LockedFlashBank);

impl embedded_storage::nor_flash::ErrorType for Flash {
    type Error =
        <LockedFlashBank as embedded_storage::nor_flash::ErrorType>::Error;
}

impl embedded_storage::nor_flash::ReadNorFlash for Flash {
    const READ_SIZE: usize = LockedFlashBank::READ_SIZE;

    fn read(
        &mut self,
        offset: u32,
        bytes: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.0.read(offset, bytes)
    }

    fn capacity(&self) -> usize {
        self.0.capacity()
    }
}

impl embedded_storage::nor_flash::NorFlash for Flash {
    const WRITE_SIZE: usize =
        stm32h7xx_hal::flash::UnlockedFlashBank::WRITE_SIZE;
    const ERASE_SIZE: usize =
        stm32h7xx_hal::flash::UnlockedFlashBank::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        let mut bank = self.0.unlocked();
        bank.erase(from, to)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        let mut bank = self.0.unlocked();
        bank.write(offset, bytes)
    }
}

#[derive(Debug)]
pub enum Error<F> {
    Postcard(postcard::Error),
    Flash(F),
}

impl<F> From<postcard::Error> for Error<F> {
    fn from(e: postcard::Error) -> Self {
        Self::Postcard(e)
    }
}

pub struct SerialSettingsPlatform {
    /// The interface to read/write data to/from serially (via text) to the user.
    pub interface: serial_settings::BestEffortInterface<
        usbd_serial::SerialPort<'static, super::UsbBus>,
    >,
    /// The Settings structure.
    pub settings: Settings,
    /// The storage mechanism used to persist settings to between boots.
    pub storage: Flash,
}

impl serial_settings::Platform for SerialSettingsPlatform {
    type Interface = serial_settings::BestEffortInterface<
        usbd_serial::SerialPort<'static, super::UsbBus>,
    >;
    type Settings = Settings;
    type Error =
        Error<<Flash as embedded_storage::nor_flash::ErrorType>::Error>;

    fn save(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
        let serialized = postcard::to_slice(self.settings(), buf)?;
        self.storage
            .erase(0, serialized.len() as u32)
            .map_err(Self::Error::Flash)?;
        self.storage
            .write(0, serialized)
            .map_err(Self::Error::Flash)?;
        Ok(())
    }

    fn cmd(&mut self, cmd: &str) {
        match cmd {
            "reboot" => cortex_m::peripheral::SCB::sys_reset(),

            "dfu" => {
                // This process is largely adapted from
                // https://community.st.com/t5/stm32-mcus/jump-to-bootloader-from-application-on-stm32h7-devices/ta-p/49510
                cortex_m::interrupt::disable();

                // Disable the SysTick peripheral.
                let systick = unsafe { &*cortex_m::peripheral::SYST::PTR };
                unsafe {
                    systick.csr.write(0);
                    systick.rvr.write(0);
                    systick.cvr.write(0);
                }

                // Disable the USB peripheral.
                let usb_otg =
                    unsafe { &*stm32h7xx_hal::stm32::OTG2_HS_GLOBAL::ptr() };
                usb_otg.gccfg.write(|w| unsafe { w.bits(0) });

                // Reset the RCC configuration.
                let rcc = unsafe { &*stm32h7xx_hal::stm32::RCC::ptr() };

                // Enable the HSI - we will be switching back to it shortly for the DFU bootloader.
                rcc.cr.modify(|_, w| w.hsion().set_bit());

                // Wait for the HSI to enable
                while !rcc.cr.read().hsirdy().is_ready() {}

                // Reset the CFGR and begin using the HSI for the system bus.
                rcc.cfgr.reset();

                // Wait for the HSI to become the active clock.
                while !rcc.cfgr.read().sws().is_hsi() {}

                // Reset the configuration register.
                rcc.cr.reset();

                // Wait for the HSE to disable
                while rcc.cr.read().hserdy().is_ready() {}

                // Wait for the PLLs to disable
                while rcc.cr.read().pll1rdy().is_ready() {}
                while rcc.cr.read().pll2rdy().is_ready() {}
                while rcc.cr.read().pll3rdy().is_ready() {}

                rcc.d1cfgr.reset();
                rcc.d2cfgr.reset();
                rcc.d3cfgr.reset();

                // Reset the PLL configuration now that PLLs are unused.
                rcc.pllckselr.reset();
                rcc.pllcfgr.reset();
                rcc.pll1divr.reset();
                rcc.pll1fracr.reset();
                rcc.pll2divr.reset();
                rcc.pll2fracr.reset();
                rcc.pll3divr.reset();
                rcc.pll3fracr.reset();

                // Disable all RCC interrupts.
                rcc.cier.reset();

                // Clear all RCC interrupt flags
                unsafe { rcc.cicr.write(|w| w.bits(u32::MAX)) };

                rcc.rsr.write(|w| w.rmvf().set_bit());

                // Reset peripherals using the RCC.
                rcc.ahb1rstr.write(|w| w.usb2otgrst().set_bit());
                rcc.apb1lrstr.write(|w| {
                    w.i2c2rst()
                        .set_bit()
                        .i2c1rst()
                        .set_bit()
                        .spi2rst()
                        .set_bit()
                        .spi3rst()
                        .set_bit()
                });
                rcc.apb2rstr.write(|w| {
                    w.spi1rst()
                        .set_bit()
                        .spi4rst()
                        .set_bit()
                        .spi5rst()
                        .set_bit()
                });
                rcc.apb4rstr.write(|w| w.spi6rst().set_bit());

                // Reset DMA and ETH
                rcc.ahb1rstr.write(|w| {
                    w.dma1rst()
                        .set_bit()
                        .dma2rst()
                        .set_bit()
                        .eth1macrst()
                        .set_bit()
                });

                // Reset GPIO peripherals
                rcc.ahb4rstr.write(|w| {
                    w.gpioarst()
                        .set_bit()
                        .gpiobrst()
                        .set_bit()
                        .gpiocrst()
                        .set_bit()
                        .gpiodrst()
                        .set_bit()
                        .gpioerst()
                        .set_bit()
                        .gpiofrst()
                        .set_bit()
                        .gpiogrst()
                        .set_bit()
                });

                // Reset TIM2, TIM3, and TIM5
                rcc.apb1lrstr.write(|w| {
                    w.tim2rst()
                        .set_bit()
                        .tim3rst()
                        .set_bit()
                        .tim5rst()
                        .set_bit()
                });

                // Reset clocking registers
                rcc.d2ccip1r.reset();
                rcc.d1ccipr.reset();
                rcc.d3ccipr.reset();

                rcc.ahb1enr.reset();
                rcc.ahb2enr.reset();
                rcc.ahb3enr.reset();
                rcc.ahb4enr.reset();

                rcc.apb1lenr.reset();
                rcc.apb1henr.reset();
                rcc.apb2enr.reset();
                rcc.apb3enr.reset();
                rcc.apb4enr.reset();
                rcc.ahb1lpenr.reset();
                rcc.ahb2lpenr.reset();
                rcc.ahb3lpenr.reset();
                rcc.ahb4lpenr.reset();

                rcc.apb1llpenr.reset();
                rcc.apb1hlpenr.reset();
                rcc.apb2lpenr.reset();
                rcc.apb3lpenr.reset();
                rcc.apb4lpenr.reset();

                // Reset the PWR register.
                let pwr = unsafe { &*stm32h7xx_hal::stm32::PWR::ptr() };
                pwr.cr1.reset();
                pwr.cr2.reset();
                pwr.cr3.reset();
                pwr.d3cr.reset();
                pwr.cpucr.reset();

                // Clear NVIC interrupt flags and enables.
                let nvic = unsafe { &*cortex_m::peripheral::NVIC::PTR };
                for reg in nvic.icer.iter() {
                    unsafe {
                        reg.write(u32::MAX);
                    }
                }

                for reg in nvic.icpr.iter() {
                    unsafe {
                        reg.write(u32::MAX);
                    }
                }

                unsafe { cortex_m::interrupt::enable() };

                // The chip does not provide a means to modify the BOOT pins during
                // run-time. Jump to the bootloader in system memory instead.
                unsafe {
                    let system_memory_address: *const u32 =
                        0x1FF0_9800 as *const u32;
                    log::info!("Jumping to DFU");
                    cortex_m::asm::bootload(system_memory_address);
                }
            }
            _ => writeln!(
                self.interface_mut(),
                "Invalid platform command `{cmd}`"
            ),
        }
        .ok();
    }

    fn settings(&self) -> &Self::Settings {
        &self.settings
    }

    fn settings_mut(&mut self) -> &mut Self::Settings {
        &mut self.settings
    }

    fn interface_mut(&mut self) -> &mut Self::Interface {
        &mut self.interface
    }
}
