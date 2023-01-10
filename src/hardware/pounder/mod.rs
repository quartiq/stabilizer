use self::attenuators::AttenuatorInterface;

use super::hal;
use crate::hardware::{shared_adc::AdcChannel, I2c1Proxy};
use ad9959::{amplitude_to_acr, frequency_to_ftw, phase_to_pow, Profile};
use embedded_hal::blocking::spi::Transfer;
use enum_iterator::Sequence;
use miniconf::Miniconf;
use serde::{Deserialize, Serialize};

pub mod attenuators;
pub mod dds_output;
pub mod hrtimer;
pub mod rf_power;

#[cfg(not(feature = "pounder_v1_0"))]
pub mod timestamp;

#[derive(Debug, Copy, Clone, Sequence)]
pub enum GpioPin {
    Led4Green,
    Led5Red,
    Led6Green,
    Led7Red,
    Led8Green,
    Led9Red,
    AttLe0,
    AttLe1,
    AttLe2,
    AttLe3,
    AttRstN,
    OscEnN,
    ExtClkSel,
}

impl From<GpioPin> for mcp230xx::Mcp23017 {
    fn from(x: GpioPin) -> Self {
        match x {
            GpioPin::Led4Green => Self::A0,
            GpioPin::Led5Red => Self::A1,
            GpioPin::Led6Green => Self::A2,
            GpioPin::Led7Red => Self::A3,
            GpioPin::Led8Green => Self::A4,
            GpioPin::Led9Red => Self::A5,
            GpioPin::AttLe0 => Self::B0,
            GpioPin::AttLe1 => Self::B1,
            GpioPin::AttLe2 => Self::B2,
            GpioPin::AttLe3 => Self::B3,
            GpioPin::AttRstN => Self::B5,
            GpioPin::OscEnN => Self::B6,
            GpioPin::ExtClkSel => Self::B7,
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub enum Error {
    Spi,
    I2c,
    Qspi(hal::xspi::QspiError),
    Bounds,
    InvalidAddress,
    InvalidChannel,
    Adc,
    InvalidState,
}

impl From<hal::xspi::QspiError> for Error {
    fn from(e: hal::xspi::QspiError) -> Error {
        Error::Qspi(e)
    }
}

/// The numerical value (discriminant) of the Channel enum is the index in the attenuator shift
/// register as well as the attenuator latch enable signal index on the GPIO extender.
#[derive(Debug, Copy, Clone)]
#[allow(dead_code)]
pub enum Channel {
    In0 = 0,
    Out0 = 1,
    In1 = 2,
    Out1 = 3,
}

impl From<Channel> for GpioPin {
    fn from(x: Channel) -> Self {
        match x {
            Channel::In0 => GpioPin::AttLe0,
            Channel::Out0 => GpioPin::AttLe1,
            Channel::In1 => GpioPin::AttLe2,
            Channel::Out1 => GpioPin::AttLe3,
        }
    }
}

#[derive(Serialize, Deserialize, Copy, Clone, Debug, Miniconf)]
pub struct DdsChannelConfig {
    pub frequency: f32,
    pub phase_offset: f32,
    pub amplitude: f32,
}

impl Default for DdsChannelConfig {
    fn default() -> Self {
        Self {
            frequency: 0.0,
            phase_offset: 0.0,
            amplitude: 0.0,
        }
    }
}

pub struct ProfileWrapper(pub Profile);

impl From<(ClockConfig, ChannelConfig)> for ProfileWrapper {
    fn from((clocking, channel): (ClockConfig, ChannelConfig)) -> Self {
        let system_clock_frequency =
            clocking.reference_clock * clocking.multiplier as f32;
        ProfileWrapper(Profile {
            ftw: frequency_to_ftw(
                channel.dds.frequency,
                system_clock_frequency,
            )
            .unwrap(),
            pow: phase_to_pow(channel.dds.phase_offset).unwrap(),
            acr: amplitude_to_acr(channel.dds.amplitude).unwrap(),
        })
    }
}

#[derive(Serialize, Deserialize, Copy, Clone, Debug, Miniconf)]
pub struct ChannelConfig {
    #[miniconf(defer)]
    pub dds: DdsChannelConfig,
    pub attenuation: f32,
}

impl Default for ChannelConfig {
    fn default() -> Self {
        ChannelConfig {
            dds: DdsChannelConfig::default(),
            attenuation: 31.5,
        }
    }
}

#[derive(
    Serialize, Deserialize, Copy, Clone, Debug, PartialEq, Miniconf,
)]
pub struct ClockConfig {
    pub multiplier: u8,
    pub reference_clock: f32,
    pub external_clock: bool,
}

impl Default for ClockConfig {
    fn default() -> Self {
        Self {
            multiplier: super::design_parameters::DDS_MULTIPLIER,
            reference_clock: super::design_parameters::DDS_REF_CLK.to_Hz()
                as f32,
            external_clock: false,
        }
    }
}

#[derive(Serialize, Deserialize, Copy, Clone, Debug, Default, Miniconf)]
pub struct PounderConfig {
    pub clock: ClockConfig,
    #[miniconf(defer)]
    pub in_ch: [ChannelConfig; 2],
    #[miniconf(defer)]
    pub out_ch: [ChannelConfig; 2],
}

impl From<Channel> for ad9959::Channel {
    /// Translate pounder channels to DDS output channels.
    fn from(other: Channel) -> Self {
        match other {
            Channel::In0 => Self::TWO,
            Channel::In1 => Self::FOUR,
            Channel::Out0 => Self::ONE,
            Channel::Out1 => Self::THREE,
        }
    }
}

/// A structure for the QSPI interface for the DDS.
pub struct QspiInterface {
    pub qspi: hal::xspi::Qspi<hal::stm32::QUADSPI>,
    mode: ad9959::Mode,
    streaming: bool,
}

impl QspiInterface {
    /// Initialize the QSPI interface.
    ///
    /// Args:
    /// * `qspi` - The QSPI peripheral driver.
    pub fn new(
        mut qspi: hal::xspi::Qspi<hal::stm32::QUADSPI>,
    ) -> Result<Self, Error> {
        // This driver only supports operation in 4-bit mode due to bus inconsistencies between the
        // QSPI peripheral and the DDS. Instead, we will bit-bang communications in
        // single-bit-two-wire to the DDS to configure it to 4-bit operation.
        qspi.configure_mode(hal::xspi::QspiMode::FourBit)?;
        Ok(Self {
            qspi,
            mode: ad9959::Mode::SingleBitTwoWire,
            streaming: false,
        })
    }

    pub fn start_stream(&mut self) -> Result<(), Error> {
        self.qspi.is_busy()?;

        // Configure QSPI for infinite transaction mode using only a data phase (no instruction or
        // address).
        let qspi_regs = unsafe { &*hal::stm32::QUADSPI::ptr() };
        qspi_regs.fcr.modify(|_, w| w.ctcf().set_bit());

        unsafe {
            qspi_regs.dlr.write(|w| w.dl().bits(0xFFFF_FFFF));
            qspi_regs.ccr.modify(|_, w| {
                w.imode().bits(0).fmode().bits(0).admode().bits(0)
            });
        }

        self.streaming = true;

        Ok(())
    }
}

impl ad9959::Interface for QspiInterface {
    type Error = Error;

    /// Configure the operations mode of the interface.
    ///
    /// Args:
    /// * `mode` - The newly desired operational mode.
    fn configure_mode(&mut self, mode: ad9959::Mode) -> Result<(), Error> {
        self.mode = mode;

        Ok(())
    }

    /// Write data over QSPI to the DDS.
    ///
    /// Args:
    /// * `addr` - The address to write over QSPI to the DDS.
    /// * `data` - The data to write.
    fn write(&mut self, addr: u8, data: &[u8]) -> Result<(), Error> {
        if (addr & 0x80) != 0 {
            return Err(Error::InvalidAddress);
        }

        // The QSPI interface implementation always operates in 4-bit mode because the AD9959 uses
        // IO3 as SYNC_IO in some output modes. In order for writes to be successful, SYNC_IO must
        // be driven low. However, the QSPI peripheral forces IO3 high when operating in 1 or 2 bit
        // modes. As a result, any writes while in single- or dual-bit modes has to instead write
        // the data encoded into 4-bit QSPI data so that IO3 can be driven low.
        match self.mode {
            ad9959::Mode::SingleBitTwoWire => {
                // Encode the data into a 4-bit QSPI pattern.

                // In 4-bit mode, we can send 2 bits of address and data per byte transfer. As
                // such, we need at least 4x more bytes than the length of data. To avoid dynamic
                // allocation, we assume the maximum transaction length for single-bit-two-wire is
                // 2 bytes.
                let mut encoded_data: [u8; 12] = [0; 12];

                if (data.len() * 4) > (encoded_data.len() - 4) {
                    return Err(Error::Bounds);
                }

                // Encode the address into the first 4 bytes.
                for address_bit in 0..8 {
                    let offset: u8 = {
                        if address_bit % 2 != 0 {
                            4
                        } else {
                            0
                        }
                    };

                    // Encode MSB first. Least significant bits are placed at the most significant
                    // byte.
                    let byte_position = 3 - (address_bit >> 1) as usize;

                    if addr & (1 << address_bit) != 0 {
                        encoded_data[byte_position] |= 1 << offset;
                    }
                }

                // Encode the data into the remaining bytes.
                for byte_index in 0..data.len() {
                    let byte = data[byte_index];
                    for bit in 0..8 {
                        let offset: u8 = {
                            if bit % 2 != 0 {
                                4
                            } else {
                                0
                            }
                        };

                        // Encode MSB first. Least significant bits are placed at the most
                        // significant byte.
                        let byte_position = 3 - (bit >> 1) as usize;

                        if byte & (1 << bit) != 0 {
                            encoded_data
                                [(byte_index + 1) * 4 + byte_position] |=
                                1 << offset;
                        }
                    }
                }

                let (encoded_address, encoded_payload) = {
                    let end_index = (1 + data.len()) * 4;
                    (encoded_data[0], &encoded_data[1..end_index])
                };

                self.qspi.write(encoded_address, encoded_payload)?;

                Ok(())
            }
            ad9959::Mode::FourBitSerial => {
                if self.streaming {
                    Err(Error::InvalidState)
                } else {
                    self.qspi.write(addr, data)?;
                    Ok(())
                }
            }
            _ => Err(Error::InvalidState),
        }
    }

    fn read(&mut self, addr: u8, dest: &mut [u8]) -> Result<(), Error> {
        if (addr & 0x80) != 0 {
            return Err(Error::InvalidAddress);
        }

        // This implementation only supports operation (read) in four-bit-serial mode.
        if self.mode != ad9959::Mode::FourBitSerial {
            return Err(Error::InvalidState);
        }

        self.qspi.read(0x80 | addr, dest)?;

        Ok(())
    }
}

/// A structure containing implementation for Pounder hardware.
pub struct PounderDevices {
    mcp23017: mcp230xx::Mcp230xx<I2c1Proxy, mcp230xx::Mcp23017>,
    pub lm75: lm75::Lm75<I2c1Proxy, lm75::ic::Lm75>,
    attenuator_spi: hal::spi::Spi<hal::stm32::SPI1, hal::spi::Enabled, u8>,
    pwr0: AdcChannel<
        'static,
        hal::stm32::ADC1,
        hal::gpio::gpiof::PF11<hal::gpio::Analog>,
    >,
    pwr1: AdcChannel<
        'static,
        hal::stm32::ADC2,
        hal::gpio::gpiof::PF14<hal::gpio::Analog>,
    >,
    aux_adc0: AdcChannel<
        'static,
        hal::stm32::ADC3,
        hal::gpio::gpiof::PF3<hal::gpio::Analog>,
    >,
    aux_adc1: AdcChannel<
        'static,
        hal::stm32::ADC3,
        hal::gpio::gpiof::PF4<hal::gpio::Analog>,
    >,
}

impl PounderDevices {
    /// Construct and initialize pounder-specific hardware.
    ///
    /// Args:
    /// * `lm75` - The temperature sensor on Pounder.
    /// * `mcp23017` - The GPIO expander on Pounder.
    /// * `attenuator_spi` - A SPI interface to control digital attenuators.
    /// * `pwr0` - The ADC channel to measure the IN0 input power.
    /// * `pwr1` - The ADC channel to measure the IN1 input power.
    /// * `aux_adc0` - The ADC channel to measure the ADC0 auxiliary input.
    /// * `aux_adc1` - The ADC channel to measure the ADC1 auxiliary input.
    pub fn new(
        lm75: lm75::Lm75<I2c1Proxy, lm75::ic::Lm75>,
        mcp23017: mcp230xx::Mcp230xx<I2c1Proxy, mcp230xx::Mcp23017>,
        attenuator_spi: hal::spi::Spi<hal::stm32::SPI1, hal::spi::Enabled, u8>,
        pwr0: AdcChannel<
            'static,
            hal::stm32::ADC1,
            hal::gpio::gpiof::PF11<hal::gpio::Analog>,
        >,
        pwr1: AdcChannel<
            'static,
            hal::stm32::ADC2,
            hal::gpio::gpiof::PF14<hal::gpio::Analog>,
        >,
        aux_adc0: AdcChannel<
            'static,
            hal::stm32::ADC3,
            hal::gpio::gpiof::PF3<hal::gpio::Analog>,
        >,
        aux_adc1: AdcChannel<
            'static,
            hal::stm32::ADC3,
            hal::gpio::gpiof::PF4<hal::gpio::Analog>,
        >,
    ) -> Result<Self, Error> {
        let mut devices = Self {
            lm75,
            mcp23017,
            attenuator_spi,
            pwr0,
            pwr1,
            aux_adc0,
            aux_adc1,
        };

        // Configure power-on-default state for pounder. All LEDs are off, on-board oscillator
        // selected and enabled, attenuators out of reset. Note that testing indicates the
        // output state needs to be set first to properly update the output registers.
        for pin in enum_iterator::all::<GpioPin>() {
            devices
                .mcp23017
                .set_gpio(pin.into(), mcp230xx::Level::Low)
                .map_err(|_| Error::I2c)?;
            devices
                .mcp23017
                .set_direction(pin.into(), mcp230xx::Direction::Output)
                .map_err(|_| Error::I2c)?;
        }
        devices.reset_attenuators().unwrap();
        Ok(devices)
    }

    /// Sample one of the two auxiliary ADC channels associated with the respective RF input channel.
    pub fn sample_aux_adc(&mut self, channel: Channel) -> Result<f32, Error> {
        let adc_scale = match channel {
            Channel::In0 => self.aux_adc0.read_normalized().unwrap(),
            Channel::In1 => self.aux_adc1.read_normalized().unwrap(),
            _ => return Err(Error::InvalidChannel),
        };

        // Convert analog percentage to voltage. Note that the ADC uses an external 2.048V analog
        // reference.
        Ok(adc_scale * 2.048)
    }

    /// Set the state (its electrical level) of the given GPIO pin on Pounder.
    pub fn set_gpio_pin(
        &mut self,
        pin: GpioPin,
        level: mcp230xx::Level,
    ) -> Result<(), Error> {
        self.mcp23017
            .set_gpio(pin.into(), level)
            .map_err(|_| Error::I2c)
    }

    /// Select external reference clock input.
    pub fn set_ext_clk(&mut self, enabled: bool) -> Result<(), Error> {
        let level = if enabled {
            mcp230xx::Level::High
        } else {
            mcp230xx::Level::Low
        };
        // Active low
        self.set_gpio_pin(GpioPin::OscEnN, level)?;
        self.set_gpio_pin(GpioPin::ExtClkSel, level)
    }
}

impl attenuators::AttenuatorInterface for PounderDevices {
    /// Reset all of the attenuators to a power-on default state.
    fn reset_attenuators(&mut self) -> Result<(), Error> {
        // Active low
        self.set_gpio_pin(GpioPin::AttRstN, mcp230xx::Level::Low)?;
        self.set_gpio_pin(GpioPin::AttRstN, mcp230xx::Level::High)
    }

    /// Latch a configuration into a digital attenuator.
    ///
    /// Args:
    /// * `channel` - The attenuator channel to latch.
    fn latch_attenuator(&mut self, channel: Channel) -> Result<(), Error> {
        // Rising edge sensitive
        // Be robust against initial state: drive low, then high (contrary to the datasheet figure).
        self.set_gpio_pin(channel.into(), mcp230xx::Level::Low)?;
        self.set_gpio_pin(channel.into(), mcp230xx::Level::High)
    }

    /// Read the raw attenuation codes stored in the attenuator shift registers.
    ///
    /// Args:
    /// * `channels` - A 4 byte slice to be shifted into the
    ///     attenuators and to contain the data shifted out.
    fn transfer_attenuators(
        &mut self,
        channels: &mut [u8; 4],
    ) -> Result<(), Error> {
        self.attenuator_spi
            .transfer(channels)
            .map_err(|_| Error::Spi)?;

        Ok(())
    }
}

impl rf_power::PowerMeasurementInterface for PounderDevices {
    /// Sample an ADC channel.
    ///
    /// Args:
    /// * `channel` - The channel to sample.
    ///
    /// Returns:
    /// The sampled voltage of the specified channel.
    fn sample_converter(&mut self, channel: Channel) -> Result<f32, Error> {
        let adc_scale = match channel {
            Channel::In0 => self.pwr0.read_normalized().unwrap(),
            Channel::In1 => self.pwr1.read_normalized().unwrap(),
            _ => return Err(Error::InvalidChannel),
        };

        // Convert analog percentage to voltage. Note that the ADC uses an external 2.048V analog
        // reference.
        Ok(adc_scale * 2.048)
    }
}
