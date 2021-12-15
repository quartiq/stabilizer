use super::hal;
use embedded_hal::{adc::OneShot, blocking::spi::Transfer};
use serde::{Deserialize, Serialize};

pub mod attenuators;
pub mod dds_output;
pub mod hrtimer;
pub mod rf_power;

#[cfg(feature = "pounder_v1_1")]
pub mod timestamp;

pub enum GpioPin {
    Led4Green = 0,
    Led5Red = 1,
    Led6Green = 2,
    Led7Red = 3,
    Led8Green = 4,
    Led9Red = 5,
    AttLe0 = 8,
    AttLe1 = 8 + 1,
    AttLe2 = 8 + 2,
    AttLe3 = 8 + 3,
    AttRstN = 8 + 5,
    OscEnN = 8 + 6,
    ExtClkSel = 8 + 7,
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

#[derive(Serialize, Deserialize, Copy, Clone, Debug)]
pub struct DdsChannelState {
    pub phase_offset: f32,
    pub frequency: f32,
    pub amplitude: f32,
    pub enabled: bool,
}

#[derive(Serialize, Deserialize, Copy, Clone, Debug)]
pub struct ChannelState {
    pub parameters: DdsChannelState,
    pub attenuation: f32,
}

#[derive(Serialize, Deserialize, Copy, Clone, Debug)]
pub struct InputChannelState {
    pub attenuation: f32,
    pub power: f32,
    pub mixer: DdsChannelState,
}

#[derive(Serialize, Deserialize, Copy, Clone, Debug)]
pub struct OutputChannelState {
    pub attenuation: f32,
    pub channel: DdsChannelState,
}

#[derive(Serialize, Deserialize, Copy, Clone, Debug)]
pub struct DdsClockConfig {
    pub multiplier: u8,
    pub reference_clock: f32,
    pub external_clock: bool,
}

impl From<Channel> for ad9959::Channel {
    /// Translate pounder channels to DDS output channels.
    fn from(other: Channel) -> Self {
        match other {
            Channel::In0 => Self::Two,
            Channel::In1 => Self::Four,
            Channel::Out0 => Self::One,
            Channel::Out1 => Self::Three,
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

                self.qspi.write(encoded_address.into(), encoded_payload)?;

                Ok(())
            }
            ad9959::Mode::FourBitSerial => {
                if self.streaming {
                    Err(Error::InvalidState)
                } else {
                    self.qspi.write(addr.into(), data)?;
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

        self.qspi.read((0x80_u8 | addr).into(), dest)?;

        Ok(())
    }
}

/// A structure containing implementation for Pounder hardware.
pub struct PounderDevices {
    mcp23017: mcp23017::MCP23017<hal::i2c::I2c<hal::stm32::I2C1>>,
    attenuator_spi: hal::spi::Spi<hal::stm32::SPI1, hal::spi::Enabled, u8>,
    adc1: hal::adc::Adc<hal::stm32::ADC1, hal::adc::Enabled>,
    adc2: hal::adc::Adc<hal::stm32::ADC2, hal::adc::Enabled>,
    adc1_in_p: hal::gpio::gpiof::PF11<hal::gpio::Analog>,
    adc2_in_p: hal::gpio::gpiof::PF14<hal::gpio::Analog>,
}

impl PounderDevices {
    /// Construct and initialize pounder-specific hardware.
    ///
    /// Args:
    /// * `attenuator_spi` - A SPI interface to control digital attenuators.
    /// * `adc1` - The ADC1 peripheral for measuring power.
    /// * `adc2` - The ADC2 peripheral for measuring power.
    /// * `adc1_in_p` - The input channel for the RF power measurement on IN0.
    /// * `adc2_in_p` - The input channel for the RF power measurement on IN1.
    pub fn new(
        mcp23017: mcp23017::MCP23017<hal::i2c::I2c<hal::stm32::I2C1>>,
        attenuator_spi: hal::spi::Spi<hal::stm32::SPI1, hal::spi::Enabled, u8>,
        adc1: hal::adc::Adc<hal::stm32::ADC1, hal::adc::Enabled>,
        adc2: hal::adc::Adc<hal::stm32::ADC2, hal::adc::Enabled>,
        adc1_in_p: hal::gpio::gpiof::PF11<hal::gpio::Analog>,
        adc2_in_p: hal::gpio::gpiof::PF14<hal::gpio::Analog>,
    ) -> Result<Self, Error> {
        let mut devices = Self {
            mcp23017,
            attenuator_spi,
            adc1,
            adc2,
            adc1_in_p,
            adc2_in_p,
        };

        // Configure power-on-default state for pounder. All LEDs are off, on-board oscillator
        // selected and enabled, attenuators out of reset. Note that testing indicates the
        // output state needs to be set first to properly update the output registers.
        devices
            .mcp23017
            .all_pin_mode(mcp23017::PinMode::OUTPUT)
            .map_err(|_| Error::I2c)?;
        devices
            .mcp23017
            .write_gpio(mcp23017::Port::GPIOA, 0x00)
            .map_err(|_| Error::I2c)?;
        devices
            .mcp23017
            .write_gpio(mcp23017::Port::GPIOB, 0x2F)
            .map_err(|_| Error::I2c)?;

        Ok(devices)
    }
}

impl attenuators::AttenuatorInterface for PounderDevices {
    /// Reset all of the attenuators to a power-on default state.
    fn reset_attenuators(&mut self) -> Result<(), Error> {
        self.mcp23017
            .write_gpio(mcp23017::Port::GPIOB, 0x0f)
            .map_err(|_| Error::I2c)?;
        // Duration of one I2C transaction is sufficiently long.
        self.mcp23017
            .write_gpio(mcp23017::Port::GPIOB, 0x2f)
            .map_err(|_| Error::I2c)?;

        Ok(())
    }

    /// Latch a configuration into a digital attenuator.
    ///
    /// Args:
    /// * `channel` - The attenuator channel to latch.
    fn latch_attenuator(&mut self, channel: Channel) -> Result<(), Error> {
        let pin = channel as u8;
        self.mcp23017
            .write_gpio(mcp23017::Port::GPIOB, 0x2f & !(1 << pin))
            .map_err(|_| Error::I2c)?;
        // Duration of one I2C transaction is sufficiently long.
        self.mcp23017
            .write_gpio(mcp23017::Port::GPIOB, 0x2f)
            .map_err(|_| Error::I2c)?;
        Ok(())
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
            Channel::In0 => {
                let adc_reading: u32 = self
                    .adc1
                    .read(&mut self.adc1_in_p)
                    .map_err(|_| Error::Adc)?;
                adc_reading as f32 / self.adc1.max_sample() as f32
            }
            Channel::In1 => {
                let adc_reading: u32 = self
                    .adc2
                    .read(&mut self.adc2_in_p)
                    .map_err(|_| Error::Adc)?;
                adc_reading as f32 / self.adc2.max_sample() as f32
            }
            _ => return Err(Error::InvalidChannel),
        };

        // Convert analog percentage to voltage. Note that the ADC uses an external 2.048V analog
        // reference.
        Ok(adc_scale * 2.048)
    }
}
