use serde::{Deserialize, Serialize};

mod attenuators;
mod rf_power;

use super::hal;
use super::hrtimer::HighResTimerE;

use attenuators::AttenuatorInterface;
use rf_power::PowerMeasurementInterface;

use embedded_hal::{adc::OneShot, blocking::spi::Transfer};

const EXT_CLK_SEL_PIN: u8 = 8 + 7;
#[allow(dead_code)]
const OSC_EN_N_PIN: u8 = 8 + 6;
const ATT_RST_N_PIN: u8 = 8 + 5;
const ATT_LE3_PIN: u8 = 8 + 3;
const ATT_LE2_PIN: u8 = 8 + 2;
const ATT_LE1_PIN: u8 = 8 + 1;
const ATT_LE0_PIN: u8 = 8 + 0;

#[derive(Debug, Copy, Clone)]
pub enum Error {
    Spi,
    I2c,
    Dds,
    Qspi,
    Bounds,
    InvalidAddress,
    InvalidChannel,
    Adc,
    Access,
}

#[derive(Debug, Copy, Clone)]
pub enum Channel {
    In0,
    In1,
    Out0,
    Out1,
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

impl Into<ad9959::Channel> for Channel {
    /// Translate pounder channels to DDS output channels.
    fn into(self) -> ad9959::Channel {
        match self {
            Channel::In0 => ad9959::Channel::Two,
            Channel::In1 => ad9959::Channel::Four,
            Channel::Out0 => ad9959::Channel::One,
            Channel::Out1 => ad9959::Channel::Three,
        }
    }
}

/// A structure for the QSPI interface for the DDS.
pub struct QspiInterface {
    pub qspi: hal::qspi::Qspi,
    mode: ad9959::Mode,
    streaming: bool,
}

impl QspiInterface {
    /// Initialize the QSPI interface.
    ///
    /// Args:
    /// * `qspi` - The QSPI peripheral driver.
    pub fn new(mut qspi: hal::qspi::Qspi) -> Result<Self, Error> {
        // This driver only supports operation in 4-bit mode due to bus inconsistencies between the
        // QSPI peripheral and the DDS. Instead, we will bit-bang communications in
        // single-bit-two-wire to the DDS to configure it to 4-bit operation.
        qspi.configure_mode(hal::qspi::QspiMode::FourBit)
            .map_err(|_| Error::Qspi)?;
        Ok(Self {
            qspi,
            mode: ad9959::Mode::SingleBitTwoWire,
            streaming: false,
        })
    }

    pub fn start_stream(&mut self) -> Result<(), Error> {
        if self.qspi.is_busy() {
            return Err(Error::Qspi);
        }

        // Configure QSPI for infinite transaction mode using only a data phase (no instruction or
        // address).
        let qspi_regs = unsafe { &*hal::stm32::QUADSPI::ptr() };
        qspi_regs.fcr.modify(|_, w| w.ctcf().set_bit());

        unsafe {
            qspi_regs.dlr.write(|w| w.dl().bits(0xFFFF_FFFF));
            qspi_regs
                .ccr
                .modify(|_, w| w.imode().bits(0).fmode().bits(1));
        }

        self.streaming = true;

        Ok(())
    }

    pub fn write_profile(&mut self, data: [u32; 4]) -> Result<(), Error> {
        if self.streaming == false {
            return Err(Error::Qspi);
        }

        let qspi_regs = unsafe { &*hal::stm32::QUADSPI::ptr() };
        unsafe {
            core::ptr::write_volatile(
                &qspi_regs.dr as *const _ as *mut u32,
                data[0],
            );
            core::ptr::write_volatile(
                &qspi_regs.dr as *const _ as *mut u32,
                data[1],
            );
            core::ptr::write_volatile(
                &qspi_regs.dr as *const _ as *mut u32,
                data[2],
            );
            core::ptr::write_volatile(
                &qspi_regs.dr as *const _ as *mut u32,
                data[3],
            );
        }

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

                self.qspi
                    .write(encoded_address, &encoded_payload)
                    .map_err(|_| Error::Qspi)
            }
            ad9959::Mode::FourBitSerial => {
                if self.streaming {
                    Err(Error::Qspi)
                } else {
                    self.qspi.write(addr, data).map_err(|_| Error::Qspi)?;
                    Ok(())
                }
            }
            _ => Err(Error::Qspi),
        }
    }

    fn read(&mut self, addr: u8, dest: &mut [u8]) -> Result<(), Error> {
        if (addr & 0x80) != 0 {
            return Err(Error::InvalidAddress);
        }

        // This implementation only supports operation (read) in four-bit-serial mode.
        if self.mode != ad9959::Mode::FourBitSerial {
            return Err(Error::Qspi);
        }

        self.qspi
            .read(0x80_u8 | addr, dest)
            .map_err(|_| Error::Qspi)
    }
}

/// A structure containing implementation for Pounder hardware.
pub struct PounderDevices {
    pub ad9959: ad9959::Ad9959<QspiInterface>,
    pub io_update_trigger: HighResTimerE,
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
    /// * `ad9959` - The DDS driver for the pounder hardware.
    /// * `attenuator_spi` - A SPI interface to control digital attenuators.
    /// * `io_update_timer` - The HRTimer with the IO_update signal connected to the output.
    /// * `adc1` - The ADC1 peripheral for measuring power.
    /// * `adc2` - The ADC2 peripheral for measuring power.
    /// * `adc1_in_p` - The input channel for the RF power measurement on IN0.
    /// * `adc2_in_p` - The input channel for the RF power measurement on IN1.
    pub fn new(
        mcp23017: mcp23017::MCP23017<hal::i2c::I2c<hal::stm32::I2C1>>,
        ad9959: ad9959::Ad9959<QspiInterface>,
        io_update_trigger: HighResTimerE,
        attenuator_spi: hal::spi::Spi<hal::stm32::SPI1, hal::spi::Enabled, u8>,
        adc1: hal::adc::Adc<hal::stm32::ADC1, hal::adc::Enabled>,
        adc2: hal::adc::Adc<hal::stm32::ADC2, hal::adc::Enabled>,
        adc1_in_p: hal::gpio::gpiof::PF11<hal::gpio::Analog>,
        adc2_in_p: hal::gpio::gpiof::PF14<hal::gpio::Analog>,
    ) -> Result<Self, Error> {
        let mut devices = Self {
            mcp23017,
            io_update_trigger,
            ad9959,
            attenuator_spi,
            adc1,
            adc2,
            adc1_in_p,
            adc2_in_p,
        };

        // Configure power-on-default state for pounder. All LEDs are on, on-board oscillator
        // selected, attenuators out of reset. Note that testing indicates the output state needs to
        // be set first to properly update the output registers.
        devices
            .mcp23017
            .all_pin_mode(mcp23017::PinMode::OUTPUT)
            .map_err(|_| Error::I2c)?;
        devices
            .mcp23017
            .write_gpio(mcp23017::Port::GPIOA, 0x3F)
            .map_err(|_| Error::I2c)?;
        devices
            .mcp23017
            .write_gpio(mcp23017::Port::GPIOB, 1 << 5)
            .map_err(|_| Error::I2c)?;

        // Select the on-board clock with a 4x prescaler (400MHz).
        devices.select_onboard_clock(4u8)?;

        // Run the DDS in stream-only mode (no read support).
        devices.ad9959.interface.start_stream().unwrap();

        Ok(devices)
    }

    /// Select the an external for the DDS reference clock source.
    ///
    /// Args:
    /// * `frequency` - The frequency of the external clock source.
    /// * `multiplier` - The multiplier of the reference clock to use in the DDS.
    fn select_external_clock(
        &mut self,
        frequency: f32,
        prescaler: u8,
    ) -> Result<(), Error> {
        self.mcp23017
            .digital_write(EXT_CLK_SEL_PIN, true)
            .map_err(|_| Error::I2c)?;
        self.ad9959
            .configure_system_clock(frequency, prescaler)
            .map_err(|_| Error::Dds)?;

        Ok(())
    }

    /// Select the onboard oscillator for the DDS reference clock source.
    ///
    /// Args:
    /// * `multiplier` - The multiplier of the reference clock to use in the DDS.
    fn select_onboard_clock(&mut self, multiplier: u8) -> Result<(), Error> {
        self.mcp23017
            .digital_write(EXT_CLK_SEL_PIN, false)
            .map_err(|_| Error::I2c)?;
        self.ad9959
            .configure_system_clock(100_000_000f32, multiplier)
            .map_err(|_| Error::Dds)?;

        Ok(())
    }

    /// Configure the Pounder DDS clock.
    ///
    /// Args:
    /// * `config` - The configuration of the DDS clock desired.
    pub fn configure_dds_clock(
        &mut self,
        config: DdsClockConfig,
    ) -> Result<(), Error> {
        if config.external_clock {
            self.select_external_clock(
                config.reference_clock,
                config.multiplier,
            )
        } else {
            self.select_onboard_clock(config.multiplier)
        }
    }

    /// Get the pounder DDS clock configuration
    ///
    /// Returns:
    /// The current pounder DDS clock configuration.
    pub fn get_dds_clock_config(&mut self) -> Result<DdsClockConfig, Error> {
        let external_clock = self
            .mcp23017
            .digital_read(EXT_CLK_SEL_PIN)
            .map_err(|_| Error::I2c)?;
        let multiplier = self
            .ad9959
            .get_reference_clock_multiplier()
            .map_err(|_| Error::Dds)?;
        let reference_clock = self.ad9959.get_reference_clock_frequency();

        Ok(DdsClockConfig {
            multiplier,
            reference_clock,
            external_clock,
        })
    }

    /// Configure a DDS channel.
    ///
    /// Args:
    /// * `channel` - The pounder channel to configure.
    /// * `state` - The state to configure the channel for.
    pub fn set_channel_state(
        &mut self,
        channel: Channel,
        state: ChannelState,
    ) -> Result<(), Error> {
        let profile = self
            .ad9959
            .serialize_profile(
                channel.into(),
                state.parameters.frequency,
                state.parameters.phase_offset,
                state.parameters.amplitude,
            )
            .map_err(|_| Error::Dds)?;
        self.ad9959.interface.write_profile(profile).unwrap();
        self.io_update_trigger.trigger();

        self.set_attenuation(channel, state.attenuation)?;

        Ok(())
    }
}

impl AttenuatorInterface for PounderDevices {
    /// Reset all of the attenuators to a power-on default state.
    fn reset_attenuators(&mut self) -> Result<(), Error> {
        self.mcp23017
            .digital_write(ATT_RST_N_PIN, false)
            .map_err(|_| Error::I2c)?;
        // TODO: Measure the I2C transaction speed to the RST pin to ensure that the delay is
        // sufficient. Document the delay here.
        self.mcp23017
            .digital_write(ATT_RST_N_PIN, true)
            .map_err(|_| Error::I2c)?;

        Ok(())
    }

    /// Latch a configuration into a digital attenuator.
    ///
    /// Args:
    /// * `channel` - The attenuator channel to latch.
    fn latch_attenuators(&mut self, channel: Channel) -> Result<(), Error> {
        let pin = match channel {
            Channel::In0 => ATT_LE0_PIN,
            Channel::In1 => ATT_LE2_PIN,
            Channel::Out0 => ATT_LE1_PIN,
            Channel::Out1 => ATT_LE3_PIN,
        };

        self.mcp23017
            .digital_write(pin, true)
            .map_err(|_| Error::I2c)?;
        // TODO: Measure the I2C transaction speed to the RST pin to ensure that the delay is
        // sufficient. Document the delay here.
        self.mcp23017
            .digital_write(pin, false)
            .map_err(|_| Error::I2c)?;

        Ok(())
    }

    /// Read the raw attenuation codes stored in the attenuator shift registers.
    ///
    /// Args:
    /// * `channels` - A slice to store the channel readings into.
    fn read_all_attenuators(
        &mut self,
        channels: &mut [u8; 4],
    ) -> Result<(), Error> {
        self.attenuator_spi
            .transfer(channels)
            .map_err(|_| Error::Spi)?;

        Ok(())
    }

    /// Write the attenuator shift registers.
    ///
    /// Args:
    /// * `channels` - The data to write into the attenuators.
    fn write_all_attenuators(
        &mut self,
        channels: &[u8; 4],
    ) -> Result<(), Error> {
        let mut result = [0_u8; 4];
        result.clone_from_slice(channels);
        self.attenuator_spi
            .transfer(&mut result)
            .map_err(|_| Error::Spi)?;

        Ok(())
    }
}

impl PowerMeasurementInterface for PounderDevices {
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
