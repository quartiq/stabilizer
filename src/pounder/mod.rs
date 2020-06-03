use mcp23017;
use ad9959;

pub mod error;
pub mod attenuators;
mod rf_power;
pub mod types;

use super::hal;

use error::Error;
use attenuators::AttenuatorInterface;
use types::{DdsChannel, InputChannel};
use rf_power::PowerMeasurementInterface;

use embedded_hal::{
    blocking::spi::Transfer,
    adc::OneShot
};

#[allow(dead_code)]
const OSC_EN_N_PIN: u8 = 8 + 7;

const EXT_CLK_SEL_PIN: u8 = 8 + 6;

const ATT_RST_N_PIN: u8 = 8 + 5;

const ATT_LE0_PIN: u8 = 8 + 0;
const ATT_LE1_PIN: u8 = 8 + 1;
const ATT_LE2_PIN: u8 = 8 + 2;
const ATT_LE3_PIN: u8 = 8 + 3;

pub struct QspiInterface {
    pub qspi: hal::qspi::Qspi,
    mode: ad9959::Mode,
}

impl QspiInterface {
    pub fn new(mut qspi: hal::qspi::Qspi) -> Result<Self, Error> {
        qspi.configure_mode(hal::qspi::QspiMode::FourBit).map_err(|_| Error::Qspi)?;
        Ok(Self { qspi: qspi, mode: ad9959::Mode::SingleBitTwoWire })
    }
}

impl ad9959::Interface for QspiInterface {
    type Error = Error;

    fn configure_mode(&mut self, mode: ad9959::Mode) -> Result<(), Error> {
        self.mode = mode;

        Ok(())
    }

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
                        if address_bit % 2 == 0 {
                            4
                        } else {
                            0
                        }
                    };

                    if addr & address_bit != 0 {
                        encoded_data[(address_bit >> 1) as usize] |= 1 << offset;
                    }
                }

                // Encode the data into the remaining bytes.
                for byte_index in 0..data.len() {
                    let byte = data[byte_index];
                    for address_bit in 0..8 {
                        let offset: u8 = {
                            if address_bit % 2 == 0 {
                                4
                            } else {
                                0
                            }
                        };

                        if byte & address_bit != 0 {
                            encoded_data[(byte_index + 1) * 4 + (address_bit >> 1) as usize] |= 1 << offset;
                        }
                    }
                }

                let (encoded_address, encoded_payload) = {
                    let end_index = (1 + data.len()) * 4;
                    (encoded_data[0], &encoded_data[1..end_index])
                };

                self.qspi.write(encoded_address, &encoded_payload).map_err(|_| Error::Qspi)
            },
            ad9959::Mode::FourBitSerial => {
                self.qspi.write(addr, &data).map_err(|_| Error::Qspi)
            },
            _ => {
                Err(Error::Qspi)
            }
        }
    }

    fn read(&mut self, addr: u8, mut dest: &mut [u8]) -> Result<(), Error> {
        if (addr & 0x80) != 0 {
            return Err(Error::InvalidAddress);
        }

        // It is not possible to read data from the AD9959 in single bit two wire mode because the
        // QSPI interface assumes that data is always received on IO1.
        if self.mode == ad9959::Mode::SingleBitTwoWire {
            return Err(Error::Qspi);
        }

        self.qspi.read(0x80_u8 | addr, &mut dest).map_err(|_| Error::Qspi)
    }
}

pub struct PounderDevices<DELAY> {
    pub ad9959: ad9959::Ad9959<QspiInterface,
                               DELAY,
                               hal::gpio::gpiog::PG7<hal::gpio::Output<hal::gpio::PushPull>>>,
    mcp23017: mcp23017::MCP23017<hal::i2c::I2c<hal::stm32::I2C1>>,
    attenuator_spi: hal::spi::Spi<hal::stm32::SPI1>,
    adc1: hal::adc::Adc<hal::stm32::ADC1, hal::adc::Enabled>,
    adc2: hal::adc::Adc<hal::stm32::ADC2, hal::adc::Enabled>,
    adc1_in_p: hal::gpio::gpiof::PF11<hal::gpio::Analog>,
    adc2_in_p: hal::gpio::gpiof::PF14<hal::gpio::Analog>,
}

impl<DELAY> PounderDevices<DELAY>
where
    DELAY: embedded_hal::blocking::delay::DelayMs<u8>,
{
    pub fn new(mcp23017: mcp23017::MCP23017<hal::i2c::I2c<hal::stm32::I2C1>>,
               ad9959: ad9959::Ad9959<QspiInterface,
                                      DELAY,
                                      hal::gpio::gpiog::PG7<
                                        hal::gpio::Output<hal::gpio::PushPull>>>,
               attenuator_spi: hal::spi::Spi<hal::stm32::SPI1>,
               adc1: hal::adc::Adc<hal::stm32::ADC1, hal::adc::Enabled>,
               adc2: hal::adc::Adc<hal::stm32::ADC2, hal::adc::Enabled>,
               adc1_in_p: hal::gpio::gpiof::PF11<hal::gpio::Analog>,
               adc2_in_p: hal::gpio::gpiof::PF14<hal::gpio::Analog>,
               ) -> Result<Self, Error> {
        let mut devices = Self {
            mcp23017,
            ad9959,
            attenuator_spi,
            adc1,
            adc2,
            adc1_in_p,
            adc2_in_p,
        };

        // Configure power-on-default state for pounder. All LEDs are on, on-board oscillator
        // selected, attenuators out of reset.
        devices.mcp23017.write_gpio(mcp23017::Port::GPIOA, 0xF).map_err(|_| Error::I2c)?;
        devices.mcp23017.write_gpio(mcp23017::Port::GPIOB,
                                    1_u8.wrapping_shl(5)).map_err(|_| Error::I2c)?;
        devices.mcp23017.all_pin_mode(mcp23017::PinMode::OUTPUT).map_err(|_| Error::I2c)?;

        devices.select_onboard_clock()?;

        Ok(devices)
    }

    pub fn select_external_clock(&mut self, frequency: u32) -> Result<(), Error>{
        self.mcp23017.digital_write(EXT_CLK_SEL_PIN, true).map_err(|_| Error::I2c)?;
        self.ad9959.set_clock_frequency(frequency).map_err(|_| Error::DDS)?;

        Ok(())
    }

    pub fn select_onboard_clock(&mut self) -> Result<(), Error> {
        self.mcp23017.digital_write(EXT_CLK_SEL_PIN, false).map_err(|_| Error::I2c)?;
        self.ad9959.set_clock_frequency(100_000_000).map_err(|_| Error::DDS)?;

        Ok(())
    }
}

impl<DELAY> AttenuatorInterface for PounderDevices<DELAY>
{
    fn reset(&mut self) -> Result<(), Error> {
        self.mcp23017.digital_write(ATT_RST_N_PIN, true).map_err(|_| Error::I2c)?;
        // TODO: Measure the I2C transaction speed to the RST pin to ensure that the delay is
        // sufficient. Document the delay here.
        self.mcp23017.digital_write(ATT_RST_N_PIN, false).map_err(|_| Error::I2c)?;

        Ok(())
    }

    fn latch(&mut self, channel: DdsChannel) -> Result<(), Error> {
        let pin = match channel {
            DdsChannel::Zero => ATT_LE1_PIN,
            DdsChannel::One => ATT_LE0_PIN,
            DdsChannel::Two => ATT_LE3_PIN,
            DdsChannel::Three => ATT_LE2_PIN,
        };

        self.mcp23017.digital_write(pin, true).map_err(|_| Error::I2c)?;
        // TODO: Measure the I2C transaction speed to the RST pin to ensure that the delay is
        // sufficient. Document the delay here.
        self.mcp23017.digital_write(pin, false).map_err(|_| Error::I2c)?;

        Ok(())
    }

    fn read_all(&mut self, channels: &mut [u8; 4]) -> Result<(), Error> {
        self.attenuator_spi.transfer(channels).map_err(|_| Error::Spi)?;

        Ok(())
    }

    fn write_all(&mut self, channels: &[u8; 4]) -> Result<(), Error> {
        let mut result = [0_u8; 4];
        result.clone_from_slice(channels);
        self.attenuator_spi.transfer(&mut result).map_err(|_| Error::Spi)?;

        Ok(())
    }
}

impl<DELAY> PowerMeasurementInterface for PounderDevices<DELAY> {
    fn sample_converter(&mut self, channel: InputChannel) -> Result<f32, Error> {
        let adc_scale = match channel {
            InputChannel::Zero => {
                let adc_reading: u32 = self.adc1.read(&mut self.adc1_in_p).map_err(|_| Error::Adc)?;
                adc_reading as f32 / self.adc1.max_sample() as f32
            },
            InputChannel::One => {
                let adc_reading: u32 = self.adc2.read(&mut self.adc2_in_p).map_err(|_| Error::Adc)?;
                adc_reading as f32 / self.adc2.max_sample() as f32
            },
        };

        // Convert analog percentage to voltage.
        Ok(adc_scale * 3.3)
    }
}
