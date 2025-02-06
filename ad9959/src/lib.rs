#![no_std]

use arbitrary_int::{u10, u14, u2, u24, u3, u4, u5, Number};
use bitbybit::{bitenum, bitfield};
use embedded_hal::{blocking::delay::DelayUs, digital::v2::OutputPin};

/// A trait that allows a HAL to provide a means of communicating with the AD9959.
pub trait Interface {
    type Error;
    fn configure_mode(&mut self, mode: Mode) -> Result<(), Self::Error>;
    fn write(&mut self, addr: Address, data: &[u8]) -> Result<(), Self::Error>;
    fn read(
        &mut self,
        addr: Address,
        data: &mut [u8],
    ) -> Result<(), Self::Error>;
}

/// Indicates various communication modes of the DDS. The value of this enumeration is equivalent to
/// the configuration bits of the DDS CSR register.
#[derive(PartialEq)]
#[bitenum(u2, exhaustive = true)]
pub enum Mode {
    SingleBitTwoWire = 0b00,
    SingleBitThreeWire = 0b01,
    TwoBitSerial = 0b10,
    FourBitSerial = 0b11,
}

pub type Channel = u4;

#[bitfield(u8, default = 0xf0)]
#[derive(Debug, PartialEq)]
pub struct Csr {
    #[bit(0, rw)]
    lsb_first: bool,
    #[bits(1..=2, rw)]
    mode: Mode,
    #[bits(4..=7, rw)]
    channel: u4,
}

#[bitfield(u24, default = 0)]
#[derive(Debug, PartialEq)]
pub struct Fr1 {
    #[bit(0, rw)]
    sw_sync: bool,
    #[bit(1, rw)]
    hw_sync: bool,
    #[bit(4, rw)]
    dac_ref_pd: bool,
    #[bit(5, rw)]
    sync_clk_pd: bool,
    #[bit(6, rw)]
    ext_pd: bool,
    #[bit(7, rw)]
    ext_clk_pd: bool,
    #[bits(8..=9, rw)]
    modulation: u2,
    #[bits(10..=11, rw)]
    ramp_up_down: u2,
    #[bits(12..=14, rw)]
    profile_pin: u3,
    #[bits(16..=17, rw)]
    charge_pump: u2,
    #[bits(18..=22, rw)]
    pll_divier: u5,
    #[bit(23, rw)]
    vco_high: bool,
}

#[bitfield(u24, default = 0)]
#[derive(Debug, PartialEq)]
pub struct Acr {
    #[bits(0..=9, rw)]
    asf: u10,
    #[bit(10, rw)]
    load_arr: bool,
    #[bit(11, rw)]
    ramp: bool,
    #[bit(12, rw)]
    multiplier: bool,
    #[bits(14..=15, rw)]
    step: u2,
    #[bits(16..=23, rw)]
    arr: u8,
}

#[allow(clippy::upper_case_acronyms)]
#[bitenum(u7)]
pub enum Address {
    CSR = 0x00,
    FR1 = 0x01,
    FR2 = 0x02,
    CFR = 0x03,
    CFTW0 = 0x04,
    CPOW0 = 0x05,
    ACR = 0x06,
    LSRR = 0x07,
    RDW = 0x08,
    FDW = 0x09,
    CW1 = 0x0a,
    CW2 = 0x0b,
    CW3 = 0x0c,
    CW4 = 0x0d,
    CW5 = 0x0e,
    CW6 = 0x0f,
    CW7 = 0x10,
    CW8 = 0x11,
    CW9 = 0x12,
    CW10 = 0x13,
    CW11 = 0x14,
    CW12 = 0x15,
    CW13 = 0x16,
    CW14 = 0x17,
    CW15 = 0x18,
}

/// Possible errors generated by the AD9959 driver.
#[derive(Debug)]
pub enum Error {
    Interface,
    Check,
    Bounds,
    Pin,
    Frequency,
}

/// A device driver for the AD9959 direct digital synthesis (DDS) chip.
///
/// This chip provides four independently controllable digital-to-analog output sinusoids with
/// configurable phase, amplitude, and frequency. All channels are inherently synchronized as they
/// are derived off a common system clock.
///
/// The chip contains a configurable PLL and supports system clock frequencies up to 500 MHz.
///
/// The chip supports a number of serial interfaces to improve data throughput, including normal,
/// dual, and quad SPI configurations.
pub struct Ad9959<I> {
    interface: I,
    ftw_per_hz: f32,
    mode: Mode,
}

impl<I: Interface> Ad9959<I> {
    /// Construct and initialize the DDS.
    ///
    /// Args:
    /// * `interface` - An interface to the DDS.
    /// * `reset_pin` - A pin connected to the DDS reset input.
    /// * `io_update` - A pin connected to the DDS io_update input.
    /// * `delay` - A delay implementation for blocking operation for specific amounts of time.
    /// * `desired_mode` - The desired communication mode of the interface to the DDS.
    /// * `clock_frequency` - The clock frequency of the reference clock input.
    /// * `multiplier` - The desired clock multiplier for the system clock. This multiplies
    ///   `clock_frequency` to generate the system clock.
    pub fn new(
        interface: I,
        reset: &mut impl OutputPin,
        io_update: &mut impl OutputPin,
        delay: &mut impl DelayUs<u8>,
        mode: Mode,
        reference_clock_frequency: f32,
        multiplier: u5,
    ) -> Result<Self, Error> {
        let mut ad9959 = Ad9959 {
            interface,
            ftw_per_hz: 0.0,
            mode,
        };
        io_update.set_low().or(Err(Error::Pin))?;

        // Reset the AD9959 (Pounder v1.1 and earlier)
        // On Pounder v1.2 and later the reset has been done through the GPIO extender in
        // PounderDevices before.
        reset.set_high().or(Err(Error::Pin))?;
        // Delays here are at least 1 SYNC_CLK period. The SYNC_CLK is guaranteed
        // to be at least 250KHz (1/4 of 1MHz minimum REF_CLK). We use 5uS instead of 4uS to
        // guarantee conformance with datasheet requirements.
        delay.delay_us(5);
        reset.set_low().or(Err(Error::Pin))?;

        ad9959
            .interface
            .configure_mode(Mode::SingleBitTwoWire)
            .or(Err(Error::Interface))?;

        let csr = Csr::default().with_channel(u4::new(0b1111)).with_mode(mode);
        ad9959.write(Address::CSR, &[csr.raw_value()])?;

        io_update.set_high().or(Err(Error::Pin))?;
        delay.delay_us(5);
        io_update.set_low().or(Err(Error::Pin))?;

        ad9959
            .interface
            .configure_mode(mode)
            .or(Err(Error::Interface))?;

        // Empirical evidence indicates a delay is necessary here for the IO update to become
        // active. This is likely due to needing to wait at least 1 clock cycle of the DDS for the
        // interface update to occur.
        delay.delay_us(5);

        // Read back the CSR to ensure it specifies the mode correctly.
        let mut updated_csr = 0u8.to_be_bytes();
        ad9959.read(Address::CSR, &mut updated_csr)?;
        if updated_csr != csr.raw_value().to_be_bytes() {
            return Err(Error::Check);
        }

        // Set the clock frequency to configure the device as necessary.
        ad9959.set_system_clock(reference_clock_frequency, multiplier)?;
        io_update.set_high().or(Err(Error::Pin))?;
        delay.delay_us(5);
        io_update.set_low().or(Err(Error::Pin))?;

        Ok(ad9959)
    }

    fn read(&mut self, reg: Address, data: &mut [u8]) -> Result<(), Error> {
        self.interface.read(reg, data).or(Err(Error::Interface))
    }

    fn write(&mut self, reg: Address, data: &[u8]) -> Result<(), Error> {
        self.interface.write(reg, data).or(Err(Error::Interface))
    }

    /// Configure the internal system clock of the chip.
    ///
    /// Arguments:
    /// * `reference_clock_frequency` - The reference clock frequency provided to the AD9959 core.
    /// * `multiplier` - The frequency multiplier of the system clock. Must be 1 or 4-20.
    ///
    /// Returns:
    /// The actual frequency configured for the internal system clock.
    fn set_system_clock(
        &mut self,
        reference_clock_frequency: f32,
        multiplier: u5,
    ) -> Result<f32, Error> {
        let sysclk = multiplier.value() as f32 * reference_clock_frequency;
        if match multiplier.value() {
            1 => !(1e6..=500e6).contains(&reference_clock_frequency),
            4..=20 => {
                !(10e6..=125e6).contains(&reference_clock_frequency)
                    || !(100e6..=500e6).contains(&sysclk)
            }
            _ => false,
        } {
            return Err(Error::Bounds);
        }
        let mut fr1 = u24::new(0).to_be_bytes();
        self.read(Address::FR1, &mut fr1)?;
        let fr1 = Fr1::new_with_raw_value(u24::from_be_bytes(fr1))
            .with_pll_divier(multiplier)
            .with_vco_high(sysclk >= 200e6);
        self.write(Address::FR1, &fr1.raw_value().to_be_bytes())?;
        self.ftw_per_hz = (1u64 << 32) as f32 / sysclk;
        Ok(sysclk)
    }

    /// Get the current CSR register.
    pub fn csr(&mut self) -> Result<Csr, Error> {
        let mut data = u8::new(0).to_be_bytes();
        self.read(Address::FR1, &mut data)?;
        Ok(Csr::new_with_raw_value(u8::from_be_bytes(data)))
    }

    /// Get the current FR1 register.
    pub fn fr1(&mut self) -> Result<Fr1, Error> {
        let mut data = u24::new(0).to_be_bytes();
        self.read(Address::FR1, &mut data)?;
        Ok(Fr1::new_with_raw_value(u24::from_be_bytes(data)))
    }

    /// Perform a self-test of the communication interface.
    ///
    /// Note:
    /// This modifies the existing channel enables. They are restored upon exit.
    ///
    /// Returns:
    /// True if the self test succeeded. False otherwise.
    pub fn self_test(&mut self) -> Result<bool, Error> {
        let mut data = [0];

        // Get current CSR.
        self.read(Address::CSR, &mut data)?;
        let old_csr = data;

        let mut csr = Csr::new_with_raw_value(data[0]);

        // Enable all channels.
        csr.set_channel(u4::new(0b1111));
        self.write(Address::CSR, &csr.raw_value().to_be_bytes())?;
        self.read(Address::CSR, &mut data)?;
        if Csr::new_with_raw_value(data[0]).channel() != csr.channel() {
            return Ok(false);
        }

        // Clear all channel enables.
        csr.set_channel(u4::new(0b0000));
        self.write(Address::CSR, &csr.raw_value().to_be_bytes())?;
        self.read(Address::CSR, &mut data)?;
        if Csr::new_with_raw_value(data[0]).channel() != csr.channel() {
            return Ok(false);
        }

        // Restore the CSR.
        self.write(Address::CSR, &old_csr)?;

        Ok(true)
    }

    /// Get the current system clock frequency in Hz.
    fn system_clock_frequency(&self) -> f32 {
        (1u64 << 32) as f32 / self.ftw_per_hz
    }

    /// Update an output channel configuration register.
    ///
    /// Args:
    /// * `channel` - The channel to configure.
    /// * `register` - The register to update.
    /// * `data` - The contents to write to the provided register.
    fn write_channel(
        &mut self,
        channel: Channel,
        register: Address,
        data: &[u8],
    ) -> Result<(), Error> {
        // Disable all other outputs so that we can update the configuration register of only the
        // specified channel.
        let csr = Csr::default().with_channel(channel).with_mode(self.mode);
        self.write(Address::CSR, &csr.raw_value().to_be_bytes())?;
        self.write(register, data)?;
        Ok(())
    }

    /// Read a configuration register of a specific channel.
    ///
    /// Args:
    /// * `channel` - The channel to read.
    /// * `register` - The register to read.
    /// * `data` - A location to store the read register contents.
    fn read_channel(
        &mut self,
        channel: Channel,
        register: Address,
        data: &mut [u8],
    ) -> Result<(), Error> {
        let csr = Csr::default().with_channel(channel).with_mode(self.mode);
        self.write(Address::CSR, &csr.raw_value().to_be_bytes())?;
        self.read(register, data)?;
        Ok(())
    }

    /// Configure the phase of a specified channel.
    ///
    /// Arguments:
    /// * `channel` - The channel to configure the frequency of.
    /// * `phase_turns` - The desired phase offset in turns.
    ///
    /// Returns:
    /// The actual programmed phase offset of the channel in turns.
    pub fn set_phase(
        &mut self,
        channel: Channel,
        phase: f32,
    ) -> Result<f32, Error> {
        let pow = u14::new((phase * (1 << 14) as f32) as u16 & 0x3FFF);
        self.write_channel(
            channel,
            Address::CPOW0,
            &pow.value().to_be_bytes(),
        )?;
        Ok(pow.value() as f32 / (1 << 14) as f32)
    }

    /// Get the current phase of a specified channel.
    ///
    /// Args:
    /// * `channel` - The channel to get the phase of.
    ///
    /// Returns:
    /// The phase of the channel in turns.
    pub fn get_phase(&mut self, channel: Channel) -> Result<f32, Error> {
        let mut pow = 0u16.to_be_bytes();
        self.read_channel(channel, Address::CPOW0, &mut pow)?;
        let pow = u16::from_be_bytes(pow) & 0x3FFF;
        Ok(pow as f32 / (1 << 14) as f32)
    }

    /// Configure the amplitude of a specified channel.
    ///
    /// Arguments:
    /// * `channel` - The channel to configure the frequency of.
    /// * `amplitude` - A normalized amplitude setting [0, 1].
    ///
    /// Returns:
    /// The actual normalized amplitude of the channel relative to full-scale range.
    pub fn set_amplitude(
        &mut self,
        channel: Channel,
        amplitude: f32,
    ) -> Result<f32, Error> {
        if !(0.0..=1.0).contains(&amplitude) {
            return Err(Error::Bounds);
        }
        let asf = (amplitude * u10::MAX.value() as f32) as u16;
        let acr = if asf >= u10::MAX.value() {
            Acr::default().with_multiplier(false)
        } else {
            Acr::default().with_multiplier(true).with_asf(u10::new(asf))
        };
        self.write_channel(
            channel,
            Address::ACR,
            &acr.raw_value().to_be_bytes(),
        )?;
        Ok(asf as f32 / u10::MAX.value() as f32)
    }

    /// Get the configured amplitude of a channel.
    ///
    /// Args:
    /// * `channel` - The channel to get the amplitude of.
    ///
    /// Returns:
    /// The normalized amplitude of the channel.
    pub fn get_amplitude(&mut self, channel: Channel) -> Result<f32, Error> {
        let mut acr = u24::new(0).to_be_bytes();
        self.read_channel(channel, Address::ACR, &mut acr)?;
        let acr = Acr::new_with_raw_value(u24::from_be_bytes(acr));
        Ok(if acr.multiplier() {
            1.0
        } else {
            acr.asf().value() as f32 / u10::MAX.value() as f32
        })
    }

    /// Configure the frequency of a specified channel.
    ///
    /// Arguments:
    /// * `channel` - The channel to configure the frequency of.
    /// * `frequency` - The desired output frequency in Hz.
    ///
    /// Returns:
    /// The actual programmed frequency of the channel.
    pub fn set_frequency(
        &mut self,
        channel: Channel,
        frequency: f32,
    ) -> Result<f32, Error> {
        if frequency < 0.0 || frequency > self.system_clock_frequency() {
            return Err(Error::Bounds);
        }
        let ftw = (frequency * self.ftw_per_hz) as u32;
        self.write_channel(channel, Address::CFTW0, &ftw.to_be_bytes())?;
        Ok(ftw as f32 / self.ftw_per_hz)
    }

    /// Get the frequency of a channel.
    ///
    /// Arguments:
    /// * `channel` - The channel to get the frequency of.
    ///
    /// Returns:
    /// The frequency of the channel in Hz.
    pub fn get_frequency(&mut self, channel: Channel) -> Result<f32, Error> {
        let mut ftw = 0u32.to_be_bytes();
        self.read_channel(channel, Address::CFTW0, &mut ftw)?;
        let ftw = u32::from_be_bytes(ftw);
        Ok(ftw as f32 / self.ftw_per_hz)
    }

    /// Finalize DDS configuration
    ///
    /// # Note
    /// This is intended for when the DDS profiles will be written as a stream of data to the DDS.
    ///
    /// # Returns
    /// (i, mode) where `i` is the interface to the DDS and `mode` is the frozen `Mode`.
    pub fn freeze(self) -> (I, Mode) {
        (self.interface, self.mode)
    }
}

/// Represents a means of serializing a DDS profile for writing to a stream.
pub struct ProfileSerializer {
    mode: Mode,
    // reorder or pad to work around https://github.com/japaric/heapless/issues/305
    // TODO: check
    // heapless::Vec<u8, 32>, especially its extend_from_slice() is slow
    index: usize,
    data: [u8; 32],
}

impl ProfileSerializer {
    /// Construct a new serializer.
    ///
    /// # Args
    /// * `mode` - The communication mode of the DDS.
    pub fn new(mode: Mode) -> Self {
        Self {
            mode,
            index: 0,
            data: [0; 32],
        }
    }

    /// Update a number of channels with the requested profile.
    ///
    /// # Args
    /// * `channels` - A set of channels to apply the configuration to.
    /// * `ftw` - If provided, indicates a frequency tuning word for the channels.
    /// * `pow` - If provided, indicates a phase offset word for the channels.
    /// * `acr` - If provided, indicates the amplitude control register for the channels. The ACR
    ///   should be stored in the 3 LSB of the word. Note that if amplitude scaling is to be used,
    ///   the "Amplitude multiplier enable" bit must be set.
    #[inline]
    pub fn push(
        &mut self,
        channels: Channel,
        ftw: Option<u32>,
        pow: Option<u14>,
        acr: Option<Acr>,
    ) {
        self.push_write(
            Address::CSR,
            &Csr::default()
                .with_mode(self.mode)
                .with_channel(channels)
                .raw_value()
                .to_be_bytes(),
        );
        if let Some(ftw) = ftw {
            self.push_write(Address::CFTW0, &ftw.to_be_bytes());
        }
        if let Some(pow) = pow {
            self.push_write(Address::CPOW0, &pow.value().to_be_bytes());
        }
        if let Some(acr) = acr {
            self.push_write(Address::ACR, &acr.raw_value().to_be_bytes());
        }
    }

    /// Add a register write to the serialization data.
    fn push_write(&mut self, register: Address, value: &[u8]) {
        let data = &mut self.data[self.index..];
        data[0] = register as u8;
        data[1..1 + value.len()].copy_from_slice(value);
        self.index += 1 + value.len();
    }

    /// Get the serialized profile as a slice of 32-bit words.
    ///
    /// # Note
    /// The serialized profile will be padded to the next 32-bit word boundary by adding dummy
    /// writes to the CSR or LSRR registers.
    ///
    /// # Returns
    /// A slice of `u32` words representing the serialized profile.
    #[inline]
    pub fn finalize(&mut self) -> &[u32] {
        // Pad the buffer to 32-bit (4 byte) alignment by adding dummy writes to CSR and LSRR.
        // In the case of 1 byte padding, this instead pads with 5 bytes as there is no
        // valid single-byte write that could be used.
        if self.index & 1 != 0 {
            // Pad with 3 bytes
            self.push_write(Address::LSRR, &0u16.to_be_bytes());
        }
        if self.index & 2 != 0 {
            // Pad with 2 bytes
            self.push_write(
                Address::CSR,
                &Csr::default()
                    .with_mode(self.mode)
                    .raw_value()
                    .to_be_bytes(),
            );
        }
        bytemuck::cast_slice(&self.data[..self.index])
    }
}
