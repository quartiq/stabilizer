use stm32h7::stm32h743 as pac;

// Adapted from stm32h7xx embedded-hal

// I2C error
#[derive(Debug)]
pub enum Error {
    // Bus error
    Bus,
    // Arbitration loss
    Arbitration,
    // Address not ACKd within a reasonable time (no device present?)
    Timeout,
    // Unexpected NACK during transfer
    NAck
}

// Maximum number of times to retry NACKed address phase before timing out
// Note that many devices indicate a busy condition by NACKing (e.g. 24xx
// EEPROMs during write)
const N_RETRY: usize = 100; // ~ 10ms @ 100 kHz bus clock


pub fn setup(i2c: &pac::I2C2) {
    // Disable the peripheral before setting timings
    i2c.cr1.modify(|_, w| w.pe().clear_bit());

    // Values from STM32MXCube for 100 kHz I2C clock with 100 MHz peripheral clock
    i2c.timingr.modify( |_,w|
        w.presc().bits(1)
         .scldel().bits(0x12)
         .sdadel().bits(0)
         .sclh().bits(0xec)
         .scll().bits(0xff)
    );

    // Enable the peripheral
    i2c.cr1.write(|w| w.pe().set_bit());
}


// Busy-wait for a flag to be asserted, erroring out on unrecoverable problems
macro_rules! busy_wait_errors {
    ($i2c:expr, $flag:ident) => {
        loop {
            let isr = $i2c.isr.read();

            if isr.berr().bit_is_set() {
                return Err(Error::Bus);
            } else if isr.arlo().bit_is_set() {
                return Err(Error::Arbitration);
            } else if isr.nackf().bit_is_set() {
                return Err(Error::NAck);
            } else if isr.$flag().bit_is_set() {
                break;
            }
        }
    };
}


fn poll_for_start_ack(
    i2c: &pac::I2C2,
    addr: u8,
    r_wn: bool,
    data_len: usize,
    autoend: bool,
    start: bool
) -> Result<(), Error>
{
    for _i in 0..N_RETRY {
        // START and prepare to send `data_len`
        i2c.cr2.write(|w| {
            w.start().bit(start)
             .sadd().bits(addr as u16)
             .add10().clear_bit()
             .rd_wrn().bit(r_wn)
             .nbytes().bits( data_len as u8 )
             .autoend().bit(autoend)
        });

        loop {
            let isr = i2c.isr.read();

            if isr.berr().bit_is_set() {
                return Err(Error::Bus);
            } else if isr.arlo().bit_is_set() {
                return Err(Error::Arbitration);
            } else if isr.nackf().bit_is_set() {
                i2c.icr.write(|w| { w.nackcf().set_bit() });
                // Wait to finish handling NACK-STOP
                loop {
                    if i2c.isr.read().busy().bit_is_clear() {
                        break;
                    }
                }
                break;
            } else if isr.txis().bit_is_set() || isr.rxne().bit_is_set() {
                return Ok(())
            }
        }
    }

    Err(Error::Timeout)
}


pub fn write_read(
    i2c: &pac::I2C2,
    addr: u8,
    bytes: &[u8],
    buffer: &mut [u8],
) -> Result<(), Error> {
    assert!(bytes.len() < 256 && !bytes.is_empty());
    assert!(buffer.len() < 256 && !buffer.is_empty());

    poll_for_start_ack(i2c, addr, false, bytes.len(), false, true)?;

    for byte in bytes {
        // Wait until we are allowed to send data (START has been ACKed or last
        // byte when through)
        busy_wait_errors!(i2c, txis);
        i2c.txdr.write(|w| w.txdata().bits(*byte));
    }

    // Wait until the last transmission is finished
    busy_wait_errors!(i2c, tc);

    poll_for_start_ack(i2c, addr|1, true, buffer.len(), true, true)?;

    for byte in buffer {
        // Wait until we have received something
        busy_wait_errors!(i2c, rxne);
        *byte = i2c.rxdr.read().rxdata().bits();
    }

    // automatic STOP
    Ok(())
}
