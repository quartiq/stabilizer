use embedded_hal::blocking::{delay::DelayMs, i2c::WriteRead};

// The EEPROM is a variant without address bits, so the 3 LSB of this word are "dont-cares".
const I2C_ADDR: u8 = 0x50;

// The MAC address is stored in the last 6 bytes of the 256 byte address space.
const MAC_POINTER: u8 = 0xFA;

pub fn read_eui48<T>(i2c: &mut T, delay: &mut impl DelayMs<u8>) -> [u8; 6]
where
    T: WriteRead,
{
    let mut previous_read: Option<[u8; 6]> = None;
    // On Stabilizer v1.1 and earlier hardware, there is a fault where the I2C bus is not connected
    // to the CPU until the P12V0A rail enables, which can take many seconds, or may never come up
    // at all. During these transient turn-on conditions, we may fail the I2C read operation. To
    // accomodate this, we repeat the I2C read for a set number of attempts with a fixed delay
    // between them. Then, we wait for the bus to stabilize by waiting until the MAC address
    // read-out is identical for two consecutive reads.
    for _ in 0..40 {
        let mut buffer = [0u8; 6];
        if i2c
            .write_read(I2C_ADDR, &[MAC_POINTER], &mut buffer)
            .is_ok()
        {
            if let Some(old_read) = previous_read {
                if old_read == buffer {
                    return buffer;
                }
            }

            previous_read.replace(buffer);
        } else {
            // Remove any pending previous read if we failed the last attempt.
            previous_read.take();
        }

        delay.delay_ms(100);
    }

    panic!("Failed to read MAC address");
}
