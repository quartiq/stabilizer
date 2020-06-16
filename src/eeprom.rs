use embedded_hal::blocking::i2c::WriteRead;

const I2C_ADDR: u8 = 0x50;

#[allow(dead_code)]
pub fn read_eui48<T>(i2c: &mut T) -> Result<[u8; 6], T::Error>
where
    T: WriteRead,
{
    let mut buffer = [0u8; 6];
    i2c.write_read(I2C_ADDR, &[0xFA_u8], &mut buffer)?;
    Ok(buffer)
}
