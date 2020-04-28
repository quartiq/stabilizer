#[derive(Debug)]
pub enum Error {
    Spi,
    I2c,
    DDS,
    Qspi,
    Bounds,
    InvalidAddress,
}
