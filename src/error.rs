//!
//! Error for the Radio Module
//! 

pub enum Error<GPIOE, SPIE> {
    // Data Provided Exceeds the Size of the FIFO
    DataExceedsFifoSize,
    // Unexpected Configuration of Register x.  Expected y, but got z.
    UnexpectedConfiguration(u8, u8, u8),
    GpioError(GPIOE),
    SpiError(SPIE),
    GpioSpiError((GPIOE, SPIE)),
}