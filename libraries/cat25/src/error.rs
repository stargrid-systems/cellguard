/// Errors returned by the CAT25 driver.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<E> {
    /// The requested address range falls outside the target memory.
    OutOfBounds,
    /// The device did not accept a write.
    ///
    /// The most likely cause is that the target is protected, by the block
    /// protection bits, the identification page lock, or the write protect pin.
    WriteProtected,
    /// A write cycle did not finish within the poll budget.
    Timeout,
    /// The underlying SPI device returned an error.
    Spi(E),
}
