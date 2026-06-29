use embedded_hal::spi::Error as SpiError;

pub struct CommunicationError<E: SpiError>(CommunicationErrorInner<E>);

impl<E: SpiError> CommunicationError<E> {
    pub(crate) const fn spi(err: E) -> Self {
        Self(CommunicationErrorInner::Spi(err))
    }
}

enum CommunicationErrorInner<E: SpiError> {
    Spi(E),
    Kind(CommunicationErrorKind),
}

impl<E: SpiError> From<CommunicationErrorKind> for CommunicationError<E> {
    fn from(kind: CommunicationErrorKind) -> Self {
        Self(CommunicationErrorInner::Kind(kind))
    }
}

pub enum CommunicationErrorKind {
    CrcMismatch,
}

/// Error indicating that the device did not reset as expected.
pub struct ResetError;

/// Registers failed to lock.
pub struct LockError;

/// Failed to write to registers.
pub struct WriteError;

/// Failed to configure the device.
pub enum ConfigError<E: SpiError> {
    /// SPI communication failed.
    Communication(CommunicationError<E>),
    /// The configuration did not read back as written.
    Verify,
}

impl<E: SpiError> From<CommunicationError<E>> for ConfigError<E> {
    fn from(err: CommunicationError<E>) -> Self {
        Self::Communication(err)
    }
}

/// A failed state transition.
///
/// State transitions consume the driver. When one fails it would otherwise drop
/// the driver and with it access to the SPI bus. This carries the driver back
/// in its original state so the caller can inspect the error, retry, or reset.
pub struct TransitionError<D, E> {
    /// The driver, still in the state it had before the transition.
    pub device: D,
    /// Why the transition failed.
    pub error: E,
}

impl<D, E> TransitionError<D, E> {
    pub(crate) const fn new(device: D, error: E) -> Self {
        Self { device, error }
    }
}
