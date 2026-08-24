use embedded_hal::spi::Error as SpiError;

/// A frame-level SPI exchange with the device failed.
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

/// Why a frame-level exchange failed beyond the bus error itself.
pub enum CommunicationErrorKind {
    /// The response frame CRC did not match its contents.
    CrcMismatch,
}

/// The device did not reset as expected.
pub enum ResetError<E: SpiError> {
    /// SPI communication failed.
    Communication(CommunicationError<E>),
    /// The device did not report the expected post-reset response.
    NotReset,
}

impl<E: SpiError> From<CommunicationError<E>> for ResetError<E> {
    fn from(err: CommunicationError<E>) -> Self {
        Self::Communication(err)
    }
}

/// The registers failed to lock or unlock.
pub enum LockError<E: SpiError> {
    /// SPI communication failed.
    Communication(CommunicationError<E>),
    /// The lock state did not change as requested.
    Failed,
}

impl<E: SpiError> From<CommunicationError<E>> for LockError<E> {
    fn from(err: CommunicationError<E>) -> Self {
        Self::Communication(err)
    }
}

/// Failed to write to registers.
pub enum WriteError<E: SpiError> {
    /// SPI communication failed.
    Communication(CommunicationError<E>),
    /// The registers did not read back as written.
    Verify,
}

impl<E: SpiError> From<CommunicationError<E>> for WriteError<E> {
    fn from(err: CommunicationError<E>) -> Self {
        Self::Communication(err)
    }
}

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

impl<E: SpiError> From<WriteError<E>> for ConfigError<E> {
    fn from(err: WriteError<E>) -> Self {
        match err {
            WriteError::Communication(err) => Self::Communication(err),
            WriteError::Verify => Self::Verify,
        }
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
