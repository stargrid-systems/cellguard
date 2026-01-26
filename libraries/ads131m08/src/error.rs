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
