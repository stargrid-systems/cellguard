use embedded_hal::spi::Error as SpiError;

pub struct Error<E: SpiError>(Inner<E>);

impl<E: SpiError> Error<E> {
    pub(crate) const fn spi(err: E) -> Self {
        Self(Inner::Spi(err))
    }
}

enum Inner<E: SpiError> {
    Spi(E),
    Kind(ErrorKind),
}

impl<E: SpiError> From<ErrorKind> for Error<E> {
    fn from(kind: ErrorKind) -> Self {
        Self(Inner::Kind(kind))
    }
}

pub enum ErrorKind {
    CrcMismatch,
}
