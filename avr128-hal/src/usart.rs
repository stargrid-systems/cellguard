//! Asynchronous USART (8N1) implementing [`embedded_io`] `Read`/`Write` (and
//! `ufmt::uWrite` with the `ufmt` feature).
//!
//! [`Usart`] is generic over a [`UsartInstance`]. Pin routing (`PORTMUX`) and
//! pin direction (TxD output, RxD input) are the application's responsibility.

#[cfg(feature = "ufmt")]
use core::convert::Infallible;

/// Default receive timeout, in milliseconds. A byte may never arrive, so the
/// blocking read gives up after this long. Override with
/// [`Usart::with_timeout_ms`].
const DEFAULT_RX_TIMEOUT_MS: u32 = 1000;

/// USART error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// No byte was received before the receive timeout elapsed.
    Timeout,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::Timeout => f.write_str("USART receive timed out"),
        }
    }
}

impl core::error::Error for Error {}

impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        match self {
            Error::Timeout => embedded_io::ErrorKind::TimedOut,
        }
    }
}

/// A USART peripheral. Implemented for each device's `USART0`..`USART5`. Not
/// for external use.
pub trait UsartInstance {
    /// Configures asynchronous 8N1 framing at the given baud register value and
    /// enables the transmitter and receiver.
    fn configure(&self, baud: u16);
    /// Whether the transmit data register can accept a byte.
    fn tx_ready(&self) -> bool;
    /// Whether the last frame has fully left the transmit shift register.
    fn tx_complete(&self) -> bool;
    /// Pushes a byte into the transmit data register.
    fn push(&self, byte: u8);
    /// Whether a received byte is available.
    fn rx_ready(&self) -> bool;
    /// Reads the received data register.
    fn pull(&self) -> u8;
}

/// Asynchronous USART built on a [`UsartInstance`].
pub struct Usart<T: UsartInstance> {
    instance: T,
    rx_budget: u32,
}

impl<T: UsartInstance> Usart<T> {
    /// Enables the USART in asynchronous 8N1 mode at `baud` bits/s, with the
    /// default receive timeout (1 s).
    ///
    /// `BAUD = (64 * f_CLK_PER) / (16 * baud)` (async normal mode). `configure`
    /// writes `BAUD`/`CTRLB`/`CTRLC` whole.
    #[must_use]
    pub fn new(instance: T, f_cpu_hz: u32, baud: u32) -> Self {
        Self::with_timeout_ms(instance, f_cpu_hz, baud, DEFAULT_RX_TIMEOUT_MS)
    }

    /// Like [`Usart::new`], but with a caller-chosen receive timeout in
    /// milliseconds (approximate, derived from `f_cpu_hz`).
    #[must_use]
    pub fn with_timeout_ms(instance: T, f_cpu_hz: u32, baud: u32, rx_timeout_ms: u32) -> Self {
        let baud_reg = ((64u64 * f_cpu_hz as u64) / (16 * baud as u64)) as u16;
        instance.configure(baud_reg);
        Self {
            instance,
            rx_budget: crate::wait::budget_ms(f_cpu_hz, rx_timeout_ms),
        }
    }

    /// Releases the underlying peripheral.
    pub fn free(self) -> T {
        self.instance
    }

    /// Blocks until the transmit buffer can accept a byte, then writes it. The
    /// transmitter is host-driven, so this cannot hang unless the peripheral is
    /// broken (in which case it panics rather than spinning forever).
    #[inline]
    pub fn write_byte(&mut self, byte: u8) {
        crate::wait::spin_until(|| self.instance.tx_ready());
        self.instance.push(byte);
    }

    /// Blocks until a byte is received, then returns it. Returns
    /// [`Error::Timeout`] if none arrives within the receive timeout.
    #[inline]
    pub fn read_byte(&mut self) -> Result<u8, Error> {
        for _ in 0..self.rx_budget {
            if self.instance.rx_ready() {
                return Ok(self.instance.pull());
            }
        }
        Err(Error::Timeout)
    }
}

impl<T: UsartInstance> embedded_io::ErrorType for Usart<T> {
    type Error = Error;
}

impl<T: UsartInstance> embedded_io::Write for Usart<T> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        for &b in buf {
            self.write_byte(b);
        }
        Ok(buf.len())
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        crate::wait::spin_until(|| self.instance.tx_complete());
        Ok(())
    }
}

impl<T: UsartInstance> embedded_io::Read for Usart<T> {
    // The contract wants `read` to block until at least one byte is available,
    // then return only what is ready. A UART never reaches EOF, so instead of
    // returning 0 we block on the first byte (up to the receive timeout) and
    // then drain whatever else is ready. Filling the whole buffer would deadlock
    // a caller waiting on a short final frame.
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }
        buf[0] = self.read_byte()?;
        let mut n = 1;
        while n < buf.len() && self.instance.rx_ready() {
            buf[n] = self.instance.pull();
            n += 1;
        }
        Ok(n)
    }
}

#[cfg(feature = "ufmt")]
impl<T: UsartInstance> ufmt::uWrite for Usart<T> {
    type Error = Infallible;
    fn write_str(&mut self, s: &str) -> Result<(), Infallible> {
        for &b in s.as_bytes() {
            self.write_byte(b);
        }
        Ok(())
    }
}

// Hidden implementation detail. The bodies are identical across the distinct
// PAC register types. This private macro only emits trait impls, not types.
macro_rules! impl_usart_instance {
    ($USART:ty) => {
        impl UsartInstance for $USART {
            fn configure(&self, baud: u16) {
                self.baud().write(|w| w.set(baud));
                self.ctrlc().write(|w| w.chsize()._8bit());
                self.ctrlb().write(|w| w.txen().set_bit().rxen().set_bit());
            }
            fn tx_ready(&self) -> bool {
                self.status().read().dreif().bit_is_set()
            }
            fn tx_complete(&self) -> bool {
                self.status().read().txcif().bit_is_set()
            }
            fn push(&self, byte: u8) {
                self.txdatal().write(|w| w.data().set(byte));
            }
            fn rx_ready(&self) -> bool {
                self.status().read().rxcif().bit_is_set()
            }
            fn pull(&self) -> u8 {
                self.rxdatal().read().data().bits()
            }
        }
    };
}

// One call per device (grouped, so instances never interleave and are hard to
// drop). db48 has USART0..4; db64/da64 add USART5.
macro_rules! impl_usarts {
    ($($USART:ty),+ $(,)?) => {
        $( impl_usart_instance!($USART); )+
    };
}

#[cfg(feature = "avr128db48")]
impl_usarts!(
    avr_device::avr128db48::USART0,
    avr_device::avr128db48::USART1,
    avr_device::avr128db48::USART2,
    avr_device::avr128db48::USART3,
    avr_device::avr128db48::USART4,
);
#[cfg(feature = "avr128db64")]
impl_usarts!(
    avr_device::avr128db64::USART0,
    avr_device::avr128db64::USART1,
    avr_device::avr128db64::USART2,
    avr_device::avr128db64::USART3,
    avr_device::avr128db64::USART4,
    avr_device::avr128db64::USART5,
);
#[cfg(feature = "avr128da64")]
impl_usarts!(
    avr_device::avr128da64::USART0,
    avr_device::avr128da64::USART1,
    avr_device::avr128da64::USART2,
    avr_device::avr128da64::USART3,
    avr_device::avr128da64::USART4,
    avr_device::avr128da64::USART5,
);
