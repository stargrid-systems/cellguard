//! Asynchronous USART (8N1) implementing [`embedded_io`] `Read`/`Write` (and
//! [`ufmt::uWrite`] with the `ufmt` feature).
//!
//! [`Usart`] is generic over a [`UsartInstance`]. Pin routing (`PORTMUX`) and
//! pin direction (TxD output, RxD input) are the application's responsibility.

use core::convert::Infallible;

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
}

impl<T: UsartInstance> Usart<T> {
    /// Enables the USART in asynchronous 8N1 mode at `baud` bits/s.
    ///
    /// `BAUD = (64 * f_CLK_PER) / (16 * baud)` (async normal mode).
    #[must_use]
    pub fn new(instance: T, f_cpu_hz: u32, baud: u32) -> Self {
        let baud_reg = ((64u64 * f_cpu_hz as u64) / (16 * baud as u64)) as u16;
        instance.configure(baud_reg);
        Self { instance }
    }

    /// Releases the underlying peripheral.
    pub fn free(self) -> T {
        self.instance
    }

    /// Blocks until the transmit buffer can accept a byte, then writes it.
    #[inline]
    pub fn write_byte(&mut self, byte: u8) {
        while !self.instance.tx_ready() {}
        self.instance.push(byte);
    }

    /// Blocks until a byte is received, then returns it.
    #[inline]
    pub fn read_byte(&mut self) -> u8 {
        while !self.instance.rx_ready() {}
        self.instance.pull()
    }
}

impl<T: UsartInstance> embedded_io::ErrorType for Usart<T> {
    type Error = Infallible;
}

impl<T: UsartInstance> embedded_io::Write for Usart<T> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        for &b in buf {
            self.write_byte(b);
        }
        Ok(buf.len())
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        while !self.instance.tx_complete() {}
        Ok(())
    }
}

impl<T: UsartInstance> embedded_io::Read for Usart<T> {
    // The contract wants `read` to block until at least one byte is available,
    // then return only what is ready. Returning 0 means EOF, which a UART never
    // reaches, so we always return at least 1. Filling the whole buffer would
    // deadlock a caller waiting on a short final frame.
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }
        buf[0] = self.read_byte();
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

// db48: USART0..4
#[cfg(feature = "avr128db48")]
impl_usart_instance!(avr_device::avr128db48::USART0);
#[cfg(feature = "avr128db48")]
impl_usart_instance!(avr_device::avr128db48::USART1);
#[cfg(feature = "avr128db48")]
impl_usart_instance!(avr_device::avr128db48::USART2);
#[cfg(feature = "avr128db48")]
impl_usart_instance!(avr_device::avr128db48::USART3);
#[cfg(feature = "avr128db48")]
impl_usart_instance!(avr_device::avr128db48::USART4);
// db64: USART0..5
#[cfg(feature = "avr128db64")]
impl_usart_instance!(avr_device::avr128db64::USART0);
#[cfg(feature = "avr128da64")]
impl_usart_instance!(avr_device::avr128da64::USART0);
#[cfg(feature = "avr128db64")]
impl_usart_instance!(avr_device::avr128db64::USART1);
#[cfg(feature = "avr128da64")]
impl_usart_instance!(avr_device::avr128da64::USART1);
#[cfg(feature = "avr128db64")]
impl_usart_instance!(avr_device::avr128db64::USART2);
#[cfg(feature = "avr128da64")]
impl_usart_instance!(avr_device::avr128da64::USART2);
#[cfg(feature = "avr128db64")]
impl_usart_instance!(avr_device::avr128db64::USART3);
#[cfg(feature = "avr128da64")]
impl_usart_instance!(avr_device::avr128da64::USART3);
#[cfg(feature = "avr128db64")]
impl_usart_instance!(avr_device::avr128db64::USART4);
#[cfg(feature = "avr128da64")]
impl_usart_instance!(avr_device::avr128da64::USART4);
#[cfg(feature = "avr128db64")]
impl_usart_instance!(avr_device::avr128db64::USART5);
#[cfg(feature = "avr128da64")]
impl_usart_instance!(avr_device::avr128da64::USART5);
