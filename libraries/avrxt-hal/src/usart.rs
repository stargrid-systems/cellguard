//! Asynchronous USART implementing [`embedded_io`] `Read`/`Write` (and
//! `ufmt::uWrite` with the `ufmt` feature).
//!
//! [`Usart`] is generic over a [`UsartInstance`]. Build one with
//! [`Usart::builder`], which requires an explicit baud rate and [`Frame`].
//! There is no default frame. Use [`Frame::EIGHT_N_1`] for plain 8N1 or
//! [`Frame::EIGHT_E_2`] for a UPDI programmer. Pin routing (`PORTMUX`) and pin
//! direction (`TxD` output, `RxD` input) are the application's responsibility.

#[cfg(feature = "ufmt")]
use core::convert::Infallible;

pub use self::builder::{Builder, Unset};

mod builder;

/// Default receive timeout, in milliseconds. A byte may never arrive, so the
/// blocking read gives up after this long. Override with
/// [`Builder::rx_timeout_ms`].
const DEFAULT_RX_TIMEOUT_MS: u32 = 1000;

/// Baud register value for the slowest baud, used to stretch a BREAK.
const BREAK_BAUD_REG: u16 = u16::MAX;

/// USART error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// No byte was received before the receive timeout elapsed.
    Timeout,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Timeout => f.write_str("USART receive timed out"),
        }
    }
}

impl core::error::Error for Error {}

impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        match self {
            Self::Timeout => embedded_io::ErrorKind::TimedOut,
        }
    }
}

/// Parity mode of a USART frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Parity {
    /// No parity bit.
    None,
    /// Even parity.
    Even,
    /// Odd parity.
    Odd,
}

/// Number of stop bits in a USART frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StopBits {
    /// One stop bit.
    One,
    /// Two stop bits.
    Two,
}

/// USART frame format. The character size is fixed at 8 data bits. There is no
/// default: a caller must pick a frame explicitly.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frame {
    /// Parity mode.
    pub parity: Parity,
    /// Number of stop bits.
    pub stop_bits: StopBits,
}

impl Frame {
    /// 8 data bits, no parity, one stop bit.
    pub const EIGHT_N_1: Self = Self {
        parity: Parity::None,
        stop_bits: StopBits::One,
    };
    /// 8 data bits, even parity, two stop bits. The UPDI frame format.
    pub const EIGHT_E_2: Self = Self {
        parity: Parity::Even,
        stop_bits: StopBits::Two,
    };
}

/// Computes the async-normal-mode baud register value for `baud` bits/s.
///
/// `BAUD = (64 * f_CLK_PER) / (16 * baud)`. Returns [`None`] when the result
/// does not fit the 16-bit register, which happens when `baud` is too low (or
/// too high) for `f_cpu_hz`.
#[must_use]
#[expect(
    clippy::cast_possible_truncation,
    reason = "guarded by the `reg <= u16::MAX` check above"
)]
const fn baud_reg(f_cpu_hz: u32, baud: u32) -> Option<u16> {
    let denom = 16 * baud as u64;
    // Round to nearest so an inexact divisor does not skew the line rate.
    let reg = (64u64 * f_cpu_hz as u64 + denom / 2) / denom;
    if reg == 0 || reg > u16::MAX as u64 {
        None
    } else {
        Some(reg as u16)
    }
}

/// The requested baud rate cannot be represented for the configured
/// `f_cpu_hz`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BaudUnattainable;

impl core::fmt::Display for BaudUnattainable {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("baud rate unattainable for this clock")
    }
}

impl core::error::Error for BaudUnattainable {}

/// Computes the baud register value, mapping an out-of-range result to
/// [`BaudUnattainable`]. Shared by [`Usart::set_baud`] and the builder.
fn baud_reg_checked(f_cpu_hz: u32, baud: u32) -> Result<u16, BaudUnattainable> {
    baud_reg(f_cpu_hz, baud).ok_or(BaudUnattainable)
}

/// A USART peripheral. Implemented for each device's `USART0`..`USART5`. Not
/// for external use.
pub trait UsartInstance {
    /// Configures the `frame` format at the given baud register value and
    /// enables the transmitter and receiver.
    fn configure(&self, baud: u16, frame: Frame);
    /// Rewrites only the baud register. Used to stretch a BREAK.
    fn set_baud_reg(&self, baud: u16);
    /// Whether the transmit data register can accept a byte.
    fn tx_ready(&self) -> bool;
    /// Whether the last frame has fully left the transmit shift register.
    fn tx_complete(&self) -> bool;
    /// Clears the transmit-complete flag so the next `tx_complete` reflects the
    /// next frame, not this one.
    fn clear_tx_complete(&self);
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
    f_cpu_hz: u32,
    baud_reg: u16,
    rx_budget: u32,
    tx_pending: bool,
}

impl<T: UsartInstance> Usart<T> {
    /// Starts building a USART on `instance`. The returned [`Builder`] requires
    /// an explicit baud rate and [`Frame`] before it can [`build`].
    ///
    /// [`build`]: Builder::build
    #[must_use]
    pub const fn builder(instance: T, f_cpu_hz: u32) -> Builder<T, Unset, Unset> {
        Builder::new(instance, f_cpu_hz)
    }

    /// Releases the underlying peripheral.
    pub fn free(self) -> T {
        self.instance
    }

    /// Changes the baud rate to `baud` bits/s.
    ///
    /// # Errors
    ///
    /// Returns [`BaudUnattainable`] when `baud` cannot be represented for this
    /// `f_cpu_hz`.
    pub fn set_baud(&mut self, baud: u32) -> Result<(), BaudUnattainable> {
        let reg = baud_reg_checked(self.f_cpu_hz, baud)?;
        // Let any in-flight frame drain before changing baud, otherwise the
        // trailing frame is truncated when the baud register changes.
        self.drain_tx();
        self.baud_reg = reg;
        self.instance.set_baud_reg(reg);
        Ok(())
    }

    /// Reconfigures the frame format, keeping the baud rate.
    ///
    /// Drains the transmit shift register first, then rewrites
    /// `CTRLC`/`CTRLB`. The receiver should be idle when this is called. An
    /// in-flight RX frame may be corrupted by the `CTRLC` rewrite. This lets
    /// one USART switch between, for example, an 8N1 command link and an 8E2
    /// UPDI one-wire link on the fly.
    pub fn set_frame(&mut self, frame: Frame) {
        self.drain_tx();
        self.instance.configure(self.baud_reg, frame);
    }

    /// Sends a BREAK: holds the line low well beyond one frame, then restores
    /// the baud rate.
    ///
    /// A UPDI host uses this to reset the target's UPDI state machine. The
    /// break byte is echoed on a one-wire link, so drain the receiver
    /// afterwards.
    pub fn send_break(&mut self) {
        // Let any in-flight frame drain before changing baud, otherwise the
        // trailing frame is truncated when the baud register changes.
        self.drain_tx();
        self.instance.set_baud_reg(BREAK_BAUD_REG);
        self.write_byte(0x00);
        self.drain_tx();
        self.instance.set_baud_reg(self.baud_reg);
    }

    /// Waits for a pending transmission to fully leave the shift register, then
    /// clears the transmit-complete flag. Does nothing when nothing is pending:
    /// TXCIF stays 0 until the first frame completes, so waiting on it before
    /// any transmission would spin until the defensive budget panics.
    fn drain_tx(&mut self) {
        if self.tx_pending {
            crate::wait::spin_until(|| self.instance.tx_complete());
            self.instance.clear_tx_complete();
            self.tx_pending = false;
        }
    }

    /// Blocks until the transmit buffer can accept a byte, then writes it. The
    /// transmitter is host-driven, so this cannot hang unless the peripheral is
    /// broken (in which case it panics rather than spinning forever).
    #[inline]
    pub fn write_byte(&mut self, byte: u8) {
        crate::wait::spin_until(|| self.instance.tx_ready());
        self.instance.push(byte);
        self.tx_pending = true;
    }

    /// Blocks until a byte is received, then returns it.
    ///
    /// # Errors
    ///
    /// Returns [`Error::Timeout`] if no byte arrives within the receive
    /// timeout.
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
        // Drains only when a frame is pending, and clears the sticky TXCIF
        // afterwards so the next flush does not return on a stale flag.
        self.drain_tx();
        Ok(())
    }
}

impl<T: UsartInstance> embedded_io::Read for Usart<T> {
    // The contract wants `read` to block until at least one byte is available,
    // then return only what is ready. A UART never reaches EOF, so instead of
    // returning 0 we block on the first byte (up to the receive timeout) and
    // then drain whatever else is ready. Filling the whole buffer would deadlock
    // a caller waiting on a short final frame.
    #[expect(
        clippy::indexing_slicing,
        reason = "indices are bounded by explicit length checks"
    )]
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
// The `CTRLC` field accessors differ by ATDF vintage: the AVR128 parts and the
// attiny406 flatten them (`chsize`/`pmode`/`sbmode`), while the older attiny416
// models `CTRLC` with register modes (`normal_chsize`/`normal_pmode`/
// `normal_sbmode`). The value setters (`_8bit`, `even`, `_2bit`, ...) are the
// same everywhere.
macro_rules! impl_usart_instance {
    ($USART:ty, $chsize:ident, $pmode:ident, $sbmode:ident) => {
        impl UsartInstance for $USART {
            fn configure(&self, baud: u16, frame: $crate::usart::Frame) {
                self.baud().write(|w| w.set(baud));
                self.ctrlc().write(|w| {
                    w.$chsize()._8bit();
                    match frame.parity {
                        $crate::usart::Parity::None => w.$pmode().disabled(),
                        $crate::usart::Parity::Even => w.$pmode().even(),
                        $crate::usart::Parity::Odd => w.$pmode().odd(),
                    };
                    match frame.stop_bits {
                        $crate::usart::StopBits::One => w.$sbmode()._1bit(),
                        $crate::usart::StopBits::Two => w.$sbmode()._2bit(),
                    }
                });
                self.ctrlb().write(|w| w.txen().set_bit().rxen().set_bit());
            }
            fn set_baud_reg(&self, baud: u16) {
                self.baud().write(|w| w.set(baud));
            }
            fn tx_ready(&self) -> bool {
                self.status().read().dreif().bit_is_set()
            }
            fn tx_complete(&self) -> bool {
                self.status().read().txcif().bit_is_set()
            }
            fn clear_tx_complete(&self) {
                // TXCIF is write-1-to-clear. Writing 0 to the other flags leaves
                // them untouched, so this does not drop a received byte.
                self.status().write(|w| w.txcif().set_bit());
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
// drop). db48 has USART0..4. db64/da64 add USART5.
macro_rules! impl_usarts {
    ($chsize:ident, $pmode:ident, $sbmode:ident; $($USART:ty),+ $(,)?) => {
        $( impl_usart_instance!($USART, $chsize, $pmode, $sbmode); )+
    };
}

#[cfg(feature = "avr128db48")]
impl_usarts!(
    chsize, pmode, sbmode;
    avr_device::avr128db48::USART0,
    avr_device::avr128db48::USART1,
    avr_device::avr128db48::USART2,
    avr_device::avr128db48::USART3,
    avr_device::avr128db48::USART4,
);
#[cfg(feature = "avr128db64")]
impl_usarts!(
    chsize, pmode, sbmode;
    avr_device::avr128db64::USART0,
    avr_device::avr128db64::USART1,
    avr_device::avr128db64::USART2,
    avr_device::avr128db64::USART3,
    avr_device::avr128db64::USART4,
    avr_device::avr128db64::USART5,
);
#[cfg(feature = "avr128da64")]
impl_usarts!(
    chsize, pmode, sbmode;
    avr_device::avr128da64::USART0,
    avr_device::avr128da64::USART1,
    avr_device::avr128da64::USART2,
    avr_device::avr128da64::USART3,
    avr_device::avr128da64::USART4,
    avr_device::avr128da64::USART5,
);
// tinyAVR has a single USART0. The attiny406 ATDF flattens `CTRLC`. The older
// attiny416 ATDF models it with register modes.
#[cfg(feature = "attiny406")]
impl_usarts!(chsize, pmode, sbmode; avr_device::attiny406::USART0);
#[cfg(feature = "attiny416")]
impl_usarts!(normal_chsize, normal_pmode, normal_sbmode; avr_device::attiny416::USART0);
