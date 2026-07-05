//! GPIO via the PORT peripheral.
//!
//! Claim a PORT once with [`Port::new`], then [`Port::split`] it into its eight
//! [`Pin`]s. Each pin carries its bit index in the type, and `split` hands out
//! every pin exactly once, so one bit can never be configured as two
//! conflicting things and pin-level aliasing is impossible in safe code.
//!
//! Configuring a pin with an `into_*` method consumes it and yields a
//! type-erased [`Output`] or [`Input`] that stores the PORT base address and
//! the bit mask at runtime. That gives a single implementation of the pin
//! operations for every port and bit, instead of one monomorphized copy per
//! pin, which matters on the flash-constrained tinyAVR parts.
//!
//! The direction/output strobe registers (`DIRSET`/`DIRCLR`/`OUTSET`/`OUTCLR`/
//! `OUTTGL`) are write-1-to-act and touch only the masked bit, so independent
//! pins on the same port never race. Every PORT register block has the same
//! layout across the modern-AVR family, so the type-erased access is portable.

use core::marker::PhantomData;
use core::ptr;

use embedded_hal::digital::{ErrorType, InputPin, OutputPin, StatefulOutputPin};

// PORT register offsets, identical across the modern-AVR family.
const DIRSET: usize = 0x01;
const DIRCLR: usize = 0x02;
const OUT: usize = 0x04;
const OUTSET: usize = 0x05;
const OUTCLR: usize = 0x06;
const OUTTGL: usize = 0x07;
const IN: usize = 0x08;
const PIN0CTRL: usize = 0x10;
const PULLUPEN: u8 = 1 << 3;

/// A PORT peripheral. Implemented for each device's `PORTA`..`PORTG`. Exposes
/// the register-block base so a configured pin can drive it without carrying
/// the port type. Not for external use.
pub trait PortInstance {
    /// Base address of the PORT register block.
    const REGS: *mut u8;
    /// Bitmask of the pins that physically exist on this port. Bit `n` set
    /// means pin `n` exists. Narrow tinyAVR ports do not populate all
    /// eight.
    const PIN_MASK: u8;
}

// Every PORT peripheral exposes a `ptr()` to its register block. All blocks
// share the same layout, so the base address is all a pin needs.
macro_rules! impl_port_instance {
    ($($PORT:ty => $mask:expr),+ $(,)?) => {
        $(
            impl PortInstance for $PORT {
                const REGS: *mut u8 = <$PORT>::ptr() as *mut u8;
                const PIN_MASK: u8 = $mask;
            }
        )+
    };
}

#[cfg(feature = "avr128db48")]
impl_port_instance!(
    avr_device::avr128db48::PORTA => 0xff,
    avr_device::avr128db48::PORTB => 0xff,
    avr_device::avr128db48::PORTC => 0xff,
    avr_device::avr128db48::PORTD => 0xff,
    avr_device::avr128db48::PORTE => 0xff,
    avr_device::avr128db48::PORTF => 0xff,
);
#[cfg(feature = "avr128db64")]
impl_port_instance!(
    avr_device::avr128db64::PORTA => 0xff,
    avr_device::avr128db64::PORTB => 0xff,
    avr_device::avr128db64::PORTC => 0xff,
    avr_device::avr128db64::PORTD => 0xff,
    avr_device::avr128db64::PORTE => 0xff,
    avr_device::avr128db64::PORTF => 0xff,
    avr_device::avr128db64::PORTG => 0xff,
);
#[cfg(feature = "avr128da64")]
impl_port_instance!(
    avr_device::avr128da64::PORTA => 0xff,
    avr_device::avr128da64::PORTB => 0xff,
    avr_device::avr128da64::PORTC => 0xff,
    avr_device::avr128da64::PORTD => 0xff,
    avr_device::avr128da64::PORTE => 0xff,
    avr_device::avr128da64::PORTF => 0xff,
    avr_device::avr128da64::PORTG => 0xff,
);
// 20-pin tinyAVR: PORTA (PA0-7), PORTB (PB0-5), PORTC (PC0-3).
#[cfg(feature = "attiny406")]
impl_port_instance!(
    avr_device::attiny406::PORTA => 0xff,
    avr_device::attiny406::PORTB => 0x3f,
    avr_device::attiny406::PORTC => 0x0f,
);
#[cfg(feature = "attiny416")]
impl_port_instance!(
    avr_device::attiny416::PORTA => 0xff,
    avr_device::attiny416::PORTB => 0x3f,
    avr_device::attiny416::PORTC => 0x0f,
);

/// Writes a mask to a write-1-to-act strobe register.
///
/// # Safety
/// `regs` must be a valid PORT base and `off` a strobe-register offset. The
/// caller must own the bits set in `mask`.
#[inline]
unsafe fn strobe(regs: *mut u8, off: usize, mask: u8) {
    // SAFETY: `off` is within the PORT block and the register is write-1-to-act,
    // so only the masked bits change. Preconditions are the caller's.
    unsafe { ptr::write_volatile(regs.add(off), mask) };
}

/// Reads a PORT register.
///
/// # Safety
/// `regs` must be a valid PORT base and `off` a readable-register offset.
#[inline]
unsafe fn read_reg(regs: *mut u8, off: usize) -> u8 {
    // SAFETY: `off` is within the PORT block. Preconditions are the caller's.
    unsafe { ptr::read_volatile(regs.add(off)) }
}

/// Sets or clears the pull-up in one pin's `PINnCTRL`.
///
/// # Safety
/// `regs` must be a valid PORT base and the caller must own bit `bit`.
#[inline]
unsafe fn set_pullup(regs: *mut u8, bit: u8, on: bool) {
    let reg = regs.wrapping_add(PIN0CTRL + bit as usize);
    // SAFETY: `PINnCTRL` is inside the PORT block. The caller owns this pin, so
    // the read-modify-write races nothing.
    unsafe {
        let mut v = ptr::read_volatile(reg);
        v = if on { v | PULLUPEN } else { v & !PULLUPEN };
        ptr::write_volatile(reg, v);
    }
}

/// GPIO pin errors. Digital pin operations on this MCU cannot fail.
#[derive(Debug, Clone, Copy)]
pub enum PinError {}

impl embedded_hal::digital::Error for PinError {
    fn kind(&self) -> embedded_hal::digital::ErrorKind {
        match *self {}
    }
}

/// A claimed PORT peripheral, ready to be split into pins.
pub struct Port<P: PortInstance> {
    _port: P,
}

impl<P: PortInstance> Port<P> {
    /// Claims a PORT peripheral.
    #[must_use]
    pub fn new(port: P) -> Self {
        Self { _port: port }
    }

    /// Splits the port into its eight pins. On narrow tinyAVR ports the pins
    /// that do not physically exist are still handed out, but configuring
    /// one is a compile error (see [`PortInstance::PIN_MASK`]).
    #[must_use]
    pub fn split(self) -> Pins<P> {
        Pins {
            p0: Pin::new(),
            p1: Pin::new(),
            p2: Pin::new(),
            p3: Pin::new(),
            p4: Pin::new(),
            p5: Pin::new(),
            p6: Pin::new(),
            p7: Pin::new(),
        }
    }
}

/// The eight pins of a port. Each field is produced exactly once.
pub struct Pins<P: PortInstance> {
    pub p0: Pin<P, 0>,
    pub p1: Pin<P, 1>,
    pub p2: Pin<P, 2>,
    pub p3: Pin<P, 3>,
    pub p4: Pin<P, 4>,
    pub p5: Pin<P, 5>,
    pub p6: Pin<P, 6>,
    pub p7: Pin<P, 7>,
}

/// A single, uniquely-owned pin of a port, not yet configured. The port and bit
/// live in the type so `split` can guarantee each pin is handed out once.
pub struct Pin<P: PortInstance, const BIT: u8> {
    _port: PhantomData<P>,
}

impl<P: PortInstance, const BIT: u8> Pin<P, BIT> {
    const MASK: u8 = 1 << BIT;

    // Compile-time guard: fails the build if this pin does not exist on the port
    // (a narrow tinyAVR port). Referenced by every `into_*` so unusable pins from
    // `split` cannot be configured, instead of silently strobing dead bits.
    const EXISTS: () = assert!(
        P::PIN_MASK & (1u8 << BIT) != 0,
        "pin does not exist on this port"
    );

    fn new() -> Self {
        Self { _port: PhantomData }
    }

    /// Configures the pin as a low output.
    #[must_use]
    pub fn into_output(self) -> Output {
        let () = Self::EXISTS;
        // SAFETY: this pin uniquely owns `BIT` of `P`, so its strobes race nothing.
        unsafe {
            strobe(P::REGS, OUTCLR, Self::MASK);
            strobe(P::REGS, DIRSET, Self::MASK);
        }
        Output {
            regs: P::REGS,
            mask: Self::MASK,
        }
    }

    /// Configures the pin as a high output.
    #[must_use]
    pub fn into_output_high(self) -> Output {
        let () = Self::EXISTS;
        // SAFETY: as in `into_output`.
        unsafe {
            strobe(P::REGS, OUTSET, Self::MASK);
            strobe(P::REGS, DIRSET, Self::MASK);
        }
        Output {
            regs: P::REGS,
            mask: Self::MASK,
        }
    }

    /// Configures the pin as a floating input.
    #[must_use]
    pub fn into_input(self) -> Input {
        let () = Self::EXISTS;
        // SAFETY: as in `into_output`.
        unsafe {
            set_pullup(P::REGS, BIT, false);
            strobe(P::REGS, DIRCLR, Self::MASK);
        }
        Input {
            regs: P::REGS,
            mask: Self::MASK,
        }
    }

    /// Configures the pin as an input with the internal pull-up enabled.
    #[must_use]
    pub fn into_input_pullup(self) -> Input {
        let () = Self::EXISTS;
        // SAFETY: as in `into_output`.
        unsafe {
            set_pullup(P::REGS, BIT, true);
            strobe(P::REGS, DIRCLR, Self::MASK);
        }
        Input {
            regs: P::REGS,
            mask: Self::MASK,
        }
    }
}

/// A push-pull output pin. Type-erased: holds its PORT base and bit mask, so
/// one implementation serves every port and bit.
pub struct Output {
    regs: *mut u8,
    mask: u8,
}

impl ErrorType for Output {
    type Error = PinError;
}

impl OutputPin for Output {
    fn set_high(&mut self) -> Result<(), Self::Error> {
        // SAFETY: `regs`/`mask` came from a uniquely-owned pin.
        unsafe { strobe(self.regs, OUTSET, self.mask) };
        Ok(())
    }
    fn set_low(&mut self) -> Result<(), Self::Error> {
        // SAFETY: as above.
        unsafe { strobe(self.regs, OUTCLR, self.mask) };
        Ok(())
    }
}

impl StatefulOutputPin for Output {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        // SAFETY: as above.
        Ok(unsafe { read_reg(self.regs, OUT) } & self.mask != 0)
    }
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        // SAFETY: as above.
        Ok(unsafe { read_reg(self.regs, OUT) } & self.mask == 0)
    }
    fn toggle(&mut self) -> Result<(), Self::Error> {
        // SAFETY: as above.
        unsafe { strobe(self.regs, OUTTGL, self.mask) };
        Ok(())
    }
}

/// A digital input pin. Type-erased like [`Output`].
pub struct Input {
    regs: *mut u8,
    mask: u8,
}

impl ErrorType for Input {
    type Error = PinError;
}

impl InputPin for Input {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        // SAFETY: `regs`/`mask` came from a uniquely-owned pin.
        Ok(unsafe { read_reg(self.regs, IN) } & self.mask != 0)
    }
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        // SAFETY: as above.
        Ok(unsafe { read_reg(self.regs, IN) } & self.mask == 0)
    }
}
