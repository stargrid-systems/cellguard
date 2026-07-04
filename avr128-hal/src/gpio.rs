//! GPIO via the PORT peripheral.
//!
//! Claim a PORT once with [`Port::new`], then [`Port::split`] it into its eight
//! [`Pin`]s. Each pin carries its bit index in the type, and `split` hands out
//! every pin exactly once. Turn a pin into an [`Output`] or [`Input`] with the
//! `into_*` methods. Those consume the pin, so one bit can never be configured
//! as two conflicting things. No `steal` in application code, and pin-level
//! aliasing is impossible in safe code.
//!
//! The direction/output strobe registers (`DIRSET`/`DIRCLR`/`OUTSET`/`OUTCLR`/
//! `OUTTGL`) are write-1-to-act and touch only the masked bit. So independent
//! pins on the same port never race.

use core::marker::PhantomData;

use embedded_hal::digital::{ErrorType, InputPin, OutputPin, StatefulOutputPin};

/// Atomic bit operations common to every PORT instance. Not for external use.
pub trait PortOps {
    fn set_dir_output(&self, mask: u8);
    fn set_dir_input(&self, mask: u8);
    fn set_high(&self, mask: u8);
    fn set_low(&self, mask: u8);
    fn toggle(&self, mask: u8);
    fn read_input(&self) -> u8;
    fn read_output(&self) -> u8;
    /// Enables or disables the internal pull-up on one pin (`bit` = 0..=7).
    fn set_pullup(&self, bit: u8, on: bool);
}

/// A PORT peripheral whose pins can re-acquire it internally. Implemented for
/// each device's `PORTA`..`PORTG`. Not for external use.
pub trait PortInstance: PortOps + Sized {
    /// Re-acquires the port handle.
    ///
    /// # Safety
    /// The caller must guarantee unique access to this port's pins.
    unsafe fn steal() -> Self;
}

// The DA PAC models DIRSET/DIRCLR/OUTSET/OUTCLR/OUTTGL as safe whole-register
// writes, while the DB PAC gives them per-pin fields and marks the raw write
// unsafe. `w.bits(mask)` is the one writer common to both. It is sound here:
// these are write-1-to-act strobe registers, so every u8 mask is valid (see the
// module docs).
macro_rules! impl_port {
    ($PORT:ty) => {
        impl PortOps for $PORT {
            #[inline]
            fn set_dir_output(&self, mask: u8) {
                // SAFETY: write-1-to-act strobe register; any mask is valid.
                self.dirset().write(|w| unsafe { w.bits(mask) });
            }
            #[inline]
            fn set_dir_input(&self, mask: u8) {
                // SAFETY: as above.
                self.dirclr().write(|w| unsafe { w.bits(mask) });
            }
            #[inline]
            fn set_high(&self, mask: u8) {
                // SAFETY: as above.
                self.outset().write(|w| unsafe { w.bits(mask) });
            }
            #[inline]
            fn set_low(&self, mask: u8) {
                // SAFETY: as above.
                self.outclr().write(|w| unsafe { w.bits(mask) });
            }
            #[inline]
            fn toggle(&self, mask: u8) {
                // SAFETY: as above.
                self.outtgl().write(|w| unsafe { w.bits(mask) });
            }
            #[inline]
            fn read_input(&self) -> u8 {
                self.in_().read().bits()
            }
            #[inline]
            fn read_output(&self) -> u8 {
                self.out().read().bits()
            }
            #[inline]
            fn set_pullup(&self, bit: u8, on: bool) {
                match bit {
                    0 => self.pin0ctrl().modify(|_, w| w.pullupen().bit(on)),
                    1 => self.pin1ctrl().modify(|_, w| w.pullupen().bit(on)),
                    2 => self.pin2ctrl().modify(|_, w| w.pullupen().bit(on)),
                    3 => self.pin3ctrl().modify(|_, w| w.pullupen().bit(on)),
                    4 => self.pin4ctrl().modify(|_, w| w.pullupen().bit(on)),
                    5 => self.pin5ctrl().modify(|_, w| w.pullupen().bit(on)),
                    6 => self.pin6ctrl().modify(|_, w| w.pullupen().bit(on)),
                    _ => self.pin7ctrl().modify(|_, w| w.pullupen().bit(on)),
                };
            }
        }
        impl PortInstance for $PORT {
            #[inline]
            unsafe fn steal() -> Self {
                // SAFETY: forwarded to the caller of this trait method.
                unsafe { <$PORT>::steal() }
            }
        }
    };
}

// One call per device (grouped, so instances never interleave and are hard to
// drop). db48 has PORTA..PORTF; db64/da64 add PORTG.
macro_rules! impl_ports {
    ($($PORT:ty),+ $(,)?) => {
        $( impl_port!($PORT); )+
    };
}

#[cfg(feature = "avr128db48")]
impl_ports!(
    avr_device::avr128db48::PORTA,
    avr_device::avr128db48::PORTB,
    avr_device::avr128db48::PORTC,
    avr_device::avr128db48::PORTD,
    avr_device::avr128db48::PORTE,
    avr_device::avr128db48::PORTF,
);
#[cfg(feature = "avr128db64")]
impl_ports!(
    avr_device::avr128db64::PORTA,
    avr_device::avr128db64::PORTB,
    avr_device::avr128db64::PORTC,
    avr_device::avr128db64::PORTD,
    avr_device::avr128db64::PORTE,
    avr_device::avr128db64::PORTF,
    avr_device::avr128db64::PORTG,
);
#[cfg(feature = "avr128da64")]
impl_ports!(
    avr_device::avr128da64::PORTA,
    avr_device::avr128da64::PORTB,
    avr_device::avr128da64::PORTC,
    avr_device::avr128da64::PORTD,
    avr_device::avr128da64::PORTE,
    avr_device::avr128da64::PORTF,
    avr_device::avr128da64::PORTG,
);

/// Re-acquires a port handle for a uniquely-owned pin.
#[inline]
fn port<P: PortInstance>() -> P {
    // SAFETY: `Port::split` consumed the one real PORT and hands out each
    // `Pin<P, BIT>` exactly once. Every `Pin`/`Output`/`Input` owns a distinct
    // bit and the strobe registers are write-1-to-act, so this handle never
    // races another pin.
    unsafe { P::steal() }
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

    /// Splits the port into its eight pins.
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

/// A single, uniquely-owned pin of a port, not yet configured.
pub struct Pin<P: PortInstance, const BIT: u8> {
    _port: PhantomData<P>,
}

impl<P: PortInstance, const BIT: u8> Pin<P, BIT> {
    const MASK: u8 = 1 << BIT;

    fn new() -> Self {
        Self { _port: PhantomData }
    }

    /// Configures the pin as a low output.
    #[must_use]
    pub fn into_output(self) -> Output<P, BIT> {
        let p = port::<P>();
        p.set_low(Self::MASK);
        p.set_dir_output(Self::MASK);
        Output { _port: PhantomData }
    }

    /// Configures the pin as a high output.
    #[must_use]
    pub fn into_output_high(self) -> Output<P, BIT> {
        let p = port::<P>();
        p.set_high(Self::MASK);
        p.set_dir_output(Self::MASK);
        Output { _port: PhantomData }
    }

    /// Configures the pin as a floating input.
    #[must_use]
    pub fn into_input(self) -> Input<P, BIT> {
        let p = port::<P>();
        p.set_pullup(BIT, false);
        p.set_dir_input(Self::MASK);
        Input { _port: PhantomData }
    }

    /// Configures the pin as an input with the internal pull-up enabled.
    #[must_use]
    pub fn into_input_pullup(self) -> Input<P, BIT> {
        let p = port::<P>();
        p.set_pullup(BIT, true);
        p.set_dir_input(Self::MASK);
        Input { _port: PhantomData }
    }
}

/// A push-pull output pin (one bit of a port).
pub struct Output<P: PortInstance, const BIT: u8> {
    _port: PhantomData<P>,
}

impl<P: PortInstance, const BIT: u8> Output<P, BIT> {
    const MASK: u8 = 1 << BIT;

    /// Releases the pin back to its unconfigured state.
    #[must_use]
    pub fn into_pin(self) -> Pin<P, BIT> {
        Pin::new()
    }
}

impl<P: PortInstance, const BIT: u8> ErrorType for Output<P, BIT> {
    type Error = PinError;
}

impl<P: PortInstance, const BIT: u8> OutputPin for Output<P, BIT> {
    fn set_high(&mut self) -> Result<(), Self::Error> {
        port::<P>().set_high(Self::MASK);
        Ok(())
    }
    fn set_low(&mut self) -> Result<(), Self::Error> {
        port::<P>().set_low(Self::MASK);
        Ok(())
    }
}

impl<P: PortInstance, const BIT: u8> StatefulOutputPin for Output<P, BIT> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(port::<P>().read_output() & Self::MASK != 0)
    }
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(port::<P>().read_output() & Self::MASK == 0)
    }
    fn toggle(&mut self) -> Result<(), Self::Error> {
        port::<P>().toggle(Self::MASK);
        Ok(())
    }
}

/// A digital input pin (one bit of a port).
pub struct Input<P: PortInstance, const BIT: u8> {
    _port: PhantomData<P>,
}

impl<P: PortInstance, const BIT: u8> Input<P, BIT> {
    const MASK: u8 = 1 << BIT;

    /// Releases the pin back to its unconfigured state.
    #[must_use]
    pub fn into_pin(self) -> Pin<P, BIT> {
        Pin::new()
    }
}

impl<P: PortInstance, const BIT: u8> ErrorType for Input<P, BIT> {
    type Error = PinError;
}

impl<P: PortInstance, const BIT: u8> InputPin for Input<P, BIT> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(port::<P>().read_input() & Self::MASK != 0)
    }
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(port::<P>().read_input() & Self::MASK == 0)
    }
}
