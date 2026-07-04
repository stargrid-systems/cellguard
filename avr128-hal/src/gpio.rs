//! GPIO via the PORT peripheral.
//!
//! All seven ports (PORTA..PORTG) share an identical register layout, captured
//! by the [`PortOps`] trait. [`Output`] and [`Input`] wrap a port handle plus a
//! pin mask and implement the `embedded-hal` digital traits.
//!
//! The direction/output strobe registers (`DIRSET`/`DIRCLR`/`OUTSET`/`OUTCLR`/
//! `OUTTGL`) are write-1-to-act and touch only the masked bits. So independent
//! pins on the same port never race. It is sound to hand each pin its own
//! `PORTx::steal()` handle.

use embedded_hal::digital::{ErrorType, InputPin, OutputPin, StatefulOutputPin};

/// Atomic bit operations common to every PORT instance.
pub trait PortOps {
    fn set_dir_output(&self, mask: u8);
    fn set_dir_input(&self, mask: u8);
    fn set_high(&self, mask: u8);
    fn set_low(&self, mask: u8);
    fn toggle(&self, mask: u8);
    fn read_input(&self) -> u8;
    fn read_output(&self) -> u8;
}

macro_rules! impl_port_ops {
    ($($PORT:ty),+ $(,)?) => {$(
        impl PortOps for $PORT {
            #[inline]
            fn set_dir_output(&self, mask: u8) { self.dirset().write(|w| w.set(mask)); }
            #[inline]
            fn set_dir_input(&self, mask: u8) { self.dirclr().write(|w| w.set(mask)); }
            #[inline]
            fn set_high(&self, mask: u8) { self.outset().write(|w| w.set(mask)); }
            #[inline]
            fn set_low(&self, mask: u8) { self.outclr().write(|w| w.set(mask)); }
            #[inline]
            fn toggle(&self, mask: u8) { self.outtgl().write(|w| w.set(mask)); }
            #[inline]
            fn read_input(&self) -> u8 { self.in_().read().bits() }
            #[inline]
            fn read_output(&self) -> u8 { self.out().read().bits() }
        }
    )+};
}

#[cfg(feature = "avr128db28")]
impl_port_ops!(
    avr_device::avr128db28::PORTA,
    avr_device::avr128db28::PORTC,
    avr_device::avr128db28::PORTD,
    avr_device::avr128db28::PORTF,
);
#[cfg(feature = "avr128db48")]
impl_port_ops!(
    avr_device::avr128db48::PORTA,
    avr_device::avr128db48::PORTB,
    avr_device::avr128db48::PORTC,
    avr_device::avr128db48::PORTD,
    avr_device::avr128db48::PORTE,
    avr_device::avr128db48::PORTF,
);
#[cfg(feature = "avr128db64")]
impl_port_ops!(
    avr_device::avr128db64::PORTA,
    avr_device::avr128db64::PORTB,
    avr_device::avr128db64::PORTC,
    avr_device::avr128db64::PORTD,
    avr_device::avr128db64::PORTE,
    avr_device::avr128db64::PORTF,
    avr_device::avr128db64::PORTG,
);
#[cfg(feature = "avr128da64")]
impl_port_ops!(
    avr_device::avr128da64::PORTA,
    avr_device::avr128da64::PORTB,
    avr_device::avr128da64::PORTC,
    avr_device::avr128da64::PORTD,
    avr_device::avr128da64::PORTE,
    avr_device::avr128da64::PORTF,
    avr_device::avr128da64::PORTG,
);

/// GPIO pin errors. Digital pin operations on this MCU cannot fail.
#[derive(Debug, Clone, Copy)]
pub enum PinError {}

impl embedded_hal::digital::Error for PinError {
    fn kind(&self) -> embedded_hal::digital::ErrorKind {
        match *self {}
    }
}

/// A push-pull output pin (single bit of a port).
pub struct Output<P: PortOps> {
    port: P,
    mask: u8,
}

impl<P: PortOps> Output<P> {
    /// Configures `bit` of `port` as a low output.
    #[must_use]
    pub fn new(port: P, bit: u8) -> Self {
        let mask = 1 << bit;
        port.set_low(mask);
        port.set_dir_output(mask);
        Self { port, mask }
    }

    /// Configures `bit` of `port` as a high output.
    #[must_use]
    pub fn new_high(port: P, bit: u8) -> Self {
        let mask = 1 << bit;
        port.set_high(mask);
        port.set_dir_output(mask);
        Self { port, mask }
    }
}

impl<P: PortOps> ErrorType for Output<P> {
    type Error = PinError;
}

impl<P: PortOps> OutputPin for Output<P> {
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.port.set_high(self.mask);
        Ok(())
    }
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.port.set_low(self.mask);
        Ok(())
    }
}

impl<P: PortOps> StatefulOutputPin for Output<P> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.port.read_output() & self.mask != 0)
    }
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.port.read_output() & self.mask == 0)
    }
    fn toggle(&mut self) -> Result<(), Self::Error> {
        self.port.toggle(self.mask);
        Ok(())
    }
}

/// A digital input pin (single bit of a port).
pub struct Input<P: PortOps> {
    port: P,
    mask: u8,
}

impl<P: PortOps> Input<P> {
    /// Configures `bit` of `port` as an input.
    #[must_use]
    pub fn new(port: P, bit: u8) -> Self {
        let mask = 1 << bit;
        port.set_dir_input(mask);
        Self { port, mask }
    }
}

impl<P: PortOps> ErrorType for Input<P> {
    type Error = PinError;
}

impl<P: PortOps> InputPin for Input<P> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.port.read_input() & self.mask != 0)
    }
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.port.read_input() & self.mask == 0)
    }
}
