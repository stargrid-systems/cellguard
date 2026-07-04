//! I2C host on the TWI peripheral, implementing [`embedded_hal::i2c::I2c`].
//!
//! [`Twi`] is generic over a [`TwiInstance`]. Construct it with the PAC
//! peripheral, e.g. `Twi::new(dp.TWI1, f_cpu, 100_000)`. Pin routing
//! (`PORTMUX`) and bus pull-ups are the application's responsibility.

use embedded_hal::i2c::{self, I2c, NoAcknowledgeSource, Operation};

/// Default transaction timeout, in milliseconds. A slave can stretch the clock
/// forever, so each wait gives up after this long. Override with
/// [`Twi::with_timeout_ms`].
const DEFAULT_TIMEOUT_MS: u32 = 100;

/// I2C bus error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// Illegal bus condition.
    Bus,
    /// Lost arbitration to another host.
    ArbitrationLoss,
    /// Address or data byte was not acknowledged.
    Nack,
    /// A wait exceeded the transaction timeout (for example a slave holding SCL
    /// low).
    Timeout,
}

impl i2c::Error for Error {
    fn kind(&self) -> i2c::ErrorKind {
        match self {
            Self::Bus => i2c::ErrorKind::Bus,
            Self::ArbitrationLoss => i2c::ErrorKind::ArbitrationLoss,
            Self::Nack => i2c::ErrorKind::NoAcknowledge(NoAcknowledgeSource::Unknown),
            Self::Timeout => i2c::ErrorKind::Other,
        }
    }
}

/// Snapshot of the host status flags relevant to a transfer.
#[derive(Clone, Copy)]
pub struct HostStatus {
    pub arbitration_lost: bool,
    pub bus_error: bool,
    pub write_done: bool,
    pub read_done: bool,
    pub nacked: bool,
    pub bus_idle: bool,
}

/// A TWI peripheral usable as an I2C host. Implemented for each device's
/// `TWI0`/`TWI1`. Not for external use.
pub trait TwiInstance {
    /// Sets the baud divider, enables the host, and forces the bus to idle.
    fn configure(&self, baud: u8);
    /// Reads the host status flags.
    fn host_status(&self) -> HostStatus;
    /// Issues (repeated) START with the addressed byte (`addr<<1 | r/w`).
    fn start_with_address(&self, byte: u8);
    /// Writes one data byte to the bus.
    fn write_byte(&self, byte: u8);
    /// Reads the last received data byte.
    fn read_byte(&self) -> u8;
    /// ACKs the received byte and clocks in the next one.
    fn ack_and_receive(&self);
    /// Arms a NACK for the received byte (sent with the following STOP).
    fn prepare_nack(&self);
    /// Issues a STOP condition.
    fn stop(&self);
}

/// I2C host built on a [`TwiInstance`].
pub struct Twi<T: TwiInstance> {
    instance: T,
    max_wait: u32,
}

impl<T: TwiInstance> Twi<T> {
    /// Enables the TWI host at the given SCL frequency, with the default
    /// transaction timeout (100 ms).
    ///
    /// `MBAUD = f_CLK_PER / (2 * f_SCL) - 5` (bus rise time neglected).
    /// `configure` writes `MBAUD`/`MCTRLA`/`MSTATUS` whole.
    #[must_use]
    pub fn new(instance: T, f_cpu_hz: u32, scl_hz: u32) -> Self {
        Self::with_timeout_ms(instance, f_cpu_hz, scl_hz, DEFAULT_TIMEOUT_MS)
    }

    /// Like [`Twi::new`], but with a caller-chosen per-wait timeout in
    /// milliseconds (approximate, derived from `f_cpu_hz`).
    #[must_use]
    pub fn with_timeout_ms(instance: T, f_cpu_hz: u32, scl_hz: u32, timeout_ms: u32) -> Self {
        let baud = (f_cpu_hz / (2 * scl_hz)).saturating_sub(5) as u8;
        instance.configure(baud);
        Self {
            instance,
            max_wait: crate::wait::budget_ms(f_cpu_hz, timeout_ms),
        }
    }

    /// Releases the underlying peripheral.
    pub fn free(self) -> T {
        self.instance
    }

    fn wait_write(&self) -> Result<(), Error> {
        for _ in 0..self.max_wait {
            let s = self.instance.host_status();
            if s.arbitration_lost {
                return Err(Error::ArbitrationLoss);
            }
            if s.bus_error {
                return Err(Error::Bus);
            }
            if s.write_done {
                return if s.nacked { Err(Error::Nack) } else { Ok(()) };
            }
        }
        Err(Error::Timeout)
    }

    fn wait_read(&self) -> Result<(), Error> {
        for _ in 0..self.max_wait {
            let s = self.instance.host_status();
            if s.arbitration_lost {
                return Err(Error::ArbitrationLoss);
            }
            if s.bus_error {
                return Err(Error::Bus);
            }
            if s.read_done {
                return Ok(());
            }
        }
        Err(Error::Timeout)
    }

    /// Waits for a read address phase to settle. On success the host has
    /// received the first data byte and `RIF` is set. If the address is not
    /// acknowledged the host raises `WIF` instead, with no data received.
    fn wait_read_address(&self) -> Result<(), Error> {
        for _ in 0..self.max_wait {
            let s = self.instance.host_status();
            if s.arbitration_lost {
                return Err(Error::ArbitrationLoss);
            }
            if s.bus_error {
                return Err(Error::Bus);
            }
            if s.read_done {
                return Ok(());
            }
            if s.write_done {
                return Err(Error::Nack);
            }
        }
        Err(Error::Timeout)
    }

    fn send_address(&mut self, address: u8, read: bool) -> Result<(), Error> {
        self.instance
            .start_with_address((address << 1) | u8::from(read));
        if read {
            self.wait_read_address()
        } else {
            self.wait_write()
        }
    }

    fn run_transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Error> {
        let op_count = operations.len();
        let mut prev_read: Option<bool> = None;

        for (i, op) in operations.iter_mut().enumerate() {
            let is_read = matches!(op, Operation::Read(_));
            // Address on the first op and on every direction change.
            if prev_read != Some(is_read) {
                self.send_address(address, is_read)?;
            }
            prev_read = Some(is_read);

            match op {
                Operation::Write(buf) => {
                    for &b in buf.iter() {
                        self.instance.write_byte(b);
                        self.wait_write()?;
                    }
                }
                Operation::Read(buf) => {
                    let last_op = i + 1 == op_count;
                    let len = buf.len();
                    for (j, slot) in buf.iter_mut().enumerate() {
                        self.wait_read()?;
                        *slot = self.instance.read_byte();
                        if j + 1 == len && last_op {
                            // NACK the final byte; STOP is issued by the caller.
                            self.instance.prepare_nack();
                        } else {
                            self.instance.ack_and_receive();
                        }
                    }
                }
            }
        }
        Ok(())
    }
}

impl<T: TwiInstance> i2c::ErrorType for Twi<T> {
    type Error = Error;
}

impl<T: TwiInstance> I2c for Twi<T> {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        // Always issue a STOP, even when an operation fails. A NACKed address
        // otherwise leaves the host owning the bus and hangs the next
        // transaction.
        let result = self.run_transaction(address, operations);
        self.instance.stop();
        result
    }
}

// Hidden implementation detail: the body is identical for both instances but
// each references its own (distinct) PAC register block, so it cannot be made
// generic. This private macro only produces trait impls, never user-facing
// types.
macro_rules! impl_twi_instance {
    ($TWI:ty) => {
        impl TwiInstance for $TWI {
            fn configure(&self, baud: u8) {
                self.mbaud().write(|w| w.baud().set(baud));
                self.mctrla().write(|w| w.enable().set_bit());
                self.mstatus().write(|w| w.busstate().idle());
            }
            fn host_status(&self) -> HostStatus {
                let s = self.mstatus().read();
                HostStatus {
                    arbitration_lost: s.arblost().bit_is_set(),
                    bus_error: s.buserr().bit_is_set(),
                    write_done: s.wif().bit_is_set(),
                    read_done: s.rif().bit_is_set(),
                    nacked: s.rxack().bit_is_set(),
                    // BUSSTATE: 0=unknown, 1=idle, 2=owner, 3=busy.
                    bus_idle: s.busstate().bits() == 1,
                }
            }
            fn start_with_address(&self, byte: u8) {
                self.maddr().write(|w| w.addr().set(byte));
            }
            fn write_byte(&self, byte: u8) {
                self.mdata().write(|w| w.data().set(byte));
            }
            fn read_byte(&self) -> u8 {
                self.mdata().read().data().bits()
            }
            fn ack_and_receive(&self) {
                self.mctrlb().write(|w| w.ackact().ack().mcmd().recvtrans());
            }
            fn prepare_nack(&self) {
                self.mctrlb().write(|w| w.ackact().nack());
            }
            fn stop(&self) {
                self.mctrlb().write(|w| w.mcmd().stop());
            }
        }
    };
}

// One call per device (grouped, so instances never interleave). All three have
// TWI0 and TWI1.
macro_rules! impl_twis {
    ($($TWI:ty),+ $(,)?) => {
        $( impl_twi_instance!($TWI); )+
    };
}

#[cfg(feature = "avr128db48")]
impl_twis!(avr_device::avr128db48::TWI0, avr_device::avr128db48::TWI1);
#[cfg(feature = "avr128db64")]
impl_twis!(avr_device::avr128db64::TWI0, avr_device::avr128db64::TWI1);
#[cfg(feature = "avr128da64")]
impl_twis!(avr_device::avr128da64::TWI0, avr_device::avr128da64::TWI1);
