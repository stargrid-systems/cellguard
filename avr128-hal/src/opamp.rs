//! On-chip operational amplifiers (OPAMP).
//!
//! [`Opamp`] is generic over an [`OpampInstance`]. The DB family has three
//! op-amps (OP0/OP1/OP2). This module enables the OPAMP system and can set up
//! an op-amp as a unity-gain voltage follower. The positive input comes from
//! the `INP` pin and the negative input is tied to the output. The CellGuard
//! board uses one as a buffer on the analog path.

/// Which op-amp to configure.
#[derive(Clone, Copy)]
pub enum OpAmp {
    Op0,
    Op1,
    Op2,
}

impl OpAmp {
    const fn index(self) -> u8 {
        match self {
            Self::Op0 => 0,
            Self::Op1 => 1,
            Self::Op2 => 2,
        }
    }
}

/// An OPAMP peripheral. Implemented for each device's `OPAMP`. Not for external use.
pub trait OpampInstance {
    /// Enables the OPAMP system with `TIMEBASE` set to `timebase` cycles
    /// (the number of `CLK_PER` cycles in 1 us).
    fn enable(&self, timebase: u8);
    /// Configures op-amp `op` (0..=2) as a unity-gain voltage follower.
    fn set_follower(&self, op: u8);
}

/// The OPAMP peripheral.
pub struct Opamp<T: OpampInstance> {
    instance: T,
}

impl<T: OpampInstance> Opamp<T> {
    /// Enables the OPAMP system. `f_cpu_mhz` sets `TIMEBASE` to 1 us worth of
    /// `CLK_PER` cycles (datasheet requirement).
    #[must_use]
    pub fn new(instance: T, f_cpu_mhz: u8) -> Self {
        instance.enable(f_cpu_mhz.saturating_sub(1));
        Self { instance }
    }

    /// Configures an op-amp as a unity-gain voltage follower.
    pub fn configure_follower(&mut self, op: OpAmp) {
        self.instance.set_follower(op.index());
    }

    /// Releases the underlying peripheral.
    pub fn free(self) -> T {
        self.instance
    }
}

// DA devices have no OPAMP, so this helper is only used by the DB impls.
#[cfg(any(feature = "avr128db28", feature = "avr128db48", feature = "avr128db64"))]
macro_rules! set_follower_op {
    ($self:ident, $inmux:ident, $ctrla:ident) => {{
        $self.$inmux().write(|w| w.muxpos().inp().muxneg().out());
        $self.$ctrla().write(|w| w.outmode().normal());
    }};
}

#[cfg(any(feature = "avr128db48", feature = "avr128db64"))]
macro_rules! impl_opamp_instance {
    ($OPAMP:ty) => {
        impl OpampInstance for $OPAMP {
            fn enable(&self, timebase: u8) {
                self.timebase().write(|w|
                    // SAFETY: TIMEBASE is a plain cycle-count field. Any value is valid.
                    unsafe { w.timebase().bits(timebase) });
                self.ctrla().write(|w| w.enable().set_bit());
            }
            fn set_follower(&self, op: u8) {
                match op {
                    0 => set_follower_op!(self, op0inmux, op0ctrla),
                    1 => set_follower_op!(self, op1inmux, op1ctrla),
                    _ => set_follower_op!(self, op2inmux, op2ctrla),
                }
            }
        }
    };
}

// db28 only has two op-amps (OP0/OP1).
#[cfg(feature = "avr128db28")]
macro_rules! impl_opamp_instance_2op {
    ($OPAMP:ty) => {
        impl OpampInstance for $OPAMP {
            fn enable(&self, timebase: u8) {
                self.timebase().write(|w|
                    // SAFETY: TIMEBASE is a plain cycle-count field. Any value is valid.
                    unsafe { w.timebase().bits(timebase) });
                self.ctrla().write(|w| w.enable().set_bit());
            }
            fn set_follower(&self, op: u8) {
                match op {
                    0 => set_follower_op!(self, op0inmux, op0ctrla),
                    _ => set_follower_op!(self, op1inmux, op1ctrla),
                }
            }
        }
    };
}

#[cfg(feature = "avr128db28")]
impl_opamp_instance_2op!(avr_device::avr128db28::OPAMP);
#[cfg(feature = "avr128db48")]
impl_opamp_instance!(avr_device::avr128db48::OPAMP);
#[cfg(feature = "avr128db64")]
impl_opamp_instance!(avr_device::avr128db64::OPAMP);
