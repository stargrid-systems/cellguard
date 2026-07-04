//! Configurable Custom Logic (CCL).
//!
//! [`Ccl`] is generic over a [`CclInstance`] (implemented for each device's
//! `CCL`). Each of the six look-up tables (LUT0..LUT5) computes a programmable
//! 3-input boolean function (an 8-entry truth table) from three selectable
//! inputs. On the CellGuard board, LUT5 combines `TINY_ALIVE` and
//! `RS485_PWR_OK` into the system-OK / emergency-off signal.
//!
//! Configure each LUT *before* enabling the peripheral.

/// Which look-up table to configure.
#[derive(Clone, Copy)]
pub enum Lut {
    Lut0,
    Lut1,
    Lut2,
    Lut3,
    Lut4,
    Lut5,
}

impl Lut {
    const fn index(self) -> u8 {
        match self {
            Self::Lut0 => 0,
            Self::Lut1 => 1,
            Self::Lut2 => 2,
            Self::Lut3 => 3,
            Self::Lut4 => 4,
            Self::Lut5 => 5,
        }
    }
}

/// A LUT input source (the common `INSEL` selections).
#[derive(Clone, Copy)]
pub enum Input {
    /// Input forced to 0.
    Masked,
    /// This LUT's own (registered) output.
    Feedback,
    /// The previous LUT's output.
    Link,
    /// Event channel A.
    EventA,
    /// Event channel B.
    EventB,
    /// The associated I/O pin.
    Io,
}

impl Input {
    /// The raw `INSEL` field code.
    const fn code(self) -> u8 {
        match self {
            Self::Masked => 0,
            Self::Feedback => 1,
            Self::Link => 2,
            Self::EventA => 3,
            Self::EventB => 4,
            Self::Io => 5,
        }
    }
}

/// Configuration for one LUT.
#[derive(Clone, Copy)]
pub struct LutConfig {
    /// The three input sources (IN0, IN1, IN2).
    pub inputs: [Input; 3],
    /// The 8-bit truth table: bit `n` is the output for input combination `n`.
    pub truth: u8,
    /// Whether to drive the LUT output on its I/O pin.
    pub output_to_pin: bool,
}

/// A CCL peripheral. Implemented for each device's `CCL`. Not for external use.
pub trait CclInstance {
    /// Configures one LUT (`lut` = 0..=5) from raw INSEL codes + truth table.
    fn write_lut(&self, lut: u8, inputs: [u8; 3], truth: u8, output: bool);
    /// Enables the peripheral.
    fn enable(&self);
}

/// The Configurable Custom Logic peripheral.
pub struct Ccl<T: CclInstance> {
    instance: T,
}

impl<T: CclInstance> Ccl<T> {
    /// Takes ownership of the CCL peripheral (disabled).
    #[must_use]
    pub fn new(instance: T) -> Self {
        Self { instance }
    }

    /// Configures one LUT. The LUT must be configured before [`Ccl::enable`].
    pub fn configure(&mut self, lut: Lut, config: LutConfig) {
        let inputs = [
            config.inputs[0].code(),
            config.inputs[1].code(),
            config.inputs[2].code(),
        ];
        self.instance
            .write_lut(lut.index(), inputs, config.truth, config.output_to_pin);
    }

    /// Enables the CCL peripheral (all configured LUTs become active).
    pub fn enable(&mut self) {
        self.instance.enable();
    }

    /// Releases the underlying peripheral.
    pub fn free(self) -> T {
        self.instance
    }
}

// Hidden implementation detail. Writes one LUT's registers. The LUTn register
// accessors are distinct types, so the match cannot be generic. This private
// macro keeps it next to the per-device impl and centralises the `unsafe`.
macro_rules! write_one_lut {
    ($self:ident, $i:ident, $truth:ident, $out:ident,
     $ctrlb:ident, $ctrlc:ident, $tr:ident, $ctrla:ident) => {{
        // SAFETY: each input code is a valid INSEL selection (0..=5).
        unsafe {
            $self
                .$ctrlb()
                .write(|w| w.insel0().bits($i[0]).insel1().bits($i[1]));
            $self.$ctrlc().write(|w| w.insel2().bits($i[2]));
        }
        $self.$tr().write(|w| w.set($truth));
        $self
            .$ctrla()
            .write(|w| w.enable().set_bit().outen().bit($out));
    }};
}

#[cfg(any(feature = "avr128db48", feature = "avr128db64", feature = "avr128da64"))]
macro_rules! impl_ccl_instance {
    ($CCL:ty) => {
        impl CclInstance for $CCL {
            fn write_lut(&self, lut: u8, i: [u8; 3], truth: u8, out: bool) {
                match lut {
                    0 => write_one_lut!(self, i, truth, out, lut0ctrlb, lut0ctrlc, truth0, lut0ctrla),
                    1 => write_one_lut!(self, i, truth, out, lut1ctrlb, lut1ctrlc, truth1, lut1ctrla),
                    2 => write_one_lut!(self, i, truth, out, lut2ctrlb, lut2ctrlc, truth2, lut2ctrla),
                    3 => write_one_lut!(self, i, truth, out, lut3ctrlb, lut3ctrlc, truth3, lut3ctrla),
                    4 => write_one_lut!(self, i, truth, out, lut4ctrlb, lut4ctrlc, truth4, lut4ctrla),
                    _ => write_one_lut!(self, i, truth, out, lut5ctrlb, lut5ctrlc, truth5, lut5ctrla),
                }
            }
            fn enable(&self) {
                self.ctrla().write(|w| w.enable().set_bit());
            }
        }
    };
}

// db28 only has four LUTs (LUT0..LUT3).
#[cfg(feature = "avr128db28")]
macro_rules! impl_ccl_instance_4lut {
    ($CCL:ty) => {
        impl CclInstance for $CCL {
            fn write_lut(&self, lut: u8, i: [u8; 3], truth: u8, out: bool) {
                match lut {
                    0 => write_one_lut!(self, i, truth, out, lut0ctrlb, lut0ctrlc, truth0, lut0ctrla),
                    1 => write_one_lut!(self, i, truth, out, lut1ctrlb, lut1ctrlc, truth1, lut1ctrla),
                    2 => write_one_lut!(self, i, truth, out, lut2ctrlb, lut2ctrlc, truth2, lut2ctrla),
                    _ => write_one_lut!(self, i, truth, out, lut3ctrlb, lut3ctrlc, truth3, lut3ctrla),
                }
            }
            fn enable(&self) {
                self.ctrla().write(|w| w.enable().set_bit());
            }
        }
    };
}

#[cfg(feature = "avr128db28")]
impl_ccl_instance_4lut!(avr_device::avr128db28::CCL);
#[cfg(feature = "avr128db48")]
impl_ccl_instance!(avr_device::avr128db48::CCL);
#[cfg(feature = "avr128db64")]
impl_ccl_instance!(avr_device::avr128db64::CCL);
#[cfg(feature = "avr128da64")]
impl_ccl_instance!(avr_device::avr128da64::CCL);
