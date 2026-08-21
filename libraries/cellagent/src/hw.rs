//! Hardware abstraction traits for the cellagent.
//!
//! The firmware crate supplies the implementations. The runtime works with
//! any implementation, so the logic is host-testable with mocks.

/// Controls the active balancer gates.
pub trait GateControl {
    /// Sets gate states from a 1-byte mask (bit 0 = `GATE_A`, bit 1 = `GATE_B`,
    /// bit 2 = `ALL_OFF`).
    fn set_gates(&mut self, mask: u8);
}

/// Reads the cellagent temperature sensor (LM61).
pub trait TempSensor {
    /// Returns temperature in centi-degrees Celsius (e.g. 2500 = 25.00 C).
    fn read_centi_celsius(&mut self) -> i16;
}
