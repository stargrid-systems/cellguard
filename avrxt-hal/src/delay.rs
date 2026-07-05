//! Busy-wait delay implementing [`embedded_hal::delay::DelayNs`].

use avr_device::asm::delay_cycles;
use embedded_hal::delay::DelayNs;

/// Cycle-counting delay. Construct with the actual CPU frequency in Hz (the
/// value the [`clock`](crate::clock) module configured).
#[derive(Clone, Copy)]
pub struct Delay {
    cycles_per_us: u32,
}

impl Delay {
    /// Creates a delay for a CPU running at `f_cpu_hz`.
    #[must_use]
    pub const fn new(f_cpu_hz: u32) -> Self {
        Self {
            cycles_per_us: f_cpu_hz / 1_000_000,
        }
    }
}

impl DelayNs for Delay {
    #[inline]
    fn delay_ns(&mut self, ns: u32) {
        // cycles = ns * cycles_per_us / 1000. Split ns into whole microseconds
        // and a sub-us remainder so the multiply divides down first and cannot
        // overflow. This is exact: floor(a*ns/1000) == a*(ns/1000) + a*(ns%1000)/1000.
        let cycles = self
            .cycles_per_us
            .saturating_mul(ns / 1000)
            .saturating_add(self.cycles_per_us.saturating_mul(ns % 1000) / 1000);
        delay_cycles(cycles.max(1));
    }

    #[inline]
    fn delay_us(&mut self, us: u32) {
        delay_cycles(self.cycles_per_us.saturating_mul(us).max(1));
    }

    #[inline]
    fn delay_ms(&mut self, ms: u32) {
        // Avoid the u32 overflow that `ms * 1000` would hit for large delays.
        for _ in 0..ms {
            self.delay_us(1000);
        }
    }
}
