//! Hardware abstraction layer for Microchip **AVR128** devices
//! (AVR128DB48 / DB64 / DA64).
//!
//! Thin, `embedded-hal`-implementing wrappers around the device peripherals.
//! Every wrapper is generic over an "instance" trait (e.g.
//! [`twi::TwiInstance`]). The user passes in the PAC peripheral. The trait is
//! implemented for that peripheral type behind the matching device feature
//! (`avr128db48`, `avr128db64`, `avr128da64`). The features are **additive**.
//! Enabling several together only adds more impls, so one build can target a
//! family of devices.
//!
//! Each module configures only its own peripheral registers. Pin direction and
//! `PORTMUX` routing are the application's job, so the HAL stays
//! board-agnostic.
//!
//! Covered peripherals: clock ([`clock`]), busy-wait delay ([`delay`]), GPIO
//! ([`gpio`]), I2C/TWI ([`twi`]), SPI ([`spi`]), USART ([`usart`]), 16-bit
//! timer PWM ([`pwm`]), analog input ([`adc`]), analog output ([`dac`]),
//! voltage reference ([`vref`]), watchdog ([`wdt`]), real-time counter
//! ([`rtc`]), custom logic ([`ccl`]), op-amps ([`opamp`]), zero-cross
//! detection ([`zcd`]) and the signature row ([`sigrow`]).

#![no_std]

// Re-exported so applications can name the PAC types they pass in.
pub use avr_device;

pub mod adc;
pub mod ccl;
pub mod clock;
pub mod dac;
pub mod delay;
pub mod gpio;
pub mod opamp;
pub mod pwm;
pub mod rtc;
pub mod sigrow;
pub mod spi;
pub mod twi;
pub mod usart;
pub mod vref;
pub mod wdt;
pub mod zcd;

mod wait;
