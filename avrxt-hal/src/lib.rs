//! Hardware abstraction layer for Microchip **modern-AVR** (AVRxt) devices:
//! AVR128DB48 / DB64 / DA64 and the tinyAVR 0/1-series (`attiny406`,
//! `attiny416`).
//!
//! Thin, `embedded-hal`-implementing wrappers around the device peripherals.
//! Every wrapper is generic over an "instance" trait (e.g.
//! [`twi::TwiInstance`]) that the HAL implements for the PAC peripheral behind
//! the matching device feature. Each module configures only its own registers.
//! Pin direction and `PORTMUX` routing are the application's job, so the HAL
//! stays board-agnostic.
//!
//! # Features
//!
//! Device features are **additive**: enabling several only adds more impls, so
//! one build can target a family of devices. Each device feature also turns on
//! an internal family marker so family-specific code compiles only where it
//! applies.
//!
//! - `avr128db48`, `avr128db64`, `avr128da64`: AVR128 DB/DA devices.
//! - `attiny406`, `attiny416`: tinyAVR 0/1-series devices.
//! - `ufmt`: implement `ufmt::uWrite` for the USART.
//!
//! `_avr128` and `_tinyavr` are internal markers set by the device features. Do
//! not enable them directly.
//!
//! # Peripherals
//!
//! - clock ([`clock`])
//! - delay ([`delay`])
//! - GPIO ([`gpio`])
//! - I2C/TWI ([`twi`])
//! - SPI ([`spi`])
//! - USART ([`usart`])
//! - timer PWM ([`pwm`])
//! - ADC ([`adc`])
//! - DAC ([`dac`])
//! - voltage reference ([`vref`])
//! - watchdog ([`wdt`])
//! - RTC ([`rtc`])
//! - custom logic ([`ccl`])
//! - op-amps ([`opamp`])
//! - zero-cross ([`zcd`])
//! - signature row ([`sigrow`])
#![cfg_attr(
    feature = "_avr128",
    doc = "- non-volatile memory controller ([`nvmctrl`]) for on-chip EEPROM and USERROW"
)]
#![no_std]
#![allow(
    unused_macros,
    reason = "device impl macros are unused when their device feature is off"
)]

// Re-exported so applications can name the PAC types they pass in.
pub use avr_device;

pub mod adc;
pub mod ccl;
pub mod clock;
pub mod dac;
pub mod delay;
pub mod gpio;
#[cfg(feature = "_avr128")]
pub mod nvmctrl;
pub mod opamp;
pub mod pwm;
pub mod rtc;
pub mod sigrow;
pub mod spi;
pub mod twi;
pub mod usart;
pub mod vref;
mod wait;
pub mod wdt;
pub mod zcd;
