use core::cell::RefCell;

use embedded_hal::digital::{ErrorType, InputPin, OutputPin, StatefulOutputPin};
use embedded_hal::i2c::I2c;

use crate::{Address, Tca9535};

struct State {
    
}

struct Pin<I> {
    i2c: I,
    addr: Address,
    index: PinIndex,
}

impl<'a, I: I2c> Pin<'a, I> {}

impl<'a, I: I2c> ErrorType for Pin<'a, I> {
    type Error;
}

impl<'a, I: I2c> InputPin for Pin<'a, I> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        todo!()
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        todo!()
    }
}

impl<'a, I: I2c> OutputPin for Pin<'a, I> {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        todo!()
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        todo!()
    }
}

impl<'a, I: I2c> StatefulOutputPin for Pin<'a, I> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        todo!()
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        todo!()
    }
}

enum PinIndex {
    P0,
    P1,
    P2,
    P3,
    P4,
    P5,
    P6,
    P7,
    P8,
    P9,
    P10,
    P11,
    P12,
    P13,
    P14,
    P15,
}

pub struct P0;

pub struct P1;

pub struct P2;

pub struct P3;

pub struct P4;

pub struct P5;

pub struct P6;

pub struct P7;

pub struct P8;

pub struct P9;

pub struct P10;

pub struct P11;

pub struct P12;

pub struct P13;

pub struct P14;

pub struct P15;
