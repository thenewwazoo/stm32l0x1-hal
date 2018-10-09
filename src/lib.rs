//! STM32L0x1 HAl
#![no_std]
//#![deny(missing_docs)]
#![feature(never_type)]

extern crate cortex_m;
extern crate embedded_hal as hal;
#[macro_use]
extern crate nb;
pub extern crate stm32l0x1;

pub mod adc;
pub mod common;
pub mod flash;
pub mod gpio;
pub mod i2c;
pub mod power;
pub mod rcc;
pub mod serial;
pub mod time;
