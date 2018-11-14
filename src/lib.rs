//! STM32L0x1 HAL
//!
//! This HAL provides an implementation of the `embedded-hal` traits for the STM32L0x1 family of
//! microcontrollers.
//!
//! **NOTE**: This crate contains chip-specific features and modules. Where convenient, common
//! peripherals (e.g. GPIO ports) are implemented by default, but some chips have more than others.

#![no_std]
#![deny(missing_docs)]

extern crate cast;
extern crate cortex_m;
extern crate embedded_hal as hal;
#[macro_use]
extern crate nb;
pub extern crate stm32l0x1;
extern crate void;
extern crate flash_embedded_hal as fhal;

pub mod adc;
pub mod common;
pub mod flash;
pub mod gpio;
pub mod i2c;
pub mod power;
pub mod rcc;
pub mod serial;
pub mod time;
pub mod timer;
