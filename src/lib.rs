//#![deny(missing_docs)]
//#![deny(warnings)]
#![feature(never_type)]
#![feature(unsize)]
#![no_std]

extern crate cast;
extern crate cortex_m;
extern crate embedded_hal as hal;
extern crate nb;
pub extern crate stm32l0x1;

pub mod common;
pub mod flash;
pub mod gpio;
pub mod power;
pub mod rcc;
pub mod serial;
pub mod time;
