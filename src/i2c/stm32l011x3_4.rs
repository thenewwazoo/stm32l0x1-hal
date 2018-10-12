use i2c::SclPin;
use i2c::SdaPin;

use gpio::AF::{AF1, AF3, AF4};
use gpio::{PA10, PA13, PA4, PA9};
use gpio::{PB6, PB7, PB8};
use stm32l0x1::I2C1;

unsafe impl SclPin<I2C1> for PA4<AF3> {}
unsafe impl SclPin<I2C1> for PA9<AF1> {}
unsafe impl SclPin<I2C1> for PB6<AF1> {}
unsafe impl SclPin<I2C1> for PB8<AF4> {}

unsafe impl SdaPin<I2C1> for PA10<AF1> {}
unsafe impl SdaPin<I2C1> for PA13<AF3> {}
unsafe impl SdaPin<I2C1> for PB7<AF1> {}
