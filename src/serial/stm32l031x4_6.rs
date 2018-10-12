//! Serial devices on the STM32L031x4/STM32L031x6

use stm32l0x1::{LPUART1, USART2};

use gpio::PC0;
use gpio::{PA10, PA13, PA14, PA15, PA2, PA3, PA9};
use gpio::{PB10, PB11, PB6, PB7};

use gpio::AF::{AF0, AF4, AF6};

use super::*;

unsafe impl TxPin<LPUART1> for PA2<AF6> {}
unsafe impl TxPin<LPUART1> for PA14<AF6> {}
unsafe impl TxPin<LPUART1> for PB10<AF6> {}

unsafe impl RxPin<LPUART1> for PA3<AF6> {}
unsafe impl RxPin<LPUART1> for PA13<AF6> {}
unsafe impl RxPin<LPUART1> for PB11<AF6> {}
unsafe impl RxPin<LPUART1> for PC0<AF6> {}

unsafe impl TxPin<USART2> for PA2<AF4> {}
unsafe impl TxPin<USART2> for PA9<AF4> {}
unsafe impl TxPin<USART2> for PA14<AF4> {}
unsafe impl TxPin<USART2> for PB6<AF0> {}

unsafe impl RxPin<USART2> for PA3<AF4> {}
unsafe impl RxPin<USART2> for PA10<AF4> {}
unsafe impl RxPin<USART2> for PA15<AF4> {}
unsafe impl RxPin<USART2> for PB7<AF0> {}
