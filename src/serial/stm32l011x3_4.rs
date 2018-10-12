//! Serial devices on the STM32L011K4

use stm32l0x1::{LPUART1, USART2};

use gpio::{PA0, PA1, PA10, PA13, PA14, PA15, PA2, PA3, PA4, PA8, PA9};
use gpio::{PB6, PB7};

use gpio::AF::{AF0, AF4, AF6};

use super::*;

unsafe impl TxPin<LPUART1> for PA1<AF6> {}
unsafe impl TxPin<LPUART1> for PA2<AF6> {}
unsafe impl TxPin<LPUART1> for PA4<AF6> {}
unsafe impl TxPin<LPUART1> for PA14<AF6> {}
unsafe impl TxPin<LPUART1> for PB6<AF6> {}

unsafe impl RxPin<LPUART1> for PA0<AF6> {}
unsafe impl RxPin<LPUART1> for PA3<AF6> {}
unsafe impl RxPin<LPUART1> for PA13<AF6> {}
unsafe impl RxPin<LPUART1> for PB7<AF6> {}

unsafe impl TxPin<USART2> for PA2<AF4> {}
unsafe impl TxPin<USART2> for PA9<AF4> {}
unsafe impl TxPin<USART2> for PA14<AF4> {}
unsafe impl TxPin<USART2> for PB6<AF0> {}
unsafe impl TxPin<USART2> for PA8<AF0> {}

unsafe impl RxPin<USART2> for PA0<AF0> {}
unsafe impl RxPin<USART2> for PA3<AF4> {}
unsafe impl RxPin<USART2> for PA10<AF4> {}
unsafe impl RxPin<USART2> for PA15<AF4> {}
unsafe impl RxPin<USART2> for PB7<AF0> {}
