//! Serial devices on the STM32L011K4

use stm32l0x1::{LPUART1, USART2};

use gpio::AF::*;
use gpio::*;

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

unsafe impl RtsDePin<LPUART1> for PB1<AF4> {}

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

unsafe impl RtsDePin<USART2> for PA1<AF4> {}
unsafe impl RtsDePin<USART2> for PA12<AF4> {}
unsafe impl RtsDePin<USART2> for PB0<AF4> {}
