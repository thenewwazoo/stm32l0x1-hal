stm32l0x1-hal
=============

## Note! This HAL implementation is partially deprecated!

I am focusing my efforts on exploring new HAL implementation design patterns. As such, my efforts are focused on [the _other_ STM32L0x1 HAL](https://github.com/thenewwazoo/stm32l0x1-context-hal) I've written. Updates, backports from there, or other submissions generally are welcomed!

---

_stm32l0x1-hal_ contains a hardware abstraction on top of the peripheral access API for the ST Micro STM32L0x1 series of microcontrollers.

This crate relies on the [stm32l0x1](https://github.com/chocol4te/stm32l0x1) crate for register and peripheral definitions, and implements a partial (and in-progress) set of the [embedded-hal](https://github.com/japaric/embedded-hal.git) traits.

This crate is being developed primarily with the [NUCLEO-STM32L011K4](http://www.st.com/en/evaluation-tools/nucleo-l011k4.html) board.

License
-------

[0-clause BSD license](LICENSE.txt)
