stm32l0x1-hal
=============

_stm32l0x1-hal_ contains a hardware abstraction on top of the peripheral access API for the ST Micro STM32L0x1 series of microcontrollers. Please note that this crate is being used extensively for rapid development, and the API is subject to sometimes-dramatic change. The goal is to release a 1.0 version, at which point the APIs will be treated more carefully and with an eye toward backware-compatibility.

This crate relies on the [stm32l0x1](https://github.com/chocol4te/stm32l0x1) crate for register and peripheral definitions, and implements a partial (and in-progress) set of the [embedded-hal](https://github.com/japaric/embedded-hal.git) traits.

License
-------

[0-clause BSD license](LICENSE.txt)
