stm32l0x1-hal
=============

_stm32l0x1-hal_ contains a hardware abstraction on top of the peripheral access API for the ST Micro STM32L0x1 series of microcontrollers. Please note that this crate is being used extensively for rapid development, and the API is subject to sometimes-dramatic change. The goal is to release a 1.0 version, at which point the APIs will be treated more carefully and with an eye toward backware-compatibility.

This crate relies on the [stm32l0](https://crates.io/crates/stm32l0) crate for register and peripheral definitions, and implements a partial (and in-progress) set of the [embedded-hal](https://github.com/japaric/embedded-hal.git) traits.

Links
-----

[Reference manual](https://www.st.com/resource/en/reference_manual/dm00108282-ultralowpower-stm32l0x1-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)

## Datasheets

[STM32L011x3 STM32L011x4](https://www.st.com/resource/en/datasheet/stm32l011d4.pdf)

License
-------

[0-clause BSD license](LICENSE.txt)
