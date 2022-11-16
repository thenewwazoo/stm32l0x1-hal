//! General Purpose Input / Output
//!
//! Implementation of the GPIO functionality for the STM32L0x1.
//!
//! Typical usage is to create a GPIO instance by trading in the Peripheral member:
//!
//! ```
//! use stm32l0x1_hal as hal;
//!
//! let d = hal::stm32l0x1::Peripherals::take().unwrap();
//! let mut power = d.PWR.constrain();
//! let mut flash = d.FLASH.constrain();
//! let mut rcc = d.RCC.constrain().freeze(&mut flash, &mut pwr);
//!
//! let gpioa = gpio::A::new(d.GPIOA, &mut self.rcc.iop);
//!
//! // configure the pin as an output
//! let pin = gpioa.PA0.into_output::<PushPull, Floating>();
//!
//! // as an analog pin
//! let pin = pin.into_analog();
//!
//! // and so on
//! ```

#![allow(unknown_lints)]
#![allow(clippy::all)]

use core::convert::Infallible;
use core::marker::PhantomData;

use hal::digital::{InputPin, OutputPin, StatefulOutputPin};
use rcc;

use stm32l0x1;

#[doc(hidden)]
mod private {
    /// Sealed stops crates other than STM32L0x1-HAL from implementing traits that use it.
    pub trait Sealed {}

    impl Sealed for super::Analog {}
    impl Sealed for super::Floating {}
    impl Sealed for super::PullDown {}
    impl Sealed for super::PullUp {}
    impl<MODE> Sealed for super::Input<MODE> {}
    impl Sealed for super::PushPull {}
    impl Sealed for super::OpenDrain {}
    impl<MODE, PUMODE> Sealed for super::Output<MODE, PUMODE> {}
}

#[doc(hidden)]
/// Helper trait for configuring PUPDR registers for analog mode
pub trait AnalogMode: private::Sealed {
    /// Used to set pin to floating
    fn modify_pupdr_bits(original: u32, offset: u32) -> u32;
}

/// Analog mode (type state) indicating that a pin is configured in Analog (high-z) input mode
pub struct Analog(());
impl AnalogMode for Analog {
    #[inline]
    fn modify_pupdr_bits(original: u32, offset: u32) -> u32 {
        original & !(0b11 << offset)
    }
}

#[doc(hidden)]
/// Helper trait for configuring PUPDR registers for the desired pull up/down mode
pub trait PullMode: private::Sealed {
    /// Manipulate pull up/down bits
    fn modify_pupdr_bits(original: u32, offset: u32) -> u32;
}

/// Floating input (type state) indicating that a pin is floating
pub struct Floating;
impl PullMode for Floating {
    #[inline]
    fn modify_pupdr_bits(original: u32, offset: u32) -> u32 {
        original & !(0b11 << offset)
    }
}

/// Pulled down input (type state) indicating that the pin is configured for pull-down
pub struct PullDown;
impl PullMode for PullDown {
    #[inline]
    fn modify_pupdr_bits(original: u32, offset: u32) -> u32 {
        (original & !(0b11 << offset)) | (0b10 << offset)
    }
}

/// Pulled up input (type state) indicating that the pin is configured for pull-up
pub struct PullUp;
impl PullMode for PullUp {
    #[inline]
    fn modify_pupdr_bits(original: u32, offset: u32) -> u32 {
        (original & !(0b11 << offset)) | (0b01 << offset)
    }
}

#[doc(hidden)]
/// Input mode (type state) indicating that the pin is configured as an input
pub struct Input<MODE> {
    #[doc(hidden)]
    _mode: PhantomData<MODE>,
}

#[doc(hidden)]
/// Helper trait for configuring the OTYPER register for the desired output drive mode
pub trait OutputMode: private::Sealed {
    /// Modify output type bits
    fn modify_otyper_bits(original: u32, idx: u8) -> u32;
}

/// Push pull output (type state) indicating that the pin is configured for push-pull
pub struct PushPull;
impl OutputMode for PushPull {
    #[inline]
    fn modify_otyper_bits(original: u32, idx: u8) -> u32 {
        original & !(0b1 << idx)
    }
}

/// Open drain output (type state) indicating that the pin is configured as an open-drain
pub struct OpenDrain;
impl OutputMode for OpenDrain {
    #[inline]
    fn modify_otyper_bits(original: u32, idx: u8) -> u32 {
        original | (0b1 << idx)
    }
}

/// Output mode (type state) indicating that the pin is configured as an output
pub struct Output<MODE, PUMODE> {
    #[doc(hidden)]
    _mode: PhantomData<MODE>,
    #[doc(hidden)]
    _pu: PhantomData<PUMODE>,
}

/// Pin drive strength
///
/// Note: Refer to the device datasheet for the frequency specifications and the power supply and
/// load conditions for each speed.
#[allow(missing_docs)]
#[repr(C)]
pub enum PinSpeed {
    Low = 0,
    Medium,
    High,
    VeryHigh,
}

macro_rules! impl_parts {
    ($($GPIOX:ident, $gpiox:ident;)+) => {
        $(
            use stm32l0x1::$GPIOX;
            impl AFRL<$GPIOX> {
                /// Opaque AFRL register
                pub(crate) fn afr(&mut self) -> &stm32l0x1::$gpiox::AFRL {
                    unsafe { &(*$GPIOX::ptr()).afrl }
                }
            }
            impl AFRH<$GPIOX> {
                /// Opaque AFRH register
                pub(crate) fn afr(&mut self) -> &stm32l0x1::$gpiox::AFRH {
                    unsafe { &(*$GPIOX::ptr()).afrh }
                }
            }
            impl MODER<$GPIOX> {
                /// Opaque MODER register
                pub(crate) fn moder(&mut self) -> &stm32l0x1::$gpiox::MODER {
                    unsafe { &(*$GPIOX::ptr()).moder }
                }
            }
            impl OTYPER<$GPIOX> {
                /// Opaque OTYPER register
                pub(crate) fn otyper(&mut self) -> &stm32l0x1::$gpiox::OTYPER {
                    unsafe { &(*$GPIOX::ptr()).otyper }
                }
            }
            impl PUPDR<$GPIOX> {
                /// Opaque PUPDR register
                pub(crate) fn pupdr(&mut self) -> &stm32l0x1::$gpiox::PUPDR {
                    unsafe { &(*$GPIOX::ptr()).pupdr }
                }
            }
            impl OSPEEDR<$GPIOX> {
                /// Opaque OSPEEDR register
                pub(crate) fn ospeedr(&mut self) -> &stm32l0x1::$gpiox::OSPEEDR {
                    unsafe { &(*$GPIOX::ptr()).ospeedr }
                }
            }
         )+
    }
}

macro_rules! impl_gpio {
    ($name:ident, $GPIOX:ident, $gpioen:ident, $gpiorst:ident) => {
        impl_gpio!($name, $GPIOX, $gpioen, $gpiorst, AFRL: [], AFRH: []);
    };
    ($name:ident, $GPIOX:ident, $gpioen:ident, $gpiorst:ident, AFRL: [$($PXiL:ident, $iL:expr;)*]) => {
        impl_gpio!($name, $GPIOX, $gpioen, $gpiorst, AFRL: [$($PXiL, $iL;)*], AFRH: []);
    };
    ($name:ident, $GPIOX:ident, $gpioen:ident, $gpiorst:ident, AFRL: [$($PXiL:ident, $iL:expr;)*], AFRH: [$($PXiH:ident, $iH:expr;)*]) => {

        impl_pins!($GPIOX, AFRL: [$($PXiL, $iL;)*]);
        impl_pins!($GPIOX, AFRH: [$($PXiH, $iH;)*]);

        #[allow(non_snake_case)]
        /// GPIO
        ///
        /// Once created, a GPIO instance is usually pulled apart by moving its fields.
        pub struct $name {
            $(
                /// Pin
                pub $PXiL: $PXiL<Analog>,
            )*
            $(
                /// Pin
                pub $PXiH: $PXiH<Analog>,
            )*
        }

        impl $name {
            /// Trade the GPIO registers for an instance
            pub fn new(_gpio: $GPIOX, iop: &mut rcc::IOP) -> Self {
                iop.enr().modify(|_,w| w.$gpioen().set_bit());
                while iop.enr().read().$gpioen().bit_is_clear() {}
                Self {
                    $(
                        $PXiL: $PXiL(PhantomData),
                    )*
                    $(
                        $PXiH: $PXiH(PhantomData),
                    )*
                }
            }
        }
    }
}

macro_rules! impl_pin {
    ($GPIOX:ident, $PXi:ident, $AFR:ident, $i:expr) => {
        /// Specific GPIO pin
        pub struct $PXi<MODE>(PhantomData<MODE>);

        impl<MODE> $PXi<MODE> {
            const OFFSET: u32 = 2 * $i;

            /// Configures the PIN to operate as a high-impedance analog input
            pub fn into_analog(self) -> $PXi<Analog> {
                let mut moder: MODER<$GPIOX> = MODER(PhantomData);
                let mut pupdr: PUPDR<$GPIOX> = PUPDR(PhantomData);
                pupdr.pupdr().modify(|r, w| unsafe {
                    w.bits(Analog::modify_pupdr_bits(r.bits(), Self::OFFSET))
                });
                moder
                    .moder()
                    .modify(|r, w| unsafe { w.bits(r.bits() | (0b11 << Self::OFFSET)) });

                $PXi(PhantomData)
            }

            /// Configures the PIN to operate as Input Pin according to Mode.
            pub fn into_input<Mode: PullMode>(self) -> $PXi<Input<Mode>> {
                let mut moder: MODER<$GPIOX> = MODER(PhantomData);
                let mut pupdr: PUPDR<$GPIOX> = PUPDR(PhantomData);

                moder
                    .moder()
                    .modify(|r, w| unsafe { w.bits(r.bits() & !(0b11 << Self::OFFSET)) });
                pupdr.pupdr().modify(|r, w| unsafe {
                    w.bits(Mode::modify_pupdr_bits(r.bits(), Self::OFFSET))
                });

                $PXi(PhantomData)
            }

            /// Set pin drive strength of the pin
            #[inline]
            pub fn set_pin_speed(&self, spd: PinSpeed) {
                let mut ospeedr: OSPEEDR<$GPIOX> = OSPEEDR(PhantomData);

                ospeedr.ospeedr().modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0b11 << Self::OFFSET)) | ((spd as u32) << Self::OFFSET))
                });
            }

            /// Configures the PIN to operate as Output Pin according to OMode and PUMode
            pub fn into_output<OMode: OutputMode, PUMode: PullMode>(
                self,
            ) -> $PXi<Output<OMode, PUMode>> {
                let mut moder: MODER<$GPIOX> = MODER(PhantomData);
                let mut otyper: OTYPER<$GPIOX> = OTYPER(PhantomData);
                let mut pupdr: PUPDR<$GPIOX> = PUPDR(PhantomData);

                moder.moder().modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0b11 << Self::OFFSET)) | (0b01 << Self::OFFSET))
                });
                pupdr.pupdr().modify(|r, w| unsafe {
                    w.bits(PUMode::modify_pupdr_bits(r.bits(), Self::OFFSET))
                });
                otyper
                    .otyper()
                    .modify(|r, w| unsafe { w.bits(OMode::modify_otyper_bits(r.bits(), $i)) });

                $PXi(PhantomData)
            }

            /// Configures the PIN to operate as Alternate Function.
            pub fn into_alt_fun<AF: AltFun>(self) -> $PXi<AF> {
                let mut moder: MODER<$GPIOX> = MODER(PhantomData);
                let mut afr: $AFR<$GPIOX> = $AFR(PhantomData);

                // AFRx pin fields are 4 bits wide, and each 8-pin bank has its own reg (L or H); e.g. pin 8's offset is _0_, within AFRH.
                const AFR_OFFSET: usize = ($i % 8) * 4;
                moder.moder().modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0b11 << Self::OFFSET)) | (0b10 << Self::OFFSET))
                });
                afr.afr().modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0b1111 << AFR_OFFSET)) | (AF::NUM << AFR_OFFSET))
                });

                $PXi(PhantomData)
            }
        }

        impl<OMODE, PUMODE> OutputPin for $PXi<Output<OMODE, PUMODE>> {
            type Error = Infallible;

            fn try_set_high(&mut self) -> Result<(), Self::Error> {
                // NOTE(unsafe) atomic write to a stateless register
                Ok(unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << $i)) })
            }

            fn try_set_low(&mut self) -> Result<(), Self::Error> {
                // NOTE(unsafe) atomic write to a stateless register
                Ok(unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << (16 + $i))) })
            }
        }

        impl<OMODE, PUMODE> StatefulOutputPin for $PXi<Output<OMODE, PUMODE>> {
            /// Returns whether high bit is set.
            fn try_is_set_high(&self) -> Result<bool, Self::Error> {
                self.try_is_set_low().map(|r| !r)
            }

            /// Returns whether low bit is set.
            fn try_is_set_low(&self) -> Result<bool, Self::Error> {
                // NOTE(unsafe) atomic read with no side effects
                Ok(unsafe { (*$GPIOX::ptr()).odr.read().bits() & (1 << $i) == 0 })
            }
        }

        impl<PUMODE> InputPin for $PXi<Input<PUMODE>> {
            type Error = Infallible;

            fn try_is_high(&self) -> Result<bool, Self::Error> {
                self.try_is_low().map(|r| !r)
            }

            fn try_is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*$GPIOX::ptr()).idr.read().bits() & (1 << $i) == 0 })
            }
        }
    };
}

macro_rules! impl_pins {
    ($GPIOX:ident, $ARF:ident: [$($PXi:ident, $i:expr;)*]) => {
        $(
            impl_pin!($GPIOX, $PXi, $ARF, $i);
         )*
    }
}

/// Opaque AFRL register
pub struct AFRL<GPIO>(PhantomData<GPIO>);
/// Opaque AFRH register
pub struct AFRH<GPIO>(PhantomData<GPIO>);
/// Opaque MODER register
pub struct MODER<GPIO>(PhantomData<GPIO>);
/// Opaque OTYPER register
pub struct OTYPER<GPIO>(PhantomData<GPIO>);
/// Opaque PUPDR register
pub struct PUPDR<GPIO>(PhantomData<GPIO>);
/// Opaque OSPEEDR register
pub struct OSPEEDR<GPIO>(PhantomData<GPIO>);

macro_rules! impl_af {
    ( [$($af:ident, $i:expr;)*] ) => {
        $(
            /// Alternate pin function (type state)
            pub struct $af;
            impl super::AltFun for $af {
                const NUM: u32 = $i;
            }
         )*
    }
}

#[doc(hidden)]
/// Helper trait to contain a numeric value used to identify alternate functions
///
/// Note: this trait SHALL NOT be implemented, and should be considered Sealed
pub trait AltFun {
    /// Number of the alternate function
    const NUM: u32;
}

#[allow(non_snake_case)]
/// Module containing the (auto-generated) alternate functions for the GPIOs
pub mod AF {
    impl_af!([AF0, 0; AF1, 1; AF2, 2; AF3, 3; AF4, 4; AF5, 5; AF6, 6; AF7, 7; AF8, 8; AF9, 9; AF10, 10; AF11, 11; AF12, 12; AF13, 13; AF14, 14; AF15, 15;]);
}

impl_parts!(
GPIOA, gpioa;
GPIOB, gpiob;
GPIOC, gpiob; // not a typo
);

impl_gpio!(A, GPIOA, iopaen, gpioarst,
 AFRL: [PA0, 0; PA1, 1; PA2, 2; PA3, 3; PA4, 4; PA5, 5; PA6, 6; PA7, 7;],
 AFRH: [PA8, 8; PA9, 9; PA10, 10; PA11, 11; PA12, 12; PA13, 13; PA14, 14; PA15, 15; ]
);
impl_gpio!(B, GPIOB, iopben, gpiobrst,
 AFRL: [PB0, 0; PB1, 1; PB2, 2; PB3, 3; PB4, 4; PB5, 5; PB6, 6; PB7, 7;],
 AFRH: [PB8, 8; PB9, 9; PB10, 10; PB11, 11; PB12, 12; PB13, 13; PB14, 14; PB15, 15; ]
);
impl_gpio!(C, GPIOC, iopcen, gpiocrst,
 AFRL: [PC0, 0; PC1, 1; PC2, 2; PC3, 3; PC4, 4; PC5, 5; PC6, 6; PC7, 7;],
 AFRH: [PC8, 8; PC9, 9; PC10, 10; PC11, 11; PC12, 12; PC13, 13; PC14, 14; PC15, 15; ]
);
