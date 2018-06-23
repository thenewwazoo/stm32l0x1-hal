//! General Purpose Input / Output
//!
//! This module was written with only the STM32L011K4 (on the NUCLEO board) on-hand. Future
//! refinements to expand to other packages is TODO.

use core::marker::PhantomData;
use core::ops::Deref;

use hal::digital::{toggleable, OutputPin, StatefulOutputPin};
use rcc;

use stm32l0x1;

/// Input Mode Trait
/// Implemented only for corresponding structs.
///
/// Note: MUST not be implemented by user.
pub trait InputMode {
    fn modify_pupdr_bits(original: u32, offset: u32) -> u32;
}

/// Floating input (type state)
pub struct Floating;
impl InputMode for Floating {
    #[inline]
    fn modify_pupdr_bits(original: u32, offset: u32) -> u32 {
        original & !(0b11 << offset)
    }
}

/// Pulled down input (type state)
pub struct PullDown;
impl InputMode for PullDown {
    #[inline]
    fn modify_pupdr_bits(original: u32, offset: u32) -> u32 {
        (original & !(0b11 << offset)) | (0b10 << offset)
    }
}

/// Pulled up input (type state)
pub struct PullUp;
impl InputMode for PullUp {
    #[inline]
    fn modify_pupdr_bits(original: u32, offset: u32) -> u32 {
        (original & !(0b11 << offset)) | (0b01 << offset)
    }
}

/// Input mode (type state)
pub struct Input<MODE> {
    _mode: PhantomData<MODE>,
}

/// Output Mode Trait
/// Implemented only for corresponding structs.
///
/// Note: MUST not be implemented by user.
pub trait OutputMode {
    fn modify_otyper_bits(original: u32, idx: u8) -> u32;
}

/// Push pull output (type state)
pub struct PushPull;
impl OutputMode for PushPull {
    #[inline]
    fn modify_otyper_bits(original: u32, idx: u8) -> u32 {
        original & !(0b1 << idx)
    }
}

/// Open drain output (type state)
pub struct OpenDrain;
impl OutputMode for OpenDrain {
    #[inline]
    fn modify_otyper_bits(original: u32, idx: u8) -> u32 {
        original | (0b1 << idx)
    }
}

/// Output mode (type state)
pub struct Output<MODE, PUMODE> {
    _mode: PhantomData<MODE>,
    _pu: PhantomData<PUMODE>,
}

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
                pub(crate) fn afr(&mut self) -> &stm32l0x1::$gpiox::AFRL {
                    unsafe { &(*$GPIOX::ptr()).afrl }
                }
            }
            impl AFRH<$GPIOX> {
                pub(crate) fn afr(&mut self) -> &stm32l0x1::$gpiox::AFRH {
                    unsafe { &(*$GPIOX::ptr()).afrh }
                }
            }
            impl MODER<$GPIOX> {
                pub(crate) fn moder(&mut self) -> &stm32l0x1::$gpiox::MODER {
                    unsafe { &(*$GPIOX::ptr()).moder }
                }
            }
            impl OTYPER<$GPIOX> {
                pub(crate) fn otyper(&mut self) -> &stm32l0x1::$gpiox::OTYPER {
                    unsafe { &(*$GPIOX::ptr()).otyper }
                }
            }
            impl PUPDR<$GPIOX> {
                pub(crate) fn pupdr(&mut self) -> &stm32l0x1::$gpiox::PUPDR {
                    unsafe { &(*$GPIOX::ptr()).pupdr }
                }
            }
            impl OSPEEDR<$GPIOX> {
                pub(crate) fn ospeedr(&mut self) -> &stm32l0x1::$gpiox::OSPEEDR {
                    unsafe { &(*$GPIOX::ptr()).ospeedr }
                }
            }
            impl BRR<$GPIOX> {
                pub fn brr(&mut self) -> &stm32l0x1::$gpiox::BRR {
                    unsafe { &(*$GPIOX::ptr()).brr }
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
        ///GPIO
        pub struct $name {
            /// Opaque AFRH register
            pub afrh: AFRH<$GPIOX>,
            /// Opaque AFRL register
            pub afrl: AFRL<$GPIOX>,
            /// Opaque MODER register
            pub moder: MODER<$GPIOX>,
            /// Opaque OTYPER register
            pub otyper: OTYPER<$GPIOX>,
            /// Opaque PUPDR register
            pub pupdr: PUPDR<$GPIOX>,
            /// Opaque OSPEEDR register
            pub ospeedr: OSPEEDR<$GPIOX>,
            /// Opaque BRR register
            pub brr: BRR<$GPIOX>,
            $(
                /// Pin
                pub $PXiL: $PXiL<Input<Floating>>,
            )*
            $(
                /// Pin
                pub $PXiH: $PXiH<Input<Floating>>,
            )*
        }

        impl $name {
            pub fn new(iop: &mut rcc::IOP) -> Self {
                iop.enr().modify(|_,w| w.$gpioen().set_bit());
                while iop.enr().read().$gpioen().bit_is_clear() {}
                Self {
                    afrh: AFRH(PhantomData),
                    afrl: AFRL(PhantomData),
                    moder: MODER(PhantomData),
                    otyper: OTYPER(PhantomData),
                    pupdr: PUPDR(PhantomData),
                    ospeedr: OSPEEDR(PhantomData),
                    brr: BRR(PhantomData),
                    $(
                        $PXiL: $PXiL(PhantomData),
                    )*
                    $(
                        $PXiH: $PXiH(PhantomData),
                    )*
                }
            }

//            pub fn reset_pin<P, R: Reset<$GPIOX, P>>(&mut self, r: R) -> P {
//                let _ = core::mem::replace(&mut r, r.reset(&mut self.brr));
//                r
//                //r.reset(&mut self.brr)
//            }
        }

    }
}

//pub trait Reset<G, P> {
//    fn reset(self, brr: &mut BRR<G>) -> P;
//}

macro_rules! impl_pin {
    ($GPIOX:ident, $PXi:ident, $AFR:ident, $i:expr) => {
        /// Specific Pin
        pub struct $PXi<MODE>(PhantomData<MODE>);

//        impl<MODE> Reset<$GPIOX, $PXi<Input<Floating>>> for $PXi<MODE> {
//
//            fn reset(self, brr: &mut BRR<$GPIOX>) -> $PXi<Input<Floating>> {
//
//                brr.brr().write(|w| unsafe { w.bits(1 << $i) });
//                $PXi(PhantomData)
//            }
//
//        }

        impl<MODE> $PXi<MODE> {
            const OFFSET: u32 = 2 * $i;

            /// Configures the PIN to operate as Input Pin according to Mode.
            pub fn into_input<Mode: InputMode>(self, moder: &mut MODER<$GPIOX>, pupdr: &mut PUPDR<$GPIOX>) -> $PXi<Input<Mode>> {
                moder.moder().modify(|r, w| unsafe { w.bits(r.bits() & !(0b11 << Self::OFFSET)) });
                pupdr.pupdr().modify(|r, w| unsafe { w.bits(Mode::modify_pupdr_bits(r.bits(), Self::OFFSET)) });

                $PXi(PhantomData)
            }

            #[inline]
            pub fn set_pin_speed(&self, spd: PinSpeed, ospeedr: &mut OSPEEDR<$GPIOX>) {
                ospeedr
                    .ospeedr()
                    .modify(|r, w| unsafe { w.bits((r.bits() & !(0b11 << Self::OFFSET)) | ((spd as u32) << Self::OFFSET)) });
            }

            /// Configures the PIN to operate as Output Pin according to Mode.
            pub fn into_output<OMode: OutputMode, PUMode: InputMode>(self, moder: &mut MODER<$GPIOX>, otyper: &mut OTYPER<$GPIOX>, pupdr: &mut PUPDR<$GPIOX>) -> $PXi<Output<OMode, PUMode>> {
                moder
                    .moder()
                    .modify(|r, w| unsafe { w.bits((r.bits() & !(0b11 << Self::OFFSET)) | (0b01 << Self::OFFSET)) });
                pupdr.pupdr().modify(|r, w| unsafe { w.bits(PUMode::modify_pupdr_bits(r.bits(), Self::OFFSET)) });
                otyper.otyper().modify(|r, w| unsafe { w.bits(OMode::modify_otyper_bits(r.bits(), $i)) });

                $PXi(PhantomData)
            }

            /// Configures the PIN to operate as Alternate Function.
            pub fn into_alt_fun<AF: AltFun>(self, moder: &mut MODER<$GPIOX>, afr: &mut $AFR<$GPIOX>) -> $PXi<AF> {
                // AFRx pin fields are 4 bits wide, and each 8-pin bank has its own reg (L or H); e.g. pin 8's offset is _0_, within AFRH.
                const AFR_OFFSET: usize = ($i % 8) * 4;
                moder
                    .moder()
                    .modify(|r, w| unsafe { w.bits((r.bits() & !(0b11 << Self::OFFSET)) | (0b10 << Self::OFFSET)) });
                afr.afr().modify(|r, w| unsafe { w.bits((r.bits() & !(0b1111 << AFR_OFFSET)) | (AF::NUM << AFR_OFFSET)) });

                $PXi(PhantomData)
            }
        }

        impl<OMODE, PUMODE> OutputPin for $PXi<Output<OMODE, PUMODE>> {
            /// Sets high bit.
            fn set_high(&mut self) {
                // NOTE(unsafe) atomic write to a stateless register
                unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << $i)) }
            }

            /// Sets low bit.
            fn set_low(&mut self) {
                // NOTE(unsafe) atomic write to a stateless register
                unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << (16 + $i))) }
            }
        }

        impl<OMODE, PUMODE> StatefulOutputPin for $PXi<Output<OMODE, PUMODE>> {
            /// Returns whether high bit is set.
            fn is_set_high(&self) -> bool {
                !self.is_set_low()
            }

            /// Returns whether low bit is set.
            fn is_set_low(&self) -> bool {
                // NOTE(unsafe) atomic read with no side effects
                unsafe { (*$GPIOX::ptr()).odr.read().bits() & (1 << $i) == 0 }
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

/// Generic LED
pub struct Led<PIN>(PIN);
impl<PIN: OutputPin + StatefulOutputPin> Led<PIN> {
    #[inline]
    /// Turns LED off.
    pub fn off(&mut self) {
        self.0.set_low();
    }
    #[inline]
    /// Checks whether LED is off
    pub fn is_off(&mut self) -> bool {
        self.0.is_set_low()
    }
    #[inline]
    /// Turns LED on.
    pub fn on(&mut self) {
        self.0.set_high()
    }
    #[inline]
    /// Checks whether LED is on
    pub fn is_on(&mut self) -> bool {
        self.0.is_set_high()
    }
}

impl<PIN: OutputPin> OutputPin for Led<PIN> {
    #[inline]
    fn set_high(&mut self) {
        self.0.set_high();
    }
    #[inline]
    fn set_low(&mut self) {
        self.0.set_low();
    }
}

impl<PIN: StatefulOutputPin> StatefulOutputPin for Led<PIN> {
    #[inline]
    fn is_set_high(&self) -> bool {
        self.0.is_set_high()
    }
    #[inline]
    fn is_set_low(&self) -> bool {
        self.0.is_set_low()
    }
}

impl<PIN: OutputPin + StatefulOutputPin> toggleable::Default for Led<PIN> {}

impl<PIN> Deref for Led<PIN> {
    type Target = PIN;

    #[inline]
    fn deref(&self) -> &PIN {
        &self.0
    }
}

#[allow(unused_macros)]
macro_rules! define_led {
    ($(#[$attr:meta])* $name:ident, $typ:ty) => {
        $(#[$attr])*
            pub type $name = Led<$typ>;
        impl Led<$typ> {
            #[inline]
            ///Creates a new instance of LED.
            ///
            ///Defined only for these PINs that can be used as LED.
            pub fn new(pin: $typ) -> Self {
                Led(pin)
            }
        }
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
/// Opaque BRR register
pub struct BRR<GPIO>(PhantomData<GPIO>);

macro_rules! impl_af {
    ( [$($af:ident, $i:expr;)*] ) => {
        $(
            pub struct $af;
            impl AltFun for $af {
                const NUM: u32 = $i;
            }
         )*
    }
}

/// Alternate Function Trait
/// Implemented only for corresponding structs.
///
/// Note: MUST not be implemented by user.
pub trait AltFun {
    const NUM: u32;
}

impl_af!([AF0, 0; AF1, 1; AF2, 2; AF3, 3; AF4, 4; AF5, 5; AF6, 6; AF7, 7; AF8, 8; AF9, 9; AF10, 10; AF11, 11; AF12, 12; AF13, 13; AF14, 14; AF15, 15;]);

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
