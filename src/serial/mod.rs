//! USART(s) and LPUART
//!
//! This is an implementation of RS232 USART(s) and LPUART. The interface for the LPUART and USARTs
//! is largely the same, though their interaction with various power modes is different (see the
//! Reference Manual).
//!
//! ```rust
//! extern crate stm32l0x1_hal;
//!
//! use stm32l0x1_hal::common::Constrain;
//!
//! let d = stm32l0x1_hal::stm32l0x1::Peripherals::take().unwrap();
//!
//! let mut rcc = d.RCC.constrain();
//! let mut flash = d.FLASH.constrain();
//!
//! rcc.freeze(&mut flash, &mut pwr);
//! let clk_ctx = rcc.cfgr.context().unwrap();
//!
//! let mut gpiob = gpio::B::new(d.GPIOB, &mut rcc.iop);
//!
//! // VCP USART
//! let vcp_rx = gpioa.PA15.into_output::<PushPull, Floating>().into_alt_fun::<AF4>();
//! vcp_rx.set_pin_speed(PinSpeed::VeryHigh);
//!
//! let vcp_tx = gpioa.PA2.into_output::<PushPull, Floating>().into_alt_fun::<AF4>();
//! vcp_tx.set_pin_speed(PinSpeed::VeryHigh);
//!
//! let vcp_serial = Serial::usart2(
//!     d.USART2,
//!     (vcp_tx, vcp_rx),
//!     Bps(115200),
//!     clocking::USARTClkSource::HSI16,
//!     &clk_ctx,
//!     &mut rcc.apb1,
//!     &mut rcc.ccipr);
//!
//! let (mut tx, mut rx) = vcp_serial.split();
//!
//! loop {
//!     block!(tx.write(block!(rx.read()).unwrap())).unwrap();
//! }
//! ```

use core::cmp::{max, min};
use core::convert::Infallible;
use core::marker::PhantomData;
use core::ptr;

use nb;

use crate::hal::serial::{Read, Write};
use crate::rcc::clocking::USARTClkSource;
use crate::rcc::ClockContext;
use crate::rcc::{APB1, CCIPR};
use crate::stm32l0::stm32l0x1;
use crate::stm32l0::stm32l0x1::{LPUART1, USART2};
use crate::time::Bps;

#[cfg(any(feature = "STM32L011x3", feature = "STM32L011x4"))]
pub mod stm32l011x3_4;

#[cfg(any(feature = "STM32L031x4", feature = "STM32L031x6"))]
pub mod stm32l031x4_6;

/// UART interrupt event sources
pub enum Event {
    /// New data has been received or overrun error detected
    Rxne,
    /// New data can be sent or framing error (in Smartcard mode)
    Txe,
    /// The line has gone idle
    Idle,
    /// The transmission is complete: a byte has been sent with no byte waiting in TDR
    Tc,
    /// Parity error
    Peie,
    /// Noise, Flag, Overrun error and Framing Error in multibuffer communication.
    Eie,
}

/// Serial errors
#[derive(Debug)]
pub enum Error {
    /// Framing error
    Framing,
    /// Noise error
    Noise,
    /// RX buffer overrun
    Overrun,
    /// Parity check error
    Parity,
    #[doc(hidden)]
    _Extensible,
}

#[derive(Debug)]
/// Errors relating to wake-from-sleep
pub enum SleepError {
    /// The selected source clock cannot wake the chip from stop mode
    BadSourceClock,
}

#[doc(hidden)]
/// Marker trait to identify that a GPIO pin can be used as Tx
///
/// Note: this trait SHALL NOT be implemented, and should be considered Sealed
pub unsafe trait TxPin<USART> {}

#[doc(hidden)]
/// Marker trait to identify that a GPIO pin can be used as Rx
///
/// Note: this trait SHALL NOT be implemented, and should be considered Sealed
pub unsafe trait RxPin<USART> {}

#[doc(hidden)]
/// Marker trait to identify that a GPIO pin can be used as an RTS or DE (RS485) pin
///
/// Note: this trait SHALL NOT be implemented, and should be considered Sealed
pub unsafe trait RtsDePin<USART> {}

/// Serial abstraction
pub struct Serial<USART, PINS> {
    usart: USART,
    #[allow(dead_code)]
    pins: PINS,
}

/// Serial receiver channel
pub struct Rx<USART> {
    _usart: PhantomData<USART>,
}

/// Serial transmitter channel
pub struct Tx<USART> {
    _usart: PhantomData<USART>,
}

/// Serial RS232 RTS / RS485 DE channel
pub struct RtsDe<USART> {
    _usart: PhantomData<USART>,
}

macro_rules! hal {
    ($(
        $USARTX:ident: (
            $usartX:ident,
            $APB:ident,
            $pclk: ident,
            $usartXen:ident,
            $usartXsel:ident),
    )+) => {
        $(

            /// USART module
            pub mod $usartX {
                use super::*;

                /// Initialize the USART peripheral for RS485
                pub fn rs485<TX, RX, DE>(
                    mut usart: $USARTX,
                    pins: (TX, RX, DE),
                    baud_rate: Bps,
                    clk_src: USARTClkSource,
                    clk_ctx: &ClockContext,
                    apb: &mut $APB,
                    ccipr: &mut CCIPR,
                ) -> Serial<$USARTX, (TX, RX, DE)>
                where
                    TX: TxPin<$USARTX>,
                    RX: RxPin<$USARTX>,
                    DE: RtsDePin<$USARTX>,
                {

                    let clk_f = match clk_src {
                        USARTClkSource::PCLK   => clk_ctx.$pclk().0,
                        USARTClkSource::SYSCLK => clk_ctx.sysclk().0,
                        USARTClkSource::HSI16  => clk_ctx.hsi16().expect("HSI16 clk not enabled").0,
                        USARTClkSource::LSE    => clk_ctx.lse().expect("LSE not enabled").0,
                    };

                    // From 25.4.10 RS232 Hardware flow control and RS485 Driver Enable:
                    //     The assertion time is the time between the activation of the DE signal
                    //     and the beginning of the START bit. It is programmed using the DEAT
                    //     [4:0] bit fields in the CR1 control register. The de-assertion time is
                    //     the time between the end of the last stop bit, in a transmitted message,
                    //     and the de-activation of the DE signal. It is programmed using the DEDT
                    //     [4:0] bit fields in the CR1 control register.
                    // For now, pick a period equal to half a bit

                    // Do not set a 0-clock period; do not set a period greater than 5 bits
                    let clks_per_half_bit: u8 =
                        min(
                            max(
                                clk_f / (baud_rate.0 * 8) / 2,
                                31
                               ) as u8,
                               1
                           );
                    let clks_per_half_bit = clks_per_half_bit & 0b11111;

                    usart.cr1.modify(|_, w| w.deat().bits(clks_per_half_bit).dedt().bits(clks_per_half_bit));

                    usart.cr3.modify(|_, w| w.dem().set_bit());

                    <Serial<$USARTX, (TX, RX, DE)>>::configure(&mut usart, baud_rate, clk_src, clk_ctx, apb, ccipr);

                    Serial { usart, pins }
                }

                /// Initialize the USART peripheral to provide RS232 8N1 asynchronous serial
                /// communication with an oversampling rate of 16.
                pub fn rs232<TX, RX>(
                    mut usart: $USARTX,
                    pins: (TX, RX),
                    baud_rate: Bps,
                    clk_src: USARTClkSource,
                    clk_ctx: &ClockContext,
                    apb: &mut $APB,
                    ccipr: &mut CCIPR,
                ) -> Serial<$USARTX, (TX, RX)>
                where
                    TX: TxPin<$USARTX>,
                    RX: RxPin<$USARTX>,
                {
                    <Serial<$USARTX, (TX, RX)>>::configure(&mut usart, baud_rate, clk_src, clk_ctx, apb, ccipr);

                    Serial { usart, pins }
                }

                impl<PINS> Serial<$USARTX, PINS> {
                    fn configure(
                        usart: &mut $USARTX,
                        baud_rate: Bps,
                        clk_src: USARTClkSource,
                        clk_ctx: &ClockContext,
                        apb: &mut $APB,
                        ccipr: &mut CCIPR,
                    ) {
                        apb.enr().modify(|_, w| w.$usartXen().set_bit());
                        while apb.enr().read().$usartXen().bit_is_clear() {}

                        let (clk_f, sel_bits) = match clk_src {
                            USARTClkSource::PCLK   => (clk_ctx.$pclk().0,   0b00),
                            USARTClkSource::SYSCLK => (clk_ctx.sysclk().0,  0b01),
                            USARTClkSource::HSI16  => (clk_ctx.hsi16().expect("HSI16 clk not enabled").0, 0b10),
                            USARTClkSource::LSE    => (clk_ctx.lse().expect("LSE not enabled").0,   0b11),
                        };

                        usart.cr1.modify(|_,w| w.ue().clear_bit()); // disable the uart

                        // From 24.5.2 "Character Transmission Procedure" and 24.5.3 "Character
                        // Reception Procedure"

                        // 1. Program the M bits in USART_CR1 to define the word length.
                        usart.cr1.modify(|_,w| w
                                         .m1().clear_bit()
                                         .m0().clear_bit());    // 8-bit word length

                        // 2. Select the desired baud rate using the baud rate register USART_BRR
                        let brr = clk_f / baud_rate.0; // 24.5.4 USART baud rate generation
                        if brr < 16 {
                            panic!("impossible BRR");
                        }
                        usart.brr.write(|w| unsafe { w.bits(brr) });

                        // 3. Program the number of stop bits in USART_CR2.
                        usart.cr2.modify(|_,w| w.stop().bits(0b00));          // 1 stop bit

                        // No CR3 configuration required

                        ccipr.inner().modify(|_,w| w.$usartXsel().bits(sel_bits));

                        // 4. Enable the USART by writing the UE bit in USART_CR1 register to 1.
                        usart.cr1.modify(|_,w| w.ue().set_bit()); // __HAL_UART_ENABLE in HAL_UART_Init

                        // 5. Select DMA enable (DMAR) in USART_CR3 if multibuffer communication is to
                        //    take place. Configure the DMA register as explained in multibuffer
                        //    communication.
                        // --> n/a for now

                        // For TX:
                        // 6. Set the TE bit in USART_CR1 to send an idle frame as first transmission.
                        // For RX:
                        // 6. Set the RE bit USART_CR1. This enables the receiver which begins searching for a start bit.
                        usart.cr1.modify(|_,w| w
                                         .pce().clear_bit()   // parity control disabled
                                         .te().set_bit()      // enable tx
                                         .re().set_bit()      // enable rx
                                         .over8().clear_bit() // 16x oversampling - default
                                        );

                        while usart.isr.read().teack().bit_is_clear() {} // UART_CheckIdleState in HAL_UART_Init
                        while usart.isr.read().reack().bit_is_clear() {}

                        usart.cr3.modify(|_,w| w.rtse().clear_bit().ctse().clear_bit()); // no hardware flow control

                        // In asynchronous mode, the following bits must be kept cleared:
                        // - LINEN and CLKEN bits in the USART_CR2 register,
                        // - SCEN, HDSEL and IREN  bits in the USART_CR3 register.
                        //  (defaults acceptable)
                        usart.cr2.modify(|_,w| w
                                         .linen().clear_bit()
                                         .clken().clear_bit()
                                        );
                        usart.cr3.modify(|_,w| w
                                         .scen().clear_bit()
                                         .hdsel().clear_bit()
                                         .iren().clear_bit()
                                        );

                    }

                    /// Start listening for an interrupt event
                    pub fn listen(&mut self, event: Event) {
                        match event {
                            Event::Rxne => self.usart.cr1.modify(|_, w| w.rxneie().set_bit()),
                            Event::Txe => self.usart.cr1.modify(|_, w| w.txeie().set_bit()),
                            Event::Idle => self.usart.cr1.modify(|_, w| w.idleie().set_bit()),
                            Event::Tc => self.usart.cr1.modify(|_,w| w.tcie().set_bit()),
                            Event::Peie => self.usart.cr1.modify(|_,w| w.peie().set_bit()),
                            Event::Eie => self.usart.cr3.modify(|_,w| w.eie().set_bit()),
                        }
                    }

                    /// Stop listening for an interrupt event
                    pub fn unlisten(&mut self, event: Event) {
                        match event {
                            Event::Rxne => self.usart.cr1.modify(|_, w| w.rxneie().clear_bit()),
                            Event::Txe => self.usart.cr1.modify(|_, w| w.txeie().clear_bit()),
                            Event::Idle => self.usart.cr1.modify(|_, w| w.idleie().clear_bit()),
                            Event::Tc => self.usart.cr1.modify(|_,w| w.tcie().clear_bit()),
                            Event::Peie => self.usart.cr1.modify(|_,w| w.peie().clear_bit()),
                            Event::Eie => self.usart.cr3.modify(|_,w| w.eie().clear_bit()),
                        }
                    }

                    /// Consume `tx` and `rx`, and return the underlying `Serial`.
                    ///
                    /// This is safe because `Serial` doesn't contain any state, and we can `steal`
                    /// the USART Peripheral member back because we consumed it in the `split`.
                    pub fn recover(pins: PINS) -> Serial<$USARTX, PINS> {
                        Serial { usart: unsafe { stm32l0x1::Peripherals::steal().$USARTX }, pins }
                    }

                    /// Split the `Serial` object into component Tx and Rx parts
                    ///
                    /// Note that once this is done, you cannot get the `Serial` object back! It is
                    /// gone.
                    pub fn split(self) -> (Tx<$USARTX>, Rx<$USARTX>) {
                        (
                            Tx {
                                _usart: PhantomData,
                            },
                            Rx {
                                _usart: PhantomData,
                            },
                            )
                    }

                }

                impl Read<u8> for Rx<$USARTX> {
                    type Error = Error;

                    fn read(&mut self) -> nb::Result<u8, Self::Error> {
                        // NOTE(unsafe) atomic read with no side effects
                        let isr = unsafe { (*$USARTX::ptr()).isr.read() };

                        Err(if isr.pe().bit_is_set() {
                            nb::Error::Other(Error::Parity)
                        } else if isr.fe().bit_is_set() {
                            nb::Error::Other(Error::Framing)
                        } else if isr.nf().bit_is_set() {
                            nb::Error::Other(Error::Noise)
                        } else if isr.ore().bit_is_set() {
                            nb::Error::Other(Error::Overrun)
                        } else if isr.rxne().bit_is_set() {
                            // NOTE(read_volatile) see `write_volatile` below
                            return Ok(unsafe {
                                ptr::read_volatile(&(*$USARTX::ptr()).rdr as *const _ as *const _)
                            });
                        } else {
                            nb::Error::WouldBlock
                        })
                    }
                }

                impl Write<u8> for Tx<$USARTX> {
                    type Error = Infallible;

                    fn flush(&mut self) -> nb::Result<(), Infallible> {
                        // NOTE(unsafe) atomic read with no side effects
                        let isr = unsafe { (*$USARTX::ptr()).isr.read() };

                        if isr.tc().bit_is_set() {
                            Ok(())
                        } else {
                            Err(nb::Error::WouldBlock)
                        }
                    }

                    fn write(&mut self, byte: u8) -> nb::Result<(), Infallible> {
                        // NOTE(unsafe) atomic read with no side effects
                        let isr = unsafe { (*$USARTX::ptr()).isr.read() };

                        if isr.txe().bit_is_set() {
                            // NOTE(unsafe) atomic write to stateless register
                            // NOTE(write_volatile) 8-bit write that's not possible through the svd2rust API
                            unsafe {
                                ptr::write_volatile(&(*$USARTX::ptr()).tdr as *const _ as *mut _, byte)
                            }
                            Ok(())
                        } else {
                            Err(nb::Error::WouldBlock)
                        }
                    }
                }

                impl Tx<$USARTX> {
                    /// Write the contents of the slice out to the USART
                    pub fn write_all(&mut self, bytes: &[u8]) -> nb::Result<(), Infallible> {
                        for b in bytes.iter() {
                            block!(self.write(*b)).unwrap();
                        }
                        Ok(())
                    }
                }

            }
        )+
    }
}

hal! {
    USART2: (usart2, APB1, apb1, usart2en, usart2sel),
}

/// LPUART1 module
pub mod lpuart1 {
    use super::*;

    /// Initialize the USART peripheral for RS485
    pub fn rs485<TX, RX, DE>(
        mut usart: LPUART1,
        pins: (TX, RX, DE),
        baud_rate: Bps,
        clk_src: USARTClkSource,
        clk_ctx: &ClockContext,
        apb: &mut APB1,
        ccipr: &mut CCIPR,
    ) -> Serial<LPUART1, (TX, RX, DE)>
    where
        TX: TxPin<LPUART1>,
        RX: RxPin<LPUART1>,
        DE: RtsDePin<LPUART1>,
    {
        let clk_f = match clk_src {
            USARTClkSource::PCLK => clk_ctx.apb1().0,
            USARTClkSource::SYSCLK => clk_ctx.sysclk().0,
            USARTClkSource::HSI16 => clk_ctx.hsi16().expect("HSI16 clk not enabled").0,
            USARTClkSource::LSE => clk_ctx.lse().expect("LSE not enabled").0,
        };

        // From 25.4.10 RS232 Hardware flow control and RS485 Driver Enable:
        //     The assertion time is the time between the activation of the DE signal
        //     and the beginning of the START bit. It is programmed using the DEAT
        //     [4:0] bit fields in the CR1 control register. The de-assertion time is
        //     the time between the end of the last stop bit, in a transmitted message,
        //     and the de-activation of the DE signal. It is programmed using the DEDT
        //     [4:0] bit fields in the CR1 control register.
        // For now, pick a period equal to half a bit

        // Do not set a 0-clock period; do not set a period greater than 5 bits
        let clks_per_half_bit: u8 = min(max(clk_f / (baud_rate.0 * 8) / 2, 31) as u8, 1);
        let clks_per_half_bit = clks_per_half_bit & 0b1_1111;

        usart.cr1.modify(|_, w| {
            w.deat()
                .bits(clks_per_half_bit)
                .dedt()
                .bits(clks_per_half_bit)
        });

        usart.cr3.modify(|_, w| w.dem().set_bit());

        <Serial<LPUART1, (TX, RX, DE)>>::configure(
            &mut usart, baud_rate, clk_src, clk_ctx, apb, ccipr,
        );

        Serial { usart, pins }
    }

    /// Create a LPUART1 peripheral to provide 8N1 asynchronous serial communication
    /// with an oversampling rate of 16.
    pub fn rs232<TX, RX>(
        mut usart: LPUART1,
        pins: (TX, RX),
        baud_rate: Bps,
        clk_src: USARTClkSource,
        clk_ctx: &ClockContext,
        apb: &mut APB1,
        ccipr: &mut CCIPR,
    ) -> Serial<LPUART1, (TX, RX)>
    where
        TX: TxPin<LPUART1>,
        RX: RxPin<LPUART1>,
    {
        <Serial<LPUART1, (TX, RX)>>::configure(&mut usart, baud_rate, clk_src, clk_ctx, apb, ccipr);

        Serial { usart, pins }
    }

    impl<PINS> Serial<LPUART1, PINS> {
        fn configure(
            usart: &mut LPUART1,
            baud_rate: Bps,
            clk_src: USARTClkSource,
            clk_ctx: &ClockContext,
            apb: &mut APB1,
            ccipr: &mut CCIPR,
        ) {
            apb.enr().modify(|_, w| w.lpuart1en().set_bit());
            while apb.enr().read().lpuart1en().bit_is_clear() {}

            let (clk_f, sel_bits) = match clk_src {
                USARTClkSource::PCLK => (clk_ctx.apb1().0, 0b00),
                USARTClkSource::SYSCLK => (clk_ctx.sysclk().0, 0b01),
                USARTClkSource::HSI16 => (clk_ctx.hsi16().expect("HSI16 clk not enabled").0, 0b10),
                USARTClkSource::LSE => (clk_ctx.lse().expect("LSE not enabled").0, 0b11),
            };

            usart.cr1.modify(|_, w| w.ue().clear_bit()); // disable the uart

            // From 25.4.2 "Character Transmission Procedure" and 25.4.3 "Character
            // Reception Procedure"

            // 1. Program the M bits in LPUART_CR1 to define the word length.
            usart.cr1.modify(|_, w| w.m1().clear_bit().m0().clear_bit()); // 8-bit word length

            // 2. Select the desired baud rate using the baud rate register LPUART_BRR

            // From 25.4.4 "fck must be in the range [3 x baud rate, 4096 x baud rate]."
            if clk_f < 3 * baud_rate.0 || clk_f > 4096 * baud_rate.0 {
                panic!("bad input clock rate");
            }

            let brr = clk_f * 256 / baud_rate.0; // 25.4.4 LPUART baud rate generation

            if brr < 0x300 {
                // from 25.4.4 "It is forbidden to write values less than 0x300
                // in the LPUART_BRR register."
                panic!("bad BRR");
            }
            usart.brr.write(|w| unsafe { w.bits(brr) });

            // 3. Program the number of stop bits in LPUART_CR2.
            usart.cr2.modify(|_, w| w.stop().bits(0b00)); // 1 stop bit

            // No CR3 configuration required

            ccipr.inner().modify(|_, w| w.lpuart1sel().bits(sel_bits));

            // 4. Enable the LPUART by writing the UE bit in LPUART_CR1 register to 1.
            usart.cr1.modify(|_, w| w.ue().set_bit()); // __HAL_UART_ENABLE in HAL_UART_Init

            // 5. Select DMA enable (DMAR) in LPUART_CR3 if multibuffer communication is to
            //    take place. Configure the DMA register as explained in multibuffer
            //    communication.
            // --> n/a for now

            // For TX:
            // 6. Set the TE bit in LPUART_CR1 to send an idle frame as first transmission.
            // For RX:
            // 6. Set the RE bit LPUART_CR1. This enables the receiver which begins searching for a start bit.
            usart.cr1.modify(
                |_, w| {
                    w.pce()
                        .clear_bit() // parity control disabled
                        .te()
                        .set_bit() // enable tx
                        .re()
                        .set_bit()
                }, // enable rx
            );

            while usart.isr.read().teack().bit_is_clear() {} // UART_CheckIdleState in HAL_UART_Init
            while usart.isr.read().reack().bit_is_clear() {}

            usart
                .cr3
                .modify(|_, w| w.rtse().clear_bit().ctse().clear_bit()); // no hardware flow control

            usart.cr3.modify(|_, w| w.hdsel().clear_bit());
        }

        /// Starts listening for an interrupt event
        pub fn listen(&mut self, event: Event) {
            match event {
                Event::Rxne => self.usart.cr1.modify(|_, w| w.rxneie().set_bit()),
                Event::Txe => self.usart.cr1.modify(|_, w| w.txeie().set_bit()),
                Event::Idle => self.usart.cr1.modify(|_, w| w.idleie().set_bit()),
                Event::Tc => self.usart.cr1.modify(|_, w| w.tcie().set_bit()),
                Event::Peie => self.usart.cr1.modify(|_, w| w.peie().set_bit()),
                Event::Eie => self.usart.cr3.modify(|_, w| w.eie().set_bit()),
            }
        }

        /// Starts listening for an interrupt event
        pub fn unlisten(&mut self, event: Event) {
            match event {
                Event::Rxne => self.usart.cr1.modify(|_, w| w.rxneie().clear_bit()),
                Event::Txe => self.usart.cr1.modify(|_, w| w.txeie().clear_bit()),
                Event::Idle => self.usart.cr1.modify(|_, w| w.idleie().clear_bit()),
                Event::Tc => self.usart.cr1.modify(|_, w| w.tcie().clear_bit()),
                Event::Peie => self.usart.cr1.modify(|_, w| w.peie().clear_bit()),
                Event::Eie => self.usart.cr3.modify(|_, w| w.eie().clear_bit()),
            }
        }

        /// Split the `Serial` object into component Tx and Rx parts
        ///
        /// Note that once this is done, you cannot get the `Serial` object back! It is
        /// gone.
        pub fn split(self) -> (Tx<LPUART1>, Rx<LPUART1>) {
            (
                Tx {
                    _usart: PhantomData,
                },
                Rx {
                    _usart: PhantomData,
                },
            )
        }
    }

    impl Read<u8> for Rx<LPUART1> {
        type Error = Error;

        fn read(&mut self) -> nb::Result<u8, Self::Error> {
            // NOTE(unsafe) atomic read with no side effects
            let isr = unsafe { (*LPUART1::ptr()).isr.read() };

            Err(if isr.pe().bit_is_set() {
                nb::Error::Other(Error::Parity)
            } else if isr.fe().bit_is_set() {
                nb::Error::Other(Error::Framing)
            } else if isr.nf().bit_is_set() {
                nb::Error::Other(Error::Noise)
            } else if isr.ore().bit_is_set() {
                nb::Error::Other(Error::Overrun)
            } else if isr.rxne().bit_is_set() {
                // NOTE(read_volatile) see `write_volatile` below
                return Ok(unsafe {
                    ptr::read_volatile(&(*LPUART1::ptr()).rdr as *const _ as *const _)
                });
            } else {
                nb::Error::WouldBlock
            })
        }
    }

    impl Write<u8> for Tx<LPUART1> {
        type Error = Infallible;

        fn flush(&mut self) -> nb::Result<(), Infallible> {
            // NOTE(unsafe) atomic read with no side effects
            let isr = unsafe { (*LPUART1::ptr()).isr.read() };

            if isr.tc().bit_is_set() {
                Ok(())
            } else {
                Err(nb::Error::WouldBlock)
            }
        }

        fn write(&mut self, byte: u8) -> nb::Result<(), Infallible> {
            // NOTE(unsafe) atomic read with no side effects
            let isr = unsafe { (*LPUART1::ptr()).isr.read() };

            if isr.txe().bit_is_set() {
                // NOTE(unsafe) atomic write to stateless register
                // NOTE(write_volatile) 8-bit write that's not possible through the svd2rust API
                unsafe { ptr::write_volatile(&(*LPUART1::ptr()).tdr as *const _ as *mut _, byte) }
                Ok(())
            } else {
                Err(nb::Error::WouldBlock)
            }
        }
    }

    impl Tx<LPUART1> {
        /// Write the contents of the slice out to the USART
        pub fn write_all(&mut self, bytes: &[u8]) -> nb::Result<(), Infallible> {
            for b in bytes.iter() {
                block!(self.write(*b)).unwrap();
            }
            Ok(())
        }
    }

    #[derive(Debug)]
    /// The event which activates the WUF (wakeup from Stop mode flag)
    pub enum SerialWakeSource {
        /// WUF active on address match (as defined by ADD\[7:0\] and ADDM7)
        AddrMatch {
            /// Address of the USART node
            add: u8,
            /// 7-bit Address Detection / 4-bit Address Detection
            addm7: bool,
        },
        /// WuF active on Start bit detection
        StartBit,
        /// WUF active on RXNE.
        Rxne,
    }
}
