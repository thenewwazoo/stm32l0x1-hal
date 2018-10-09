//! Serial

use core::marker::PhantomData;
use core::ptr;

use hal::prelude::*;
use hal::serial;
use nb;
use stm32l0x1::{LPUART1, USART2};

use rcc::clocking::USARTClkSource;
use rcc::ClockContext;
use rcc::{APB1, CCIPR};
use time::Bps;

#[cfg(any(feature = "STM32L011x3", feature = "STM32L011x4"))]
pub mod stm32l011x3_4;

#[cfg(any(feature = "STM32L031x4", feature = "STM32L031x6"))]
pub mod stm32l031x4_6;

/// Interrupt event
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

/// Serial error
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

// FIXME these should be "closed" traits
/// TX pin - DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait TxPin<USART> {}

/// RX pin - DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait RxPin<USART> {}

/// Serial abstraction
pub struct Serial<USART, PINS> {
    usart: USART,
    #[allow(dead_code)]
    pins: PINS,
}

/// Serial receiver
pub struct Rx<USART> {
    _usart: PhantomData<USART>,
}

/// Serial transmitter
pub struct Tx<USART> {
    _usart: PhantomData<USART>,
}

macro_rules! hal {
    ($(
        $USARTX:ident: (
            $usartX:ident,
            $APB:ident,
            $pclk: ident,
            $usartXen:ident,
            $usartXsel0:ident,
            $usartXsel1:ident),
    )+) => {
        $(

            impl<TX, RX> Serial<$USARTX, (TX, RX)> {

                /// Create a $USARTX peripheral to provide 8N1 asynchronous serial communication
                /// with an oversampling rate of 16.
                pub fn $usartX(
                    usart: $USARTX,
                    pins: (TX, RX),
                    baud_rate: Bps,
                    clk_src: USARTClkSource,
                    clk_ctx: &ClockContext,
                    apb: &mut $APB,
                    ccipr: &mut CCIPR,
                ) -> Self
                where TX: TxPin<$USARTX>,
                      RX: RxPin<$USARTX>,
                {
                    apb.enr().modify(|_, w| w.$usartXen().set_bit());
                    while apb.enr().read().$usartXen().bit_is_clear() {}

                    let (clk_f, sel_bit1, sel_bit0) = match clk_src {
                        USARTClkSource::PCLK   => (clk_ctx.$pclk().0,   false, false),
                        USARTClkSource::SYSCLK => (clk_ctx.sysclk().0,  false, true),
                        USARTClkSource::HSI16  => (clk_ctx.hsi16().expect("HSI16 clk not enabled").0, true, false),
                        USARTClkSource::LSE    => (clk_ctx.lse().expect("LSE not enabled").0,   true, true),
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
                    usart.cr2.modify(|_,w| unsafe { w.stop().bits(0b00) });          // 1 stop bit

                    // No CR3 configuration required

                    ccipr.inner().modify(|_,w| w.$usartXsel0().bit(sel_bit0).$usartXsel1().bit(sel_bit1));

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

                    Serial { usart, pins }

                }

                /// Starts listening for an interrupt event
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

                /// Starts listening for an interrupt event
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

            impl serial::Read<u8> for Rx<$USARTX> {
                type Error = Error;

                fn read(&mut self) -> nb::Result<u8, Error> {
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

            impl serial::Write<u8> for Tx<$USARTX> {
                type Error = !;

                fn flush(&mut self) -> nb::Result<(), !> {
                    // NOTE(unsafe) atomic read with no side effects
                    let isr = unsafe { (*$USARTX::ptr()).isr.read() };

                    if isr.tc().bit_is_set() {
                        Ok(())
                    } else {
                        Err(nb::Error::WouldBlock)
                    }
                }

                fn write(&mut self, byte: u8) -> nb::Result<(), !> {
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
                pub fn write_all(&mut self, bytes: &[u8]) -> nb::Result<(), !> {
                    for b in bytes.iter() {
                        block!(self.write(*b)).unwrap();
                    }
                    Ok(())
                }
            }
        )+
    }
}

hal! {
    USART2: (usart2, APB1, apb1, usart2en, usart2sel0, usart2sel1),
}

impl<TX, RX> Serial<LPUART1, (TX, RX)> {
    /// Create a LPUART1 peripheral to provide 8N1 asynchronous serial communication
    /// with an oversampling rate of 16.
    pub fn lpuart1(
        usart: LPUART1,
        pins: (TX, RX),
        baud_rate: Bps,
        clk_src: USARTClkSource,
        clk_ctx: &ClockContext,
        apb: &mut APB1,
        ccipr: &mut CCIPR,
    ) -> Self
    where
        TX: TxPin<LPUART1>,
        RX: RxPin<LPUART1>,
    {
        apb.enr().modify(|_, w| w.lpuart1en().set_bit());
        while apb.enr().read().lpuart1en().bit_is_clear() {}

        let (clk_f, sel_bit1, sel_bit0) = match clk_src {
            USARTClkSource::PCLK => (clk_ctx.apb1().0, false, false),
            USARTClkSource::SYSCLK => (clk_ctx.sysclk().0, false, true),
            USARTClkSource::HSI16 => (
                clk_ctx.hsi16().expect("HSI16 clk not enabled").0,
                true,
                false,
            ),
            USARTClkSource::LSE => (clk_ctx.lse().expect("LSE not enabled").0, true, true),
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
        usart.cr2.modify(|_, w| unsafe { w.stop().bits(0b00) }); // 1 stop bit

        // No CR3 configuration required

        ccipr
            .inner()
            .modify(|_, w| w.lpuart1sel0().bit(sel_bit0).lpuart1sel1().bit(sel_bit1));

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

        Serial { usart, pins }
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

impl serial::Read<u8> for Rx<LPUART1> {
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Error> {
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

impl serial::Write<u8> for Tx<LPUART1> {
    type Error = !;

    fn flush(&mut self) -> nb::Result<(), !> {
        // NOTE(unsafe) atomic read with no side effects
        let isr = unsafe { (*LPUART1::ptr()).isr.read() };

        if isr.tc().bit_is_set() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), !> {
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

#[derive(Debug)]
/// The event which activates the WUF (wakeup from Stop mode flag)
pub enum SerialWakeSource {
    /// WUF active on address match (as defined by ADD[7:0] and ADDM7)
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
