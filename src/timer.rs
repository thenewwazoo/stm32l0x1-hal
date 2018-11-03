//! Timers

use void::Void;

use hal::timer::*;
use cast::{u16, u32};

use time::Hertz;
use rcc::{ClockContext, APB1, APB2};
use rcc::clocking::LPTimerClkSource;
use stm32l0x1::{TIM2, TIM21};
#[cfg(any(feature = "STM32L031x4", feature = "STM32L031x6"))]
use stm32l0x1::TIM22;

/// 16-bit timer
pub struct Timer<'c, TIM> {
    /// The underlying timer peripheral
    timer: TIM,
    /// The timer's period
    timeout: Hertz,
    /// A handle to the clock configuration
    clk_ctx: &'c ClockContext,
}

/// Timer events that can be subscribed to
pub enum Event {
    /// The timer has expired
    Timeout,
}

macro_rules! impl_timer {
    ($($TIMX:ident: ($timX:ident, $APB:ident, $apb:ident, $enr_bit:ident, $rstr_bit:ident),)+) => {
        $(
            impl<'c> Periodic for Timer<'c, $TIMX> {}

            impl<'c> Cancel for Timer<'c, $TIMX> {
                type Error = Void;

                fn cancel(&mut self) -> Result<(), Self::Error> {
                    // disable the timer
                    self.timer.cr1.modify(|_, w| w.cen().clear_bit());
                    Ok(())
                }
            }

            impl<'c> CountDown for Timer<'c, $TIMX> {
                type Time = Hertz;

                fn start<T>(&mut self, timeout: T)
                where
                    T: Into<Self::Time>
                {
                    // disable the timer
                    self.timer.cr1.modify(|_, w| w.cen().clear_bit());
                    // reset its counter
                    self.timer.cnt.reset();

                    self.timeout = timeout.into();

                    // timer clock is multiplied by 2 is APBx presc is != 1
                    let timx_prsc = if self.clk_ctx.hclk_fclk() == self.clk_ctx.$apb() { 1 } else { 2 };

                    // timer clock ticks per timeout cycle
                    let ticks = self.clk_ctx.$apb().0 * timx_prsc / self.timeout.0;

                    // prescale the timer clock to account for multiples of the 16-bit counter
                    // size
                    let psc = u16((ticks - 1) / (1 << 16)).unwrap();
                    self.timer.psc.write(|w| unsafe { w.psc().bits(psc) });

                    // now set the auto-reload value
                    let arr = u16(ticks / u32(psc + 1)).unwrap();
                    self.timer.arr.write(|w| unsafe { w.bits(u32(arr)) });

                    // Trigger an update event to load the prescaler value to the clock
                    self.timer.egr.write(|w| w.ug().set_bit());
                    // The above line raises an update event which will indicate
                    // that the timer is already finnished. Since this is not the case,
                    // it should be cleared
                    self.reset_overflow();

                    // start counter
                    self.timer.cr1.modify(|_, w| w.cen().set_bit());
                }

                fn wait(&mut self) -> nb::Result<(), Void> {
                    match self.timer.sr.read().uif().bit_is_clear() {
                        true => Err(nb::Error::WouldBlock),
                        false => {
                            self.reset_overflow();
                            Ok(())
                        }
                    }
                }
            }

            impl<'c> Timer<'c, $TIMX> {
                /// Instantiate a new timer
                pub fn $timX<T: Into<Hertz>>(timer: $TIMX, clk_ctx: &'c ClockContext, timeout: T, apb: &mut $APB) -> Self {
                    // enable and reset peripheral to a clean slate state
                    apb.enr().modify(|_, w| w.$enr_bit().set_bit());
                    apb.rstr().modify(|_, w| w.$rstr_bit().set_bit());
                    apb.rstr().modify(|_, w| w.$rstr_bit().clear_bit());

                    Timer { timer, timeout: timeout.into(), clk_ctx }
                }

                /// Starts listening for an `event`
                pub fn subscribe(&mut self, event: Event) {
                    match event {
                        Event::Timeout => self.timer.dier.write(|w| w.uie().set_bit())
                    }
                }

                /// Stops listening for an `event`
                pub fn unsubscribe(&mut self, event: Event) {
                    match event {
                        Event::Timeout => self.timer.dier.write(|w| w.uie().clear_bit())
                    }
                }

                #[inline(always)]
                /// Resets SR's UIF register to clear status of overflow.
                ///
                /// Unless reset is done, Interrupt handler is going to be continiously called.
                pub fn reset_overflow(&mut self) {
                    self.timer.sr.modify(|_, w| w.uif().clear_bit());
                }

                /// Pauses timer and releases the TIM peripheral
                pub fn free(self) -> $TIMX {
                    self.timer.cr1.modify(|_, w| w.cen().clear_bit());
                    self.timer
                }
            }
        )+
    }
}

impl_timer!{
    TIM2: (tim2, APB1, apb1, tim2en, tim2rst),
    TIM21: (tim21, APB2, apb2, tim21en, tim21rst),
}

#[cfg(any(feature = "STM32L031x4", feature = "STM32L031x6"))]
impl_timer!{
    TIM22: (tim22, APB2, apb2, tim22en, tim22rst),
}
