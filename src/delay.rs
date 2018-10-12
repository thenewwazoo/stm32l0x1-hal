//! Delays

use cast::u32;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::SYST;

use hal::blocking::delay::{DelayMs, DelayUs};
use rcc::ClockContext;

/// System timer (SysTick) as a delay provider
pub struct Delay<'c> {
    /// Reference to the (frozen) clocks
    clocks: &'c ClockContext,
    /// The SYST peripheral
    syst: SYST,
}

impl Delay {
    /// Configures the system timer (SysTick) as a delay provider by owning it. Note that this will
    /// not permit using the SYST exception handler for tick counting!
    pub fn new(mut syst: SYST, clocks: &ClockContext) -> Self {
        syst.disable_interrupt();
        syst.set_clock_source(SystClkSource::Core);

        Delay { syst, clocks }
    }

    /// Releases the system timer (SysTick) resource
    pub fn free(self) -> SYST {
        self.syst
    }
}

impl DelayMs<u32> for Delay {
    fn delay_ms(&mut self, ms: u32) {
        self.delay_us(ms * 1_000);
    }
}

impl DelayMs<u16> for Delay {
    fn delay_ms(&mut self, ms: u16) {
        self.delay_ms(u32(ms));
    }
}

impl DelayMs<u8> for Delay {
    fn delay_ms(&mut self, ms: u8) {
        self.delay_ms(u32(ms));
    }
}

impl DelayUs<u32> for Delay {
    fn delay_us(&mut self, us: u32) {
        // The SysTick Reload Value register supports values between 1 and 0x00FF_FFFF (24 bits).
        const MAX_RVR: u32 =  0x00FF_FFFF;

        let mut total_rvr = us * (self.clocks.sysclk().0 / 1_000_000);

        while total_rvr != 0 {
            let current_rvr = if total_rvr <= MAX_RVR {
                total_rvr
            } else {
                MAX_RVR
            };

            self.syst.set_reload(current_rvr);
            self.syst.clear_current();
            self.syst.enable_counter();

            // Update the tracking variable while we are waiting...
            total_rvr -= current_rvr;

            while !self.syst.has_wrapped() {}

            self.syst.disable_counter();
        }
    }
}

impl DelayUs<u16> for Delay {
    fn delay_us(&mut self, us: u16) {
        self.delay_us(u32(us))
    }
}

impl DelayUs<u8> for Delay {
    fn delay_us(&mut self, us: u8) {
        self.delay_us(u32(us))
    }
}
