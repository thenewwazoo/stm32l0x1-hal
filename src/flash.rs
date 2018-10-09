//! Flash memory

use common::Constrain;
use power::{self, Power};
use stm32l0x1::{flash, FLASH};
use time::Hertz;

impl Constrain<Flash> for FLASH {
    fn constrain(self) -> Flash {
        Flash { acr: ACR(()) }
    }
}

pub struct Flash {
    acr: ACR,
}

impl Flash {
    pub fn set_latency<VDD, VCORE, RTC>(&mut self, sysclk: Hertz, _pwr: &Power<VDD, VCORE, RTC>)
    where
        VCORE: Latency,
    {
        VCORE::latency(sysclk).set(&mut self.acr);
    }

    pub fn get_latency(&mut self) -> FlashLatency {
        match self.acr.acr().read().latency().bit() {
            true => FlashLatency::_1_Clk,
            false => FlashLatency::_0_Clk,
        }
    }
}

/// Couples the necessary flash latency to the VCore power range of the cpu
pub trait Latency {
    /// Taken from fig. 11 "Performance versus Vdd and Vcore range"
    fn latency(f: Hertz) -> FlashLatency;
}

impl Latency for power::VCoreRange1 {
    fn latency(f: Hertz) -> FlashLatency {
        if f.0 > 16_000_000 {
            FlashLatency::_1_Clk
        } else {
            FlashLatency::_0_Clk
        }
    }
}

impl Latency for power::VCoreRange2 {
    fn latency(f: Hertz) -> FlashLatency {
        if f.0 > 8_000_000 {
            FlashLatency::_1_Clk
        } else {
            FlashLatency::_0_Clk
        }
    }
}

impl Latency for power::VCoreRange3 {
    fn latency(_: Hertz) -> FlashLatency {
        FlashLatency::_0_Clk
    }
}

#[allow(non_camel_case_types)]
pub enum FlashLatency {
    _1_Clk,
    _0_Clk,
}

impl FlashLatency {
    pub fn set(&self, acr: &mut ACR) {
        match self {
            FlashLatency::_1_Clk => acr.flash_latency_1(),
            FlashLatency::_0_Clk => acr.flash_latency_0(),
        };
    }
}

/// Access control register
pub struct ACR(());

impl ACR {
    /// Direct access to FLASH_ACR
    pub(crate) fn acr(&mut self) -> &flash::ACR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*FLASH::ptr()).acr }
    }

    /// Set the flash latency to 1 clock cycle
    pub(crate) fn flash_latency_1(&mut self) -> FlashLatency {
        self.acr().modify(|_, w| w.latency().set_bit());
        while self.acr().read().latency().bit_is_clear() {}
        FlashLatency::_1_Clk
    }

    /// Set the flash latency to 0 clock cycles
    pub(crate) fn flash_latency_0(&mut self) -> FlashLatency {
        self.acr().modify(|_, w| w.latency().clear_bit());
        while self.acr().read().latency().bit_is_set() {}
        FlashLatency::_0_Clk
    }
}
