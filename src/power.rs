//! Power control

#![allow(unknown_lints)]
#![allow(clippy)]

use stm32l0x1::{pwr, PWR};

use common::Constrain;
use rcc::APB1;

pub enum PowerError {
    LowVDD,
}

#[derive(PartialEq)]
/// Figure 11. Performance versus VDD and VCORE range
pub enum VDDRange {
    /// 1.71 V - 3.6 V
    High,
    /// 1.65 V - 3.6 V
    Low,
}

#[derive(PartialEq)]
/// 6.1.4 Dynamic voltage scaling management
pub enum VCoreRange {
    /// Range 1 is the "high performance" range. Vcore = 1.8V
    Range1,
    /// Range 2 is the "medium performance" range. Vcore = 1.5V
    Range2,
    /// Range 3 is the "low power" range. Vcore = 1.2V
    Range3,
}

/// Constrained Power control module
pub struct Power {
    /// Control register
    pub cr: CR, // TODO make private
    /// Control and Status Register
    pub csr: CSR, // TODO make private
    /// MCU VDD voltage range. This is external to the chip.
    vdd_range: VDDRange,
}

impl Constrain<Power> for PWR {
    fn constrain(self) -> Power {
        Power {
            cr: CR(()),
            csr: CSR(()),
            vdd_range: VDDRange::High,
        }
    }
}

impl Power {
    pub fn set_vddrange(&mut self, vdd_range: VDDRange) {
        self.vdd_range = vdd_range;
    }

    pub fn set_vcore_range(&mut self, vcore_range: VCoreRange, apb1: &mut APB1) -> Result<(), PowerError> {
        match vcore_range {
            VCoreRange::Range1 => match self.vdd_range {
                VDDRange::Low => Err(PowerError::LowVDD),
                _ => {
                    self.cr.set_vcore_range(self.csr.csr(), vcore_range, apb1);
                    Ok(())
                }
            },
            _ => {
                self.cr.set_vcore_range(self.csr.csr(), vcore_range, apb1);
                Ok(())
            }
        }
    }

    pub fn vcore_range(&self) -> VCoreRange {
        self.cr.get_vcore_range()
    }
}

pub struct CR(());

impl CR {
    /// Return a raw pointer to the CR register
    #[inline]
    pub fn inner(&mut self) -> &pwr::CR {
        unsafe { &(*PWR::ptr()).cr }
    }

    /// Set the VCore range (see 6.1.4 Dynamic voltage scaling management)
    pub fn set_vcore_range(&mut self, csr: &pwr::CSR, vcore_range: VCoreRange, apb1: &mut APB1) {
        // Procedure from sec 6.1.5 Dynamic voltage scaling configuration

        apb1.enr().modify(|_, w| w.pwren().set_bit());
        while !apb1.enr().read().pwren().bit_is_set() {}

        // 1. Check VDD to identify which ranges are allowed (see Figure 11: Performance versus VDD
        //    and VCORE range).
        //
        // This is performed by the caller. We assume that the requested range is valid.

        // 2. Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0.
        while csr.read().vosf().bit_is_set() {}

        self.inner().modify(|_, w| unsafe {
            // 3. Configure the voltage scaling range by setting the VOS[1:0] bits in the PWR_CR
            //    register.
            w.vos().bits(match vcore_range {
                VCoreRange::Range1 => 0b01,
                VCoreRange::Range2 => 0b10,
                VCoreRange::Range3 => 0b11,
            })
        });

        // 4. Poll VOSF bit of in PWR_CSR register. Wait until it is reset to 0.
        while csr.read().vosf().bit_is_set() {}

        apb1.enr().modify(|_, w| w.pwren().clear_bit());
        while apb1.enr().read().pwren().bit_is_set() {}
    }

    pub fn get_vcore_range(&self) -> VCoreRange {
        // self.inner() takes &mut self, but this is a side-effect-free read, so we don't want to
        // make this function take &mut self as an argument. Instead, we will use the pointer
        // directly.
        let inner = unsafe { &(*PWR::ptr()).cr };
        match inner.read().vos().bits() {
            0b01 => VCoreRange::Range1,
            0b10 => VCoreRange::Range2,
            0b11 => VCoreRange::Range3,
            _ => panic!("invalid VOS"),
        }
    }
}

pub struct CSR(());

impl CSR {
    /// Return a raw pointer to the CSR register
    #[inline]
    pub fn csr(&mut self) -> &pwr::CSR {
        unsafe { &(*PWR::ptr()).csr }
    }
}
