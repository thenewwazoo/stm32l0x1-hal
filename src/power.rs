//! Power control

use stm32l0x1::{pwr, PWR};

use common::Constrain;

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

    pub fn set_vcore_range(&mut self, vcore_range: &VCoreRange) -> Result<(), PowerError> {
        match vcore_range {
            VCoreRange::Range1 => match self.vdd_range {
                VDDRange::Low => Err(PowerError::LowVDD),
                _ => {
                    self.cr.set_vcore_range(vcore_range);
                    Ok(())
                }
            },
            _ => {
                self.cr.set_vcore_range(vcore_range);
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

    pub fn set_vcore_range(&mut self, vcore_range: &VCoreRange) {
        self.inner().modify(|_, w| unsafe {
            w.vos().bits(match vcore_range {
                VCoreRange::Range1 => 0b01,
                VCoreRange::Range2 => 0b10,
                VCoreRange::Range3 => 0b11,
            })
        });
    }

    pub fn get_vcore_range(&self) -> VCoreRange {
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
