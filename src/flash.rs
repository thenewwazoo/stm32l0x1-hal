//! Flash memory
//!
//! This currently implements basic chip startup-related functions. You will probably not need to
//! use this interface directly, as the Flash peripheral is mostly configured by `Rcc::freeze`

use core::ptr;

use common::Constrain;
use fhal::flash::{Locking, WriteErase};
use power::{self, Power};
use stm32l0x1::{flash, FLASH};
use time::Hertz;

mod private {
    /// The Sealed trait prevents other crates from implementing helper traits defined here
    pub trait Sealed {}

    impl Sealed for ::power::VCoreRange1 {}
    impl Sealed for ::power::VCoreRange2 {}
    impl Sealed for ::power::VCoreRange3 {}
}

impl Constrain<Flash> for FLASH {
    fn constrain(self) -> Flash {
        Flash {
            acr: ACR(()),
            sr: SR(()),
            pecr: PECR(()),
            pekeyr: PEKEYR(()),
            prgkeyr: PRGKEYR(()),
        }
    }
}

#[derive(Debug)]
/// Non-error flash read and write statuses
pub enum FlashStatus {
    /// Memory interface busy
    ///
    /// Write/erase operations are in progress.
    Busy,
    /// End of program
    ///
    /// This bit is set by hardware at the end of a write or erase operation when the operation has
    /// not been aborted.
    Done,
    /// Ready for read and write/erase operations
    Ready,
    /// High voltage is executing a write/erase operation in the NVM
    HVOngoing,
}

#[derive(Debug)]
/// Error states
pub enum FlashError {
    /// The data eeprom and FLASH_PECR register is locked
    Locked,
    /// Write protection error
    ///
    /// This bit is set by hardware when an address to be programmed or erased is write-protected.
    WriteProtect,
    /// Programming alignment error
    ///
    /// This bit is set by hardware when an alignment error has happened: the first word of a
    /// half-page operation is not aligned to a half-page, or one of the following words in a
    /// half-page operation does not belong to the same half-page as the first word.
    Alignment,
    /// Size error
    ///
    /// This bit is set by hardware when the size of data to program is not correct.
    Size,
    /// Read protection error
    ///
    /// This bit is set by hardware when the user tries to read an area protected by PcROP.
    ReadProtection,
    /// The write operation is attempting to write to a not-erased region
    ///
    /// This bit is set by hardware when a program in the Flash program or System Memory tries to
    /// overwrite a not-zero area.
    NotZero,
    /// A write/erase operation aborted to perform a fetch.
    ///
    /// This bit is set by hardware when a write/erase operation is aborted to perform a fetch.
    /// This is not a real error, but it is used to inform that the write/erase operation did not
    /// execute.
    FetchAbort,
    /// Some otherwise unknown error has happened or an unexpected status has arisen
    Unknown,
}

impl FlashError {
    /// Helper method to clear error bits raised
    pub fn clear(&self, sr: &mut SR) {
        match self {
            FlashError::WriteProtect => sr.inner().modify(|_, w| w.wrperr().set_bit()),
            FlashError::Alignment => sr.inner().modify(|_, w| w.pgaerr().set_bit()),
            FlashError::Size => sr.inner().modify(|_, w| w.sizerr().set_bit()),
            FlashError::ReadProtection => sr.inner().modify(|_, w| w.rderr().set_bit()),
            FlashError::NotZero => sr.inner().modify(|_, w| w.notzeroerr().set_bit()),
            FlashError::FetchAbort => sr.inner().modify(|_, w| w.fwwerr().set_bit()),
            _ => {}
        };
    }
}

/// A constrained FLASH peripheral
pub struct Flash {
    /// FLASH_ACR
    acr: ACR,
    /// FLASH_SR
    sr: SR,
    /// FLASH_PECR
    pecr: PECR,
    /// FLASH_PEKEYR
    pekeyr: PEKEYR,
    /// FLASH_PRGKEYR
    prgkeyr: PRGKEYR,
}

impl Flash {
    /// Sets the clock cycle latency for the flash peripheral based on power level and clock speed
    pub fn set_latency<VDD, VCORE, RTC>(&mut self, sysclk: Hertz, _pwr: &Power<VDD, VCORE, RTC>)
    where
        VCORE: Latency,
    {
        unsafe {
            VCORE::latency(sysclk).set(&mut self.acr);
        }
    }

    /// Retrieves the clock cycle latency based on current register settings
    pub fn get_latency(&mut self) -> FlashLatency {
        match self.acr.acr().read().latency().bit() {
            true => FlashLatency::_1_Clk,
            false => FlashLatency::_0_Clk,
        }
    }

    /// Immediately write `value` to `address`
    pub unsafe fn program_word_immediate(&mut self, address: usize, value: u32) {
        ptr::write_volatile(address as *mut u32, value);
    }
}

/// Couples the necessary flash latency to the VCore power range of the cpu
pub trait Latency: private::Sealed {
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
/// Flash latencies available for this chip
pub enum FlashLatency {
    /// Flash reads have a latency of 1 clock cycle
    _1_Clk,
    /// Flash reads have a latency of 0 clock cycles
    _0_Clk,
}

impl FlashLatency {
    /// Use the ACR register to set the flash value according to the variant
    ///
    /// This function is unsafe because it does not (cannot) check to see that the latency value
    /// being set is appropriate for the current VCore range and clock speed of the chip
    pub unsafe fn set(&self, acr: &mut ACR) {
        match self {
            FlashLatency::_1_Clk => acr.flash_latency_1(),
            FlashLatency::_0_Clk => acr.flash_latency_0(),
        };
    }
}

/// Opaque access control register
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

/// Status register
pub struct SR(());

impl SR {
    /// Retrieve a pointer to the SR register
    pub(crate) fn inner(&self) -> &flash::SR {
        unsafe { &(*FLASH::ptr()).sr }
    }

    /// Is the memory interface busy?
    pub fn is_busy(&self) -> bool {
        self.inner().read().bsy().bit_is_set()
    }
}

/// Program and erase control register
pub struct PECR(());

impl PECR {
    /// Retrieve a pointer to the PECR register
    fn inner() -> &'static flash::PECR {
        unsafe { &(*FLASH::ptr()).pecr }
    }

    /// Is is FLASH_PECR register locked?
    pub fn is_pecr_locked(&self) -> bool {
        Self::inner().read().pelock().bit_is_set()
    }

    /// Is program memory locked?
    pub fn is_prgmem_locked(&self) -> bool {
        Self::inner().read().prglock().bit_is_set()
    }

    /// Lock the PECR
    pub fn lock(&mut self) {
        Self::inner().modify(|_, w| w.pelock().set_bit());
    }

    /// Enable flash to be erased (requires that PECR is unlocked)
    pub fn enable_erase(&mut self) -> Result<(), FlashError> {
        if self.is_pecr_locked() {
            Err(FlashError::Locked)
        } else {
            Self::inner().modify(|_, w| w.erase().set_bit().prog().set_bit());
            Ok(())
        }
    }

    /// Disable writes to flash (requires that PECR is unlocked)
    pub fn disable_erase(&mut self) -> Result<(), FlashError> {
        if self.is_pecr_locked() {
            Err(FlashError::Locked)
        } else {
            Self::inner().modify(|_, w| w.erase().clear_bit().prog().clear_bit());
            Ok(())
        }
    }
}

/// PECR unlock key register
pub struct PEKEYR(());

impl PEKEYR {
    /// Retrieve a pointer to the PEKEYR register
    pub(crate) fn inner(&mut self) -> &flash::PEKEYR {
        unsafe { &(*FLASH::ptr()).pekeyr }
    }
}

/// Program and erase key register
pub struct PRGKEYR(());

impl PRGKEYR {
    /// Retrieve a pointer tot he PRGKEYR register
    pub(crate) fn inner(&mut self) -> &flash::PRGKEYR {
        unsafe { &(*FLASH::ptr()).prgkeyr }
    }
}

/// First key value to unlock data EEPROM and the PECR register (see 3.3.4)
const PEKEY1: u32 = 0x89ABCDEF;
/// Second key value to unlock data EEPROM and the PECR register (see 3.3.4)
const PEKEY2: u32 = 0x02030405;

/// First key value to unlock flash write (see 3.3.4)
const PRGKEY1: u32 = 0x8C9DAEBF;
/// Second key value to unlock flash write (see 3.3.4)
const PRGKEY2: u32 = 0x13141516;

impl Locking for Flash {
    type Error = FlashError;

    fn is_locked(&self) -> bool {
        self.pecr.is_prgmem_locked()
    }

    fn lock(&mut self) {
        self.pecr.lock();
    }

    fn unlock(&mut self) {
        if self.sr.is_busy() {
            panic!("cannot lock! flash is busy");
        }

        if self.pecr.is_pecr_locked() {
            self.pekeyr.inner().write(|w| unsafe { w.bits(PEKEY1) });
            self.pekeyr.inner().write(|w| unsafe { w.bits(PEKEY2) });

            if self.pecr.is_prgmem_locked() {
                self.prgkeyr.inner().write(|w| unsafe { w.bits(PRGKEY1) });
                self.prgkeyr.inner().write(|w| unsafe { w.bits(PRGKEY2) });
            }
        }
    }
}

impl WriteErase for Flash
where
    Flash: Locking,
{
    type Error = FlashError;
    type Status = FlashStatus;

    /// Return the current Flash status
    fn status(&self) -> Result<Self::Status, Self::Error> {
        if self.sr.is_busy() {
            Ok(FlashStatus::Busy)
        } else if self.sr.inner().read().eop().bit_is_set() {
            Ok(FlashStatus::Done)
        } else if self.sr.inner().read().ready().bit_is_set() {
            Ok(FlashStatus::Ready)
        } else if self.sr.inner().read().endhv().bit_is_clear() {
            Ok(FlashStatus::HVOngoing)
        } else if self.sr.inner().read().wrperr().bit_is_set() {
            Err(FlashError::WriteProtect)
        } else if self.sr.inner().read().pgaerr().bit_is_set() {
            Err(FlashError::Alignment)
        } else if self.sr.inner().read().sizerr().bit_is_set() {
            Err(FlashError::Size)
        } else if self.sr.inner().read().rderr().bit_is_set() {
            Err(FlashError::ReadProtection)
        } else if self.sr.inner().read().notzeroerr().bit_is_set() {
            Err(FlashError::NotZero)
        } else if self.sr.inner().read().fwwerr().bit_is_set() {
            Err(FlashError::FetchAbort)
        } else {
            Err(FlashError::Unknown)
        }
    }

    fn erase_page(&mut self, address: usize) -> Result<(), FlashError> {
        self.pecr.enable_erase()?;
        unsafe {
            ptr::write_volatile(address as *mut u32, 0);
        }

        while self.sr.is_busy() {}

        let result = match self.status() {
            Ok(_) => Ok(()),
            Err(e) => Err(e),
        };

        self.pecr.disable_erase()?;
        result
    }

    fn program_word(&mut self, address: usize, value: u32) -> Result<(), FlashError> {
        self.erase_page(address)?;
        unsafe { self.program_word_immediate(address, value) };
        while self.sr.is_busy() {}
        match self.status() {
            Ok(_) => Ok(()),
            Err(e) => Err(e),
        }
    }
}
