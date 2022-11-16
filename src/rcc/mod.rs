//! Reset and Clock Control peripheral
//!
//! This module contains a partial abstraction over the RCC peripheral, as well as types and traits
//! related to the various clocks on the chip. The usual pattern here is to mutate clock settings
//! as you wish them to be, freeze the clocks, and then pass references to the `ClockContext` to
//! other peripherals that need frequency information.
//!
//! ```
//! use stm32l0x1_hal;
//!
//! let d = stm32l0x1_hal::stm32l0x1::Peripherals::take().unwrap();
//! let mut flash = d.FLASH.constrain();
//! let mut pwr = d.POWER.constrain();
//! let mut rcc = d.RCC.constrain();
//!
//! {
//!     let cfgr: &mut stm32l0x1_hal::rcc::CFGR = rcc.cfgr.config().unwrap(); // for convenience o
//!
//!     cfgr.msi.enable(); // enable the MSI clock (it's enabled by default)
//!     cfgr.msi.set_freq(clocking::MsiFreq::Hz_131_072); // set it to 131 kHz
//!     cfgr.hsi16.enable();             // turn on the 16 MHz internal RC
//!     cfgr.hclk_fclk = Hertz(131_072); // the HCLK bus should not be prescaled
//!     cfgr.pclk1 = Hertz(131_072);     // same for the APB1 clock
//!     cfgr.pclk2 = Hertz(131_072);     // and the same for the APB2 clock
//!     cfgr.sysclk_src = stm32l0x1_hal::rcc::clocking::SysClkSource::MSI;
//!     // use the MSI to clock the CPU
//! }
//!
//! rcc.freeze(&mut flash, &mut pwr);
//!
//! let clk_ctx = rcc.cfgr.context().unwrap(); // get a clk_ctx handle to pass around
//! ```

use core::mem::replace;

use crate::common::Constrain;
use crate::flash;
use crate::power;
use crate::stm32l0x1::{rcc, RCC};
use crate::time::Hertz;

pub mod clocking;

impl Constrain<Rcc> for RCC {
    fn constrain(self) -> Rcc {
        Rcc {
            ahb: AHB(()),
            apb1: APB1(()),
            apb2: APB2(()),
            iop: IOP(()),
            ccipr: CCIPR(()),
            cr: CR(()),
            csr: CSR(()),
            icscr: ICSCR(()),
            cfgr: ClockConfig::Open(CFGR {
                hclk_fclk: Hertz(2_097_000),
                pclk1: Hertz(2_097_000),
                pclk2: Hertz(2_097_000),
                sysclk_src: clocking::SysClkSource::MSI,
                lsi: clocking::LowSpeedInternalRC::new(),
                msi: clocking::MediumSpeedInternalRC::new(true, clocking::MsiFreq::Hz_2_097_000),
                hsi16: clocking::HighSpeedInternal16RC::new(),
                //pll: PLL,
                lse: None,
                //hse: Option<HighSpeedExternalOSC>,
            }),
        }
    }
}

/// Returns an already-frozen default `Rcc` config
///
/// This config matches the default start-up configuration of the MCU. This is an easy way to elide
/// the code space necessary to support more flexible configuration capability.
pub fn as_default(_rcc: RCC) -> Rcc {
    Rcc {
        ahb: AHB(()),
        apb1: APB1(()),
        apb2: APB2(()),
        iop: IOP(()),
        ccipr: CCIPR(()),
        cr: CR(()),
        csr: CSR(()),
        icscr: ICSCR(()),
        cfgr: ClockConfig::Frozen(ClockContext {
            sysclk: Hertz(2_097_000),
            /// HCLK/FCLK
            hclk: Hertz(2_097_000),
            /// PCLK1 to APB1 peripherals
            pclk1: Hertz(2_097_000),
            /// PCLK2 to APB2 peripherals
            pclk2: Hertz(2_097_000),
            /// Low-speed internal RC clock speed
            lsiclk: None,
            /// Medium-speed internal RC clock speed
            msiclk: Some(Hertz(2_097_000)),
            /// High-speed internal 16 MHz clock speed (optionally /4)
            hsi16clk: None,
            /// Low-speed external 32kHz clock speed
            lseclk: None,
        }),
    }
}

/// Struct representing the RCC peripheral
pub struct Rcc {
    /// AMBA High-performance Bus (AHB) registers.
    pub ahb: AHB,
    /// APB1 peripheral registers.
    pub apb1: APB1,
    /// APB2 peripheral registers.
    pub apb2: APB2,
    /// GPIO port configuration
    pub iop: IOP,
    /// Peripherals independent clock configuration register
    pub ccipr: CCIPR,
    /// Clock control register
    pub cr: CR,
    /// Control/status register
    pub csr: CSR,
    /// HW clock configuration.
    pub cfgr: ClockConfig,
    /// Internal clock sources calibration register
    pub icscr: ICSCR,
}

impl Rcc {
    /// "Freeze" the clock configuration
    ///
    /// This is an operation upon which _most_ other operations will depend. This provides a
    /// `ClockContext` that provides clocking information to other peripherals, and which is used
    /// to calculate bus timing info.
    ///
    /// This function also manipulates the `Flash` and `Power` settings of the chip to achieve the
    /// requested clock speeds.
    ///
    /// Note that this `mem::replace()`s the contents of `self.cfgr`. You cannot change the clock
    /// configuration after this function is called, and future calls to `rcc.cfgr.config()` will
    /// return an `Err`.
    pub fn freeze<VDD, VCORE, RTC>(
        &mut self,
        flash: &mut flash::Flash,
        pwr: &mut power::Power<VDD, VCORE, RTC>,
    ) where
        VCORE: power::FreqLimit + flash::Latency + power::Vos,
    {
        // Enable the system configuration clock so our changes will take effect.
        self.apb2.enr().modify(|_, w| w.syscfgen().set_bit());
        while !self.apb2.enr().read().syscfgen().bit_is_set() {}

        // Configure the clock to which we will switch early. We will then do some ancillary tasks
        // and switch to it before dealing with any other clocks. This assures that we can turn off
        // the current sysclk source if necessary.
        let sysclk = match self.cfgr.config().unwrap().sysclk_src {
            clocking::SysClkSource::MSI => self
                .cfgr
                .config()
                .unwrap()
                .msi
                .configure(&mut self.icscr, &mut self.cr),
            clocking::SysClkSource::HSI16 => self
                .cfgr
                .config()
                .unwrap()
                .hsi16
                .configure(&mut self.icscr, &mut self.cr), //clocking::SysClkSource::HSE => ..=
                                                           //clocking::SysClkSource::PLLCLK => ..=
        }
        .expect("selected SYSCLK source is not enabled!");

        let max_f = VCORE::max_freq();
        if sysclk > max_f {
            panic!("sysclk too high for vcore");
        }

        let (hpre_bits, hpre_ratio) = match sysclk.0 / self.cfgr.config().unwrap().hclk_fclk.0 {
            0 => unreachable!(),
            1 => (0b0000, 1),
            2 => (0b1000, 2),
            3..=5 => (0b1001, 4),
            6..=11 => (0b1010, 8),
            12..=39 => (0b1011, 16),
            40..=95 => (0b1100, 64),
            96..=191 => (0b1101, 128),
            192..=383 => (0b1110, 256),
            _ => (0b1111, 512),
        };

        let hclk = sysclk.0 / hpre_ratio;

        let (ppre1_bits, ppre1_ratio) =
            match self.cfgr.config().unwrap().hclk_fclk.0 / self.cfgr.config().unwrap().pclk1.0 {
                0 => unreachable!(),
                1 => (0b000, 1),
                2 => (0b100, 2),
                3..=5 => (0b101, 4),
                6..=11 => (0b110, 8),
                _ => (0b111, 16),
            };

        let pclk1 = Hertz(hclk / ppre1_ratio);

        let (ppre2_bits, ppre2_ratio) =
            match self.cfgr.config().unwrap().hclk_fclk.0 / self.cfgr.config().unwrap().pclk2.0 {
                0 => unreachable!(),
                1 => (0b000, 1),
                2 => (0b100, 2),
                3..=5 => (0b101, 4),
                6..=11 => (0b110, 8),
                _ => (0b111, 16),
            };

        let pclk2 = Hertz(hclk / ppre2_ratio);
        let hclk = Hertz(hclk);

        let sw_bits = match self.cfgr.config().unwrap().sysclk_src {
            clocking::SysClkSource::MSI => 0b00,
            clocking::SysClkSource::HSI16 => 0b01,
            //clocking::SysClkSource::HSE => 0b10,
            //clocking::SysClkSource::PLLCLK => 0b11,
        };

        // Configure the remaining clocks, including possibly turning them off.
        let lsiclk = self.cfgr.config().unwrap().lsi.configure(&mut self.csr);
        let msiclk = self
            .cfgr
            .config()
            .unwrap()
            .msi
            .configure(&mut self.icscr, &mut self.cr);
        let hsi16clk = self
            .cfgr
            .config()
            .unwrap()
            .hsi16
            .configure(&mut self.icscr, &mut self.cr);
        let lseclk = if let Some(lse) = self.cfgr.config().unwrap().lse.as_ref() {
            lse.configure(&mut self.csr, pwr)
        } else {
            None
        };

        // The following is messy and intends to encode ordering relationships between configuring
        // PWR, FLASH, and RCC. Specifically, the following must hold:
        //
        // 3.7.1: FLASH_ACR bit 0, LATENCY:
        //     To increase the clock frequency, the user has to change this bit to ‘1’, then to
        //     increase the frequency. To reduce the clock frequency, the user has to decrease the
        //     frequency, then to change this bit to ‘0’.
        //
        // as well as
        //
        // 6.1.7: Voltage regulator and clock management when modifying the VCORE range:
        //     • When the voltage range is above the targeted voltage range (e.g. from range 1 to 2):
        //         a) Adapt the clock frequency to the lower voltage range that will be selected at
        //         next step.
        //         b) Select the required voltage range.
        //     • When the voltage range is below the targeted voltage range (e.g. from range 3 to 1):
        //         a) Select the required voltage range.
        //         b) Tune the clock frequency if needed.

        let curr_flash_latency = flash.get_latency();
        match (curr_flash_latency, VCORE::latency(sysclk)) {
            (flash::FlashLatency::_1_Clk, flash::FlashLatency::_0_Clk) => {
                // set clock, then lower latency

                if VCORE::range() > pwr.read_vcore_range() {
                    // we're increasing the power, so set up power before setting the clock
                    unsafe { pwr.enact(&mut self.apb1) };
                }

                self.cfgr.config().unwrap().inner().write(|w| unsafe {
                    w.ppre1()
                        .bits(ppre1_bits)
                        .ppre2()
                        .bits(ppre2_bits)
                        .hpre()
                        .bits(hpre_bits)
                        .sw()
                        .bits(sw_bits)
                });

                if VCORE::range() <= pwr.read_vcore_range() {
                    // lower the power after decreasing the clock
                    unsafe { pwr.enact(&mut self.apb1) };
                }

                flash.set_latency(sysclk, pwr);
            }
            (flash::FlashLatency::_0_Clk, flash::FlashLatency::_1_Clk) => {
                // raise latency, then set clock

                flash.set_latency(sysclk, pwr);

                if VCORE::range() > pwr.read_vcore_range() {
                    // increase power before raising the clock
                    unsafe { pwr.enact(&mut self.apb1) };
                }

                self.cfgr.config().unwrap().inner().write(|w| unsafe {
                    w.ppre1()
                        .bits(ppre1_bits)
                        .ppre2()
                        .bits(ppre2_bits)
                        .hpre()
                        .bits(hpre_bits)
                        .sw()
                        .bits(sw_bits)
                });

                if VCORE::range() <= pwr.read_vcore_range() {
                    // lower the power after decreasing the clock
                    unsafe { pwr.enact(&mut self.apb1) };
                }
            }
            _ => {
                // no change to latency, so just set clock

                if VCORE::range() > pwr.read_vcore_range() {
                    // increase power before raising the clock
                    unsafe { pwr.enact(&mut self.apb1) };
                }

                self.cfgr.config().unwrap().inner().write(|w| unsafe {
                    w.ppre1()
                        .bits(ppre1_bits)
                        .ppre2()
                        .bits(ppre2_bits)
                        .hpre()
                        .bits(hpre_bits)
                        .sw()
                        .bits(sw_bits)
                });

                if VCORE::range() <= pwr.read_vcore_range() {
                    // decrease power after lowering the clock
                    unsafe { pwr.enact(&mut self.apb1) };
                }
            }
        };

        self.apb2.enr().modify(|_, w| w.syscfgen().clear_bit());
        while !self.apb2.enr().read().syscfgen().bit_is_clear() {}

        let clk_ctx = ClockContext {
            sysclk,
            hclk,
            pclk1,
            pclk2,
            lsiclk,
            msiclk,
            hsi16clk,
            lseclk,
        };

        let _ = replace(&mut self.cfgr, ClockConfig::Frozen(clk_ctx));
    }
}

/// "Frozen" clock context
pub struct ClockContext {
    /// Core system clock
    sysclk: Hertz,
    /// HCLK/FCLK
    hclk: Hertz,
    /// PCLK1 to APB1 peripherals
    pclk1: Hertz,
    /// PCLK2 to APB2 peripherals
    pclk2: Hertz,
    /// Low-speed internal RC clock speed
    lsiclk: Option<Hertz>,
    /// Medium-speed internal RC clock speed
    msiclk: Option<Hertz>,
    /// High-speed internal 16 MHz clock speed (optionally /4)
    hsi16clk: Option<Hertz>,
    /// Low-speed external 32kHz clock speed
    lseclk: Option<Hertz>,
}

impl ClockContext {
    /// Returns the frequency of the system clock SYSCLK, also CK_PWR
    pub fn sysclk(&self) -> Hertz {
        self.sysclk
    }

    /// Returns the frequency of HCLK/FCLK
    pub fn hclk_fclk(&self) -> Hertz {
        self.hclk
    }

    /// Returns the frequency of APB1
    pub fn apb1(&self) -> Hertz {
        self.pclk1
    }

    /// Returns the frequency of APB2
    pub fn apb2(&self) -> Hertz {
        self.pclk2
    }

    /// Returns the frequency of the low speed internal RC, if turned on
    pub fn lsi(&self) -> Option<Hertz> {
        self.lsiclk
    }

    /// Returns the frequency of the medium speed internal RC, if turned on
    pub fn msi(&self) -> Option<Hertz> {
        self.msiclk
    }

    /// Returns the frequency of the high speed 16 MHz internal RC, if turned on
    pub fn hsi16(&self) -> Option<Hertz> {
        self.hsi16clk
    }

    /// Returns the frequency of the low speed external oscillator, if enabled
    pub fn lse(&self) -> Option<Hertz> {
        self.lseclk
    }
}

/// AHB 1-3 register access
pub struct AHB(());
impl AHB {
    /// Access AHB reset register
    pub fn rstr(&mut self) -> &rcc::AHBRSTR {
        unsafe { &(*RCC::ptr()).ahbrstr }
    }

    /// Access AHB clock enable register
    pub fn enr(&mut self) -> &rcc::AHBENR {
        unsafe { &(*RCC::ptr()).ahbenr }
    }

    /// Access AHB enable clock in sleep mode register
    pub fn smenr(&mut self) -> &rcc::AHBSMENR {
        unsafe { &(*RCC::ptr()).ahbsmenr }
    }
}

/// APB1 register access
pub struct APB1(());
impl APB1 {
    /// Access APB1RSTR1 reset register
    pub fn rstr(&mut self) -> &rcc::APB1RSTR {
        unsafe { &(*RCC::ptr()).apb1rstr }
    }

    /// Access APB1ENR reset register
    pub fn enr(&mut self) -> &rcc::APB1ENR {
        unsafe { &(*RCC::ptr()).apb1enr }
    }
}

/// APB2 register access
pub struct APB2(());
impl APB2 {
    /// Access APB2RSTR reset register
    pub fn rstr(&mut self) -> &rcc::APB2RSTR {
        unsafe { &(*RCC::ptr()).apb2rstr }
    }
    /// Access APB2ENR reset register
    pub fn enr(&mut self) -> &rcc::APB2ENR {
        unsafe { &(*RCC::ptr()).apb2enr }
    }
}

/// I/O port register access
pub struct IOP(());
impl IOP {
    /// Access IOPENR enable register
    pub fn enr(&mut self) -> &rcc::IOPENR {
        unsafe { &(*RCC::ptr()).iopenr }
    }

    /// Access IOPRSTR reset register
    pub fn rstr(&mut self) -> &rcc::IOPRSTR {
        unsafe { &(*RCC::ptr()).ioprstr }
    }

    /// Access IOPSMENR sleep mode register
    pub fn smenr(&mut self) -> &rcc::IOPSMEN {
        unsafe { &(*RCC::ptr()).iopsmen }
    }
}

/// Clock configuration register
pub struct CCIPR(());
impl CCIPR {
    /// Direct access to RCC_CCIPR
    #[inline]
    pub fn inner(&mut self) -> &rcc::CCIPR {
        unsafe { &(*RCC::ptr()).ccipr }
    }
}

/// Clock control register
pub struct CR(());
impl CR {
    /// Direct access to RCC_CR
    #[inline]
    pub fn inner(&mut self) -> &rcc::CR {
        unsafe { &(*RCC::ptr()).cr }
    }
}

/// Control/status register
pub struct CSR(());
impl CSR {
    /// Direct access to RCC_CSR
    #[inline]
    pub fn inner(&mut self) -> &rcc::CSR {
        unsafe { &(*RCC::ptr()).csr }
    }
}

/// MCU clock configuration - either a configurable register or a "frozen" context
pub enum ClockConfig {
    /// Access to configure the clocks
    Open(CFGR),
    /// Frozen clock context that cannot be changed
    Frozen(ClockContext),
}

#[derive(Debug)]
/// Possible errors raised when trying to access the RCC clock configuration
pub enum CfgErr {
    /// The clock configuration is not frozen
    Configurable,
    /// The clock configuration cannot be configured
    Frozen,
}

impl ClockConfig {
    /// Retrieve the configurable clock settings, if not yet frozen
    pub fn config(&mut self) -> Result<&mut CFGR, CfgErr> {
        match self {
            ClockConfig::Open(cfgr) => Ok(cfgr),
            _ => Err(CfgErr::Frozen),
        }
    }

    /// Retrieve the frozen clock settings, if frozen
    pub fn context(&self) -> Result<&ClockContext, CfgErr> {
        match self {
            ClockConfig::Frozen(ctx) => Ok(ctx),
            _ => Err(CfgErr::Configurable),
        }
    }
}

/// Clock configuration register
pub struct CFGR {
    /// Desired HCLK/FCLK frequency
    pub hclk_fclk: Hertz,
    /// Desired peripheral clock 1 frequency
    pub pclk1: Hertz,
    /// Desired peripheral clock 2 frequency
    pub pclk2: Hertz,
    /// SYSCLK source selection
    pub sysclk_src: clocking::SysClkSource,
    /// Low-speed internal RC configuration
    pub lsi: clocking::LowSpeedInternalRC,
    /// Medium-speed internal RC configuration
    pub msi: clocking::MediumSpeedInternalRC,
    /// High-speed 16 MHz internal RC configuration
    pub hsi16: clocking::HighSpeedInternal16RC,
    /// Low-speed external oscillator configuration
    pub lse: Option<clocking::LowSpeedExternalOSC>,
    //pub pll: clocking::PLL,
    //pub hse: Option<clocking::HighSpeedExternalOSC>,
}

/// Clock configuration register
impl CFGR {
    /// Direct access to RCC_CFGR
    #[inline]
    pub(crate) fn inner(&mut self) -> &rcc::CFGR {
        unsafe { &(*RCC::ptr()).cfgr }
    }
}

/// Internal clock sources calibration register
pub struct ICSCR(());
impl ICSCR {
    /// Direct access to RCC_ICSCR
    #[inline]
    pub fn inner(&mut self) -> &rcc::ICSCR {
        unsafe { &(*RCC::ptr()).icscr }
    }
}
