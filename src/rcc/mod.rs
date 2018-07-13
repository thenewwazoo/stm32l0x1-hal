
#![allow(unknown_lints)]
#![allow(clippy)]

use super::power::Power;
use super::time::Hertz;
use flash::ACR;
use rcc::clocking::AutoConf;
use stm32l0x1::{rcc, RCC};

pub mod clocking;

// TODO: Make the HSE and LSE clocks consume the relevant GPIO pins because
//
// 8.3.13 Using the HSE or LSE oscillator pins as GPIOs
//
// When the HSE or LSE oscillator is switched OFF (default state after reset), the related
// oscillator pins can be used as normal GPIOs.
//
// When the HSE or LSE oscillator is switched ON (by setting the HSEON or LSEON bit in the RCC_CSR
// register) the oscillator takes control of its associated pins and the GPIO configuration of
// these pins has no effect.                                             ^^^^^^^^^^^^^^^^^^^^^
// ^^^^^^^^^^^^^^^^^^^^^^^^^
// When the oscillator is configured in a user external clock mode, only the OSC_IN, CK_IN or
// OSC32_IN pin is reserved for clock input and the OSC_OUT or OSC32_OUT pin can still be used as
// normal GPIO.

/// Extension trait that constrains the `RCC` peripheral
pub trait RccExt {
    /// Constrains the `RCC` peripheral so it plays nicely with the other abstractions
    fn constrain(self) -> Rcc;
}

impl RccExt for RCC {
    fn constrain(self) -> Rcc {
        Rcc {
            ahb: AHB(()),
            apb1: APB1(()),
            apb2: APB2(()),
            iop: IOP(()),
            ccipr: CCIPR(()),
            cfgr: CFGR {
                hclk: None,
                pclk1: None,
                pclk2: None,
                sysclk: clocking::SysClkSource::MSI(clocking::MediumSpeedInternalRC::new(
                    2_097_000,
                )),
            },
        }
    }
}

/// Constrained RCC peripheral
pub struct Rcc {
    /// AMBA High-performance Bus (AHB) registers.
    pub ahb: AHB,
    /// APB1 peripheral registers.
    pub apb1: APB1,
    /// APB2 peripheral registers.
    pub apb2: APB2,
    /// Peripherals independent clock configuration register
    pub ccipr: CCIPR,
    /// HW clock configuration.
    pub cfgr: CFGR,
    /// GPIO port configuration
    pub iop: IOP,
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

pub struct IOP(());
impl IOP {
    pub fn enr(&mut self) -> &rcc::IOPENR {
        unsafe { &(*RCC::ptr()).iopenr }
    }

    pub fn rstr(&mut self) -> &rcc::IOPRSTR {
        unsafe { &(*RCC::ptr()).ioprstr }
    }

    pub fn smenr(&mut self) -> &rcc::IOPSMEN {
        unsafe { &(*RCC::ptr()).iopsmen }
    }
}

pub struct CCIPR(());
impl CCIPR {
    #[inline]
    pub fn inner(&mut self) -> &rcc::CCIPR {
        unsafe { &(*RCC::ptr()).ccipr }
    }
}

pub struct CFGR {
    /// AHB bus frequency
    hclk: Option<u32>,
    /// APB1 frequency
    pclk1: Option<u32>,
    /// APB2 frequency
    pclk2: Option<u32>,
    /// SYSCLK - not Option because it cannot be None
    sysclk: clocking::SysClkSource,
}

impl CFGR {
    pub fn hclk<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.hclk = Some(freq.into().0);
        self
    }

    pub fn pclk1<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.pclk1 = Some(freq.into().0);
        self
    }

    pub fn pclk2<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.pclk2 = Some(freq.into().0);
        self
    }

    pub fn sysclk(mut self, src: clocking::SysClkSource) -> Self {
        self.sysclk = src;
        self
    }

    pub fn freeze(self, acr: &mut ACR, pwr: Power) -> Clocks {
        // TODO
        // To guarantee 32 MHz operation at VDD =1.8 V±5%, with 1 wait state, and VCORE range 1,
        // the CPU frequency in run mode must be managed to prevent any changes exceeding a ratio
        // of 4 in one shot. A delay of 5 μs must be respected between 2 changes. There is no
        // limitation when waking up from low-power mode.

        let rcc = unsafe { &*RCC::ptr() };

        let cfgd = self.sysclk.configure(acr, rcc, pwr);
        let sysclk = cfgd.f;
        let sw_bits = cfgd.bits;
        let power = cfgd.release();

        let (hpre_bits, hpre_ratio) = self
            .hclk
            .map(|hclk| match sysclk / hclk {
                0 => unreachable!(),
                1 => (0b0000, 1),
                2 => (0b1000, 2),
                3...5 => (0b1001, 4),
                6...11 => (0b1010, 8),
                12...39 => (0b1011, 16),
                40...95 => (0b1100, 64),
                96...191 => (0b1101, 128),
                192...383 => (0b1110, 256),
                _ => (0b1111, 512),
            })
            .unwrap_or((0b0111, 1));

        let hclk = sysclk / hpre_ratio;

        let (ppre1_bits, ppre1_ratio) = self
            .pclk1
            .map(|pclk1| match hclk / pclk1 {
                0 => unreachable!(),
                1 => (0b000, 1),
                2 => (0b100, 2),
                3...5 => (0b101, 4),
                6...11 => (0b110, 8),
                _ => (0b111, 16),
            })
            .unwrap_or((0b011, 1));

        let pclk1 = hclk / ppre1_ratio;

        let (ppre2_bits, ppre2_ratio) = self
            .pclk2
            .map(|pclk2| match hclk / pclk2 {
                0 => unreachable!(),
                1 => (0b011, 1),
                2 => (0b100, 2),
                3...5 => (0b101, 4),
                6...11 => (0b110, 8),
                _ => (0b111, 16),
            })
            .unwrap_or((0b011, 16));

        let pclk2 = hclk / ppre2_ratio;

        let rcc = unsafe { &*RCC::ptr() };
        rcc.cfgr.write(|w| unsafe {
            w.ppre1()
                .bits(ppre1_bits)
                .ppre2()
                .bits(ppre2_bits)
                .hpre()
                .bits(hpre_bits)
                .sw()
                .bits(sw_bits)
        });

        Clocks {
            hclk: Hertz(hclk),
            pclk1: Hertz(pclk1),
            pclk2: Hertz(pclk2),
            sysclk: Hertz(sysclk),
            pwr: power,
        }
    }
}

/// Frozen clock frequencies
///
/// The existence of this value indicates that the clock configuration can no longer be changed
pub struct Clocks {
    hclk: Hertz,
    pclk1: Hertz,
    pclk2: Hertz,
    sysclk: Hertz,
    pwr: Power,
}

impl Clocks {
    /// Returns the frequency of the AHB
    pub fn hclk(&self) -> Hertz {
        self.hclk
    }

    /// Returns the frequency of the APB
    pub fn pclk1(&self) -> Hertz {
        self.pclk1
    }

    /// Returns the frequency of the APB
    pub fn pclk2(&self) -> Hertz {
        self.pclk2
    }

    /// Returns the system (core) frequency
    pub fn sysclk(&self) -> Hertz {
        self.sysclk
    }

    /// Release the power peripheral so it can be reconfigured
    pub fn release(self) -> Power {
        self.pwr
    }
}
