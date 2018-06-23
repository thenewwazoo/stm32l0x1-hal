
use stm32l0x1::{rcc, RCC};
use flash::ACR;
use super::time::Hertz;
use super::power::{Power, VCoreRange};
use rcc::clocking::AutoConf;

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
                sysclk: clocking::SysClkSource::MSI(
                    clocking::MediumSpeedInternalRC::new(2_097_000)
                    ),
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

    pub fn sysclk(mut self, src: clocking::SysClkSource) -> Self
    {
        self.sysclk = src;
        self
    }

    pub fn freeze(self, acr: &mut ACR, pwr: Power) -> Clocks {
        let rcc = unsafe { &*RCC::ptr() };

        let cfgd = self.sysclk.configure(rcc, pwr);
        let sysclk = cfgd.f;
        let bits = cfgd.bits;
        let power = cfgd.release();

        if sysclk > 32_000_000 {
            panic!("sysclk too high");
        }

        match power.vcore_range() {
            VCoreRange::Range1 => {
                if sysclk > 16_000_000 {
                    acr.acr().modify(|_, w| w.latency().set_bit());
                } else {
                    acr.acr().modify(|_, w| w.latency().clear_bit());
                }
            },
            VCoreRange::Range2 => {
                if acr.acr().read().latency().bit() {
                    // 1 wait state => max 16 MHz
                } else {
                    // 0 wait states => max 8 MHz
                }
            },
            VCoreRange::Range3 => {
                // max 4.2 MHz
            },
        }

        let hpre_bits = self.hclk
            .map(|hclk| match sysclk / hclk {
                0 => unreachable!(),
                1 => 0b0111,
                2 => 0b1000,
                3...5 => 0b1001,
                6...11 => 0b1010,
                12...39 => 0b1011,
                40...95 => 0b1100,
                96...191 => 0b1101,
                192...383 => 0b1110,
                _ => 0b1111,
            })
            .unwrap_or(0b0111);

        let hclk = sysclk / (1 << (hpre_bits - 0b0111));

        let ppre1_bits = self.pclk1
            .map(|pclk1| match hclk / pclk1 {
                0 => unreachable!(),
                1 => 0b011,
                2 => 0b100,
                3...5 => 0b101,
                6...11 => 0b110,
                _ => 0b111,
            })
            .unwrap_or(0b011);

        let ppre2_bits = self.pclk2
            .map(|pclk2| match hclk / pclk2 {
                0 => unreachable!(),
                1 => 0b011,
                2 => 0b100,
                3...5 => 0b101,
                6...11 => 0b110,
                _ => 0b111,
            })
            .unwrap_or(0b011);

        let ppre1: u8 = 1 << (ppre1_bits - 0b011);
        let ppre2: u8 = 1 << (ppre2_bits - 0b011);
        let pclk1 = hclk / ppre1 as u32;
        let pclk2 = hclk / ppre2 as u32;

        let rcc = unsafe { &*RCC::ptr() };
        // use HSI as source
        rcc.cfgr
            .write(|w| unsafe { w.ppre1().bits(ppre1_bits).hpre().bits(hpre_bits).sw().bits(bits) });

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
