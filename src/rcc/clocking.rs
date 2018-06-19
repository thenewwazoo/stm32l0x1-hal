//! The `clocking` module contains representations of the various objects in the STM32L0x1
//! clock tree (see Reference Manual Figs 17) useful for wiring them up.
//!
//! There are two main concepts: sources (enum variants) and clocks (structs). For example, the
//! SYSCLK clock can be driven by any one of four sources: HSI16, MSI, HSE, and PLLCLK. This
//! knowledge is encoded in the SysClkSources enum.
//!
//! Each enum variant contains information about the clock it represents. Some clocks are
//! configurable, and thus have fields, and some are not. For example, the LSI
//! (LowSpeedInternalRC) clock is always 32 kHz, but the MSI (MediumSpeedInternalRC) clock can
//! be configured and thus has a frequency component.
//!
//! To use them, compose them and feed them to, e.g., sysclk.
//!
//! ```rust
//! let mut rcc = RCC.constrain();
//! let msi_clk = clocking::MediumSpeedInternalRC::new(8_000_000, false);
//! let sys_clk_src = clocking::SysClkSource::MSI(msi_clk);
//! let cfgr = rcc.cfgr.sysclk(sys_clk_src);
//! ```
//!
//! The PLL is a bit more complex because it _is_ a source (`PLLClkOutput`) and also _requires_
//! a source (`PLLClkSource`), but you compose the types similarly.

use super::rcc;
//use time::Hertz;
use cortex_m::asm;

use power::{Power, VCoreRange};

/// Clocks (OSCs or RCs) that can be used as inputs to peripherals
///
/// This trait isn't actually specified anywhere, and is used only by convention.
pub trait InputClock {
    /// Return the frequency of the clock (either calculated, configured, or intrinsic)
    fn freq(&self) -> u32;
}

/// Clock sources can configure themselves. This is a by-convention trait.
pub trait AutoConf {
    /// Clock, configure thyself
    fn configure(self, rcc: &rcc::RegisterBlock, pwr: Power) -> ConfiguredClock;
}

/// The result of configuring a clock
pub struct ConfiguredClock {
    /// The resulting output frequency of the configured clock
    pub f: u32,
    /// The System clock switch bits necessary to select this clock (see 7.3.3)
    pub bits: u8,
    /// The Power peripheral that may control the clock's operating parameters
    pwr: Power,
}

impl ConfiguredClock {
    /// Release the power peripheral so it can be reconfigured
    pub fn release(self) -> Power {
        self.pwr
    }
}

/// High-speed internal 16 MHz RC
pub struct HighSpeedInternal16RC;

impl AutoConf for HighSpeedInternal16RC {
    /// Applies the selection options to the configuration registers and turns the clock on
    fn configure(self, rcc: &rcc::RegisterBlock, pwr: Power) -> ConfiguredClock {
        rcc.icscr.modify(|_,w| unsafe { w.hsi16trim().bits(0x10) }); // 16 is the default value
        rcc.cr.modify(|_, w| w.hsi16on().set_bit());
        while rcc.cr.read().hsi16rdyf().bit_is_clear() {}
        ConfiguredClock{ f: self.freq(), bits: 0b01, pwr: pwr }
    }
}

impl InputClock for HighSpeedInternal16RC {
    fn freq(&self) -> u32 {
        16_000_000
    }
}

/// Medium-speed internal 65 kHz - 4 MHz RC
pub struct MediumSpeedInternalRC {
    freq: u32,
}

impl MediumSpeedInternalRC {
    /// Create a new MSI RC
    ///
    /// `freq` must be a valid MSI RC frequency range (see 7.2.3)
    /// TODO make freq a repr(C) enum
    pub fn new(freq: u32) -> Self {
        MediumSpeedInternalRC { freq }
    }

    /// Convert the freq range to MSIRANGE bits (7.3.2). Panics if `freq` is invalid.
    pub fn bits(&self) -> u8 {
        match self.freq {
            65_536 => 0b000,
            131_072 => 0b001,
            262_144 => 0b010,
            524_288 => 0b011,
            1_048_000 => 0b100,
            2_097_000 => 0b101,
            4_194_000 => 0b110,
            _ => panic!("bad MSI speed value!"),
        }
    }
}

impl AutoConf for MediumSpeedInternalRC {
    /// Configures the MSI to the specified frequency, and enables hardware
    /// auto-calibration if requested by enabling (and waiting for) the LSE.
    fn configure(self, rcc: &rcc::RegisterBlock, pwr: Power) -> ConfiguredClock {
        rcc.icscr.modify(|_, w| unsafe { w.msirange().bits(self.bits()) });
        while rcc.cr.read().msirdy().bit_is_clear() {}

        ConfiguredClock{ f: self.freq(), bits: 0b00, pwr: pwr }
    }
}

impl InputClock for MediumSpeedInternalRC {
    fn freq(&self) -> u32 {
        self.freq
    }
}

/// High-speed external 1-24 MHz oscillator
pub struct HighSpeedExternalOSC {
    f: u32,
}

impl InputClock for HighSpeedExternalOSC {
    fn freq(&self) -> u32 {
        self.f
    }
}

impl HighSpeedExternalOSC {
    pub fn new(f: u32) -> Self {
        HighSpeedExternalOSC{ f }
    }
}

impl AutoConf for HighSpeedExternalOSC {
    /// Turns on the HSE oscillator.
    ///
    /// (Should this also configure the pin?)
    fn configure(self, rcc: &rcc::RegisterBlock, pwr: Power) -> ConfiguredClock {

        assert!(self.freq() < match pwr.vcore_range() {
            VCoreRange::Range1 => 32_000_000, // 24 MHz max for crystal!!!
            VCoreRange::Range2 => 16_000_000,
            VCoreRange::Range3 => 8_000_000,
        }, "HSE speed too high for VCore");

        rcc.cr.modify(|_, w| w.hseon().set_bit());
        while rcc.cr.read().hserdy().bit_is_clear() {}

        ConfiguredClock { f: self.freq(), bits: 0b10, pwr: pwr }
    }
}

// TODO implement RTC support

/// Selectable clocks for the SYSCLK signal (HCLK bus)
pub enum SysClkSource {
    /// High speed internal 16 MHz RC
    HSI16(HighSpeedInternal16RC),
    /// Medium speed internal 65kHz-4MHz RC
    MSI(MediumSpeedInternalRC),
    /// High-speed external oscillator
    HSE(HighSpeedExternalOSC),
    /// PLLCLK signal (output of PLL)
    PLL(PLLClkOutput),
}

impl AutoConf for SysClkSource {
    fn configure(self, rcc: &rcc::RegisterBlock, pwr: Power) -> ConfiguredClock {
        match self {
            SysClkSource::HSI16(s) => s.configure(rcc, pwr),
            SysClkSource::MSI(s) => s.configure(rcc, pwr),
            SysClkSource::HSE(s) => s.configure(rcc, pwr),
            SysClkSource::PLL(s) => s.configure(rcc, pwr),
        }
    }
}

#[derive(Copy, Clone)]
/// The multiplier applied to the PLL's input clock. Generates PLLVCO.
pub enum PllMul {
    /// x3
    Mul3 = 3,
    /// x4
    Mul4 = 4,
    /// x6
    Mul6 = 6,
    /// x8
    Mul8 = 8,
    /// x12
    Mul12 = 12,
    /// x16
    Mul16 = 16,
    /// x24
    Mul24 = 24,
    /// x32
    Mul32 = 32,
    /// x48
    Mul48 = 48,
}

impl PllMul {
    /// Return bits suitable for RCC_CFGR.PLLMUL
    pub fn bits(self) -> u8 {
        match self {
            PllMul::Mul3  => 0b0000,
            PllMul::Mul4  => 0b0001,
            PllMul::Mul6  => 0b0010,
            PllMul::Mul8  => 0b0011,
            PllMul::Mul12 => 0b0100,
            PllMul::Mul16 => 0b0101,
            PllMul::Mul24 => 0b0110,
            PllMul::Mul32 => 0b0111,
            PllMul::Mul48 => 0b1000,
        }
    }
}

#[derive(Copy, Clone)]
/// The divisor applied to PLLVCO
pub enum PllDiv {
    /// /2
    Div2 = 2,
    /// /3
    Div3 = 3,
    /// /4
    Div4 = 4,
}

impl PllDiv {
    /// Return bits suitable for RCC_CFGR.PLLDIV
    pub fn bits(self) -> u8 {
        match self {
            PllDiv::Div2 => 0b01,
            PllDiv::Div3 => 0b10,
            PllDiv::Div4 => 0b11,
        }
    }
}

/// PLLCLK output of PLL module
pub struct PLLClkOutput {
    /// The clock that will drive the PLL
    src: PLLClkSource,
    /// Bit to configure which input to use
    bits: u8,
    /// The input multiplier
    mul: PllMul,
    /// The PLLVCO divisor
    div: PllDiv,
}

impl PLLClkOutput {
    /// Create a new PLL clock source to use as an input.
    ///
    /// Panics if the configuration is invalid.
    pub fn new(src: PLLClkSource, mul: PllMul, div: PllDiv) -> Self {

        if let PLLClkSource::HSE(ref s) = src {
            if s.freq() < 2_000_000 {
                panic!("HSE clock {} too slow to drive PLL", s.freq());
            }
        }

        let bits = match src {
            PLLClkSource::HSI16(_) => 0b0,
            PLLClkSource::HSE(_)  => 0b1,
        };

        PLLClkOutput { src, bits, div, mul }
    }
}

impl AutoConf for PLLClkOutput {
    /// AutoConf the PLL to enable the PLLCLK output. Also configures the PLL's source clock
    fn configure(self, rcc: &rcc::RegisterBlock, pwr: Power) -> ConfiguredClock {

        asm::bkpt();

        // Configure the PLL's input
        let pllsrc_cfg = self.src.configure(rcc, pwr);

        rcc.cr.modify(|_, w| w.pllon().clear_bit());
        while rcc.cr.read().pllrdy().bit_is_set() {}

        let input_freq = pllsrc_cfg.f;

        let pllvco = input_freq * self.mul as u32;

        let pwr = pllsrc_cfg.release();

        /*
        assert!(pllvco <= match pwr.vcore_range() {
            VCoreRange::Range1 => 96_000_000,
            VCoreRange::Range2 => 48_000_000,
            VCoreRange::Range3 => 24_000_000,
        }, "pllvco too high: {}", pllvco);
        */

        let f = pllvco / self.div as u32;

        /*
        assert!(f <= match pwr.vcore_range() {
            VCoreRange::Range1 => 32_000_000,
            VCoreRange::Range2 => 16_000_000,
            VCoreRange::Range3 => 4_000_000,
        }, "pll output too high!");
        */

        // because we partially moved self, we need to move relevant members out before we can use
        // them in the modify closure
        let (bits, mul, div) = (self.bits, self.mul, self.div);
        rcc.cfgr.modify(|_, w| unsafe {
            w.pllsrc()
                .bit(bits == 0)
                .pllmul()
                .bits(mul.bits())
                .plldiv()
                .bits(div.bits())
        });

        rcc.cr.modify(|_, w| w.pllon().set_bit());
        while rcc.cr.read().pllrdy().bit_is_clear() {}

        ConfiguredClock { f: f, bits: 0b11, pwr: pwr }
    }
}

// /// PLLADC2CLK output of PLLSAI2
// #[derive(Clone, Copy)]
// pub struct PLLADC2Clk {
// src: PLLClkSource,
// ...,
// }
//

/// Selectable PLL module input sources
pub enum PLLClkSource {
    /// HSI16
    HSI16(HighSpeedInternal16RC),
    /// HSE
    HSE(HighSpeedExternalOSC),
}

impl AutoConf for PLLClkSource {
    /// This configures the input to the PLL. It's usually only called by
    /// PLLClkOutput::configure.
    fn configure(self, rcc: &rcc::RegisterBlock, pwr: Power) -> ConfiguredClock {
        match self {
            PLLClkSource::HSI16(s) => {
                let cfg = s.configure(rcc, pwr);
                ConfiguredClock{ f: cfg.f, bits: 0b0, pwr: cfg.release() }
            },
            PLLClkSource::HSE(s) => {
                let cfg = s.configure(rcc, pwr);
                ConfiguredClock{ f: cfg.f, bits: 0b1, pwr: cfg.release() }
            },
        }
    }
}

impl InputClock for PLLClkSource {
    fn freq(&self) -> u32 {
        match self {
            PLLClkSource::HSI16(ref s) => s.freq(), // *siiiigh*
            PLLClkSource::HSE(ref s) => s.freq(),
        }
    }
}

/*
pub enum USARTClkSource {
    PCLK(PeripheralClock),
    /// U(S)ART-specific peripheral clock (PCLK1, PCLK2)
    LSE,
    HSI16(HighSpeedInternal16RC),
    SYSCLK(Hertz),
}

pub enum PeripheralClock {
    PCLK1(Hertz),
    PCLK2(Hertz),
}

impl InputClock for PeripheralClock {
    fn freq(&self) -> u32 {
        match *self {
            PeripheralClock::PCLK1(s) => s.0.into(),
            PeripheralClock::PCLK2(s) => s.0.into(),
        }
    }
}
*/
