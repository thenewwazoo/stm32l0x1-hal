//! Analog-digital conversion
//!
//! This is a (partial) implementation of the ADC for the STM32L0x1 family.
//!
//! The ADC has three basic run modes: Single, Continuous, and Discontinuous. For a discussion of
//! these modes, see 13.3.9 for Single, 13.3.10 for Continuous, and 13.4.1 for Disconinuous modes.
//! Only the `OneShot` trait is implemented, which greatly underutilizes the hardware.
//!
//! ```rust
//! use stm32l0x1_hal;
//!
//! let d = stm32l0x1_hal::stm32l0x1::Peripherals::take().unwrap();
//! let adc = d.ADC;
//! let mut flash = d.FLASH.constrain();
//! let mut pwr = d.PWR.constrain();
//! let mut rcc = d.RCC.constrain().freeze(&mut flash, &mut pwr);
//!
//! // Configure the ADC
//! let mut adc: adc::Adc<adc::Res12Bit, adc::Single> = adc::Adc::new(
//!     adc,
//!     Duration::new(0, 0),
//!     adc::AdcClkSrc::Hsi16,
//!     &pwr,
//!     &rcc.cfgr.context().unwrap(),
//!     &mut rcc.apb2,
//!     );
//!
//! // Set up a pin that supports being used as an ADC input
//! let gpioa = gpio::A::new(d.GPIOA, &mut self.rcc.iop);
//! let mut light_sense = gpioa
//!     .PA7
//!     .into_input::<Floating>(&mut gpioa.moder, &mut gpioa.pupdr)
//!     .into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
//!
//! let lvl = block!(adc.read_channel(&mut light_sense)).unwrap();
//! ```

use core::marker::PhantomData;
use core::time::Duration;

use gpio::Analog;
use gpio::{PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PB0, PB1};
use hal::adc::{Channel, OneShot};
use nb;
use power::{self, VCoreRange};
use rcc::{self, ClockContext};
use stm32l0x1::{adc, ADC};

#[doc(hidden)]
mod private {
    #[doc(hidden)]
    /// Sealed stops crates other than STM32L0x1-HAL from implementing traits that use it.
    pub trait Sealed {}

    impl Sealed for super::Res12Bit {}
    impl Sealed for super::Res10Bit {}
    impl Sealed for super::Res8Bit {}
    impl Sealed for super::Res6Bit {}

    impl Sealed for super::Single {}
    impl Sealed for super::Continuous {}
    impl Sealed for super::Discontinuous {}

    impl Sealed for super::ScanUp {}
    impl Sealed for super::ScanDown {}
}

/// ADC-related errors
#[derive(Debug)]
pub enum Error {
    /// A conversion on a different channel is already in progress
    AlreadyInProgress,
    /// The result waiting is for a different channel
    WrongChannel,
}

/// ADC clock input selection
pub enum AdcClkSrc {
    /// Clock from APB bus
    ///
    /// Has the advantage of bypassing the clock domain resynchronizations. This can be useful when
    /// the ADC is triggered by a timer and if the application requires that the ADC is precisely
    /// triggered without any jitter.
    Pclk,
    /// Clocked by HSI16
    ///
    /// Has the advantage of reaching the maximum ADC clock frequency whatever the APB clock scheme
    /// selected.
    Hsi16,
}

#[doc(hidden)]
/// Helper trait to store configuration bits and word size for the given resolution
pub trait Resolution: private::Sealed {
    /// Data size of ADC reading
    type Word;
    /// Config bits in register
    const BITS: u8;
}

/// 12-bit resolution (type state)
pub struct Res12Bit(());
impl Resolution for Res12Bit {
    type Word = u16;
    const BITS: u8 = 0b00;
}
/// 10-bit resolution (type state)
pub struct Res10Bit(());
impl Resolution for Res10Bit {
    type Word = u16;
    const BITS: u8 = 0b01;
}
/// 8-bit resolution (type state)
pub struct Res8Bit(());
impl Resolution for Res8Bit {
    type Word = u8;
    const BITS: u8 = 0b10;
}
/// 6-bit resolution (type state)
pub struct Res6Bit(());
impl Resolution for Res6Bit {
    type Word = u8;
    const BITS: u8 = 0b11;
}

#[doc(hidden)]
/// Denotes ADC operating mode
pub trait RunMode: private::Sealed {
    /// Auto-configure the ADC run mode
    #[doc(hidden)]
    fn cfg(cfgr1: &mut adc::cfgr1::W) -> &mut adc::cfgr1::W;
}

/// Single (type state) conversion mode
///
/// The ADC performs a single sequence of conversions, converting all the channels once.
pub struct Single(());
impl RunMode for Single {
    fn cfg(w: &mut adc::cfgr1::W) -> &mut adc::cfgr1::W {
        w.cont().clear_bit().discen().clear_bit().autoff().set_bit()
    }
}

/// Continuous (type state) conversion mode
///
/// When a software or hardware trigger event occurs, the ADC performs a sequence of conversions,
/// converting all the channels once and then automatically re-starts and continuously performs the
/// same sequence of conversions.
pub struct Continuous(());
impl RunMode for Continuous {
    fn cfg(w: &mut adc::cfgr1::W) -> &mut adc::cfgr1::W {
        w.discen().clear_bit().cont().set_bit()
    }
}

/// Discontinuous (type state) mode
///
/// A hardware or software trigger event is required to start each conversion defined in the sequence.
pub struct Discontinuous(());
impl RunMode for Discontinuous {
    fn cfg(w: &mut adc::cfgr1::W) -> &mut adc::cfgr1::W {
        w.cont().clear_bit().discen().set_bit()
    }
}

#[doc(hidden)]
/// Helper trait to store configuration bit for setting the scan direction
///
/// Note: this configuration feature is not yet implemented. This is a placeholder.
pub trait ScanDir: private::Sealed {
    /// Helper to configure the ADC for the desired direction
    fn cfg(w: &mut adc::cfgr1::W) -> &mut adc::cfgr1::W;
}

/// Scan from channel 0 to the highest-numbered channel (varies based on chip) (type state)
///
/// Note: this configuration feature is not yet implemented. This is a placeholder.
pub struct ScanUp(());
impl ScanDir for ScanUp {
    fn cfg(w: &mut adc::cfgr1::W) -> &mut adc::cfgr1::W {
        w.scandir().bit(false)
    }
}

/// Scan from the highest-numbered channel to 0 (varies based on chip) (type state)
///
/// Note: this configuration feature is not yet implemented. This is a placeholder.
pub struct ScanDown(());
impl ScanDir for ScanDown {
    fn cfg(w: &mut adc::cfgr1::W) -> &mut adc::cfgr1::W {
        w.scandir().bit(true)
    }
}

macro_rules! adc_pin {
    ($PXi:ident, $i:expr) => {
        impl<RES, MODE> Channel<Adc<RES, MODE>> for $PXi<Analog> where MODE: RunMode, RES: Resolution {
            type ID = u8;
            fn channel() -> u8 { $i }
        }
    };
}

adc_pin!(PA0, 0);
adc_pin!(PA1, 1);
adc_pin!(PA2, 2);
adc_pin!(PA3, 3);
adc_pin!(PA4, 4);
adc_pin!(PA5, 5);
adc_pin!(PA6, 6);
adc_pin!(PA7, 7);
adc_pin!(PB0, 8);
adc_pin!(PB1, 9);

/// ADC interrupt sources
pub enum Events {
    /// One-shot sample has completed
    SampleEnd,
    /// A running conversion has ended(?)
    ConversionEnd,
    /// The sequence of conversions has completed
    SeqConvEnd,
    /// Analog watchdog
    AnalogWD,
    /// Overrun
    Overrun,
}

/// The constrained ADC peripheral
pub struct Adc<RES, MODE>
where
    RES: Resolution,
    MODE: RunMode,
{
    /// The raw ADC peripheral
    adc: ADC,
    /// ADC calibration factor
    cal_fact: u8,
    /// ADC Resolution
    #[doc(hidden)]
    _res: PhantomData<RES>,
    /// Running mode (single-ended, continuous, etc)
    #[doc(hidden)]
    _mode: PhantomData<MODE>,
}

impl<RES, MODE> Adc<RES, MODE>
where
    RES: Resolution,
    MODE: RunMode,
{
    /// (Private) access to the inner ADC peripheral register set
    fn adc(&mut self) -> &mut ADC {
        &mut self.adc
    }
}

impl<WORD, RES, PIN> OneShot<Adc<RES, Single>, RES::Word, PIN> for Adc<RES, Single>
where
    WORD: From<u16>,
    RES: Resolution<Word = WORD>,
    PIN: Channel<Adc<RES, Single>, ID = u8>,
{
    type Error = Error;

    fn read(&mut self, _pin: &mut PIN) -> nb::Result<RES::Word, Error> {
        let chan = 1 << PIN::channel();

        // if a conversion is ongoing
        if self.adc().cr.read().adstart().bit_is_set() {
            if self.adc().chselr.read().bits() != chan {
                // it's not the same channel, so return an error
                Err(nb::Error::Other(Error::AlreadyInProgress))
            } else {
                // it's the same channel, so block
                Err(nb::Error::WouldBlock)
            }
        } else {
            if self.adc().isr.read().eoc().bit_is_set() {
                // a conversion is complete!
                if self.adc().chselr.read().bits() != chan {
                    // it's not the same channel, so return an error
                    Err(nb::Error::Other(Error::WrongChannel))
                } else {
                    let result = self.adc().dr.read().data().bits();
                    self.adc().chselr.reset();
                    Ok(result.into())
                }
            } else {
                // select the channel
                self.adc()
                    .chselr
                    .write(|w| unsafe { w.bits(1 << PIN::channel()) });
                self.start();
                Err(nb::Error::WouldBlock)
            }
        }
    }
}

impl<RES, MODE> Adc<RES, MODE>
where
    RES: Resolution,
    MODE: RunMode,
{
    /// Constrain the ADC register set and create a new Adc peripheral
    pub fn new<VDD, VCORE, RTC>(
        raw: ADC,
        samp_time: Duration,
        clk_src: AdcClkSrc,
        pwr: &power::Power<VDD, VCORE, RTC>,
        clk_ctx: &ClockContext,
        apb2: &mut rcc::APB2,
    ) -> Adc<RES, MODE>
    where
        VCORE: power::Vos,
    {
        apb2.enr().modify(|_, w| w.adcen().set_bit());
        while apb2.enr().read().adcen().bit_is_clear() {}

        raw.cfgr1
            .modify(|_, w| unsafe { MODE::cfg(w).res().bits(RES::BITS).autoff().set_bit() });

        let fadc_limits = (
            140_000,
            match pwr.read_vcore_range() {
                VCoreRange::Range1 => 16_000_000,
                VCoreRange::Range2 => 8_000_000,
                VCoreRange::Range3 => 4_000_000,
            },
        );

        let adc_clk = match clk_src {
            AdcClkSrc::Pclk => {
                if clk_ctx.apb2().0 < fadc_limits.0 {
                    panic!("pclk too low to drive adc");
                }

                if clk_ctx.apb2().0 <= fadc_limits.1 {
                    raw.cfgr2.modify(|_, w| unsafe { w.ckmode().bits(0b11) });
                    clk_ctx.apb2().0
                } else if clk_ctx.apb2().0 / 2 <= fadc_limits.1 {
                    raw.cfgr2.modify(|_, w| unsafe { w.ckmode().bits(0b01) });
                    clk_ctx.apb2().0 / 2
                } else if clk_ctx.apb2().0 / 4 <= fadc_limits.1 {
                    raw.cfgr2.modify(|_, w| unsafe { w.ckmode().bits(0b10) });
                    clk_ctx.apb2().0 / 4
                } else {
                    panic!("pclk too high to drive adc");
                }
            }
            AdcClkSrc::Hsi16 => {
                if let Some(f) = clk_ctx.hsi16() {
                    raw.cfgr2.modify(|_, w| unsafe { w.ckmode().bits(0b00) });

                    match f.0 / fadc_limits.1 {
                        1 => {
                            raw.ccr.modify(|_, w| unsafe { w.presc().bits(0b0000) });
                            f.0
                        }
                        2 => {
                            raw.ccr.modify(|_, w| unsafe { w.presc().bits(0b0001) });
                            f.0 / 2
                        }
                        4 => {
                            raw.ccr.modify(|_, w| unsafe { w.presc().bits(0b0010) });
                            f.0 / 4
                        }
                        _ => panic!("your hsi16 is not 16"),
                    }
                } else {
                    panic!("hsi16 not enabled but is selected for adc clk");
                }
            }
        };

        if adc_clk < 3_500_000 {
            // enable the Low Frequency Mode
            raw.ccr.modify(|_, w| w.lfmen().set_bit());
        }

        let adc_clk = adc_clk as f32;

        let n = |n| (n / adc_clk * 1000000000.0) as u32;
        if samp_time.subsec_nanos() < n(1.5) {
            raw.smpr.modify(|_, w| unsafe { w.smpr().bits(0b000) });
        } else if samp_time.subsec_nanos() < n(3.5) {
            raw.smpr.modify(|_, w| unsafe { w.smpr().bits(0b001) });
        } else if samp_time.subsec_nanos() < n(7.5) {
            raw.smpr.modify(|_, w| unsafe { w.smpr().bits(0b010) });
        } else if samp_time.subsec_nanos() < n(12.5) {
            raw.smpr.modify(|_, w| unsafe { w.smpr().bits(0b011) });
        } else if samp_time.subsec_nanos() < n(19.5) {
            raw.smpr.modify(|_, w| unsafe { w.smpr().bits(0b100) });
        } else if samp_time.subsec_nanos() < n(39.5) {
            raw.smpr.modify(|_, w| unsafe { w.smpr().bits(0b101) });
        } else if samp_time.subsec_nanos() < n(79.5) {
            raw.smpr.modify(|_, w| unsafe { w.smpr().bits(0b110) });
        } else if samp_time.subsec_nanos() < n(160.5) {
            raw.smpr.modify(|_, w| unsafe { w.smpr().bits(0b111) });
        } else {
            panic!("sampling time too long to settle");
        }

        let mut adc = Adc {
            adc: raw,
            cal_fact: 0,
            _res: PhantomData,
            _mode: PhantomData,
        };

        adc.calibrate();

        adc
    }

    /// Start conversion
    fn start(&mut self) {
        self.adc().cr.modify(|_, w| w.adstart().set_bit());
    }

    /// Calibrate the ADC, and store its calibration value
    pub fn calibrate(&mut self) {
        // (1) Ensure that ADEN = 0 */
        // (2) Clear ADEN */
        self.adc().cr.modify(|_, w| w.aden().clear_bit());
        while self.adc().cr.read().aden().bit_is_set() {}

        // (3) Set ADCAL=1 */
        self.adc().cr.modify(|_, w| w.adcal().set_bit());

        // (4) Wait until EOCAL=1 */
        while self.adc().isr.read().eocal().bit_is_clear() {}

        // (5) Clear EOCAL */
        self.adc().isr.modify(|_, w| w.eocal().clear_bit());

        self.cal_fact = self.adc().calfact.read().calfact().bits();
    }
}

/*

// Note: In Auto-off mode (AUTOFF=1) the power-on/off phases are performed automatically, by
// hardware and the ADRDY flag is not set.

/// Enable the ADC, powering it on and readying it for use. Not necessary for single-shot mode
/// use.
fn enable(&mut self) {
    // 1. Clear the ADRDY bit in ADC_ISR register by programming this bit to 1.
    self.adc().isr.modify(|_, w| w.adrdy().set_bit());
    // 2. Set ADEN=1 in the ADC_CR register.
    self.adc().cr.modify(|_, w| w.aden().set_bit());
    // 3. Wait until ADRDY=1 in the ADC_ISR register (ADRDY is set after the ADC startup time).
    //    This can be handled by interrupt if the interrupt is enabled by setting the ADRDYIE
    //    bit in the ADC_IER register.
    while self.adc().isr.read().adrdy().bit_is_clear() {}
}

/// Disable the ADC. This does not turn off the internal voltage reference.
fn disable(&mut self) {
    // 1. Check that ADSTART=0 in the ADC_CR register to ensure that no conversion is ongoing.
    //    If required, stop any ongoing conversion by writing 1 to the ADSTP bit in the ADC_CR
    //    register and waiting until this bit is read at 0.
    self.adc().cr.modify(|_, w| w.adstp().set_bit());
    while self.adc().cr.read().adstart().bit_is_set() {}
    // 2. Set ADDIS=1 in the ADC_CR register.
    self.adc().cr.modify(|_, w| w.addis().set_bit());
    // 3. If required by the application, wait until ADEN=0 in the ADC_CR register, indicating
    //    that the ADC is fully disabled (ADDIS is automatically reset once ADEN=0).
    while self.adc().cr.read().aden().bit_is_set() {}
    // 4. Clear the ADRDY bit in ADC_ISR register by programming this bit to 1 (optional).
    self.adc().isr.modify(|_, w| w.adrdy().set_bit());
}

/// Enable ADC Vreg
fn enable_advreg(&mut self) {
    // write ADVREGEN 1
    self.adc().cr.modify(|_, w| w.advregen().set_bit());
}

/// Disable ADC Vreg
fn disable_advreg(&mut self) {
    self.disable();
    // clear ADVREGEN
    self.adc().cr.modify(|_, w| w.advregen().clear_bit());
}
*/
