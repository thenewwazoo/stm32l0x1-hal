//! ADC-related things

use core::marker::PhantomData;
use core::time::Duration;

use gpio::Analog;
use gpio::{PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PB0, PB1};
use hal::adc::{Channel, OneShot};
use nb;
use power::{self, VCoreRange};
use rcc::{self, ClockContext};
use stm32l0x1::{adc, ADC};

/// ADC related errors
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

/// ADC resolution trait
///
/// Do not implement!
pub trait Resolution {
    /// Data size of ADC reading
    type Word;
    /// Config bits in register
    const BITS: u8;
}

/// 12-bit resolution
pub struct Res12Bit(());
impl Resolution for Res12Bit {
    type Word = u16;
    const BITS: u8 = 0b00;
}
/// 10-bit resolution
pub struct Res10Bit(());
impl Resolution for Res10Bit {
    type Word = u16;
    const BITS: u8 = 0b01;
}
/// 8-bit resolution
pub struct Res8Bit(());
impl Resolution for Res8Bit {
    type Word = u8;
    const BITS: u8 = 0b10;
}
/// 6-bit resolution
pub struct Res6Bit(());
impl Resolution for Res6Bit {
    type Word = u8;
    const BITS: u8 = 0b11;
}

/// Denotes ADC operating mode
///
/// Do not implement!
pub trait RunMode {
    /// Auto-configure the ADC run mode
    #[doc(hidden)]
    fn cfg(cfgr1: &mut adc::cfgr1::W) -> &mut adc::cfgr1::W;
}

/// Single-ended conversion marker type
pub struct Single(());
impl RunMode for Single {
    fn cfg(w: &mut adc::cfgr1::W) -> &mut adc::cfgr1::W {
        w.cont().clear_bit().discen().clear_bit().autoff().set_bit()
    }
}

/// Continuous readings
pub struct Continuous(());
impl RunMode for Continuous {
    fn cfg(w: &mut adc::cfgr1::W) -> &mut adc::cfgr1::W {
        w.discen().clear_bit().cont().set_bit()
    }
}

/// Sequence of conversions
pub struct Scan<DIR> {
    #[doc(hidden)]
    _dir: PhantomData<DIR>,
}

impl<DIR> RunMode for Scan<DIR>
where
    DIR: ScanDir,
{
    fn cfg(w: &mut adc::cfgr1::W) -> &mut adc::cfgr1::W {
        w.scandir().bit(DIR::DIR)
    }
}

/// Scan direction trait
///
/// Do not implement!
pub trait ScanDir {
    /// Indicates direction; false = up, true = down
    const DIR: bool;
}

/// Scan from 0 to 18
pub struct ScanUp(());
impl ScanDir for ScanUp {
    const DIR: bool = false;
}

/// Scan from 18 to 0
pub struct ScanDown(());
impl ScanDir for ScanDown {
    const DIR: bool = true;
}

/// Discontinuous sequence of conversions
pub struct Discontinuous(());
impl RunMode for Discontinuous {
    fn cfg(w: &mut adc::cfgr1::W) -> &mut adc::cfgr1::W {
        w.cont().clear_bit().discen().set_bit()
    }
}

macro_rules! adc_pin {
    ($PXi:ident, $i:expr) => {
        impl Channel for $PXi<Analog> {
            type ID = u32;
            const CHANNEL: u32 = $i;
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
    /// Single-ended sample has completed
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

// allow dead code because we want to hold the references until I get context stuff implemented
#[allow(dead_code)]
/// Represents the ADC peripheral
pub struct Adc<RES, MODE> where RES: Resolution, MODE: RunMode {
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

impl<RES, MODE> Adc<RES, MODE> where RES: Resolution, MODE: RunMode {
    fn adc(&mut self) -> &mut ADC {
        &mut self.adc
    }
}

impl<WORD, RES, PIN> OneShot<Adc<RES, Single>, RES::Word, PIN> for Adc<RES, Single>
where
    WORD: From<u16>,
    RES: Resolution<Word = WORD>,
    PIN: Channel<ID=u8>,
{
    type Error = Error;

    fn read(&mut self, _pin: &mut PIN) -> nb::Result<RES::Word, Error> {
        let chan = 1 << PIN::CHANNEL;

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
                    .write(|w| unsafe { w.bits(1 << PIN::CHANNEL) });
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
    /// Create a new adc
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
