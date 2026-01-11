//! SARADC HAL for DP32G030.
//!
//! This is a small, idiomatic wrapper around the DP32G030 SARADC peripheral:
//! - uses the PAC (`dp32g030`) for register access
//! - exposes a minimal [`Config`] mirroring the reference C firmware setup
//! - provides typed pins for battery voltage/current channels (A9=CH4, A14=CH9)
//! - offers non-blocking (`nb`) and simple blocking read helpers
//!
//! Reference C firmware: `uv-k5-firmware-custom/driver/adc.c` + `board.c`.

use core::{error, fmt};

use dp32g030 as pac;

use crate::gpio::{Pin, Port};

/// ADC configuration.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Config {
    /// Sampling clock divider for SARADC (SYSCON.CLK_SEL.SARADC_SMPL_CLK_SEL).
    pub sample_clk_div: SampleClkDiv,
    /// Averaging (1/2/4/8 samples).
    pub avg: Avg,
    /// One-shot or continuous.
    pub continuous: bool,
    /// Store converted data in per-channel registers (true) or FIFO (false).
    pub channel_mode: bool,
    /// Internal (true) vs external (false) sampling clock method.
    pub internal_sample_clk: bool,
    /// Internal sample window setting (raw 0..=7).
    pub internal_sample_win: u8,
    /// External sample setup cycles setting (raw 0..=7).
    pub sample_setup: u8,
    /// Trigger source: CPU (false) or external (true).
    pub external_trigger: bool,
    /// Enable DMA reads of FIFO (not used in this HAL; kept for completeness).
    pub dma: bool,
    /// Whether to mark offset calibration as valid.
    pub calib_offset_valid: bool,
    /// Whether to mark KD calibration as valid.
    pub calib_kd_valid: bool,
    /// Channel select bitmask (bit0..bit15 enable CH0..CH15).
    pub channel_mask: u16,
}

impl Config {
    /// Default config matching the UV-K5 reference firmware for battery reads (CH4 + CH9).
    #[inline]
    pub const fn battery_default() -> Self {
        Self {
            sample_clk_div: SampleClkDiv::Div2,
            channel_mask: (1u16 << 4) | (1u16 << 9),
            avg: Avg::Samples8,
            continuous: false,
            channel_mode: true,
            internal_sample_clk: true,
            internal_sample_win: 7, // 15 cycles in HW encoding
            sample_setup: 0,        // 1 cycle in HW encoding
            external_trigger: false,
            dma: false,
            calib_kd_valid: true,
            calib_offset_valid: true,
        }
    }
}

/// SARADC sampling clock divider.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum SampleClkDiv {
    Div1,
    Div2,
    Div4,
    Div8,
}

impl SampleClkDiv {
    #[inline(always)]
    const fn bits(self) -> u8 {
        match self {
            SampleClkDiv::Div1 => 0b00,
            SampleClkDiv::Div2 => 0b01,
            SampleClkDiv::Div4 => 0b10,
            SampleClkDiv::Div8 => 0b11,
        }
    }
}

/// SARADC averaging configuration.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Avg {
    Samples1,
    Samples2,
    Samples4,
    Samples8,
}

impl Avg {
    #[inline(always)]
    const fn bits(self) -> u8 {
        match self {
            Avg::Samples1 => 0b00,
            Avg::Samples2 => 0b01,
            Avg::Samples4 => 0b10,
            Avg::Samples8 => 0b11,
        }
    }
}

/// ADC channels (0..=15).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Channel {
    Ch0 = 0,
    Ch1 = 1,
    Ch2 = 2,
    Ch3 = 3,
    Ch4 = 4,
    Ch5 = 5,
    Ch6 = 6,
    Ch7 = 7,
    Ch8 = 8,
    Ch9 = 9,
    Ch10 = 10,
    Ch11 = 11,
    Ch12 = 12,
    Ch13 = 13,
    Ch14 = 14,
    Ch15 = 15,
}

impl Channel {
    #[inline(always)]
    const fn bit(self) -> u16 {
        1u16 << (self as u8)
    }
}

/// Errors returned by the ADC HAL.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Error {
    /// Invalid configuration (e.g. channel not enabled, bad parameters).
    BadConfig,
    /// Timed out while waiting for end-of-conversion.
    Timeout,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            Error::BadConfig => "ADC bad configuration",
            Error::Timeout => "ADC timeout",
        };
        f.write_str(s)
    }
}

impl error::Error for Error {}

/// Error when assigning a pin to a SARADC channel.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct InvalidPin;

/// A pin configured for SARADC channel 4 (battery voltage), **PA9**.
pub struct Ch4Pin {
    _priv: (),
}

/// A pin configured for SARADC channel 9 (battery current), **PA14**.
pub struct Ch9Pin {
    _priv: (),
}

impl Ch4Pin {
    #[inline]
    pub fn new(pin: Pin<crate::gpio::Disabled>) -> Result<Self, InvalidPin> {
        if pin.port() == Port::A && pin.pin() == 9 {
            Ok(Self { _priv: () })
        } else {
            Err(InvalidPin)
        }
    }
}

impl Ch9Pin {
    #[inline]
    pub fn new(pin: Pin<crate::gpio::Disabled>) -> Result<Self, InvalidPin> {
        if pin.port() == Port::A && pin.pin() == 14 {
            Ok(Self { _priv: () })
        } else {
            Err(InvalidPin)
        }
    }
}

/// ADC peripheral wrapper.
pub struct Adc {
    _saradc: pac::SARADC,
    channel_mask: u16,
}

impl Adc {
    /// Create and configure SARADC.
    ///
    /// This mirrors the reference C firmware:
    /// - enable SARADC clock gate
    /// - set SARADC sampling clock divider in `SYSCON.CLK_SEL`
    /// - set mux in `PORTCON.PORTA_SEL1` for PA9/PA14 if pins provided
    /// - configure ADC_CFG fields
    /// - enable ADC and perform a soft reset
    pub fn new(
        saradc: pac::SARADC,
        syscon: &pac::SYSCON,
        portcon: &pac::PORTCON,
        vbat: Option<Ch4Pin>,
        cur: Option<Ch9Pin>,
        config: Config,
    ) -> Result<Self, Error> {
        let _ = vbat;
        let _ = cur;

        // Enable SARADC clock gate.
        syscon
            .dev_clk_gate()
            .modify(|_, w| w.saradc_clk_gate().set_bit());

        // Sampling clock divider (note: SVD uses bits [10:11]; C firmware notes doc mismatch).
        syscon
            .clk_sel()
            .modify(|_, w| unsafe { w.saradc_smpl_clk_sel().bits(config.sample_clk_div.bits()) });

        // Configure pin mux for CH4/CH9 on PA9/PA14.
        // Encoding from SVD/docs: for PA9 and PA14, SARADC_CHx is function value 0b0101.
        // We only touch the relevant nibbles, leaving other pins untouched.
        if (config.channel_mask & Channel::Ch4.bit()) != 0 {
            set_porta_sel1_nibble(portcon, 9, 0b0101);
            // Match the reference firmware behavior: PA9 is *not* configured as GPIO input.
            // Ensure we don't have digital pad features (IE/PU/PD/OD) fighting the analog net,
            // especially with the high impedance battery divider.
            porta_disable_digital_pad(portcon, 9);
        }
        if (config.channel_mask & Channel::Ch9.bit()) != 0 {
            set_porta_sel1_nibble(portcon, 14, 0b0101);
            porta_disable_digital_pad(portcon, 14);
        }

        let regs = unsafe { &*pac::SARADC::ptr() };

        // Disable ADC before reconfiguring.
        regs.adc_cfg().modify(|_, w| w.adc_en().clear_bit());

        // Program ADC_CFG.
        regs.adc_cfg().write(|w| unsafe {
            w.adc_ch_sel().bits(config.channel_mask);
            w.avg().bits(config.avg.bits());
            w.cont().bit(config.continuous);
            w.smpl_setup().bits(config.sample_setup & 0x7);
            w.adc_mem_mode().bit(config.channel_mode);
            w.adc_smpl_clk().bit(config.internal_sample_clk);
            w.in_smpl_win().bits(config.internal_sample_win & 0x7);
            w.adc_trig().bit(config.external_trigger);
            w.dma_en().bit(config.dma);
            // Enable ADC.
            w.adc_en().set_bit();
            w
        });

        // Calibration valid flags.
        regs.adc_calib_offset()
            .modify(|_, w| w.offset_valid().bit(config.calib_offset_valid));
        regs.adc_calib_kd()
            .modify(|_, w| w.kd_valid().bit(config.calib_kd_valid));

        // Clear all pending interrupt flags.
        regs.adc_if().write(|w| unsafe { w.bits(u32::MAX) });
        // Disable interrupts by default.
        regs.adc_ie().write(|w| unsafe { w.bits(0) });

        // Soft reset: assert (0) then deassert (1) like the C firmware.
        regs.adc_start().modify(|_, w| w.soft_reset().clear_bit());
        regs.adc_start().modify(|_, w| w.soft_reset().set_bit());

        Ok(Self {
            _saradc: saradc,
            channel_mask: config.channel_mask,
        })
    }

    /// Trigger a conversion of all enabled channels (one-shot).
    #[inline(always)]
    pub fn start(&mut self) {
        let regs = unsafe { &*pac::SARADC::ptr() };
        regs.adc_start().modify(|_, w| w.start().set_bit());
    }

    /// Check whether the end-of-conversion flag is set for a channel.
    #[inline(always)]
    pub fn eoc(&self, ch: Channel) -> bool {
        let regs = unsafe { &*pac::SARADC::ptr() };
        match ch {
            Channel::Ch0 => regs.adc_ch0_stat().read().adc_ch_eoc().bit_is_set(),
            Channel::Ch1 => regs.adc_ch1_stat().read().adc_ch_eoc().bit_is_set(),
            Channel::Ch2 => regs.adc_ch2_stat().read().adc_ch_eoc().bit_is_set(),
            Channel::Ch3 => regs.adc_ch3_stat().read().adc_ch_eoc().bit_is_set(),
            Channel::Ch4 => regs.adc_ch4_stat().read().adc_ch_eoc().bit_is_set(),
            Channel::Ch5 => regs.adc_ch5_stat().read().adc_ch_eoc().bit_is_set(),
            Channel::Ch6 => regs.adc_ch6_stat().read().adc_ch_eoc().bit_is_set(),
            Channel::Ch7 => regs.adc_ch7_stat().read().adc_ch_eoc().bit_is_set(),
            Channel::Ch8 => regs.adc_ch8_stat().read().adc_ch_eoc().bit_is_set(),
            Channel::Ch9 => regs.adc_ch9_stat().read().adc_ch_eoc().bit_is_set(),
            Channel::Ch10 => regs.adc_ch10_stat().read().adc_ch_eoc().bit_is_set(),
            Channel::Ch11 => regs.adc_ch11_stat().read().adc_ch_eoc().bit_is_set(),
            Channel::Ch12 => regs.adc_ch12_stat().read().adc_ch_eoc().bit_is_set(),
            Channel::Ch13 => regs.adc_ch13_stat().read().adc_ch_eoc().bit_is_set(),
            Channel::Ch14 => regs.adc_ch14_stat().read().adc_ch_eoc().bit_is_set(),
            Channel::Ch15 => regs.adc_ch15_stat().read().adc_ch_eoc().bit_is_set(),
        }
    }

    /// Non-blocking read (returns `WouldBlock` until EOC is set).
    pub fn try_read(&mut self, ch: Channel) -> nb::Result<u16, Error> {
        if (self.channel_mask & ch.bit()) == 0 {
            return Err(nb::Error::Other(Error::BadConfig));
        }

        if !self.eoc(ch) {
            return Err(nb::Error::WouldBlock);
        }

        Ok(self.read_and_clear(ch))
    }

    /// Blocking read with a bounded busy-wait loop.
    pub fn read_blocking(&mut self, ch: Channel) -> Result<u16, Error> {
        const MAX_SPINS: u32 = 2_000_000;

        self.start();
        for _ in 0..MAX_SPINS {
            if self.eoc(ch) {
                return Ok(self.read_and_clear(ch));
            }
        }
        Err(Error::Timeout)
    }

    #[inline(always)]
    fn read_and_clear(&mut self, ch: Channel) -> u16 {
        let regs = unsafe { &*pac::SARADC::ptr() };

        // Clear EOC by writing 1 to the corresponding ADC_IF bit.
        regs.adc_if()
            .write(|w| unsafe { w.adc_chx_eoc_ist().bits(ch.bit()) });

        // Read data from the per-channel DATA register (12-bit).
        

        match ch {
            Channel::Ch0 => regs.adc_ch0_data().read().adc_ch_data().bits(),
            Channel::Ch1 => regs.adc_ch1_data().read().adc_ch_data().bits(),
            Channel::Ch2 => regs.adc_ch2_data().read().adc_ch_data().bits(),
            Channel::Ch3 => regs.adc_ch3_data().read().adc_ch_data().bits(),
            Channel::Ch4 => regs.adc_ch4_data().read().adc_ch_data().bits(),
            Channel::Ch5 => regs.adc_ch5_data().read().adc_ch_data().bits(),
            Channel::Ch6 => regs.adc_ch6_data().read().adc_ch_data().bits(),
            Channel::Ch7 => regs.adc_ch7_data().read().adc_ch_data().bits(),
            Channel::Ch8 => regs.adc_ch8_data().read().adc_ch_data().bits(),
            Channel::Ch9 => regs.adc_ch9_data().read().adc_ch_data().bits(),
            Channel::Ch10 => regs.adc_ch10_data().read().adc_ch_data().bits(),
            Channel::Ch11 => regs.adc_ch11_data().read().adc_ch_data().bits(),
            Channel::Ch12 => regs.adc_ch12_data().read().adc_ch_data().bits(),
            Channel::Ch13 => regs.adc_ch13_data().read().adc_ch_data().bits(),
            Channel::Ch14 => regs.adc_ch14_data().read().adc_ch_data().bits(),
            Channel::Ch15 => regs.adc_ch15_data().read().adc_ch_data().bits(),
        }
    }
}

// --- PORTCON mux helper ------------------------------------------------------

#[inline(always)]
fn set_porta_sel1_nibble(portcon: &pac::PORTCON, pin: u8, function: u32) {
    debug_assert!((8..=15).contains(&pin));
    let shift = ((pin - 8) as u32) * 4;
    let mask = 0xFu32 << shift;
    portcon
        .porta_sel1()
        .modify(|r, w| unsafe { w.bits((r.bits() & !mask) | ((function & 0xF) << shift)) });
}

/// Disable digital input buffer and pulls on a PORTA pin.
///
/// The stock UV-K5 firmware does *not* enable `PORTA_IE` for PA9/PA14 when used as SARADC
/// channels. Keeping the digital pad disabled avoids biasing the ADC node (battery divider).
#[inline(always)]
fn porta_disable_digital_pad(portcon: &pac::PORTCON, pin: u8) {
    debug_assert!(pin <= 15);
    let bit = 1u32 << (pin as u32);

    // Input enable OFF.
    portcon
        .porta_ie()
        .modify(|r, w| unsafe { w.bits(r.bits() & !bit) });
    // Pull-up OFF.
    portcon
        .porta_pu()
        .modify(|r, w| unsafe { w.bits(r.bits() & !bit) });
    // Pull-down OFF.
    portcon
        .porta_pd()
        .modify(|r, w| unsafe { w.bits(r.bits() & !bit) });
    // Open-drain OFF.
    portcon
        .porta_od()
        .modify(|r, w| unsafe { w.bits(r.bits() & !bit) });
}
