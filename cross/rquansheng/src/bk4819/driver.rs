use core::cmp::min;

use embedded_hal::delay::DelayNs;

use crate::{
    bk4819_bitbang::{Bk4819, Bk4819Bus},
    bk4819_n::{self, Reg67},
    radio::SquelchThresholds,
};

use super::regs::{
    GpioPin as RegGpioPin, Register_old, REG_07_MODE_CDCSS, REG_07_MODE_CTC1, REG_07_MODE_CTC2,
    REG_24_ENABLE, REG_24_SELECT_DTMF, REG_24_SHIFT_MAX_SYMBOLS, REG_24_SHIFT_THRESHOLD,
    REG_24_SHIFT_UNKNOWN_15, REG_24_SHIFT_UNKNOWN_6, REG_30_DISABLE_MIC_ADC,
    REG_30_DISABLE_PA_GAIN, REG_30_DISABLE_RX_DSP, REG_30_DISABLE_RX_LINK, REG_30_DISABLE_TX_DSP,
    REG_30_DISABLE_UNKNOWN, REG_30_ENABLE_AF_DAC, REG_30_ENABLE_DISC_MODE, REG_30_ENABLE_PA_GAIN,
    REG_30_ENABLE_PLL_VCO, REG_30_ENABLE_RX_DSP, REG_30_ENABLE_RX_LINK, REG_30_ENABLE_TX_DSP,
    REG_30_ENABLE_UNKNOWN, REG_30_ENABLE_VCO_CALIB, REG_51_1050HZ_DETECTION,
    REG_51_1050HZ_NO_DETECTION, REG_51_AUTO_CDCSS_BW_DISABLE, REG_51_AUTO_CDCSS_BW_ENABLE,
    REG_51_AUTO_CTCSS_BW_DISABLE, REG_51_AUTO_CTCSS_BW_ENABLE, REG_51_CDCSS_23_BIT,
    REG_51_DISABLE_CXCSS, REG_51_ENABLE_CXCSS, REG_51_GPIO6_PIN2_NORMAL, REG_51_MODE_CDCSS,
    REG_51_MODE_CTCSS, REG_51_SHIFT_CXCSS_TX_GAIN1, REG_51_TX_CDCSS_POSITIVE, REG_70_ENABLE_TONE1,
    REG_70_ENABLE_TONE2, REG_70_SHIFT_TONE1_TUNING_GAIN, REG_70_SHIFT_TONE2_TUNING_GAIN,
};

/// AF output selection (C enum `BK4819_AF_Type_t`).
#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum AfType {
    Mute = 0,
    Fm = 1,
    Alarm = 2,
    Beep = 3,
    Baseband1 = 4,
    Baseband2 = 5,
    Ctco = 6,
    Am = 7,
    Fsko = 8,
    Unknown3 = 9,
    Unknown4 = 10,
    Unknown5 = 11,
    Unknown6 = 12,
    Unknown7 = 13,
    Unknown8 = 14,
    Unknown9 = 15,
}

/// RX/TX bandwidth preset (C enum `BK4819_FilterBandwidth_t`).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum FilterBandwidth {
    Wide,
    Narrow,
    Narrower,
    U1k7,
}

/// Result of CxCSS scan (C enum `BK4819_CssScanResult_t`).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum CssScanResult {
    NotFound,
    Ctcss,
    Cdcss,
}

/// Compander mode (maps to C `BK4819_SetCompander(mode)`).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum CompanderMode {
    Off = 0,
    Tx = 1,
    Rx = 2,
    TxRx = 3,
}

/// GPIO pins (matches the C driver identifiers).
pub type GpioPin = RegGpioPin;

/// Roger beep mode (C references EEPROM setting).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum RogerMode {
    Off,
    Roger,
    Mdc,
}

/// Board/app hook for enabling/disabling the audio path (speaker/amp).
pub trait AudioPath {
    type Error;
    fn on(&mut self) -> Result<(), Self::Error>;
    fn off(&mut self) -> Result<(), Self::Error>;
}

/// High-level driver, owning a `Bk4819` instance plus a small amount of state
/// that was global in the C implementation (GPIO out shadow + rx idle flag).
pub struct Bk4819Driver<BUS> {
    bk: Bk4819<BUS>,
    gpio_out_state: u16,
    /// If true, radio is considered asleep/not listening (C global `gRxIdleMode`).
    pub rx_idle_mode: bool,
}

impl<BUS> Bk4819Driver<BUS>
where
    BUS: Bk4819Bus,
{
    /// Default microphone gain (0.5 dB/step, 0..=31).
    ///
    /// Reference firmware uses 5 presets: {3, 8, 16, 24, 31}. We choose the mid preset (16).
    pub const DEFAULT_MIC_GAIN: u8 = 16;

    pub const fn new(bk: Bk4819<BUS>) -> Self {
        Self {
            bk,
            gpio_out_state: 0,
            rx_idle_mode: false,
        }
    }

    #[inline]
    pub fn free(self) -> Bk4819<BUS> {
        self.bk
    }

    #[inline]
    pub fn bk_mut(&mut self) -> &mut Bk4819<BUS> {
        &mut self.bk
    }

    #[inline]
    #[deprecated]
    pub fn read_register_raw(&mut self, reg: Register_old) -> Result<u16, BUS::Error> {
        self.bk.read_reg_raw(reg.as_u8())
    }

    #[inline]
    #[deprecated]
    pub fn write_register_raw(&mut self, reg: Register_old, value: u16) -> Result<(), BUS::Error> {
        self.bk.write_reg_raw(reg.as_u8(), value)
    }

    pub fn write_register<R: bk4819_n::Bk4819Register>(
        &mut self,
        reg: R,
    ) -> Result<(), BUS::Error> {
        self.bk.write_reg(reg)
    }

    pub fn read_register<R: bk4819_n::Bk4819Register>(&mut self) -> Result<R, BUS::Error> {
        self.bk.read_reg::<R>()
    }

    /// Port of `scale_freq()` from the C firmware.
    #[inline]
    pub fn scale_freq(freq_hz: u16) -> u16 {
        // C:
        // (((freq * 1353245) + (1 << 16)) >> 17)
        ((((freq_hz as u32) * 1_353_245u32) + (1u32 << 16)) >> 17) as u16
    }

    // --- Core init / AGC ----------------------------------------------------

    /// Port of `BK4819_Init()`.
    pub fn init(&mut self) -> Result<(), BUS::Error> {
        use bk4819_n::{Reg00, Reg36, Reg37};
        // Soft reset
        self.write_register(Reg00::new().with_soft_reset(bk4819_n::Reg00SoftReset::Reset))?;
        self.write_register(Reg00::new().with_soft_reset(bk4819_n::Reg00SoftReset::Normal))?;

        //self.write_register(Register::Reg37, 0x1D0F)?; // 0b0 001 1101 00001111
        self.write_register(
            Reg37::new()
                .with_bg_en(true)
                .with_xtal_en(true)
                .with_dsp_en(true)
                .with_undocumented_0(true)
                .with_pll_ldo_sel(true)
                .with_vco_ldo_sel(true)
                .with_ana_ldo_sel(true)
                .with_dsp_volt(1),
        )?;

        self.write_register(Reg36::new().with_pa_gain2(0b010).with_pa_gain1(0b100))?;

        self.init_agc(false)?;
        self.set_agc(true)?;

        // REG_19: <15> MIC AGC 1=disable 0=enable
        //self.write_register(Register::Reg19, 0b0001_0000_0100_0001)?;
        self.write_register(
            bk4819_n::Reg19::new()
                .with_mic_agc_disable(false)
                .with_undocumented_1(0b001000001000001),
        )?;

        // REG_7D: mic gain tuning (0.5 dB/step, 0..=31) in the low 5 bits.
        // No EEPROM is available here, so use a sensible default.
        //self.set_mic_gain(Self::DEFAULT_MIC_GAIN)?;

        self.set_mic_gain(Self::DEFAULT_MIC_GAIN)?;

        // REG_48 .. RX AF level (see C comments)
        //self.write_register(Register::Reg48, (11u16 << 12) | (58u16 << 4) | 8u16)?;
        self.write_register(
            bk4819_n::Reg48::new()
                .with_af_dac_gain(8)
                .with_afrx_gain2(58)
                .with_afrx_gain1(0)
                .with_undocumented(11),
        )?;

        // DTMF coefficients table
        const DTMF_COEFFS: [u16; 16] = [
            111, 107, 103, 98, 80, 71, 58, 44, 65, 55, 37, 23, 228, 203, 181, 159,
        ];
        for (i, &c) in DTMF_COEFFS.iter().enumerate() {
            self.write_register(
                bk4819_n::Reg09::new()
                    .with_coefficient(c as u8)
                    .with_symbol_number(i as u8),
            )?;
        }

        //self.write_register(crate::bk4819::regs::Register::Reg1F, 0x5454)?;
        self.write_register(
            bk4819_n::Reg1F::new()
                .with_pll_cp_bit(4)
                .with_undocumented(0b10101000101),
        )?;

        //self.write_register(Register::Reg3E, 0xA037)?;
        self.write_register(bk4819_n::Reg3E::new().with_band_thresh(0xA037))?;

        self.gpio_out_state = 0x9000;
        //self.write_register(Register::Reg33, self.gpio_out_state)?;
        self.write_register(
            bk4819_n::Reg33::new()
                .with_gpio_out_disable((self.gpio_out_state >> 8) as u8)
                .with_gpio_out_value(0x00),
        )?;
        //self.write_register(Register::Reg3F, 0x0000)?;
        self.write_register(bk4819_n::Reg3F::new())?;

        Ok(())
    }

    /// Set microphone gain tuning (BK4819 REG_7D, low 5 bits).
    ///
    /// `gain` is in 0.5 dB/step, 0..=31.
    pub fn set_mic_gain(&mut self, gain: u8) -> Result<(), BUS::Error> {
        //self.write_register(Register::Reg7D, 0xE940 | gain)
        self.write_register(
            bk4819_n::Reg7D::new()
                .with_mic_sens(gain)
                .with_undocumented(0b11101001010),
        )?;

        Ok(())
    }

    /// Port of `BK4819_SetAGC(enable)`.
    pub fn set_agc(&mut self, enable: bool) -> Result<(), BUS::Error> {
        // REG_7E layout is modeled in `bk4819_n::Reg7E`:
        // - bit15: AGC fix mode (1=fix => AGC off, 0=auto => AGC on)
        // - bits14:12: AGC fix index
        // Everything else (including undocumented bits) must be preserved.
        let r7e: bk4819_n::Reg7E = self.read_register::<bk4819_n::Reg7E>()?;

        // C uses: if(!(regVal & (1<<15)) == enable) return;
        let currently_enabled = !r7e.agc_fix_mode();
        if currently_enabled == enable {
            return Ok(());
        }

        let next = r7e
            .with_agc_fix_mode(!enable) // 0=auto (AGC on), 1=fix (AGC off)
            .with_agc_fix_index(3); // fix index (matches reference firmware)

        self.write_register(next)
    }

    /// Port of `BK4819_InitAGC(amModulation)`.
    pub fn init_agc(&mut self, am_modulation: bool) -> Result<(), BUS::Error> {
        self.write_register_raw(Register_old::Reg13, 0x03BE)?;
        self.write_register_raw(Register_old::Reg12, 0x037B)?;
        self.write_register_raw(Register_old::Reg11, 0x027B)?;
        self.write_register_raw(Register_old::Reg10, 0x007A)?;

        if am_modulation {
            self.write_register_raw(Register_old::Reg14, 0x0000)?;
            self.write_register_raw(Register_old::Reg49, (50u16 << 7) | 32u16)?;
        } else {
            self.write_register_raw(Register_old::Reg14, 0x0019)?;
            self.write_register_raw(Register_old::Reg49, (84u16 << 7) | 56u16)?;
        }

        self.write_register_raw(Register_old::Reg7B, 0x8420)?;
        Ok(())
    }

    // --- GPIO ---------------------------------------------------------------

    /// Port of `BK4819_ToggleGpioOut(pin, set)`.
    pub fn toggle_gpio_out(&mut self, pin: GpioPin, set: bool) -> Result<(), BUS::Error> {
        let bit = 0x40u16 >> (pin as u8);
        if set {
            self.gpio_out_state |= bit;
        } else {
            self.gpio_out_state &= !bit;
        }
        self.write_register_raw(Register_old::Reg33, self.gpio_out_state)
    }

    // --- CTCSS / CDCSS ------------------------------------------------------

    /// Port of `BK4819_SetCDCSSCodeWord(code_word)`.
    pub fn set_cdcss_code_word(&mut self, code_word: u32) -> Result<(), BUS::Error> {
        self.write_register_raw(
            Register_old::Reg51,
            REG_51_ENABLE_CXCSS
                | REG_51_GPIO6_PIN2_NORMAL
                | REG_51_TX_CDCSS_POSITIVE
                | REG_51_MODE_CDCSS
                | REG_51_CDCSS_23_BIT
                | REG_51_1050HZ_NO_DETECTION
                | REG_51_AUTO_CDCSS_BW_ENABLE
                | REG_51_AUTO_CTCSS_BW_ENABLE
                | (51u16 << REG_51_SHIFT_CXCSS_TX_GAIN1),
        )?;

        // C: BK4819_REG_07_MODE_CTC1 | 2775
        self.write_register_raw(Register_old::Reg07, REG_07_MODE_CTC1 | 2775u16)?;

        self.write_register_raw(Register_old::Reg08, (code_word as u16) & 0x0FFF)?;
        self.write_register_raw(
            Register_old::Reg08,
            (1u16 << 15) | ((code_word >> 12) as u16 & 0x0FFF),
        )?;
        Ok(())
    }

    /// Port of `BK4819_SetCTCSSFrequency(freq_0p1Hz)`.
    ///
    /// The C implementation expects the input in **0.1Hz units** (Hz * 10).
    pub fn set_ctcss_frequency_0p1hz(&mut self, freq_0p1_hz: u32) -> Result<(), BUS::Error> {
        let config = if freq_0p1_hz == 2625 {
            // Enables 1050Hz detection mode (1050/4 = 262.5Hz)
            0x944A
        } else {
            0x904A
        };
        self.write_register_raw(Register_old::Reg51, config)?;

        // REG_07 = mode CTC1 + (freq * 20.64888) for XTAL 13/26M
        // C: (((FreqControlWord * 206488u) + 50000u) / 100000u)
        let word = (((freq_0p1_hz * 206_488u32) + 50_000u32) / 100_000u32) as u16;
        self.write_register_raw(Register_old::Reg07, REG_07_MODE_CTC1 | word)?;
        Ok(())
    }

    /// Port of `BK4819_SetTailDetection(freq_10Hz)`, where `freq_10Hz = Hz * 10`.
    pub fn set_tail_detection_10hz(&mut self, freq_10hz: u32) -> Result<(), BUS::Error> {
        // C: (253910 + freq/2) / freq
        let word = ((253_910u32 + (freq_10hz / 2)) / freq_10hz) as u16;
        self.write_register_raw(Register_old::Reg07, REG_07_MODE_CTC2 | word)
    }

    // --- VOX ----------------------------------------------------------------

    /// Port of `BK4819_EnableVox(enable_th, disable_th)`.
    pub fn enable_vox(
        &mut self,
        enable_threshold: u16,
        disable_threshold: u16,
    ) -> Result<(), BUS::Error> {
        let reg31 = self.read_register_raw(Register_old::Reg31)?;
        self.write_register_raw(Register_old::Reg46, 0xA000 | (enable_threshold & 0x07FF))?;
        self.write_register_raw(Register_old::Reg79, 0x1800 | (disable_threshold & 0x07FF))?;
        self.write_register_raw(Register_old::Reg7A, 0x289A)?; // disable delay ~= 640ms
        self.write_register_raw(Register_old::Reg31, reg31 | (1u16 << 2))
    }

    /// Port of `BK4819_DisableVox()`.
    pub fn disable_vox(&mut self) -> Result<(), BUS::Error> {
        let v = self.read_register_raw(Register_old::Reg31)?;
        self.write_register_raw(Register_old::Reg31, v & 0xFFFB)
    }

    // --- Bandwidth / PA / Frequency ----------------------------------------

    /// Port of `BK4819_SetFilterBandwidth(bw, weak_no_different)`.
    pub fn set_filter_bandwidth(
        &mut self,
        bandwidth: FilterBandwidth,
        weak_no_different: bool,
    ) -> Result<(), BUS::Error> {
        let val: u16 = match bandwidth {
            FilterBandwidth::Wide => {
                let mut v = (4u16 << 12) | (6u16 << 6) | (2u16 << 4) | (1u16 << 3);
                v |= if weak_no_different {
                    4u16 << 9
                } else {
                    2u16 << 9
                };
                v
            }
            FilterBandwidth::Narrow => {
                let mut v = (4u16 << 12) | (1u16 << 3);
                v |= if weak_no_different {
                    4u16 << 9
                } else {
                    2u16 << 9
                };
                v
            }
            FilterBandwidth::Narrower => {
                (2u16 << 12) | (2u16 << 9) | (1u16 << 6) | (1u16 << 4) | (1u16 << 3)
            }
            FilterBandwidth::U1k7 => (1u16 << 6) | (1u16 << 4) | (1u16 << 3),
        };
        self.write_register_raw(Register_old::Reg43, val)
    }

    /// Port of `BK4819_SetupPowerAmplifier(bias, frequency)`.
    ///
    pub fn setup_power_amplifier(&mut self, bias: u8, frequency_hz: u32) -> Result<(), BUS::Error> {
        let gain: u8 = if frequency_hz < 280_000_000 {
            1u8 << 3
        } else {
            (4u8 << 3) | 2u8
        };
        let enable: u8 = 1;
        self.write_register_raw(
            Register_old::Reg36,
            ((bias as u16) << 8) | ((enable as u16) << 7) | (gain as u16),
        )
    }

    /// Port of `BK4819_SetFrequency(freq)`.
    ///
    pub fn set_frequency(&mut self, frequency_hz: u32) -> Result<(), BUS::Error> {
        let frequency_10hz = frequency_hz / 10;

        self.write_register(bk4819_n::Reg38::new().with_freq_lo(frequency_10hz as u16))?;
        self.write_register(bk4819_n::Reg39::new().with_freq_hi((frequency_10hz >> 16) as u16))?;
        Ok(())
    }

    /// Port of `BK4819_PickRXFilterPathBasedOnFrequency(freq)`.
    pub fn pick_rx_filter_path_based_on_frequency(
        &mut self,
        frequency_hz: u32,
    ) -> Result<(), BUS::Error> {
        if frequency_hz < 280_000_000 {
            self.toggle_gpio_out(GpioPin::Gpio4Pin32VhfLna, true)?;
            self.toggle_gpio_out(GpioPin::Gpio3Pin31UhfLna, false)?;
        } else if frequency_hz == 0xFFFF_FFFF {
            self.toggle_gpio_out(GpioPin::Gpio4Pin32VhfLna, false)?;
            self.toggle_gpio_out(GpioPin::Gpio3Pin31UhfLna, false)?;
        } else {
            self.toggle_gpio_out(GpioPin::Gpio4Pin32VhfLna, false)?;
            self.toggle_gpio_out(GpioPin::Gpio3Pin31UhfLna, true)?;
        }
        Ok(())
    }

    // --- Squelch / RX on ----------------------------------------------------

    /// Port of `BK4819_SetupSquelch(...)`.
    #[allow(clippy::too_many_arguments)]
    pub fn setup_squelch(&mut self, thresholds: SquelchThresholds) -> Result<(), BUS::Error> {
        // Disable tones
        self.write_register_raw(Register_old::Reg70, 0)?;

        // Glitch threshold for squelch close
        self.write_register_raw(Register_old::Reg4D, 0xA000 | thresholds.close_glitch as u16)?;

        // Squelch open/close delay + glitch open threshold
        self.write_register_raw(
            Register_old::Reg4E,
            (1u16 << 14) | (5u16 << 11) | (6u16 << 9) | thresholds.open_glitch as u16,
        )?;

        // Ex-noise close/open
        self.write_register_raw(
            Register_old::Reg4F,
            ((thresholds.close_noise as u16) << 8) | (thresholds.open_noise as u16),
        )?;

        // RSSI open/close (0.5dB/step)
        self.write_register_raw(
            Register_old::Reg78,
            ((thresholds.open_rssi as u16) << 8) | (thresholds.close_rssi as u16),
        )?;

        self.set_af(AfType::Mute)?;
        self.rx_turn_on()
    }

    /// Port of `BK4819_SetAF(AF)`.
    pub fn set_af(&mut self, af: AfType) -> Result<(), BUS::Error> {
        self.write_register_raw(
            Register_old::Reg47,
            (6u16 << 12) | ((af as u16) << 8) | (1u16 << 6),
        )
    }

    /// Port of `BK4819_RX_TurnOn()`.
    pub fn rx_turn_on(&mut self) -> Result<(), BUS::Error> {
        self.write_register_raw(Register_old::Reg37, 0x1F0F)?;
        self.write_register_raw(Register_old::Reg30, 0x0000)?;
        self.write_register_raw(
            Register_old::Reg30,
            REG_30_ENABLE_VCO_CALIB
                | REG_30_DISABLE_UNKNOWN
                | REG_30_ENABLE_RX_LINK
                | REG_30_ENABLE_AF_DAC
                | REG_30_ENABLE_DISC_MODE
                | REG_30_ENABLE_PLL_VCO
                | REG_30_DISABLE_PA_GAIN
                | REG_30_DISABLE_MIC_ADC
                | REG_30_DISABLE_TX_DSP
                | REG_30_ENABLE_RX_DSP,
        )
    }

    // --- Scramble / Compander ----------------------------------------------

    pub fn disable_scramble(&mut self) -> Result<(), BUS::Error> {
        let v = self.read_register_raw(Register_old::Reg31)?;
        self.write_register_raw(Register_old::Reg31, v & !(1u16 << 1))
    }

    pub fn enable_scramble(&mut self, ty: u8) -> Result<(), BUS::Error> {
        let v = self.read_register_raw(Register_old::Reg31)?;
        self.write_register_raw(Register_old::Reg31, v | (1u16 << 1))?;
        self.write_register_raw(
            Register_old::Reg71,
            0x68DCu16.wrapping_add((ty as u16) * 1032),
        )
    }

    pub fn compander_enabled(&mut self) -> Result<bool, BUS::Error> {
        Ok((self.read_register_raw(Register_old::Reg31)? & (1u16 << 3)) != 0)
    }

    pub fn set_compander(&mut self, mode: CompanderMode) -> Result<(), BUS::Error> {
        let r31 = self.read_register_raw(Register_old::Reg31)?;
        if mode == CompanderMode::Off {
            self.write_register_raw(Register_old::Reg31, r31 & !(1u16 << 3))?;
            return Ok(());
        }

        let compress_ratio: u16 = if mode == CompanderMode::Tx || mode == CompanderMode::TxRx {
            2
        } else {
            0
        };
        let compress_0db: u16 = 86;
        let compress_noise_db: u16 = 64;
        self.write_register_raw(
            Register_old::Reg29,
            (compress_ratio << 14) | (compress_0db << 7) | compress_noise_db,
        )?;

        let expand_ratio: u16 = if mode == CompanderMode::Rx || mode == CompanderMode::TxRx {
            1
        } else {
            0
        };
        let expand_0db: u16 = 86;
        let expand_noise_db: u16 = 56;
        self.write_register_raw(
            Register_old::Reg28,
            (expand_ratio << 14) | (expand_0db << 7) | expand_noise_db,
        )?;

        self.write_register_raw(Register_old::Reg31, r31 | (1u16 << 3))
    }

    // --- DTMF ---------------------------------------------------------------

    pub fn disable_dtmf(&mut self) -> Result<(), BUS::Error> {
        self.write_register_raw(Register_old::Reg24, 0)
    }

    pub fn enable_dtmf(&mut self) -> Result<(), BUS::Error> {
        self.write_register_raw(Register_old::Reg21, 0x06D8)?;
        let threshold: u16 = 130;
        self.write_register_raw(
            Register_old::Reg24,
            (1u16 << REG_24_SHIFT_UNKNOWN_15)
                | (threshold << REG_24_SHIFT_THRESHOLD)
                | (1u16 << REG_24_SHIFT_UNKNOWN_6)
                | REG_24_ENABLE
                | REG_24_SELECT_DTMF
                | (15u16 << REG_24_SHIFT_MAX_SYMBOLS),
        )
    }

    /// Port of `BK4819_PlayDTMF(code)`; assumes DTMF TX is already configured.
    pub fn play_dtmf(&mut self, code: char) -> Result<(), BUS::Error> {
        #[derive(Copy, Clone)]
        struct Pair {
            t1: u16,
            t2: u16,
        }
        const TONES: [Pair; 16] = [
            Pair { t1: 941, t2: 1336 },
            Pair { t1: 697, t2: 1209 },
            Pair { t1: 697, t2: 1336 },
            Pair { t1: 697, t2: 1477 },
            Pair { t1: 770, t2: 1209 },
            Pair { t1: 770, t2: 1336 },
            Pair { t1: 770, t2: 1477 },
            Pair { t1: 852, t2: 1209 },
            Pair { t1: 852, t2: 1336 },
            Pair { t1: 852, t2: 1477 },
            Pair { t1: 697, t2: 1633 },
            Pair { t1: 770, t2: 1633 },
            Pair { t1: 852, t2: 1633 },
            Pair { t1: 941, t2: 1633 },
            Pair { t1: 941, t2: 1209 },
            Pair { t1: 941, t2: 1477 },
        ];

        let selected: Option<Pair> = match code {
            '0'..='9' => Some(TONES[(code as u8 - b'0') as usize]),
            'A'..='D' => Some(TONES[10 + (code as u8 - b'A') as usize]),
            '*' => Some(TONES[14]),
            '#' => Some(TONES[15]),
            _ => None,
        };

        if let Some(p) = selected {
            // C uses 103244/10000 scaling here (different from scale_freq()).
            let w1 = (((p.t1 as u32) * 103_244u32) + 5_000u32) / 10_000u32;
            let w2 = (((p.t2 as u32) * 103_244u32) + 5_000u32) / 10_000u32;
            self.write_register_raw(Register_old::Reg71, w1 as u16)?;
            self.write_register_raw(Register_old::Reg72, w2 as u16)?;
        }
        Ok(())
    }

    pub fn enter_dtmf_tx(&mut self, local_loopback: bool) -> Result<(), BUS::Error> {
        const DTMF_TONE1_GAIN: u16 = 65;
        const DTMF_TONE2_GAIN: u16 = 93;
        self.enable_dtmf()?;
        self.enter_tx_mute()?;
        self.set_af(if local_loopback {
            AfType::Beep
        } else {
            AfType::Mute
        })?;
        self.write_register_raw(
            Register_old::Reg70,
            REG_70_ENABLE_TONE1
                | (DTMF_TONE1_GAIN << REG_70_SHIFT_TONE1_TUNING_GAIN)
                | REG_70_ENABLE_TONE2
                | (DTMF_TONE2_GAIN << REG_70_SHIFT_TONE2_TUNING_GAIN),
        )?;
        self.enable_tx_link()
    }

    pub fn exit_dtmf_tx(&mut self, keep_muted: bool) -> Result<(), BUS::Error> {
        self.enter_tx_mute()?;
        self.set_af(AfType::Mute)?;
        self.write_register_raw(Register_old::Reg70, 0x0000)?;
        self.disable_dtmf()?;
        self.write_register_raw(Register_old::Reg30, 0xC1FE)?;
        if !keep_muted {
            self.exit_tx_mute()?;
        }
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    pub fn play_dtmf_string<D: DelayNs>(
        &mut self,
        s: &str,
        delay_first: bool,
        first_persist_ms: u16,
        hash_persist_ms: u16,
        code_persist_ms: u16,
        code_internal_ms: u16,
        delay: &mut D,
    ) -> Result<(), BUS::Error> {
        for (i, ch) in s.chars().enumerate() {
            self.play_dtmf(ch)?;
            self.exit_tx_mute()?;

            let d = if delay_first && i == 0 {
                first_persist_ms
            } else if ch == '*' || ch == '#' {
                hash_persist_ms
            } else {
                code_persist_ms
            };
            delay.delay_ms(d as u32);

            self.enter_tx_mute()?;
            delay.delay_ms(code_internal_ms as u32);
        }
        Ok(())
    }

    // --- Tone / TX link / mute ---------------------------------------------

    pub fn enter_tx_mute(&mut self) -> Result<(), BUS::Error> {
        self.write_register_raw(Register_old::Reg50, 0xBB20)
    }

    pub fn exit_tx_mute(&mut self) -> Result<(), BUS::Error> {
        self.write_register_raw(Register_old::Reg50, 0x3B20)
    }

    /// Port of `BK4819_EnableTXLink()`.
    pub fn enable_tx_link(&mut self) -> Result<(), BUS::Error> {
        self.write_register_raw(
            Register_old::Reg30,
            REG_30_ENABLE_VCO_CALIB
                | REG_30_ENABLE_UNKNOWN
                | REG_30_DISABLE_RX_LINK
                | REG_30_ENABLE_AF_DAC
                | REG_30_ENABLE_DISC_MODE
                | REG_30_ENABLE_PLL_VCO
                | REG_30_ENABLE_PA_GAIN
                | REG_30_DISABLE_MIC_ADC
                | REG_30_ENABLE_TX_DSP
                | REG_30_DISABLE_RX_DSP,
        )
    }

    /// Port of `BK4819_PlayTone(freq, tuningGainSwitch)`.
    pub fn play_tone(
        &mut self,
        frequency_hz: u16,
        tuning_gain_switch: bool,
    ) -> Result<(), BUS::Error> {
        let mut cfg: u16 = REG_70_ENABLE_TONE1;
        self.enter_tx_mute()?;
        self.set_af(AfType::Beep)?;

        let gain: u16 = if !tuning_gain_switch { 96 } else { 28 };
        cfg |= gain << REG_70_SHIFT_TONE1_TUNING_GAIN;
        self.write_register_raw(Register_old::Reg70, cfg)?;

        self.write_register_raw(Register_old::Reg30, 0x0000)?;
        self.write_register_raw(
            Register_old::Reg30,
            REG_30_ENABLE_AF_DAC | REG_30_ENABLE_DISC_MODE | REG_30_ENABLE_TX_DSP,
        )?;
        self.write_register_raw(Register_old::Reg71, Self::scale_freq(frequency_hz))
    }

    /// Port of `BK4819_PlaySingleTone(...)`.
    pub fn play_single_tone<D: DelayNs, A: AudioPath>(
        &mut self,
        tone_hz: u32,
        delay_ms: u32,
        level: u8,
        play_speaker: bool,
        delay: &mut D,
        audio: &mut A,
    ) -> Result<(), BUS::Error> {
        self.enter_tx_mute()?;
        if play_speaker {
            let _ = audio.on();
            self.set_af(AfType::Beep)?;
        } else {
            self.set_af(AfType::Mute)?;
        }

        self.write_register_raw(
            Register_old::Reg70,
            REG_70_ENABLE_TONE1 | (((level & 0x7F) as u16) << REG_70_SHIFT_TONE1_TUNING_GAIN),
        )?;

        self.enable_tx_link()?;
        delay.delay_ms(50);

        self.write_register_raw(Register_old::Reg71, Self::scale_freq(tone_hz as u16))?;
        self.exit_tx_mute()?;

        delay.delay_ms(delay_ms);
        self.enter_tx_mute()?;

        if play_speaker {
            let _ = audio.off();
            self.set_af(AfType::Mute)?;
        }

        self.write_register_raw(Register_old::Reg70, 0x0000)?;
        self.write_register_raw(Register_old::Reg30, 0xC1FE)?;
        self.exit_tx_mute()
    }

    /// Port of `BK4819_TransmitTone(localLoopback, freqHz)`.
    pub fn transmit_tone<D: DelayNs>(
        &mut self,
        local_loopback: bool,
        frequency_hz: u32,
        delay: &mut D,
    ) -> Result<(), BUS::Error> {
        self.enter_tx_mute()?;
        self.write_register_raw(
            Register_old::Reg70,
            REG_70_ENABLE_TONE1 | (66u16 << REG_70_SHIFT_TONE1_TUNING_GAIN),
        )?;
        self.write_register_raw(Register_old::Reg71, Self::scale_freq(frequency_hz as u16))?;
        self.set_af(if local_loopback {
            AfType::Beep
        } else {
            AfType::Mute
        })?;
        self.enable_tx_link()?;
        delay.delay_ms(50);
        self.exit_tx_mute()
    }

    // --- Power states -------------------------------------------------------

    pub fn sleep(&mut self) -> Result<(), BUS::Error> {
        self.write_register_raw(Register_old::Reg30, 0x0000)?;
        self.write_register_raw(Register_old::Reg37, 0x1D00)
    }

    pub fn idle(&mut self) -> Result<(), BUS::Error> {
        self.write_register_raw(Register_old::Reg30, 0x0000)
    }

    pub fn turns_off_tones_turns_on_rx(&mut self) -> Result<(), BUS::Error> {
        self.write_register_raw(Register_old::Reg70, 0x0000)?;
        self.set_af(AfType::Mute)?;
        self.exit_tx_mute()?;
        self.write_register_raw(Register_old::Reg30, 0x0000)?;
        self.write_register_raw(
            Register_old::Reg30,
            REG_30_ENABLE_VCO_CALIB
                | REG_30_ENABLE_RX_LINK
                | REG_30_ENABLE_AF_DAC
                | REG_30_ENABLE_DISC_MODE
                | REG_30_ENABLE_PLL_VCO
                | REG_30_ENABLE_RX_DSP,
        )
    }

    pub fn exit_bypass(&mut self) -> Result<(), BUS::Error> {
        self.set_af(AfType::Mute)?;
        let reg_val = self.read_register_raw(Register_old::Reg7E)?;
        self.write_register_raw(
            Register_old::Reg7E,
            (reg_val & !(0b111u16 << 3)) | (5u16 << 3),
        )
    }

    pub fn prepare_transmit(&mut self) -> Result<(), BUS::Error> {
        self.exit_bypass()?;
        self.exit_tx_mute()?;
        self.tx_on_beep()
    }

    pub fn tx_on_beep(&mut self) -> Result<(), BUS::Error> {
        self.write_register_raw(Register_old::Reg37, 0x1D0F)?;
        self.write_register_raw(Register_old::Reg52, 0x028F)?;
        self.write_register_raw(Register_old::Reg30, 0x0000)?;
        self.write_register_raw(Register_old::Reg30, 0xC1FE)
    }

    pub fn exit_sub_au(&mut self) -> Result<(), BUS::Error> {
        self.write_register_raw(Register_old::Reg51, 0x0000)
    }

    pub fn conditional_rx_turn_on_and_gpio0_enable(&mut self) -> Result<(), BUS::Error> {
        if self.rx_idle_mode {
            self.toggle_gpio_out(GpioPin::Gpio0Pin28RxEnable, true)?;
            self.rx_turn_on()?;
        }
        Ok(())
    }

    // --- Tails --------------------------------------------------------------

    pub fn gen_tail(&mut self, tail: u8) -> Result<(), BUS::Error> {
        match tail {
            0 => self.write_register_raw(Register_old::Reg52, 0x828F)?,
            1 => self.write_register_raw(Register_old::Reg52, 0xA28F)?,
            2 => self.write_register_raw(Register_old::Reg52, 0xC28F)?,
            3 => self.write_register_raw(Register_old::Reg52, 0xE28F)?,
            4 => self.write_register_raw(Register_old::Reg07, 0x046F)?,
            _ => {}
        }
        Ok(())
    }

    pub fn play_cdcss_tail(&mut self) -> Result<(), BUS::Error> {
        self.gen_tail(0)?;
        self.write_register_raw(Register_old::Reg51, 0x804A)
    }

    pub fn play_ctcss_tail(&mut self) -> Result<(), BUS::Error> {
        // C uses optional phase shift; default is 55Hz
        self.gen_tail(4)?;
        self.write_register_raw(Register_old::Reg51, 0x904A)
    }

    // --- Indicators / measurements -----------------------------------------

    pub fn get_rssi(&mut self) -> Result<u16, BUS::Error> {
        let r67: Reg67 = self.read_register::<Reg67>()?;
        Ok(r67.rssi())
    }

    pub fn get_rssi_dbm(&mut self) -> Result<i16, BUS::Error> {
        let rssi = self.get_rssi()? as i16;
        Ok((rssi / 2) - 160)
    }

    pub fn get_glitch_indicator(&mut self) -> Result<u8, BUS::Error> {
        Ok((self.read_register_raw(Register_old::Reg63)? & 0x00FF) as u8)
    }

    pub fn get_ex_noise_indicator(&mut self) -> Result<u8, BUS::Error> {
        Ok((self.read_register_raw(Register_old::Reg65)? & 0x007F) as u8)
    }

    pub fn get_voice_amplitude_out(&mut self) -> Result<u16, BUS::Error> {
        self.read_register_raw(Register_old::Reg64)
    }

    pub fn get_af_tx_rx(&mut self) -> Result<u8, BUS::Error> {
        Ok((self.read_register_raw(Register_old::Reg6F)? & 0x003F) as u8)
    }

    /// Port of `BK4819_GetRxGain_dB()`.
    pub fn get_rx_gain_db(&mut self) -> Result<i8, BUS::Error> {
        let reg7e = self.read_register_raw(Register_old::Reg7E)?;
        let gain_idx_raw = ((reg7e >> 12) & 0x7) as i8;
        let gain_idx = if gain_idx_raw >= 4 {
            gain_idx_raw - 8
        } else {
            gain_idx_raw
        };

        let gain_reg_addr: u8 = if gain_idx < 0 {
            Register_old::Reg14.as_u8()
        } else {
            (Register_old::Reg10.as_u8()).wrapping_add(gain_idx as u8)
        };
        let agc_gain_reg = self.bk.read_reg_raw(gain_reg_addr)?;

        let pga = (agc_gain_reg & 0b111) as usize;
        let mixer = ((agc_gain_reg >> 3) & 0b11) as usize;
        let lna = ((agc_gain_reg >> 5) & 0b111) as usize;
        let lna_s = ((agc_gain_reg >> 8) & 0b11) as usize;

        const LNA_SHORT: [i8; 4] = [-28, -24, -19, 0];
        const LNA: [i8; 8] = [-24, -19, -14, -9, -6, -4, -2, 0];
        const MIXER: [i8; 4] = [-8, -6, -3, 0];
        const PGA: [i8; 8] = [-33, -27, -21, -15, -9, -6, -3, 0];

        Ok(LNA_SHORT[lna_s] + LNA[lna] + MIXER[mixer] + PGA[pga])
    }

    // --- Scan ---------------------------------------------------------------

    pub fn get_frequency_scan_result(&mut self) -> Result<Option<u32>, BUS::Error> {
        let high = self.read_register_raw(Register_old::Reg0D)?;
        let finished = (high & 0x8000) == 0;
        if !finished {
            return Ok(None);
        }
        let low = self.read_register_raw(Register_old::Reg0E)?;
        Ok(Some(((high as u32 & 0x7FF) << 16) | (low as u32)))
    }

    pub fn get_cxcss_scan_result(&mut self) -> Result<(CssScanResult, u32, u16), BUS::Error> {
        // returns (kind, cdcss_word, ctcss_freq_hz_x10)
        let high = self.read_register_raw(Register_old::Reg69)?;
        if (high & 0x8000) == 0 {
            let low = self.read_register_raw(Register_old::Reg6A)?;
            let cdcss = (((high & 0x0FFF) as u32) << 12) | ((low & 0x0FFF) as u32);
            return Ok((CssScanResult::Cdcss, cdcss, 0));
        }

        let low = self.read_register_raw(Register_old::Reg68)?;
        if (low & 0x8000) == 0 {
            let ctcss_hz_x10 = (((low & 0x1FFF) as u32) * 4843u32) / 10_000u32;
            return Ok((CssScanResult::Ctcss, 0, ctcss_hz_x10 as u16));
        }

        Ok((CssScanResult::NotFound, 0, 0))
    }

    pub fn disable_frequency_scan(&mut self) -> Result<(), BUS::Error> {
        self.write_register_raw(Register_old::Reg32, 290u16 << 1)
    }

    pub fn enable_frequency_scan(&mut self) -> Result<(), BUS::Error> {
        self.write_register_raw(Register_old::Reg32, (290u16 << 1) | 1u16)
    }

    pub fn set_scan_frequency_10hz(&mut self, frequency_10hz: u32) -> Result<(), BUS::Error> {
        self.set_frequency(frequency_10hz)?;
        self.write_register_raw(
            Register_old::Reg51,
            REG_51_DISABLE_CXCSS
                | REG_51_GPIO6_PIN2_NORMAL
                | REG_51_TX_CDCSS_POSITIVE
                | REG_51_MODE_CDCSS
                | REG_51_CDCSS_23_BIT
                | REG_51_1050HZ_NO_DETECTION
                | REG_51_AUTO_CDCSS_BW_DISABLE
                | REG_51_AUTO_CTCSS_BW_DISABLE,
        )?;
        self.rx_turn_on()
    }

    pub fn disable(&mut self) -> Result<(), BUS::Error> {
        self.write_register_raw(Register_old::Reg30, 0x0000)
    }

    pub fn stop_scan(&mut self) -> Result<(), BUS::Error> {
        self.disable_frequency_scan()?;
        self.disable()
    }

    // --- Misc getters -------------------------------------------------------

    pub fn get_dtmf_5tone_code(&mut self) -> Result<u8, BUS::Error> {
        Ok(((self.read_register_raw(Register_old::Reg0B)? >> 8) & 0x0F) as u8)
    }

    pub fn get_cdcss_code_type(&mut self) -> Result<u8, BUS::Error> {
        Ok(((self.read_register_raw(Register_old::Reg0C)? >> 14) & 0x03) as u8)
    }

    pub fn get_ctc_shift(&mut self) -> Result<u8, BUS::Error> {
        Ok(((self.read_register_raw(Register_old::Reg0C)? >> 12) & 0x03) as u8)
    }

    pub fn get_ctc_type(&mut self) -> Result<u8, BUS::Error> {
        Ok(((self.read_register_raw(Register_old::Reg0C)? >> 10) & 0x03) as u8)
    }

    // --- FSK ---------------------------------------------------------------

    pub fn reset_fsk<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), BUS::Error> {
        self.write_register_raw(Register_old::Reg3F, 0x0000)?;
        self.write_register_raw(Register_old::Reg59, 0x0068)?;
        delay.delay_ms(30);
        self.idle()
    }

    /// Port of `BK4819_SendFSKData(pData)`; `data_words` must be length 36.
    pub fn send_fsk_data<D: DelayNs>(
        &mut self,
        data_words: &[u16],
        delay: &mut D,
    ) -> Result<(), BUS::Error> {
        let n = min(data_words.len(), 36);
        delay.delay_ms(20);
        self.write_register_raw(Register_old::Reg3F, 1u16 << 15)?; // FSK TX finished interrupt
        self.write_register_raw(Register_old::Reg59, 0x8068)?;
        self.write_register_raw(Register_old::Reg59, 0x0068)?;
        for &w in data_words.iter().take(n) {
            self.write_register_raw(Register_old::Reg5F, w)?;
        }
        delay.delay_ms(20);
        self.write_register_raw(Register_old::Reg59, 0x2868)?;

        // crude timeout loop (matches C behavior)
        let mut timeout: u8 = 200;
        while timeout > 0 {
            let r0c = self.read_register_raw(Register_old::Reg0C)?;
            if (r0c & 1u16) != 0 {
                break;
            }
            timeout -= 1;
            delay.delay_ms(5);
        }

        self.write_register_raw(Register_old::Reg02, 0x0000)?;
        delay.delay_ms(20);
        self.reset_fsk(delay)
    }

    pub fn prepare_fsk_receive(&mut self) -> Result<(), BUS::Error> {
        // Mirror C ordering (minus delays handled externally if needed).
        self.write_register_raw(Register_old::Reg3F, 0x0000)?;
        self.write_register_raw(Register_old::Reg59, 0x0068)?;
        self.idle()?;
        self.write_register_raw(Register_old::Reg02, 0x0000)?;
        self.write_register_raw(Register_old::Reg3F, 0x0000)?;
        self.rx_turn_on()?;
        // Enable FSK RX finished + FIFO almost full
        self.write_register_raw(Register_old::Reg3F, (1u16 << 13) | (1u16 << 12))?;
        // Clear RX FIFO + preamble len 7 bytes
        self.write_register_raw(Register_old::Reg59, 0x4068)?;
        // Enable FSK scramble + RX
        self.write_register_raw(Register_old::Reg59, 0x3068)?;
        Ok(())
    }

    // --- Roger --------------------------------------------------------------

    pub fn play_roger<D: DelayNs>(
        &mut self,
        mode: RogerMode,
        delay: &mut D,
    ) -> Result<(), BUS::Error> {
        match mode {
            RogerMode::Off => Ok(()),
            RogerMode::Roger => self.play_roger_normal(delay),
            RogerMode::Mdc => self.play_roger_mdc(delay),
        }
    }

    fn play_roger_normal<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), BUS::Error> {
        let tone1_hz: u32 = 1540;
        let tone2_hz: u32 = 1310;

        self.enter_tx_mute()?;
        self.set_af(AfType::Mute)?;
        self.write_register_raw(
            Register_old::Reg70,
            REG_70_ENABLE_TONE1 | (66u16 << REG_70_SHIFT_TONE1_TUNING_GAIN),
        )?;
        self.enable_tx_link()?;
        delay.delay_ms(50);

        self.write_register_raw(Register_old::Reg71, Self::scale_freq(tone1_hz as u16))?;
        self.exit_tx_mute()?;
        delay.delay_ms(80);
        self.enter_tx_mute()?;

        self.write_register_raw(Register_old::Reg71, Self::scale_freq(tone2_hz as u16))?;
        self.exit_tx_mute()?;
        delay.delay_ms(80);
        self.enter_tx_mute()?;

        self.write_register_raw(Register_old::Reg70, 0x0000)?;
        self.write_register_raw(Register_old::Reg30, 0xC1FE)?;
        Ok(())
    }

    fn play_roger_mdc<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), BUS::Error> {
        const FSK_ROGER_TABLE: [u16; 7] = [0xF1A2, 0x7446, 0x61A4, 0x6544, 0x4E8A, 0xE044, 0xEA84];

        self.set_af(AfType::Mute)?;
        // RogerMDC_Configuration table from C
        self.write_register_raw(Register_old::Reg58, 0x37C3)?;
        self.write_register_raw(Register_old::Reg72, 0x3065)?;
        self.write_register_raw(Register_old::Reg70, 0x00E0)?;
        self.write_register_raw(Register_old::Reg5D, 0x0D00)?;
        self.write_register_raw(Register_old::Reg59, 0x8068)?;
        self.write_register_raw(Register_old::Reg59, 0x0068)?;
        self.write_register_raw(Register_old::Reg5A, 0x5555)?;
        self.write_register_raw(Register_old::Reg5B, 0x55AA)?;
        self.write_register_raw(Register_old::Reg5C, 0xAA30)?;

        for &w in &FSK_ROGER_TABLE {
            self.write_register_raw(Register_old::Reg5F, w)?;
        }
        delay.delay_ms(20);

        self.write_register_raw(Register_old::Reg59, 0x0868)?; // enable FSK TX
        delay.delay_ms(180);

        self.write_register_raw(Register_old::Reg59, 0x0068)?;
        self.write_register_raw(Register_old::Reg70, 0x0000)?;
        self.write_register_raw(Register_old::Reg58, 0x0000)?;
        Ok(())
    }

    // --- Small helpers mirroring the C "utility" funcs ----------------------

    pub fn enable_af_dac_disc_mode_tx_dsp(&mut self) -> Result<(), BUS::Error> {
        self.write_register_raw(Register_old::Reg30, 0x0000)?;
        self.write_register_raw(Register_old::Reg30, 0x0302)
    }

    pub fn get_vox_amp(&mut self) -> Result<u16, BUS::Error> {
        Ok(self.read_register_raw(Register_old::Reg64)? & 0x7FFF)
    }

    pub fn set_scramble_frequency_control_word(
        &mut self,
        frequency_hz: u32,
    ) -> Result<(), BUS::Error> {
        self.write_register_raw(Register_old::Reg71, Self::scale_freq(frequency_hz as u16))
    }

    pub fn play_dtmf_ex<D: DelayNs>(
        &mut self,
        local_loopback: bool,
        code: char,
        delay: &mut D,
    ) -> Result<(), BUS::Error> {
        self.enter_dtmf_tx(local_loopback)?;
        delay.delay_ms(50);
        self.play_dtmf(code)?;
        self.exit_tx_mute()
    }

    /// Convenience: disable all RF/TX/RX blocks (same as `BK4819_Disable()`).
    pub fn power_down(&mut self) -> Result<(), BUS::Error> {
        self.disable()
    }

    /// Convenience: set scan tone generator to CDCSS baudrate (C uses REG_07 MODE_CDCSS).
    pub fn set_cdcss_baud_control_word(&mut self, word: u16) -> Result<(), BUS::Error> {
        self.write_register_raw(Register_old::Reg07, REG_07_MODE_CDCSS | (word & 0x1FFF))
    }

    /// Convenience: force CxCSS off (as in `BK4819_ExitSubAu()` and scan setup).
    pub fn disable_cxcss(&mut self) -> Result<(), BUS::Error> {
        self.write_register_raw(Register_old::Reg51, 0x0000)
    }

    /// Convenience: set CTCSS mode with given TX gain1 value.
    pub fn enable_ctcss_mode(&mut self, tx_gain1: u8, detect_1050: bool) -> Result<(), BUS::Error> {
        let tx_gain1 = tx_gain1.min(127) as u16;
        let det = if detect_1050 {
            REG_51_1050HZ_DETECTION
        } else {
            REG_51_1050HZ_NO_DETECTION
        };
        self.write_register_raw(
            Register_old::Reg51,
            REG_51_ENABLE_CXCSS
                | REG_51_GPIO6_PIN2_NORMAL
                | REG_51_TX_CDCSS_POSITIVE
                | REG_51_MODE_CTCSS
                | det
                | REG_51_AUTO_CDCSS_BW_ENABLE
                | REG_51_AUTO_CTCSS_BW_ENABLE
                | (tx_gain1 << REG_51_SHIFT_CXCSS_TX_GAIN1),
        )
    }
}
