use embedded_hal::delay::DelayNs;

use crate::{bk4819::regs::*, bk_common::BkCommonBus, radio::SquelchThresholds};

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
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum GpioPin {
    Gpio0RxEnable,
    Gpio1PaEnable,
    Gpio3UhfLna,
    Gpio4VhfLna,
    Gpio5Red,
    Gpio6Green,
}

/// Roger beep mode (C references EEPROM setting).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum RogerMode {
    Off,
    Roger,
}

/// Snapshot of the current AGC/gain state as reported by the chip.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub struct GainSnapshot {
    /// True when AGC is in auto mode (REG_7E fix mode = 0).
    pub agc_enabled: bool,
    /// AGC gain index as stored in REG_7E bits [14:12] (0..=7).
    pub agc_index: i8,
    /// Instantaneous AGC signal strength from REG_7E bits [11:5] (0..=127).
    pub agc_sig_strength: i8,
    /// Effective gain entry currently selected by AGC (decoded to dB).
    pub current: Gains,
}

/// Port of `scale_freq()` from the C firmware.
pub fn scale_freq(freq_hz: u16) -> u16 {
    // C:
    // (((freq * 1353245) + (1 << 16)) >> 17)
    ((((freq_hz as u32) * 1_353_245u32) + (1u32 << 16)) >> 17) as u16
}

/// High-level driver, owning a `Bk4819` instance plus a small amount of state
/// that was global in the C implementation (GPIO out shadow + rx idle flag).
pub struct Bk4819Driver<BUS> {
    pub bitbang: BUS,
    gpio_out_state: Reg33,
    /// If true, radio is considered asleep/not listening (C global `gRxIdleMode`).
    pub rx_idle_mode: bool,
}

impl<BUS> Bk4819Driver<BUS>
where
    BUS: BkCommonBus,
{
    /// Default microphone gain (0.5 dB/step, 0..=31).
    ///
    /// Reference firmware uses 5 presets: {3, 8, 16, 24, 31}. We choose the mid preset (16).
    pub const DEFAULT_MIC_GAIN: u8 = 16;

    pub const fn new(bk: BUS) -> Self {
        Self {
            bitbang: bk,
            gpio_out_state: Reg33::new(),
            rx_idle_mode: false,
        }
    }

    #[inline]
    pub fn bk_mut(&mut self) -> &mut BUS {
        &mut self.bitbang
    }

    #[deprecated(note = "do NOT use this function, it's for the serial protocol only")]
    pub fn __internal_write_register_raw(&mut self, reg: u8, value: u16) -> Result<(), BUS::Error> {
        self.bitbang.write_reg_raw(reg, value)
    }

    #[deprecated(note = "do NOT use this function, it's for the serial protocol only")]
    pub fn __internal_read_register_raw(&mut self, reg: u8) -> Result<u16, BUS::Error> {
        self.bitbang.read_reg_raw(reg)
    }

    // --- Core init / AGC ----------------------------------------------------

    /// Port of `BK4819_Init()`.
    pub fn init(&mut self) -> Result<(), BUS::Error> {
        // Soft reset
        self.bitbang
            .write_reg(Reg00::new().with_soft_reset(Reg00SoftReset::Reset))?;
        self.bitbang
            .write_reg(Reg00::new().with_soft_reset(Reg00SoftReset::Normal))?;

        //self.write_register(Register::Reg37, 0x1D0F)?; // 0b0 001 1101 00001111
        self.bitbang.write_reg(
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

        self.bitbang
            .write_reg(Reg36::new().with_pa_gain2(0b010).with_pa_gain1(0b100))?;

        self.init_agc(false)?;
        self.set_agc(true)?;

        // REG_19: <15> MIC AGC 1=disable 0=enable
        //self.write_register(Register::Reg19, 0b0001_0000_0100_0001)?;
        self.bitbang.write_reg(
            Reg19::new()
                .with_mic_agc_disable(false)
                .with_undocumented_1(0b001000001000001),
        )?;

        // REG_7D: mic gain tuning (0.5 dB/step, 0..=31) in the low 5 bits.
        // No EEPROM is available here, so use a sensible default.
        //self.set_mic_gain(Self::DEFAULT_MIC_GAIN)?;

        self.set_mic_gain(Self::DEFAULT_MIC_GAIN)?;

        // REG_48 .. RX AF level (see C comments)
        self.set_gains(58, 8)?;

        // DTMF coefficients table
        const DTMF_COEFFS: [u16; 16] = [
            111, 107, 103, 98, 80, 71, 58, 44, 65, 55, 37, 23, 228, 203, 181, 159,
        ];
        for (i, &c) in DTMF_COEFFS.iter().enumerate() {
            self.bitbang.write_reg(
                Reg09::new()
                    .with_coefficient(c as u8)
                    .with_symbol_number(i as u8),
            )?;
        }

        //self.write_register(crate::bk4819::regs::Register::Reg1F, 0x5454)?;
        self.bitbang.write_reg(
            Reg1F::new()
                .with_pll_cp_bit(4)
                .with_undocumented(0b10101000101),
        )?;

        //self.write_register(Register::Reg3E, 0xA037)?;
        self.bitbang
            .write_reg(Reg3E::new().with_band_thresh(0xA037))?;

        // Preserve legacy initialization exactly (raw value 0x9000).
        // This sets the GPIO output-disable mask as in the original C port.
        self.gpio_out_state = Reg33::from(0x9000);
        self.bitbang.write_reg(self.gpio_out_state)?;
        self.disable_interrupts()?;

        Ok(())
    }

    /// Set microphone gain tuning (BK4819 REG_7D, low 5 bits).
    ///
    /// `gain` is in 0.5 dB/step, 0..=31.
    pub fn set_mic_gain(&mut self, gain: u8) -> Result<(), BUS::Error> {
        //self.write_register(Register::Reg7D, 0xE940 | gain)
        self.bitbang.write_reg(
            Reg7D::new()
                .with_mic_sens(gain)
                .with_undocumented(0b11101001010),
        )?;

        Ok(())
    }

    /// Port of `BK4819_SetAGC(enable)`.
    pub fn set_agc(&mut self, enable: bool) -> Result<(), BUS::Error> {
        let r7e = self.bitbang.read_reg::<Reg7E>()?;

        if r7e.agc_fix_mode() != enable {
            return Ok(());
        }

        let next = r7e
            .with_agc_fix_mode(!enable) // 0=auto (AGC on), 1=fix (AGC off)
            .with_agc_fix_index(3); // fix index (matches reference firmware)

        self.bitbang.write_reg(next)
    }

    /// Port of `BK4819_InitAGC(amModulation)`.
    pub fn init_agc(&mut self, am_modulation: bool) -> Result<(), BUS::Error> {
        self.bitbang.write_reg(Reg13::from(0x03BE))?;
        self.bitbang.write_reg(Reg12::from(0x037B))?;
        self.bitbang.write_reg(Reg11::from(0x027B))?;
        self.bitbang.write_reg(Reg10::from(0x007A))?;

        if am_modulation {
            self.bitbang.write_reg(Reg14::from(0x0000))?;
            self.bitbang.write_reg(Reg49::from((50u16 << 7) | 32u16))?;
        } else {
            self.bitbang.write_reg(Reg14::from(0x0019))?;
            self.bitbang.write_reg(Reg49::from((84u16 << 7) | 56u16))?;
        }

        self.bitbang
            .write_reg(Reg7B::new().with_undocumented(0x8420))?;
        Ok(())
    }

    // --- GPIO ---------------------------------------------------------------

    /// Port of `BK4819_ToggleGpioOut(pin, set)`.
    pub fn toggle_gpio_out(&mut self, pin: GpioPin, set: bool) -> Result<(), BUS::Error> {
        self.gpio_out_state = match pin {
            GpioPin::Gpio0RxEnable => self.gpio_out_state.with_gpio0_out(set),
            GpioPin::Gpio1PaEnable => self.gpio_out_state.with_gpio1_out(set),
            GpioPin::Gpio3UhfLna => self.gpio_out_state.with_gpio3_out(set),
            GpioPin::Gpio4VhfLna => self.gpio_out_state.with_gpio4_out(set),
            GpioPin::Gpio5Red => self.gpio_out_state.with_gpio5_out(set),
            GpioPin::Gpio6Green => self.gpio_out_state.with_gpio6_out(set),
        };

        self.bitbang.write_reg(self.gpio_out_state)
    }

    // --- Bandwidth / PA / Frequency ----------------------------------------

    /// Port of `BK4819_SetFilterBandwidth(bw, weak_no_different)`.
    pub fn set_filter_bandwidth(
        &mut self,
        bandwidth: FilterBandwidth,
        weak_no_different: bool,
    ) -> Result<(), BUS::Error> {
        match bandwidth {
            FilterBandwidth::Wide => {
                self.bitbang.write_reg(
                    Reg43::new()
                        .with_rf_bw_strong(4)
                        .with_rf_bw_weak(if weak_no_different { 4 } else { 2 })
                        .with_aftx_lpf2_bw(6)
                        .with_bw_mode(2)
                        .with_undocumented_1(1)
                        .with_fm_demod_gain(false),
                )?;
            }
            FilterBandwidth::Narrow => {
                self.bitbang.write_reg(
                    Reg43::new()
                        .with_rf_bw_strong(4)
                        .with_rf_bw_weak(if weak_no_different { 4 } else { 2 })
                        .with_aftx_lpf2_bw(0)
                        .with_bw_mode(0)
                        .with_undocumented_1(1)
                        .with_fm_demod_gain(false),
                )?;
            }
            FilterBandwidth::Narrower => {
                self.bitbang.write_reg(
                    Reg43::new()
                        .with_rf_bw_strong(2)
                        .with_rf_bw_weak(2)
                        .with_aftx_lpf2_bw(1)
                        .with_bw_mode(1)
                        .with_undocumented_1(1)
                        .with_fm_demod_gain(false),
                )?;
            }
            FilterBandwidth::U1k7 => {
                self.bitbang.write_reg(
                    Reg43::new()
                        .with_rf_bw_strong(0)
                        .with_rf_bw_weak(0)
                        .with_aftx_lpf2_bw(1)
                        .with_bw_mode(1)
                        .with_undocumented_1(1)
                        .with_fm_demod_gain(false),
                )?;
            }
        }
        Ok(())
    }

    /// Port of `BK4819_SetupPowerAmplifier(bias, frequency)`.
    ///
    pub fn setup_power_amplifier(&mut self, bias: u8, frequency_hz: u32) -> Result<(), BUS::Error> {
        if frequency_hz < 280_000_000 {
            self.bitbang.write_reg::<Reg36>(
                Reg36::new()
                    .with_pa_bias(bias)
                    .with_pa_ctl_output(true)
                    .with_pa_gain1(1)
                    .with_pa_gain2(0), //-14.9dBm
            )
        } else {
            self.bitbang.write_reg::<Reg36>(
                Reg36::new()
                    .with_pa_bias(bias)
                    .with_pa_ctl_output(true)
                    .with_pa_gain1(4)
                    .with_pa_gain2(2), //0.13dBm
            )
        }
    }

    /// Port of `BK4819_SetFrequency(freq)`.
    ///
    pub fn set_frequency(&mut self, frequency_hz: u32) -> Result<(), BUS::Error> {
        let frequency_10hz = frequency_hz / 10;

        self.bitbang
            .write_reg(Reg38::new().with_freq_lo(frequency_10hz as u16))?;
        self.bitbang
            .write_reg(Reg39::new().with_freq_hi((frequency_10hz >> 16) as u16))?;
        Ok(())
    }

    pub fn pick_rx_filter_path_based_on_frequency(
        &mut self,
        frequency_hz: u32,
    ) -> Result<(), BUS::Error> {
        if frequency_hz < 280_000_000 {
            // VHF
            self.toggle_gpio_out(GpioPin::Gpio4VhfLna, true)?;
            self.toggle_gpio_out(GpioPin::Gpio3UhfLna, false)?;
        } else {
            // UHF
            self.toggle_gpio_out(GpioPin::Gpio4VhfLna, false)?;
            self.toggle_gpio_out(GpioPin::Gpio3UhfLna, true)?;
        }
        Ok(())
    }

    pub fn rx_filter_off(&mut self) -> Result<(), BUS::Error> {
        // OFF
        self.toggle_gpio_out(GpioPin::Gpio4VhfLna, false)?;
        self.toggle_gpio_out(GpioPin::Gpio3UhfLna, false)?;
        Ok(())
    }

    // --- Squelch / RX on ----------------------------------------------------

    /// Port of `BK4819_SetupSquelch(...)`.
    #[allow(clippy::too_many_arguments)]
    pub fn setup_squelch(&mut self, thresholds: SquelchThresholds) -> Result<(), BUS::Error> {
        // Disable tones
        self.disable_tones()?;

        // Glitch threshold for squelch close
        self.bitbang.write_reg(
            Reg4D::new()
                .with_glitch_th0(thresholds.close_glitch)
                .with_undocumented(0xA0),
        )?;

        // Squelch open/close delay + glitch open threshold
        // XXX DOCUMENTATION IS PROBABLY WRONG ON THIS REGISTER
        // SQ0_DELAY HAS OFFSET 9 (IN THE DOCS) BUT IS 3-WIDE, CLASHING WITH THE VALUE AFTER
        // IT HAS PROBABLY OFFSET 8
        // BUT THE ORIGINAL FIRMWARE USES THE OFFSET 9, PUTTING THE BITS WHERE THEY DO NOT MAKE SENSE
        self.bitbang.write_reg(
            Reg4E::new()
                .with_glitch_th1(thresholds.open_glitch)
                .with_sq0_delay(6)
                .with_sq1_delay(5)
                .with_undocumented_1(1),
        )?;

        /*
           BK4819_WriteRegister(BK4819_REG_4E,  // 01 101 11 1 00000000
                   (1u << 14) |                  //  1 ???
                   (5u << 11) |                  // *5  squelch = open  delay .. 0 ~ 7
                   (6u <<  9) |                  // *3  squelch = close delay .. 0 ~ 3
                   SquelchOpenGlitchThresh);     //  0 ~ 255
        */

        // Ex-noise close/open
        self.bitbang.write_reg(
            Reg4F::new()
                .with_exnoise_th0(thresholds.close_noise)
                .with_exnoise_th1(thresholds.open_noise),
        )?;

        // RSSI open/close (0.5dB/step)
        self.bitbang.write_reg(
            Reg78::new()
                .with_th_sq1(thresholds.open_rssi)
                .with_th_sq0(thresholds.close_rssi),
        )?;

        self.set_af(AfOutSel::Mute)?;
        self.rx_turn_on()
    }

    pub fn set_af(&mut self, af: AfOutSel) -> Result<(), BUS::Error> {
        self.bitbang.write_reg(
            Reg47::new()
                .with_af_output_selection(af)
                .with_undocumented_0(1 << 5)
                .with_af_out_invert(true)
                .with_undocumented_2(1),
        )
    }

    /// Port of `BK4819_RX_TurnOn()`.
    pub fn rx_turn_on(&mut self) -> Result<(), BUS::Error> {
        self.bitbang.write_reg(Reg37::from(0x1F0F))?;
        self.disable()?;
        self.bitbang.write_reg(
            Reg30::new()
                .with_vco_cal_en(true)
                .with_undocumented(false)
                .with_rx_link_en(0xF)
                .with_af_dac_en(true)
                .with_disc_mode_disable(true)
                .with_pll_vco_en(0xF)
                .with_pa_gain_en(false)
                .with_mic_adc_en(false)
                .with_tx_dsp_en(false)
                .with_rx_dsp_en(true),
        )?;

        Ok(())
    }

    pub fn disable_tones(&mut self) -> Result<(), BUS::Error> {
        self.bitbang.write_reg(Reg70::from(0))
    }
    // --- Tone / TX link / mute ---------------------------------------------

    /// disable mic for tone transmission
    pub fn enter_tx_mute(&mut self) -> Result<(), BUS::Error> {
        self.bitbang.write_reg(
            Reg50::new()
                .with_aftx_mute(true)
                .with_undocumented(0b11101100100000),
        )?;

        Ok(())
    }
    /// enable mic
    pub fn exit_tx_mute(&mut self) -> Result<(), BUS::Error> {
        self.bitbang.write_reg(
            Reg50::new()
                .with_aftx_mute(false)
                .with_undocumented(0b11101100100000),
        )?;
        Ok(())
    }

    /// Port of `BK4819_EnableTXLink()`.
    pub fn enable_tx_link(&mut self) -> Result<(), BUS::Error> {
        self.bitbang.write_reg(
            Reg30::new()
                .with_vco_cal_en(true)
                .with_undocumented(true)
                .with_rx_link_en(0)
                .with_af_dac_en(true)
                .with_disc_mode_disable(true)
                .with_pll_vco_en(0xF)
                .with_pa_gain_en(true)
                .with_mic_adc_en(false)
                .with_tx_dsp_en(true)
                .with_rx_dsp_en(false),
        )?;
        Ok(())
    }

    /// Port of `BK4819_PlayTone(freq, tuningGainSwitch)`.
    pub fn play_tone(
        &mut self,
        frequency_hz: u16,
        tuning_gain_switch: bool,
    ) -> Result<(), BUS::Error> {
        self.enter_tx_mute()?;
        self.set_af(AfOutSel::BeepTx)?;

        let gain = if !tuning_gain_switch { 96 } else { 28 };
        self.bitbang
            .write_reg(Reg70::new().with_tone1_en(true).with_tone1_gain(gain))?;

        self.disable()?;
        self.bitbang.write_reg(
            Reg30::new()
                .with_af_dac_en(true)
                .with_disc_mode_disable(true)
                .with_tx_dsp_en(true),
        )?;
        self.bitbang
            .write_reg(Reg71::new().with_word(scale_freq(frequency_hz)))?;
        Ok(())
    }

    /// Port of `BK4819_PlaySingleTone(...)`.
    pub fn play_single_tone<D: DelayNs, A: crate::radio_platform::RadioPlatform>(
        &mut self,
        tone_hz: u32,
        delay_ms: u32,
        level: u8,
        play_speaker: bool,
        delay: &mut D,
        platform: &mut A,
    ) -> Result<(), BUS::Error> {
        self.enter_tx_mute()?;
        if play_speaker {
            platform.set_audio_path(true);
            self.set_af(AfOutSel::BeepTx)?;
        } else {
            self.set_af(AfOutSel::Mute)?;
        }

        self.bitbang
            .write_reg(Reg70::new().with_tone1_en(true).with_tone1_gain(level))?;

        self.enable_tx_link()?;
        delay.delay_ms(50);

        self.bitbang
            .write_reg(Reg71::new().with_word(scale_freq(tone_hz as u16)))?;
        self.exit_tx_mute()?;

        delay.delay_ms(delay_ms);
        self.enter_tx_mute()?;

        if play_speaker {
            platform.set_audio_path(false);
            self.set_af(AfOutSel::Mute)?;
        }

        self.disable_tones()?;
        self.bitbang.write_reg(Reg30::from(0xC1FE))?;
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
        self.bitbang
            .write_reg(Reg70::new().with_tone1_en(true).with_tone1_gain(66))?;
        self.bitbang
            .write_reg(Reg71::new().with_word(scale_freq(frequency_hz as u16)))?;
        self.set_af(if local_loopback {
            AfOutSel::BeepTx
        } else {
            AfOutSel::Mute
        })?;
        self.enable_tx_link()?;
        delay.delay_ms(50);
        self.exit_tx_mute()
    }

    // --- Power states -------------------------------------------------------

    pub fn sleep(&mut self) -> Result<(), BUS::Error> {
        self.disable()?;
        self.bitbang.write_reg(Reg37::from(0x1D00))?;
        Ok(())
    }

    pub fn turns_off_tones_turns_on_rx(&mut self) -> Result<(), BUS::Error> {
        self.disable_tones()?;
        self.set_af(AfOutSel::Mute)?;
        self.exit_tx_mute()?;
        self.disable()?;

        self.bitbang.write_reg(
            Reg30::new()
                .with_rx_dsp_en(true)
                .with_tx_dsp_en(false)
                .with_mic_adc_en(false)
                .with_pa_gain_en(false)
                .with_pll_vco_en(0xF)
                .with_disc_mode_disable(true)
                .with_af_dac_en(true)
                .with_rx_link_en(0xF)
                .with_undocumented(true)
                .with_vco_cal_en(true),
        )?;

        Ok(())
    }

    pub fn exit_bypass(&mut self) -> Result<(), BUS::Error> {
        self.set_af(AfOutSel::Mute)?;

        let reg_7e = self.bitbang.read_reg::<Reg7E>()?;

        self.bitbang.write_reg(reg_7e.with_dcf_bw_tx(5))
    }

    pub fn prepare_transmit(&mut self) -> Result<(), BUS::Error> {
        self.exit_bypass()?;
        self.exit_tx_mute()?;
        self.txon_beep()
    }

    pub fn txon_beep(&mut self) -> Result<(), BUS::Error> {
        self.bitbang.write_reg(
            Reg37::new()
                .with_bg_en(true)
                .with_xtal_en(true)
                .with_dsp_en(true)
                .with_undocumented_0(true)
                .with_pll_ldo_byp(false)
                .with_rf_ldo_byp(false)
                .with_vco_ldo_byp(false)
                .with_ana_ldo_byp(false)
                .with_pll_ldo_sel(true)
                .with_rf_ldo_sel(false)
                .with_vco_ldo_sel(true)
                .with_ana_ldo_sel(true)
                .with_dsp_volt(1),
        )?;

        self.bitbang.write_reg(
            Reg52::new()
                .with_ctcss_lost_th(15)
                .with_ctcss_found_th(10)
                .with_detect_thresh_mode(false)
                .with_tail_mode(Reg52TailMode::Tail1344)
                .with_tail_shift_en(false),
        )?;

        self.disable()?;

        self.bitbang.write_reg(
            Reg30::new()
                .with_rx_dsp_en(false)
                .with_tx_dsp_en(true)
                .with_mic_adc_en(true)
                .with_pa_gain_en(true)
                .with_pll_vco_en(15)
                .with_disc_mode_disable(true)
                .with_af_dac_en(false)
                .with_rx_link_en(0)
                .with_undocumented(true)
                .with_vco_cal_en(true),
        )
    }

    pub fn exit_sub_au(&mut self) -> Result<(), BUS::Error> {
        self.bitbang.write_reg(Reg51::from(0))
    }

    pub fn conditional_rx_turn_on_and_gpio0_enable(&mut self) -> Result<(), BUS::Error> {
        if self.rx_idle_mode {
            self.toggle_gpio_out(GpioPin::Gpio0RxEnable, true)?;
            self.rx_turn_on()?;
        }
        Ok(())
    }

    // --- Indicators / measurements -----------------------------------------

    pub fn get_rssi(&mut self) -> Result<u16, BUS::Error> {
        let r67: Reg67 = self.bitbang.read_reg::<Reg67>()?;
        Ok(r67.rssi())
    }

    pub fn get_rssi_dbm(&mut self) -> Result<i16, BUS::Error> {
        let rssi = self.get_rssi()? as i16;
        Ok((rssi / 2) - 160)
    }

    pub fn get_glitch_indicator(&mut self) -> Result<u8, BUS::Error> {
        Ok(self.bitbang.read_reg::<Reg63>()?.glitch())
    }

    pub fn get_ex_noise_indicator(&mut self) -> Result<u8, BUS::Error> {
        Ok(self.bitbang.read_reg::<Reg65>()?.exnoise())
    }

    pub fn get_voice_amplitude_out(&mut self) -> Result<u16, BUS::Error> {
        Ok(self.bitbang.read_reg::<Reg64>()?.voice_amp())
    }

    pub fn get_af_tx_rx(&mut self) -> Result<u8, BUS::Error> {
        Ok(self.bitbang.read_reg::<Reg6F>()?.af_amp_db())
    }

    /// Port of `BK4819_GetRxGain_dB()`.
    pub fn get_rx_gain_db(&mut self) -> Option<Gains> {
        let reg7e = self.bitbang.read_reg::<Reg7E>().ok()?;
        let gain_idx_raw = reg7e.agc_fix_index() as i8;
        let gain_idx = if gain_idx_raw >= 4 {
            gain_idx_raw - 8
        } else {
            gain_idx_raw
        };

        let gains = if gain_idx < 0 {
            Gains::from_reg14(self.bitbang.read_reg::<Reg14>().ok()?)
        } else {
            match 10 + gain_idx {
                10 => Gains::from_reg10(self.bitbang.read_reg::<Reg10>().ok()?),
                11 => Gains::from_reg11(self.bitbang.read_reg::<Reg11>().ok()?),
                12 => Gains::from_reg12(self.bitbang.read_reg::<Reg12>().ok()?),
                13 => Gains::from_reg13(self.bitbang.read_reg::<Reg13>().ok()?),
                14 => Gains::from_reg14(self.bitbang.read_reg::<Reg14>().ok()?),
                _ => return None,
            }
        };

        Some(gains)
    }

    pub fn get_gain(&mut self) -> Result<GainSnapshot, BUS::Error> {
        let reg7e = self.bitbang.read_reg::<Reg7E>()?;

        let agc_sig_strength = reg7e.undocumented(); //((reg7e_raw >> 5) & 0x7f) as u8;

        let table = [
            Gains::from_reg10(self.bitbang.read_reg::<Reg10>()?),
            Gains::from_reg11(self.bitbang.read_reg::<Reg11>()?),
            Gains::from_reg12(self.bitbang.read_reg::<Reg12>()?),
            Gains::from_reg13(self.bitbang.read_reg::<Reg13>()?),
            Gains::from_reg14(self.bitbang.read_reg::<Reg14>()?),
        ];

        let current = match reg7e.agc_fix_index() {
            -4..=-1 => table[4],
            0..=3 => table[reg7e.agc_fix_index() as usize],
            _ => table[4],
        };

        Ok(GainSnapshot {
            agc_enabled: !reg7e.agc_fix_mode(),
            agc_index: reg7e.agc_fix_index(),
            agc_sig_strength,
            current,
        })
    }

    pub fn disable(&mut self) -> Result<(), BUS::Error> {
        self.bitbang.write_reg::<Reg30>(Reg30::from(0))
    }

    // --- Misc getters -------------------------------------------------------

    pub fn get_irq_indicators(&mut self) -> Result<Reg0C, BUS::Error> {
        self.bitbang.read_reg::<Reg0C>()
    }

    // this function is used everywhere in the original fw but it's not documented ??
    pub fn clear_interrupts(&mut self) -> Result<(), BUS::Error> {
        self.bitbang.write_reg::<Reg02>(Reg02::default())
    }

    pub fn get_interrupts(&mut self) -> Result<Reg02, BUS::Error> {
        self.bitbang.read_reg::<Reg02>()
    }

    pub fn disable_interrupts(&mut self) -> Result<(), BUS::Error> {
        self.bitbang.write_reg::<Reg3F>(Reg3F::from(0))
    }

    pub fn enable_squelch_interrupts(&mut self) -> Result<(), BUS::Error> {
        self.bitbang.write_reg(
            Reg3F::new()
                .with_squelch_found_en(true)
                .with_squelch_lost_en(true),
        )
    }

    pub fn set_gains(&mut self, volume_gain: u8, dac_gain: u8) -> Result<(), BUS::Error> {
        self.bitbang.write_reg(
            Reg48::new()
                .with_af_dac_gain(dac_gain)
                .with_afrx_gain2(volume_gain)
                .with_afrx_gain1(0)
                .with_undocumented(11),
        )
    }

    /// Set only the AF DAC gain (REG_48[3:0]) preserving current AF RX gains.
    pub fn set_af_dac_gain(&mut self, dac_gain: u8) -> Result<(), BUS::Error> {
        let r48 = self.bitbang.read_reg::<Reg48>()?;
        self.bitbang.write_reg(r48.with_af_dac_gain(dac_gain))
    }

    /// Write IF selection coefficient (REG_3D) as raw value.
    pub fn set_if_coeff(&mut self, coeff: u16) -> Result<(), BUS::Error> {
        // Use a typed write preserving the exact raw value.
        self.bitbang.write_reg(Reg3D::from(coeff))
    }

    /// Enable/disable AFC (REG_73[?] `afc_disable`) by raw access.
    pub fn set_afc_disable(&mut self, disable: bool) -> Result<(), BUS::Error> {
        let r73 = self.bitbang.read_reg::<Reg73>()?;
        self.bitbang.write_reg(r73.with_afc_disable(disable))
    }

    /// Write REG_13 directly (used by AM-fix logic).
    #[inline]
    pub fn write_reg13_raw(&mut self, value: u16) -> Result<(), BUS::Error> {
        // Use a typed write preserving the exact raw value.
        self.bitbang.write_reg(Reg13::from(value))
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
        }
    }

    fn play_roger_normal<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), BUS::Error> {
        let tone1_hz: u32 = 1540;
        let tone2_hz: u32 = 1310;

        self.enter_tx_mute()?;
        self.set_af(AfOutSel::Mute)?;
        self.bitbang
            .write_reg(Reg70::new().with_tone1_en(true).with_tone1_gain(66))?;
        self.enable_tx_link()?;
        delay.delay_ms(10);

        self.bitbang
            .write_reg(Reg71::new().with_word(scale_freq(tone1_hz as u16)))?;
        self.exit_tx_mute()?;
        delay.delay_ms(20);
        self.enter_tx_mute()?;

        self.bitbang
            .write_reg(Reg71::new().with_word(scale_freq(tone2_hz as u16)))?;
        self.exit_tx_mute()?;
        delay.delay_ms(20);
        self.enter_tx_mute()?;

        self.disable_tones()?;
        self.bitbang.write_reg(Reg30::from(0xC1FE))?;
        Ok(())
    }

    /// Convenience: force CxCSS off (as in `BK4819_ExitSubAu()` and scan setup).
    pub fn disable_cxcss(&mut self) -> Result<(), BUS::Error> {
        self.bitbang.write_reg(Reg51::from(0))
    }
}
