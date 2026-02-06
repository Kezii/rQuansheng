//! Minimal radio state machine built on top of the BK4819 driver.
//!
//! Goals:
//! - Keep the raw BK4819 register driver (`bk4819::Bk4819Driver`) focused on chip programming.
//! - Provide a higher-level "radio controller" that manages RX/TX state transitions and
//!   periodic interrupt polling (as in the reference C firmware).

use embedded_hal::delay::DelayNs;

use crate::am_fix::AmFix;
use crate::bk1080::Bk1080;
use crate::bk4819::{AfOutSel, Bk4819Driver, FilterBandwidth, GpioPin, RogerMode};
use crate::bk_common::BkCommonBus;
use crate::dialer::Dialer;
use crate::display::VuMeter;
use crate::frequencies::{calculate_output_power_setting, FrequencyBand};
use crate::keyboard::{KeyEvent, QuanshengKey};
use crate::radio_platform::RadioPlatform;

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Mode {
    Rx,
    Tx,
    WfmRx,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum CodeType {
    None,
    CTCSS,
    DCS,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Modulation {
    FM,
    AM,
    USB,
    BYP,
    RAW,
}

impl Modulation {
    pub fn next(&self) -> Self {
        match self {
            Modulation::FM => Modulation::AM,
            Modulation::AM => Modulation::USB,
            Modulation::USB => Modulation::BYP,
            Modulation::BYP => Modulation::RAW,
            Modulation::RAW => Modulation::FM,
        }
    }

    pub fn af_out_sel(self) -> AfOutSel {
        // Port of the C mapping in `RADIO_SetModulation()`.
        match self {
            Modulation::FM => AfOutSel::Normal,
            Modulation::AM => AfOutSel::Am,
            Modulation::USB => AfOutSel::Baseband2,
            Modulation::BYP => AfOutSel::Unknown3,
            Modulation::RAW => AfOutSel::Baseband1,
        }
    }
}

#[repr(i8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum OutputPower {
    Off,
    Low,
    Mid,
    High,
}

impl OutputPower {
    pub fn to_string(self) -> &'static str {
        match self {
            OutputPower::Off => "Off",
            OutputPower::Low => "Low",
            OutputPower::Mid => "Mid",
            OutputPower::High => "Hi",
        }
    }
}

impl OutputPower {
    fn next(&self) -> Self {
        match self {
            OutputPower::Off => OutputPower::Low,
            OutputPower::Low => OutputPower::Mid,
            OutputPower::Mid => OutputPower::High,
            OutputPower::High => OutputPower::Off,
        }
    }
}

impl OutputPower {
    pub fn to_eeprom_index(self) -> Option<u16> {
        match self {
            OutputPower::Low => Some(0),
            OutputPower::Mid => Some(1),
            OutputPower::High => Some(2),
            _ => None,
        }
    }
}

#[repr(i8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum SquelchLevel {
    Squelch0 = 0,
    Squelch1,
    Squelch2,
    Squelch3,
    Squelch4,
    Squelch5,
    Squelch6,
    Squelch7,
    Squelch8,
    Squelch9,
}

#[derive(Copy, Clone, Debug, Default)]
pub struct SLevelConfig {
    pub s0_level: i16,
    pub s9_level: i16,
}

#[derive(Copy, Clone, Debug)]
pub struct SquelchThresholds {
    pub open_rssi: u8,
    pub close_rssi: u8,
    pub open_noise: u8,
    pub close_noise: u8,
    pub close_glitch: u8,
    pub open_glitch: u8,
}

#[derive(Copy, Clone, Debug)]
pub struct ChannelConfig {
    /// RX/TX frequency in Hz
    pub frequency_hz: u32,
    pub bandwidth: FilterBandwidth,
    /// PA bias (board/calibration dependent). Use conservative values by default.
    pub tx_bias: u8,
    /// Microphone gain tuning (BK4819 REG_7D, 0.5 dB/step, 0..=31).
    ///
    /// No EEPROM is available in this project, so we hardcode a sensible default.
    pub mic_gain: u8,

    pub roger_mode: RogerMode,

    pub code_type: CodeType,

    pub modulation: Modulation,

    pub output_power: OutputPower,

    pub squelch_level: SquelchLevel,

    pub frequency_step_hz: u32,

    pub frequency_offset_hz: i32,
}

impl Default for ChannelConfig {
    fn default() -> Self {
        Self {
            frequency_hz: 433_000_000, // 433.00000 MHz
            bandwidth: FilterBandwidth::Wide,
            tx_bias: 20,
            mic_gain: 16, // ~8.0 dB (matches the reference firmware's mid preset)
            roger_mode: RogerMode::Roger,
            code_type: CodeType::None,
            modulation: Modulation::FM,
            output_power: OutputPower::Off,
            squelch_level: SquelchLevel::Squelch1,
            frequency_step_hz: 2500,
            frequency_offset_hz: 0,
        }
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Events {
    /// `Some(true)` when squelch opened; `Some(false)` when squelch closed.
    pub squelch_open: Option<bool>,
}

impl Events {
    pub const fn none() -> Self {
        Self { squelch_open: None }
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Screen {
    RadioState,
    Splash,
}

pub struct Slevel {
    pub rssi: i16,
    pub s_level: i16,
    pub over_s9_dbm: i16,
}

pub struct RadioController<BUS, BUS1080, PLATFORM>
where
    BUS: BkCommonBus + 'static,
    BUS1080: BkCommonBus + 'static,
    PLATFORM: RadioPlatform + 'static,
{
    pub bk: Bk4819Driver<BUS>,
    pub bk1080: Bk1080<BUS1080>,
    pub platform: PLATFORM,
    pub channel_cfg: ChannelConfig,
    pub mode: Mode,
    pub squelch_open: bool,
    pub force_squelch_open: bool,
    pub audio_on: bool,
    pub dialer: Dialer<8>,
    pub should_update_display: bool,
    pub alt_function: bool,
    pub backlight_on: bool,
    pub am_fix: AmFix,
    pub vu_meter: VuMeter,
    /// this is just a cache for eeprom values
    pub s_levels: SLevelConfig,
}

impl<BUS, BUS1080, PLATFORM> RadioController<BUS, BUS1080, PLATFORM>
where
    BUS: BkCommonBus,
    BUS1080: BkCommonBus,
    PLATFORM: RadioPlatform,
{
    pub fn new(bk: Bk4819Driver<BUS>, bk1080: Bk1080<BUS1080>, mut platform: PLATFORM) -> Self {
        platform.set_backlight(true);

        let mut radio = Self {
            bk,
            bk1080,
            platform,
            channel_cfg: ChannelConfig::default(),
            mode: Mode::Rx,
            squelch_open: false,
            force_squelch_open: false,
            audio_on: false,
            dialer: Dialer::default(),
            should_update_display: false,
            alt_function: false,
            backlight_on: true,
            am_fix: AmFix::default(),
            vu_meter: VuMeter::new(),
            s_levels: SLevelConfig::default(),
        };

        radio.s_levels = radio.read_s_levels_from_eeprom();

        radio
    }

    fn open_squelch(&mut self) -> Result<(), BUS::Error> {
        if self.squelch_open {
            return Ok(());
        }

        self.squelch_open = true;
        self.bk.toggle_gpio_out(GpioPin::Gpio6Green, true)?;
        self.bk.set_af(self.channel_cfg.modulation.af_out_sel())?;

        Ok(())
    }

    fn close_squelch(&mut self) -> Result<(), BUS::Error> {
        if !self.squelch_open {
            return Ok(());
        }

        self.squelch_open = false;
        self.bk.toggle_gpio_out(GpioPin::Gpio6Green, false)?;
        self.bk.set_af(AfOutSel::Mute)?;

        Ok(())
    }

    /// Desired audio path state for the board (speaker/amp).
    ///
    /// - TX: always off (matches reference firmware)
    /// - RX: on only when squelch is open
    #[inline]
    pub fn think_platform(&mut self) {
        let new_audio_on = match self.mode {
            Mode::Tx => false,
            Mode::Rx => self.squelch_open,
            Mode::WfmRx => true,
        };

        if new_audio_on != self.audio_on {
            self.audio_on = new_audio_on;
            self.platform.set_audio_path(new_audio_on);
        }
    }

    pub fn init(&mut self) -> Result<(), BUS::Error> {
        self.bk.init()?;
        self.enter_rx()?;

        Ok(())
    }

    /// for now as a demo
    fn _enable_wfm_rx(&mut self) -> Result<(), BUS::Error> {
        self.platform.bk1080_enabled(true);
        self.platform.set_audio_path(true);
        self.mode = Mode::WfmRx;
        let mut delay = crate::delay::CycleDelay::new(48_000_000);
        let _ = self.bk1080.init(&mut delay, Some(1065));
        let _ = self.bk1080.mute(false);
        Ok(())
    }

    pub fn eat_keyboard_event<D: DelayNs>(&mut self, delay: &mut D) {
        let event = self.platform.poll_keyboard();

        let event = match event {
            Some(event) => {
                self.should_update_display = true;
                event
            }
            None => {
                return;
            }
        };

        if let KeyEvent::KeyPressed(QuanshengKey::Ptt) = event {
            let _ = self.enter_tx(delay);
        }

        if let KeyEvent::KeyReleased(QuanshengKey::Ptt) = event {
            if self.mode == Mode::Tx {
                self.bk.play_roger(self.channel_cfg.roger_mode, delay).ok();
            }
            let _ = self.enter_rx();
        }

        if let KeyEvent::KeyPressed(key) = event {
            match key {
                QuanshengKey::F => {
                    self.alt_function = !self.alt_function;
                }
                QuanshengKey::Side1 => {
                    self.platform.set_backlight(!self.backlight_on);
                    self.backlight_on = !self.backlight_on;
                }
                QuanshengKey::Side2 => {
                    if self.mode == Mode::Rx {
                        self.force_squelch_open = !self.force_squelch_open;
                        if self.force_squelch_open {
                            self.open_squelch().ok();
                        } else {
                            self.close_squelch().ok();
                        }
                    }
                }

                QuanshengKey::Up => {
                    self.channel_cfg.frequency_hz += self.channel_cfg.frequency_step_hz;
                    let _ = self.enter_rx();
                }
                QuanshengKey::Down => {
                    self.channel_cfg.frequency_hz -= self.channel_cfg.frequency_step_hz;
                    let _ = self.enter_rx();
                }

                _ => {}
            }
        }

        if self.alt_function {
            match event {
                KeyEvent::KeyPressed(QuanshengKey::Num6) => {
                    self.channel_cfg.output_power = self.channel_cfg.output_power.next();
                    self.alt_function = false;
                }
                KeyEvent::KeyPressed(QuanshengKey::Num0) => {
                    self.channel_cfg.modulation = self.channel_cfg.modulation.next();
                    let _ = self.enter_rx();
                    self.alt_function = false;
                }
                _ => {}
            }
        } else {
            self.dialer.eat_keyboard_event(event);

            if let Some(frequency) = self.dialer.get_frequency() {
                self.channel_cfg.frequency_hz = frequency * 10;
                log::info!("dialed frequency: {}", self.channel_cfg.frequency_hz);
                let _ = self.enter_rx();
            }
        }
    }

    pub fn should_update_display(&mut self) -> bool {
        let should_update = self.should_update_display;
        self.should_update_display = false;
        should_update
    }

    pub fn render_display(&mut self, screen: Screen) -> Result<(), BUS::Error> {
        match screen {
            Screen::RadioState => {
                let _ = self.render_main();
            }
            Screen::Splash => {
                //let _ = self.rendering_mgr.render_splash(display, &[]);
            }
        }

        self.platform.flush_display().unwrap();

        Ok(())
    }

    pub fn get_corrected_rssi(&mut self) -> i16 {
        let rssi = self.bk.get_rssi_dbm().unwrap_or(0);
        let band = FrequencyBand::from_frequency_hz(self.channel_cfg.frequency_hz);
        rssi.saturating_add(band.rssi_dbm_correction())
    }

    pub fn get_s_level(&mut self) -> Slevel {
        fn compute_over_s9_dbm(rssi_dbm: i16, s_levels: SLevelConfig) -> i16 {
            let over = rssi_dbm + s_levels.s9_level;
            over.clamp(0, 99)
        }

        let rssi_dbm = self.get_corrected_rssi();

        let s_levels = self.s_levels;

        let s0_dbm = -s_levels.s0_level;
        let s0_9 = s_levels.s0_level - s_levels.s9_level;
        if s0_9 <= 0 {
            return Slevel {
                rssi: rssi_dbm,
                s_level: 0,
                over_s9_dbm: 0,
            };
        }
        let scaled = ((rssi_dbm - s0_dbm) as i32 * 9) / s0_9 as i32;
        let s_scaled = scaled.clamp(0, 9) as i16;

        let over_s9_dbm = compute_over_s9_dbm(rssi_dbm, s_levels);

        Slevel {
            rssi: rssi_dbm,
            s_level: s_scaled,
            over_s9_dbm,
        }
    }

    pub fn read_s_levels_from_eeprom(&mut self) -> SLevelConfig {
        let mut data = [0u8; 8];
        if self.platform.eeprom_read(0x0EA0, &mut data).is_ok() {
            let s0 = data[1] as i16;
            let s9 = data[2] as i16;
            if s0 < 200 && s0 > 90 && s0 < 160 && s9 > 50 && s9 < s0 - 9 {
                return SLevelConfig {
                    s0_level: s0,
                    s9_level: s9,
                };
            }
        }

        SLevelConfig {
            s0_level: 130,
            s9_level: 76,
        }
    }

    pub fn update_vfo(&mut self) -> Result<(), BUS::Error> {
        self.bk.set_frequency(self.channel_cfg.frequency_hz)?;

        // this is necessary for tx too
        self.bk
            .pick_rx_filter_path_based_on_frequency(self.channel_cfg.frequency_hz)?;

        Ok(())
    }

    /// Enter RX mode (minimal port of the C sequencing used in `RADIO_SetupRegisters()`).
    pub fn enter_rx(&mut self) -> Result<(), BUS::Error> {
        self.mode = Mode::Rx;
        self.squelch_open = false;

        self.bk
            .set_filter_bandwidth(self.channel_cfg.bandwidth, true)?;

        self.bk.setup_power_amplifier(0, 0)?;
        self.bk.toggle_gpio_out(GpioPin::Gpio1PaEnable, false)?;
        self.bk.toggle_gpio_out(GpioPin::Gpio5Red, false)?;
        self.bk.toggle_gpio_out(GpioPin::Gpio6Green, false)?;

        self.bk.clear_interrupts()?;
        self.bk.disable_interrupts()?;

        self.bk.set_mic_gain(self.channel_cfg.mic_gain)?;

        self.update_vfo()?;

        // --- Modulation / AFC / IF coeff / AGC ----------------------------------
        //
        // Port of `RADIO_SetModulation()` plus the AM-fix behavior from `RADIO_SetupAGC()`.
        // We program the "static" registers up-front, and the actual AF output selection
        // will be switched between Mute and the desired demod output by squelch events.
        self.bk.set_af_dac_gain(0xF)?;
        self.bk
            .set_if_coeff(if self.channel_cfg.modulation == Modulation::USB {
                0
            } else {
                0x2AAB
            })?;
        self.bk
            .set_afc_disable(self.channel_cfg.modulation != Modulation::FM)?;

        if self.channel_cfg.modulation == Modulation::AM {
            // Always enable the AM fix in this Rust port (no EEPROM/menu yet).
            self.am_fix.set_enabled(true, self.channel_cfg.frequency_hz);

            // Match the C AM-fix path: lock AGC so the fixer can control gain.
            // Also keep the AGC tables in their "FM" baseline state.
            self.bk.init_agc(false)?;
            self.bk.set_agc(false)?;
        } else {
            self.am_fix
                .set_enabled(false, self.channel_cfg.frequency_hz);
            self.bk
                .init_agc(self.channel_cfg.modulation == Modulation::AM)?;
            self.bk.set_agc(true)?;
        }

        let thresholds = self.get_squelch_threshold_from_eeprom().unwrap();
        self.bk.setup_squelch(thresholds)?;

        self.bk.toggle_gpio_out(GpioPin::Gpio0RxEnable, true)?;

        self.bk.enable_squelch_interrupts()?;

        // Start muted; tick task will unmute on squelch-open event.
        let _ = self.bk.set_af(AfOutSel::Mute);

        if self.force_squelch_open {
            self.open_squelch().ok();
        }

        Ok(())
    }

    /// Enter TX mode (port of the C sequencing used in `RADIO_SetTxParameters()`).
    pub fn enter_tx<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), BUS::Error> {
        if self.channel_cfg.modulation != Modulation::FM {
            return Ok(());
        }

        self.mode = Mode::Tx;
        self.squelch_open = false;
        self.force_squelch_open = false;

        self.bk.toggle_gpio_out(GpioPin::Gpio0RxEnable, false)?;

        self.bk
            .set_filter_bandwidth(self.channel_cfg.bandwidth, true)?;
        self.update_vfo()?;

        self.bk.set_mic_gain(self.channel_cfg.mic_gain)?;

        // compander should be set here

        self.bk.prepare_transmit()?;

        delay.delay_ms(10);

        self.bk.toggle_gpio_out(GpioPin::Gpio1PaEnable, true)?;

        delay.delay_ms(5);

        let bias_settings = self.get_bias()?;
        self.bk
            .setup_power_amplifier(bias_settings, self.channel_cfg.frequency_hz)?;

        delay.delay_ms(10);

        match self.channel_cfg.code_type {
            CodeType::None => {
                self.bk.exit_sub_au()?;
            }
            CodeType::CTCSS => {}
            CodeType::DCS => {}
        }

        self.bk.disable_tones()?;
        self.bk.set_af(AfOutSel::Normal)?;

        self.bk.toggle_gpio_out(GpioPin::Gpio5Red, true)?;
        self.bk.toggle_gpio_out(GpioPin::Gpio6Green, false)?;

        Ok(())
    }

    pub fn get_bias(&mut self) -> Result<u8, BUS::Error> {
        // Port of the C logic in `RADIO_SetTxParameters()`:
        // - Band = FREQUENCY_GetBand(freq)
        // - EEPROM_ReadBuffer(0x1ED0 + (Band * 16) + (OUTPUT_POWER * 3), Txp, 3)
        // - TXP_CalculatedSetting = FREQUENCY_CalculateOutputPower(...)

        let band = FrequencyBand::from_frequency_hz(self.channel_cfg.frequency_hz);

        let mut txp = [0u8; 3];

        let band_idx = band.eeprom_index();

        let pwr_idx = self.channel_cfg.output_power.to_eeprom_index();

        let pwr_idx = match pwr_idx {
            Some(idx) => idx,
            None => return Ok(0),
        };

        let addr = 0x1ED0u16 + (band_idx * 16) + (pwr_idx * 3);

        // If EEPROM isn't available / read fails, fall back to the manual `tx_bias` field.
        if self.platform.eeprom_read(addr, &mut txp).is_err() {
            return Ok(0);
        }

        let (lower, upper) = band.limits_hz().unwrap_or((50_000_000, 76_000_000));
        let middle = (lower + upper) / 2;

        let setting = calculate_output_power_setting(
            txp[0],
            txp[1],
            txp[2],
            lower,
            middle,
            upper,
            self.channel_cfg.frequency_hz,
        );

        Ok(setting)
    }

    #[allow(dead_code, clippy::identity_op)]
    pub fn get_squelch_threshold_from_eeprom(&mut self) -> Option<SquelchThresholds> {
        let band = FrequencyBand::from_frequency_hz(self.channel_cfg.frequency_hz);
        // Port of `RADIO_ConfigureSquelchAndOutputPower()`

        let squelch_level: u16 = self.channel_cfg.squelch_level as u16;

        // Squelch == 0 means "off" with fixed thresholds (C firmware behavior).
        if squelch_level == 0 {
            return Some(SquelchThresholds {
                open_rssi: 0,
                close_rssi: 0,
                open_noise: 127,
                close_noise: 127,
                close_glitch: 255,
                open_glitch: 255,
            });
        }

        let base: u16 = match band {
            FrequencyBand::Band1_50MHz
            | FrequencyBand::Band2_108MHz
            | FrequencyBand::Band3_137MHz => 0x1E60,
            _ => 0x1E00,
        } + squelch_level;

        let mut read_u8 = |addr: u16| -> Option<u8> {
            let mut b = [0u8; 1];
            self.platform.eeprom_read(addr, &mut b).ok()?;
            Some(b[0])
        };

        let open_rssi = read_u8(base)?;
        let close_rssi = read_u8(base + 0x10)?;
        let open_noise = read_u8(base + 0x20)?.min(127);
        let close_noise = read_u8(base + 0x30)?.min(127);
        let close_glitch = read_u8(base + 0x40)?;
        let open_glitch = read_u8(base + 0x50)?;

        Some(SquelchThresholds {
            open_rssi,
            close_rssi,
            open_noise,
            close_noise,
            close_glitch,
            open_glitch,
        })
    }

    /// Poll BK4819 interrupt status in the same way the reference C firmware does.
    ///
    /// Returns squelch events (open/close) if observed.
    pub fn poll_interrupts(&mut self) -> Result<(), BUS::Error> {
        if self.mode != Mode::Rx {
            return Ok(());
        }

        // Match C firmware behavior:
        // while (ReadRegister(REG_0C) & 1) { WriteRegister(REG_02, 0); st=ReadRegister(REG_02); ... }
        // Safety cap: avoid spinning forever if the line is stuck.
        for _ in 0..8 {
            let irq = self.bk.get_irq_indicators()?;

            if !irq.irq() {
                break;
            }

            self.bk.clear_interrupts()?;

            let interrupts = self.bk.get_interrupts()?;

            if self.force_squelch_open {
                self.open_squelch()?;
            } else {
                // the interrupts are documented the other way around ??
                if interrupts.squelch_lost() {
                    self.open_squelch()?;
                }
                if interrupts.squelch_found() {
                    self.close_squelch()?;
                }
            }
        }

        // AM fix needs a periodic tick while listening AM (reference firmware does this at 10ms).
        if self.channel_cfg.modulation == Modulation::AM {
            self.am_fix
                .tick(&mut self.bk, self.channel_cfg.frequency_hz)?;
        }

        Ok(())
    }
}
