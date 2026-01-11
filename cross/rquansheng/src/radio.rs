//! Minimal radio state machine built on top of the BK4819 driver.
//!
//! Goals:
//! - Keep the raw BK4819 register driver (`bk4819::Bk4819Driver`) focused on chip programming.
//! - Provide a higher-level "radio controller" that manages RX/TX state transitions and
//!   periodic interrupt polling (as in the reference C firmware).

use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::DrawTarget;
use embedded_hal::delay::DelayNs;

use crate::bk4819::{Bk4819Driver, FilterBandwidth, GpioPin, RogerMode};
use crate::bk4819_bitbang::Bk4819Bus;
use crate::bk4819_n::{AfOutSel, Reg3F};
use crate::dialer::Dialer;
use crate::display::RenderingMgr;
use crate::frequencies::{calculate_output_power_setting, FrequencyBand};
use crate::keyboard::{KeyEvent, QuanshengKey};
use crate::radio_platform::RadioPlatform;

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Mode {
    Rx,
    Tx,
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

#[repr(i8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum OutputPower {
    Low = 0,
    Mid,
    High,
}

#[derive(Copy, Clone, Debug)]
pub struct ChannelConfig {
    /// RX/TX frequency in Hz
    pub freq: u32,
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
}

impl Default for ChannelConfig {
    fn default() -> Self {
        Self {
            freq: 433_000_000, // 433.00000 MHz
            bandwidth: FilterBandwidth::Wide,
            tx_bias: 20,
            mic_gain: 16, // ~8.0 dB (matches the reference firmware's mid preset)
            roger_mode: RogerMode::Roger,
            code_type: CodeType::None,
            modulation: Modulation::FM,
            output_power: OutputPower::Low,
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

pub struct RadioController<BUS, PLATFORM>
where
    BUS: Bk4819Bus,
    PLATFORM: RadioPlatform,
{
    pub bk: Bk4819Driver<BUS>,
    pub platform: PLATFORM,
    pub channel_cfg: ChannelConfig,
    mode: Mode,
    squelch_open: bool,
    audio_on: bool,
    rendering_mgr: RenderingMgr,
    dialer: Dialer<8>,
}

impl<BUS, PLATFORM> RadioController<BUS, PLATFORM>
where
    BUS: Bk4819Bus,
    PLATFORM: RadioPlatform,
{
    pub fn new(bk: Bk4819Driver<BUS>, mut platform: PLATFORM) -> Self {
        platform.audio_path_off();
        platform.backlight_on();
        Self {
            bk,
            platform,
            channel_cfg: ChannelConfig::default(),
            mode: Mode::Rx,
            squelch_open: false,
            audio_on: false,
            rendering_mgr: RenderingMgr::default(),
            dialer: Dialer::default(),
        }
    }

    #[inline]
    pub fn bk_mut(&mut self) -> &mut Bk4819Driver<BUS> {
        &mut self.bk
    }

    #[inline]
    pub fn mode(&self) -> Mode {
        self.mode
    }

    #[inline]
    pub fn squelch_open(&self) -> bool {
        self.squelch_open
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
        };

        if new_audio_on != self.audio_on {
            self.audio_on = new_audio_on;
            if new_audio_on {
                self.platform.audio_path_on();
            } else {
                self.platform.audio_path_off();
            }
        }
    }

    pub fn init(&mut self) -> Result<(), BUS::Error> {
        self.bk.init()?;
        self.enter_rx()
    }

    pub fn eat_keyboard_event<D: DelayNs>(&mut self, event: Option<KeyEvent>, delay: &mut D) {
        if let Some(KeyEvent::KeyPressed(QuanshengKey::Ptt)) = event {
            let _ = self.enter_tx(delay);
        }

        if let Some(KeyEvent::KeyReleased(QuanshengKey::Ptt)) = event {
            self.bk.play_roger(self.channel_cfg.roger_mode, delay).ok();
            let _ = self.enter_rx();
        }

        if let Some(event) = event {
            self.dialer.eat_keyboard_event(event);
        }

        if let Some(frequency) = self.dialer.get_frequency() {
            self.channel_cfg.freq = frequency * 10;
            log::info!("dialed frequency: {}", self.channel_cfg.freq);
        }
    }

    pub fn render_display<D: DrawTarget<Color = BinaryColor>>(
        &mut self,
        display: &mut D,
    ) -> Result<(), BUS::Error> {
        let rssi = self.bk.get_rssi_dbm().unwrap_or(0);

        let _ = self.rendering_mgr.render_main(
            display,
            self.channel_cfg,
            rssi,
            &self.dialer,
            self.mode,
        );

        Ok(())
    }

    /// Enter RX mode (minimal port of the C sequencing used in `RADIO_SetupRegisters()`).
    pub fn enter_rx(&mut self) -> Result<(), BUS::Error> {
        self.mode = Mode::Rx;
        self.squelch_open = false;

        self.platform.audio_path_off();

        self.bk
            .set_filter_bandwidth(self.channel_cfg.bandwidth, true)?;

        self.bk.setup_power_amplifier(0, 0)?;
        self.bk.toggle_gpio_out(GpioPin::Gpio1PaEnable, false)?;
        self.bk.toggle_gpio_out(GpioPin::Gpio5Red, false)?;
        self.bk.toggle_gpio_out(GpioPin::Gpio6Green, false)?;

        self.bk.clear_interrupts()?;
        self.bk.disable_interrupts()?;

        self.bk.set_mic_gain(self.channel_cfg.mic_gain)?;

        self.bk.set_frequency(self.channel_cfg.freq)?;
        self.bk
            .pick_rx_filter_path_based_on_frequency(self.channel_cfg.freq)?;

        // Squelch thresholds: no EEPROM, pick conservative defaults.
        let thresholds = default_squelch_thresholds(self.channel_cfg.freq);
        self.bk.setup_squelch(thresholds)?;

        self.bk.toggle_gpio_out(GpioPin::Gpio0RxEnable, true)?;

        self.bk.bk_mut().write_reg(
            Reg3F::new()
                .with_squelch_found_en(true)
                .with_squelch_lost_en(true),
        )?;

        self.bk
            .init_agc(self.channel_cfg.modulation == Modulation::AM)?;
        self.bk.set_agc(true)?;

        // Start muted; tick task will unmute on squelch-open event.
        let _ = self.bk.set_af(AfOutSel::Mute);

        Ok(())
    }

    /// Enter TX mode (port of the C sequencing used in `RADIO_SetTxParameters()`).
    pub fn enter_tx<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), BUS::Error> {
        self.mode = Mode::Tx;
        self.squelch_open = false;

        self.platform.audio_path_off();

        self.bk.toggle_gpio_out(GpioPin::Gpio0RxEnable, false)?;

        self.bk
            .set_filter_bandwidth(self.channel_cfg.bandwidth, true)?;
        self.bk.set_frequency(self.channel_cfg.freq)?;

        self.bk.set_mic_gain(self.channel_cfg.mic_gain)?;

        // compander should be set here

        self.bk.prepare_transmit()?;

        delay.delay_ms(10);

        self.bk
            .pick_rx_filter_path_based_on_frequency(self.channel_cfg.freq)?;

        self.bk.toggle_gpio_out(GpioPin::Gpio1PaEnable, true)?;

        delay.delay_ms(5);

        let bias_settings = self.get_bias()?;
        self.bk
            .setup_power_amplifier(bias_settings, self.channel_cfg.freq)?;

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

        let band = FrequencyBand::from_frequency_hz(self.channel_cfg.freq);

        let mut txp = [0u8; 3];

        let band_idx = band.eeprom_index();

        let pwr_idx: u16 = self.channel_cfg.output_power as u16;

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
            self.channel_cfg.freq,
        );

        Ok(setting)
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

            // the interrupts are documented the other way around ??
            if interrupts.squelch_lost() {
                self.squelch_open = true;
                self.bk.toggle_gpio_out(GpioPin::Gpio6Green, true)?;
                self.bk.set_af(AfOutSel::Normal)?;
            }
            if interrupts.squelch_found() {
                self.squelch_open = false;
                self.bk.toggle_gpio_out(GpioPin::Gpio6Green, false)?;
                self.bk.set_af(AfOutSel::Mute)?;
            }
        }

        Ok(())
    }
}

pub struct SquelchThresholds {
    pub open_rssi: u8,
    pub close_rssi: u8,
    pub open_noise: u8,
    pub close_noise: u8,
    pub close_glitch: u8,
    pub open_glitch: u8,
}

fn default_squelch_thresholds(freq_hz: u32) -> SquelchThresholds {
    // Same as the previous app-level helper, but lives here now.
    let is_vhf = freq_hz < 170_400_000;
    if is_vhf {
        SquelchThresholds {
            open_rssi: 70,
            close_rssi: 55,
            open_noise: 50,
            close_noise: 60,
            close_glitch: 70,
            open_glitch: 80,
        }
    } else {
        SquelchThresholds {
            open_rssi: 40,
            close_rssi: 30,
            open_noise: 45,
            close_noise: 55,
            close_glitch: 70,
            open_glitch: 80,
        }
    }
}
