//! Minimal radio state machine built on top of the BK4819 driver.
//!
//! Goals:
//! - Keep the raw BK4819 register driver (`bk4819::Bk4819Driver`) focused on chip programming.
//! - Provide a higher-level "radio controller" that manages RX/TX state transitions and
//!   periodic interrupt polling (as in the reference C firmware).

use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::DrawTarget;
use embedded_hal::delay::DelayNs;

use crate::bk4819::regs::Register_old;
use crate::bk4819::{AfType, Bk4819Driver, FilterBandwidth, GpioPin};
use crate::bk4819_bitbang::Bk4819Bus;
use crate::bk4819_n::Reg3F;
use crate::dialer::Dialer;
use crate::display::RenderingMgr;
use crate::keyboard::{KeyEvent, KeyboardState, QuanshengKey};

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Mode {
    Rx,
    Tx,
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
}

impl Default for ChannelConfig {
    fn default() -> Self {
        Self {
            freq: 433_000_000, // 433.00000 MHz
            bandwidth: FilterBandwidth::Wide,
            tx_bias: 20,
            mic_gain: 16, // ~8.0 dB (matches the reference firmware's mid preset)
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

pub struct RadioController<BUS>
where
    BUS: Bk4819Bus,
{
    pub bk: Bk4819Driver<BUS>,
    pub channel_cfg: ChannelConfig,
    mode: Mode,
    squelch_open: bool,
    rendering_mgr: RenderingMgr,
    dialer: Dialer<8>,
    keyboard_state: KeyboardState,
}

impl<BUS> RadioController<BUS>
where
    BUS: Bk4819Bus,
{
    pub fn new(bk: Bk4819Driver<BUS>) -> Self {
        Self {
            bk,
            channel_cfg: ChannelConfig::default(),
            mode: Mode::Rx,
            squelch_open: false,
            rendering_mgr: RenderingMgr::default(),
            dialer: Dialer::default(),
            keyboard_state: KeyboardState::default(),
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
    pub fn desired_audio_on(&self) -> bool {
        match self.mode {
            Mode::Tx => false,
            Mode::Rx => self.squelch_open,
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

        self.bk
            .set_filter_bandwidth(self.channel_cfg.bandwidth, true)?;
        self.bk.setup_power_amplifier(0, 0)?;
        self.bk
            .toggle_gpio_out(GpioPin::Gpio1Pin29PaEnable, false)?;
        self.bk.toggle_gpio_out(GpioPin::Gpio5Pin1Red, false)?;
        self.bk.toggle_gpio_out(GpioPin::Gpio6Pin2Green, false)?;

        self.bk.set_frequency(self.channel_cfg.freq)?;
        self.bk
            .pick_rx_filter_path_based_on_frequency(self.channel_cfg.freq)?;
        self.bk.toggle_gpio_out(GpioPin::Gpio0Pin28RxEnable, true)?;

        // Squelch thresholds: no EEPROM, pick conservative defaults.
        let thresholds = default_squelch_thresholds(self.channel_cfg.freq);
        self.bk.setup_squelch(thresholds)?;

        self.bk.write_register(
            Reg3F::new()
                .with_squelch_found_en(true)
                .with_squelch_lost_en(true),
        )?;

        // Start muted; tick task will unmute on squelch-open event.
        let _ = self.bk.set_af(AfType::Mute);

        Ok(())
    }

    /// Enter TX mode (minimal port of the C sequencing used in `RADIO_SetTxParameters()`).
    pub fn enter_tx<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), BUS::Error> {
        self.mode = Mode::Tx;
        self.squelch_open = false;

        self.bk
            .toggle_gpio_out(GpioPin::Gpio0Pin28RxEnable, false)?;
        self.bk.toggle_gpio_out(GpioPin::Gpio6Pin2Green, false)?;
        self.bk.toggle_gpio_out(GpioPin::Gpio5Pin1Red, true)?;

        self.bk
            .set_filter_bandwidth(self.channel_cfg.bandwidth, true)?;
        self.bk.set_frequency(self.channel_cfg.freq)?;
        // Mic gain (C: BK4819_REG_7D = 0xE940 | (mic & 0x1f)).
        self.bk.set_mic_gain(self.channel_cfg.mic_gain)?;
        self.bk.prepare_transmit()?;

        delay.delay_ms(10);

        self.bk
            .pick_rx_filter_path_based_on_frequency(self.channel_cfg.freq)?;
        self.bk.toggle_gpio_out(GpioPin::Gpio1Pin29PaEnable, true)?;

        delay.delay_ms(5);
        self.bk
            .setup_power_amplifier(self.channel_cfg.tx_bias, self.channel_cfg.freq)?;

        delay.delay_ms(10);
        self.bk.exit_sub_au()?;
        // Make sure we're in normal voice TX:
        // - tones disabled
        // - modulation set (FM)
        // - TX is already enabled by `prepare_transmit()` (REG_30=0xC1FE), so just leave it running.
        self.bk.write_register_raw(Register_old::Reg70, 0x0000)?;
        self.bk.set_af(AfType::Fm)?;

        Ok(())
    }

    /// Poll BK4819 interrupt status in the same way the reference C firmware does.
    ///
    /// Returns squelch events (open/close) if observed.
    pub fn poll_interrupts(&mut self) -> Result<Events, BUS::Error> {
        if self.mode != Mode::Rx {
            return Ok(Events::none());
        }

        // BK4819 REG_02 interrupt/status bits (ported from `bk4819-regs.h`):
        const REG_02_SQUELCH_FOUND: u16 = 1u16 << 3;
        const REG_02_SQUELCH_LOST: u16 = 1u16 << 2;

        let mut ev = Events::none();

        // Match C firmware behavior:
        // while (ReadRegister(REG_0C) & 1) { WriteRegister(REG_02, 0); st=ReadRegister(REG_02); ... }
        // Safety cap: avoid spinning forever if the line is stuck.
        for _ in 0..8 {
            let irq_req = self.bk.read_register_raw(Register_old::Reg0C)? & 1;
            if irq_req == 0 {
                break;
            }

            // clear interrupts first (as in C)
            self.bk.write_register_raw(Register_old::Reg02, 0)?;

            // then read status bits
            let st = self.bk.read_register_raw(Register_old::Reg02)?;

            if (st & REG_02_SQUELCH_LOST) != 0 {
                self.squelch_open = true;
                ev.squelch_open = Some(true);
            }
            if (st & REG_02_SQUELCH_FOUND) != 0 {
                self.squelch_open = false;
                ev.squelch_open = Some(false);
            }
        }

        // Mirror C LED behavior:
        // - sqlLost  => GREEN on
        // - sqlFound => GREEN off
        if let Some(open) = ev.squelch_open {
            let _ = self.bk.toggle_gpio_out(GpioPin::Gpio6Pin2Green, open);
            let _ = self.bk.set_af(if open { AfType::Fm } else { AfType::Mute });
        }

        Ok(ev)
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
