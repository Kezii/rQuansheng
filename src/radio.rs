//! Minimal radio state machine built on top of the BK4819 driver.
//!
//! Goals:
//! - Keep the raw BK4819 register driver (`bk4819::Bk4819Driver`) focused on chip programming.
//! - Provide a higher-level "radio controller" that manages RX/TX state transitions and
//!   periodic interrupt polling (as in the reference C firmware).

use embedded_hal::delay::DelayNs;

use crate::bk4819::regs::Register;
use crate::bk4819::{AfType, Bk4819Driver, FilterBandwidth, GpioPin};
use crate::bk4819_bitbang::Bk4819Bus;

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Mode {
    Rx,
    Tx,
}

#[derive(Copy, Clone, Debug)]
pub struct Config {
    /// RX/TX frequency in 10Hz units (matches the C firmware).
    pub freq_10hz: u32,
    pub bandwidth: FilterBandwidth,
    /// PA bias (board/calibration dependent). Use conservative values by default.
    pub tx_bias: u8,
    /// Continuous test tone in TX (Hz).
    pub tx_tone_hz: u32,
}

impl Config {
    pub const fn default_uhf_433() -> Self {
        Self {
            freq_10hz: 43_300_000, // 433.00000 MHz
            bandwidth: FilterBandwidth::Narrow,
            tx_bias: 20,
            tx_tone_hz: 1000,
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
    bk: Bk4819Driver<BUS>,
    cfg: Config,
    mode: Mode,
    squelch_open: bool,
}

impl<BUS> RadioController<BUS>
where
    BUS: Bk4819Bus,
{
    pub fn new(bk: Bk4819Driver<BUS>, cfg: Config) -> Self {
        Self {
            bk,
            cfg,
            mode: Mode::Rx,
            squelch_open: false,
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

    /// Enter RX mode (minimal port of the C sequencing used in `RADIO_SetupRegisters()`).
    pub fn enter_rx(&mut self) -> Result<(), BUS::Error> {
        self.mode = Mode::Rx;
        self.squelch_open = false;

        self.bk.set_filter_bandwidth(self.cfg.bandwidth, true)?;
        self.bk.setup_power_amplifier(0, 0)?;
        self.bk
            .toggle_gpio_out(GpioPin::Gpio1Pin29PaEnable, false)?;
        self.bk.toggle_gpio_out(GpioPin::Gpio5Pin1Red, false)?;
        self.bk.toggle_gpio_out(GpioPin::Gpio6Pin2Green, false)?;

        self.bk.set_frequency_10hz(self.cfg.freq_10hz)?;
        self.bk
            .pick_rx_filter_path_based_on_frequency(self.cfg.freq_10hz)?;
        self.bk.toggle_gpio_out(GpioPin::Gpio0Pin28RxEnable, true)?;

        // Squelch thresholds: no EEPROM, pick conservative defaults.
        let (open_rssi, close_rssi, open_noise, close_noise, close_glitch, open_glitch) =
            default_squelch_thresholds(self.cfg.freq_10hz);
        self.bk.setup_squelch(
            open_rssi,
            close_rssi,
            open_noise,
            close_noise,
            close_glitch,
            open_glitch,
        )?;

        // Enable squelch interrupts (C: InterruptMask includes FOUND/LOST, then write REG_3F).
        const REG_3F_SQUELCH_FOUND: u16 = 1u16 << 3;
        const REG_3F_SQUELCH_LOST: u16 = 1u16 << 2;
        self.bk
            .write_register(Register::Reg3F, REG_3F_SQUELCH_FOUND | REG_3F_SQUELCH_LOST)?;
        // Clear pending interrupt/status bits.
        let _ = self.bk.write_register(Register::Reg02, 0);

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

        self.bk.set_filter_bandwidth(self.cfg.bandwidth, true)?;
        self.bk.set_frequency_10hz(self.cfg.freq_10hz)?;
        self.bk.prepare_transmit()?;

        delay.delay_ms(10);

        self.bk
            .pick_rx_filter_path_based_on_frequency(self.cfg.freq_10hz)?;
        self.bk.toggle_gpio_out(GpioPin::Gpio1Pin29PaEnable, true)?;

        delay.delay_ms(5);
        self.bk
            .setup_power_amplifier(self.cfg.tx_bias, self.cfg.freq_10hz)?;

        delay.delay_ms(10);
        self.bk.exit_sub_au()?;

        // Continuous test tone: leaves TX running.
        self.bk.transmit_tone(false, self.cfg.tx_tone_hz, delay)?;

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
            let irq_req = self.bk.read_register(Register::Reg0C)? & 1;
            if irq_req == 0 {
                break;
            }

            // clear interrupts first (as in C)
            self.bk.write_register(Register::Reg02, 0)?;

            // then read status bits
            let st = self.bk.read_register(Register::Reg02)?;

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

fn default_squelch_thresholds(freq_10hz: u32) -> (u8, u8, u8, u8, u8, u8) {
    // Same as the previous app-level helper, but lives here now.
    let is_vhf = freq_10hz < 17_400_000;
    if is_vhf {
        (70, 55, 50, 60, 70, 80)
    } else {
        (40, 30, 45, 55, 70, 80)
    }
}
