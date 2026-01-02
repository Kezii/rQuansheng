#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use rquangsheng::{self as _, dp30g030_hal}; // global logger + panicking-behavior + memory layout

use rtic_monotonics::systick::prelude::*;
use static_cell::StaticCell;

systick_monotonic!(Mono, 1_00);

// Use SysTick-based monotonic (100 Hz) as defmt timestamp: show uptime in seconds.
defmt::timestamp!("{=f32}s", {
    let ticks = Mono::now().duration_since_epoch().ticks();

    ticks as f32 / 100.0
});

static SERIAL: StaticCell<dp30g030_hal::uart::Uart1> = StaticCell::new();

// TODO(7) Configure the `rtic::app` macro
#[rtic::app(
    device = rquangsheng::dp30g030_hal,
    // TODO: Replace the `FreeInterrupt1, ...` with free interrupt vectors if software tasks are used
    // You can usually find the names of the interrupt vectors in the some_hal::pac::interrupt enum.
    dispatchers = [IWDT]
)]

mod app {
    use core::cmp::min;

    use cortex_m::asm;
    use embedded_hal::delay::DelayNs;
    use embedded_hal::digital::OutputPin;
    use rquangsheng::bk4819_bitbang::{Bk4819, Bk4819BitBang, Dp32g030BidiPin};
    use rquangsheng::dp30g030_hal::gpio::{Output, Pin, Port};
    use rquangsheng::dp30g030_hal::uart;
    use rtic_monotonics::{fugit::ExtU32, Monotonic as _};

    use crate::Mono;

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
    }

    // Local resources go here
    #[local]
    struct Local {
        pin_flashlight: Pin<Output>,
        pin_backlight: Pin<Output>,
        //uart1: uart::Uart1,
        pin_audio_path: Pin<Output>,
        bk: Bk4819<Bk4819BitBang<Pin<Output>, Pin<Output>, Dp32g030BidiPin, CycleDelay>>,
    }

    /// Simple busy-wait delay based on core clock.
    ///
    /// This is used only for BK4819 bit-banging (short delays on the order of 1µs).
    struct CycleDelay {
        cycles_per_us: u32,
    }

    impl CycleDelay {
        const fn new(cpu_hz: u32) -> Self {
            Self {
                cycles_per_us: cpu_hz / 1_000_000,
            }
        }

        #[inline(always)]
        fn delay_cycles(&mut self, cycles: u32) {
            // cortex-m busy loop; `0` is fine.
            asm::delay(cycles);
        }
    }

    impl DelayNs for CycleDelay {
        #[inline]
        fn delay_ns(&mut self, ns: u32) {
            // cycles = cpu_hz * ns / 1e9
            // Round up a bit to avoid being too fast.
            let cycles = ((self.cycles_per_us as u64) * (ns as u64)).div_ceil(1000);
            self.delay_cycles(min(cycles, u32::MAX as u64) as u32);
        }

        #[inline]
        fn delay_us(&mut self, us: u32) {
            self.delay_cycles(self.cycles_per_us.saturating_mul(us));
        }

        #[inline]
        fn delay_ms(&mut self, ms: u32) {
            self.delay_us(ms.saturating_mul(1_000));
        }
    }

    // --- Minimal BK4819 init + tone (ported from uv-k5-firmware-custom) ---

    const BK_REG_00: u8 = 0x00;
    const BK_REG_0C: u8 = 0x0C;
    const BK_REG_09: u8 = 0x09;
    const BK_REG_10: u8 = 0x10;
    const BK_REG_11: u8 = 0x11;
    const BK_REG_12: u8 = 0x12;
    const BK_REG_13: u8 = 0x13;
    const BK_REG_14: u8 = 0x14;
    const BK_REG_19: u8 = 0x19;
    const BK_REG_1F: u8 = 0x1F;
    const BK_REG_30: u8 = 0x30;
    const BK_REG_33: u8 = 0x33;
    const BK_REG_36: u8 = 0x36;
    const BK_REG_37: u8 = 0x37;
    const BK_REG_3E: u8 = 0x3E;
    const BK_REG_3F: u8 = 0x3F;
    const BK_REG_47: u8 = 0x47;
    const BK_REG_48: u8 = 0x48;
    const BK_REG_49: u8 = 0x49;
    const BK_REG_50: u8 = 0x50;
    const BK_REG_70: u8 = 0x70;
    const BK_REG_71: u8 = 0x71;
    const BK_REG_7B: u8 = 0x7B;
    const BK_REG_7D: u8 = 0x7D;
    const BK_REG_7E: u8 = 0x7E;

    const BK_REG_30_ENABLE_AF_DAC: u16 = 1 << 9;
    const BK_REG_30_ENABLE_DISC_MODE: u16 = 1 << 8;
    const BK_REG_30_ENABLE_TX_DSP: u16 = 1 << 1;

    const BK_REG_70_ENABLE_TONE1: u16 = 1 << 15;
    const BK_REG_70_SHIFT_TONE1_GAIN: u16 = 8;

    const BK_AF_MUTE: u16 = 0;
    const BK_AF_BEEP: u16 = 3;

    #[inline]
    fn scale_freq(freq_hz: u16) -> u16 {
        // Matches C firmware:
        // (((freq * 1353245) + (1 << 16)) >> 17)
        ((((freq_hz as u32) * 1_353_245u32) + (1u32 << 16)) >> 17) as u16
    }

    fn bk_set_af<BUS>(bk: &mut Bk4819<BUS>, af: u16) -> Result<(), BUS::Error>
    where
        BUS: rquangsheng::bk4819_bitbang::Bk4819Bus,
    {
        // C: (6u<<12) | (AF<<8) | (1u<<6)
        bk.write_reg(BK_REG_47, (6u16 << 12) | ((af & 0x0F) << 8) | (1u16 << 6))
    }

    fn bk_enter_tx_mute<BUS>(bk: &mut Bk4819<BUS>) -> Result<(), BUS::Error>
    where
        BUS: rquangsheng::bk4819_bitbang::Bk4819Bus,
    {
        bk.write_reg(BK_REG_50, 0xBB20)
    }

    fn bk_exit_tx_mute<BUS>(bk: &mut Bk4819<BUS>) -> Result<(), BUS::Error>
    where
        BUS: rquangsheng::bk4819_bitbang::Bk4819Bus,
    {
        bk.write_reg(BK_REG_50, 0x3B20)
    }

    fn bk_set_agc<BUS>(bk: &mut Bk4819<BUS>, enable: bool) -> Result<(), BUS::Error>
    where
        BUS: rquangsheng::bk4819_bitbang::Bk4819Bus,
    {
        let reg_val = bk.read_reg(BK_REG_7E)?;
        let currently_enabled = (reg_val & (1u16 << 15)) == 0;
        if currently_enabled == enable {
            return Ok(());
        }

        let next = (reg_val & !(1u16 << 15) & !(0b111u16 << 12))
            | ((!enable as u16) << 15) // 0=auto(AGC on), 1=fix(AGC off)
            | (3u16 << 12); // fix index
        bk.write_reg(BK_REG_7E, next)
    }

    fn bk_init_agc<BUS>(bk: &mut Bk4819<BUS>, am_modulation: bool) -> Result<(), BUS::Error>
    where
        BUS: rquangsheng::bk4819_bitbang::Bk4819Bus,
    {
        // Port of BK4819_InitAGC(false) minimal subset.
        bk.write_reg(BK_REG_13, 0x03BE)?;
        bk.write_reg(BK_REG_12, 0x037B)?;
        bk.write_reg(BK_REG_11, 0x027B)?;
        bk.write_reg(BK_REG_10, 0x007A)?;

        if am_modulation {
            bk.write_reg(BK_REG_14, 0x0000)?;
            bk.write_reg(BK_REG_49, (0u16 << 14) | (50u16 << 7) | (32u16 << 0))?;
        } else {
            bk.write_reg(BK_REG_14, 0x0019)?;
            bk.write_reg(BK_REG_49, (0u16 << 14) | (84u16 << 7) | (56u16 << 0))?;
        }

        bk.write_reg(BK_REG_7B, 0x8420)?;
        Ok(())
    }

    fn bk_init<BUS>(bk: &mut Bk4819<BUS>) -> Result<(), BUS::Error>
    where
        BUS: rquangsheng::bk4819_bitbang::Bk4819Bus,
    {
        // Soft reset
        bk.write_reg(BK_REG_00, 0x8000)?;
        bk.write_reg(BK_REG_00, 0x0000)?;

        bk.write_reg(BK_REG_37, 0x1D0F)?;
        bk.write_reg(BK_REG_36, 0x0022)?;

        bk_init_agc(bk, false)?;
        bk_set_agc(bk, true)?;

        bk.write_reg(BK_REG_19, 0x1041)?;
        bk.write_reg(BK_REG_7D, 0xE940)?;

        // RX AF level / DAC gain setup
        bk.write_reg(
            BK_REG_48,
            (11u16 << 12) | (0u16 << 10) | (58u16 << 4) | (8u16 << 0),
        )?;

        // DTMF coefficients table (kept from reference init)
        const DTMF_COEFFS: [u16; 16] = [
            111, 107, 103, 98, 80, 71, 58, 44, 65, 55, 37, 23, 228, 203, 181, 159,
        ];
        for (i, &c) in DTMF_COEFFS.iter().enumerate() {
            bk.write_reg(BK_REG_09, ((i as u16) << 12) | c)?;
        }

        bk.write_reg(BK_REG_1F, 0x5454)?;
        bk.write_reg(BK_REG_3E, 0xA037)?;

        // GPIO out state / interrupts mask
        bk.write_reg(BK_REG_33, 0x9000)?;
        bk.write_reg(BK_REG_3F, 0x0000)?;

        // Mute AF by default.
        bk_set_af(bk, BK_AF_MUTE)?;
        Ok(())
    }

    fn bk_start_tone<BUS>(bk: &mut Bk4819<BUS>, freq_hz: u16) -> Result<(), BUS::Error>
    where
        BUS: rquangsheng::bk4819_bitbang::Bk4819Bus,
    {
        // Mirrors BK4819_PlayTone() (setup only; caller can ExitTxMute() to make it audible).
        bk_enter_tx_mute(bk)?;
        bk_set_af(bk, BK_AF_BEEP)?;

        // Use the "tuning gain switch = true" path from C beep code (lower gain).
        let tone_gain: u16 = 28;
        bk.write_reg(
            BK_REG_70,
            BK_REG_70_ENABLE_TONE1 | (tone_gain << BK_REG_70_SHIFT_TONE1_GAIN),
        )?;

        bk.write_reg(BK_REG_30, 0x0000)?;
        bk.write_reg(
            BK_REG_30,
            BK_REG_30_ENABLE_AF_DAC | BK_REG_30_ENABLE_DISC_MODE | BK_REG_30_ENABLE_TX_DSP,
        )?;

        bk.write_reg(BK_REG_71, scale_freq(freq_hz))?;
        Ok(())
    }

    fn bk_log_reg<BUS>(bk: &mut Bk4819<BUS>, reg: u8, name: &'static str)
    where
        BUS: rquangsheng::bk4819_bitbang::Bk4819Bus,
    {
        match bk.read_reg(reg) {
            Ok(v) => defmt::info!("BK4819 {=str} (0x{=u8:02x}) = 0x{=u16:04x}", name, reg, v),
            Err(_) => defmt::warn!("BK4819 read failed: {=str} (0x{=u8:02x})", name, reg),
        }
    }

    fn bk_write_verify_strict<BUS>(
        bk: &mut Bk4819<BUS>,
        reg: u8,
        value: u16,
        name: &'static str,
    ) -> Result<(), ()>
    where
        BUS: rquangsheng::bk4819_bitbang::Bk4819Bus,
    {
        if bk.write_reg(reg, value).is_err() {
            defmt::error!(
                "BK4819 write failed: {=str} (0x{=u8:02x}) <= 0x{=u16:04x}",
                name,
                reg,
                value
            );
            return Err(());
        }
        match bk.read_reg(reg) {
            Ok(rb) => {
                defmt::assert!(
                    rb == value,
                    "BK4819 mismatch: {=str} (0x{=u8:02x}) wrote 0x{=u16:04x} read 0x{=u16:04x}",
                    name,
                    reg,
                    value,
                    rb
                );
                defmt::info!(
                    "BK4819 write/readback OK: {=str} (0x{=u8:02x}) = 0x{=u16:04x}",
                    name,
                    reg,
                    rb
                );
                Ok(())
            }
            Err(_) => {
                defmt::error!("BK4819 readback failed: {=str} (0x{=u8:02x})", name, reg);
                Err(())
            }
        }
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        // TODO setup monotonic if used
        // let sysclk = { /* clock setup + returning sysclk as an u32 */ };
        // let token = rtic_monotonics::create_systick_token!();
        // rtic_monotonics::systick::Systick::new(cx.core.SYST, sysclk, token);

        let pin_flashlight =
            Pin::new(Port::C, 3).into_push_pull_output(&cx.device.SYSCON, &cx.device.PORTCON);
        let pin_backlight =
            Pin::new(Port::B, 6).into_push_pull_output(&cx.device.SYSCON, &cx.device.PORTCON);
        let mut pin_audio_path =
            Pin::new(Port::C, 4).into_push_pull_output(&cx.device.SYSCON, &cx.device.PORTCON);
        // Start with audio path OFF like the reference firmware does.
        let _ = pin_audio_path.set_low();

        // BK4819 bit-bang pins: PC0=SCN, PC1=SCL, PC2=SDA (bidirectional).
        let scn = Pin::new(Port::C, 0).into_push_pull_output(&cx.device.SYSCON, &cx.device.PORTCON);
        let scl = Pin::new(Port::C, 1).into_push_pull_output(&cx.device.SYSCON, &cx.device.PORTCON);
        let sda = Dp32g030BidiPin::new(Port::C, 2, &cx.device.SYSCON, &cx.device.PORTCON).unwrap();

        let delay = CycleDelay::new(48_000_000);
        let bus = Bk4819BitBang::new(scn, scl, sda, delay).unwrap();
        let bk = Bk4819::new(bus);

        // UART example: UART1 on PA7 (TX) / PA8 (RX), 38400-8N1.
        let uart1_tx =
            uart::TxPin::<rquangsheng::dp30g030_hal::UART1>::new(Pin::new(Port::A, 7)).unwrap();
        let uart1_rx =
            uart::RxPin::<rquangsheng::dp30g030_hal::UART1>::new(Pin::new(Port::A, 8)).unwrap();
        let uart1_cfg = uart::Config::new(48_000_000, 38_400);
        let uart1: uart::Uart1 = uart::Uart::<
            rquangsheng::dp30g030_hal::UART1,
            uart::TxPin<rquangsheng::dp30g030_hal::UART1>,
            uart::RxPin<rquangsheng::dp30g030_hal::UART1>,
        >::new(
            cx.device.UART1,
            &cx.device.SYSCON,
            &cx.device.PORTCON,
            uart1_tx,
            uart1_rx,
            uart1_cfg,
        )
        .unwrap();

        defmt_serial::defmt_serial(crate::SERIAL.init(uart1));

        Mono::start(cx.core.SYST, 48_000_000);

        defmt::info!("init");

        task1::spawn().ok();
        uart_task::spawn().ok();
        beep_task::spawn().ok();

        (
            Shared {
                // Initialization of shared resources go here
            },
            Local {
                // Initialization of local resources go here
                pin_flashlight,
                pin_backlight,
                //uart1,
                pin_audio_path,
                bk,
            },
        )
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            continue;
        }
    }

    // TODO: Add tasks
    #[task(priority = 1, local = [pin_flashlight, pin_backlight])]
    async fn task1(cx: task1::Context<Local>) {
        defmt::info!("Hello from task1!");

        loop {
            cx.local.pin_flashlight.set_high();
            cx.local.pin_backlight.set_low();
            Mono::delay(500.millis()).await;

            cx.local.pin_flashlight.set_low();
            cx.local.pin_backlight.set_high();
            Mono::delay(500.millis()).await;
        }
    }

    // UART demo task: writes a message periodically on UART1.
    #[task(priority = 1)]
    async fn uart_task(_cx: uart_task::Context) {
        loop {
            defmt::info!("log task");

            Mono::delay(2.secs()).await;
        }
    }

    /// Bring up the BK4819 and start a continuous tone on the speaker.
    ///
    /// This is intentionally "no RF": we only reset/init the chip and enable the audio DAC + tone generator.
    #[task(priority = 1, local = [bk, pin_audio_path])]
    async fn beep_task(cx: beep_task::Context) {
        defmt::info!("BK4819 init + start tone");

        // Small settle time similar to C firmware beep path.
        Mono::delay(20.millis()).await;

        // --- Bus sanity checks (read/write/readback) ---
        bk_log_reg(cx.local.bk, BK_REG_0C, "REG_0C status (pre)");
        bk_log_reg(cx.local.bk, BK_REG_7E, "REG_7E AGC (pre)");

        // Try a "safe" R/W register before we do anything else.
        // REG_33 is used by the C firmware as GPIO out state (written early during init).
        if bk_write_verify_strict(
            cx.local.bk,
            BK_REG_33,
            0x9000,
            "REG_33 gpio_out_state (probe)",
        )
        .is_err()
        {
            defmt::error!("BK4819 bus probe failed (REG_33)");
            return;
        }

        // Init BK4819 core configuration.
        if let Err(_) = bk_init(cx.local.bk) {
            defmt::warn!("BK4819 init failed");
            return;
        }

        // Program tone generator at 880Hz, then enable audio path and unmute.
        if let Err(_) = bk_start_tone(cx.local.bk, 880) {
            defmt::warn!("BK4819 tone setup failed");
            return;
        }

        Mono::delay(2.millis()).await;
        let _ = cx.local.pin_audio_path.set_high(); // speaker/amp enable
        Mono::delay(60.millis()).await;

        if let Err(_) = bk_exit_tx_mute(cx.local.bk) {
            defmt::warn!("BK4819 exit-tx-mute failed");
            return;
        }

        defmt::info!("Tone running (880Hz)");
        loop {
            Mono::delay(5.secs()).await;
        }
    }
}
