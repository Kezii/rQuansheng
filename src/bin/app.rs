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
    use embedded_hal::digital::{InputPin, OutputPin};
    use rquangsheng::bk4819::Bk4819Driver;
    use rquangsheng::bk4819_bitbang::{Bk4819, Bk4819BitBang, Dp32g030BidiPin};
    use rquangsheng::dp30g030_hal::adc;
    use rquangsheng::dp30g030_hal::gpio::{Input, Output, Pin, Port};
    use rquangsheng::dp30g030_hal::uart;
    use rquangsheng::radio::{Config as RadioConfig, RadioController};
    use rtic_monotonics::{fugit::ExtU32, Monotonic as _};

    use crate::Mono;

    // Shared resources go here
    #[shared]
    struct Shared {
        radio:
            RadioController<Bk4819BitBang<Pin<Output>, Pin<Output>, Dp32g030BidiPin, CycleDelay>>,
        audio_on: bool,
    }

    // Local resources go here
    #[local]
    struct Local {
        pin_flashlight: Pin<Output>,
        pin_backlight: Pin<Output>,
        //uart1: uart::Uart1,
        pin_audio_path: Pin<Output>,
        pin_ptt: Pin<Input>,
        adc: adc::Adc,
        radio_delay: CycleDelay,
    }

    /// Simple busy-wait delay based on core clock.
    ///
    /// This is used only for BK4819 bit-banging (short delays on the order of 1µs).
    pub struct CycleDelay {
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

    // NOTE: BK4819 driver logic has moved to `src/bk4819/`.

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

        let delay_bb = CycleDelay::new(48_000_000);
        let bus = Bk4819BitBang::new(scn, scl, sda, delay_bb).unwrap();
        let bk = Bk4819Driver::new(Bk4819::new(bus));
        let mut radio = RadioController::new(bk, RadioConfig::default_uhf_433());

        // PTT is PC5, active-low, with pull-up enabled in the reference firmware.
        // We do simple polling + debounce in `radio_10ms_task`.
        let pin_ptt: Pin<Input> =
            Pin::new(Port::C, 5).into_pull_up_input(&cx.device.SYSCON, &cx.device.PORTCON);

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

        // SARADC: battery voltage is on SARADC CH4, pin PA9.
        // C firmware conversion: v_10mV = raw * 760 / gBatteryCalibration[3].
        // We do not use EEPROM calibration here; keep a default in the middle
        // of the allowed calibration range (MENU_BATCAL is 1600..2200).
        let vbat_pin = adc::Ch4Pin::new(Pin::new(Port::A, 9)).unwrap();
        let mut adc_cfg = adc::Config::battery_default();
        adc_cfg.channel_mask = 1u16 << (adc::Channel::Ch4 as u8);
        let adc = adc::Adc::new(
            cx.device.SARADC,
            &cx.device.SYSCON,
            &cx.device.PORTCON,
            Some(vbat_pin),
            None,
            adc_cfg,
        )
        .unwrap();

        Mono::start(cx.core.SYST, 48_000_000);

        defmt::info!("init");

        // Bring up BK4819 once, then spawn control + tick tasks.
        if radio.init().is_err() {
            defmt::warn!("radio init failed");
        }

        task1::spawn().ok();
        uart_task::spawn().ok();
        radio_10ms_task::spawn().ok();

        (
            Shared {
                // Initialization of shared resources go here
                radio,
                audio_on: false,
            },
            Local {
                // Initialization of local resources go here
                pin_flashlight,
                pin_backlight,
                //uart1,
                pin_audio_path,
                pin_ptt,
                adc,
                radio_delay: CycleDelay::new(48_000_000),
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
            //cx.local.pin_backlight.set_low();
            Mono::delay(10.millis()).await;

            cx.local.pin_flashlight.set_low();
            //cx.local.pin_backlight.set_high();
            Mono::delay(500.millis()).await;
        }
    }

    // UART demo task: writes a message periodically on UART1.
    #[task(priority = 1, local = [adc])]
    async fn uart_task(cx: uart_task::Context) {
        loop {
            let raw = cx.local.adc.read_blocking(adc::Channel::Ch4).unwrap_or(0);

            defmt::info!("battery: (raw={=u16})", raw);

            Mono::delay(2.secs()).await;
        }
    }

    /// 10ms tick task: poll+debounce PTT, poll BK4819 interrupts, and update audio.
    #[task(priority = 1, shared = [radio, audio_on], local = [pin_audio_path, pin_ptt, radio_delay])]
    async fn radio_10ms_task(mut cx: radio_10ms_task::Context) {
        // Simple debounce (like C firmware): require 3 consecutive 10ms samples.
        let mut ptt_last_sample = false;
        let mut ptt_stable = false;
        let mut ptt_stable_count: u8 = 0;

        loop {
            // Active-low: pressed when pin is low.
            let pressed_now = cx.local.pin_ptt.is_low().unwrap_or(false);

            if pressed_now == ptt_last_sample {
                ptt_stable_count = ptt_stable_count.saturating_add(1);
            } else {
                ptt_last_sample = pressed_now;
                ptt_stable_count = 0;
            }

            if ptt_stable_count >= 3 {
                ptt_stable = pressed_now;
            }

            // Do everything that touches BK4819 under one lock, then act on the GPIO audio path.
            let desired_audio_on = cx.shared.radio.lock(|r| {
                // PTT-driven state transitions (RX <-> TX).
                match (ptt_stable, r.mode()) {
                    (true, rquangsheng::radio::Mode::Rx) => {
                        let _ = r.enter_tx(cx.local.radio_delay);
                    }
                    (false, rquangsheng::radio::Mode::Tx) => {
                        let _ = r.enter_rx();
                    }
                    _ => {}
                }

                // Poll BK IRQ/status (C-style) and update internal state/LED/AF.
                let _ = r.poll_interrupts();

                // Board audio path should follow controller's desired state.
                r.desired_audio_on()
            });

            cx.shared.audio_on.lock(|a| *a = desired_audio_on);

            if desired_audio_on {
                let _ = cx.local.pin_audio_path.set_high();
            } else {
                let _ = cx.local.pin_audio_path.set_low();
            }

            Mono::delay(10.millis()).await;
        }
    }
}
