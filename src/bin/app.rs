#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use rquansheng::{self as _, dp30g030_hal}; // global logger + panicking-behavior + memory layout

use rtic_monotonics::systick::prelude::*;
use st7565::{
    types::{BoosterRatio, PowerControlMode},
    DisplaySpecs,
};
use static_cell::StaticCell;

systick_monotonic!(Mono, 1_00);

#[derive(Copy, Clone)]
struct Uptime {
    secs: u32,
    csecs: u8,
}

impl defmt::Format for Uptime {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{=u32:06}.{=u8:02}s", self.secs, self.csecs);
    }
}

// Use SysTick-based monotonic (100 Hz) as defmt timestamp: show uptime in seconds.
defmt::timestamp!("{}", {
    let ticks = Mono::now().duration_since_epoch().ticks();

    let secs = ticks / 100;
    let csecs = ticks % 100;
    Uptime {
        secs,
        csecs: csecs as u8,
    }
});

static SERIAL: StaticCell<dp30g030_hal::uart::Uart1> = StaticCell::new();

#[allow(non_camel_case_types)]
pub struct FG12864390_FKFW;
impl DisplaySpecs<128, 64, 8> for FG12864390_FKFW {
    const FLIP_ROWS: bool = false;
    const FLIP_COLUMNS: bool = true;
    const INVERTED: bool = false;
    const BIAS_MODE_1: bool = false;
    const POWER_CONTROL: PowerControlMode = PowerControlMode {
        booster_circuit: true,
        voltage_regulator_circuit: true,
        voltage_follower_circuit: true,
    };
    const VOLTAGE_REGULATOR_RESISTOR_RATIO: u8 = 0b100; // RR=4
    const ELECTRONIC_VOLUME: u8 = 31;
    const BOOSTER_RATIO: BoosterRatio = BoosterRatio::StepUp2x3x4x;
    const COLUMN_OFFSET: u8 = 4;
}

// TODO(7) Configure the `rtic::app` macro
#[rtic::app(
    device = rquansheng::dp30g030_hal,
    // TODO: Replace the `FreeInterrupt1, ...` with free interrupt vectors if software tasks are used
    // You can usually find the names of the interrupt vectors in the some_hal::pac::interrupt enum.
    dispatchers = [IWDT]
)]

mod app {
    use core::cmp::min;

    use cortex_m::asm;
    use display_interface_spi::SPIInterface;
    use embedded_graphics::draw_target::DrawTarget;
    use embedded_graphics::mono_font::ascii::FONT_8X13_BOLD;
    use embedded_graphics::mono_font::MonoTextStyle;
    use embedded_graphics::pixelcolor::BinaryColor;
    use embedded_graphics::prelude::Size;
    use embedded_graphics::prelude::{Point, Primitive};
    use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};
    use embedded_graphics::text::Text;
    use embedded_graphics::Drawable;
    use embedded_hal::delay::DelayNs;
    use embedded_hal::digital::{InputPin, OutputPin};
    use embedded_hal_bus::spi::ExclusiveDevice;
    use rquansheng::bk4819::Bk4819Driver;
    use rquansheng::bk4819_bitbang::{Bk4819, Bk4819BitBang, Dp32g030BidiPin};
    use rquansheng::dp30g030_hal::adc;
    use rquansheng::dp30g030_hal::gpio::{Input, Output, Pin, Port};
    use rquansheng::dp30g030_hal::spi;
    use rquansheng::dp30g030_hal::uart;
    use rquansheng::keyboard::KeyboardState;
    use rquansheng::radio::{Config as RadioConfig, RadioController};
    use rtic_monotonics::{fugit::ExtU32, Monotonic as _};
    use st7565::{GraphicsPageBuffer, ST7565};
    use static_cell::StaticCell;

    use crate::{number_to_string, Mono, FG12864390_FKFW};

    static PAGE_BUFFER: StaticCell<GraphicsPageBuffer<128, 8>> = StaticCell::new();

    type DisplaySpiDevice = ExclusiveDevice<spi::Spi0, Pin<Output>, embedded_hal_bus::spi::NoDelay>;
    type DisplayInterface = SPIInterface<DisplaySpiDevice, Pin<Output>>;
    type Display = ST7565<
        DisplayInterface,
        FG12864390_FKFW,
        st7565::modes::GraphicsMode<'static, 128, 8>,
        128,
        64,
        8,
    >;

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
        display: Display,
        keyboard_state: KeyboardState,
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
            uart::TxPin::<rquansheng::dp30g030_hal::UART1>::new(Pin::new(Port::A, 7)).unwrap();
        let uart1_rx =
            uart::RxPin::<rquansheng::dp30g030_hal::UART1>::new(Pin::new(Port::A, 8)).unwrap();
        let uart1_cfg = uart::Config::new(48_000_000, 38_400);
        let uart1: uart::Uart1 = uart::Uart::<
            rquansheng::dp30g030_hal::UART1,
            uart::TxPin<rquansheng::dp30g030_hal::UART1>,
            uart::RxPin<rquansheng::dp30g030_hal::UART1>,
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

        // --- Display (ST7565) -----------------------------------------------------
        //
        // Wiring from the reference UV-K5 firmware:
        // - SPI0: PB8=CLK, PB10=MOSI (PB9 is used as A0/DC, so we run write-only, no MISO)
        // - CS:   PB7 (we manage it as GPIO; embedded-hal `SpiBus` is bus-only)
        // - A0/DC: PB9
        // - RST:  PB11 (shared with SWDIO in stock firmware)
        let spi0_sck =
            spi::SckPin::<rquansheng::dp30g030_hal::SPI0>::new(Pin::new(Port::B, 8)).unwrap();
        let spi0_mosi =
            spi::MosiPin::<rquansheng::dp30g030_hal::SPI0>::new(Pin::new(Port::B, 10)).unwrap();
        let spi0_cfg = spi::Config::uvk5_display_default();
        let spi0: spi::Spi0 = spi::Spi::<
            rquansheng::dp30g030_hal::SPI0,
            spi::SckPin<rquansheng::dp30g030_hal::SPI0>,
            spi::MosiPin<rquansheng::dp30g030_hal::SPI0>,
            spi::NoMiso,
        >::new(
            cx.device.SPI0,
            &cx.device.SYSCON,
            &cx.device.PORTCON,
            spi0_sck,
            spi0_mosi,
            spi::NoMiso,
            spi0_cfg,
        )
        .unwrap();

        let pin_lcd_cs =
            Pin::new(Port::B, 7).into_push_pull_output(&cx.device.SYSCON, &cx.device.PORTCON);
        let pin_lcd_a0 =
            Pin::new(Port::B, 9).into_push_pull_output(&cx.device.SYSCON, &cx.device.PORTCON);
        let mut pin_lcd_rst =
            Pin::new(Port::B, 11).into_push_pull_output(&cx.device.SYSCON, &cx.device.PORTCON);

        let mut disp_delay = CycleDelay::new(48_000_000);

        let disp_spidevice = ExclusiveDevice::new_no_delay(spi0, pin_lcd_cs).unwrap();
        let disp_interface = SPIInterface::new(disp_spidevice, pin_lcd_a0);

        let page_buffer = PAGE_BUFFER.init(GraphicsPageBuffer::new());
        let mut display: Display =
            ST7565::new(disp_interface, FG12864390_FKFW).into_graphics_mode(page_buffer);

        display.reset(&mut pin_lcd_rst, &mut disp_delay).ok();
        display.set_display_on(true).unwrap();
        display.flush().unwrap();

        Mono::start(cx.core.SYST, 48_000_000);

        defmt::info!("init");

        // Bring up BK4819 once, then spawn control + tick tasks.
        if radio.init().is_err() {
            defmt::warn!("radio init failed");
        }

        task1::spawn().ok();
        uart_task::spawn().ok();
        radio_10ms_task::spawn().ok();
        display_task::spawn().ok();

        let keyboard_state = rquansheng::keyboard::KeyboardState::default();

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
                display,
                keyboard_state,
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
    #[task(priority = 1, local = [pin_flashlight])]
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

    // Display demo task: updates the ST7565 periodically with a simple pattern.
    #[task(priority = 1, local = [pin_backlight,display], shared = [radio])]
    async fn display_task(mut cx: display_task::Context) {
        cx.local.pin_backlight.set_high();

        loop {
            let frequency = cx.shared.radio.lock(|r| r.cfg.freq);

            cx.local.display.clear(BinaryColor::Off).unwrap();

            let font = MonoTextStyle::new(&FONT_8X13_BOLD, BinaryColor::On);
            Text::new("Hello,\nRust!", Point::new(43, 22), font)
                .draw(cx.local.display)
                .unwrap();

            // Send content to display
            cx.local.display.flush().unwrap();

            Rectangle::new(Point::new(41, 11), Size::new(50, 27))
                .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 2))
                .draw(cx.local.display)
                .unwrap();

            Text::new(
                number_to_string(frequency).as_str(),
                Point::new(30, 52),
                font,
            )
            .draw(cx.local.display)
            .unwrap();

            cx.local.display.flush().unwrap();
            Mono::delay(1000.millis()).await;
        }
    }

    /// 10ms tick task: poll+debounce PTT, poll BK4819 interrupts, and update audio.
    #[task(priority = 1, shared = [radio, audio_on], local = [pin_audio_path, pin_ptt, radio_delay, keyboard_state])]
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
                    (true, rquansheng::radio::Mode::Rx) => {
                        let _ = r.enter_tx(cx.local.radio_delay);
                    }
                    (false, rquansheng::radio::Mode::Tx) => {
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

            {
                let mut keyboard = rquansheng::keyboard::Keyboard::init();

                if let Some(key) =
                    keyboard.get_event(&mut cx.local.keyboard_state, &mut cx.local.radio_delay)
                {
                    defmt::info!("key pressed: {:?}", key);
                }
            }

            Mono::delay(10.millis()).await;
        }
    }
}

fn number_to_string(number: u32) -> heapless::String<10> {
    use core::fmt::Write as _;

    let mut string = heapless::String::new();
    write!(&mut string, "{}", number).unwrap();
    string
}
