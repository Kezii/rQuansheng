#![no_main]
#![no_std]
//#![feature(type_alias_impl_trait)]

use rquansheng::{self as _}; // global logger + panicking-behavior + memory layout

use dp30g030_hal as _;

use rtic_monotonics::systick::prelude::*;

use static_cell::StaticCell;

systick_monotonic!(Mono, 1_00);

#[derive(Copy, Clone)]
struct Uptime {
    secs: u32,
    csecs: u8,
}

impl defmt::Format for Uptime {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{=u32:06}.{=u8:02}", self.secs, self.csecs);
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

// TODO(7) Configure the `rtic::app` macro
#[rtic::app(
    device = dp30g030_hal,
    // TODO: Replace the `FreeInterrupt1, ...` with free interrupt vectors if software tasks are used
    // You can usually find the names of the interrupt vectors in the some_hal::pac::interrupt enum.
    dispatchers = [IWDT]
)]

mod app {
    use dp30g030_hal::adc;
    use dp30g030_hal::gpio::{Input, Output, Pin, Port};
    use dp30g030_hal::uart;
    use embedded_hal::digital::{InputPin, OutputPin};
    use embedded_io_async::{Read as AsyncRead, Write as AsyncWrite};
    use heapless::Vec;
    use rquansheng::bk4819::Bk4819Driver;
    use rquansheng::bk4819_bitbang::{Bk4819, Bk4819BitBang, Dp32g030BidiPin};
    use rquansheng::delay::CycleDelay;
    use rquansheng::display::DisplayMgr;
    use rquansheng::keyboard::KeyboardState;
    use rquansheng::messages::{decode_line, encode_line, HostBound, RadioBound};
    use rquansheng::radio::RadioController;
    use rtic_monotonics::{fugit::ExtU32, Monotonic as _};
    use rtic_sync::signal::{Signal, SignalReader, SignalWriter};

    use crate::Mono;

    #[derive(Debug)]
    enum ReadLineError {
        Uart,
        TooLong,
    }

    async fn read_until_zero<R>(rx: &mut R, max_len: usize) -> Result<Vec<u8, 64>, ReadLineError>
    where
        R: AsyncRead,
    {
        let limit = max_len.min(64);
        let mut out = Vec::<u8, 64>::new();
        let mut buf = [0u8; 1];

        loop {
            if out.len() >= limit {
                return Err(ReadLineError::TooLong);
            }

            // `embedded-io-async` waits until at least 1 byte is available (for non-empty buffers).
            rx.read_exact(&mut buf)
                .await
                .map_err(|_| ReadLineError::Uart)?;
            let b = buf[0];

            out.push(b).map_err(|_| ReadLineError::TooLong)?;
            if b == 0 {
                return Ok(out);
            }
        }
    }

    // Shared resources go here
    #[shared]
    struct Shared {
        radio:
            RadioController<Bk4819BitBang<Pin<Output>, Pin<Output>, Dp32g030BidiPin, CycleDelay>>,
        audio_on: bool,
        //dialer: Dialer,
    }

    // Local resources go here
    #[local]
    struct Local {
        pin_flashlight: Pin<Output>,
        pin_backlight: Pin<Output>,
        uart1: Option<uart::Uart1>,
        pin_audio_path: Pin<Output>,
        pin_ptt: Pin<Input>,
        adc: adc::Adc,
        keyboard_state: KeyboardState,
        display: DisplayMgr,
        display_update_reader: SignalReader<'static, bool>,
        display_update_writer: SignalWriter<'static, bool>,
    }

    // NOTE: BK4819 driver logic has moved to `src/bk4819/`.

    #[init(local = [poke_display_update: Signal<bool> = Signal::new()])]
    fn init(cx: init::Context) -> (Shared, Local) {
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
        let bus = Bk4819BitBang::new(scn, scl, sda, delay_bb);
        let bk = Bk4819Driver::new(Bk4819::new(bus));

        let mut radio = RadioController::new(bk);

        // PTT is PC5, active-low, with pull-up enabled in the reference firmware.
        // We do simple polling + debounce in `radio_10ms_task`.
        let pin_ptt: Pin<Input> =
            Pin::new(Port::C, 5).into_pull_up_input(&cx.device.SYSCON, &cx.device.PORTCON);

        // UART example: UART1 on PA7 (TX) / PA8 (RX), 38400-8N1.
        let uart1_tx = uart::TxPin::<dp30g030_hal::UART1>::new(Pin::new(Port::A, 7)).unwrap();
        let uart1_rx = uart::RxPin::<dp30g030_hal::UART1>::new(Pin::new(Port::A, 8)).unwrap();
        let uart1_cfg = uart::Config::new(48_000_000, 38_400);
        let uart1: uart::Uart1 = uart::Uart::<
            dp30g030_hal::UART1,
            uart::TxPin<dp30g030_hal::UART1>,
            uart::RxPin<dp30g030_hal::UART1>,
        >::new(
            cx.device.UART1,
            &cx.device.SYSCON,
            &cx.device.PORTCON,
            uart1_tx,
            uart1_rx,
            uart1_cfg,
        )
        .unwrap();

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

        let (display_update_writer, display_update_reader) = cx.local.poke_display_update.split();

        let display = DisplayMgr::new(cx.device.SPI0, &cx.device.SYSCON, &cx.device.PORTCON);

        Mono::start(cx.core.SYST, 48_000_000);

        defmt::info!("init");

        if radio.init().is_err() {
            defmt::warn!("radio init failed");
        }

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
                uart1: Some(uart1),
                pin_audio_path,
                pin_ptt,
                adc,
                display,
                keyboard_state,
                display_update_reader,
                display_update_writer,
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

    #[task(priority = 1, local = [uart1, pin_flashlight], shared = [radio])]
    async fn uart_task(mut cx: uart_task::Context) {
        //defmt_serial::defmt_serial(crate::SERIAL.init(uart1));
        let uart1 = cx
            .local
            .uart1
            .take()
            .expect("uart_task started without uart1");
        let (mut tx, mut _rx) = uart1.split();

        loop {
            let line = read_until_zero(&mut _rx, 64).await;
            let line = line.unwrap_or(Vec::new());
            cx.local.pin_flashlight.set_high();

            let message = decode_line::<RadioBound>(&line);

            if let Ok(message) = message {
                match message {
                    RadioBound::Ping => {
                        let reply = HostBound::Pong;
                        let reply_encoded = encode_line(&reply).unwrap();
                        tx.write_all(&reply_encoded).await.unwrap();
                    }

                    RadioBound::WriteRegister(reg, value) => {
                        cx.shared
                            .radio
                            .lock(|r| r.bk.__internal_write_register_raw(reg, value));
                        let reply = HostBound::WriteAck(reg, value);
                        let reply_encoded = encode_line(&reply).unwrap();
                        tx.write_all(&reply_encoded).await.unwrap();
                    }

                    RadioBound::ReadRegister(reg) => {
                        let value = cx
                            .shared
                            .radio
                            .lock(|r| r.bk.__internal_read_register_raw(reg));
                        let reply = HostBound::Register(reg, value.unwrap_or(0));
                        let reply_encoded = encode_line(&reply).unwrap();
                        tx.write_all(&reply_encoded).await.unwrap();
                    }

                    _ => {}
                }
            }
            cx.local.pin_flashlight.set_low();
        }
    }

    // UART demo task: writes a message periodically on UART1.
    #[task(priority = 1, local = [adc])]
    async fn adc_task(cx: adc_task::Context) {
        loop {
            let raw = cx.local.adc.read_blocking(adc::Channel::Ch4).unwrap_or(0);

            defmt::info!("battery: (raw={=u16})", raw);

            Mono::delay(2.secs()).await;
        }
    }

    #[task(priority = 1, local = [pin_backlight,display,display_update_reader], shared = [radio])]
    async fn display_task(mut cx: display_task::Context) {
        cx.local.pin_backlight.set_high();

        loop {
            let _ = cx
                .shared
                .radio
                .lock(|r| r.render_display(&mut cx.local.display.display));

            cx.local.display.display.flush().unwrap();

            let left = cx.local.display_update_reader.wait();
            let right = Mono::delay(100.millis());
            embassy_futures::select::select(left, right).await;
        }
    }

    /// 10ms tick task: poll+debounce PTT, poll BK4819 interrupts, and update audio.
    #[task(priority = 1, shared = [radio, audio_on], local = [pin_audio_path, pin_ptt, keyboard_state,display_update_writer])]
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

            let key = if ptt_stable {
                Some(rquansheng::keyboard::QuanshengKey::Ptt)
            } else {
                rquansheng::keyboard::Keyboard::init().poll(&mut CycleDelay::new(48_000_000))
            };

            let event = cx.local.keyboard_state.eat_key(key);

            // Do everything that touches BK4819 under one lock, then act on the GPIO audio path.

            let desired_audio_on = cx.shared.radio.lock(|r| {
                r.eat_keyboard_event(event, &mut CycleDelay::new(48_000_000));

                //r.eat_ptt(ptt_stable, &mut cx.local.radio_delay);

                // Poll BK IRQ/status (C-style) and update internal state/LED/AF.
                r.poll_interrupts().ok();

                // Board audio path should follow controller's desired state.
                r.desired_audio_on()
            });

            cx.shared.audio_on.lock(|a| *a = desired_audio_on);

            if desired_audio_on {
                let _ = cx.local.pin_audio_path.set_high();
            } else {
                let _ = cx.local.pin_audio_path.set_low();
            }

            // if a key was pressed, we hurry a display update
            if let Some(key) = event {
                cx.local.display_update_writer.write(true);
                defmt::info!("key pressed: {:?}", key);
            }

            Mono::delay(10.millis()).await;
        }
    }
}
