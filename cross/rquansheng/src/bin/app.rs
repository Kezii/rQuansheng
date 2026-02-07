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
    use defmt::info;
    use dp30g030_hal::adc;
    use dp30g030_hal::gpio::{Output, Pin, Port};
    use embedded_hal::digital::{InputPin, OutputPin};
    use embedded_io_async::Read as AsyncRead;
    use heapless::Vec;
    use rquansheng::bk1080::{Bk1080, Bk1080BitBangBus};
    use rquansheng::bk4819::Bk4819Driver;
    use rquansheng::bk4819_bitbang::{bk4819_sda_pin, Bk4819BitBang};
    use rquansheng::board::{get_ptt_pin, get_uart};
    use rquansheng::delay::CycleDelay;
    use rquansheng::display::DisplayMgr;
    use rquansheng::messages::{decode_line, HostBound, RadioBound};
    use rquansheng::radio::{RadioController, Screen};
    use rquansheng::radio_platform::{DebounceBool, UVK5RadioPlatform};
    use rtic_monotonics::{fugit::ExtU32, Monotonic as _};
    use rtic_sync::signal::{Signal, SignalReader, SignalWriter};

    use crate::{Mono, SERIAL};

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
        radio: RadioController<
            Bk4819BitBang<Pin<Output>, Pin<Output>, dp30g030_hal::gpio::FlexPin, CycleDelay>,
            Bk1080BitBangBus<CycleDelay>,
            UVK5RadioPlatform,
        >,
    }

    // Local resources go here
    #[local]
    struct Local {
        adc: adc::Adc,
        display_update_reader: SignalReader<'static, bool>,
        display_update_writer: SignalWriter<'static, bool>,
        pin_flashlight: Pin<Output>,
    }

    // NOTE: BK4819 driver logic has moved to `src/bk4819/`.

    #[init(local = [poke_display_update: Signal<bool> = Signal::new()])]
    fn init(cx: init::Context) -> (Shared, Local) {
        let serial_logs = true;

        if serial_logs {
            let uart1 = get_uart();
            defmt_serial::defmt_serial(SERIAL.init(uart1));
        } else {
            uart_task::spawn().ok();
        }

        info!("Hello!");

        // this pin is in platform too, but we duplicate it because it's used in the uart task
        let pin_flashlight =
            Pin::new(Port::C, 3).into_push_pull_output(&cx.device.SYSCON, &cx.device.PORTCON);

        // BK4819 bit-bang pins: PC0=SCN, PC1=SCL, PC2=SDA (bidirectional).
        let scn = Pin::new(Port::C, 0).into_push_pull_output(&cx.device.SYSCON, &cx.device.PORTCON);
        let scl = Pin::new(Port::C, 1).into_push_pull_output(&cx.device.SYSCON, &cx.device.PORTCON);
        let sda = bk4819_sda_pin(Port::C, 2, &cx.device.SYSCON, &cx.device.PORTCON).unwrap();

        let delay_bb = CycleDelay::new(48_000_000);
        let bus = Bk4819BitBang::new(scn, scl, sda, delay_bb);
        let bk = Bk4819Driver::new(bus);

        let platform =
            UVK5RadioPlatform::new(cx.device.SPI0, &cx.device.SYSCON, &cx.device.PORTCON);

        let delay_bb_1080 = CycleDelay::new(48_000_000);
        let bus_1080 = Bk1080BitBangBus::uvk5_shared(delay_bb_1080);
        let bk1080 = Bk1080::new(bus_1080);

        let mut radio = RadioController::new(bk, bk1080, platform);

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

        Mono::start(cx.core.SYST, 48_000_000);

        info!("Initializing radio");

        if radio.init().is_err() {
            defmt::warn!("radio init failed");
        }

        radio_10ms_task::spawn().ok();
        display_task::spawn().ok();

        (
            Shared {
                // Initialization of shared resources go here
                radio,
            },
            Local {
                // Initialization of local resources go here
                adc,
                display_update_reader,
                display_update_writer,
                pin_flashlight,
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

    #[task(priority = 1, local = [pin_flashlight], shared = [radio])]
    async fn uart_task(mut cx: uart_task::Context) {
        use rquansheng::radio_platform::RadioPlatform;

        let uart1 = get_uart();
        let (mut tx, mut _rx) = uart1.split();

        loop {
            let line = read_until_zero(&mut _rx, rquansheng::messages::MAX_FRAME_LEN).await;

            let line = if let Ok(line) = line {
                line
            } else {
                Mono::delay(10.millis()).await;
                continue;
            };

            let message = decode_line::<RadioBound>(&line);

            if let Ok(message) = message {
                cx.local.pin_flashlight.set_high();

                let reply: Option<HostBound> = cx.shared.radio.lock(|r| match message {
                    RadioBound::Ping => Some(HostBound::Pong),

                    RadioBound::WriteBk4819Register(reg, value) => {
                        r.bk.__internal_write_register_raw(reg, value);
                        Some(HostBound::WriteAck(reg, value))
                    }

                    RadioBound::ReadBk4819Register(reg) => {
                        let value = r.bk.__internal_read_register_raw(reg);
                        Some(HostBound::Register(reg, value.unwrap_or(0)))
                    }

                    RadioBound::ReadEepromByte { address } => {
                        let mut d = CycleDelay::new(48_000_000);
                        let mut buf = [0u8; 1];
                        let value = rquansheng::eeprom::read_buffer(&mut d, address, &mut buf)
                            .ok()
                            .map(|_| buf[0])
                            .unwrap_or(0);

                        Some(HostBound::EepromByte { address, value })
                    }

                    RadioBound::WriteBk1080Register(reg, value) => {
                        let _ = r.bk1080.__internal_write_register_raw(reg, value);
                        Some(HostBound::WriteAck(reg, value))
                    }

                    RadioBound::ReadBk1080Register(reg) => {
                        let value = r.bk1080.__internal_read_register_raw(reg);
                        Some(HostBound::Register(reg, value.unwrap_or(0)))
                    }

                    RadioBound::SetAudioPath(on) => {
                        r.platform.set_audio_path(on);
                        None
                    }

                    RadioBound::SetBacklight(on) => {
                        r.platform.set_backlight(on);
                        None
                    }

                    RadioBound::SetFlashlight(on) => {
                        r.platform.set_flashlight(on);
                        None
                    }
                });

                if let Some(reply) = reply {
                    let _ = reply.write(&mut tx).await;
                }
                cx.local.pin_flashlight.set_low();
            } else {
                Mono::delay(10.millis()).await;
            }
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

    #[task(priority = 1, local = [display_update_reader], shared = [radio])]
    async fn display_task(mut cx: display_task::Context) {
        loop {
            let _ = cx
                .shared
                .radio
                .lock(|r| r.render_display(Screen::RadioState));

            let left = cx.local.display_update_reader.wait();
            let right = Mono::delay(500.millis());
            embassy_futures::select::select(left, right).await;
        }
    }

    /// 10ms tick task: poll+debounce PTT, poll BK4819 interrupts, and update audio.
    #[task(priority = 1, shared = [radio], local = [display_update_writer])]
    async fn radio_10ms_task(mut cx: radio_10ms_task::Context) {
        // Simple debounce (like C firmware): require 3 consecutive 10ms samples.

        loop {
            let should_update_display = cx.shared.radio.lock(|r| {
                r.eat_keyboard_event(&mut CycleDelay::new(48_000_000));

                r.poll_interrupts().ok();

                r.think_platform();

                r.should_update_display()
            });

            // we let the radio business logic decide if the display should be updated sooner

            if should_update_display {
                cx.local.display_update_writer.write(true);
            }

            Mono::delay(10.millis()).await;
        }
    }
}

// the following works but consumes about 10k of flash...
/*
use core::panic::PanicInfo;

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    //defmt::error!("Panic: {:?}", info);

    let mut buf = heapless::String::<128>::new();

    use core::fmt::Write;

    if let Some(location) = info.location() {
        write!(buf, "{}:{} {:?}", location.file(), location.line(), info.message()).ok();
    } else {
        write!(buf, "Unknown panic: {:?}", info.message()).ok();
    }

    write!(buf, "{}:{} {:?}", info.location().unwrap().file(), info.location().unwrap().line(), info.message()).ok();

    let p = unsafe { dp32g030::Peripherals::steal() };

    let mut display = rquansheng::display::DisplayMgr::new(p.SPI0, &p.SYSCON, &p.PORTCON);


    display.show_raw_message(buf.as_str());

    loop {
        continue;
    }
}
*/
