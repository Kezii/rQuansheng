use embedded_graphics::{
    mono_font::MonoTextStyle,
    pixelcolor::{BinaryColor, Rgb888},
    prelude::*,
    text::Text,
};
use embedded_graphics_simulator::{
    BinaryColorTheme, OutputSettingsBuilder, SimulatorDisplay, SimulatorEvent, Window,
    sdl2::{Keycode, MouseButton},
};

use rquansheng::{
    bk1080::Bk1080,
    bk4819::Bk4819Driver,
    keyboard::{KeyEvent, QuanshengKey},
    radio::{RadioController, Screen},
};
use std::time::Duration;

use host_sw::{
    dummyradiobus::{DummyRadioBus, DummyRadioBus1080},
    uartbackedbus::SerialProtocolRadioBus,
};

fn main() -> Result<(), core::convert::Infallible> {
    env_logger::init();
    let mut display = SimulatorDisplay::<BinaryColor>::new(Size::new(128, 64));
    let output_settings = OutputSettingsBuilder::new()
        .theme(BinaryColorTheme::Custom {
            color_off: Rgb888::new(216, 127, 64),
            color_on: Rgb888::new(49, 22, 13),
        })
        .scale_non_square(Size::new(8, 12))
        .build();
    let mut window = Window::new("rQuansheng", &output_settings);

    let radio_bus =
        SerialProtocolRadioBus::open("/dev/ttyUSB0", 38400, Duration::from_millis(150)).unwrap();

    //let radio_bus = DummyRadioBus;
    let mut dummy_delay = host_sw::delay::DummyDelay;

    let (tx, rx) = std::sync::mpsc::channel();

    let hosted_platform = host_sw::dummy_platform::HostedPlatform::new(rx, display, unsafe {
        std::mem::transmute(&mut window)
    });

    let bus_1080 = DummyRadioBus1080;
    let bk1080 = Bk1080::new(bus_1080);

    let mut radio = RadioController::new(Bk4819Driver::new(radio_bus), bk1080, hosted_platform);

    'main: loop {
        radio.render_display(Screen::RadioState).unwrap();

        let mut wait = false;

        let events = radio.platform.window.events().collect::<Vec<_>>();
        for event in events {
            if let SimulatorEvent::Quit = event {
                break 'main;
            }

            if let SimulatorEvent::MouseButtonDown {
                mouse_btn: MouseButton::Left,
                point,
            } = event
            {
                //draw_text_override(&mut display, &format!("{:#?}", point));
                log::info!("mouse button down: {:?}", point);
                wait = true;
            }

            if let Some(event) = simulator_event_to_quansheng_key(event) {
                tx.send(event).unwrap();
            }

            radio.eat_keyboard_event(&mut dummy_delay);
        }

        if wait {
            std::thread::sleep(std::time::Duration::from_millis(1000));
        }

        radio.poll_interrupts().ok();

        std::thread::sleep(std::time::Duration::from_millis(100));
    }

    Ok(())
}

pub fn simulator_event_to_quansheng_key(event: SimulatorEvent) -> Option<KeyEvent> {
    match event {
        SimulatorEvent::MouseButtonDown {
            mouse_btn: MouseButton::Right,
            ..
        } => Some(KeyEvent::KeyPressed(QuanshengKey::Ptt)),
        SimulatorEvent::MouseButtonUp {
            mouse_btn: MouseButton::Right,
            ..
        } => Some(KeyEvent::KeyReleased(QuanshengKey::Ptt)),

        SimulatorEvent::KeyDown { keycode, .. } => {
            keycode_to_quansheng_key(keycode).map(KeyEvent::KeyPressed)
        }
        SimulatorEvent::KeyUp { keycode, .. } => {
            keycode_to_quansheng_key(keycode).map(KeyEvent::KeyReleased)
        }
        _ => None,
    }
}

pub fn keycode_to_quansheng_key(keycode: Keycode) -> Option<QuanshengKey> {
    match keycode {
        Keycode::Num0 => Some(QuanshengKey::Num0),
        Keycode::Num1 => Some(QuanshengKey::Num1),
        Keycode::Num2 => Some(QuanshengKey::Num2),
        Keycode::Num3 => Some(QuanshengKey::Num3),
        Keycode::Num4 => Some(QuanshengKey::Num4),
        Keycode::Num5 => Some(QuanshengKey::Num5),
        Keycode::Num6 => Some(QuanshengKey::Num6),
        Keycode::Num7 => Some(QuanshengKey::Num7),
        Keycode::Num8 => Some(QuanshengKey::Num8),
        Keycode::Num9 => Some(QuanshengKey::Num9),
        Keycode::Menu => Some(QuanshengKey::Menu),
        Keycode::Up => Some(QuanshengKey::Up),
        Keycode::Down => Some(QuanshengKey::Down),
        Keycode::Escape => Some(QuanshengKey::Exit),
        Keycode::Asterisk => Some(QuanshengKey::Star),
        Keycode::F => Some(QuanshengKey::F),
        Keycode::Space => Some(QuanshengKey::Ptt),
        Keycode::F1 => Some(QuanshengKey::Side2),
        Keycode::F2 => Some(QuanshengKey::Side1),
        _ => None,
    }
}

pub fn draw_text_override(display: &mut SimulatorDisplay<BinaryColor>, text: &str) {
    let font = MonoTextStyle::new(
        &embedded_graphics::mono_font::ascii::FONT_8X13_BOLD,
        BinaryColor::On,
    );
    display.clear(BinaryColor::Off).unwrap();
    Text::new(text, Point::new(1, 13), font)
        .draw(display)
        .unwrap();
}
