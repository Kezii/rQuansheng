use embedded_graphics::{
    pixelcolor::{BinaryColor, Rgb888},
    prelude::*,
};
use embedded_graphics_simulator::{
    BinaryColorTheme, OutputSettingsBuilder, SimulatorDisplay, Window,
};
use rquansheng::{dialer::Dialer, display::RenderingMgr, radio::ChannelConfig};

fn main() -> Result<(), core::convert::Infallible> {
    let mut display = SimulatorDisplay::<BinaryColor>::new(Size::new(128, 64));

    let channel_cfg = ChannelConfig::default();
    let dialer = Dialer::default();
    let rssi = 0.0;

    RenderingMgr::render_main(&mut display, channel_cfg, rssi, &dialer)?;

    let output_settings = OutputSettingsBuilder::new()
        .theme(BinaryColorTheme::Custom {
            color_off: Rgb888::new(216, 127, 64),
            color_on: Rgb888::new(49, 22, 13),
        })
        .scale(4)
        .pixel_aspect_ratio(2, 3)
        .build();
    Window::new("rQuansheng", &output_settings).show_static(&display);

    Ok(())
}
