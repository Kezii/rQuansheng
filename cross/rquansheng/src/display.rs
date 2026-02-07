use display_interface_spi::SPIInterface;
use dp32g030::{PORTCON, SPI0, SYSCON};
use embedded_graphics::{
    mono_font::MonoTextStyle,
    pixelcolor::BinaryColor,
    prelude::{Point, Primitive, Size},
    primitives::{PrimitiveStyle, Rectangle},
    text::Text,
    Pixel,
};
use embedded_hal_bus::spi::ExclusiveDevice;
use heapless::String;
use st7565::{
    types::{BoosterRatio, PowerControlMode},
    DisplaySpecs, GraphicsPageBuffer, ST7565,
};
use static_cell::StaticCell;

use crate::{
    bk_common::BkCommonBus,
    delay::CycleDelay,
    dialer::Dialer,
    frequencies::FrequencyBand,
    radio::{ChannelConfig, Mode, Modulation, RadioController, SLevelConfig},
    radio_platform::RadioPlatform,
    ui::{FlexText, FontSizes, UiTextLineLayout, UiWidget, UiWidgetType},
};
use dp30g030_hal::{
    self,
    gpio::{Output, Pin, Port},
    spi::{self},
};

use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::Drawable;

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

static mut PAGE_BUFFER: GraphicsPageBuffer<128, 8> = GraphicsPageBuffer::new();

pub struct DisplayMgr {
    pub display: Display,
}

impl DisplayMgr {
    pub fn new(spi0: SPI0, syscon: &SYSCON, portcon: &PORTCON) -> Self {
        // --- Display (ST7565) -----------------------------------------------------
        //
        // Wiring from the reference UV-K5 firmware:
        // - SPI0: PB8=CLK, PB10=MOSI (PB9 is used as A0/DC, so we run write-only, no MISO)
        // - CS:   PB7 (we manage it as GPIO; embedded-hal `SpiBus` is bus-only)
        // - A0/DC: PB9
        // - RST:  PB11 (shared with SWDIO in stock firmware)
        let spi0_sck = spi::SckPin::<dp30g030_hal::SPI0>::new(Pin::new(Port::B, 8)).unwrap();
        let spi0_mosi = spi::MosiPin::<dp30g030_hal::SPI0>::new(Pin::new(Port::B, 10)).unwrap();
        let spi0_cfg = spi::Config::uvk5_display_default();
        let spi0: spi::Spi0 = spi::Spi::<
            dp30g030_hal::SPI0,
            spi::SckPin<dp30g030_hal::SPI0>,
            spi::MosiPin<dp30g030_hal::SPI0>,
            spi::NoMiso,
        >::new(
            spi0,
            syscon,
            portcon,
            spi0_sck,
            spi0_mosi,
            spi::NoMiso,
            spi0_cfg,
        )
        .unwrap();

        let pin_lcd_cs = Pin::new(Port::B, 7).into_push_pull_output(syscon, portcon);
        let pin_lcd_a0 = Pin::new(Port::B, 9).into_push_pull_output(syscon, portcon);
        let mut pin_lcd_rst = Pin::new(Port::B, 11).into_push_pull_output(syscon, portcon);

        let mut disp_delay = CycleDelay::new(48_000_000);

        let disp_spidevice = ExclusiveDevice::new_no_delay(spi0, pin_lcd_cs).unwrap();
        let disp_interface = SPIInterface::new(disp_spidevice, pin_lcd_a0);

        let page_buffer: &mut GraphicsPageBuffer<128, 8> = unsafe { &mut PAGE_BUFFER };
        let mut display: Display =
            ST7565::new(disp_interface, FG12864390_FKFW).into_graphics_mode(page_buffer);

        display.reset(&mut pin_lcd_rst, &mut disp_delay).ok();
        display.set_display_on(true).unwrap();
        display.flush().unwrap();

        Self { display }
    }

    pub fn show_raw_message(&mut self, message: &str) {
        self.display.clear(BinaryColor::Off).ok();
        let font = FontSizes::VerySmall.get_font_style();

        // divide message in lines, composed by a fixed number of characters

        let nlines = message.len() / 16;
        for i in 0..nlines {
            let line = &message[i * 16..(i + 1) * 16];
            Text::new(
                line,
                Point::new(0, 10 + i as i32 * 10),
                MonoTextStyle::new(font, BinaryColor::On),
            )
            .draw(&mut self.display)
            .ok();
        }

        self.display.flush().ok();
    }
}

impl<BUS, BUS1080, PLATFORM> RadioController<BUS, BUS1080, PLATFORM>
where
    BUS: BkCommonBus,
    BUS1080: BkCommonBus,
    PLATFORM: RadioPlatform,
{
    pub fn render_main(&mut self) -> Result<(), PLATFORM::Error> {
        self.platform.clear(BinaryColor::Off)?;

        // layout elements

        let main_frequency_y = 34;
        let line_1_y = 44;
        let line_2_y = 54;

        use core::fmt::Write;

        {
            let mut main_frequency = String::<8>::new();
            if self.dialer.is_dialing() {
                main_frequency = self.dialer.get_as_string();
            } else {
                write!(main_frequency, "{}", self.channel_cfg.frequency_hz / 10).ok();
            }
            let split = main_frequency.as_str().split_at_checked(6);
            let f6 = if let Some((first, _)) = split {
                first
            } else {
                main_frequency.as_str()
            };
            let l2 = if let Some((_, last)) = split {
                last
            } else {
                ""
            };
            Text::new(
                f6,
                Point::new(7, main_frequency_y),
                MonoTextStyle::new(FontSizes::LargeNumbers.get_font_style(), BinaryColor::On),
            )
            .draw(&mut self.platform)?;
            Text::new(
                l2,
                Point::new(7 + 6 * 16, main_frequency_y - 1),
                MonoTextStyle::new(FontSizes::MidNumbers.get_font_style(), BinaryColor::On),
            )
            .draw(&mut self.platform)?;
            Rectangle::new(
                Point::new(3 * 16 + 5, main_frequency_y - 2),
                Size::new(2, 2),
            )
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
            .draw(&mut self.platform)?;
        }

        if self.alt_function {
            Text::new(
                "F",
                Point::new(1, 7),
                MonoTextStyle::new(FontSizes::VerySmall.get_font_style(), BinaryColor::On),
            )
            .draw(&mut self.platform)?;
        }

        let slevel = self.get_s_level();

        let slevel_widget = match (
            self.squelch_open,
            self.dialer.is_dialing(),
            self.mode,
            slevel.over_s9_dbm,
        ) {
            // if we are dialing, show "DIAL"
            (false, true, _, _) => FlexText::from("DIAL"),
            // if we are in TX mode, show "TX"
            (false, false, Mode::Tx, _) => FlexText::from("TX"),
            // if we are in RX small signal, show the Slevel and the RSSI
            (_, false, _, ..=9) => {
                FlexText::write(|s| write!(s, "S{} {}", slevel.s_level, slevel.rssi))
            }
            // with bign signal shor S9+
            (_, false, _, 10..) => FlexText::write(|s| write!(s, "S9+{}", slevel.over_s9_dbm)),
            _ => FlexText::from(""),
        };

        let line = [
            FlexText::from_dbg(self.channel_cfg.bandwidth).with_rel_offset(8),
            FlexText::from(self.channel_cfg.output_power.to_string()).with_rel_offset(29),
            FlexText::from_dbg(self.channel_cfg.modulation).with_rel_offset(23),
            slevel_widget.with_rel_offset(23),
        ];

        UiTextLineLayout::new(line_1_y, FontSizes::VerySmall, &line).draw(&mut self.platform)?;

        let line = [
            FlexText::write(|s| {
                write!(
                    s,
                    "{}.{}k",
                    self.channel_cfg.frequency_step_hz / 1000,
                    self.channel_cfg.frequency_step_hz % 1000 / 10
                )
            })
            .with_rel_offset(8),
            FlexText::write(|s| {
                write!(
                    s,
                    "{}.{}k",
                    self.channel_cfg.frequency_offset_hz / 1000,
                    self.channel_cfg.frequency_offset_hz % 1000 / 10
                )
            })
            .with_abs_offset(80),
        ];
        UiTextLineLayout::new(line_2_y, FontSizes::VerySmallNumbers, &line)
            .draw(&mut self.platform)?;

        // header row
        {
            let battery = self.get_battery_percentage();
            let gains = self.bk.get_gain().unwrap_or_default();

            let line = [
                FlexText::write(|s| write!(s, "AGC{}", gains.agc_index))
                    .with_rel_offset(10)
                    .with_visible(
                        gains.agc_enabled && self.channel_cfg.modulation != Modulation::AM,
                    ),
                FlexText::write(|s| write!(s, "AMF{}", self.am_fix.gain_table_index))
                    .with_rel_offset(10)
                    .with_visible(self.channel_cfg.modulation == Modulation::AM),
                FlexText::write(|s| write!(s, "{}", gains.agc_sig_strength)).with_rel_offset(40),
                FlexText::from_dbg(gains.current.total_db()).with_rel_offset(20),
                FlexText::write(|s| write!(s, "{}%", battery)).with_abs_offset(100),
            ];

            UiTextLineLayout::new(8, FontSizes::VerySmallNumbers, &line)
                .draw(&mut self.platform)?;
        }

        self.vu_meter
            .update_and_draw(&mut self.platform, slevel.rssi, self.s_levels)?;

        Ok(())
    }

    pub fn render_splash<D: DrawTarget<Color = BinaryColor>>(
        &mut self,
        display: &mut D,
        splash: &[u8; 1024],
    ) -> Result<(), D::Error> {
        display.clear(BinaryColor::Off)?;

        // `splash` è nel layout ST7565 "page buffer":
        // - index = page*128 + x
        // - bit = 1 << (y%8), page = y/8
        let pixels = splash.iter().copied().enumerate().flat_map(|(i, byte)| {
            let x = (i % 128) as i32;
            let page = (i / 128) as i32;
            (0u8..8).filter_map(move |bit| {
                if (byte & (1u8 << bit)) != 0 {
                    Some(Pixel(Point::new(x, page * 8 + bit as i32), BinaryColor::On))
                } else {
                    None
                }
            })
        });
        display.draw_iter(pixels)?;

        Ok(())
    }
}

/// Scrive `value` in formato decimale fisso, senza `format!`/`core::fmt`.
///
/// - `frac_digits`: numero di cifre dopo la virgola (0 = nessuna virgola).
/// - Appende alla stringa (non la pulisce).
pub fn write_float_simple_prec<const N: usize>(
    string: &mut String<N>,
    value: f32,
    frac_digits: u8,
) {
    // Gestione casi speciali f32
    if value.is_nan() {
        let _ = string.push_str("NaN");
        return;
    }
    if value.is_infinite() {
        if value.is_sign_negative() {
            let _ = string.push('-');
        }
        let _ = string.push_str("inf");
        return;
    }

    // Segno
    let mut v = value;
    if v.is_sign_negative() {
        let _ = string.push('-');
        v = -v;
    }

    // Calcola 10^frac_digits (limitato a u64 per semplicità)
    let mut pow10: u32 = 1;
    let mut i = 0u8;
    while i < frac_digits {
        pow10 = pow10.saturating_mul(10);
        i += 1;
    }

    // Arrotondamento a frac_digits: scaled = round(v * 10^d)
    let scaled = (v * (pow10 as f32) + 0.5) as u32;
    let int_part = if pow10 == 0 { scaled } else { scaled / pow10 };
    let frac_part = if pow10 == 0 { 0 } else { scaled % pow10 };

    // Scrivi parte intera (base10) con buffer su stack.
    if int_part == 0 {
        let _ = string.push('0');
    } else {
        // u64 ha al massimo 20 cifre decimali.
        let mut buf = [0u8; 20];
        let mut len = 0usize;
        let mut n = int_part;
        while n != 0 {
            let digit = (n % 10) as u8;
            buf[len] = digit;
            len += 1;
            n /= 10;
        }
        while len != 0 {
            len -= 1;
            let _ = string.push((b'0' + buf[len]) as char);
        }
    }

    // Scrivi parte frazionaria con zeri iniziali se serve.
    if frac_digits != 0 {
        let _ = string.push('.');

        // Stampa esattamente `frac_digits` cifre (con padding a sinistra).
        // Esempio: frac_digits=3, frac_part=5 -> "005".
        let mut div: u32 = 1;
        let mut j = 1u8;
        while j < frac_digits {
            div = div.saturating_mul(10);
            j += 1;
        }
        let mut rem = frac_part;
        let mut k = 0u8;
        while k < frac_digits {
            let digit = if div == 0 { 0 } else { (rem / div) as u8 };
            let _ = string.push((b'0' + digit) as char);
            if div != 0 {
                rem %= div;
                div /= 10;
            }
            k += 1;
        }
    }
}

pub struct VuMeter {
    peak_rssi_dbm: i16,
    decay_tick: u8,
}

impl VuMeter {
    const BAR_X: i32 = 0;
    const BAR_Y: i32 = 58;
    const BAR_WIDTH: i32 = 128;
    const BAR_HEIGHT: u32 = 6;
    const MAX_OVER_S9_DBM: i16 = 30;
    const PEAK_DECAY_TICKS: u8 = 4;

    pub const fn new() -> Self {
        Self {
            peak_rssi_dbm: i16::MIN,
            decay_tick: 0,
        }
    }

    pub fn update_and_draw<D: DrawTarget<Color = BinaryColor>>(
        &mut self,
        display: &mut D,
        rssi_dbm: i16,
        s_levels: SLevelConfig,
    ) -> Result<(), D::Error> {
        self.update_peak(rssi_dbm);

        Rectangle::new(
            Point::new(Self::BAR_X, Self::BAR_Y),
            Size::new(Self::BAR_WIDTH as u32, Self::BAR_HEIGHT),
        )
        .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
        .draw(display)?;

        let current_width = self.rssi_to_width(rssi_dbm, s_levels);
        if current_width > 0 {
            Rectangle::new(
                Point::new(Self::BAR_X, Self::BAR_Y),
                Size::new(current_width as u32, Self::BAR_HEIGHT),
            )
            .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
            .draw(display)?;
        }

        self.draw_ticks(display, s_levels, current_width)?;
        self.draw_peak(display, s_levels, current_width)?;

        Ok(())
    }

    fn update_peak(&mut self, rssi_dbm: i16) {
        if self.peak_rssi_dbm == i16::MIN || rssi_dbm > self.peak_rssi_dbm {
            self.peak_rssi_dbm = rssi_dbm;
            self.decay_tick = 0;
            return;
        }

        if self.peak_rssi_dbm > rssi_dbm {
            self.decay_tick = self.decay_tick.saturating_add(1);
            if self.decay_tick >= Self::PEAK_DECAY_TICKS {
                self.peak_rssi_dbm = self.peak_rssi_dbm.saturating_sub(1);
                self.decay_tick = 0;
            }
        }
    }

    fn draw_ticks<D: DrawTarget<Color = BinaryColor>>(
        &self,
        display: &mut D,
        s_levels: SLevelConfig,
        current_width: i32,
    ) -> Result<(), D::Error> {
        let tick_height = 3u32;
        let tick_y = Self::BAR_Y + Self::BAR_HEIGHT as i32 - tick_height as i32;
        let s0_9 = s_levels.s0_level - s_levels.s9_level;
        if s0_9 <= 0 {
            return Ok(());
        }
        let s0_dbm = -s_levels.s0_level;

        for s in 1..=9 {
            let rssi_dbm = s0_dbm + (s0_9 * s) / 9;
            let x = self
                .rssi_to_width(rssi_dbm, s_levels)
                .min(Self::BAR_WIDTH - 1);
            let color = if x < current_width {
                BinaryColor::Off
            } else {
                BinaryColor::On
            };
            Rectangle::new(
                Point::new(Self::BAR_X + x, tick_y),
                Size::new(1, tick_height),
            )
            .into_styled(PrimitiveStyle::with_stroke(color, 1))
            .draw(display)?;
        }

        Ok(())
    }

    fn draw_peak<D: DrawTarget<Color = BinaryColor>>(
        &self,
        display: &mut D,
        s_levels: SLevelConfig,
        current_width: i32,
    ) -> Result<(), D::Error> {
        if self.peak_rssi_dbm == i16::MIN {
            return Ok(());
        }
        let x = self
            .rssi_to_width(self.peak_rssi_dbm, s_levels)
            .min(Self::BAR_WIDTH - 1);
        let color = if x < current_width {
            BinaryColor::Off
        } else {
            BinaryColor::On
        };
        Rectangle::new(
            Point::new(Self::BAR_X + x, Self::BAR_Y),
            Size::new(2, Self::BAR_HEIGHT),
        )
        .into_styled(PrimitiveStyle::with_fill(color))
        .draw(display)?;
        Rectangle::new(
            Point::new(Self::BAR_X + x.saturating_sub(1), Self::BAR_Y - 1),
            Size::new(4, 1),
        )
        .into_styled(PrimitiveStyle::with_fill(color))
        .draw(display)?;

        Ok(())
    }

    fn rssi_to_width(&self, rssi_dbm: i16, s_levels: SLevelConfig) -> i32 {
        let s0_dbm = -s_levels.s0_level;
        let s9_dbm = -s_levels.s9_level;
        let max_dbm = s9_dbm.saturating_add(Self::MAX_OVER_S9_DBM);
        let span = (max_dbm - s0_dbm) as i32;
        if span <= 0 {
            return 0;
        }

        let scaled = ((rssi_dbm - s0_dbm) as i32 * Self::BAR_WIDTH) / span;
        scaled.clamp(0, Self::BAR_WIDTH)
    }
}
