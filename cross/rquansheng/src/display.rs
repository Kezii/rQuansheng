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
    delay::CycleDelay,
    dialer::Dialer,
    radio::{ChannelConfig, Mode},
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

static PAGE_BUFFER: StaticCell<GraphicsPageBuffer<128, 8>> = StaticCell::new();

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

        let page_buffer = PAGE_BUFFER.init(GraphicsPageBuffer::new());
        let mut display: Display =
            ST7565::new(disp_interface, FG12864390_FKFW).into_graphics_mode(page_buffer);

        display.reset(&mut pin_lcd_rst, &mut disp_delay).ok();
        display.set_display_on(true).unwrap();
        display.flush().unwrap();

        Self { display }
    }
}

#[derive(Default)]
pub struct RenderingMgr {
    historical_rssi: CircularBuffer<u8, 128>,
}

impl RenderingMgr {
    pub fn render_main<D: DrawTarget<Color = BinaryColor>>(
        &mut self,
        display: &mut D,
        channel_cfg: ChannelConfig,
        rssi: i16,
        dialer: &Dialer<8>,
        mode: Mode,
    ) -> Result<(), D::Error> {
        display.clear(BinaryColor::Off)?;

        // layout elements

        let main_frequency_y = 35;
        let under_main_frequency_y = 43;

        use core::fmt::Write;

        let verysmallfont = MonoTextStyle::new(
            &embedded_graphics::mono_font::ascii::FONT_6X12,
            BinaryColor::On,
        );

        let font_10_digits =
            MonoTextStyle::new(&crate::fonts_generated::PROFONT_10_POINT, BinaryColor::On);
        let font_14_digits =
            MonoTextStyle::new(&crate::fonts_generated::PROFONT_14_POINT, BinaryColor::On);
        let font_24_digits =
            MonoTextStyle::new(&crate::fonts_generated::PROFONT_24_POINT, BinaryColor::On);

        {
            let mut main_frequency = String::<8>::new();
            if dialer.is_dialing() {
                main_frequency = dialer.get_as_string();
            } else {
                write!(main_frequency, "{}", channel_cfg.freq / 10).ok();
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
            Text::new(f6, Point::new(10, main_frequency_y), font_24_digits).draw(display)?;
            Text::new(
                l2,
                Point::new(10 + 6 * 16, main_frequency_y - 1),
                font_14_digits,
            )
            .draw(display)?;
            Rectangle::new(
                Point::new(3 * 16 + 8, main_frequency_y - 2),
                Size::new(2, 2),
            )
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
            .draw(display)?;
        }

        let mut bandwidth_string = String::<6>::new();
        write!(bandwidth_string, "{:?}", channel_cfg.bandwidth).ok();
        Text::new(
            &bandwidth_string,
            Point::new(16, under_main_frequency_y),
            verysmallfont,
        )
        .draw(display)?;

        let mut rssi_string = String::<6>::new();
        if dialer.is_dialing() {
            write!(rssi_string, "DIAL").ok();
        } else if mode == Mode::Tx {
            write!(rssi_string, " TX").ok();
        } else {
            write!(rssi_string, "{}", rssi).ok();
        }
        Text::new(
            &rssi_string,
            Point::new(75, under_main_frequency_y),
            verysmallfont,
        )
        .draw(display)?;

        {
            let battery = 100; // stub

            let mut battery_string = String::<6>::new();
            write!(battery_string, "{}%", battery).ok();
            Text::new(&battery_string, Point::new(100, 8), font_10_digits).draw(display)?;
        }

        self.historical_rssi.push(((rssi + 100) / 8) as u8);

        for i in 0..128 {
            if let Some(rssi) = self.historical_rssi.get(i) {
                let pixel = Pixel(Point::new(i as i32, 63 - *rssi as i32), BinaryColor::On);
                display.draw_iter(core::iter::once(pixel))?;
            }
        }

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

pub struct CircularBuffer<T, const N: usize> {
    buffer: [T; N],
    head: usize,
    tail: usize,
}

impl<T: Default, const N: usize> Default for CircularBuffer<T, N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Default, const N: usize> CircularBuffer<T, N> {
    pub fn new() -> Self {
        Self {
            buffer: core::array::from_fn(|_| T::default()),
            head: 0,
            tail: 0,
        }
    }

    pub fn push(&mut self, value: T) {
        self.buffer[self.head] = value;
        self.head = (self.head + 1) % N;
        if self.head == self.tail {
            self.tail = (self.tail + 1) % N;
        }
    }

    pub fn get(&self, index: usize) -> Option<&T> {
        if index >= N {
            return None;
        }
        let offset = (self.head + index) % N;
        Some(&self.buffer[offset])
    }

    pub fn len(&self) -> usize {
        N
    }

    pub fn is_empty(&self) -> bool {
        self.head == self.tail
    }
}
