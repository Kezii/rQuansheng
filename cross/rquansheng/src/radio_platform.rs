use core::convert::Infallible;

use dp30g030_hal::adc;
use dp30g030_hal::gpio::{Input, Output, Pin, Port};
use dp32g030::{PORTCON, SARADC, SPI0, SYSCON};
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::Pixel;
use embedded_hal::digital::InputPin;

use crate::board::get_ptt_pin;
use crate::delay::CycleDelay;
use crate::display::DisplayMgr;
use crate::eeprom;
use crate::keyboard::{KeyEvent, Keyboard, KeyboardState, QuanshengKey};
use embedded_graphics::prelude::{Dimensions, DrawTarget};

pub trait RadioPlatform: DrawTarget<Color = BinaryColor, Error = Infallible> {
    type EepromError;

    fn eeprom_read(&mut self, address: u16, data: &mut [u8]) -> Result<(), Self::EepromError>;
    fn eeprom_write(&mut self, address: u16, data: &[u8; 8]) -> Result<(), Self::EepromError>;

    fn set_backlight(&mut self, on: bool);
    fn set_flashlight(&mut self, on: bool);
    fn set_audio_path(&mut self, on: bool);
    fn set_bk1080(&mut self, on: bool);

    fn get_battery_adc(&mut self) -> u16;

    fn poll_keyboard(&mut self) -> Option<KeyEvent>;

    fn flush_display(&mut self) -> Result<(), Self::Error>;
}

pub struct UVK5RadioPlatform {
    pin_backlight: Pin<Output>,
    pin_flashlight: Pin<Output>,
    pin_audio_path: Pin<Output>,
    pin_bk1080_enable: Pin<Output>,
    pin_ptt: Pin<Input>,
    ptt_debouncer: DebounceBool,
    keyboard_state: KeyboardState,
    display: DisplayMgr,
    adc: adc::Adc,
}

impl UVK5RadioPlatform {
    pub fn new(spi0: SPI0, saradc: SARADC, syscon: &SYSCON, portcon: &PORTCON) -> Self {
        let pin_flashlight = Pin::new(Port::C, 3).into_push_pull_output(syscon, portcon);
        let pin_backlight = Pin::new(Port::B, 6).into_push_pull_output(syscon, portcon);
        let pin_audio_path = Pin::new(Port::C, 4).into_push_pull_output(syscon, portcon);
        let pin_bk1080_enable = Pin::new(Port::B, 15).into_push_pull_output(syscon, portcon);
        let pin_ptt = get_ptt_pin();
        let ptt_debouncer = DebounceBool::new(3);
        let display = DisplayMgr::new(spi0, syscon, portcon);

        // SARADC: battery voltage is on SARADC CH4, pin PA9.
        // C firmware conversion: v_10mV = raw * 760 / gBatteryCalibration[3].
        // We do not use EEPROM calibration here; keep a default in the middle
        // of the allowed calibration range (MENU_BATCAL is 1600..2200).
        let vbat_pin = adc::Ch4Pin::new(Pin::new(Port::A, 9)).unwrap();
        let mut adc_cfg = adc::Config::battery_default();
        adc_cfg.channel_mask = 1u16 << (adc::Channel::Ch4 as u8);
        let adc = adc::Adc::new(saradc, syscon, portcon, Some(vbat_pin), None, adc_cfg).unwrap();

        Self {
            pin_flashlight,
            pin_backlight,
            pin_audio_path,
            pin_bk1080_enable,
            pin_ptt,
            ptt_debouncer,
            keyboard_state: KeyboardState::default(),
            display,
            adc,
        }
    }
}

impl DrawTarget for UVK5RadioPlatform {
    type Color = BinaryColor;
    type Error = Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        self.display.display.draw_iter(pixels)
    }
}

impl Dimensions for UVK5RadioPlatform {
    fn bounding_box(&self) -> Rectangle {
        self.display.display.bounding_box()
    }
}

impl RadioPlatform for UVK5RadioPlatform {
    type EepromError = eeprom::Error;

    fn eeprom_read(&mut self, address: u16, data: &mut [u8]) -> Result<(), Self::EepromError> {
        // 48MHz core clock (matches usage elsewhere in the codebase).
        let mut delay = CycleDelay::new(48_000_000);
        eeprom::read_buffer(&mut delay, address, data)
    }

    fn eeprom_write(&mut self, address: u16, data: &[u8; 8]) -> Result<(), Self::EepromError> {
        // 48MHz core clock (matches usage elsewhere in the codebase).
        let mut delay = CycleDelay::new(48_000_000);
        eeprom::write_buffer_8(&mut delay, address, data)
    }

    fn set_backlight(&mut self, on: bool) {
        if on {
            let _ = embedded_hal::digital::OutputPin::set_high(&mut self.pin_backlight);
        } else {
            let _ = embedded_hal::digital::OutputPin::set_low(&mut self.pin_backlight);
        }
    }

    fn set_flashlight(&mut self, on: bool) {
        if on {
            let _ = embedded_hal::digital::OutputPin::set_high(&mut self.pin_flashlight);
        } else {
            let _ = embedded_hal::digital::OutputPin::set_low(&mut self.pin_flashlight);
        }
    }

    fn set_audio_path(&mut self, on: bool) {
        if on {
            let _ = embedded_hal::digital::OutputPin::set_high(&mut self.pin_audio_path);
        } else {
            let _ = embedded_hal::digital::OutputPin::set_low(&mut self.pin_audio_path);
        }
    }

    fn set_bk1080(&mut self, enabled: bool) {
        if enabled {
            let _ = embedded_hal::digital::OutputPin::set_low(&mut self.pin_bk1080_enable);
        } else {
            let _ = embedded_hal::digital::OutputPin::set_high(&mut self.pin_bk1080_enable);
        }
    }

    fn get_battery_adc(&mut self) -> u16 {
        self.adc.read_blocking(adc::Channel::Ch4).unwrap_or(0)
    }

    fn poll_keyboard(&mut self) -> Option<KeyEvent> {
        let pressed_now = self.pin_ptt.is_low().unwrap_or(false);
        let ptt_stable = self.ptt_debouncer.update(pressed_now);
        let key = if ptt_stable {
            Some(QuanshengKey::Ptt)
        } else {
            Keyboard::init().poll(&mut CycleDelay::new(48_000_000))
        };

        self.keyboard_state.eat_key(key)
    }

    fn flush_display(&mut self) -> Result<(), Self::Error> {
        self.display.display.flush().unwrap();
        Ok(())
    }
}

pub struct DebounceBool {
    threshold: u8,
    last_sample: bool,
    stable: bool,
    stable_count: u8,
}

impl DebounceBool {
    pub fn new(threshold: u8) -> Self {
        Self {
            threshold,
            last_sample: false,
            stable: false,
            stable_count: 0,
        }
    }

    /// Feeds one sample and returns the current stable value.
    pub fn update(&mut self, sample: bool) -> bool {
        // Threshold == 0 => no debounce.
        if self.threshold == 0 {
            self.last_sample = sample;
            self.stable = sample;
            self.stable_count = 0;
            return self.stable;
        }

        if sample == self.last_sample {
            self.stable_count = self.stable_count.saturating_add(1);
        } else {
            self.last_sample = sample;
            self.stable_count = 0;
        }

        if self.stable_count >= self.threshold {
            self.stable = sample;
        }

        self.stable
    }
}
