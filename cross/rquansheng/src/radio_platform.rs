use dp30g030_hal::gpio::{Output, Pin, Port};
use dp32g030::{PORTCON, SYSCON};

use crate::delay::CycleDelay;
use crate::eeprom;

pub trait RadioPlatform {
    type EepromError;

    fn eeprom_read(&mut self, address: u16, data: &mut [u8]) -> Result<(), Self::EepromError>;
    fn eeprom_write(&mut self, address: u16, data: &[u8; 8]) -> Result<(), Self::EepromError>;

    fn set_backlight(&mut self, on: bool);
    fn set_flashlight(&mut self, on: bool);
    fn set_audio_path(&mut self, on: bool);
    fn bk1080_enabled(&mut self, enabled: bool);
}

pub struct UVK5RadioPlatform {
    pin_backlight: Pin<Output>,
    pin_flashlight: Pin<Output>,
    pin_audio_path: Pin<Output>,
    pin_bk1080_enable: Pin<Output>,
}

impl UVK5RadioPlatform {
    pub fn new(syscon: &SYSCON, portcon: &PORTCON) -> Self {
        let pin_flashlight = Pin::new(Port::C, 3).into_push_pull_output(syscon, portcon);
        let pin_backlight = Pin::new(Port::B, 6).into_push_pull_output(syscon, portcon);
        let pin_audio_path = Pin::new(Port::C, 4).into_push_pull_output(syscon, portcon);
        let pin_bk1080_enable = Pin::new(Port::B, 15).into_push_pull_output(syscon, portcon);
        Self {
            pin_flashlight,
            pin_backlight,
            pin_audio_path,
            pin_bk1080_enable,
        }
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

    fn bk1080_enabled(&mut self, enabled: bool) {
        if enabled {
            let _ = embedded_hal::digital::OutputPin::set_low(&mut self.pin_bk1080_enable);
        } else {
            let _ = embedded_hal::digital::OutputPin::set_high(&mut self.pin_bk1080_enable);
        }
    }
}
