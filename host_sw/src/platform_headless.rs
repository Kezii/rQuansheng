use std::sync::mpsc::Receiver;

use embedded_graphics::{
    Pixel,
    pixelcolor::BinaryColor,
    prelude::{Dimensions, DrawTarget, Point, Size},
    primitives::Rectangle,
};
use embedded_graphics_simulator::{SimulatorDisplay, Window};
use log::{debug, info};
use rquansheng::{keyboard::KeyEvent, radio_platform::RadioPlatform};

pub struct HostedHeadlessPlatform {
    pub receiver: Receiver<KeyEvent>,
}

impl HostedHeadlessPlatform {
    pub fn new(receiver: Receiver<KeyEvent>) -> Self {
        Self { receiver }
    }
}

impl RadioPlatform for HostedHeadlessPlatform {
    fn eeprom_read(&mut self, address: u16, data: &mut [u8]) -> Result<(), Self::EepromError> {
        const EEPROM_DUMP: &[u8] = include_bytes!("../../docs/eeprom_dump.bin");

        assert!(EEPROM_DUMP.len() == 0x2000);
        assert!((address + data.len() as u16) <= 0x2000);

        let addr = address as usize;
        let end = addr + data.len();

        data.copy_from_slice(&EEPROM_DUMP[addr..end]);

        Ok(())
    }

    type EepromError = std::io::Error;

    fn eeprom_write(&mut self, address: u16, data: &[u8; 8]) -> Result<(), Self::EepromError> {
        info!("eeprom_write: 0x{:x} {:?}", address, data);
        Ok(())
    }

    fn set_backlight(&mut self, on: bool) {
        info!("set_backlight: {}", on);
    }

    fn set_flashlight(&mut self, on: bool) {
        info!("set_flashlight: {}", on);
    }

    fn set_audio_path(&mut self, on: bool) {
        info!("set_audio_path: {}", on);
    }

    fn set_bk1080(&mut self, enabled: bool) {
        info!("bk1080_enabled: {}", enabled);
    }

    fn poll_keyboard(&mut self) -> Option<KeyEvent> {
        debug!("poll_keyboard");
        let key = self.receiver.try_recv().ok();

        if let Some(key) = key {
            info!("poll_keyboard: {:?}", key);
        }
        key
    }

    fn get_battery_adc(&mut self) -> u16 {
        debug!("get_battery_adc");
        2000
    }

    fn flush_display(&mut self) -> Result<(), Self::Error> {
        debug!("flush_display");

        Ok(())
    }
}

impl DrawTarget for HostedHeadlessPlatform {
    type Color = BinaryColor;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, _pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        Ok(())
    }
}

impl Dimensions for HostedHeadlessPlatform {
    fn bounding_box(&self) -> Rectangle {
        Rectangle::new(Point::new(0, 0), Size::new(128, 64))
    }
}
