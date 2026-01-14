use std::sync::mpsc::Receiver;

use log::info;
use rquansheng::{keyboard::KeyEvent, radio_platform::RadioPlatform};

pub struct HostedPlatform {
    receiver: Receiver<KeyEvent>,
}

impl HostedPlatform {
    pub fn new(receiver: Receiver<KeyEvent>) -> Self {
        Self { receiver }
    }
}

impl RadioPlatform for HostedPlatform {
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

    fn bk1080_enabled(&mut self, enabled: bool) {
        info!("bk1080_enabled: {}", enabled);
    }

    fn poll_keyboard(&mut self) -> Option<KeyEvent> {
        info!("poll_keyboard");
        self.receiver.try_recv().ok()
    }
}
