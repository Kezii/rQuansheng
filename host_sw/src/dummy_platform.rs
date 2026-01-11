use log::info;
use rquansheng::radio_platform::RadioPlatform;

pub struct DummyPlatform;

impl RadioPlatform for DummyPlatform {
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

    fn backlight_on(&mut self) {
        info!("backlight_on");
    }

    fn backlight_off(&mut self) {
        info!("backlight_off");
    }

    fn flashlight_on(&mut self) {
        info!("flashlight_on");
    }

    fn flashlight_off(&mut self) {
        info!("flashlight_off");
    }

    fn audio_path_on(&mut self) {
        info!("audio_path_on");
    }

    fn audio_path_off(&mut self) {
        info!("audio_path_off");
    }

    fn bk1080_enabled(&mut self, enabled: bool) {
        info!("bk1080_enabled: {}", enabled);
    }
}
