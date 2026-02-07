use log::{debug, info};
use rquansheng::bk_common::{BkCommonBus, DeviceRegister};

pub struct DummyRadioBus;

impl BkCommonBus for DummyRadioBus {
    type Error = std::io::Error;

    fn write_reg_raw(&mut self, reg: u8, value: u16) -> Result<(), Self::Error> {
        debug!("write_reg_raw: 0x{:x} 0x{:x}", reg, value);
        Ok(())
    }

    fn read_reg_raw(&mut self, reg: u8) -> Result<u16, Self::Error> {
        debug!("read_reg_raw: 0x{:x}", reg);
        Ok(0)
    }

    fn write_reg<R: DeviceRegister>(&mut self, reg: R) -> Result<(), Self::Error> {
        debug!(
            "write_reg: 0x{:x} 0x{:x} -- {:?}",
            R::get_address(),
            reg.serialize(),
            reg
        );
        Ok(())
    }

    fn read_reg<R: DeviceRegister>(&mut self) -> Result<R, Self::Error> {
        let ret = R::default();
        debug!("read_reg: 0x{:x} -- {:?}", R::get_address(), ret);
        Ok(ret)
    }
}

pub struct DummyRadioBus1080;

impl BkCommonBus for DummyRadioBus1080 {
    type Error = std::io::Error;

    fn write_reg_raw(&mut self, reg: u8, value: u16) -> Result<(), Self::Error> {
        debug!("write_reg_raw: 0x{:x} 0x{:x}", reg, value);
        Ok(())
    }

    fn read_reg_raw(&mut self, reg: u8) -> Result<u16, Self::Error> {
        debug!("read_reg_raw: 0x{:x}", reg);
        Ok(0)
    }
}
