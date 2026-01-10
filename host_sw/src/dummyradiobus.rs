use log::info;
use rquansheng::bk4819_bitbang::Bk4819Bus;

pub struct DummyRadioBus;

impl Bk4819Bus for DummyRadioBus {
    type Error = std::io::Error;

    fn write_reg_raw(&mut self, reg: u8, value: u16) -> Result<(), Self::Error> {
        info!("write_reg_raw: 0x{:x} 0x{:x}", reg, value);
        Ok(())
    }

    fn read_reg_raw(&mut self, reg: u8) -> Result<u16, Self::Error> {
        info!("read_reg_raw: 0x{:x}", reg);
        Ok(0)
    }

    fn write_reg<R: rquansheng::bk4819_n::Bk4819Register>(
        &mut self,
        reg: R,
    ) -> Result<(), Self::Error> {
        info!(
            "write_reg: 0x{:x} 0x{:x} -- {:?}",
            R::get_address(),
            reg.serialize(),
            reg
        );
        Ok(())
    }

    fn read_reg<R: rquansheng::bk4819_n::Bk4819Register>(&mut self) -> Result<R, Self::Error> {
        let ret = R::default();
        info!("read_reg: 0x{:x} -- {:?}", R::get_address(), ret);
        Ok(ret)
    }
}
