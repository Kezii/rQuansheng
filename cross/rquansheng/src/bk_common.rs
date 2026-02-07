pub trait DeviceRegister: Copy + From<u16> + Into<u16> + core::fmt::Debug + Default {
    const ADDRESS: u8;

    #[inline]
    fn get_address() -> u8
    where
        Self: Sized,
    {
        Self::ADDRESS
    }

    #[inline]
    fn serialize(self) -> u16 {
        self.into()
    }

    #[inline]
    fn deserialize(data: u16) -> Self {
        Self::from(data)
    }
}

/// A minimal bus trait for BK4819 register access.
pub trait BkCommonBus {
    type Error;

    fn write_reg_raw(&mut self, reg: u8, value: u16) -> Result<(), Self::Error>;
    fn read_reg_raw(&mut self, reg: u8) -> Result<u16, Self::Error>;

    fn write_reg<R: DeviceRegister>(&mut self, reg: R) -> Result<(), Self::Error> {
        // calls to the log crate should not end up in the firwmare
        log::debug!(
            "write_reg: 0x{:x} 0x{:x} -- {:?}",
            R::get_address(),
            reg.serialize(),
            reg
        );
        self.write_reg_raw(R::ADDRESS, reg.serialize())
    }
    fn read_reg<R: DeviceRegister>(&mut self) -> Result<R, Self::Error> {
        let value = self.read_reg_raw(R::ADDRESS)?;

        log::debug!("read_reg: 0x{:x} -- {:?}", R::get_address(), value);

        Ok(R::deserialize(value))
    }
}
