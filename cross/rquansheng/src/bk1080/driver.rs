//! BK1080 high-level driver.
//!
//! Ported from `uv-k5-firmware-custom/driver/bk1080.c`.

use dp30g030_hal::gpio::{Output, Pin, Port};
use dp32g030 as pac;
use embedded_hal::delay::DelayNs;

use crate::bk1080::regs;
use crate::i2c_bitbang::{BitBangI2c, Error as I2cError};
use crate::DeviceRegister;

/// First command/address byte used by the BK1080 protocol on UV-K5.
///
/// The reference firmware always writes `0x80` first.
const BK1080_ADDR_CMD: u8 = 0x80;

const BK1080_RW_WRITE: u8 = 0;
const BK1080_RW_READ: u8 = 1;

/// Register init table from the reference firmware (`BK1080_RegisterTable`).
///
/// Written to registers 0x00..=0x21 on first init.
pub const INIT_REG_TABLE: [u16; 34] = [
    0x0008, 0x1080, 0x0201, 0x0000, 0x40C0, 0x0A1F, 0x002E, 0x02FF, 0x5B11, 0x0000, 0x411E, 0x0000,
    0xCE00, 0x0000, 0x0000, 0x1000, 0x3197, 0x0000, 0x13FF, 0x9852, 0x0000, 0x0000, 0x0008, 0x0000,
    0x51E1, 0xA8BC, 0x2645, 0x00E4, 0x1CD8, 0x3A50, 0xEAE0, 0x3000, 0x0200, 0x0000,
];

/// Power configuration values used by the reference firmware.
pub mod power_config {
    /// Normal power-on.
    pub const ON: u16 = 0x0201;
    /// Power-on with mute enabled (note: differs by 0x4000).
    pub const ON_MUTE: u16 = 0x4201;
    /// Power-down (used before disabling the chip via GPIO).
    pub const OFF: u16 = 0x0241;
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Error {
    NoAck,
}

impl From<I2cError> for Error {
    fn from(_value: I2cError) -> Self {
        Error::NoAck
    }
}

/// Low-level BK1080 bus access trait.
pub trait Bk1080Bus {
    type Error;

    fn write_reg_raw(&mut self, reg: u8, value: u16) -> Result<(), Self::Error>;
    fn read_reg_raw(&mut self, reg: u8) -> Result<u16, Self::Error>;

    #[inline]
    fn write_reg<R: DeviceRegister>(&mut self, value: R) -> Result<(), Self::Error> {
        self.write_reg_raw(R::ADDRESS, value.serialize())
    }

    #[inline]
    fn read_reg<R: DeviceRegister>(&mut self) -> Result<R, Self::Error> {
        let v = self.read_reg_raw(R::ADDRESS)?;
        Ok(R::deserialize(v))
    }
}

/// Bit-banged BK1080 bus implementation (UV-K5 shared PA10/PA11 pins).
pub struct Bk1080BitBangBus<D: DelayNs> {
    i2c: BitBangI2c,
    delay: D,
}

impl<D: DelayNs> Bk1080BitBangBus<D> {
    #[inline]
    pub const fn new(i2c: BitBangI2c, delay: D) -> Self {
        Self { i2c, delay }
    }

    /// Convenience constructor for UV-K5: PA10=SCL, PA11=SDA and a caller-provided delay.
    #[inline]
    pub const fn uvk5_shared(delay: D) -> Self {
        Self::new(BitBangI2c::uvk5_shared_pins(), delay)
    }

    /// Configure the I2C pins.
    #[inline]
    pub fn init_pins(&self, syscon: &pac::SYSCON, portcon: &pac::PORTCON) {
        self.i2c.init(syscon, portcon);
    }
}

impl<D: DelayNs> Bk1080Bus for Bk1080BitBangBus<D> {
    type Error = Error;

    fn write_reg_raw(&mut self, reg: u8, value: u16) -> Result<(), Self::Error> {
        let peripherals = unsafe { pac::Peripherals::steal() };
        let syscon = &peripherals.SYSCON;
        let portcon = &peripherals.PORTCON;
        // Important: PA10/PA11 are shared with keypad scanning; re-init pins like the C firmware.
        self.i2c.init(syscon, portcon);
        self.i2c.start(&mut self.delay);
        self.i2c
            .write_byte(&mut self.delay, portcon, BK1080_ADDR_CMD)
            .map_err(|I2cError::NoAck| Error::NoAck)?;
        self.i2c
            .write_byte(&mut self.delay, portcon, (reg << 1) | BK1080_RW_WRITE)
            .map_err(|I2cError::NoAck| Error::NoAck)?;

        // Reference firmware swaps the u16 then writes it as bytes; on little-endian MCUs that
        // results in a big-endian on-wire order. We directly send big-endian here.
        let be = value.to_be_bytes();
        self.i2c
            .write_buffer(&mut self.delay, portcon, &be)
            .map_err(|I2cError::NoAck| Error::NoAck)?;
        self.i2c.stop(&mut self.delay);
        Ok(())
    }

    fn read_reg_raw(&mut self, reg: u8) -> Result<u16, Self::Error> {
        let peripherals = unsafe { pac::Peripherals::steal() };
        let syscon = &peripherals.SYSCON;
        let portcon = &peripherals.PORTCON;
        let mut buf = [0u8; 2];

        // Important: PA10/PA11 are shared with keypad scanning; re-init pins like the C firmware.
        self.i2c.init(syscon, portcon);
        self.i2c.start(&mut self.delay);
        self.i2c
            .write_byte(&mut self.delay, portcon, BK1080_ADDR_CMD)
            .map_err(|I2cError::NoAck| Error::NoAck)?;
        self.i2c
            .write_byte(&mut self.delay, portcon, (reg << 1) | BK1080_RW_READ)
            .map_err(|I2cError::NoAck| Error::NoAck)?;
        self.i2c.read_buffer(&mut self.delay, portcon, &mut buf);
        self.i2c.stop(&mut self.delay);

        Ok(u16::from_be_bytes(buf))
    }
}

/// High-level BK1080 driver.
///
/// `enable_pin` is **active-low** on UV-K5 (matches C firmware: `GPIOB_PIN_BK1080`).
pub struct Bk1080<BUS> {
    bus: BUS,
    is_inited: bool,
    pub base_frequency: u16,
    pub frequency_deviation: u16,
    pub band: regs::FmBand,
}

impl<BUS> Bk1080<BUS>
where
    BUS: Bk1080Bus,
{
    pub fn new(bus: BUS) -> Self {
        Self {
            bus,
            is_inited: false,
            base_frequency: 0,
            frequency_deviation: 0,
            band: regs::FmBand::UsEu875To1080,
        }
    }

    #[inline]
    pub fn bus_mut(&mut self) -> &mut BUS {
        &mut self.bus
    }

    /// Initialize or power down the BK1080, mirroring `BK1080_Init(freq, band)`.
    ///
    /// - `frequency_100khz`: `Some(freq)` powers on + configures; `None` powers down.
    pub fn init<D: DelayNs>(
        &mut self,
        delay: &mut D,
        frequency_100khz: Option<u16>,
    ) -> Result<(), BUS::Error> {
        if let Some(freq) = frequency_100khz {
            //self.set_enabled(true);

            if !self.is_inited {
                for (i, &v) in INIT_REG_TABLE.iter().enumerate() {
                    self.bus.write_reg_raw(i as u8, v)?;
                }

                delay.delay_ms(250);

                self.bus.write_reg_raw(regs::Reg19::ADDRESS, 0xA83C)?;
                self.bus.write_reg_raw(regs::Reg19::ADDRESS, 0xA8BC)?;

                delay.delay_ms(60);
                self.is_inited = true;
            } else {
                self.bus
                    .write_reg_raw(regs::Reg02::ADDRESS, power_config::ON)?;
            }

            self.bus.write_reg_raw(regs::Reg05::ADDRESS, 0x0A1F)?;
            self.set_frequency(delay, freq)?;
        } else {
            self.bus
                .write_reg_raw(regs::Reg02::ADDRESS, power_config::OFF)?;
            //self.set_enabled(false);
        }

        Ok(())
    }

    /// Mute/unmute audio, mirroring `BK1080_Mute(Mute)`.
    pub fn mute(&mut self, mute: bool) -> Result<(), BUS::Error> {
        self.bus.write_reg_raw(
            regs::Reg02::ADDRESS,
            if mute {
                power_config::ON_MUTE
            } else {
                power_config::ON
            },
        )
    }

    pub fn __internal_write_register_raw(&mut self, reg: u8, value: u16) -> Result<(), BUS::Error> {
        self.bus.write_reg_raw(reg, value)
    }

    pub fn __internal_read_register_raw(&mut self, reg: u8) -> Result<u16, BUS::Error> {
        self.bus.read_reg_raw(reg)
    }

    /// Set the receive frequency (100 kHz units) and band.
    ///
    /// Mirrors `BK1080_SetFrequency(frequency, band)`.
    pub fn set_frequency<D: DelayNs>(
        &mut self,
        delay: &mut D,
        frequency_100khz: u16,
    ) -> Result<(), BUS::Error> {
        let channel = frequency_100khz.wrapping_sub(self.band.lo_limit_100khz());

        // Read-modify-write band bits in REG_05.
        let mut reg05: regs::Reg05 = self.bus.read_reg()?;
        reg05.set_band(self.band);
        self.bus.write_reg(reg05)?;

        // Program channel, then set TUNE bit.
        let reg03 = regs::Reg03::new().with_channel(channel).with_tune(false);
        self.bus.write_reg(reg03)?;
        delay.delay_ms(10);
        let reg03 = reg03.with_tune(true);
        self.bus.write_reg(reg03)?;

        Ok(())
    }

    /// Update `base_frequency` and `frequency_deviation`, mirroring `BK1080_GetFrequencyDeviation`.
    pub fn update_frequency_deviation(
        &mut self,
        base_frequency_100khz: u16,
    ) -> Result<(), BUS::Error> {
        self.base_frequency = base_frequency_100khz;
        let reg07: regs::Reg07 = self.bus.read_reg()?;
        self.frequency_deviation = reg07.freqd();
        Ok(())
    }

    /// Read the RSSI and AFCRL status.
    pub fn read_signal_status(&mut self) -> Result<regs::Reg0A, BUS::Error> {
        self.bus.read_reg()
    }
}

/// Convenience helper to create the BK1080 enable pin on UV-K5 (PB15, active-low).
#[inline]
pub fn uvk5_bk1080_enable_pin(syscon: &pac::SYSCON, portcon: &pac::PORTCON) -> Pin<Output> {
    Pin::new(Port::B, 15).into_push_pull_output(syscon, portcon)
}
