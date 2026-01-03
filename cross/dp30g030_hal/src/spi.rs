//! SPI HAL for DP32G030, implemented in a clean embedded Rust style.
//!
//! This module is intentionally minimal and "standard":
//! - Uses `embedded-hal` 1.0 `spi::SpiBus` (bus-only; chip-select is user-managed).
//! - Uses the PAC (`dp32g030`) for register access (svd2rust-generated).
//! - Provides a small `Config` and strongly-typed SCK/MOSI/MISO pins with runtime validation.
//!
//! Notes:
//! - The reference C firmware for the UV-K5 uses SPI0 mode 3 (CPOL=1, CPHA=1) at FPCLK/16.
//! - The C firmware also waits on an **undocumented** SPI IF bit (0x20) to ensure TX is complete.
//!   We expose the same behavior via [`Spi::flush`] and internal `wait_tx_done`.

use core::marker::PhantomData;
use core::{error, fmt};

use dp32g030 as pac;

use embedded_hal::spi;

use crate::gpio::Port;

/// SPI clock divisor as encoded by the hardware `SPR` field.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum ClockDiv {
    Div4 = 0,
    Div8 = 1,
    Div16 = 2,
    Div32 = 3,
    Div64 = 4,
    Div128 = 5,
    Div256 = 6,
    Div512 = 7,
}

/// SPI configuration.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Config {
    /// SPI mode (CPOL/CPHA).
    pub mode: spi::Mode,
    /// Clock divider applied to FPCLK.
    pub clock_div: ClockDiv,
    /// LSB-first bit order when `true`.
    pub lsb_first: bool,
    /// RX DMA enable bit (just sets SPI CTRL bit; DMA setup is out of scope).
    pub rx_dma: bool,
    /// TX DMA enable bit (just sets SPI CTRL bit; DMA setup is out of scope).
    pub tx_dma: bool,
}

impl Config {
    /// Create a basic SPI configuration.
    #[inline]
    pub const fn new(mode: spi::Mode, clock_div: ClockDiv) -> Self {
        Self {
            mode,
            clock_div,
            lsb_first: false,
            rx_dma: false,
            tx_dma: false,
        }
    }

    /// Convenience for the UV-K5 display settings (SPI mode 3, FPCLK/16).
    #[inline]
    pub const fn uvk5_display_default() -> Self {
        Self::new(spi::MODE_3, ClockDiv::Div16)
    }
}

/// Errors returned by the SPI HAL.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Error {
    /// Timed out while waiting for TX completion (undocumented IF bit stayed asserted).
    TxTimeout,
    /// RX FIFO overflow (latched in IF register; if available).
    Overrun,
    /// Invalid configuration or pin mapping.
    BadConfig,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            Error::TxTimeout => "SPI TX timeout",
            Error::Overrun => "SPI overrun",
            Error::BadConfig => "SPI bad configuration",
        };
        f.write_str(s)
    }
}

impl error::Error for Error {}

impl spi::Error for Error {
    #[inline]
    fn kind(&self) -> spi::ErrorKind {
        match self {
            Error::TxTimeout | Error::BadConfig => spi::ErrorKind::Other,
            Error::Overrun => spi::ErrorKind::Overrun,
        }
    }
}

/// Error when assigning a pin to a SPI function.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct InvalidPin;

/// A SCK pin configured for a specific SPI instance.
pub struct SckPin<SPI> {
    port: Port,
    pin: u8,
    _spi: PhantomData<SPI>,
}

/// A MOSI pin configured for a specific SPI instance.
pub struct MosiPin<SPI> {
    port: Port,
    pin: u8,
    _spi: PhantomData<SPI>,
}

/// A MISO pin configured for a specific SPI instance.
pub struct MisoPin<SPI> {
    port: Port,
    pin: u8,
    _spi: PhantomData<SPI>,
}

impl<SPI> SckPin<SPI> {
    #[inline(always)]
    pub fn port(&self) -> Port {
        self.port
    }
    #[inline(always)]
    pub fn pin(&self) -> u8 {
        self.pin
    }
}

impl<SPI> MosiPin<SPI> {
    #[inline(always)]
    pub fn port(&self) -> Port {
        self.port
    }
    #[inline(always)]
    pub fn pin(&self) -> u8 {
        self.pin
    }
}

impl<SPI> MisoPin<SPI> {
    #[inline(always)]
    pub fn port(&self) -> Port {
        self.port
    }
    #[inline(always)]
    pub fn pin(&self) -> u8 {
        self.pin
    }
}

/// Marker type for "no MISO pin" (write-only SPI).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct NoMiso;

/// A SPI peripheral together with its configured pins.
pub struct Spi<SPI, SCK, MOSI, MISO> {
    spi: SPI,
    _sck: SCK,
    _mosi: MOSI,
    _miso: MISO,
}

mod sealed {
    pub trait Sealed {}
}

/// Common interface needed from a SPI instance.
pub trait Instance: sealed::Sealed {
    /// Numeric SPI index (0/1) used for pin mapping.
    const NUM: u8;
    fn enable_clock(syscon: &pac::SYSCON);
    fn regs() -> &'static Self::Regs;
    type Regs;
}

impl sealed::Sealed for pac::SPI0 {}
impl sealed::Sealed for pac::SPI1 {}

impl Instance for pac::SPI0 {
    type Regs = pac::spi0::RegisterBlock;
    const NUM: u8 = 0;

    #[inline(always)]
    fn enable_clock(syscon: &pac::SYSCON) {
        syscon
            .dev_clk_gate()
            .modify(|_, w| w.spi0_clk_gate().set_bit());
    }

    #[inline(always)]
    fn regs() -> &'static Self::Regs {
        unsafe { &*pac::SPI0::ptr() }
    }
}

impl Instance for pac::SPI1 {
    type Regs = pac::spi1::RegisterBlock;
    const NUM: u8 = 1;

    #[inline(always)]
    fn enable_clock(syscon: &pac::SYSCON) {
        syscon
            .dev_clk_gate()
            .modify(|_, w| w.spi1_clk_gate().set_bit());
    }

    #[inline(always)]
    fn regs() -> &'static Self::Regs {
        unsafe { &*pac::SPI1::ptr() }
    }
}

// --- Pin mapping helpers (PORTCON alternate functions) ----------------------

#[inline(always)]
fn enable_gpio_clock(syscon: &pac::SYSCON, port: Port) {
    syscon.dev_clk_gate().modify(|_, w| match port {
        Port::A => w.gpioa_clk_gate().set_bit(),
        Port::B => w.gpiob_clk_gate().set_bit(),
        Port::C => w.gpioc_clk_gate().set_bit(),
    });
}

#[inline(always)]
fn set_input_enable(portcon: &pac::PORTCON, port: Port, pin: u8, enable: bool) {
    let bit = 1u32 << (pin as u32);
    let set = |v: u32| if enable { v | bit } else { v & !bit };

    match port {
        Port::A => portcon
            .porta_ie()
            .modify(|r, w| unsafe { w.bits(set(r.bits())) }),
        Port::B => portcon
            .portb_ie()
            .modify(|r, w| unsafe { w.bits(set(r.bits())) }),
        Port::C => portcon
            .portc_ie()
            .modify(|r, w| unsafe { w.bits(set(r.bits())) }),
    }
}

#[inline(always)]
fn set_direction(port: Port, pin: u8, output: bool) {
    let bit = 1u32 << (pin as u32);
    let set = |v: u32| if output { v | bit } else { v & !bit };

    cortex_m::interrupt::free(|_| match port {
        Port::A => unsafe {
            let gpioa = &*pac::GPIOA::ptr();
            gpioa.gpioa_dir().modify(|r, w| w.bits(set(r.bits())));
        },
        Port::B => unsafe {
            let gpiob = &*pac::GPIOB::ptr();
            gpiob.gpiob_dir().modify(|r, w| w.bits(set(r.bits())));
        },
        Port::C => unsafe {
            let gpioc = &*pac::GPIOC::ptr();
            gpioc.gpioc_dir().modify(|r, w| w.bits(set(r.bits())));
        },
    });
}

#[inline(always)]
fn set_pin_function(portcon: &pac::PORTCON, port: Port, pin: u8, function: u8) {
    let shift = ((pin % 8) as u32) * 4;
    let mask = 0xFu32 << shift;
    let f = (function as u32) & 0xF;

    match port {
        Port::A => {
            if pin < 8 {
                portcon
                    .porta_sel0()
                    .modify(|r, w| unsafe { w.bits((r.bits() & !mask) | (f << shift)) });
            } else {
                portcon
                    .porta_sel1()
                    .modify(|r, w| unsafe { w.bits((r.bits() & !mask) | (f << shift)) });
            }
        }
        Port::B => {
            if pin < 8 {
                portcon
                    .portb_sel0()
                    .modify(|r, w| unsafe { w.bits((r.bits() & !mask) | (f << shift)) });
            } else {
                portcon
                    .portb_sel1()
                    .modify(|r, w| unsafe { w.bits((r.bits() & !mask) | (f << shift)) });
            }
        }
        Port::C => {
            // On DP32G030, PORTC function select uses only SEL0 for pins 0..7.
            portcon
                .portc_sel0()
                .modify(|r, w| unsafe { w.bits((r.bits() & !mask) | (f << shift)) });
        }
    }
}

#[inline(always)]
fn spi0_sck_fn(port: Port, pin: u8) -> Option<u8> {
    // Derived from `uv-k5-firmware-custom/hardware/dp32g030/portcon.def`.
    match (port, pin) {
        (Port::A, 10) => Some(1),
        (Port::B, 8) => Some(1),
        _ => None,
    }
}

#[inline(always)]
fn spi0_mosi_fn(port: Port, pin: u8) -> Option<u8> {
    match (port, pin) {
        (Port::A, 12) => Some(1),
        (Port::B, 10) => Some(1),
        _ => None,
    }
}

#[inline(always)]
fn spi0_miso_fn(port: Port, pin: u8) -> Option<u8> {
    match (port, pin) {
        (Port::A, 11) => Some(1),
        (Port::B, 9) => Some(1),
        _ => None,
    }
}

#[inline(always)]
fn spi1_sck_fn(port: Port, pin: u8) -> Option<u8> {
    match (port, pin) {
        (Port::B, 3) => Some(1),
        (Port::C, 0) => Some(1),
        _ => None,
    }
}

#[inline(always)]
fn spi1_mosi_fn(port: Port, pin: u8) -> Option<u8> {
    match (port, pin) {
        (Port::B, 5) => Some(1),
        (Port::C, 2) => Some(1),
        _ => None,
    }
}

#[inline(always)]
fn spi1_miso_fn(port: Port, pin: u8) -> Option<u8> {
    match (port, pin) {
        (Port::B, 4) => Some(1),
        (Port::C, 1) => Some(1),
        _ => None,
    }
}

// --- Optional MISO handling --------------------------------------------------

/// Internal helper: configure an optional MISO pin (or nothing).
pub trait MisoConfig<SPI> {
    fn configure(&self, syscon: &pac::SYSCON, portcon: &pac::PORTCON) -> Result<(), Error>;
}

impl<SPI> MisoConfig<SPI> for NoMiso {
    #[inline(always)]
    fn configure(&self, _syscon: &pac::SYSCON, _portcon: &pac::PORTCON) -> Result<(), Error> {
        Ok(())
    }
}

impl MisoConfig<pac::SPI0> for MisoPin<pac::SPI0> {
    #[inline]
    fn configure(&self, syscon: &pac::SYSCON, portcon: &pac::PORTCON) -> Result<(), Error> {
        enable_gpio_clock(syscon, self.port);
        let fn_ = spi0_miso_fn(self.port, self.pin).ok_or(Error::BadConfig)?;
        set_pin_function(portcon, self.port, self.pin, fn_);
        set_direction(self.port, self.pin, false);
        set_input_enable(portcon, self.port, self.pin, true);
        Ok(())
    }
}

impl MisoConfig<pac::SPI1> for MisoPin<pac::SPI1> {
    #[inline]
    fn configure(&self, syscon: &pac::SYSCON, portcon: &pac::PORTCON) -> Result<(), Error> {
        enable_gpio_clock(syscon, self.port);
        let fn_ = spi1_miso_fn(self.port, self.pin).ok_or(Error::BadConfig)?;
        set_pin_function(portcon, self.port, self.pin, fn_);
        set_direction(self.port, self.pin, false);
        set_input_enable(portcon, self.port, self.pin, true);
        Ok(())
    }
}

// --- Pin constructors --------------------------------------------------------

impl SckPin<pac::SPI0> {
    #[inline]
    pub fn new(
        pin: crate::gpio::Pin<crate::gpio::Disabled>,
    ) -> Result<Self, InvalidPin> {
        let port = pin.port();
        let p = pin.pin();
        spi0_sck_fn(port, p)
            .map(|_| Self {
                port,
                pin: p,
                _spi: PhantomData,
            })
            .ok_or(InvalidPin)
    }
}

impl MosiPin<pac::SPI0> {
    #[inline]
    pub fn new(
        pin: crate::gpio::Pin<crate::gpio::Disabled>,
    ) -> Result<Self, InvalidPin> {
        let port = pin.port();
        let p = pin.pin();
        spi0_mosi_fn(port, p)
            .map(|_| Self {
                port,
                pin: p,
                _spi: PhantomData,
            })
            .ok_or(InvalidPin)
    }
}

impl MisoPin<pac::SPI0> {
    #[inline]
    pub fn new(
        pin: crate::gpio::Pin<crate::gpio::Disabled>,
    ) -> Result<Self, InvalidPin> {
        let port = pin.port();
        let p = pin.pin();
        spi0_miso_fn(port, p)
            .map(|_| Self {
                port,
                pin: p,
                _spi: PhantomData,
            })
            .ok_or(InvalidPin)
    }
}

impl SckPin<pac::SPI1> {
    #[inline]
    pub fn new(
        pin: crate::gpio::Pin<crate::gpio::Disabled>,
    ) -> Result<Self, InvalidPin> {
        let port = pin.port();
        let p = pin.pin();
        spi1_sck_fn(port, p)
            .map(|_| Self {
                port,
                pin: p,
                _spi: PhantomData,
            })
            .ok_or(InvalidPin)
    }
}

impl MosiPin<pac::SPI1> {
    #[inline]
    pub fn new(
        pin: crate::gpio::Pin<crate::gpio::Disabled>,
    ) -> Result<Self, InvalidPin> {
        let port = pin.port();
        let p = pin.pin();
        spi1_mosi_fn(port, p)
            .map(|_| Self {
                port,
                pin: p,
                _spi: PhantomData,
            })
            .ok_or(InvalidPin)
    }
}

impl MisoPin<pac::SPI1> {
    #[inline]
    pub fn new(
        pin: crate::gpio::Pin<crate::gpio::Disabled>,
    ) -> Result<Self, InvalidPin> {
        let port = pin.port();
        let p = pin.pin();
        spi1_miso_fn(port, p)
            .map(|_| Self {
                port,
                pin: p,
                _spi: PhantomData,
            })
            .ok_or(InvalidPin)
    }
}

// --- Spi constructors --------------------------------------------------------

impl<SPI: Instance, SCK, MOSI, MISO> Spi<SPI, SCK, MOSI, MISO> {
    /// Release the owned SPI peripheral and pins.
    #[inline]
    pub fn free(self) -> (SPI, SCK, MOSI, MISO) {
        (self.spi, self._sck, self._mosi, self._miso)
    }
}

macro_rules! impl_spi {
    ($SPI:ty, $cr:ident, $wdr:ident, $rdr:ident, $ie:ident, $if_:ident, $fifost:ident) => {
        impl<MISO> Spi<$SPI, SckPin<$SPI>, MosiPin<$SPI>, MISO>
        where
            $SPI: Instance,
            MISO: MisoConfig<$SPI>,
        {
            /// Create and configure a SPI instance (master).
            ///
            /// This sets up:
            /// - SPI clock gate in `SYSCON.DEV_CLK_GATE`
            /// - Pin mux in `PORTCON.PORTx_SEL*`
            /// - GPIO direction (SCK/MOSI outputs, optional MISO input + input-enable)
            /// - SPI CTRL (mode, clock divider, bit order, optional DMA bits)
            pub fn new(
                spi: $SPI,
                syscon: &pac::SYSCON,
                portcon: &pac::PORTCON,
                sck: SckPin<$SPI>,
                mosi: MosiPin<$SPI>,
                miso: MISO,
                config: Config,
            ) -> Result<Spi<$SPI, SckPin<$SPI>, MosiPin<$SPI>, MISO>, Error> {
                <$SPI as Instance>::enable_clock(syscon);

                // GPIO clocks needed for direction + input buffer settings.
                enable_gpio_clock(syscon, sck.port);
                enable_gpio_clock(syscon, mosi.port);

                // Configure mux for SCK/MOSI.
                let sck_fn = match <$SPI as Instance>::NUM {
                    0 => spi0_sck_fn(sck.port, sck.pin),
                    _ => spi1_sck_fn(sck.port, sck.pin),
                }
                .ok_or(Error::BadConfig)?;
                let mosi_fn = match <$SPI as Instance>::NUM {
                    0 => spi0_mosi_fn(mosi.port, mosi.pin),
                    _ => spi1_mosi_fn(mosi.port, mosi.pin),
                }
                .ok_or(Error::BadConfig)?;
                set_pin_function(portcon, sck.port, sck.pin, sck_fn);
                set_pin_function(portcon, mosi.port, mosi.pin, mosi_fn);

                // Direction: SCK/MOSI are outputs.
                set_direction(sck.port, sck.pin, true);
                set_direction(mosi.port, mosi.pin, true);

                // Optional MISO pin.
                miso.configure(syscon, portcon)?;

                let regs = <$SPI as Instance>::regs();

                // Disable SPI before reconfiguring: clear SPE bit (bit 3).
                regs.$cr()
                    .modify(|r, w| unsafe { w.bits(r.bits() & !(1u32 << 3)) });

                // Clear FIFOs once (RF_CLR bit 15, TF_CLR bit 16).
                regs.$cr()
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1u32 << 15) | (1u32 << 16)) });

                // Build CTRL bits:
                // - SPR [2:0]
                // - SPE [3] left disabled until the end
                // - CPHA [4], CPOL [5]
                // - MSTR [6] forced 1 (master)
                // - LSB [7]
                // - RXDMAEN [13], TXDMAEN [14]
                let (cpol, cpha) = match config.mode {
                    spi::MODE_0 => (false, false),
                    spi::MODE_1 => (false, true),
                    spi::MODE_2 => (true, false),
                    spi::MODE_3 => (true, true),
                };

                let mut cr: u32 = 0;
                cr |= (config.clock_div as u32) & 0x7;
                if cpha {
                    cr |= 1u32 << 4;
                }
                if cpol {
                    cr |= 1u32 << 5;
                }
                cr |= 1u32 << 6; // MSTR=1
                if config.lsb_first {
                    cr |= 1u32 << 7;
                }
                if config.rx_dma {
                    cr |= 1u32 << 13;
                }
                if config.tx_dma {
                    cr |= 1u32 << 14;
                }

                // Write CTRL (keep SPE=0 for now).
                regs.$cr().write(|w| unsafe { w.bits(cr) });

                // Disable interrupts by default.
                regs.$ie().write(|w| unsafe { w.bits(0) });

                // Finally enable SPI (SPE bit 3).
                regs.$cr()
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1u32 << 3)) });

                Ok(Spi {
                    spi,
                    _sck: sck,
                    _mosi: mosi,
                    _miso: miso,
                })
            }
        }

        impl<MISO> spi::ErrorType for Spi<$SPI, SckPin<$SPI>, MosiPin<$SPI>, MISO> {
            type Error = Error;
        }

        impl<MISO> Spi<$SPI, SckPin<$SPI>, MosiPin<$SPI>, MISO> {
            #[inline(always)]
            fn regs() -> &'static <$SPI as Instance>::Regs {
                <$SPI as Instance>::regs()
            }

            // SPI_CR MSR_SSN bit (bit 12) as defined by the DP32G030 reference headers.
            // In the stock firmware this is toggled via `SPI_ToggleMasterMode()`.
            const CR_MSR_SSN_BIT: u32 = 1u32 << 12;

            #[inline(always)]
            fn tx_fifo_full() -> bool {
                // FIFOST.TFF bit 4
                (Self::regs().$fifost().read().bits() & (1u32 << 4)) != 0
            }

            #[inline(always)]
            fn rx_fifo_empty() -> bool {
                // FIFOST.RFE bit 0
                (Self::regs().$fifost().read().bits() & (1u32 << 0)) != 0
            }

            #[inline(always)]
            fn wait_tx_done(&self) -> Result<(), Error> {
                // Reference firmware waits for an undocumented IF bit (0x20) to clear.
                // Keep a bounded loop to avoid hard lockups.
                let mut timeout: u32 = 0;
                loop {
                    let ifr = Self::regs().$if_().read().bits();
                    if (ifr & 0x20) == 0 {
                        return Ok(());
                    }
                    timeout = timeout.wrapping_add(1);
                    if timeout > 100_000 {
                        return Err(Error::TxTimeout);
                    }
                }
            }

            /// Toggle the "MSR_SSN" bit in `SPI_CR` (bit 12).
            ///
            /// This mirrors the reference C helper `SPI_ToggleMasterMode()` which does:
            /// `CR = (CR & ~MSR_SSN_MASK) | (ENABLE/DISABLE)`.
            ///
            /// Note: this does **not** touch the `MSTR` bit (bit 6). In this HAL we always
            /// configure `MSTR=1` in [`Spi::new`] and chip-select is user-managed.
            #[inline]
            pub fn set_master_mode(&mut self, is_master: bool) {
                Self::regs().$cr().modify(|r, w| unsafe {
                    let mut bits = r.bits() & !Self::CR_MSR_SSN_BIT;
                    if is_master {
                        bits |= Self::CR_MSR_SSN_BIT;
                    } else {
                        bits &= !Self::CR_MSR_SSN_BIT;
                    }
                    w.bits(bits)
                });
            }

            /// Returns `true` if the `SPI_CR.MSR_SSN` bit (bit 12) is set.
            #[inline]
            pub fn master_mode(&self) -> bool {
                (Self::regs().$cr().read().bits() & Self::CR_MSR_SSN_BIT) != 0
            }
        }

        impl<MISO> spi::SpiBus<u8> for Spi<$SPI, SckPin<$SPI>, MosiPin<$SPI>, MISO> {
            fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
                // Full-duplex read: write dummy bytes and read back.
                for w in words.iter_mut() {
                    while Self::tx_fifo_full() {}
                    Self::regs().$wdr().write(|x| unsafe { x.bits(0u32) });
                    while Self::rx_fifo_empty() {}
                    *w = (Self::regs().$rdr().read().bits() & 0xFF) as u8;
                }
                Ok(())
            }

            fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
                for &b in words {
                    while Self::tx_fifo_full() {}
                    Self::regs().$wdr().write(|x| unsafe { x.bits(b as u32) });
                }
                Ok(())
            }

            fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
                let n = core::cmp::min(read.len(), write.len());
                for i in 0..n {
                    while Self::tx_fifo_full() {}
                    Self::regs()
                        .$wdr()
                        .write(|x| unsafe { x.bits(write[i] as u32) });
                    while Self::rx_fifo_empty() {}
                    read[i] = (Self::regs().$rdr().read().bits() & 0xFF) as u8;
                }
                Ok(())
            }

            fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
                for b in words.iter_mut() {
                    while Self::tx_fifo_full() {}
                    Self::regs().$wdr().write(|x| unsafe { x.bits(*b as u32) });
                    while Self::rx_fifo_empty() {}
                    *b = (Self::regs().$rdr().read().bits() & 0xFF) as u8;
                }
                Ok(())
            }

            fn flush(&mut self) -> Result<(), Self::Error> {
                self.wait_tx_done()
            }
        }
    };
}

impl_spi!(
    pac::SPI0,
    spi0_cr,
    spi0_wdr,
    spi0_rdr,
    spi0_ie,
    spi0_if,
    spi0_fifost
);
impl_spi!(
    pac::SPI1,
    spi1_cr,
    spi1_wdr,
    spi1_rdr,
    spi1_ie,
    spi1_if,
    spi1_fifost
);

// Convenience aliases.
pub type Spi0<MISO = NoMiso> = Spi<pac::SPI0, SckPin<pac::SPI0>, MosiPin<pac::SPI0>, MISO>;
pub type Spi1<MISO = NoMiso> = Spi<pac::SPI1, SckPin<pac::SPI1>, MosiPin<pac::SPI1>, MISO>;

/// DP32G030-specific control for the `SPI_CR.MSR_SSN` bit (bit 12).
///
/// The stock UV-K5 firmware toggles this around ST7565 transactions via
/// `SPI_ToggleMasterMode(&SPI0->CR, ...)`.
pub trait MsrSsnControl {
    /// Set/clear `SPI_CR.MSR_SSN` (bit 12).
    fn set_msr_ssn(&mut self, enable: bool);
    /// Read `SPI_CR.MSR_SSN` (bit 12).
    fn msr_ssn(&self) -> bool;
}

impl<MISO> MsrSsnControl for Spi0<MISO> {
    #[inline]
    fn set_msr_ssn(&mut self, enable: bool) {
        self.set_master_mode(enable)
    }
    #[inline]
    fn msr_ssn(&self) -> bool {
        self.master_mode()
    }
}

impl<MISO> MsrSsnControl for Spi1<MISO> {
    #[inline]
    fn set_msr_ssn(&mut self, enable: bool) {
        self.set_master_mode(enable)
    }
    #[inline]
    fn msr_ssn(&self) -> bool {
        self.master_mode()
    }
}
