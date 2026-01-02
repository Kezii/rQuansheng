//! UART HAL for DP32G030, implemented in an idiomatic embedded Rust style.
//!
//! This module is intentionally minimal and "standard":
//! - Uses `embedded-hal-nb` serial traits (`Read`/`Write`) and `nb` for non-blocking I/O.
//! - Uses the PAC (`dp32g030`) for register access (svd2rust-generated).
//! - Provides a small `Config` and strongly-typed TX/RX pins with runtime validation.
//!
//! Notes on baud rate:
//! The upstream C firmware configures `UARTx_BAUD` with `baud_div = uart_clk_hz / baud`.
//! We expose the same computation via [`Config::baud_divisor`].

use core::marker::PhantomData;
use core::{error, fmt};

use dp32g030 as pac;

use embedded_hal_nb::serial;
use embedded_io as eio;

use crate::dp30g030_hal::gpio::Port;

/// UART configuration.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Config {
    /// UART input clock in Hz (e.g. calibrated RCHF around 48MHz).
    pub uart_clk_hz: u32,
    /// Requested baud rate (bps).
    pub baud: u32,
    /// Parity configuration.
    pub parity: Parity,
    /// Data bits (8/9).
    pub data_bits: DataBits,
    /// Optional RX timeout value (raw register value).
    pub rx_timeout: Option<u8>,
    /// RX DMA enable bit (just sets UART CTRL bit; DMA setup is out of scope).
    pub rx_dma: bool,
    /// TX DMA enable bit (just sets UART CTRL bit; DMA setup is out of scope).
    pub tx_dma: bool,
    /// RX FIFO interrupt water level (raw 0..=7). Not used unless you enable IRQs yourself.
    pub rx_fifo_level: u8,
    /// TX FIFO interrupt water level (raw 0..=7). Not used unless you enable IRQs yourself.
    pub tx_fifo_level: u8,
}

impl Config {
    /// Create a basic UART configuration.
    #[inline]
    pub const fn new(uart_clk_hz: u32, baud: u32) -> Self {
        Self {
            uart_clk_hz,
            baud,
            parity: Parity::None,
            data_bits: DataBits::Eight,
            rx_timeout: None,
            rx_dma: false,
            tx_dma: false,
            rx_fifo_level: 7, // 8 bytes in HW encoding
            tx_fifo_level: 0,
        }
    }

    /// Compute the `UARTx_BAUD` divisor.
    ///
    /// This follows the formula used by the reference C firmware: `div = uart_clk_hz / baud`.
    /// Returns `None` if the result does not fit in 16 bits or if `baud == 0`.
    #[inline]
    pub const fn baud_divisor(&self) -> Option<u16> {
        if self.baud == 0 {
            return None;
        }
        let div = self.uart_clk_hz / self.baud;
        if div == 0 || div > u16::MAX as u32 {
            None
        } else {
            Some(div as u16)
        }
    }
}

/// UART parity configuration.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Parity {
    None,
    Odd,
    Even,
    /// "Mark" parity (forced 1).
    Mark,
    /// "Space" parity (forced 0).
    Space,
}

/// UART data bits.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum DataBits {
    Eight,
    Nine,
}

/// Errors returned by the UART HAL.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Error {
    /// RX FIFO overflow.
    Overrun,
    /// Parity error.
    Parity,
    /// Stop-bit / framing error.
    Frame,
    /// RX timeout flag.
    RxTimeout,
    /// Invalid configuration (e.g. baud divisor does not fit).
    BadConfig,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            Error::Overrun => "UART overrun",
            Error::Parity => "UART parity error",
            Error::Frame => "UART framing error",
            Error::RxTimeout => "UART RX timeout",
            Error::BadConfig => "UART bad configuration",
        };
        f.write_str(s)
    }
}

impl error::Error for Error {}

impl serial::Error for Error {
    #[inline]
    fn kind(&self) -> serial::ErrorKind {
        match self {
            Error::Overrun => serial::ErrorKind::Overrun,
            Error::Parity => serial::ErrorKind::Parity,
            Error::Frame => serial::ErrorKind::FrameFormat,
            Error::RxTimeout | Error::BadConfig => serial::ErrorKind::Other,
        }
    }
}

impl eio::Error for Error {
    #[inline]
    fn kind(&self) -> eio::ErrorKind {
        match self {
            Error::RxTimeout => eio::ErrorKind::TimedOut,
            Error::BadConfig => eio::ErrorKind::InvalidInput,
            Error::Overrun | Error::Parity | Error::Frame => eio::ErrorKind::Other,
        }
    }
}

/// Error when assigning a pin to a UART function.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct InvalidPin;

/// A TX pin configured for a specific UART instance.
pub struct TxPin<UART> {
    port: Port,
    pin: u8,
    _uart: PhantomData<UART>,
}

/// An RX pin configured for a specific UART instance.
pub struct RxPin<UART> {
    port: Port,
    pin: u8,
    _uart: PhantomData<UART>,
}

impl<UART> TxPin<UART> {
    #[inline(always)]
    pub fn port(&self) -> Port {
        self.port
    }

    #[inline(always)]
    pub fn pin(&self) -> u8 {
        self.pin
    }
}

impl<UART> RxPin<UART> {
    #[inline(always)]
    pub fn port(&self) -> Port {
        self.port
    }

    #[inline(always)]
    pub fn pin(&self) -> u8 {
        self.pin
    }
}

/// A UART peripheral together with its configured pins.
pub struct Uart<UART, TX, RX> {
    uart: UART,
    _tx: TX,
    _rx: RX,
}

mod sealed {
    pub trait Sealed {}
}

/// Common interface needed from a UART instance.
pub trait Instance: sealed::Sealed {
    /// Numeric UART index (0/1/2) used for pin mapping.
    const NUM: u8;
    fn enable_clock(syscon: &pac::SYSCON);
    fn regs() -> &'static Self::Regs;
    type Regs;
}

impl sealed::Sealed for pac::UART0 {}
impl sealed::Sealed for pac::UART1 {}
impl sealed::Sealed for pac::UART2 {}

impl Instance for pac::UART0 {
    type Regs = pac::uart0::RegisterBlock;
    const NUM: u8 = 0;

    #[inline(always)]
    fn enable_clock(syscon: &pac::SYSCON) {
        syscon
            .dev_clk_gate()
            .modify(|_, w| w.uart0_clk_gate().set_bit());
    }

    #[inline(always)]
    fn regs() -> &'static Self::Regs {
        unsafe { &*pac::UART0::ptr() }
    }
}

impl Instance for pac::UART1 {
    type Regs = pac::uart1::RegisterBlock;
    const NUM: u8 = 1;

    #[inline(always)]
    fn enable_clock(syscon: &pac::SYSCON) {
        syscon
            .dev_clk_gate()
            .modify(|_, w| w.uart1_clk_gate().set_bit());
    }

    #[inline(always)]
    fn regs() -> &'static Self::Regs {
        unsafe { &*pac::UART1::ptr() }
    }
}

impl Instance for pac::UART2 {
    type Regs = pac::uart2::RegisterBlock;
    const NUM: u8 = 2;

    #[inline(always)]
    fn enable_clock(syscon: &pac::SYSCON) {
        syscon
            .dev_clk_gate()
            .modify(|_, w| w.uart2_clk_gate().set_bit());
    }

    #[inline(always)]
    fn regs() -> &'static Self::Regs {
        unsafe { &*pac::UART2::ptr() }
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
fn uart_tx_function<UART: Instance>(port: Port, pin: u8) -> Option<u8> {
    // Values derived from `uv-k5-firmware-custom/hardware/dp32g030/portcon.def`.
    match UART::NUM {
        0 => match (port, pin) {
            (Port::B, 7) => Some(2),
            (Port::C, 3) => Some(1),
            _ => None,
        },
        1 => match (port, pin) {
            (Port::A, 7) => Some(1),
            (Port::B, 12) => Some(1),
            _ => None,
        },
        // UART2
        _ => match (port, pin) {
            (Port::B, 0) => Some(1),
            (Port::B, 14) => Some(2),
            _ => None,
        },
    }
}

#[inline(always)]
fn uart_rx_function<UART: Instance>(port: Port, pin: u8) -> Option<u8> {
    match UART::NUM {
        0 => match (port, pin) {
            (Port::B, 8) => Some(2),
            (Port::C, 4) => Some(1),
            _ => None,
        },
        1 => match (port, pin) {
            (Port::A, 8) => Some(1),
            (Port::B, 13) => Some(1),
            _ => None,
        },
        // UART2
        _ => match (port, pin) {
            (Port::B, 1) => Some(1),
            (Port::B, 15) => Some(2),
            _ => None,
        },
    }
}

// --- Pin constructors --------------------------------------------------------

impl TxPin<pac::UART0> {
    #[inline]
    pub fn new(
        pin: crate::dp30g030_hal::gpio::Pin<crate::dp30g030_hal::gpio::Disabled>,
    ) -> Result<Self, InvalidPin> {
        let port = pin.port();
        let p = pin.pin();
        uart_tx_function::<pac::UART0>(port, p)
            .map(|_| Self {
                port,
                pin: p,
                _uart: PhantomData,
            })
            .ok_or(InvalidPin)
    }
}

impl RxPin<pac::UART0> {
    #[inline]
    pub fn new(
        pin: crate::dp30g030_hal::gpio::Pin<crate::dp30g030_hal::gpio::Disabled>,
    ) -> Result<Self, InvalidPin> {
        let port = pin.port();
        let p = pin.pin();
        uart_rx_function::<pac::UART0>(port, p)
            .map(|_| Self {
                port,
                pin: p,
                _uart: PhantomData,
            })
            .ok_or(InvalidPin)
    }
}

impl TxPin<pac::UART1> {
    #[inline]
    pub fn new(
        pin: crate::dp30g030_hal::gpio::Pin<crate::dp30g030_hal::gpio::Disabled>,
    ) -> Result<Self, InvalidPin> {
        let port = pin.port();
        let p = pin.pin();
        uart_tx_function::<pac::UART1>(port, p)
            .map(|_| Self {
                port,
                pin: p,
                _uart: PhantomData,
            })
            .ok_or(InvalidPin)
    }
}

impl RxPin<pac::UART1> {
    #[inline]
    pub fn new(
        pin: crate::dp30g030_hal::gpio::Pin<crate::dp30g030_hal::gpio::Disabled>,
    ) -> Result<Self, InvalidPin> {
        let port = pin.port();
        let p = pin.pin();
        uart_rx_function::<pac::UART1>(port, p)
            .map(|_| Self {
                port,
                pin: p,
                _uart: PhantomData,
            })
            .ok_or(InvalidPin)
    }
}

impl TxPin<pac::UART2> {
    #[inline]
    pub fn new(
        pin: crate::dp30g030_hal::gpio::Pin<crate::dp30g030_hal::gpio::Disabled>,
    ) -> Result<Self, InvalidPin> {
        let port = pin.port();
        let p = pin.pin();
        uart_tx_function::<pac::UART2>(port, p)
            .map(|_| Self {
                port,
                pin: p,
                _uart: PhantomData,
            })
            .ok_or(InvalidPin)
    }
}

impl RxPin<pac::UART2> {
    #[inline]
    pub fn new(
        pin: crate::dp30g030_hal::gpio::Pin<crate::dp30g030_hal::gpio::Disabled>,
    ) -> Result<Self, InvalidPin> {
        let port = pin.port();
        let p = pin.pin();
        uart_rx_function::<pac::UART2>(port, p)
            .map(|_| Self {
                port,
                pin: p,
                _uart: PhantomData,
            })
            .ok_or(InvalidPin)
    }
}

// --- Uart constructors -------------------------------------------------------

impl<UART: Instance, TX, RX> Uart<UART, TX, RX> {
    /// Release the owned UART peripheral and pins.
    #[inline]
    pub fn free(self) -> (UART, TX, RX) {
        (self.uart, self._tx, self._rx)
    }
}

macro_rules! impl_uart {
    ($UART:ty,
     $ctrl:ident, $baud:ident, $tdr:ident, $rdr:ident, $ie:ident, $if_:ident, $fifo:ident, $fc:ident, $rxto:ident) => {
        impl<TX, RX> Uart<$UART, TX, RX>
        where
            $UART: Instance,
        {
            /// Create and configure a UART instance.
            ///
            /// This sets up:
            /// - UART clock gate in `SYSCON.DEV_CLK_GATE`
            /// - Pin mux in `PORTCON.PORTx_SEL*`
            /// - GPIO direction (TX output, RX input) and RX input enable
            /// - UART CTRL (enable RX/TX, parity/data bits, optional DMA bits)
            /// - UART BAUD divisor, FIFO levels, optional RX timeout
            pub fn new(
                uart: $UART,
                syscon: &pac::SYSCON,
                portcon: &pac::PORTCON,
                tx: TxPin<$UART>,
                rx: RxPin<$UART>,
                config: Config,
            ) -> Result<Uart<$UART, TxPin<$UART>, RxPin<$UART>>, Error> {
                <$UART as Instance>::enable_clock(syscon);

                // Ensure GPIO block clocks are enabled for direction + input buffer settings.
                enable_gpio_clock(syscon, tx.port);
                enable_gpio_clock(syscon, rx.port);

                // Configure mux for TX/RX.
                let tx_fn = uart_tx_function::<$UART>(tx.port, tx.pin).ok_or(Error::BadConfig)?;
                let rx_fn = uart_rx_function::<$UART>(rx.port, rx.pin).ok_or(Error::BadConfig)?;
                set_pin_function(portcon, tx.port, tx.pin, tx_fn);
                set_pin_function(portcon, rx.port, rx.pin, rx_fn);

                // Configure GPIO direction and RX input buffer.
                set_direction(tx.port, tx.pin, true);
                set_direction(rx.port, rx.pin, false);
                set_input_enable(portcon, rx.port, rx.pin, true);

                let regs = <$UART as Instance>::regs();

                // Disable UART before reconfiguring.
                regs.$ctrl().modify(|_, w| w.uarten().clear_bit());

                // Baud divisor.
                let div = config.baud_divisor().ok_or(Error::BadConfig)?;
                regs.$baud().write(|w| unsafe { w.baud().bits(div) });

                // FIFO config: levels + clear FIFOs once.
                let rx_level = config.rx_fifo_level & 0x7;
                let tx_level = config.tx_fifo_level & 0x7;
                regs.$fifo().modify(|r, w| unsafe {
                    let mut v = r.bits();
                    // bits [2:0] RF_LEVEL, [5:3] TF_LEVEL, [6] RF_CLR, [7] TF_CLR
                    v &= !0x3F;
                    v |= (rx_level as u32) & 0x7;
                    v |= ((tx_level as u32) & 0x7) << 3;
                    // Clear FIFOs (one-to-clear).
                    v |= (1u32 << 6) | (1u32 << 7);
                    w.bits(v)
                });

                // RX timeout.
                if let Some(v) = config.rx_timeout {
                    regs.$rxto().write(|w| unsafe { w.rxto().bits(v) });
                }

                // Flow control disabled by default.
                regs.$fc().write(|w| unsafe { w.bits(0) });

                // Disable interrupts by default; user can enable explicitly via PAC if desired.
                regs.$ie().write(|w| unsafe { w.bits(0) });

                // Configure CTRL:
                // - enable RX/TX
                // - parity / data bits
                // - optional DMA bits
                regs.$ctrl().modify(|_, w| {
                    w.rxen().set_bit();
                    w.txen().set_bit();
                    w.rxdmaen().bit(config.rx_dma);
                    w.txdmaen().bit(config.tx_dma);
                    w.ninebit().bit(matches!(config.data_bits, DataBits::Nine));
                    w.paren().bit(!matches!(config.parity, Parity::None));

                    // NOTE: PARMD is only meaningful if PAREN=1.
                    let parmd_bits = match config.parity {
                        Parity::Odd | Parity::None => 0b00,
                        Parity::Even => 0b01,
                        Parity::Mark => 0b10,
                        Parity::Space => 0b11,
                    };
                    unsafe { w.parmd().bits(parmd_bits) };
                    w
                });

                // Finally enable UART.
                regs.$ctrl().modify(|_, w| w.uarten().set_bit());

                Ok(Uart {
                    uart,
                    _tx: tx,
                    _rx: rx,
                })
            }
        }

        impl<TX, RX> serial::ErrorType for Uart<$UART, TX, RX> {
            type Error = Error;
        }

        impl<TX, RX> eio::ErrorType for Uart<$UART, TX, RX> {
            type Error = Error;
        }

        impl<TX, RX> serial::Read<u8> for Uart<$UART, TX, RX> {
            fn read(&mut self) -> nb::Result<u8, Self::Error> {
                let regs = <$UART as Instance>::regs();

                let ifr = regs.$if_().read();

                // Latched error flags (write-1-to-clear)
                if ifr.rxfifo_ovf().bit_is_set() {
                    regs.$if_().write(|w| unsafe { w.bits(1u32 << 8) });
                    return Err(nb::Error::Other(Error::Overrun));
                }
                if ifr.parritye().bit_is_set() {
                    regs.$if_().write(|w| unsafe { w.bits(1u32 << 3) });
                    return Err(nb::Error::Other(Error::Parity));
                }
                if ifr.stope().bit_is_set() {
                    regs.$if_().write(|w| unsafe { w.bits(1u32 << 4) });
                    return Err(nb::Error::Other(Error::Frame));
                }
                if ifr.rxto().bit_is_set() {
                    regs.$if_().write(|w| unsafe { w.bits(1u32 << 5) });
                    return Err(nb::Error::Other(Error::RxTimeout));
                }

                if ifr.rxfifo_empty().bit_is_set() {
                    return Err(nb::Error::WouldBlock);
                }

                let data = regs.$rdr().read().rdr().bits() as u16;
                Ok((data & 0xFF) as u8)
            }
        }

        impl<TX, RX> serial::Write<u8> for Uart<$UART, TX, RX> {
            fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
                let regs = <$UART as Instance>::regs();
                let ifr = regs.$if_().read();

                if ifr.txfifo_full().bit_is_set() {
                    return Err(nb::Error::WouldBlock);
                }

                regs.$tdr().write(|w| unsafe { w.tdr().bits(word as u16) });
                Ok(())
            }

            fn flush(&mut self) -> nb::Result<(), Self::Error> {
                let regs = <$UART as Instance>::regs();
                let ifr = regs.$if_().read();
                if ifr.txbusy().bit_is_set() {
                    Err(nb::Error::WouldBlock)
                } else {
                    Ok(())
                }
            }
        }

        impl<TX, RX> eio::Write for Uart<$UART, TX, RX> {
            fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
                for &b in buf {
                    nb::block!(<Self as serial::Write<u8>>::write(self, b))?;
                }
                Ok(buf.len())
            }

            fn flush(&mut self) -> Result<(), Self::Error> {
                nb::block!(<Self as serial::Write<u8>>::flush(self))
            }
        }
    };
}

// Implement for UART0/1/2 with their PAC register accessor names.
impl_uart!(
    pac::UART0,
    uart0_ctrl,
    uart0_baud,
    uart0_tdr,
    uart0_rdr,
    uart0_ie,
    uart0_if,
    uart0_fifo,
    uart0_fc,
    uart0_rxto
);
impl_uart!(
    pac::UART1,
    uart1_ctrl,
    uart1_baud,
    uart1_tdr,
    uart1_rdr,
    uart1_ie,
    uart1_if,
    uart1_fifo,
    uart1_fc,
    uart1_rxto
);
impl_uart!(
    pac::UART2,
    uart2_ctrl,
    uart2_baud,
    uart2_tdr,
    uart2_rdr,
    uart2_ie,
    uart2_if,
    uart2_fifo,
    uart2_fc,
    uart2_rxto
);

// Convenience aliases.
pub type Uart0 = Uart<pac::UART0, TxPin<pac::UART0>, RxPin<pac::UART0>>;
pub type Uart1 = Uart<pac::UART1, TxPin<pac::UART1>, RxPin<pac::UART1>>;
pub type Uart2 = Uart<pac::UART2, TxPin<pac::UART2>, RxPin<pac::UART2>>;
