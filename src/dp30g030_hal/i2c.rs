//! I2C (IIC) HAL for DP32G030, implemented in an idiomatic embedded Rust style.
//!
//! This module provides a minimal, "standard" embedded-hal implementation:
//! - Uses `embedded-hal` 1.0 blocking I2C traits (`embedded_hal::i2c::I2c`)
//! - Polling / blocking operation (no IRQ/DMA)
//! - Strongly-typed SCL/SDA pins with runtime validation (pin mux via PORTCON)
//! - Uses the PAC (`dp32g030`) for register access
//!
//! Notes:
//! - This targets the **hardware** IIC peripherals (IIC0/IIC1), not the bit-banged
//!   GPIO I2C used by the original C firmware (`uv-k5-firmware-custom/driver/i2c.c`).
//! - Timing is configured through `IICx_MSPC` using an integer search for a close
//!   match to the requested bus frequency.
//! - Timeouts are implemented as simple iteration limits (no timer dependency).

use core::marker::PhantomData;

use dp32g030 as pac;

use embedded_hal::i2c;

use crate::dp30g030_hal::gpio::Port;

/// IIC / I2C configuration.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Config {
    /// IIC peripheral clock in Hz (pclk / bus clock used for MSPC timing).
    pub pclk_hz: u32,
    /// Desired I2C bus speed in Hz (e.g. 100_000 / 400_000).
    pub bus_hz: u32,
    /// Digital noise filter length (DNF field), 0..=15.
    pub dnf: u8,
    /// SDA data hold time (DAT_HD field), 0..=15.
    pub dat_hd: u8,
    /// Max poll iterations for each wait loop (timeout).
    pub timeout_iters: u32,
}

impl Config {
    /// Create a basic config for a target I2C bus frequency.
    #[inline]
    pub const fn new(pclk_hz: u32, bus_hz: u32) -> Self {
        Self {
            pclk_hz,
            bus_hz,
            dnf: 0,
            dat_hd: 0,
            timeout_iters: 100_000,
        }
    }

    #[inline]
    const fn clamp4(v: u8) -> u8 {
        if v > 15 {
            15
        } else {
            v
        }
    }

    #[inline]
    fn norm(self) -> Self {
        Self {
            dnf: Self::clamp4(self.dnf),
            dat_hd: Self::clamp4(self.dat_hd),
            ..self
        }
    }
}

/// Errors returned by this I2C HAL.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Error {
    /// Invalid configuration (bad clock, impossible timings, invalid address, etc).
    BadConfig,
    /// Bus is stuck busy or did not return idle.
    BusBusy,
    /// No ACK received (address or data).
    NoAck(i2c::NoAcknowledgeSource),
    /// SCL low timeout (MLTO).
    Timeout,
    /// Receive overrun/overflow.
    Overrun,
    /// Generic bus error.
    Bus,
}

impl i2c::Error for Error {
    #[inline]
    fn kind(&self) -> i2c::ErrorKind {
        match *self {
            Error::BadConfig => i2c::ErrorKind::Other,
            Error::BusBusy => i2c::ErrorKind::Bus,
            Error::NoAck(src) => i2c::ErrorKind::NoAcknowledge(src),
            Error::Timeout => i2c::ErrorKind::Other,
            Error::Overrun => i2c::ErrorKind::Overrun,
            Error::Bus => i2c::ErrorKind::Bus,
        }
    }
}

/// Error when assigning a pin to an IIC function.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct InvalidPin;

/// An SCL pin configured for a specific IIC instance.
pub struct SclPin<IIC> {
    port: Port,
    pin: u8,
    _iic: PhantomData<IIC>,
}

/// An SDA pin configured for a specific IIC instance.
pub struct SdaPin<IIC> {
    port: Port,
    pin: u8,
    _iic: PhantomData<IIC>,
}

impl<IIC> SclPin<IIC> {
    #[inline(always)]
    pub fn port(&self) -> Port {
        self.port
    }

    #[inline(always)]
    pub fn pin(&self) -> u8 {
        self.pin
    }
}

impl<IIC> SdaPin<IIC> {
    #[inline(always)]
    pub fn port(&self) -> Port {
        self.port
    }

    #[inline(always)]
    pub fn pin(&self) -> u8 {
        self.pin
    }
}

/// An I2C master peripheral together with its configured pins.
pub struct I2c<IIC, SCL, SDA> {
    iic: IIC,
    _scl: SCL,
    _sda: SDA,
    config: Config,
}

mod sealed {
    pub trait Sealed {}
}

/// Common interface needed from an IIC instance.
pub trait Instance: sealed::Sealed {
    const NUM: u8;
    fn enable_clock(syscon: &pac::SYSCON);
    fn regs() -> &'static Self::Regs;
    type Regs;
}

impl sealed::Sealed for pac::IIC0 {}
impl sealed::Sealed for pac::IIC1 {}

impl Instance for pac::IIC0 {
    type Regs = pac::iic0::RegisterBlock;
    const NUM: u8 = 0;

    #[inline(always)]
    fn enable_clock(syscon: &pac::SYSCON) {
        syscon
            .dev_clk_gate()
            .modify(|_, w| w.iic0_clk_gate().set_bit());
    }

    #[inline(always)]
    fn regs() -> &'static Self::Regs {
        unsafe { &*pac::IIC0::ptr() }
    }
}

impl Instance for pac::IIC1 {
    type Regs = pac::iic1::RegisterBlock;
    const NUM: u8 = 1;

    #[inline(always)]
    fn enable_clock(syscon: &pac::SYSCON) {
        syscon
            .dev_clk_gate()
            .modify(|_, w| w.iic1_clk_gate().set_bit());
    }

    #[inline(always)]
    fn regs() -> &'static Self::Regs {
        unsafe { &*pac::IIC1::ptr() }
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
fn set_open_drain(portcon: &pac::PORTCON, port: Port, pin: u8, enable: bool) {
    let bit = 1u32 << (pin as u32);
    let set = |v: u32| if enable { v | bit } else { v & !bit };

    match port {
        Port::A => portcon
            .porta_od()
            .modify(|r, w| unsafe { w.bits(set(r.bits())) }),
        Port::B => portcon
            .portb_od()
            .modify(|r, w| unsafe { w.bits(set(r.bits())) }),
        Port::C => portcon
            .portc_od()
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
            portcon
                .portc_sel0()
                .modify(|r, w| unsafe { w.bits((r.bits() & !mask) | (f << shift)) });
        }
    }
}

#[inline(always)]
fn iic_scl_function<IIC: Instance>(port: Port, pin: u8) -> Option<u8> {
    // Values derived from `uv-k5-firmware-custom/hardware/dp32g030/portcon.def`.
    match IIC::NUM {
        // IIC0
        0 => match (port, pin) {
            (Port::B, 0) => Some(2),
            (Port::B, 7) => Some(3),
            (Port::C, 3) => Some(2),
            _ => None,
        },
        // IIC1
        _ => match (port, pin) {
            (Port::B, 4) => Some(2),
            (Port::B, 12) => Some(2),
            (Port::C, 6) => Some(1),
            _ => None,
        },
    }
}

#[inline(always)]
fn iic_sda_function<IIC: Instance>(port: Port, pin: u8) -> Option<u8> {
    match IIC::NUM {
        // IIC0
        0 => match (port, pin) {
            (Port::B, 1) => Some(2),
            (Port::B, 8) => Some(3),
            (Port::C, 4) => Some(2),
            _ => None,
        },
        // IIC1
        _ => match (port, pin) {
            (Port::B, 3) => Some(2),
            (Port::B, 13) => Some(2),
            (Port::C, 7) => Some(1),
            _ => None,
        },
    }
}

// --- Pin constructors --------------------------------------------------------

impl SclPin<pac::IIC0> {
    #[inline]
    pub fn new(
        pin: crate::dp30g030_hal::gpio::Pin<crate::dp30g030_hal::gpio::Disabled>,
    ) -> Result<Self, InvalidPin> {
        let port = pin.port();
        let p = pin.pin();
        iic_scl_function::<pac::IIC0>(port, p)
            .map(|_| Self {
                port,
                pin: p,
                _iic: PhantomData,
            })
            .ok_or(InvalidPin)
    }
}

impl SdaPin<pac::IIC0> {
    #[inline]
    pub fn new(
        pin: crate::dp30g030_hal::gpio::Pin<crate::dp30g030_hal::gpio::Disabled>,
    ) -> Result<Self, InvalidPin> {
        let port = pin.port();
        let p = pin.pin();
        iic_sda_function::<pac::IIC0>(port, p)
            .map(|_| Self {
                port,
                pin: p,
                _iic: PhantomData,
            })
            .ok_or(InvalidPin)
    }
}

impl SclPin<pac::IIC1> {
    #[inline]
    pub fn new(
        pin: crate::dp30g030_hal::gpio::Pin<crate::dp30g030_hal::gpio::Disabled>,
    ) -> Result<Self, InvalidPin> {
        let port = pin.port();
        let p = pin.pin();
        iic_scl_function::<pac::IIC1>(port, p)
            .map(|_| Self {
                port,
                pin: p,
                _iic: PhantomData,
            })
            .ok_or(InvalidPin)
    }
}

impl SdaPin<pac::IIC1> {
    #[inline]
    pub fn new(
        pin: crate::dp30g030_hal::gpio::Pin<crate::dp30g030_hal::gpio::Disabled>,
    ) -> Result<Self, InvalidPin> {
        let port = pin.port();
        let p = pin.pin();
        iic_sda_function::<pac::IIC1>(port, p)
            .map(|_| Self {
                port,
                pin: p,
                _iic: PhantomData,
            })
            .ok_or(InvalidPin)
    }
}

// --- Internal timing ---------------------------------------------------------

#[derive(Copy, Clone, Debug)]
struct Timing {
    scl_low: u8,
    scl_hi: u8,
    cpd: u8,
    dat_hd: u8,
}

fn compute_timing(cfg: Config) -> Option<Timing> {
    let cfg = cfg.norm();
    if cfg.pclk_hz == 0 || cfg.bus_hz == 0 {
        return None;
    }
    // Total ticks per I2C period in pclk cycles.
    let target_ticks = cfg.pclk_hz / cfg.bus_hz;
    if target_ticks < 10 {
        return None;
    }

    // Split high/low with recommended ~1:2.
    let target_hi = target_ticks / 3;
    let target_lo = target_ticks.saturating_sub(target_hi);

    let dnf = cfg.dnf as u32;
    let dat_hd = cfg.dat_hd as u32;

    // Brute-force search for best (CPD, SCL_HI, SCL_LOW) fitting in 8-bit fields.
    let mut best: Option<(u32, Timing)> = None;

    for cpd in 0u32..=255 {
        let cp = cpd + 1;
        // Remove fixed offsets (DNF+6, DAT_HD+5), keep at least 1 tick.
        let hi_base = target_hi.saturating_sub(dnf + 6);
        let lo_base = target_lo.saturating_sub(dat_hd + 5);
        if hi_base == 0 || lo_base == 0 {
            continue;
        }

        // Compute (SCL_HI+1) and (SCL_LOW+1) as ceiling division.
        let scl_hi1 = hi_base.div_ceil(cp);
        let scl_lo1 = lo_base.div_ceil(cp);
        if !(1..=256).contains(&scl_hi1) || !(1..=256).contains(&scl_lo1) {
            continue;
        }

        let scl_hi = (scl_hi1 - 1) as u8;
        let scl_low = (scl_lo1 - 1) as u8;

        // Estimate achieved ticks.
        let t_hi = (scl_hi1 * cp) + dnf + 6;
        let t_lo = (scl_lo1 * cp) + dat_hd + 5;
        let achieved = t_hi + t_lo;
        let err = achieved.abs_diff(target_ticks);

        let timing = Timing {
            scl_low,
            scl_hi,
            cpd: cpd as u8,
            dat_hd: cfg.dat_hd,
        };

        best = match best {
            None => Some((err, timing)),
            Some((best_err, best_t)) => {
                if err < best_err {
                    Some((err, timing))
                } else {
                    Some((best_err, best_t))
                }
            }
        };
    }

    best.map(|(_, t)| t)
}

// --- Constructors / API ------------------------------------------------------

impl<IIC: Instance, SCL, SDA> I2c<IIC, SCL, SDA> {
    /// Release the owned IIC peripheral and pins.
    #[inline]
    pub fn free(self) -> (IIC, SCL, SDA) {
        (self.iic, self._scl, self._sda)
    }
}

macro_rules! impl_iic_master {
    ($IIC:ty,
     $ccfg:ident, $cst:ident, $ctrans:ident, $rxdata:ident, $txdata:ident, $ie:ident, $if_:ident, $mctrl:ident, $mspc:ident) => {
        impl I2c<$IIC, SclPin<$IIC>, SdaPin<$IIC>>
        where
            $IIC: Instance,
        {
            /// Create and configure a hardware IIC instance (master mode).
            ///
            /// This sets up:
            /// - IIC clock gate in `SYSCON.DEV_CLK_GATE`
            /// - Pin mux in `PORTCON.PORTx_SEL*`
            /// - Open-drain + input enable for SCL/SDA
            /// - IIC timing (`IICx_MSPC`) and enable (`IICx_CCFG`)
            pub fn new(
                iic: $IIC,
                syscon: &pac::SYSCON,
                portcon: &pac::PORTCON,
                scl: SclPin<$IIC>,
                sda: SdaPin<$IIC>,
                config: Config,
            ) -> Result<Self, Error> {
                let config = config.norm();
                let timing = compute_timing(config).ok_or(Error::BadConfig)?;

                <$IIC as Instance>::enable_clock(syscon);

                // Ensure GPIO block clocks are enabled for direction / IO settings.
                enable_gpio_clock(syscon, scl.port);
                enable_gpio_clock(syscon, sda.port);

                // Configure mux for SCL/SDA.
                let scl_fn = iic_scl_function::<$IIC>(scl.port, scl.pin).ok_or(Error::BadConfig)?;
                let sda_fn = iic_sda_function::<$IIC>(sda.port, sda.pin).ok_or(Error::BadConfig)?;
                set_pin_function(portcon, scl.port, scl.pin, scl_fn);
                set_pin_function(portcon, sda.port, sda.pin, sda_fn);

                // I2C uses open-drain lines. Enable input buffers too (SCL sampled for stretching).
                set_open_drain(portcon, scl.port, scl.pin, true);
                set_open_drain(portcon, sda.port, sda.pin, true);
                set_input_enable(portcon, scl.port, scl.pin, true);
                set_input_enable(portcon, sda.port, sda.pin, true);

                // Set direction output; open-drain means "high" is released.
                set_direction(scl.port, scl.pin, true);
                set_direction(sda.port, sda.pin, true);

                let regs = <$IIC as Instance>::regs();

                // Disable and configure.
                regs.$ccfg().modify(|_, w| w.en().clear_bit());

                // Disable interrupts by default.
                regs.$ie().write(|w| unsafe { w.bits(0) });

                // Configure timings.
                regs.$mspc().modify(|_, w| unsafe {
                    w.scl_low().bits(timing.scl_low);
                    w.scl_hi().bits(timing.scl_hi);
                    w.cpd().bits(timing.cpd);
                    w.dat_hd().bits(timing.dat_hd);
                    w
                });

                // Configure master mode + DNF + enable bus.
                regs.$ccfg().modify(|_, w| unsafe {
                    w.mode().set_bit();
                    w.dnf().bits(config.dnf);
                    w.en().set_bit();
                    w
                });

                Ok(Self {
                    iic,
                    _scl: scl,
                    _sda: sda,
                    config,
                })
            }
        }

        impl i2c::ErrorType for I2c<$IIC, SclPin<$IIC>, SdaPin<$IIC>> {
            type Error = Error;
        }

        impl i2c::I2c<i2c::SevenBitAddress> for I2c<$IIC, SclPin<$IIC>, SdaPin<$IIC>> {
            fn transaction(
                &mut self,
                address: u8,
                operations: &mut [i2c::Operation<'_>],
            ) -> Result<(), Self::Error> {
                if address > 0x7F {
                    return Err(Error::BadConfig);
                }
                let regs = <$IIC as Instance>::regs();
                let cfg = self.config;

                // If bus is busy before we start, bail (caller may retry / recover).
                if regs.$cst().read().busy().bit_is_set() {
                    return Err(Error::BusBusy);
                }

                // Best-effort clear sticky flags before starting.
                // (write-1-to-clear bits; harmless if already clear)
                regs.$if_().write(|w| {
                    w.mlto()
                        .bit(true)
                        .rxovf()
                        .bit(true)
                        .txf()
                        .bit(true)
                        .rxf()
                        .bit(true)
                });

                let mut started = false;

                for (op_idx, op) in operations.iter_mut().enumerate() {
                    let is_first = op_idx == 0;
                    let _ = is_first;

                    match op {
                        i2c::Operation::Write(bytes) => {
                            if bytes.is_empty() {
                                continue;
                            }
                            // START (or repeated START) + address (write) as first byte.
                            if !started {
                                self.master_start_address(regs, address, false, cfg.timeout_iters)?;
                                started = true;
                            } else {
                                self.master_start_address(regs, address, false, cfg.timeout_iters)?;
                            }
                            self.master_write_bytes(regs, bytes, cfg.timeout_iters)?;
                        }
                        i2c::Operation::Read(buf) => {
                            if buf.is_empty() {
                                continue;
                            }
                            // START (or repeated START) + address (read)
                            if !started {
                                self.master_start_address(regs, address, true, cfg.timeout_iters)?;
                                started = true;
                            } else {
                                self.master_start_address(regs, address, true, cfg.timeout_iters)?;
                            }
                            self.master_read_bytes(regs, buf, cfg.timeout_iters)?;
                        }
                    }
                }

                // Always terminate with STOP if we started anything.
                if started {
                    self.master_stop(regs, cfg.timeout_iters)?;
                }

                Ok(())
            }
        }

        impl I2c<$IIC, SclPin<$IIC>, SdaPin<$IIC>> {
            #[inline(always)]
            fn wait_flag_set(
                &self,
                regs: &<$IIC as Instance>::Regs,
                mut pred: impl FnMut() -> bool,
                timeout: u32,
            ) -> Result<(), Error> {
                for _ in 0..timeout {
                    // Surface timeout/overrun flags even while waiting.
                    let ifr = regs.$if_().read();
                    if ifr.mlto().bit_is_set() {
                        regs.$if_().write(|w| w.mlto().bit(true));
                        return Err(Error::Timeout);
                    }
                    if ifr.rxovf().bit_is_set() {
                        regs.$if_().write(|w| w.rxovf().bit(true));
                        return Err(Error::Overrun);
                    }
                    if pred() {
                        return Ok(());
                    }
                }
                Err(Error::Timeout)
            }

            #[inline(always)]
            fn wait_txe(&self, regs: &<$IIC as Instance>::Regs, timeout: u32) -> Result<(), Error> {
                self.wait_flag_set(regs, || regs.$if_().read().txe().bit_is_set(), timeout)
            }

            #[inline(always)]
            fn wait_txf(&self, regs: &<$IIC as Instance>::Regs, timeout: u32) -> Result<(), Error> {
                self.wait_flag_set(regs, || regs.$if_().read().txf().bit_is_set(), timeout)
            }

            #[inline(always)]
            fn wait_rxf(&self, regs: &<$IIC as Instance>::Regs, timeout: u32) -> Result<(), Error> {
                self.wait_flag_set(regs, || regs.$if_().read().rxf().bit_is_set(), timeout)
            }

            #[inline(always)]
            fn check_noack(
                &self,
                regs: &<$IIC as Instance>::Regs,
                src: i2c::NoAcknowledgeSource,
            ) -> Result<(), Error> {
                if regs.$ctrans().read().rx_ack().bit_is_set() {
                    Err(Error::NoAck(src))
                } else {
                    Ok(())
                }
            }

            fn master_start_address(
                &mut self,
                regs: &<$IIC as Instance>::Regs,
                address: u8,
                read: bool,
                timeout: u32,
            ) -> Result<(), Error> {
                // Prepare address byte.
                let addr = (address << 1) | (read as u8);

                self.wait_txe(regs, timeout)?;
                regs.$txdata().write(|w| unsafe { w.txdata().bits(addr) });

                // Generate START + send byte.
                regs.$mctrl().write(|w| w.sta().set_bit().wr().set_bit());

                self.wait_txf(regs, timeout)?;
                // Clear TXF
                regs.$if_().write(|w| w.txf().bit(true));
                // Check ACK
                self.check_noack(regs, i2c::NoAcknowledgeSource::Address)?;
                Ok(())
            }

            fn master_write_bytes(
                &mut self,
                regs: &<$IIC as Instance>::Regs,
                bytes: &[u8],
                timeout: u32,
            ) -> Result<(), Error> {
                for &b in bytes {
                    self.wait_txe(regs, timeout)?;
                    regs.$txdata().write(|w| unsafe { w.txdata().bits(b) });
                    regs.$mctrl().write(|w| w.wr().set_bit());
                    self.wait_txf(regs, timeout)?;
                    regs.$if_().write(|w| w.txf().bit(true));
                    self.check_noack(regs, i2c::NoAcknowledgeSource::Data)?;
                }
                Ok(())
            }

            fn master_read_bytes(
                &mut self,
                regs: &<$IIC as Instance>::Regs,
                buf: &mut [u8],
                timeout: u32,
            ) -> Result<(), Error> {
                let n = buf.len();
                for (i, out) in buf.iter_mut().enumerate() {
                    let last = i + 1 == n;
                    // For last byte, send NACK; otherwise ACK.
                    regs.$ctrans().modify(|_, w| w.tx_ack().bit(last));

                    regs.$mctrl().write(|w| w.rd().set_bit());
                    self.wait_rxf(regs, timeout)?;

                    *out = regs.$rxdata().read().rxdata().bits();
                    regs.$if_().write(|w| w.rxf().bit(true));
                }
                Ok(())
            }

            fn master_stop(
                &mut self,
                regs: &<$IIC as Instance>::Regs,
                timeout: u32,
            ) -> Result<(), Error> {
                regs.$mctrl().write(|w| w.sto().set_bit());

                // Wait until bus returns idle.
                for _ in 0..timeout {
                    if !regs.$cst().read().busy().bit_is_set() {
                        return Ok(());
                    }
                }
                Err(Error::BusBusy)
            }
        }
    };
}

// Implement for IIC0/1 with their PAC register accessor names.
impl_iic_master!(
    pac::IIC0,
    iic0_ccfg,
    iic0_cst,
    iic0_ctrans,
    iic0_rxdata,
    iic0_txdata,
    iic0_ie,
    iic0_if,
    iic0_mctrl,
    iic0_mspc
);
impl_iic_master!(
    pac::IIC1,
    iic1_ccfg,
    iic1_cst,
    iic1_ctrans,
    iic1_rxdata,
    iic1_txdata,
    iic1_ie,
    iic1_if,
    iic1_mctrl,
    iic1_mspc
);

// Convenience aliases.
pub type Iic0 = I2c<pac::IIC0, SclPin<pac::IIC0>, SdaPin<pac::IIC0>>;
pub type Iic1 = I2c<pac::IIC1, SclPin<pac::IIC1>, SdaPin<pac::IIC1>>;
