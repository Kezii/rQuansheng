//! BK4819 bit-banged 3-wire bus (SCN/CS, SCL/CLK, SDA/SDIO).
//!
//! This is a direct Rust port of the low-level wire protocol used in
//! `uv-k5-firmware-custom/driver/bk4819.c`:
//! - write: `SCN↓, write_u8(reg), write_u16(data), SCN↑`
//! - read:  `SCN↓, write_u8(reg|0x80), read_u16(), SCN↑`
//! - SDA is bidirectional; reads temporarily switch SDA to input and enable the input buffer.
//!
//! The higher-level register map/config is intentionally not implemented here: this module
//! focuses on a small, ergonomic, reusable bus interface.

#![allow(dead_code)]

use core::convert::Infallible;

use dp32g030 as pac;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};

use crate::bk4819_n;
use dp30g030_hal::gpio::Port;

/// A bidirectional GPIO line (used for BK4819 SDA/SDIO).
///
/// The BK4819 uses a single data pin for both MOSI and MISO. During reads the host must:
/// - enable the input buffer
/// - switch the pin direction to input
///   and then restore output mode afterwards.
pub trait BidiPin: OutputPin + InputPin {
    /// Switch the pin to input mode (and enable input buffer if applicable).
    fn set_to_input(&mut self);
    /// Switch the pin to output mode (and optionally disable input buffer).
    fn set_to_output(&mut self);
}

/// A minimal bus trait for BK4819 register access.
pub trait Bk4819Bus {
    type Error;

    fn write_reg_raw(&mut self, reg: u8, value: u16) -> Result<(), Self::Error>;
    fn read_reg_raw(&mut self, reg: u8) -> Result<u16, Self::Error>;

    fn write_reg<R: bk4819_n::Bk4819Register>(&mut self, reg: R) -> Result<(), Self::Error>;
    fn read_reg<R: bk4819_n::Bk4819Register>(&mut self) -> Result<R, Self::Error>;
}

/// Bit-banged BK4819 bus implementation.
///
/// `SCN` and `SCL` are standard push-pull outputs.
/// `SDA` must be bidirectional (`BidiPin`).
///
/// gpio operations do return Result, but the error type is infallible
/// so we don't waste time handling errors in this module
pub struct Bk4819BitBang<SCN, SCL, SDA, D> {
    scn: SCN,
    scl: SCL,
    sda: SDA,
    delay: D,
    /// Delay used throughout the waveform (microseconds).
    t_us: u32,
}

impl<SCN, SCL, SDA, D> Bk4819BitBang<SCN, SCL, SDA, D>
where
    SCN: OutputPin,
    SCL: OutputPin,
    SDA: BidiPin,
    D: DelayNs,
{
    /// Create a new bus instance.
    ///
    /// By default this uses a 1µs bit delay, matching the reference firmware.
    /// Lines are left in the idle state (SCN=1, SCL=1, SDA=1, SDA as output).
    pub fn new(mut scn: SCN, mut scl: SCL, mut sda: SDA, delay: D) -> Self {
        // Idle state: high/high/high.
        let _ = scn.set_high();
        let _ = scl.set_high();
        sda.set_to_output();
        let _ = sda.set_high();

        Self {
            scn,
            scl,
            sda,
            delay,
            t_us: 1,
        }
    }

    /// Set the delay used by the waveform, in microseconds.
    ///
    /// The reference C code uses 1µs.
    #[inline]
    pub fn set_timing_us(&mut self, t_us: u32) {
        self.t_us = t_us.max(1);
    }

    /// Destroy the bus and return the owned peripherals.
    #[inline]
    pub fn free(self) -> (SCN, SCL, SDA, D) {
        (self.scn, self.scl, self.sda, self.delay)
    }

    #[inline(always)]
    fn dly(&mut self) {
        self.delay.delay_us(self.t_us);
    }

    fn write_u8(&mut self, mut data: u8) {
        self.sda.set_to_output();
        let _ = self.scl.set_low();

        for _ in 0..8 {
            if (data & 0x80) == 0 {
                let _ = self.sda.set_low();
            } else {
                let _ = self.sda.set_high();
            }

            self.dly();
            let _ = self.scl.set_high();
            self.dly();

            data <<= 1;
            let _ = self.scl.set_low();
            self.dly();
        }
    }

    fn write_u16(&mut self, mut data: u16) {
        self.sda.set_to_output();
        let _ = self.scl.set_low();

        for _ in 0..16 {
            if (data & 0x8000) == 0 {
                let _ = self.sda.set_low();
            } else {
                let _ = self.sda.set_high();
            }

            self.dly();
            let _ = self.scl.set_high();

            data <<= 1;

            self.dly();
            let _ = self.scl.set_low();
            self.dly();
        }
    }

    fn read_u16(&mut self) -> u16 {
        self.sda.set_to_input();
        self.dly();

        let mut value: u16 = 0;
        for _ in 0..16 {
            value <<= 1;
            if let Ok(true) = self.sda.is_high() {
                value |= 1;
            }

            let _ = self.scl.set_high();
            self.dly();
            let _ = self.scl.set_low();
            self.dly();
        }

        self.sda.set_to_output();
        value
    }

    /// Low-level register read, raw `u8` register address.
    ///
    /// Mirrors `BK4819_ReadRegister()` in the C reference.
    fn read_reg_raw(&mut self, reg: u8) -> u16 {
        let _ = self.scn.set_high();
        let _ = self.scl.set_low();
        self.dly();

        let _ = self.scn.set_low();
        self.write_u8(reg | 0x80);
        let value = self.read_u16();
        let _ = self.scn.set_high();

        self.dly();

        // Return to idle.
        let _ = self.scl.set_high();
        self.sda.set_to_output();
        let _ = self.sda.set_high();

        value
    }

    /// Low-level register write, raw `u8` register address.
    ///
    /// Mirrors `BK4819_WriteRegister()` in the C reference.
    fn write_reg_raw(&mut self, reg: u8, data: u16) {
        let _ = self.scn.set_high();
        let _ = self.scl.set_low();
        self.dly();

        let _ = self.scn.set_low();
        self.write_u8(reg);
        self.dly();
        self.write_u16(data);
        self.dly();

        let _ = self.scn.set_high();
        self.dly();

        // Return to idle.
        let _ = self.scl.set_high();
        self.sda.set_to_output();
        let _ = self.sda.set_high();
    }

    pub fn write_reg_n<R: bk4819_n::Bk4819Register>(&mut self, reg: R) {
        self.write_reg_raw(R::ADDRESS, reg.serialize());
    }

    pub fn read_reg_n<R: bk4819_n::Bk4819Register>(&mut self) -> R {
        let value = self.read_reg_raw(R::ADDRESS);
        <R as bk4819_n::Bk4819Register>::deserialize(value)
    }
}

impl<SCN, SCL, SDA, D> Bk4819Bus for Bk4819BitBang<SCN, SCL, SDA, D>
where
    SCN: OutputPin,
    SCL: OutputPin,
    SDA: BidiPin,
    D: DelayNs,
{
    type Error = Infallible;

    #[inline]
    fn write_reg_raw(&mut self, reg: u8, value: u16) -> Result<(), Self::Error> {
        self.write_reg_raw(reg, value);
        Ok(())
    }

    #[inline]
    fn read_reg_raw(&mut self, reg: u8) -> Result<u16, Self::Error> {
        let value = self.read_reg_raw(reg);
        Ok(value)
    }

    #[inline]
    fn write_reg<R: bk4819_n::Bk4819Register>(&mut self, reg: R) -> Result<(), Self::Error> {
        Bk4819BitBang::write_reg_n(self, reg);
        Ok(())
    }

    #[inline]
    fn read_reg<R: bk4819_n::Bk4819Register>(&mut self) -> Result<R, Self::Error> {
        let value = Bk4819BitBang::read_reg_n::<R>(self);
        Ok(value)
    }
}

/// A thin convenience wrapper representing a BK4819 accessed through some bus.
pub struct Bk4819<BUS> {
    bus: BUS,
}

impl<BUS> Bk4819<BUS> {
    #[inline]
    pub const fn new(bus: BUS) -> Self {
        Self { bus }
    }

    #[inline]
    pub fn free(self) -> BUS {
        self.bus
    }
}

impl<BUS> Bk4819<BUS>
where
    BUS: Bk4819Bus,
{
    #[inline]
    pub fn write_reg_raw(&mut self, reg: u8, value: u16) -> Result<(), BUS::Error> {
        self.bus.write_reg_raw(reg, value)
    }

    #[inline]
    pub fn read_reg_raw(&mut self, reg: u8) -> Result<u16, BUS::Error> {
        self.bus.read_reg_raw(reg)
    }

    #[inline]
    pub fn write_reg<R: bk4819_n::Bk4819Register>(&mut self, reg: R) -> Result<(), BUS::Error> {
        self.bus.write_reg(reg)
    }

    #[inline]
    pub fn read_reg<R: bk4819_n::Bk4819Register>(&mut self) -> Result<R, BUS::Error> {
        self.bus.read_reg::<R>()
    }

    /// Read-modify-write helper.
    #[inline]
    pub fn update_reg<F>(&mut self, reg: u8, f: F) -> Result<u16, BUS::Error>
    where
        F: FnOnce(u16) -> u16,
    {
        let cur = self.read_reg_raw(reg)?;
        let next = f(cur);
        self.write_reg_raw(reg, next)?;
        Ok(next)
    }
}

/// Invalid DP32G030 GPIO pin (out of range for the port).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct InvalidPin;

/// DP32G030-specific bidirectional GPIO pin implementation for BK4819 SDA.
///
/// This mirrors the reference firmware behavior:
/// - on read: enable input buffer + set DIR=input
/// - on write: disable input buffer + set DIR=output
pub struct Dp32g030BidiPin {
    port: Port,
    pin: u8,
}

impl Dp32g030BidiPin {
    /// Configure a GPIO pin for BK4819 SDA usage.
    ///
    /// - selects GPIO function for the pin
    /// - starts in output mode with input buffer disabled (like the C code)
    pub fn new(
        port: Port,
        pin: u8,
        syscon: &pac::SYSCON,
        portcon: &pac::portcon::RegisterBlock,
    ) -> Result<Self, InvalidPin> {
        if !is_valid_pin(port, pin) {
            return Err(InvalidPin);
        }

        enable_gpio_clock(syscon, port);
        select_gpio_function(portcon, port, pin);

        // Default to output with input buffer disabled.
        set_input_enable(portcon, port, pin, false);
        set_direction(port, pin, true);

        Ok(Self { port, pin })
    }

    #[inline(always)]
    fn read(&self) -> bool {
        read_data_bit(self.port, self.pin)
    }

    #[inline(always)]
    fn write(&self, high: bool) {
        write_data_bit(self.port, self.pin, high)
    }
}

impl embedded_hal::digital::ErrorType for Dp32g030BidiPin {
    type Error = Infallible;
}

impl OutputPin for Dp32g030BidiPin {
    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.write(false);
        Ok(())
    }

    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.write(true);
        Ok(())
    }
}

impl InputPin for Dp32g030BidiPin {
    #[inline]
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.read())
    }

    #[inline]
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!self.read())
    }
}

impl BidiPin for Dp32g030BidiPin {
    #[inline]
    fn set_to_input(&mut self) {
        unsafe {
            let portcon = &*pac::PORTCON::ptr();
            set_input_enable(portcon, self.port, self.pin, true);
        }
        set_direction(self.port, self.pin, false);
    }

    #[inline]
    fn set_to_output(&mut self) {
        unsafe {
            let portcon = &*pac::PORTCON::ptr();
            set_input_enable(portcon, self.port, self.pin, false);
        }
        set_direction(self.port, self.pin, true);
    }
}

// --- DP32G030 register helpers (local, to keep this module self-contained) ---

#[inline(always)]
fn is_valid_pin(port: Port, pin: u8) -> bool {
    match port {
        Port::A | Port::B => pin <= 15,
        Port::C => pin <= 7,
    }
}

#[inline(always)]
fn enable_gpio_clock(syscon: &pac::SYSCON, port: Port) {
    syscon.dev_clk_gate().modify(|_, w| match port {
        Port::A => w.gpioa_clk_gate().set_bit(),
        Port::B => w.gpiob_clk_gate().set_bit(),
        Port::C => w.gpioc_clk_gate().set_bit(),
    });
}

#[inline(always)]
fn select_gpio_function(portcon: &pac::portcon::RegisterBlock, port: Port, pin: u8) {
    // Function 0 selects GPIO for all PORTx_SEL* fields.
    const GPIO_FUNCTION: u32 = 0;
    let shift = ((pin % 8) as u32) * 4;
    let mask = 0xFu32 << shift;

    match port {
        Port::A => {
            if pin < 8 {
                portcon.porta_sel0().modify(|r, w| unsafe {
                    w.bits((r.bits() & !mask) | (GPIO_FUNCTION << shift))
                });
            } else {
                portcon.porta_sel1().modify(|r, w| unsafe {
                    w.bits((r.bits() & !mask) | (GPIO_FUNCTION << shift))
                });
            }
        }
        Port::B => {
            if pin < 8 {
                portcon.portb_sel0().modify(|r, w| unsafe {
                    w.bits((r.bits() & !mask) | (GPIO_FUNCTION << shift))
                });
            } else {
                portcon.portb_sel1().modify(|r, w| unsafe {
                    w.bits((r.bits() & !mask) | (GPIO_FUNCTION << shift))
                });
            }
        }
        Port::C => {
            portcon
                .portc_sel0()
                .modify(|r, w| unsafe { w.bits((r.bits() & !mask) | (GPIO_FUNCTION << shift)) });
        }
    }
}

#[inline(always)]
fn set_input_enable(portcon: &pac::portcon::RegisterBlock, port: Port, pin: u8, enable: bool) {
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
fn read_data_bit(port: Port, pin: u8) -> bool {
    let mask = 1u32 << (pin as u32);
    match port {
        Port::A => unsafe { (&*pac::GPIOA::ptr()).gpioa_data().read().bits() & mask != 0 },
        Port::B => unsafe { (&*pac::GPIOB::ptr()).gpiob_data().read().bits() & mask != 0 },
        Port::C => unsafe { (&*pac::GPIOC::ptr()).gpioc_data().read().bits() & mask != 0 },
    }
}

#[inline(always)]
fn write_data_bit(port: Port, pin: u8, high: bool) {
    let bit = 1u32 << (pin as u32);
    let set = |v: u32| if high { v | bit } else { v & !bit };

    cortex_m::interrupt::free(|_| match port {
        Port::A => unsafe {
            let gpioa = &*pac::GPIOA::ptr();
            gpioa.gpioa_data().modify(|r, w| w.bits(set(r.bits())));
        },
        Port::B => unsafe {
            let gpiob = &*pac::GPIOB::ptr();
            gpiob.gpiob_data().modify(|r, w| w.bits(set(r.bits())));
        },
        Port::C => unsafe {
            let gpioc = &*pac::GPIOC::ptr();
            gpioc.gpioc_data().modify(|r, w| w.bits(set(r.bits())));
        },
    });
}
