use embedded_hal::digital::{ErrorType, InputPin, OutputPin, StatefulOutputPin};

use core::{convert::Infallible, marker::PhantomData};

use dp32g030 as pac;

/// GPIO port identifier.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Port {
    A,
    B,
    C,
}

/// Marker: pin is not configured yet.
pub struct Disabled;
/// Marker: pin configured as input.
pub struct Input;
/// Marker: pin configured as push-pull output.
pub struct Output;

/// A single GPIO pin.
///
/// This is intentionally **not** `Copy` to preserve single-owner semantics
/// typical of embedded HALs (even though the underlying hardware is shared).
pub struct Pin<MODE> {
    port: Port,
    pin: u8,
    _mode: PhantomData<MODE>,
}

impl<MODE> Pin<MODE> {
    #[inline(always)]
    pub fn port(&self) -> Port {
        self.port
    }

    #[inline(always)]
    pub fn pin(&self) -> u8 {
        self.pin
    }
}

/// A "flexible" GPIO pin which can be toggled between input/output at runtime.
///
/// This is useful for bit-banged buses (e.g. I2C) where SDA needs to switch
/// between driving and sampling without consuming/recreating pin types.
///
/// It intentionally exposes a low-level API (direction, open-drain, input-enable).
#[derive(Copy, Clone)]
pub struct FlexPin {
    port: Port,
    pin: u8,
}

impl FlexPin {
    #[inline(always)]
    pub const fn new(port: Port, pin: u8) -> Self {
        Self { port, pin }
    }

    #[inline(always)]
    pub fn port(&self) -> Port {
        self.port
    }

    #[inline(always)]
    pub fn pin(&self) -> u8 {
        self.pin
    }

    /// Enable GPIO clock + select GPIO function for this pin.
    #[inline]
    pub fn configure_gpio(&self, syscon: &pac::SYSCON, portcon: &pac::portcon::RegisterBlock) {
        debug_assert!(is_valid_pin(self.port, self.pin));
        enable_gpio_clock(syscon, self.port);
        select_gpio_function(portcon, self.port, self.pin);
    }

    /// Enable/disable input buffer.
    #[inline]
    pub fn set_input_enable(&self, portcon: &pac::portcon::RegisterBlock, enable: bool) {
        debug_assert!(is_valid_pin(self.port, self.pin));
        set_input_enable(portcon, self.port, self.pin, enable);
    }

    /// Enable/disable open-drain pad.
    #[inline]
    pub fn set_open_drain(&self, portcon: &pac::portcon::RegisterBlock, enable: bool) {
        debug_assert!(is_valid_pin(self.port, self.pin));
        set_open_drain(portcon, self.port, self.pin, enable);
    }

    /// Set pin direction (output=true, input=false).
    #[inline]
    pub fn set_output(&self, output: bool) {
        debug_assert!(is_valid_pin(self.port, self.pin));
        set_direction(self.port, self.pin, output);
    }

    /// Drive the pin high/low (for output mode).
    #[inline]
    pub fn write(&self, high: bool) {
        debug_assert!(is_valid_pin(self.port, self.pin));
        write_data_bit(self.port, self.pin, high);
    }

    /// Sample pin level (for input mode).
    #[inline]
    pub fn read(&self) -> bool {
        debug_assert!(is_valid_pin(self.port, self.pin));
        read_data_bit(self.port, self.pin)
    }
}

// Allow using FlexPin directly with embedded-hal digital traits (useful for bit-banged buses).
impl ErrorType for FlexPin {
    type Error = Infallible;
}

impl OutputPin for FlexPin {
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

impl InputPin for FlexPin {
    #[inline]
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.read())
    }

    #[inline]
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!self.read())
    }
}

impl Pin<Disabled> {
    /// Create a new pin handle.
    ///
    /// Note: this does not touch hardware. Call `into_*` to configure mux/dir.
    #[inline(always)]
    pub const fn new(port: Port, pin: u8) -> Self {
        Self {
            port,
            pin,
            _mode: PhantomData,
        }
    }

    /// Configure this pin as push-pull output.
    ///
    /// - Enables the GPIO peripheral clock (via `SYSCON.DEV_CLK_GATE`)
    /// - Selects GPIO function in `PORTCON.PORTx_SEL*` (sets function = 0)
    /// - Disables open-drain for this pin
    /// - Sets direction bit to output
    #[inline]
    pub fn into_push_pull_output(
        self,
        syscon: &pac::SYSCON,
        portcon: &pac::portcon::RegisterBlock,
    ) -> Pin<Output> {
        debug_assert!(is_valid_pin(self.port, self.pin));
        enable_gpio_clock(syscon, self.port);
        select_gpio_function(portcon, self.port, self.pin);
        set_open_drain(portcon, self.port, self.pin, false);
        set_direction(self.port, self.pin, true);

        Pin {
            port: self.port,
            pin: self.pin,
            _mode: PhantomData,
        }
    }

    /// Configure this pin as input (floating).
    ///
    /// - Enables the GPIO peripheral clock
    /// - Selects GPIO function in `PORTCON.PORTx_SEL*`
    /// - Enables input buffer (via `PORTCON.PORTx_IE`)
    /// - Sets direction bit to input
    #[inline]
    pub fn into_floating_input(self, syscon: &pac::SYSCON, portcon: &pac::PORTCON) -> Pin<Input> {
        self.into_floating_input_rb(syscon, portcon)
    }

    /// Same as `into_floating_input` but takes a `RegisterBlock` (usable with `PORTCON::ptr()`).
    #[inline]
    pub fn into_floating_input_rb(
        self,
        syscon: &pac::SYSCON,
        portcon: &pac::portcon::RegisterBlock,
    ) -> Pin<Input> {
        debug_assert!(is_valid_pin(self.port, self.pin));
        enable_gpio_clock(syscon, self.port);
        select_gpio_function(portcon, self.port, self.pin);
        set_input_enable(portcon, self.port, self.pin, true);
        set_direction(self.port, self.pin, false);

        Pin {
            port: self.port,
            pin: self.pin,
            _mode: PhantomData,
        }
    }

    /// Configure this pin as input with pull-up enabled.
    ///
    /// This mirrors what the reference C firmware does for the PTT button:
    /// - GPIO function
    /// - input buffer enabled
    /// - pull-up enabled, pull-down disabled
    /// - open-drain enabled (pad setting used by the reference firmware)
    /// - direction set to input
    #[inline]
    pub fn into_pull_up_input(self, syscon: &pac::SYSCON, portcon: &pac::PORTCON) -> Pin<Input> {
        self.into_pull_up_input_rb(syscon, portcon)
    }

    /// Same as `into_pull_up_input` but takes a `RegisterBlock` (usable with `PORTCON::ptr()`).
    #[inline]
    pub fn into_pull_up_input_rb(
        self,
        syscon: &pac::SYSCON,
        portcon: &pac::portcon::RegisterBlock,
    ) -> Pin<Input> {
        debug_assert!(is_valid_pin(self.port, self.pin));
        enable_gpio_clock(syscon, self.port);
        select_gpio_function(portcon, self.port, self.pin);
        set_input_enable(portcon, self.port, self.pin, true);
        set_pull_up(portcon, self.port, self.pin, true);
        set_pull_down(portcon, self.port, self.pin, false);
        set_open_drain(portcon, self.port, self.pin, true);
        set_direction(self.port, self.pin, false);

        Pin {
            port: self.port,
            pin: self.pin,
            _mode: PhantomData,
        }
    }
}

impl ErrorType for Pin<Output> {
    type Error = Infallible;
}

impl OutputPin for Pin<Output> {
    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        write_data_bit(self.port, self.pin, false);
        Ok(())
    }

    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        write_data_bit(self.port, self.pin, true);
        Ok(())
    }
}

impl StatefulOutputPin for Pin<Output> {
    #[inline]
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(read_data_bit(self.port, self.pin))
    }

    #[inline]
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!read_data_bit(self.port, self.pin))
    }
}

impl ErrorType for Pin<Input> {
    type Error = Infallible;
}

impl InputPin for Pin<Input> {
    #[inline]
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(read_data_bit(self.port, self.pin))
    }

    #[inline]
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!read_data_bit(self.port, self.pin))
    }
}

/// Return `true` if `pin` exists on the given `port`.
///
/// - Ports A/B have pins 0..=15
/// - Port C has pins 0..=7
#[inline(always)]
pub const fn is_valid_pin(port: Port, pin: u8) -> bool {
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
    // Function 0 selects GPIO for all PORTx_SEL* fields (see SVD and C firmware headers).
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
        Port::A => {
            portcon
                .porta_ie()
                .modify(|r, w| unsafe { w.bits(set(r.bits())) });
        }
        Port::B => {
            portcon
                .portb_ie()
                .modify(|r, w| unsafe { w.bits(set(r.bits())) });
        }
        Port::C => {
            portcon
                .portc_ie()
                .modify(|r, w| unsafe { w.bits(set(r.bits())) });
        }
    }
}

#[inline(always)]
fn set_open_drain(portcon: &pac::portcon::RegisterBlock, port: Port, pin: u8, enable: bool) {
    let bit = 1u32 << (pin as u32);
    let set = |v: u32| if enable { v | bit } else { v & !bit };

    match port {
        Port::A => {
            portcon
                .porta_od()
                .modify(|r, w| unsafe { w.bits(set(r.bits())) });
        }
        Port::B => {
            portcon
                .portb_od()
                .modify(|r, w| unsafe { w.bits(set(r.bits())) });
        }
        Port::C => {
            portcon
                .portc_od()
                .modify(|r, w| unsafe { w.bits(set(r.bits())) });
        }
    }
}

#[inline(always)]
fn set_pull_up(portcon: &pac::portcon::RegisterBlock, port: Port, pin: u8, enable: bool) {
    let bit = 1u32 << (pin as u32);
    let set = |v: u32| if enable { v | bit } else { v & !bit };

    match port {
        Port::A => {
            portcon
                .porta_pu()
                .modify(|r, w| unsafe { w.bits(set(r.bits())) });
        }
        Port::B => {
            portcon
                .portb_pu()
                .modify(|r, w| unsafe { w.bits(set(r.bits())) });
        }
        Port::C => {
            portcon
                .portc_pu()
                .modify(|r, w| unsafe { w.bits(set(r.bits())) });
        }
    }
}

#[inline(always)]
fn set_pull_down(portcon: &pac::portcon::RegisterBlock, port: Port, pin: u8, enable: bool) {
    let bit = 1u32 << (pin as u32);
    let set = |v: u32| if enable { v | bit } else { v & !bit };

    match port {
        Port::A => {
            portcon
                .porta_pd()
                .modify(|r, w| unsafe { w.bits(set(r.bits())) });
        }
        Port::B => {
            portcon
                .portb_pd()
                .modify(|r, w| unsafe { w.bits(set(r.bits())) });
        }
        Port::C => {
            portcon
                .portc_pd()
                .modify(|r, w| unsafe { w.bits(set(r.bits())) });
        }
    }
}

#[inline(always)]
fn set_direction(port: Port, pin: u8, output: bool) {
    let bit = 1u32 << (pin as u32);
    let set = |v: u32| if output { v | bit } else { v & !bit };

    // Keep the update atomic w.r.t. interrupts (RMW register).
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

    // Keep the update atomic w.r.t. interrupts (RMW register).
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
