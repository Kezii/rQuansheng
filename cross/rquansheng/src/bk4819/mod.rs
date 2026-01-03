//! High-level BK4819 driver (register programming, audio/DSP features).
//!
//! This is still a bit shit and needs a rewrite

pub mod driver;
pub mod regs;

pub use driver::{
    AfType, Bk4819Driver, CompanderMode, CssScanResult, FilterBandwidth, GpioPin, RogerMode,
};
