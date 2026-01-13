//! BK1080 FM broadcast receiver support (UV-K5).
//!
//! This module is a Rust port of the reference firmware implementation:
//! `uv-k5-firmware-custom/driver/bk1080.c` + `driver/bk1080-regs.h`.

pub mod driver;
pub mod regs;

pub use driver::*;
