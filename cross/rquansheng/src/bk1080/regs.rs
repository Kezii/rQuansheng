//! BK1080 register definitions.
//!
//! This is a best-effort mapping based on the C reference firmware:
//! `uv-k5-firmware-custom/driver/bk1080-regs.h` and usage in `driver/bk1080.c`.
//!
//! Units note: the C firmware uses `u16` frequencies in **100 kHz steps**
//! (e.g. 875 == 87.5 MHz, 1080 == 108.0 MHz).

use bitfield_struct::{bitenum, bitfield};
use bk4819_reg_macros::address;

// Reuse the same address trait as BK4819: `#[address(..)]` implements this.
pub use crate::bk4819_n::RegisterAddress;

/// BK1080 FM band selection (REG_05[7:6]).
///
/// Derived from the reference firmware behavior:
/// - `BK1080_GetFreqLoLimit(band)` / `BK1080_GetFreqHiLimit(band)`
/// - UI prints ".5" only for band 0 (87.5 MHz lower edge)
#[bitenum()]
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FmBand {
    #[fallback]
    /// 87.5–108.0 MHz (common EU/US broadcast band).
    UsEu875To1080 = 0,
    /// 76.0–108.0 MHz (Japan wide).
    JapanWide760To1080 = 1,
    /// 76.0–90.0 MHz (Japan).
    Japan760To900 = 2,
    /// 64.0–76.0 MHz (low VHF FM; sometimes referred to as OIRT-like).
    Low640To760 = 3,
}

impl FmBand {
    /// Lower frequency limit in 100 kHz units (e.g. 875 == 87.5 MHz).
    ///
    /// Mirrors `BK1080_GetFreqLoLimit(band)`.
    #[inline]
    pub const fn lo_limit_100khz(self) -> u16 {
        match self {
            Self::UsEu875To1080 => 875,
            Self::JapanWide760To1080 => 760,
            Self::Japan760To900 => 760,
            Self::Low640To760 => 640,
        }
    }

    /// Upper frequency limit in 100 kHz units (e.g. 1080 == 108.0 MHz).
    ///
    /// Mirrors `BK1080_GetFreqHiLimit(band)`.
    #[inline]
    pub const fn hi_limit_100khz(self) -> u16 {
        match self {
            Self::UsEu875To1080 => 1080,
            Self::JapanWide760To1080 => 1080,
            Self::Japan760To900 => 900,
            Self::Low640To760 => 760,
        }
    }
}

/// Marker trait for BK1080 register value types.
pub trait Bk1080Register:
    RegisterAddress + Copy + From<u16> + Into<u16> + core::fmt::Debug + Default
{
    #[inline]
    fn serialize(self) -> u16 {
        self.into()
    }

    #[inline]
    fn deserialize(data: u16) -> Self {
        Self::from(data)
    }
}

impl<T> Bk1080Register for T where
    T: RegisterAddress + Copy + From<u16> + Into<u16> + core::fmt::Debug + Default
{
}

// --- Typed register bitfields (only for fields used by the firmware) --------

/// REG_02: Power configuration (raw 16-bit value).
///
/// The reference firmware writes fixed values (e.g. `0x0201`, `0x4201`, `0x0241`).
#[bitfield(u16)]
#[address(0x02)]
pub struct Reg02 {
    #[bits(16)]
    pub raw: u16,
}

/// REG_03: Channel + Tune bit.
///
/// The reference firmware writes:
/// - `channel`
/// - then `channel | 0x8000` to trigger tune.
#[bitfield(u16)]
#[address(0x03)]
pub struct Reg03 {
    /// Channel number (0..=0x7FFF).
    #[bits(15)]
    pub channel: u16,
    /// Tune trigger.
    pub tune: bool,
}

/// REG_05: System configuration 2.
///
/// The reference firmware masks bits 6..7 to set band:
/// `regval = (regval & ~(0b11 << 6)) | ((band & 0b11) << 6);`
#[bitfield(u16)]
#[address(0x05)]
pub struct Reg05 {
    #[bits(4)]
    pub undocumented_0: u8,
    /// Channel spacing (unused in the reference firmware; kept for completeness).
    #[bits(2)]
    pub spacing: u8,
    /// Band selection (2 bits).
    #[bits(2)]
    pub band: FmBand,
    #[bits(8)]
    pub undocumented_1: u8,
}

/// REG_07: Frequency deviation + SNR.
///
/// From `bk1080-regs.h`:
/// - FREQD: bits 15..4 (12 bits)
/// - SNR:   bits 3..0  (4 bits)
#[bitfield(u16)]
#[address(0x07)]
pub struct Reg07 {
    /// SNR (4 bits).
    #[bits(4)]
    pub snr: u8,
    /// Frequency deviation (12 bits).
    #[bits(12)]
    pub freqd: u16,
}

/// REG_0A: RSSI + AFC rail indicator.
///
/// From `bk1080-regs.h`:
/// - RSSI:  bits 7..0
/// - AFCRL: bit 12
#[bitfield(u16)]
#[address(0x0A)]
pub struct Reg0A {
    /// RSSI (8 bits).
    #[bits(8)]
    pub rssi: u8,
    #[bits(4)]
    pub undocumented_0: u8,
    /// AFC rail indicator.
    pub afcrl: bool,
    #[bits(3)]
    pub undocumented_1: u8,
}

/// REG_19: Internal register (raw 16-bit value).
///
/// The reference firmware writes `0xA83C` then `0xA8BC`.
#[bitfield(u16)]
#[address(0x19)]
pub struct Reg19 {
    #[bits(16)]
    pub raw: u16,
}
