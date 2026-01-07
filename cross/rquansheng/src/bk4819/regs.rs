#![deprecated]
//! BK4819 register map and bitfield helpers.
//!
//! Ported from `uv-k5-firmware-custom/driver/bk4819-regs.h`.

/// BK4819 register address.
///
/// Matches the `BK4819_REGISTER_t` enum in the C reference.
#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Register_old {
    Reg00 = 0x00,
    Reg02 = 0x02,
    Reg06 = 0x06,
    Reg07 = 0x07,
    Reg08 = 0x08,
    Reg09 = 0x09,
    Reg0B = 0x0B,
    Reg0C = 0x0C,
    Reg0D = 0x0D,
    Reg0E = 0x0E,
    Reg10 = 0x10,
    Reg11 = 0x11,
    Reg12 = 0x12,
    Reg13 = 0x13,
    Reg14 = 0x14,
    Reg19 = 0x19,
    Reg1F = 0x1F,
    Reg20 = 0x20,
    Reg21 = 0x21,
    Reg24 = 0x24,
    Reg28 = 0x28,
    Reg29 = 0x29,
    Reg2B = 0x2B,
    Reg30 = 0x30,
    Reg31 = 0x31,
    Reg32 = 0x32,
    Reg33 = 0x33,
    Reg36 = 0x36,
    Reg37 = 0x37,
    Reg38 = 0x38,
    Reg39 = 0x39,
    Reg3A = 0x3A,
    Reg3B = 0x3B,
    Reg3C = 0x3C,
    Reg3D = 0x3D,
    Reg3E = 0x3E,
    Reg3F = 0x3F,
    Reg43 = 0x43,
    Reg46 = 0x46,
    Reg47 = 0x47,
    Reg48 = 0x48,
    Reg49 = 0x49,
    Reg4D = 0x4D,
    Reg4E = 0x4E,
    Reg4F = 0x4F,
    Reg50 = 0x50,
    Reg51 = 0x51,
    Reg52 = 0x52,
    Reg58 = 0x58,
    Reg59 = 0x59,
    Reg5A = 0x5A,
    Reg5B = 0x5B,
    Reg5C = 0x5C,
    Reg5D = 0x5D,
    Reg5F = 0x5F,
    Reg63 = 0x63,
    Reg64 = 0x64,
    Reg65 = 0x65,
    Reg67 = 0x67,
    Reg68 = 0x68,
    Reg69 = 0x69,
    Reg6A = 0x6A,
    Reg6F = 0x6F,
    Reg70 = 0x70,
    Reg71 = 0x71,
    Reg72 = 0x72,
    Reg73 = 0x73,
    Reg78 = 0x78,
    Reg79 = 0x79,
    Reg7A = 0x7A,
    Reg7B = 0x7B,
    Reg7C = 0x7C,
    Reg7D = 0x7D,
    Reg7E = 0x7E,
}

impl Register_old {
    #[inline]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }
}

/// GPIO pins as exposed by the C driver.
#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum GpioPin {
    Gpio0Pin28RxEnable = 0,
    Gpio1Pin29PaEnable = 1,
    Gpio3Pin31UhfLna = 3,
    Gpio4Pin32VhfLna = 4,
    Gpio5Pin1Red = 5,
    Gpio6Pin2Green = 6,
}

// --- REG_07 ---

pub const REG_07_SHIFT_FREQUENCY_MODE: u16 = 13;
pub const REG_07_SHIFT_FREQUENCY: u16 = 0;

pub const REG_07_MODE_CTC1: u16 = 0u16 << REG_07_SHIFT_FREQUENCY_MODE;
pub const REG_07_MODE_CTC2: u16 = 1u16 << REG_07_SHIFT_FREQUENCY_MODE;
pub const REG_07_MODE_CDCSS: u16 = 2u16 << REG_07_SHIFT_FREQUENCY_MODE;

// --- REG_24 ---

pub const REG_24_SHIFT_UNKNOWN_15: u16 = 15;
pub const REG_24_SHIFT_THRESHOLD: u16 = 7;
pub const REG_24_SHIFT_UNKNOWN_6: u16 = 6;
pub const REG_24_SHIFT_ENABLE: u16 = 5;
pub const REG_24_SHIFT_SELECT: u16 = 4;
pub const REG_24_SHIFT_MAX_SYMBOLS: u16 = 0;

pub const REG_24_ENABLE: u16 = 1u16 << REG_24_SHIFT_ENABLE;
pub const REG_24_DISABLE: u16 = 0u16 << REG_24_SHIFT_ENABLE;
pub const REG_24_SELECT_DTMF: u16 = 1u16 << REG_24_SHIFT_SELECT;
pub const REG_24_SELECT_SELCALL: u16 = 0u16 << REG_24_SHIFT_SELECT;

// --- REG_30 ---

pub const REG_30_SHIFT_ENABLE_VCO_CALIB: u16 = 15;
pub const REG_30_SHIFT_ENABLE_UNKNOWN: u16 = 14;
pub const REG_30_SHIFT_ENABLE_RX_LINK: u16 = 10;
pub const REG_30_SHIFT_ENABLE_AF_DAC: u16 = 9;
pub const REG_30_SHIFT_ENABLE_DISC_MODE: u16 = 8;
pub const REG_30_SHIFT_ENABLE_PLL_VCO: u16 = 4;
pub const REG_30_SHIFT_ENABLE_PA_GAIN: u16 = 3;
pub const REG_30_SHIFT_ENABLE_MIC_ADC: u16 = 2;
pub const REG_30_SHIFT_ENABLE_TX_DSP: u16 = 1;
pub const REG_30_SHIFT_ENABLE_RX_DSP: u16 = 0;

pub const REG_30_ENABLE_VCO_CALIB: u16 = 1u16 << REG_30_SHIFT_ENABLE_VCO_CALIB;
pub const REG_30_DISABLE_VCO_CALIB: u16 = 0u16 << REG_30_SHIFT_ENABLE_VCO_CALIB;
pub const REG_30_ENABLE_UNKNOWN: u16 = 1u16 << REG_30_SHIFT_ENABLE_UNKNOWN;
pub const REG_30_DISABLE_UNKNOWN: u16 = 0u16 << REG_30_SHIFT_ENABLE_UNKNOWN;
pub const REG_30_ENABLE_RX_LINK: u16 = 0xFu16 << REG_30_SHIFT_ENABLE_RX_LINK;
pub const REG_30_DISABLE_RX_LINK: u16 = 0u16 << REG_30_SHIFT_ENABLE_RX_LINK;
pub const REG_30_ENABLE_AF_DAC: u16 = 1u16 << REG_30_SHIFT_ENABLE_AF_DAC;
pub const REG_30_DISABLE_AF_DAC: u16 = 0u16 << REG_30_SHIFT_ENABLE_AF_DAC;
pub const REG_30_ENABLE_DISC_MODE: u16 = 1u16 << REG_30_SHIFT_ENABLE_DISC_MODE;
pub const REG_30_DISABLE_DISC_MODE: u16 = 0u16 << REG_30_SHIFT_ENABLE_DISC_MODE;
pub const REG_30_ENABLE_PLL_VCO: u16 = 0xFu16 << REG_30_SHIFT_ENABLE_PLL_VCO;
pub const REG_30_DISABLE_PLL_VCO: u16 = 0u16 << REG_30_SHIFT_ENABLE_PLL_VCO;
pub const REG_30_ENABLE_PA_GAIN: u16 = 1u16 << REG_30_SHIFT_ENABLE_PA_GAIN;
pub const REG_30_DISABLE_PA_GAIN: u16 = 0u16 << REG_30_SHIFT_ENABLE_PA_GAIN;
pub const REG_30_ENABLE_MIC_ADC: u16 = 1u16 << REG_30_SHIFT_ENABLE_MIC_ADC;
pub const REG_30_DISABLE_MIC_ADC: u16 = 0u16 << REG_30_SHIFT_ENABLE_MIC_ADC;
pub const REG_30_ENABLE_TX_DSP: u16 = 1u16 << REG_30_SHIFT_ENABLE_TX_DSP;
pub const REG_30_DISABLE_TX_DSP: u16 = 0u16 << REG_30_SHIFT_ENABLE_TX_DSP;
pub const REG_30_ENABLE_RX_DSP: u16 = 1u16 << REG_30_SHIFT_ENABLE_RX_DSP;
pub const REG_30_DISABLE_RX_DSP: u16 = 0u16 << REG_30_SHIFT_ENABLE_RX_DSP;

// --- REG_51 ---

pub const REG_51_SHIFT_ENABLE_CXCSS: u16 = 15;
pub const REG_51_SHIFT_GPIO6_PIN2_INPUT: u16 = 14;
pub const REG_51_SHIFT_TX_CDCSS_POLARITY: u16 = 13;
pub const REG_51_SHIFT_CXCSS_MODE: u16 = 12;
pub const REG_51_SHIFT_CDCSS_BIT_WIDTH: u16 = 11;
pub const REG_51_SHIFT_1050HZ_DETECTION: u16 = 10;
pub const REG_51_SHIFT_AUTO_CDCSS_BW: u16 = 9;
pub const REG_51_SHIFT_AUTO_CTCSS_BW: u16 = 8;
pub const REG_51_SHIFT_CXCSS_TX_GAIN1: u16 = 0;

pub const REG_51_ENABLE_CXCSS: u16 = 1u16 << REG_51_SHIFT_ENABLE_CXCSS;
pub const REG_51_DISABLE_CXCSS: u16 = 0u16 << REG_51_SHIFT_ENABLE_CXCSS;

pub const REG_51_GPIO6_PIN2_INPUT: u16 = 1u16 << REG_51_SHIFT_GPIO6_PIN2_INPUT;
pub const REG_51_GPIO6_PIN2_NORMAL: u16 = 0u16 << REG_51_SHIFT_GPIO6_PIN2_INPUT;

pub const REG_51_TX_CDCSS_NEGATIVE: u16 = 1u16 << REG_51_SHIFT_TX_CDCSS_POLARITY;
pub const REG_51_TX_CDCSS_POSITIVE: u16 = 0u16 << REG_51_SHIFT_TX_CDCSS_POLARITY;

pub const REG_51_MODE_CTCSS: u16 = 1u16 << REG_51_SHIFT_CXCSS_MODE;
pub const REG_51_MODE_CDCSS: u16 = 0u16 << REG_51_SHIFT_CXCSS_MODE;

pub const REG_51_CDCSS_24_BIT: u16 = 1u16 << REG_51_SHIFT_CDCSS_BIT_WIDTH;
pub const REG_51_CDCSS_23_BIT: u16 = 0u16 << REG_51_SHIFT_CDCSS_BIT_WIDTH;

pub const REG_51_1050HZ_DETECTION: u16 = 1u16 << REG_51_SHIFT_1050HZ_DETECTION;
pub const REG_51_1050HZ_NO_DETECTION: u16 = 0u16 << REG_51_SHIFT_1050HZ_DETECTION;

pub const REG_51_AUTO_CDCSS_BW_DISABLE: u16 = 1u16 << REG_51_SHIFT_AUTO_CDCSS_BW;
pub const REG_51_AUTO_CDCSS_BW_ENABLE: u16 = 0u16 << REG_51_SHIFT_AUTO_CDCSS_BW;

pub const REG_51_AUTO_CTCSS_BW_DISABLE: u16 = 1u16 << REG_51_SHIFT_AUTO_CTCSS_BW;
pub const REG_51_AUTO_CTCSS_BW_ENABLE: u16 = 0u16 << REG_51_SHIFT_AUTO_CTCSS_BW;

// --- REG_70 ---

pub const REG_70_SHIFT_ENABLE_TONE1: u16 = 15;
pub const REG_70_SHIFT_TONE1_TUNING_GAIN: u16 = 8;
pub const REG_70_SHIFT_ENABLE_TONE2: u16 = 7;
pub const REG_70_SHIFT_TONE2_TUNING_GAIN: u16 = 0;

pub const REG_70_ENABLE_TONE1: u16 = 1u16 << REG_70_SHIFT_ENABLE_TONE1;
pub const REG_70_ENABLE_TONE2: u16 = 1u16 << REG_70_SHIFT_ENABLE_TONE2;
