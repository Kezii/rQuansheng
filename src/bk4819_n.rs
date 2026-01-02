use bitfield_struct::{bitenum, bitfield};

// THIS DRIVER IS A BEST EFFORT ATTEMPT AT PARSING THE DOCUMENTATION
// WARNING: THIS DRIVER IS NOT GUARANTEED TO BE CORRECT
// WARNING: THIS HAS BEEN TOUCHED BY AN LLM
// WARNING: EVEN IF CORRECT, SOME FIELDS ARE UNDOCUMENTED IN THE DOCUMENTATION
//          BUT THEY ARE USED IN THE FIRMWARE

#[derive(Copy, Clone, Debug)]
pub enum Register {
    Reg00(Reg00),
    Reg02(Reg02),
    Reg07(Reg07),
    Reg08(Reg08),
    Reg09(Reg09),
    Reg0A(Reg0A),
    Reg0B(Reg0B),
    Reg0C(Reg0C),
    Reg0D(Reg0D),
    Reg0E(Reg0E),
    Reg10(Reg10),
    Reg11(Reg11),
    Reg12(Reg12),
    Reg13(Reg13),
    Reg14(Reg14),
    Reg19(Reg19),
    Reg1A(Reg1A),
    Reg1F(Reg1F),
    Reg24(Reg24),
    Reg28(Reg28),
    Reg29(Reg29),
    Reg2B(Reg2B),
    Reg2E(Reg2E),
    Reg30(Reg30),
    Reg31(Reg31),
    Reg32(Reg32),
    Reg33(Reg33),
    Reg34(Reg34),
    Reg35(Reg35),
    Reg36(Reg36),
    Reg37(Reg37),
    Reg38(Reg38),
    Reg39(Reg39),
    Reg3B(Reg3B),
    Reg3C(Reg3C),
    Reg3D(Reg3D),
    Reg3E(Reg3E),
    Reg3F(Reg3F),
    Reg40(Reg40),
    Reg43(Reg43),
    Reg44(Reg44),
    Reg45(Reg45),
    Reg46(Reg46),
    Reg47(Reg47),
    Reg48(Reg48),
    Reg4B(Reg4B),
    Reg4D(Reg4D),
    Reg4E(Reg4E),
    Reg4F(Reg4F),
    Reg50(Reg50),
    Reg51(Reg51),
    Reg52(Reg52),
    Reg54(Reg54),
    Reg55(Reg55),
    Reg58(Reg58),
    Reg59(Reg59),
    Reg5A(Reg5A),
    Reg5B(Reg5B),
    Reg5C(Reg5C),
    Reg5D(Reg5D),
    Reg5E(Reg5E),
    Reg5F(Reg5F),
    Reg63(Reg63),
    Reg64(Reg64),
    Reg65(Reg65),
    Reg67(Reg67),
    Reg68(Reg68),
    Reg69(Reg69),
    Reg6A(Reg6A),
    Reg6F(Reg6F),
    Reg70(Reg70),
    Reg71(Reg71),
    Reg72(Reg72),
    Reg73(Reg73),
    Reg74(Reg74),
    Reg75(Reg75),
    Reg78(Reg78),
    Reg79(Reg79),
    Reg7A(Reg7A),
    Reg7D(Reg7D),
    Reg7E(Reg7E),
}

impl Register {
    /// Return the BK4819 register address (0x00..=0x7E).
    pub const fn as_u8(&self) -> u8 {
        match self {
            Register::Reg00(_) => 0x00,
            Register::Reg02(_) => 0x02,
            Register::Reg07(_) => 0x07,
            Register::Reg08(_) => 0x08,
            Register::Reg09(_) => 0x09,
            Register::Reg0A(_) => 0x0A,
            Register::Reg0B(_) => 0x0B,
            Register::Reg0C(_) => 0x0C,
            Register::Reg0D(_) => 0x0D,
            Register::Reg0E(_) => 0x0E,
            Register::Reg10(_) => 0x10,
            Register::Reg11(_) => 0x11,
            Register::Reg12(_) => 0x12,
            Register::Reg13(_) => 0x13,
            Register::Reg14(_) => 0x14,
            Register::Reg19(_) => 0x19,
            Register::Reg1A(_) => 0x1A,
            Register::Reg1F(_) => 0x1F,
            Register::Reg24(_) => 0x24,
            Register::Reg28(_) => 0x28,
            Register::Reg29(_) => 0x29,
            Register::Reg2B(_) => 0x2B,
            Register::Reg2E(_) => 0x2E,
            Register::Reg30(_) => 0x30,
            Register::Reg31(_) => 0x31,
            Register::Reg32(_) => 0x32,
            Register::Reg33(_) => 0x33,
            Register::Reg34(_) => 0x34,
            Register::Reg35(_) => 0x35,
            Register::Reg36(_) => 0x36,
            Register::Reg37(_) => 0x37,
            Register::Reg38(_) => 0x38,
            Register::Reg39(_) => 0x39,
            Register::Reg3B(_) => 0x3B,
            Register::Reg3C(_) => 0x3C,
            Register::Reg3D(_) => 0x3D,
            Register::Reg3E(_) => 0x3E,
            Register::Reg3F(_) => 0x3F,
            Register::Reg40(_) => 0x40,
            Register::Reg43(_) => 0x43,
            Register::Reg44(_) => 0x44,
            Register::Reg45(_) => 0x45,
            Register::Reg46(_) => 0x46,
            Register::Reg47(_) => 0x47,
            Register::Reg48(_) => 0x48,
            Register::Reg4B(_) => 0x4B,
            Register::Reg4D(_) => 0x4D,
            Register::Reg4E(_) => 0x4E,
            Register::Reg4F(_) => 0x4F,
            Register::Reg50(_) => 0x50,
            Register::Reg51(_) => 0x51,
            Register::Reg52(_) => 0x52,
            Register::Reg54(_) => 0x54,
            Register::Reg55(_) => 0x55,
            Register::Reg58(_) => 0x58,
            Register::Reg59(_) => 0x59,
            Register::Reg5A(_) => 0x5A,
            Register::Reg5B(_) => 0x5B,
            Register::Reg5C(_) => 0x5C,
            Register::Reg5D(_) => 0x5D,
            Register::Reg5E(_) => 0x5E,
            Register::Reg5F(_) => 0x5F,
            Register::Reg63(_) => 0x63,
            Register::Reg64(_) => 0x64,
            Register::Reg65(_) => 0x65,
            Register::Reg67(_) => 0x67,
            Register::Reg68(_) => 0x68,
            Register::Reg69(_) => 0x69,
            Register::Reg6A(_) => 0x6A,
            Register::Reg6F(_) => 0x6F,
            Register::Reg70(_) => 0x70,
            Register::Reg71(_) => 0x71,
            Register::Reg72(_) => 0x72,
            Register::Reg73(_) => 0x73,
            Register::Reg74(_) => 0x74,
            Register::Reg75(_) => 0x75,
            Register::Reg78(_) => 0x78,
            Register::Reg79(_) => 0x79,
            Register::Reg7A(_) => 0x7A,
            Register::Reg7D(_) => 0x7D,
            Register::Reg7E(_) => 0x7E,
        }
    }

    pub fn serialize(&self) -> u16 {
        match self {
            Register::Reg00(reg) => (*reg).into(),
            Register::Reg02(reg) => (*reg).into(),
            Register::Reg07(reg) => (*reg).into(),
            Register::Reg08(reg) => (*reg).into(),
            Register::Reg09(reg) => (*reg).into(),
            Register::Reg0A(reg) => (*reg).into(),
            Register::Reg0B(reg) => (*reg).into(),
            Register::Reg0C(reg) => (*reg).into(),
            Register::Reg0D(reg) => (*reg).into(),
            Register::Reg0E(reg) => (*reg).into(),
            Register::Reg10(reg) => (*reg).into(),
            Register::Reg11(reg) => (*reg).into(),
            Register::Reg12(reg) => (*reg).into(),
            Register::Reg13(reg) => (*reg).into(),
            Register::Reg14(reg) => (*reg).into(),
            Register::Reg19(reg) => (*reg).into(),
            Register::Reg1A(reg) => (*reg).into(),
            Register::Reg1F(reg) => (*reg).into(),
            Register::Reg24(reg) => (*reg).into(),
            Register::Reg28(reg) => (*reg).into(),
            Register::Reg29(reg) => (*reg).into(),
            Register::Reg2B(reg) => (*reg).into(),
            Register::Reg2E(reg) => (*reg).into(),
            Register::Reg30(reg) => (*reg).into(),
            Register::Reg31(reg) => (*reg).into(),
            Register::Reg32(reg) => (*reg).into(),
            Register::Reg33(reg) => (*reg).into(),
            Register::Reg34(reg) => (*reg).into(),
            Register::Reg35(reg) => (*reg).into(),
            Register::Reg36(reg) => (*reg).into(),
            Register::Reg37(reg) => (*reg).into(),
            Register::Reg38(reg) => (*reg).into(),
            Register::Reg39(reg) => (*reg).into(),
            Register::Reg3B(reg) => (*reg).into(),
            Register::Reg3C(reg) => (*reg).into(),
            Register::Reg3D(reg) => (*reg).into(),
            Register::Reg3E(reg) => (*reg).into(),
            Register::Reg3F(reg) => (*reg).into(),
            Register::Reg40(reg) => (*reg).into(),
            Register::Reg43(reg) => (*reg).into(),
            Register::Reg44(reg) => (*reg).into(),
            Register::Reg45(reg) => (*reg).into(),
            Register::Reg46(reg) => (*reg).into(),
            Register::Reg47(reg) => (*reg).into(),
            Register::Reg48(reg) => (*reg).into(),
            Register::Reg4B(reg) => (*reg).into(),
            Register::Reg4D(reg) => (*reg).into(),
            Register::Reg4E(reg) => (*reg).into(),
            Register::Reg4F(reg) => (*reg).into(),
            Register::Reg50(reg) => (*reg).into(),
            Register::Reg51(reg) => (*reg).into(),
            Register::Reg52(reg) => (*reg).into(),
            Register::Reg54(reg) => (*reg).into(),
            Register::Reg55(reg) => (*reg).into(),
            Register::Reg58(reg) => (*reg).into(),
            Register::Reg59(reg) => (*reg).into(),
            Register::Reg5A(reg) => (*reg).into(),
            Register::Reg5B(reg) => (*reg).into(),
            Register::Reg5C(reg) => (*reg).into(),
            Register::Reg5D(reg) => (*reg).into(),
            Register::Reg5E(reg) => (*reg).into(),
            Register::Reg5F(reg) => (*reg).into(),
            Register::Reg63(reg) => (*reg).into(),
            Register::Reg64(reg) => (*reg).into(),
            Register::Reg65(reg) => (*reg).into(),
            Register::Reg67(reg) => (*reg).into(),
            Register::Reg68(reg) => (*reg).into(),
            Register::Reg69(reg) => (*reg).into(),
            Register::Reg6A(reg) => (*reg).into(),
            Register::Reg6F(reg) => (*reg).into(),
            Register::Reg70(reg) => (*reg).into(),
            Register::Reg71(reg) => (*reg).into(),
            Register::Reg72(reg) => (*reg).into(),
            Register::Reg73(reg) => (*reg).into(),
            Register::Reg74(reg) => (*reg).into(),
            Register::Reg75(reg) => (*reg).into(),
            Register::Reg78(reg) => (*reg).into(),
            Register::Reg79(reg) => (*reg).into(),
            Register::Reg7A(reg) => (*reg).into(),
            Register::Reg7D(reg) => (*reg).into(),
            Register::Reg7E(reg) => (*reg).into(),
        }
    }

    pub fn deserialize(&self, data: u16) -> Self {
        match self {
            Register::Reg00(_) => Register::Reg00(Reg00::from(data)),
            Register::Reg02(_) => Register::Reg02(Reg02::from(data)),
            Register::Reg07(_) => Register::Reg07(Reg07::from(data)),
            Register::Reg08(_) => Register::Reg08(Reg08::from(data)),
            Register::Reg09(_) => Register::Reg09(Reg09::from(data)),
            Register::Reg0A(_) => Register::Reg0A(Reg0A::from(data)),
            Register::Reg0B(_) => Register::Reg0B(Reg0B::from(data)),
            Register::Reg0C(_) => Register::Reg0C(Reg0C::from(data)),
            Register::Reg0D(_) => Register::Reg0D(Reg0D::from(data)),
            Register::Reg0E(_) => Register::Reg0E(Reg0E::from(data)),
            Register::Reg10(_) => Register::Reg10(Reg10::from(data)),
            Register::Reg11(_) => Register::Reg11(Reg11::from(data)),
            Register::Reg12(_) => Register::Reg12(Reg12::from(data)),
            Register::Reg13(_) => Register::Reg13(Reg13::from(data)),
            Register::Reg14(_) => Register::Reg14(Reg14::from(data)),
            Register::Reg19(_) => Register::Reg19(Reg19::from(data)),
            Register::Reg1A(_) => Register::Reg1A(Reg1A::from(data)),
            Register::Reg1F(_) => Register::Reg1F(Reg1F::from(data)),
            Register::Reg24(_) => Register::Reg24(Reg24::from(data)),
            Register::Reg28(_) => Register::Reg28(Reg28::from(data)),
            Register::Reg29(_) => Register::Reg29(Reg29::from(data)),
            Register::Reg2B(_) => Register::Reg2B(Reg2B::from(data)),
            Register::Reg2E(_) => Register::Reg2E(Reg2E::from(data)),
            Register::Reg30(_) => Register::Reg30(Reg30::from(data)),
            Register::Reg31(_) => Register::Reg31(Reg31::from(data)),
            Register::Reg32(_) => Register::Reg32(Reg32::from(data)),
            Register::Reg33(_) => Register::Reg33(Reg33::from(data)),
            Register::Reg34(_) => Register::Reg34(Reg34::from(data)),
            Register::Reg35(_) => Register::Reg35(Reg35::from(data)),
            Register::Reg36(_) => Register::Reg36(Reg36::from(data)),
            Register::Reg37(_) => Register::Reg37(Reg37::from(data)),
            Register::Reg38(_) => Register::Reg38(Reg38::from(data)),
            Register::Reg39(_) => Register::Reg39(Reg39::from(data)),
            Register::Reg3B(_) => Register::Reg3B(Reg3B::from(data)),
            Register::Reg3C(_) => Register::Reg3C(Reg3C::from(data)),
            Register::Reg3D(_) => Register::Reg3D(Reg3D::from(data)),
            Register::Reg3E(_) => Register::Reg3E(Reg3E::from(data)),
            Register::Reg3F(_) => Register::Reg3F(Reg3F::from(data)),
            Register::Reg40(_) => Register::Reg40(Reg40::from(data)),
            Register::Reg43(_) => Register::Reg43(Reg43::from(data)),
            Register::Reg44(_) => Register::Reg44(Reg44::from(data)),
            Register::Reg45(_) => Register::Reg45(Reg45::from(data)),
            Register::Reg46(_) => Register::Reg46(Reg46::from(data)),
            Register::Reg47(_) => Register::Reg47(Reg47::from(data)),
            Register::Reg48(_) => Register::Reg48(Reg48::from(data)),
            Register::Reg4B(_) => Register::Reg4B(Reg4B::from(data)),
            Register::Reg4D(_) => Register::Reg4D(Reg4D::from(data)),
            Register::Reg4E(_) => Register::Reg4E(Reg4E::from(data)),
            Register::Reg4F(_) => Register::Reg4F(Reg4F::from(data)),
            Register::Reg50(_) => Register::Reg50(Reg50::from(data)),
            Register::Reg51(_) => Register::Reg51(Reg51::from(data)),
            Register::Reg52(_) => Register::Reg52(Reg52::from(data)),
            Register::Reg54(_) => Register::Reg54(Reg54::from(data)),
            Register::Reg55(_) => Register::Reg55(Reg55::from(data)),
            Register::Reg58(_) => Register::Reg58(Reg58::from(data)),
            Register::Reg59(_) => Register::Reg59(Reg59::from(data)),
            Register::Reg5A(_) => Register::Reg5A(Reg5A::from(data)),
            Register::Reg5B(_) => Register::Reg5B(Reg5B::from(data)),
            Register::Reg5C(_) => Register::Reg5C(Reg5C::from(data)),
            Register::Reg5D(_) => Register::Reg5D(Reg5D::from(data)),
            Register::Reg5E(_) => Register::Reg5E(Reg5E::from(data)),
            Register::Reg5F(_) => Register::Reg5F(Reg5F::from(data)),
            Register::Reg63(_) => Register::Reg63(Reg63::from(data)),
            Register::Reg64(_) => Register::Reg64(Reg64::from(data)),
            Register::Reg65(_) => Register::Reg65(Reg65::from(data)),
            Register::Reg67(_) => Register::Reg67(Reg67::from(data)),
            Register::Reg68(_) => Register::Reg68(Reg68::from(data)),
            Register::Reg69(_) => Register::Reg69(Reg69::from(data)),
            Register::Reg6A(_) => Register::Reg6A(Reg6A::from(data)),
            Register::Reg6F(_) => Register::Reg6F(Reg6F::from(data)),
            Register::Reg70(_) => Register::Reg70(Reg70::from(data)),
            Register::Reg71(_) => Register::Reg71(Reg71::from(data)),
            Register::Reg72(_) => Register::Reg72(Reg72::from(data)),
            Register::Reg73(_) => Register::Reg73(Reg73::from(data)),
            Register::Reg74(_) => Register::Reg74(Reg74::from(data)),
            Register::Reg75(_) => Register::Reg75(Reg75::from(data)),
            Register::Reg78(_) => Register::Reg78(Reg78::from(data)),
            Register::Reg79(_) => Register::Reg79(Reg79::from(data)),
            Register::Reg7A(_) => Register::Reg7A(Reg7A::from(data)),
            Register::Reg7D(_) => Register::Reg7D(Reg7D::from(data)),
            Register::Reg7E(_) => Register::Reg7E(Reg7E::from(data)),
        }
    }
}

/// Auto-generated register bitfields/enums from `BK4819V3.svd`.
///
/// Note: This is a best-effort mapping; verify against the datasheet for tricky fields.

#[bitenum()]
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Reg00SoftReset {
    #[fallback]
    /// Normal operation
    Normal = 0,
    /// Reset
    Reset = 1,
}

#[bitenum()]
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Reg07Mode {
    #[fallback]
    /// CTC1 frequency control word in WORD[12:0]
    Ctc1 = 0,
    /// CTC2 (Tail 55 Hz RX detection) control word in WORD[12:0]
    Ctc2 = 1,
    /// CDCSS 134.4 Hz baud control word in WORD[12:0]
    Cdcss1344 = 2,
}

#[bitenum()]
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Reg08CodeHigh {
    #[fallback]
    /// CDCSS Low 12 bits
    Low = 0,
    /// CDCSS High 12 bits
    High = 1,
}

#[bitenum()]
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Reg0BFskCrcPass {
    #[fallback]
    /// CRC Fail
    Fail = 0,
    /// CRC Pass
    Pass = 1,
}

#[bitenum()]
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Reg0CCtcssPhaseShift {
    #[fallback]
    /// No phase shift
    None = 0,
    /// 120° phase shift
    Deg120 = 1,
    /// 180° phase shift
    Deg180 = 2,
    /// 240° phase shift
    Deg240 = 3,
}

#[bitenum()]
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Reg10LnaGainShort {
    #[fallback]
    /// -19 dB
    Minus19Db = 0,
    /// -16 dB
    Minus16Db = 1,
    /// -11 dB
    Minus11Db = 2,
    /// 0 dB
    ZeroDb = 3,
}

#[bitenum()]
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Reg11LnaGainShort {
    #[fallback]
    /// -19 dB
    Minus19Db = 0,
    /// -16 dB
    Minus16Db = 1,
    /// -11 dB
    Minus11Db = 2,
    /// 0 dB
    ZeroDb = 3,
}

#[bitenum()]
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Reg12LnaGainShort {
    #[fallback]
    /// -19 dB
    Minus19Db = 0,
    /// -16 dB
    Minus16Db = 1,
    /// -11 dB
    Minus11Db = 2,
    /// 0 dB
    ZeroDb = 3,
}

#[bitenum()]
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Reg13LnaGainShort {
    #[fallback]
    /// -19 dB
    Minus19Db = 0,
    /// -16 dB
    Minus16Db = 1,
    /// -11 dB
    Minus11Db = 2,
    /// 0 dB
    ZeroDb = 3,
}

#[bitenum()]
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Reg14LnaGainShort {
    #[fallback]
    /// -19 dB
    Minus19Db = 0,
    /// -16 dB
    Minus16Db = 1,
    /// -11 dB
    Minus11Db = 2,
    /// 0 dB
    ZeroDb = 3,
}

#[bitenum()]
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Reg34Gpio4Type {
    #[fallback]
    /// High/Low
    HighLow = 0,
    /// Interrupt
    Interrupt = 1,
    /// Squelch
    Squelch = 2,
    /// VoX
    Vox = 3,
    /// CTCSS/CDCSS Compared Result
    CtcssCdcssResult = 4,
    /// CTCSS Compared Result
    CtcssResult = 5,
    /// CDCSS Compared Result
    CdcssResult = 6,
    /// Tail Detected Result
    TailDetected = 7,
    /// DTMF/5Tone Symbol Received Flag
    Dtmf5toneFlag = 8,
    /// CTCSS/CDCSS Digital Wave
    CtcssCdcssWave = 9,
}

#[bitenum()]
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Reg34Gpio5Type {
    #[fallback]
    /// High/Low
    HighLow = 0,
    /// Interrupt
    Interrupt = 1,
    /// Squelch
    Squelch = 2,
    /// VoX
    Vox = 3,
    /// CTCSS/CDCSS Compared Result
    CtcssCdcssResult = 4,
    /// CTCSS Compared Result
    CtcssResult = 5,
    /// CDCSS Compared Result
    CdcssResult = 6,
    /// Tail Detected Result
    TailDetected = 7,
    /// DTMF/5Tone Symbol Received Flag
    Dtmf5toneFlag = 8,
    /// CTCSS/CDCSS Digital Wave
    CtcssCdcssWave = 9,
}

#[bitenum()]
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Reg34Gpio6Type {
    #[fallback]
    /// High/Low
    HighLow = 0,
    /// Interrupt
    Interrupt = 1,
    /// Squelch
    Squelch = 2,
    /// VoX
    Vox = 3,
    /// CTCSS/CDCSS Compared Result
    CtcssCdcssResult = 4,
    /// CTCSS Compared Result
    CtcssResult = 5,
    /// CDCSS Compared Result
    CdcssResult = 6,
    /// Tail Detected Result
    TailDetected = 7,
    /// DTMF/5Tone Symbol Received Flag
    Dtmf5toneFlag = 8,
    /// CTCSS/CDCSS Digital Wave
    CtcssCdcssWave = 9,
}

#[bitenum()]
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Reg35Gpio0Type {
    #[fallback]
    /// High/Low
    HighLow = 0,
    /// Interrupt
    Interrupt = 1,
    /// Squelch
    Squelch = 2,
    /// VoX
    Vox = 3,
    /// CTCSS/CDCSS Compared Result
    CtcssCdcssResult = 4,
    /// CTCSS Compared Result
    CtcssResult = 5,
    /// CDCSS Compared Result
    CdcssResult = 6,
    /// Tail Detected Result
    TailDetected = 7,
    /// DTMF/5Tone Symbol Received Flag
    Dtmf5toneFlag = 8,
    /// CTCSS/CDCSS Digital Wave
    CtcssCdcssWave = 9,
}

#[bitenum()]
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Reg35Gpio1Type {
    #[fallback]
    /// High/Low
    HighLow = 0,
    /// Interrupt
    Interrupt = 1,
    /// Squelch
    Squelch = 2,
    /// VoX
    Vox = 3,
    /// CTCSS/CDCSS Compared Result
    CtcssCdcssResult = 4,
    /// CTCSS Compared Result
    CtcssResult = 5,
    /// CDCSS Compared Result
    CdcssResult = 6,
    /// Tail Detected Result
    TailDetected = 7,
    /// DTMF/5Tone Symbol Received Flag
    Dtmf5toneFlag = 8,
    /// CTCSS/CDCSS Digital Wave
    CtcssCdcssWave = 9,
}

#[bitenum()]
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Reg35Gpio2Type {
    #[fallback]
    /// High/Low
    HighLow = 0,
    /// Interrupt
    Interrupt = 1,
    /// Squelch
    Squelch = 2,
    /// VoX
    Vox = 3,
    /// CTCSS/CDCSS Compared Result
    CtcssCdcssResult = 4,
    /// CTCSS Compared Result
    CtcssResult = 5,
    /// CDCSS Compared Result
    CdcssResult = 6,
    /// Tail Detected Result
    TailDetected = 7,
    /// DTMF/5Tone Symbol Received Flag
    Dtmf5toneFlag = 8,
    /// CTCSS/CDCSS Digital Wave
    CtcssCdcssWave = 9,
}

#[bitenum()]
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Reg35Gpio3Type {
    #[fallback]
    /// High/Low
    HighLow = 0,
    /// Interrupt
    Interrupt = 1,
    /// Squelch
    Squelch = 2,
    /// VoX
    Vox = 3,
    /// CTCSS/CDCSS Compared Result
    CtcssCdcssResult = 4,
    /// CTCSS Compared Result
    CtcssResult = 5,
    /// CDCSS Compared Result
    CdcssResult = 6,
    /// Tail Detected Result
    TailDetected = 7,
    /// DTMF/5Tone Symbol Received Flag
    Dtmf5toneFlag = 8,
    /// CTCSS/CDCSS Digital Wave
    CtcssCdcssWave = 9,
}

#[bitenum()]
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Reg3CXtalMode {
    #[fallback]
    /// ~13MHz
    Mhz13 = 0,
    /// ~19.2MHz
    Mhz192 = 1,
    /// ~26MHz
    Mhz26 = 2,
    /// ~38.4MHz
    Mhz384 = 3,
}

#[bitenum()]
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Reg47AfOutSel {
    #[fallback]
    /// Mute
    Mute = 0,
    /// Normal AF out
    Normal = 1,
    /// Tone out for Rx (enable Tone1)
    ToneRx = 2,
    /// Beep out for Tx (enable Tone1 and REG_03[9]=1)
    BeepTx = 3,
    /// CTCSS/CDCSS out for Rx test
    CtcssCdcssRxTest = 6,
    /// FSK out for Rx test
    FskRxTest = 8,
}

#[bitenum()]
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Reg51CodeHigh {
    #[fallback]
    /// Low 12 bits
    Low = 0,
    /// High 12 bits
    High = 1,
}

#[bitenum()]
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Reg52TailMode {
    #[fallback]
    /// 134.4Hz CTCSS tail when CDCSS mode
    Tail1344 = 0,
    /// CTCSS 120° phase shift
    Shift120 = 1,
    /// CTCSS 180° phase shift
    Shift180 = 2,
    /// CTCSS 240° phase shift
    Shift240 = 3,
}

#[bitenum()]
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Reg63Glitch {
    #[fallback]
    /// Glitch indicator.
    Glitch = 0,
}

#[bitenum()]
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Reg70Tone1En {
    #[fallback]
    /// Enable TONE1.
    Tone1En = 0,
}

#[bitenum()]
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Reg78ThSq1 {
    #[fallback]
    /// RSSI threshold for Squelch=1 (0.5dB/step).
    ThSq1 = 0,
}

#[bitenum()]
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Reg79VoxInterval {
    #[fallback]
    /// VoX Detection Interval Time.
    VoxInterval = 0,
}

#[bitenum()]
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Reg7AVox0Delay {
    #[fallback]
    /// VOX=0 Detection delay (multiplier *128ms).
    Vox0Delay = 0,
}

/// REG_00: Control register 0: soft reset.
#[bitfield(u16)]
pub struct Reg00 {
    #[bits(15)]
    __: u16,
    /// Soft Reset. 1=Reset; 0=Normal.
    #[bits(1)]
    pub soft_reset: Reg00SoftReset,
}

/// REG_02: Interrupt/status flags (read-only).
#[bitfield(u16)]
pub struct Reg02 {
    __0: bool,
    /// FSK Rx Sync Interrupt.
    pub fsk_rx_sync: bool,
    /// Squelch Lost Interrupt.
    pub squelch_lost: bool,
    /// Squelch Found Interrupt.
    pub squelch_found: bool,
    /// VoX Lost Interrupt.
    pub vox_lost: bool,
    /// VoX Found Interrupt.
    pub vox_found: bool,
    /// CTCSS Lost Interrupt.
    pub ctcss_lost: bool,
    /// CTCSS Found Interrupt.
    pub ctcss_found: bool,
    /// CDCSS Lost Interrupt.
    pub cdcss_lost: bool,
    /// CDCSS Found Interrupt.
    pub cdcss_found: bool,
    /// CTCSS/CDCSS Tail Found Interrupt.
    pub ctcss_cdcss_tail_found: bool,
    /// DTMF/5TONE Found Interrupt.
    pub dtmf_5tone_found: bool,
    /// FSK FIFO Almost Full Interrupt.
    pub fsk_fifo_almost_full: bool,
    /// FSK Rx Finished Interrupt.
    pub fsk_rx_finished: bool,
    /// FSK FIFO Almost Empty Interrupt.
    pub fsk_fifo_almost_empty: bool,
    /// FSK Tx Finished Interrupt.
    pub fsk_tx_finished: bool,
}

/// REG_07: CTCSS/CTC2/CDCSS control word (mode + 13-bit word).
#[bitfield(u16)]
pub struct Reg07 {
    /// Frequency/baud control word.
    #[bits(13)]
    pub word: u16,
    /// Mode select (best-effort, see datasheet).
    #[bits(3)]
    pub mode: Reg07Mode,
}

/// REG_08: CDCSS code high/low 12-bit selector + code.
#[bitfield(u16)]
pub struct Reg08 {
    /// CDCSS high/low 12-bit code.
    #[bits(12)]
    pub code: u16,
    #[bits(3)]
    __0: u8,
    /// 1=CDCSS high 12-bit, 0=CDCSS low 12-bit.
    #[bits(1)]
    pub code_high: Reg08CodeHigh,
}

/// REG_09: DTMF/SelCall Symbol Coefficient for Detection.
#[bitfield(u16)]
pub struct Reg09 {
    /// Coefficient.
    #[bits(8)]
    pub coefficient: u8,
    #[bits(4)]
    __0: u8,
    /// Symbol Number.
    #[bits(4)]
    pub symbol_number: u8,
}

/// REG_0A: GPIO input indicator (read-only).
#[bitfield(u16)]
pub struct Reg0A {
    /// GPIOs Input Indicator. 1=High; 0=Low.
    #[bits(8)]
    pub gpio_in: u8,
    #[bits(8)]
    __0: u8,
}

/// REG_0B: DTMF/5Tone received + FSK sync/CRC indicators (read-only).
#[bitfield(u16)]
pub struct Reg0B {
    #[bits(4)]
    __0: u8,
    /// FSK Rx CRC Indicator. 1=Pass; 0=Fail.
    #[bits(1)]
    pub fsk_crc_pass: Reg0BFskCrcPass,
    #[bits(1)]
    __1: bool,
    /// FSK Rx Sync Positive has been found.
    pub fsk_sync_pos: bool,
    /// FSK Rx Sync Negative has been found.
    pub fsk_sync_neg: bool,
    /// DTMF/5Tone Code Received.
    #[bits(4)]
    pub dtmf_5tone_code: u8,
    #[bits(4)]
    __3: u8,
}

/// REG_0C: CTCSS/CDCSS/VoX/Squelch/IRQ indicators (read-only).
#[bitfield(u16)]
pub struct Reg0C {
    /// Interrupt Indicator. 1=IRQ request.
    pub irq: bool,
    /// Squelch result output. 1=Link; 0=Loss.
    pub squelch: bool,
    /// VoX Indicator.
    pub vox: bool,
    #[bits(7)]
    __0: u8,
    /// CTC1 received.
    pub ctc1_received: bool,
    /// CTC2 (55 Hz) received.
    pub ctc2_received: bool,
    /// CTCSS Phase Shift Received.
    #[bits(2)]
    pub ctcss_phase_shift: Reg0CCtcssPhaseShift,
    /// CDCSS positive code received.
    pub cdcss_pos: bool,
    /// CDCSS negative code received.
    pub cdcss_neg: bool,
}

/// REG_0D: Frequency scan indicator + high bits (read-only).
#[bitfield(u16)]
pub struct Reg0D {
    /// Frequency Scan High bits (see datasheet).
    #[bits(11)]
    pub scan_hi: u16,
    #[bits(4)]
    __0: u8,
    /// Frequency Scan Indicator. 1=Busy; 0=Finished.
    pub scan_busy: bool,
}

/// REG_0E: Frequency scan low 16 bits (read-only).
#[bitfield(u16)]
pub struct Reg0E {
    /// Frequency Scan Low 16 bits.
    #[bits(16)]
    pub scan_lo: u16,
}

/// REG_10: Rx AGC Gain Table[0] entry.
#[bitfield(u16)]
pub struct Reg10 {
    /// PGA Gain.
    #[bits(3)]
    pub pga_gain: u8,
    /// MIXER Gain.
    #[bits(2)]
    pub mixer_gain: u8,
    /// LNA Gain.
    #[bits(3)]
    pub lna_gain: u8,
    /// LNA Gain Short.
    #[bits(2)]
    pub lna_gain_short: Reg10LnaGainShort,
    #[bits(6)]
    __0: u8,
}

/// REG_11: Rx AGC Gain Table[1] entry.
#[bitfield(u16)]
pub struct Reg11 {
    /// PGA Gain.
    #[bits(3)]
    pub pga_gain: u8,
    /// MIXER Gain.
    #[bits(2)]
    pub mixer_gain: u8,
    /// LNA Gain.
    #[bits(3)]
    pub lna_gain: u8,
    /// LNA Gain Short.
    #[bits(2)]
    pub lna_gain_short: Reg11LnaGainShort,
    #[bits(6)]
    __0: u8,
}

/// REG_12: Rx AGC Gain Table[2] entry.
#[bitfield(u16)]
pub struct Reg12 {
    /// PGA Gain.
    #[bits(3)]
    pub pga_gain: u8,
    /// MIXER Gain.
    #[bits(2)]
    pub mixer_gain: u8,
    /// LNA Gain.
    #[bits(3)]
    pub lna_gain: u8,
    /// LNA Gain Short.
    #[bits(2)]
    pub lna_gain_short: Reg12LnaGainShort,
    #[bits(6)]
    __0: u8,
}

/// REG_13: Rx AGC Gain Table[3] entry.
#[bitfield(u16)]
pub struct Reg13 {
    /// PGA Gain.
    #[bits(3)]
    pub pga_gain: u8,
    /// MIXER Gain.
    #[bits(2)]
    pub mixer_gain: u8,
    /// LNA Gain.
    #[bits(3)]
    pub lna_gain: u8,
    /// LNA Gain Short.
    #[bits(2)]
    pub lna_gain_short: Reg13LnaGainShort,
    #[bits(6)]
    __0: u8,
}

/// REG_14: Rx AGC Gain Table[4] entry.
#[bitfield(u16)]
pub struct Reg14 {
    /// PGA Gain.
    #[bits(3)]
    pub pga_gain: u8,
    /// MIXER Gain.
    #[bits(2)]
    pub mixer_gain: u8,
    /// LNA Gain.
    #[bits(3)]
    pub lna_gain: u8,
    /// LNA Gain Short.
    #[bits(2)]
    pub lna_gain_short: Reg14LnaGainShort,
    #[bits(6)]
    __0: u8,
}

/// REG_19: MIC AGC control.
#[bitfield(u16)]
pub struct Reg19 {
    #[bits(15)]
    pub undocumented_1: u16,
    /// Automatic MIC PGA Gain Controller Disable. 1=Disable; 0=Enable.
    pub mic_agc_disable: bool,
}

/// REG_1A: Crystal regulator settings.
#[bitfield(u16)]
pub struct Reg1A {
    #[bits(8)]
    __0: u8,
    /// Crystal iBit.
    #[bits(4)]
    pub xtal_ibit: u8,
    /// Crystal vReg Bit.
    #[bits(4)]
    pub xtal_vreg_bit: u8,
}

/// REG_1F: PLL charge pump setting.
#[bitfield(u16)]
pub struct Reg1F {
    /// PLL CP bit.
    #[bits(4)]
    pub pll_cp_bit: u8,
    #[bits(12)]
    pub undocumented: u16,
}

/// REG_24: DTMF/SelCall detection control.
#[bitfield(u16)]
pub struct Reg24 {
    /// Max Symbol Number for SelCall Detection.
    #[bits(4)]
    pub max_symbol_num: u8,
    /// 1=DTMF; 0=SelCall.
    pub dtmf_mode: bool,
    /// DTMF/SelCall Enable. 1=Enable; 0=Disable.
    pub dtmf_selcall_en: bool,
    #[bits(10)]
    __0: u16,
}

/// REG_28: AF Rx expander settings.
#[bitfield(u16)]
pub struct Reg28 {
    /// Noise point (dB).
    #[bits(7)]
    pub expand_noise_point: u8,
    /// 0 dB point (dB).
    #[bits(7)]
    pub expand_0db_point: u8,
    /// Expander ratio. 00=Disable; 01=1:2; 10=1:3; 11=1:4
    #[bits(2)]
    pub expand_ratio: u8,
}

/// REG_29: AF Tx compressor settings.
#[bitfield(u16)]
pub struct Reg29 {
    /// Noise point (dB).
    #[bits(7)]
    pub compress_noise_point: u8,
    /// 0 dB point (dB).
    #[bits(7)]
    pub compress_0db_point: u8,
    /// Compressor ratio. 00=Disable; 01=1.333:1; 10=2:1; 11=4:1
    #[bits(2)]
    pub compress_ratio: u8,
}

/// REG_2B: AF Tx/Rx filter enable/disable bits (1=Disable).
#[bitfield(u16)]
pub struct Reg2B {
    /// Disable AF Tx pre-emphasis filter. 0=Enable;1=Disable.
    pub aftx_preemp_disable: bool,
    /// Disable AF Tx LPF1 filter. 0=Enable;1=Disable.
    pub aftx_lpf1_disable: bool,
    /// Disable AF Tx HPF 300 filter. 0=Enable;1=Disable.
    pub aftx_hpf300_disable: bool,
    #[bits(5)]
    __0: u8,
    /// Disable AF Rx de-emphasis filter. 0=Enable;1=Disable.
    pub afrx_deemp_disable: bool,
    /// Disable AF Rx LPF 3K filter. 0=Enable;1=Disable.
    pub afrx_lpf3k_disable: bool,
    /// Disable AF Rx HPF 300 filter. 0=Enable;1=Disable.
    pub afrx_hpf300_disable: bool,
    #[bits(5)]
    __1: u8,
}

/// REG_2E: CTCSS/CDCSS Tx Gain2 tuning (after Gain1).
#[bitfield(u16)]
pub struct Reg2E {
    #[bits(8)]
    __0: u8,
    /// CTCSS/CDCSS Tx Gain2 Tuning. 00=12dB;01=6dB;10=0dB;11=-6dB
    #[bits(2)]
    pub ctc_gain2: u8,
    #[bits(6)]
    __1: u8,
}

/// REG_30: Top-level enable bits.
#[bitfield(u16)]
pub struct Reg30 {
    /// Rx DSP Enable.
    pub rx_dsp_en: bool,
    /// Tx DSP Enable.
    pub tx_dsp_en: bool,
    /// MIC ADC Enable.
    pub mic_adc_en: bool,
    /// PA Gain Enable.
    pub pa_gain_en: bool,
    /// PLL/VCO Enable. 1111=Enable;0000=Disable.
    #[bits(4)]
    pub pll_vco_en: u8,
    /// DISC Mode Disable. 1=Disable;0=Enable.
    pub disc_mode_disable: bool,
    /// AF DAC Enable.
    pub af_dac_en: bool,
    /// Rx Link Enable (LNA/MIXER/PGA/ADC). 1111=Enable;0000=Disable.
    #[bits(4)]
    pub rx_link_en: u8,
    #[bits(1)]
    __0: bool,
    /// VCO Calibration Enable.
    pub vco_cal_en: bool,
}

/// REG_31: Compander / VOX / Scramble enables.
#[bitfield(u16)]
pub struct Reg31 {
    #[bits(1)]
    __0: bool,
    /// Enable Scramble Function.
    pub scramble_en: bool,
    /// Enable VOX detection.
    pub vox_en: bool,
    /// Enable Compander Function.
    pub compander_en: bool,
    #[bits(12)]
    __1: u16,
}

/// REG_32: Frequency scan control.
#[bitfield(u16)]
pub struct Reg32 {
    /// FrequencyScan Enable.
    pub scan_en: bool,
    #[bits(13)]
    __0: u16,
    /// FrequencyScan Time. 00=0.2s;01=0.4s;10=0.8s;11=1.6s
    #[bits(2)]
    pub scan_time: u8,
}

/// REG_33: GPIO output control.
#[bitfield(u16)]
pub struct Reg33 {
    /// GPIOs Output Value (when enabled).
    #[bits(8)]
    pub gpio_out_value: u8,
    /// GPIOs Output Disable. 1=Disable;0=Enable.
    #[bits(8)]
    pub gpio_out_disable: u8,
}

/// REG_34: GPIO4/5/6 output type selection.
#[bitfield(u16)]
pub struct Reg34 {
    /// GPIO6 Output Type Selection.
    #[bits(4)]
    pub gpio6_type: Reg34Gpio6Type,

    #[bits(4)]
    pub undocumented_1: u8,
    /// GPIO5 Output Type Selection.
    #[bits(4)]
    pub gpio5_type: Reg34Gpio5Type,
    /// GPIO4 Output Type Selection.
    #[bits(4)]
    pub gpio4_type: Reg34Gpio4Type,
}

/// REG_35: GPIO0/1/2/3 output type selection.
#[bitfield(u16)]
pub struct Reg35 {
    /// GPIO3 Output Type Selection.
    #[bits(4)]
    pub gpio3_type: Reg35Gpio3Type,
    /// GPIO2 Output Type Selection.
    #[bits(4)]
    pub gpio2_type: Reg35Gpio2Type,
    /// GPIO1 Output Type Selection.
    #[bits(4)]
    pub gpio1_type: Reg35Gpio1Type,
    /// GPIO0 Output Type Selection.
    #[bits(4)]
    pub gpio0_type: Reg35Gpio0Type,
}

/// REG_36: PA control: bias output + gains.
#[bitfield(u16)]
pub struct Reg36 {
    /// PA Gain2 Tuning. 111(max)->000(min).
    #[bits(3)]
    pub pa_gain2: u8,
    /// PA Gain1 Tuning. 111(max)->000(min).
    #[bits(3)]
    pub pa_gain1: u8,
    #[bits(1)]
    __0: bool,
    /// Enable PACTL output; 0=Disable (0V).
    pub pa_ctl_output: bool,
    /// PA Bias output 0~3.2V (0x00=0V ... 0xFF=3.2V)
    #[bits(8)]
    pub pa_bias: u8,
}

/// REG_37: Power/LDO control and enables.
#[bitfield(u16)]
pub struct Reg37 {
    /// Band-Gap Enable.
    pub bg_en: bool,
    /// XTAL Enable.
    pub xtal_en: bool,
    /// DSP Enable.
    pub dsp_en: bool,
    #[bits(1)]
    pub undocumented_1: bool,
    /// PLL LDO Bypass. 1=Bypass,0=Enable
    pub pll_ldo_byp: bool,
    /// RF LDO Bypass. 1=Bypass,0=Enable
    pub rf_ldo_byp: bool,
    /// VCO LDO Bypass. 1=Bypass,0=Enable
    pub vco_ldo_byp: bool,
    /// ANA LDO Bypass. 1=Bypass,0=Enable
    pub ana_ldo_byp: bool,
    /// PLL LDO Selection. 1=2.7V,0=2.4V
    pub pll_ldo_sel: bool,
    /// RF LDO Selection. 1=2.7V,0=2.4V
    pub rf_ldo_sel: bool,
    /// VCO LDO Selection. 1=2.7V,0=2.4V
    pub vco_ldo_sel: bool,
    /// ANA LDO Selection. 1=2.7V,0=2.4V
    pub ana_ldo_sel: bool,
    /// DSP Voltage Setting.
    #[bits(3)]
    pub dsp_volt: u8,
    #[bits(1)]
    __1: bool,
}

/// REG_38: Frequency low 16 bits. Frequency(Hz)=(hi<<16 + lo)*10
#[bitfield(u16)]
pub struct Reg38 {
    /// Frequency low 16 bits.
    #[bits(16)]
    pub freq_lo: u16,
}

/// REG_39: Frequency high 16 bits. Frequency(Hz)=(hi<<16 + lo)*10
#[bitfield(u16)]
pub struct Reg39 {
    /// Frequency high 16 bits.
    #[bits(16)]
    pub freq_hi: u16,
}

/// REG_3B: Crystal frequency low 16 bits (LSB->5Hz).
#[bitfield(u16)]
pub struct Reg3B {
    /// Crystal Frequency Low 16 bits.
    #[bits(16)]
    pub xtal_freq_lo: u16,
}

/// REG_3C: Crystal frequency high 8 bits + mode selection.
#[bitfield(u16)]
pub struct Reg3C {
    #[bits(6)]
    __0: u8,
    /// Crystal Frequency Mode Selection.
    #[bits(2)]
    pub xtal_mode: Reg3CXtalMode,
    /// Crystal Frequency High 8 bits.
    #[bits(8)]
    pub xtal_freq_hi: u8,
}

/// REG_3D: IF selection coefficient.
#[bitfield(u16)]
pub struct Reg3D {
    /// IF selection coefficient (see datasheet mapping).
    #[bits(16)]
    pub if_coeff: u16,
}

/// REG_3E: Band Selection Threshold (~= VCO Max Frequency(Hz)/96/640).
#[bitfield(u16)]
pub struct Reg3E {
    /// Band selection threshold.
    #[bits(16)]
    pub band_thresh: u16,
}

/// REG_3F: Interrupt enable bits.
#[bitfield(u16)]
pub struct Reg3F {
    pub _undocumented: bool,
    /// FSK Rx Sync Interrupt Enable.
    pub fsk_rx_sync_en: bool,
    /// Squelch Lost Interrupt Enable.
    pub squelch_lost_en: bool,
    /// Squelch Found Interrupt Enable.
    pub squelch_found_en: bool,
    /// VoX Lost Interrupt Enable.
    pub vox_lost_en: bool,
    /// VoX Found Interrupt Enable.
    pub vox_found_en: bool,
    /// CTCSS Lost Interrupt Enable.
    pub ctcss_lost_en: bool,
    /// CTCSS Found Interrupt Enable.
    pub ctcss_found_en: bool,
    /// CDCSS Lost Interrupt Enable.
    pub cdcss_lost_en: bool,
    /// CDCSS Found Interrupt Enable.
    pub cdcss_found_en: bool,
    /// CTCSS/CDCSS Tail Found Interrupt Enable.
    pub ctcss_cdcss_tail_found_en: bool,
    /// DTMF/5TONE Found Interrupt Enable.
    pub dtmf_5tone_found_en: bool,
    /// FSK FIFO Almost Full Interrupt Enable.
    pub fsk_fifo_almost_full_en: bool,
    /// FSK Rx Finished Interrupt Enable.
    pub fsk_rx_finished_en: bool,
    /// FSK FIFO Almost Empty Interrupt Enable.
    pub fsk_fifo_almost_empty_en: bool,
    /// FSK Tx Finished Interrupt Enable.
    pub fsk_tx_finished_en: bool,
}

/// REG_40: RF Tx deviation control.
#[bitfield(u16)]
pub struct Reg40 {
    /// RF Tx Deviation Tuning (0=min;0xFFF=max).
    #[bits(12)]
    pub rf_tx_dev: u16,
    /// Enable RF Tx Deviation.
    pub rf_tx_dev_en: bool,
    #[bits(3)]
    __0: u8,
}

/// REG_43: RF/AF bandwidth and demod gain settings.
#[bitfield(u16)]
pub struct Reg43 {
    #[bits(2)]
    __0: u8,
    /// Gain after FM Demodulation. 1=6dB;0=0dB
    pub fm_demod_gain: bool,
    #[bits(1)]
    __: u8,
    /// BW Mode Selection. 00=12.5k;01=6.25k;10=25k/20k
    #[bits(2)]
    pub bw_mode: u8,
    /// AFTx LPF2 bandwidth (Apass=1dB).
    #[bits(3)]
    pub aftx_lpf2_bw: u8,
    /// RF filter bandwidth (Apass=0.1dB) when signal is weak.
    #[bits(3)]
    pub rf_bw_weak: u8,
    /// RF filter bandwidth (Apass=0.1dB) when signal is strong.
    #[bits(3)]
    pub rf_bw_strong: u8,
    #[bits(1)]
    __1: u8,
}

/// REG_44: 300Hz AF response coefficient for Tx (part 1).
#[bitfield(u16)]
pub struct Reg44 {
    /// Coefficient.
    #[bits(16)]
    pub coeff: u16,
}

/// REG_45: 300Hz AF response coefficient for Tx (part 2).
#[bitfield(u16)]
pub struct Reg45 {
    /// Coefficient.
    #[bits(16)]
    pub coeff: u16,
}

/// REG_46: Voice amplitude threshold for VOX=1 detect.
#[bitfield(u16)]
pub struct Reg46 {
    /// Threshold.
    #[bits(11)]
    pub vox1_thresh: u16,
    #[bits(5)]
    __0: u8,
}

/// REG_47: AF output selection / invert and Tx filter bypass.
#[bitfield(u16)]
pub struct Reg47 {
    /// AF Tx Filter Bypass All. 1=Bypass;0=Normal.
    pub aftx_filter_bypass_all: bool,
    #[bits(7)]
    __0: u8,
    /// AF Output Selection.
    #[bits(4)]
    pub af_out_sel: Reg47AfOutSel,
    #[bits(1)]
    __: u8,
    /// AF Output Inverse Mode. 1=Inverse.
    pub af_out_invert: bool,
    #[bits(2)]
    __: u8,
}

/// REG_48: AF Rx gain and DAC gain settings.
#[bitfield(u16)]
pub struct Reg48 {
    /// AF DAC Gain (after Gain1+Gain2). 1111=max;0000=min
    #[bits(4)]
    pub af_dac_gain: u8,
    /// AF Rx Gain2 (-26dB~5.5dB, 0.5dB/step). 0x00=mute
    #[bits(6)]
    pub afrx_gain2: u8,
    /// AF Rx Gain1. 00=0dB;01=-6dB;10=-12dB;11=-18dB
    #[bits(2)]
    pub afrx_gain1: u8,
    #[bits(4)]
    pub undocumented: u8,
}

/// REG_4B: AF level controller (ALC) control.
#[bitfield(u16)]
pub struct Reg4B {
    #[bits(5)]
    __0: u8,
    /// AF Level Controller (ALC) Disable. 1=Disable;0=Enable.
    pub alc_disable: bool,
    #[bits(10)]
    __1: u16,
}

/// REG_4D: Glitch threshold for Squelch=0.
#[bitfield(u16)]
pub struct Reg4D {
    /// Glitch threshold for Squelch=0.
    #[bits(8)]
    pub glitch_th0: u8,
    #[bits(8)]
    __0: u8,
}

/// REG_4E: Squelch delay + glitch threshold for Squelch=1.
#[bitfield(u16)]
pub struct Reg4E {
    /// Glitch threshold for Squelch=1.
    #[bits(8)]
    pub glitch_th1: u8,
    #[bits(1)]
    __: u8,
    /// Squelch=0 Delay Setting.
    #[bits(2)]
    pub sq0_delay: u8,
    /// Squelch=1 Delay Setting.
    #[bits(3)]
    pub sq1_delay: u8,
    #[bits(2)]
    __: u8,
}

/// REG_4F: Ex-noise thresholds for squelch.
#[bitfield(u16)]
pub struct Reg4F {
    /// Ex-noise threshold for Squelch=1.
    #[bits(7)]
    pub exnoise_th1: u8,
    #[bits(1)]
    __0: bool,
    /// Ex-noise threshold for Squelch=0.
    #[bits(7)]
    pub exnoise_th0: u8,
    #[bits(1)]
    __1: bool,
}

/// REG_50: AF Tx mute control.
#[bitfield(u16)]
pub struct Reg50 {
    #[bits(15)]
    __0: u16,
    /// Enable AF Tx Mute. 1=Mute;0=Normal.
    pub aftx_mute: bool,
}

/// REG_51: CTCSS/CDCSS Tx control.
#[bitfield(u16)]
pub struct Reg51 {
    /// CTCSS/CDCSS Tx Gain1 Tuning. 0=min;0x7F=max
    #[bits(7)]
    pub tx_gain1: u8,
    #[bits(1)]
    __: u8,
    /// Auto CTCSS BW Mode. 1=Disable;0=Enable
    pub auto_ctcss_bw_disable: bool,
    /// Auto CDCSS BW Mode. 1=Disable;0=Enable
    pub auto_cdcss_bw_disable: bool,
    /// 1050Hz Detection Mode. 1=1050/4 detect enable
    pub detect_1050: bool,
    /// CDCSS 24/23bit selection. 1=24bit;0=23bit
    pub cdcss_24bit: bool,
    /// Mode select. 1=CTCSS;0=CDCSS
    pub mode_ctcss: bool,
    /// Transmit negative CDCSS code. 1=neg;0=pos
    pub tx_cdcss_neg: bool,
    /// GPIO0 Input for CDCSS (BK4819v3). 1=GPIO0 input;0=normal
    pub gpio0_in_cdcss: bool,
    /// Enable Tx CTCSS/CDCSS.
    pub tx_ctc_en: bool,
}

/// REG_52: CTCSS/CTCSS-tail/CDCSS detection thresholds and phase tail options.
#[bitfield(u16)]
pub struct Reg52 {
    /// CTCSS lost detect threshold.
    #[bits(6)]
    pub ctcss_lost_th: u8,
    /// CTCSS found detect threshold.
    #[bits(6)]
    pub ctcss_found_th: u8,
    /// CTCSS Detection Threshold Mode. 1=~0.1%;0=0.1Hz
    pub detect_thresh_mode: bool,
    /// CTCSS tail mode selection (valid when TAIL_SHIFT_EN=1).
    #[bits(2)]
    pub tail_mode: Reg52TailMode,
    /// Enable 120/180/240 degree shift CTCSS or 134.4Hz tail when CDCSS mode.
    pub tail_shift_en: bool,
}

/// REG_54: 300Hz AF response coefficient for Rx (part 1).
#[bitfield(u16)]
pub struct Reg54 {
    /// Coefficient.
    #[bits(16)]
    pub coeff: u16,
}

/// REG_55: 300Hz AF response coefficient for Rx (part 2).
#[bitfield(u16)]
pub struct Reg55 {
    /// Coefficient.
    #[bits(16)]
    pub coeff: u16,
}

/// REG_58: FSK mode and enable.
#[bitfield(u16)]
pub struct Reg58 {
    /// FSK Enable.
    pub fsk_en: bool,
    /// FSK Rx Bandwidth Setting.
    #[bits(3)]
    pub fsk_rx_bw: u8,
    /// FSK Preamble Type Selection.
    #[bits(2)]
    pub preamble_type: u8,
    #[bits(2)]
    __0: u8,
    /// FSK Rx Gain.
    #[bits(2)]
    pub fsk_rx_gain: u8,
    /// FSK Rx Mode Selection.
    #[bits(3)]
    pub fsk_rx_mode: u8,
    /// FSK Tx Mode Selection.
    #[bits(3)]
    pub fsk_tx_mode: u8,
}

/// REG_59: FSK FIFO control and options.
#[bitfield(u16)]
pub struct Reg59 {
    #[bits(3)]
    __0: u8,
    /// FSK Sync Length Selection. 1=4 bytes;0=2 bytes.
    pub sync_len: bool,
    /// FSK Preamble Length Selection (0=1 byte ... 15=16 bytes).
    #[bits(4)]
    pub preamble_len: u8,
    #[bits(1)]
    __: u8,
    /// Invert FSK data when TX.
    pub inv_data_tx: bool,
    /// Invert FSK data when RX.
    pub inv_data_rx: bool,
    /// Enable FSK TX.
    pub fsk_tx_en: bool,
    /// Enable FSK RX.
    pub fsk_rx_en: bool,
    /// Enable FSK Scramble.
    pub scramble_en: bool,
    /// Clear RX FIFO, 1=clear.
    pub clr_rx_fifo: bool,
    /// Clear TX FIFO, 1=clear.
    pub clr_tx_fifo: bool,
}

/// REG_5A: FSK sync bytes 0 and 1.
#[bitfield(u16)]
pub struct Reg5A {
    /// FSK Sync Byte 1.
    #[bits(8)]
    pub sync1: u8,
    /// FSK Sync Byte 0.
    #[bits(8)]
    pub sync0: u8,
}

/// REG_5B: FSK sync bytes 2 and 3.
#[bitfield(u16)]
pub struct Reg5B {
    /// FSK Sync Byte 3.
    #[bits(8)]
    pub sync3: u8,
    /// FSK Sync Byte 2.
    #[bits(8)]
    pub sync2: u8,
}

/// REG_5C: FSK CRC option.
#[bitfield(u16)]
pub struct Reg5C {
    #[bits(6)]
    __0: u8,
    /// CRC Option Enable. 1=Enable;0=Disable.
    pub crc_en: bool,
    #[bits(9)]
    __1: u16,
}

/// REG_5D: FSK data length (11 bits total for BK4819v3).
#[bitfield(u16)]
pub struct Reg5D {
    #[bits(5)]
    __0: u8,
    /// FSK Data Length high 3 bits.
    #[bits(3)]
    pub len_hi: u8,
    /// FSK Data Length low 8 bits (example: 0x0F means 16 bytes).
    #[bits(8)]
    pub len_lo: u8,
}

/// REG_5E: FSK FIFO thresholds.
#[bitfield(u16)]
pub struct Reg5E {
    /// FSK Rx FIFO Almost Full Threshold (0..7 words).
    #[bits(3)]
    pub rx_af_th: u8,
    /// FSK Tx FIFO Almost Empty Threshold (0..127 words).
    #[bits(7)]
    pub tx_ae_th: u8,
    #[bits(6)]
    __0: u8,
}

/// REG_5F: FSK word input/output.
#[bitfield(u16)]
pub struct Reg5F {
    /// FSK Word Input/Output.
    #[bits(16)]
    pub word: u16,
}

/// REG_63: Glitch indicator (read-only).
#[bitfield(u16)]
pub struct Reg63 {
    /// Glitch indicator.
    #[bits(8)]
    pub glitch: u8,
    #[bits(8)]
    __0: u8,
}

/// REG_64: Voice amplitude out (read-only).
#[bitfield(u16)]
pub struct Reg64 {
    /// Voice amplitude output.
    #[bits(16)]
    pub voice_amp: u16,
}

/// REG_65: Ex-noise indicator (read-only).
#[bitfield(u16)]
pub struct Reg65 {
    /// Ex-noise indicator (dB/step).
    #[bits(7)]
    pub exnoise: u8,
    #[bits(9)]
    __0: u16,
}

/// REG_67: RSSI (read-only). dBm ~= REG/2 - 160.
#[bitfield(u16)]
pub struct Reg67 {
    /// RSSI value (0.5 dB/step).
    #[bits(9)]
    pub rssi: u16,
    #[bits(7)]
    __0: u8,
}

/// REG_68: CTCSS scan indicator+frequency (read-only).
#[bitfield(u16)]
pub struct Reg68 {
    /// CTCSS Frequency word.
    #[bits(13)]
    pub freq_word: u16,
    #[bits(2)]
    __0: u8,
    /// CTCSS Scan Indicator. 1=Busy;0=Found.
    pub busy: bool,
}

/// REG_69: CDCSS scan indicator (read-only).
#[bitfield(u16)]
pub struct Reg69 {
    /// CDCSS high 12 bits.
    #[bits(12)]
    pub code_hi: u16,
    #[bits(2)]
    __0: u8,
    /// 23/24-bit indicator (BK4819v3). 1=24bit;0=23bit.
    pub is_24bit: bool,
    /// CDCSS Scan Indicator. 1=Busy;0=Found.
    pub busy: bool,
}

/// REG_6A: CDCSS low 12 bits (read-only).
#[bitfield(u16)]
pub struct Reg6A {
    /// CDCSS low 12 bits.
    #[bits(12)]
    pub code_lo: u16,
    #[bits(4)]
    __0: u8,
}

/// REG_6F: AF Tx/Rx input amplitude (read-only).
#[bitfield(u16)]
pub struct Reg6F {
    /// AF Tx/Rx Input Amplitude (dB).
    #[bits(7)]
    pub af_amp_db: u8,
    #[bits(9)]
    __0: u16,
}

/// REG_70: Tone enable and gain.
#[bitfield(u16)]
pub struct Reg70 {
    /// TONE2/FSK tuning gain.
    #[bits(7)]
    pub tone2_gain: u8,
    /// Enable TONE2.
    pub tone2_en: bool,
    /// TONE1 tuning gain.
    #[bits(7)]
    pub tone1_gain: u8,
    /// Enable TONE1.
    pub tone1_en: bool,
}

/// REG_71: TONE1/Scramble frequency control word.
#[bitfield(u16)]
pub struct Reg71 {
    /// Frequency control word.
    #[bits(16)]
    pub word: u16,
}

/// REG_72: TONE2/FSK frequency control word.
#[bitfield(u16)]
pub struct Reg72 {
    /// Frequency control word.
    #[bits(16)]
    pub word: u16,
}

/// REG_73: Automatic Frequency Correction (AFC) settings.
#[bitfield(u16)]
pub struct Reg73 {
    #[bits(4)]
    __0: u8,
    /// Automatic Frequency Correction Disable. 1=Disable;0=Enable.
    pub afc_disable: bool,
    #[bits(6)]
    __1: u8,
    /// AFC Range Selection. 000=max;111=min
    #[bits(3)]
    pub afc_range: u8,
    #[bits(2)]
    __2: u8,
}

/// REG_74: 3000Hz AF response coefficient for Tx.
#[bitfield(u16)]
pub struct Reg74 {
    /// Coefficient.
    #[bits(16)]
    pub coeff: u16,
}

/// REG_75: 3000Hz AF response coefficient for Rx.
#[bitfield(u16)]
pub struct Reg75 {
    /// Coefficient.
    #[bits(16)]
    pub coeff: u16,
}

/// REG_78: RSSI thresholds for squelch.
#[bitfield(u16)]
pub struct Reg78 {
    /// RSSI threshold for Squelch=0 (0.5dB/step).
    #[bits(8)]
    pub th_sq0: u8,
    /// RSSI threshold for Squelch=1 (0.5dB/step).
    #[bits(8)]
    pub th_sq1: u8,
}

/// REG_79: VOX detection interval and VOX=0 threshold.
#[bitfield(u16)]
pub struct Reg79 {
    /// Voice Amplitude Threshold for VOX=0 detect.
    #[bits(11)]
    pub vox0_thresh: u16,
    /// VoX Detection Interval Time.
    #[bits(5)]
    pub vox_interval: u8,
}

/// REG_7A: VOX=0 detection delay (*128ms).
#[bitfield(u16)]
pub struct Reg7A {
    #[bits(12)]
    __0: u16,
    /// VOX=0 Detection delay (multiplier *128ms).
    #[bits(4)]
    pub vox0_delay: u8,
}

/// REG_7D: MIC sensitivity tuning.
#[bitfield(u16)]
pub struct Reg7D {
    /// MIC Sensitivity Tuning. 0=min;0x1F=max;0.5dB/step.
    #[bits(5)]
    pub mic_sens: u8,
    #[bits(11)]
    pub undocumented: u16,
}

/// REG_7E: AGC and DC filter bandwidth settings.
#[bitfield(u16)]
pub struct Reg7E {
    /// DC Filter Bandwidth for Rx (IF in). 000=Bypass.
    #[bits(3)]
    pub dcf_bw_rx: u8,
    /// DC Filter Bandwidth for Tx (MIC in). 000=Bypass.
    #[bits(3)]
    pub dcf_bw_tx: u8,
    #[bits(6)]
    __0: u8,
    /// AGC Fix Index. 011=max ... 100=min.
    #[bits(3)]
    pub agc_fix_index: u8,
    /// AGC Fix Mode. 1=Fix;0=Auto.
    pub agc_fix_mode: bool,
}

/// Compatibility alias for the initial WIP code in this file.
pub type Mode = Reg07Mode;
