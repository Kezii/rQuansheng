#[repr(i8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum FrequencyBand {
    BandNone = -1,
    Band1_50MHz = 0,
    Band2_108MHz,
    Band3_137MHz,
    Band4_174MHz,
    Band5_350MHz,
    Band6_400MHz,
    Band7_470MHz,
}

impl FrequencyBand {
    /// Port of the C firmware `frequencyBandTable` (QS original) but expressed in **Hz**.
    ///
    /// Note: the original C project stores frequency in **10 Hz** units; in this Rust project
    /// we generally pass around **Hz** (see `Bk4819Driver::set_frequency`).
    pub fn limits_hz(self) -> Option<(u32, u32)> {
        match self {
            Self::Band1_50MHz => Some((50_000_000, 76_000_000)),
            Self::Band2_108MHz => Some((108_000_000, 137_000_000)),
            Self::Band3_137MHz => Some((137_000_000, 174_000_000)),
            Self::Band4_174MHz => Some((174_000_000, 350_000_000)),
            Self::Band5_350MHz => Some((350_000_000, 400_000_000)),
            Self::Band6_400MHz => Some((400_000_000, 470_000_000)),
            Self::Band7_470MHz => Some((470_000_000, 600_000_000)),
            Self::BandNone => None,
        }
    }

    /// Port of the C firmware `FREQUENCY_GetBand()`.
    ///
    /// Returns `Band1_50MHz` as the fallback for out-of-range frequencies, matching the C code.
    pub fn from_frequency_hz(frequency_hz: u32) -> Self {
        // Reverse scan: pick the highest band whose lower limit is <= frequency.
        // This matches:
        //   for (band=BAND_N_ELEM-1; band>=0; band--)
        //     if (Frequency >= frequencyBandTable[band].lower) return band;
        let bands = [
            Self::Band1_50MHz,
            Self::Band2_108MHz,
            Self::Band3_137MHz,
            Self::Band4_174MHz,
            Self::Band5_350MHz,
            Self::Band6_400MHz,
            Self::Band7_470MHz,
        ];

        for &band in bands.iter().rev() {
            if let Some((lower, _upper)) = band.limits_hz() {
                if frequency_hz >= lower {
                    return band;
                }
            }
        }

        Self::Band1_50MHz
    }

    /// Index used by the stock/custom firmware calibration tables in EEPROM.
    ///
    /// This corresponds to the C enum order:
    /// `BAND1_50MHz=0 .. BAND7_470MHz=6`.
    pub const fn eeprom_index(self) -> u16 {
        match self {
            Self::Band1_50MHz => 0,
            Self::Band2_108MHz => 1,
            Self::Band3_137MHz => 2,
            Self::Band4_174MHz => 3,
            Self::Band5_350MHz => 4,
            Self::Band6_400MHz => 5,
            Self::Band7_470MHz => 6,
            Self::BandNone => 0,
        }
    }
}

/// Port of the C firmware `FREQUENCY_CalculateOutputPower(...)`.
///
/// Important: this mirrors the C math exactly (even if it looks unusual).
pub fn calculate_output_power_setting(
    txp_low: u8,
    txp_mid: u8,
    txp_high: u8,
    lower_limit_hz: u32,
    middle_hz: u32,
    upper_limit_hz: u32,
    frequency_hz: u32,
) -> u8 {
    let (txp_low, txp_mid, txp_high) = (txp_low as i32, txp_mid as i32, txp_high as i32);
    let (lower, middle, upper, freq) = (
        lower_limit_hz as i32,
        middle_hz as i32,
        upper_limit_hz as i32,
        frequency_hz as i32,
    );

    if freq <= lower {
        return txp_low as u8;
    }

    if upper <= freq {
        return txp_high as u8;
    }

    if freq <= middle {
        let mut txp_mid = txp_mid;
        txp_mid += ((txp_mid - txp_low) * (freq - lower)) / (middle - lower);
        return txp_mid.clamp(0, 255) as u8;
    }

    let mut txp_mid = txp_mid;
    txp_mid += ((txp_high - txp_mid) * (freq - middle)) / (upper - middle);
    txp_mid.clamp(0, 255) as u8
}
