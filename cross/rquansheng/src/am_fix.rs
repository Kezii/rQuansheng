use crate::{bk4819::Bk4819Driver, bk_common::BkCommonBus};

#[derive(Copy, Clone)]
struct AmFixGainEntry {
    reg13: u16,
    gain_db: i8,
}

/// Minimal port of `uv-k5-firmware-custom/am_fix.c`.
///
/// This is a single-VFO simplification: it tracks one active RX frequency.
pub struct AmFix {
    enabled: bool,
    gain_table_index: usize,
    prev_rssi: i16,
    hold_counter_10ms: u8,
    last_freq_hz: u32,
    current_gain_diff_db: i8,
}

impl Default for AmFix {
    fn default() -> Self {
        Self {
            enabled: false,
            gain_table_index: 0,
            prev_rssi: 0,
            hold_counter_10ms: 0,
            last_freq_hz: 0,
            current_gain_diff_db: 0,
        }
    }
}

impl AmFix {
    // -89dBm expressed in BK4819 RSSI units (0.5 dB/step, offset 160 dB).
    const DESIRED_RSSI: i16 = (-89 + 160) * 2;

    // Lookup table copied from the reference firmware (REG_13 values + nominal dB).
    const GAIN_TABLE: &'static [AmFixGainEntry] = &[
        AmFixGainEntry {
            reg13: 0x03BE,
            gain_db: -7,
        }, // original QS setting
        AmFixGainEntry {
            reg13: 0x0000,
            gain_db: -93,
        },
        AmFixGainEntry {
            reg13: 0x0008,
            gain_db: -91,
        },
        AmFixGainEntry {
            reg13: 0x0010,
            gain_db: -88,
        },
        AmFixGainEntry {
            reg13: 0x0001,
            gain_db: -87,
        },
        AmFixGainEntry {
            reg13: 0x0009,
            gain_db: -85,
        },
        AmFixGainEntry {
            reg13: 0x0011,
            gain_db: -82,
        },
        AmFixGainEntry {
            reg13: 0x0002,
            gain_db: -81,
        },
        AmFixGainEntry {
            reg13: 0x000A,
            gain_db: -79,
        },
        AmFixGainEntry {
            reg13: 0x0012,
            gain_db: -76,
        },
        AmFixGainEntry {
            reg13: 0x0003,
            gain_db: -75,
        },
        AmFixGainEntry {
            reg13: 0x000B,
            gain_db: -73,
        },
        AmFixGainEntry {
            reg13: 0x0013,
            gain_db: -70,
        },
        AmFixGainEntry {
            reg13: 0x0004,
            gain_db: -69,
        },
        AmFixGainEntry {
            reg13: 0x000C,
            gain_db: -67,
        },
        AmFixGainEntry {
            reg13: 0x000D,
            gain_db: -64,
        },
        AmFixGainEntry {
            reg13: 0x001C,
            gain_db: -61,
        },
        AmFixGainEntry {
            reg13: 0x001D,
            gain_db: -58,
        },
        AmFixGainEntry {
            reg13: 0x001E,
            gain_db: -55,
        },
        AmFixGainEntry {
            reg13: 0x001F,
            gain_db: -52,
        },
        AmFixGainEntry {
            reg13: 0x003E,
            gain_db: -50,
        },
        AmFixGainEntry {
            reg13: 0x003F,
            gain_db: -47,
        },
        AmFixGainEntry {
            reg13: 0x005E,
            gain_db: -45,
        },
        AmFixGainEntry {
            reg13: 0x005F,
            gain_db: -42,
        },
        AmFixGainEntry {
            reg13: 0x007E,
            gain_db: -40,
        },
        AmFixGainEntry {
            reg13: 0x007F,
            gain_db: -37,
        },
        AmFixGainEntry {
            reg13: 0x009F,
            gain_db: -34,
        },
        AmFixGainEntry {
            reg13: 0x00BF,
            gain_db: -32,
        },
        AmFixGainEntry {
            reg13: 0x00DF,
            gain_db: -30,
        },
        AmFixGainEntry {
            reg13: 0x00FF,
            gain_db: -28,
        },
        AmFixGainEntry {
            reg13: 0x01DF,
            gain_db: -26,
        },
        AmFixGainEntry {
            reg13: 0x01FF,
            gain_db: -24,
        },
        AmFixGainEntry {
            reg13: 0x02BF,
            gain_db: -23,
        },
        AmFixGainEntry {
            reg13: 0x02DF,
            gain_db: -21,
        },
        AmFixGainEntry {
            reg13: 0x02FF,
            gain_db: -19,
        },
        AmFixGainEntry {
            reg13: 0x035E,
            gain_db: -17,
        },
        AmFixGainEntry {
            reg13: 0x035F,
            gain_db: -14,
        },
        AmFixGainEntry {
            reg13: 0x037E,
            gain_db: -12,
        },
        AmFixGainEntry {
            reg13: 0x037F,
            gain_db: -9,
        },
        AmFixGainEntry {
            reg13: 0x038F,
            gain_db: -6,
        },
        AmFixGainEntry {
            reg13: 0x03BF,
            gain_db: -4,
        },
        AmFixGainEntry {
            reg13: 0x03DF,
            gain_db: -2,
        },
        AmFixGainEntry {
            reg13: 0x03FF,
            gain_db: 0,
        },
    ];

    pub fn set_enabled(&mut self, enabled: bool, freq_hz: u32) {
        self.enabled = enabled;
        if enabled {
            self.reset(freq_hz);
        }
    }

    fn reset(&mut self, freq_hz: u32) {
        self.gain_table_index = 0;
        self.prev_rssi = 0;
        self.hold_counter_10ms = 0;
        self.last_freq_hz = freq_hz;
        self.current_gain_diff_db = 0;
    }

    pub fn tick<BUS: BkCommonBus>(
        &mut self,
        bk: &mut Bk4819Driver<BUS>,
        freq_hz: u32,
    ) -> Result<(), BUS::Error> {
        if !self.enabled {
            return Ok(());
        }

        if self.last_freq_hz != freq_hz {
            self.reset(freq_hz);
        }

        let new_rssi = bk.get_rssi()? as i16;
        let rssi = if self.prev_rssi > 0 {
            (self.prev_rssi + new_rssi) / 2
        } else {
            new_rssi
        };
        self.prev_rssi = new_rssi;

        if self.hold_counter_10ms > 0 {
            self.hold_counter_10ms -= 1;
        }

        // dB difference between actual and desired RSSI level
        let diff_db: i16 = (rssi - Self::DESIRED_RSSI) / 2;

        if diff_db > 0 {
            // decrease gain
            let mut index = self.gain_table_index;

            if diff_db >= 10 {
                // jump closer quickly, but keep 8 dB headroom (spike immunity)
                let desired_gain_db =
                    (Self::GAIN_TABLE[index].gain_db as i16) - diff_db + 8 /* immunity */;

                while index > 1 {
                    let next = index - 1;
                    if (Self::GAIN_TABLE[next].gain_db as i16) <= desired_gain_db {
                        index = next;
                        break;
                    }
                    index = next;
                }
            } else if index > 1 {
                // step down slowly for noise/spike immunity
                index -= 1;
            }

            index = index.max(1);

            if self.gain_table_index != index {
                self.gain_table_index = index;
                self.hold_counter_10ms = 30; // 300ms hold
            }
        }

        if diff_db >= -6 {
            // 6 dB hysteresis
            self.hold_counter_10ms = 30;
        }

        if self.hold_counter_10ms == 0 {
            let next = self.gain_table_index + 1;
            self.gain_table_index = next.min(Self::GAIN_TABLE.len() - 1);
        }

        let index = self.gain_table_index;
        self.current_gain_diff_db = Self::GAIN_TABLE[0].gain_db - Self::GAIN_TABLE[index].gain_db;

        bk.write_reg13_raw(Self::GAIN_TABLE[index].reg13)?;

        Ok(())
    }
}
