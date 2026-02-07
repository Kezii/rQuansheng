use embedded_hal::delay::DelayNs;
use heapless::Vec;

use crate::bk4819::{AfOutSel, Bk4819Driver};
use crate::bk_common::BkCommonBus;
use crate::radio::RadioController;

pub const SPECTRUM_BINS: usize = 128;

#[derive(Copy, Clone, Debug)]
pub struct SpectrumRequest {
    pub start_freq_hz: u32,
    pub step_hz: u32,
    pub settle_delay_us: u32,
    pub max_glitch_tries: u16,
}

impl Default for SpectrumRequest {
    fn default() -> Self {
        Self {
            start_freq_hz: 145_000_000,
            step_hz: 12_500,
            settle_delay_us: 200,
            max_glitch_tries: 50,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct SpectrumSample {
    pub freq_hz: u32,
    pub rssi: u16,
}

fn scan_bw_reg_for_step(step_hz: u32) -> u16 {
    match step_hz {
        10 | 100 | 500 | 1_000 | 2_500 => 0b0000_0000_0101_1000, // 6.25k
        5_000 => 0b0010_0100_0101_1000,                          // 6.25k
        6_250 => 0b0100_1000_0101_1000,                          // 6.25k
        8_330 | 10_000 => 0b0110_1100_0100_1000,                 // 6.25k
        12_500 | 15_000 | 20_000 => 0b0111_1111_0000_1000,       // 12.5k
        25_000 | 50_000 | 100_000 => 0b0011_0110_0010_1000,      // 25k
        _ => {
            if step_hz <= 12_500 {
                0b0111_1111_0000_1000
            } else {
                0b0011_0110_0010_1000
            }
        }
    }
}

fn wait_glitch_settle<BUS: BkCommonBus, D: DelayNs>(
    bk: &mut Bk4819Driver<BUS>,
    max_tries: u16,
    delay: &mut D,
) -> Result<(), BUS::Error> {
    for _ in 0..max_tries {
        let glitch = bk.get_glitch_indicator()?;
        if glitch < 255 {
            return Ok(());
        }
        delay.delay_us(100);
    }
    Ok(())
}

fn save_regs<BUS: BkCommonBus>(bk: &mut Bk4819Driver<BUS>) -> Result<[u16; 7], BUS::Error> {
    let regs = [0x30, 0x37, 0x3D, 0x43, 0x47, 0x48, 0x7E];
    let mut out = [0u16; 7];
    for (idx, reg) in regs.iter().enumerate() {
        out[idx] = bk.__internal_read_register_raw(*reg)?;
    }
    Ok(out)
}

fn restore_regs<BUS: BkCommonBus>(
    bk: &mut Bk4819Driver<BUS>,
    saved: &[u16; 7],
) -> Result<(), BUS::Error> {
    let regs = [0x30, 0x37, 0x3D, 0x43, 0x47, 0x48, 0x7E];
    for (idx, reg) in regs.iter().enumerate() {
        bk.__internal_write_register_raw(*reg, saved[idx])?;
    }
    Ok(())
}

pub fn fetch_spectrum_raw<BUS: BkCommonBus, D: DelayNs>(
    bk: &mut Bk4819Driver<BUS>,
    req: &SpectrumRequest,
    delay: &mut D,
) -> Result<Vec<SpectrumSample, SPECTRUM_BINS>, BUS::Error> {
    let saved = save_regs(bk)?;

    let scan_result = (|| {
        bk.set_af(AfOutSel::Mute)?;
        bk.rx_turn_on()?;

        let reg43_scan = scan_bw_reg_for_step(req.step_hz);
        bk.__internal_write_register_raw(0x43, reg43_scan)?;

        let reg30_work = bk.__internal_read_register_raw(0x30)?;

        let mut out = Vec::<SpectrumSample, SPECTRUM_BINS>::new();
        let mut freq = req.start_freq_hz;

        for _ in 0..SPECTRUM_BINS {
            bk.set_frequency(freq)?;
            //bk.pick_rx_filter_path_based_on_frequency(freq)?;

            bk.__internal_write_register_raw(0x30, 0)?;
            bk.__internal_write_register_raw(0x30, reg30_work)?;

            if req.settle_delay_us != 0 {
                delay.delay_us(req.settle_delay_us);
            }

            wait_glitch_settle(bk, req.max_glitch_tries, delay)?;
            let rssi = bk.get_rssi()?;

            let _ = out.push(SpectrumSample {
                freq_hz: freq,
                rssi,
            });
            freq = freq.saturating_add(req.step_hz);
        }

        Ok(out)
    })();

    restore_regs(bk, &saved)?;
    scan_result
}

impl<BUS, BUS1080, PLATFORM> RadioController<BUS, BUS1080, PLATFORM>
where
    BUS: BkCommonBus,
    BUS1080: BkCommonBus,
    PLATFORM: crate::radio_platform::RadioPlatform,
{
    pub fn fetch_spectrum<D: DelayNs>(
        &mut self,
        req: &SpectrumRequest,
        delay: &mut D,
    ) -> Result<Vec<SpectrumSample, SPECTRUM_BINS>, BUS::Error> {
        fetch_spectrum_raw(&mut self.bk, req, delay)
    }
}
