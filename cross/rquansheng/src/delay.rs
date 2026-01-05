use core::cmp::min;

use cortex_m::asm;
use embedded_hal::delay::DelayNs;
use rtic_monotonics::fugit::Duration;

/// Simple busy-wait delay based on core clock.
///
/// This is used only for BK4819 bit-banging (short delays on the order of 1µs).
pub struct CycleDelay {
    cycles_per_us: u32,
}

impl CycleDelay {
    pub const fn new(cpu_hz: u32) -> Self {
        Self {
            cycles_per_us: cpu_hz / 1_000_000,
        }
    }

    #[inline(always)]
    fn delay_cycles(&mut self, cycles: u32) {
        // cortex-m busy loop; `0` is fine.
        asm::delay(cycles);
    }
}

impl DelayNs for CycleDelay {
    #[inline]
    fn delay_ns(&mut self, ns: u32) {
        // cycles = cpu_hz * ns / 1e9
        // Round up a bit to avoid being too fast.
        let cycles = ((self.cycles_per_us as u64) * (ns as u64)).div_ceil(1000);
        self.delay_cycles(min(cycles, u32::MAX as u64) as u32);
    }

    #[inline]
    fn delay_us(&mut self, us: u32) {
        self.delay_cycles(self.cycles_per_us.saturating_mul(us));
    }

    #[inline]
    fn delay_ms(&mut self, ms: u32) {
        self.delay_us(ms.saturating_mul(1_000));
    }
}
