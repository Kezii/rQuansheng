#![no_main]
#![no_std]

use cortex_m_rt as _;
use defmt_serial as _;

use panic_probe as _;

// TODO(6) Import your HAL
pub mod bk4819;
pub mod bk4819_bitbang;
pub mod bk4819_n;
pub mod delay;
pub mod dialer;
pub mod display;
pub mod keyboard;
pub mod radio;

use dp32g030 as _;

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

/// Terminates the application and makes `probe-rs` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
