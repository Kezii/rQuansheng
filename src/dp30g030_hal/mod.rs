pub mod adc;
pub mod gpio;
pub mod i2c;
pub mod uart;

use dp32g030 as _;
pub use dp32g030::*;

pub mod interrupt {
    pub use dp32g030::Interrupt::*;
}
