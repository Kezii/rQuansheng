pub mod gpio;

use dp32g030 as _;
pub use dp32g030::*;

pub mod interrupt {
    pub use dp32g030::Interrupt::*;
}
