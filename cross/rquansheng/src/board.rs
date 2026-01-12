use dp30g030_hal::{
    gpio::{Pin, Port},
    uart,
};

pub fn get_uart() -> uart::Uart1 {
    let p = unsafe { dp32g030::Peripherals::steal() };
    let uart1 = p.UART1;
    let syscon = p.SYSCON;
    let portcon = p.PORTCON;
    // UART example: UART1 on PA7 (TX) / PA8 (RX), 38400-8N1.
    let uart1_tx = uart::TxPin::<dp30g030_hal::UART1>::new(Pin::new(Port::A, 7)).unwrap();
    let uart1_rx = uart::RxPin::<dp30g030_hal::UART1>::new(Pin::new(Port::A, 8)).unwrap();
    let uart1_cfg = uart::Config::new(48_000_000, 38_400);
    let uart1: uart::Uart1 = uart::Uart::<
        dp30g030_hal::UART1,
        uart::TxPin<dp30g030_hal::UART1>,
        uart::RxPin<dp30g030_hal::UART1>,
    >::new(uart1, &syscon, &portcon, uart1_tx, uart1_rx, uart1_cfg)
    .unwrap();

    uart1
}
