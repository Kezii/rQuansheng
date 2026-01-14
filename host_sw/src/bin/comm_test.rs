use std::collections::HashMap;
use std::io::Write;
use std::sync::Arc;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;
use std::sync::mpsc;
use std::thread;
use std::time::Duration;

use host_sw::uartbackedbus::SerialProtocolRadioBus;
use host_sw::uartbackedbus::read_line_from_port;
use rquansheng::bk_common::BkCommonBus;
use rquansheng::bk4819::Reg37;
use rquansheng::messages::HostBound;
use rquansheng::messages::RadioBound;

fn main() {
    env_logger::init();

    let mut bus =
        SerialProtocolRadioBus::open("/dev/ttyUSB0", 38400, Duration::from_millis(100)).unwrap();

    let mut latency_hist = HashMap::new();

    // ctrl c handle

    let mut should_run = Arc::new(AtomicBool::new(true));
    let should_run_clone = should_run.clone();

    ctrlc::set_handler(move || {
        println!("received Ctrl+C!");
        should_run_clone.store(false, Ordering::Relaxed);
    })
    .expect("Error setting Ctrl-C handler");

    let mut now = std::time::Instant::now();
    loop {
        //let reg = Reg37::new().with_dsp_en(true).serialize();

        //bus.write_reg_raw(0x37, reg).unwrap();

        //bus.send(&RadioBound::Ping).unwrap();
        //assert_eq!(bus.recv_hostbound().unwrap(), HostBound::Pong);

        bus.read_reg_raw(0x37).unwrap();

        let elapsed = now.elapsed();

        latency_hist.insert(
            elapsed.as_millis(),
            latency_hist.get(&elapsed.as_millis()).unwrap_or(&0) + 1,
        );
        println!("messages per second: {}", 1.0 / elapsed.as_secs_f64());
        now = std::time::Instant::now();

        if !should_run.load(Ordering::Relaxed) {
            break;
        }
    }

    let mut as_vec = latency_hist.iter().collect::<Vec<_>>();

    as_vec.sort_by_key(|(latency, _)| *latency);

    for (latency, count) in as_vec {
        println!("latency: {}ms, count: {}", latency, count);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rquansheng::messages::RadioBound;
    use rquansheng::messages::decode_line;
    use rquansheng::messages::encode_line;

    #[test]
    fn test_encode_decode() {
        let ping = RadioBound::Ping;
        let ping_encoded = encode_line(&ping).unwrap();
        let ping_decoded = decode_line::<RadioBound>(&ping_encoded).unwrap();
        assert_eq!(ping_decoded, ping);

        let write_register = RadioBound::WriteBk4819Register(0x10, 0x20);
        let write_register_encoded = encode_line(&write_register).unwrap();
        let write_register_decoded = decode_line::<RadioBound>(&write_register_encoded).unwrap();
        assert_eq!(write_register_decoded, write_register);
    }
}
