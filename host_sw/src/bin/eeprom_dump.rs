use std::fs::File;
use std::io::Write;
use std::sync::mpsc;
use std::thread;
use std::time::Duration;

use host_sw::uartbackedbus::read_line_from_port;
use rquansheng::messages::HostBound;
use rquansheng::messages::RadioBound;
use rquansheng::messages::decode_line;
use rquansheng::messages::encode_line;

fn main() {
    let mut port = serialport::new("/dev/ttyUSB0", 38400)
        .timeout(Duration::from_millis(5000))
        .open()
        .expect("Failed to open port");

    let mut port_clone = port.try_clone().unwrap();

    let (tx, rx) = mpsc::channel();

    let _ = thread::spawn(move || {
        loop {
            let line = read_line_from_port(&mut port, 256).unwrap();
            let reply = decode_line::<HostBound>(&line).unwrap();
            println!("Reply: {:?}", reply);
            tx.send(reply).unwrap();
        }
    });

    let filename = "eeprom_dump.bin";
    let max_size = 0x2000;
    let mut file = File::create(filename).unwrap();

    for addr in 0..max_size {
        let ping = RadioBound::ReadEepromByte { address: addr };
        let ping_encoded = encode_line(&ping).unwrap();
        port_clone.write_all(&ping_encoded).unwrap();

        let reply = rx.recv().unwrap();
        let _ = rx.try_recv();

        if let HostBound::EepromByte { address, value } = reply {
            assert_eq!(address, addr);
            file.write_all(&[value]).unwrap();
        }

        std::thread::sleep(std::time::Duration::from_millis(1));
    }
}
