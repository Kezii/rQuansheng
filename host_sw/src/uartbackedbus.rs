use std::{
    io::{self, Read},
    time::Duration,
};

use log::{error, info};
use rquansheng::{
    bk4819_bitbang::Bk4819Bus,
    messages::{HostBound, RadioBound, decode_line, encode_line},
};
use serialport::SerialPort;

pub struct SerialProtocolRadioBus {
    port: Box<dyn SerialPort>,
}

fn read_until_zero(port: &mut dyn Read, max_len: usize) -> io::Result<Vec<u8>> {
    let mut out = Vec::with_capacity(max_len.min(256));
    let mut byte = [0u8; 1];
    while out.len() < max_len {
        match port.read(&mut byte) {
            Ok(0) => continue,
            Ok(1) => {
                out.push(byte[0]);
                if byte[0] == 0 {
                    return Ok(out);
                }
            }
            Ok(_) => unreachable!("read(1 byte) returned >1"),
            Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
            Err(e) => return Err(e),
        }
    }
    Err(io::Error::new(
        io::ErrorKind::InvalidData,
        "UART line too long (missing '\\n')",
    ))
}

impl SerialProtocolRadioBus {
    pub fn open(path: &str, baud: u32, timeout: Duration) -> io::Result<Self> {
        let mut port = serialport::new(path, baud)
            .timeout(timeout)
            .open()
            .map_err(|e| io::Error::other(e.to_string()))?;
        port.set_timeout(timeout)
            .map_err(|e| io::Error::other(e.to_string()))?;
        Ok(Self { port })
    }

    fn send(&mut self, msg: &RadioBound) -> io::Result<()> {
        info!("send: {:?}", msg);
        let encoded = encode_line(msg)
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e.to_string()))?;
        self.port.write_all(&encoded)?;
        self.port.flush()?;
        Ok(())
    }

    fn recv_hostbound(&mut self) -> io::Result<HostBound> {
        let line = read_until_zero(&mut *self.port, 256)?;
        let decoded = decode_line::<HostBound>(&line)
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e.to_string()));

        info!("recv: {:?}", decoded);
        decoded
    }
}

impl Bk4819Bus for SerialProtocolRadioBus {
    type Error = io::Error;

    fn write_reg_raw(&mut self, reg: u8, value: u16) -> Result<(), Self::Error> {
        self.send(&RadioBound::WriteRegister(reg, value))?;

        if let Ok(HostBound::WriteAck(reg, value)) = self.recv_hostbound() {
            info!("WriteAck: 0x{:x} 0x{:x}", reg, value);
            return Ok(());
        } else {
            error!("no Ready reply");
        }

        Ok(())
    }

    fn read_reg_raw(&mut self, reg: u8) -> Result<u16, Self::Error> {
        self.send(&RadioBound::ReadRegister(reg))?;

        // Be tolerant: ignore unrelated replies (e.g. late Pong).
        for _ in 0..8 {
            match self.recv_hostbound()? {
                HostBound::Register(r, v) if r == reg => {
                    return Ok(v);
                }
                _ => continue,
            }
        }

        Err(io::Error::new(
            io::ErrorKind::TimedOut,
            format!("no Register reply for reg 0x{reg:02x}"),
        ))
    }

    fn write_reg<R: rquansheng::bk4819_n::Bk4819Register>(
        &mut self,
        reg: R,
    ) -> Result<(), Self::Error> {
        self.write_reg_raw(R::ADDRESS, reg.serialize())
    }

    fn read_reg<R: rquansheng::bk4819_n::Bk4819Register>(&mut self) -> Result<R, Self::Error> {
        let v = self.read_reg_raw(R::ADDRESS)?;
        Ok(R::deserialize(v))
    }
}
