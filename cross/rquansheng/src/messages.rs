use crc::CRC_32_ISCSI;
use embedded_io_async::Write;
use heapless::vec;
use serde::{Deserialize, Serialize};

const MAX_POSTCARD_LEN: usize = 64;
// COBS encoding overhead is <= ceil(n/254) bytes. We also append one 0x00 sentinel as frame delimiter.
pub const MAX_FRAME_LEN: usize = cobs::max_encoding_length(MAX_POSTCARD_LEN) + 1;

/// Encode a message as a single COBS frame terminated by `0x00`.
#[inline(never)]
pub fn encode_line<T: Serialize>(msg: &T) -> Result<vec::Vec<u8, MAX_FRAME_LEN>, postcard::Error> {
    let mut buf = [0u8; MAX_POSTCARD_LEN];
    let crc = crc::Crc::<u32>::new(&CRC_32_ISCSI);
    let used = postcard::to_slice_crc32(msg, &mut buf, crc.digest())?;

    let mut enc = [0u8; cobs::max_encoding_length(MAX_POSTCARD_LEN)];
    let enc_len = cobs::encode(used, &mut enc);

    let mut out = vec::Vec::<u8, MAX_FRAME_LEN>::new();
    out.extend_from_slice(&enc[..enc_len])
        .map_err(|_| postcard::Error::SerializeBufferFull)?;
    out.push(0)
        .map_err(|_| postcard::Error::SerializeBufferFull)?;
    Ok(out)
}

/// Decode a message from a COBS frame (optionally terminated by `0x00`, and tolerant of extra bytes after it).
#[inline(never)]
pub fn decode_line<T>(data: &[u8]) -> Result<T, postcard::Error>
where
    T: for<'de> Deserialize<'de>,
{
    let crc = crc::Crc::<u32>::new(&CRC_32_ISCSI);

    let mut decoded = [0u8; MAX_POSTCARD_LEN];
    let report =
        cobs::decode(data, &mut decoded).map_err(|_| postcard::Error::DeserializeBadEncoding)?;
    postcard::from_bytes_crc32(&decoded[..report.frame_size()], crc.digest())
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum RadioBound {
    Ping,
    WriteBk4819Register(u8, u16),
    ReadBk4819Register(u8),
    WriteBk1080Register(u8, u16),
    ReadBk1080Register(u8),
    /// Read a single byte from EEPROM at `address`.
    ReadEepromByte {
        address: u16,
    },
    SetAudioPath(bool),
    SetBacklight(bool),
    SetFlashlight(bool),
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum HostBound {
    Pong,
    Register(u8, u16),
    WriteAck(u8, u16),
    EepromByte { address: u16, value: u8 },
    Ready,
}

impl HostBound {
    pub async fn write<W: Write>(self, tx: &mut W) -> Result<(), W::Error> {
        if let Ok(encoded) = encode_line(&self) {
            tx.write_all(&encoded).await?;
        }
        Ok(())
    }
}
