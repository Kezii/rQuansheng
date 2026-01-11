use heapless::vec;
use serde::{Deserialize, Serialize};

const MAX_POSTCARD_LEN: usize = 64;
// COBS encoding overhead is <= ceil(n/254) bytes. We also append one 0x00 sentinel as frame delimiter.
const MAX_FRAME_LEN: usize = cobs::max_encoding_length(MAX_POSTCARD_LEN) + 1;

/// Encode a message as a single COBS frame terminated by `0x00`.
pub fn encode_line<T: Serialize>(msg: &T) -> Result<vec::Vec<u8, MAX_FRAME_LEN>, postcard::Error> {
    let mut buf = [0u8; MAX_POSTCARD_LEN];
    let used = postcard::to_slice(msg, &mut buf)?;

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
pub fn decode_line<T>(data: &[u8]) -> Result<T, postcard::Error>
where
    T: for<'de> Deserialize<'de>,
{
    // Accept either a full frame terminated by 0x00 or a raw COBS payload without the terminator.
    // Also tolerate fixed-size buffers with extra bytes after the first 0x00.
    let mut start = 0usize;
    while start < data.len() && data[start] == 0 {
        start += 1;
    }
    let data = &data[start..];
    let end = match data.iter().position(|&b| b == 0) {
        Some(i) => (i + 1).min(data.len()),
        None => data.len(),
    };
    let frame = &data[..end];

    let mut decoded = [0u8; MAX_POSTCARD_LEN];
    let report =
        cobs::decode(frame, &mut decoded).map_err(|_| postcard::Error::DeserializeBadEncoding)?;
    postcard::from_bytes(&decoded[..report.frame_size()])
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
