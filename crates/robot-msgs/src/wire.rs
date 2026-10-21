//! Framing for the Pi-to-desktop TCP link: a little-endian u32 length, then
//! a postcard-encoded body. The Pi sends `Stamped<Msg>`, the desktop sends
//! `Command`.

use serde::{Serialize, de::DeserializeOwned};

/// Refuse frames above this size so a corrupt length can't exhaust memory.
pub const MAX_FRAME_LEN: usize = 64 * 1024;

pub fn encode_frame<T: Serialize>(value: &T) -> postcard::Result<Vec<u8>> {
    let body = postcard::to_allocvec(value)?;
    let mut frame = Vec::with_capacity(4 + body.len());
    frame.extend_from_slice(&(body.len() as u32).to_le_bytes());
    frame.extend_from_slice(&body);
    Ok(frame)
}

pub fn decode_body<T: DeserializeOwned>(body: &[u8]) -> postcard::Result<T> {
    postcard::from_bytes(body)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Msg, Stamped, WheelPositions};

    #[test]
    fn frame_round_trips() {
        let msg = Stamped {
            t_ns: 42,
            seq: 7,
            msg: Msg::Wheels(WheelPositions {
                left_mm: -12,
                right_mm: 30,
            }),
        };
        let frame = encode_frame(&msg).unwrap();
        let len = u32::from_le_bytes(frame[..4].try_into().unwrap()) as usize;
        assert_eq!(len, frame.len() - 4);
        let back: Stamped<Msg> = decode_body(&frame[4..]).unwrap();
        assert_eq!(back, msg);
    }
}
