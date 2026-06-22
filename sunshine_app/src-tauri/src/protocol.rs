use crate::ffi::{SunshineInput, SunshineState};
use std::mem::size_of;

pub const FRAME_START: u8 = 0xAA;
pub const TYPE_TELEM_FRAME: u8 = 0x01;
pub const TYPE_CTRL_PACKET: u8 = 0x02;
pub const TYPE_STATUS:      u8 = 0x03;
pub const TYPE_HEARTBEAT:   u8 = 0x04;
pub const TYPE_RX_RSSI:     u8 = 0x05;

pub const STATUS_OK:                 u8 = 0x00;
pub const STATUS_BRAIN_CONNECTED:    u8 = 0x01;
pub const STATUS_BRAIN_DISCONNECTED: u8 = 0x02;

// Brain sends 20 inputs per ESP-NOW v2 packet @ 50 Hz (requires IDF >= 5.4 for the
// >250-byte payload). Layout: 2 (frame_id) + 1 (type) + 2×SunshineState
// (start + midpoint → 100 Hz real state) + 20×SunshineInput. No vars on the wire
// (host recomputes them). Derived from the structs so it can never drift.
pub const INPUTS_PER_FRAME:  usize = 20;
pub const ESPNOW_TELEM_SIZE: usize = 3 + 2 * size_of::<SunshineState>() + INPUTS_PER_FRAME * size_of::<SunshineInput>();
const MAX_USB_PAYLOAD_SIZE: usize = ESPNOW_TELEM_SIZE;

#[derive(Debug, Clone)]
pub struct TelemetryFrame {
    pub frame_id:  u16,
    pub state:     SunshineState,   // real filter state at the FIRST input of the frame
    pub state_mid: SunshineState,   // real filter state at the MIDPOINT input → 100 Hz real state
    pub inputs:    [SunshineInput; INPUTS_PER_FRAME],
}

#[derive(Debug)]
pub enum ReceiverFrame {
    Telemetry(TelemetryFrame),
    Status { code: u8, message: String },
    Heartbeat { timestamp_ms: u32 },
    RxRssi { rssi: i8 },
    /// Serial port dropped (device reset / flashing started).
    SerialLost,
    /// Serial port came back after a SerialLost (auto-reconnect succeeded).
    SerialReconnected,
}

pub struct FrameParser {
    state:        ParserState,
    frame_type:   u8,
    expected_len: u16,
    buf:          Vec<u8>,
    computed_cs:  u8,
}

#[derive(PartialEq)]
enum ParserState { Idle, Type, Len0, Len1, Payload, Checksum }

impl FrameParser {
    pub fn new() -> Self {
        FrameParser {
            state:        ParserState::Idle,
            frame_type:   0,
            expected_len: 0,
            buf:          Vec::with_capacity(ESPNOW_TELEM_SIZE),
            computed_cs:  0,
        }
    }

    pub fn feed(&mut self, byte: u8) -> Option<(u8, Vec<u8>)> {
        match self.state {
            ParserState::Idle => {
                if byte == FRAME_START { self.state = ParserState::Type; }
            }
            ParserState::Type => {
                self.frame_type = byte;
                self.state = ParserState::Len0;
            }
            ParserState::Len0 => {
                self.expected_len = byte as u16;
                self.state = ParserState::Len1;
            }
            ParserState::Len1 => {
                self.expected_len |= (byte as u16) << 8;
                self.buf.clear();
                self.computed_cs = 0;
                self.state = if self.expected_len == 0 {
                    ParserState::Checksum
                } else if self.expected_len as usize <= MAX_USB_PAYLOAD_SIZE {
                    ParserState::Payload
                } else {
                    ParserState::Idle
                };
            }
            ParserState::Payload => {
                self.buf.push(byte);
                self.computed_cs ^= byte;
                if self.buf.len() == self.expected_len as usize {
                    self.state = ParserState::Checksum;
                }
            }
            ParserState::Checksum => {
                self.state = ParserState::Idle;
                if byte == self.computed_cs {
                    return Some((self.frame_type, self.buf.clone()));
                }
            }
        }
        None
    }
}

pub fn parse_frame(frame_type: u8, payload: &[u8]) -> Option<ReceiverFrame> {
    match frame_type {
        TYPE_TELEM_FRAME if payload.len() == ESPNOW_TELEM_SIZE => {
            Some(ReceiverFrame::Telemetry(parse_telem(payload)))
        }
        TYPE_STATUS if payload.len() >= 1 => {
            let code = payload[0];
            let msg  = String::from_utf8_lossy(&payload[1..]).trim_end_matches('\0').to_string();
            Some(ReceiverFrame::Status { code, message: msg })
        }
        TYPE_HEARTBEAT if payload.len() == 4 => {
            let ts = u32::from_le_bytes(payload[0..4].try_into().ok()?);
            Some(ReceiverFrame::Heartbeat { timestamp_ms: ts })
        }
        TYPE_RX_RSSI if payload.len() == 1 => {
            Some(ReceiverFrame::RxRssi { rssi: payload[0] as i8 })
        }
        _ => None,
    }
}

fn parse_telem(payload: &[u8]) -> TelemetryFrame {
    let frame_id = u16::from_le_bytes(payload[0..2].try_into().unwrap());
    let st  = size_of::<SunshineState>();
    let isz = size_of::<SunshineInput>();
    let state: SunshineState = unsafe {
        std::ptr::read_unaligned(payload[3..3+st].as_ptr() as *const SunshineState)
    };
    let state_mid: SunshineState = unsafe {
        std::ptr::read_unaligned(payload[3+st..3+2*st].as_ptr() as *const SunshineState)
    };
    let mut inputs = [SunshineInput::default(); INPUTS_PER_FRAME];
    let base = 3 + 2 * st;
    for (i, inp) in inputs.iter_mut().enumerate() {
        let off = base + i * isz;
        *inp = unsafe {
            std::ptr::read_unaligned(payload[off..off+isz].as_ptr() as *const SunshineInput)
        };
    }
    TelemetryFrame { frame_id, state, state_mid, inputs }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn telem_frame_size_matches_structs_and_parses_20_inputs() {
        assert_eq!(INPUTS_PER_FRAME, 20);
        let expect = 3 + 2 * size_of::<SunshineState>() + 20 * size_of::<SunshineInput>();
        assert_eq!(ESPNOW_TELEM_SIZE, expect, "frame size constant must match structs");
        let mut payload = vec![0u8; ESPNOW_TELEM_SIZE];
        payload[0] = 0x34; payload[1] = 0x12;          // frame_id = 0x1234
        let f = parse_telem(&payload);
        assert_eq!(f.frame_id, 0x1234);
        assert_eq!(f.inputs.len(), INPUTS_PER_FRAME);
    }
}

pub fn encode_ctrl(mode: u8, x: i8, y: i8, theta: i8, throttle: u8) -> Vec<u8> {
    let payload = [mode, x as u8, y as u8, theta as u8, throttle];
    let cs: u8 = payload.iter().fold(0u8, |a, b| a ^ b);
    let mut frame = Vec::with_capacity(10);
    frame.push(FRAME_START);
    frame.push(TYPE_CTRL_PACKET);
    frame.push(5u8);
    frame.push(0u8);
    frame.extend_from_slice(&payload);
    frame.push(cs);
    frame
}
