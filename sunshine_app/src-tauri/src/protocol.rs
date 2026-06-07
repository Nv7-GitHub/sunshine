use crate::ffi::{SunshineInput, SunshineState};

pub const FRAME_START: u8 = 0xAA;
pub const TYPE_TELEM_FRAME: u8 = 0x01;
pub const TYPE_CTRL_PACKET: u8 = 0x02;
pub const TYPE_STATUS:      u8 = 0x03;
pub const TYPE_HEARTBEAT:   u8 = 0x04;
pub const TYPE_RX_RSSI:     u8 = 0x05;

pub const STATUS_OK:                 u8 = 0x00;
pub const STATUS_BRAIN_CONNECTED:    u8 = 0x01;
pub const STATUS_BRAIN_DISCONNECTED: u8 = 0x02;

// Brain sends 6 inputs per ESP-NOW packet (250-byte ESP-NOW limit prevents 20).
// 2 (frame_id) + 1 (type) + 60 (SunshineState) + 6×29 (SunshineInput) = 237
pub const ESPNOW_TELEM_SIZE: usize = 237;
pub const INPUTS_PER_FRAME:  usize = 6;
const MAX_USB_PAYLOAD_SIZE: usize = ESPNOW_TELEM_SIZE;

#[derive(Debug, Clone)]
pub struct TelemetryFrame {
    pub frame_id: u16,
    pub state:    SunshineState,
    pub inputs:   [SunshineInput; INPUTS_PER_FRAME],
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
    let state: SunshineState = unsafe {
        std::ptr::read_unaligned(payload[3..3+60].as_ptr() as *const SunshineState)
    };
    let mut inputs = [SunshineInput::default(); INPUTS_PER_FRAME];
    for i in 0..INPUTS_PER_FRAME {
        let off = 63 + i * 29;
        inputs[i] = unsafe {
            std::ptr::read_unaligned(payload[off..off+29].as_ptr() as *const SunshineInput)
        };
    }
    TelemetryFrame { frame_id, state, inputs }
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
