# sunshine_app Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the Tauri 2.x + React host application: live telemetry display, log file writing, replay, simulation, and a uPlot-based graphing panel — all wired through the `sunshine_core` FFI.

**Architecture:** Rust backend owns all data (ring buffer, log writer, replay engine, simulation, serial port). React frontend is purely presentational — it calls Tauri commands and listens to events. The single data pipeline (`pipeline.rs`) drives `sunshine_step()` × 20 for every 50 Hz frame regardless of source (live/replay/sim), keeping all three modes identical post-FFI. Graph data is pull-based: frontend calls `get_graph_data` with viewport bounds; backend decimates using min/max envelope.

**Tech Stack:** Tauri 2.x, React 18, TypeScript, uPlot, Rust (2021 edition), `cc` crate (build.rs), `serialport` crate, `serde`/`serde_json`.

**Prerequisites:** `sunshine_core/` fully built and tested (Plan 1). `sunshine_receiver/` for live testing (Plan 2).

---

## File Structure

```
sunshine_app/
├── src-tauri/
│   ├── Cargo.toml
│   ├── build.rs
│   ├── tauri.conf.json
│   └── src/
│       ├── lib.rs          # Tauri app builder + AppState
│       ├── ffi.rs          # C struct layouts + extern "C" + safe wrappers
│       ├── protocol.rs     # USB frame encode/decode (mirrors receiver/include/protocol.h)
│       ├── serial.rs       # Serial port open/close/reader loop
│       ├── logging.rs      # .sun file writer
│       ├── pipeline.rs     # Ring buffer + brain_step driver + get_graph_data
│       ├── replay.rs       # .sun file reader + replay loop
│       ├── simulation.rs   # Brushed DC motor physics → SunshineInput each tick
│       ├── controls.rs     # Latest control state (keys → filtered values)
│       └── commands.rs     # All #[tauri::command] handlers
├── src/
│   ├── main.tsx
│   ├── App.tsx
│   ├── types/sunshine.ts   # TS types mirroring Rust event payloads
│   ├── hooks/
│   │   ├── useAppState.ts  # top-level state + event listeners
│   │   └── useKeyboard.ts  # key → LP-filtered control values
│   └── components/
│       ├── Header.tsx
│       ├── ModeButtons.tsx
│       ├── StatusBar.tsx
│       ├── LoggingControl.tsx
│       ├── ConnectionPanel.tsx
│       ├── LiveTab.tsx
│       ├── ReplayTab.tsx
│       ├── SimulationTab.tsx
│       ├── GraphPanel.tsx
│       ├── ChannelSelector.tsx
│       └── UPlotCanvas.tsx
├── index.html
├── package.json
└── vite.config.ts
```

---

## Task 1: Project Scaffold + build.rs + sunshine_core FFI

**Files:**
- Create: `sunshine_app/src-tauri/Cargo.toml`
- Create: `sunshine_app/src-tauri/build.rs`
- Create: `sunshine_app/src-tauri/src/ffi.rs`
- Create: `sunshine_app/src-tauri/src/lib.rs`

- [ ] **Step 1: Scaffold Tauri 2.x project**

```bash
cd sunshine_app
npm create tauri-app@latest . -- --template react-ts --manager npm
# Select: React + TypeScript when prompted
npm install
npm install uplot
npm install --save-dev @types/uplot
```

- [ ] **Step 2: Replace src-tauri/Cargo.toml**

```toml
[package]
name    = "sunshine_app"
version = "0.1.0"
edition = "2021"

[lib]
name         = "sunshine_app_lib"
crate-type   = ["staticlib", "cdylib"]

[build-dependencies]
cc = "1"

[dependencies]
tauri        = { version = "2", features = ["protocol-asset"] }
serde        = { version = "1", features = ["derive"] }
serde_json   = "1"
serialport   = "4"
byteorder    = "1"
parking_lot  = "0.12"
tokio        = { version = "1", features = ["full"] }

[profile.release]
panic = "abort"
codegen-units = 1
lto = true
```

- [ ] **Step 3: Create src-tauri/build.rs**

```rust
fn main() {
    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    let core_dir = format!("{}/../../sunshine_core", manifest_dir);

    cc::Build::new()
        .files([
            format!("{}/src/utils.c",        core_dir),
            format!("{}/src/kalman.c",       core_dir),
            format!("{}/src/derot_filter.c", core_dir),
            format!("{}/src/control.c",      core_dir),
            format!("{}/src/brain.c",        core_dir),
        ])
        .include(format!("{}/include", core_dir))
        .flag_if_supported("-lm")
        .compile("sunshine_core");

    println!("cargo:rerun-if-changed={}/src", core_dir);
    println!("cargo:rerun-if-changed={}/include", core_dir);
}
```

- [ ] **Step 4: Create src-tauri/src/ffi.rs**

```rust
//! sunshine_core C FFI bindings.
//! This is the ONLY file with `unsafe` for sunshine_core calls.
//! Struct layouts must exactly match sunshine_core.h (packed, no padding).

use std::mem::size_of;

// ── Packed C struct mirrors ───────────────────────────────────────────────────

#[repr(C, packed)]
#[derive(Clone, Copy, Default, Debug)]
pub struct SunshineInput {
    pub time_us:       u32,
    pub accel_x:       i16,
    pub accel_y:       i16,
    pub accel_z:       i16,
    pub mag_x:         i16,
    pub mag_y:         i16,
    pub mag_z:         i16,
    pub erpm_left:     u16,
    pub erpm_right:    u16,
    pub rssi:          i8,
    pub ctrl_x:        i8,
    pub ctrl_y:        i8,
    pub ctrl_theta:    i8,
    pub ctrl_throttle: u8,
    pub batt_offset:   i8,
    pub dshot_left_q:  u8,
    pub dshot_right_q: u8,
    pub mode:          u8,
}

#[repr(C, packed)]
#[derive(Clone, Copy, Default, Debug)]
pub struct SunshineState {
    pub kf_theta:      f32,
    pub kf_omega:      f32,
    pub kf_p:          [f32; 4],
    pub theta_offset:  f32,
    pub derot_lp_i:    [f32; 4],
    pub derot_lp_q:    [f32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct SunshineVars {
    pub omega_from_accel:  f32,
    pub derot_i:           f32,
    pub derot_q:           f32,
    pub mag_angle:         f32,
    pub est_theta:         f32,
    pub est_omega:         f32,
    pub dshot_cmd_left:    f32,
    pub dshot_cmd_right:   f32,
    pub batt_voltage:      f32,
    pub erpm_left:         f32,
    pub erpm_right:        f32,
    pub centripetal_ms2:   f32,
    pub led_on:            bool,
    pub accel_saturated:   bool,
    pub mag_valid:         bool,
    pub loop_overrun:      bool,
}

// ── Compile-time size assertions ──────────────────────────────────────────────
const _: () = {
    assert!(size_of::<SunshineInput>() == 29, "SunshineInput size mismatch");
    assert!(size_of::<SunshineState>() == 60, "SunshineState size mismatch");
};

// ── Raw C bindings ────────────────────────────────────────────────────────────
extern "C" {
    fn sunshine_step(i: *const SunshineInput, s: *mut SunshineState, v: *mut SunshineVars);
    fn sunshine_state_init(s: *mut SunshineState);
    fn sunshine_schema_version() -> u32;
    fn sunshine_f16_to_f32(half: u16) -> f32;
    fn sunshine_f32_to_f16(f: f32) -> u16;
}

// ── Safe wrappers ─────────────────────────────────────────────────────────────

pub fn brain_step(input: &SunshineInput, state: &mut SunshineState) -> SunshineVars {
    let mut vars = SunshineVars::default();
    unsafe { sunshine_step(input as *const _, state as *mut _, &mut vars as *mut _) }
    vars
}

pub fn state_init(state: &mut SunshineState) {
    unsafe { sunshine_state_init(state as *mut _) }
}

pub fn schema_version() -> u32 {
    unsafe { sunshine_schema_version() }
}

pub fn f16_to_f32(half: u16) -> f32 {
    unsafe { sunshine_f16_to_f32(half) }
}

pub fn f32_to_f16(f: f32) -> u16 {
    unsafe { sunshine_f32_to_f16(f) }
}
```

- [ ] **Step 5: Create src-tauri/src/lib.rs**

```rust
pub mod ffi;
pub mod protocol;
pub mod serial;
pub mod logging;
pub mod pipeline;
pub mod replay;
pub mod simulation;
pub mod controls;
pub mod commands;

use std::sync::Arc;
use parking_lot::Mutex;
use pipeline::Pipeline;
use controls::ControlState;

pub struct AppState {
    pub pipeline: Arc<Mutex<Pipeline>>,
    pub controls: Arc<Mutex<ControlState>>,
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    let pipeline = Arc::new(Mutex::new(Pipeline::new()));
    let controls = Arc::new(Mutex::new(ControlState::default()));

    tauri::Builder::default()
        .manage(AppState { pipeline: pipeline.clone(), controls: controls.clone() })
        .invoke_handler(tauri::generate_handler![
            commands::list_serial_ports,
            commands::connect_serial,
            commands::disconnect_serial,
            commands::open_replay,
            commands::start_simulation,
            commands::stop_source,
            commands::set_mode,
            commands::set_controls,
            commands::enable_logging,
            commands::disable_logging,
            commands::get_graph_data,
        ])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
```

- [ ] **Step 6: Verify build**

```bash
cd sunshine_app && npm run tauri build -- --debug 2>&1 | grep -E "^error" | head -20
```
Expected: compiles (may have warnings for empty module stubs — add `#![allow(dead_code)]` to each stub).

- [ ] **Step 7: Commit**

```bash
git add sunshine_app/
git commit -m "feat(app): Tauri scaffold + build.rs + sunshine_core FFI layer"
```

---

## Task 2: Protocol Decode + Serial Port Management

**Files:**
- Create: `sunshine_app/src-tauri/src/protocol.rs`
- Create: `sunshine_app/src-tauri/src/serial.rs`

- [ ] **Step 1: Create src-tauri/src/protocol.rs**

```rust
//! USB serial frame encode/decode — mirrors sunshine_receiver/include/protocol.h

use crate::ffi::{SunshineInput, SunshineState};
use byteorder::{LittleEndian, ReadBytesExt};
use std::io::Cursor;

pub const FRAME_START: u8 = 0xAA;
pub const TYPE_TELEM_FRAME: u8 = 0x01;
pub const TYPE_CTRL_PACKET: u8 = 0x02;
pub const TYPE_STATUS:      u8 = 0x03;
pub const TYPE_HEARTBEAT:   u8 = 0x04;
pub const TYPE_RX_RSSI:     u8 = 0x05;

pub const STATUS_OK:                 u8 = 0x00;
pub const STATUS_BRAIN_CONNECTED:    u8 = 0x01;
pub const STATUS_BRAIN_DISCONNECTED: u8 = 0x02;

pub const ESPNOW_TELEM_SIZE: usize = 643;

/// Parsed telemetry frame (brain → receiver → host)
#[derive(Debug, Clone)]
pub struct TelemetryFrame {
    pub frame_id: u16,
    pub state:    SunshineState,
    pub inputs:   [SunshineInput; 20],
}

/// Parsed inbound frame from receiver
#[derive(Debug)]
pub enum ReceiverFrame {
    Telemetry(TelemetryFrame),
    Status { code: u8, message: String },
    Heartbeat { timestamp_ms: u32 },
    RxRssi { rssi: i8 },
}

/// Incremental frame parser — feed bytes one at a time
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

    /// Feed one byte. Returns Some(payload bytes + type) when a complete frame is received.
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
                self.state = if self.expected_len == 0 { ParserState::Checksum }
                             else { ParserState::Payload };
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

/// Parse a validated payload into a ReceiverFrame
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
    // type byte at [2] ignored
    let state: SunshineState = unsafe {
        std::ptr::read_unaligned(payload[3..3+60].as_ptr() as *const SunshineState)
    };
    let mut inputs = [SunshineInput::default(); 20];
    for i in 0..20 {
        let off = 63 + i * 29;
        inputs[i] = unsafe {
            std::ptr::read_unaligned(payload[off..off+29].as_ptr() as *const SunshineInput)
        };
    }
    TelemetryFrame { frame_id, state, inputs }
}

/// Encode a CTRL_PACKET to send to receiver
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
```

- [ ] **Step 2: Create src-tauri/src/serial.rs**

```rust
//! Serial port management — open, read loop, send control frames.

use crate::protocol::{FrameParser, ReceiverFrame, parse_frame};
use serialport::SerialPort;
use std::sync::{Arc, atomic::{AtomicBool, Ordering}};
use std::thread;
use std::time::Duration;
use parking_lot::Mutex;

pub fn list_ports() -> Vec<String> {
    serialport::available_ports()
        .unwrap_or_default()
        .into_iter()
        .map(|p| p.port_name)
        .collect()
}

/// Callback type invoked on every parsed ReceiverFrame (called from reader thread)
pub type FrameCallback = Arc<dyn Fn(ReceiverFrame) + Send + Sync>;

pub struct SerialConnection {
    port:    Arc<Mutex<Box<dyn SerialPort>>>,
    running: Arc<AtomicBool>,
}

impl SerialConnection {
    pub fn open(port_name: &str, on_frame: FrameCallback) -> Result<Self, String> {
        let port = serialport::new(port_name, 921600)
            .timeout(Duration::from_millis(10))
            .open()
            .map_err(|e| e.to_string())?;

        let port     = Arc::new(Mutex::new(port));
        let running  = Arc::new(AtomicBool::new(true));
        let port_rx  = port.clone();
        let running2 = running.clone();

        thread::spawn(move || {
            let mut parser = FrameParser::new();
            let mut buf    = [0u8; 1024];
            while running2.load(Ordering::Relaxed) {
                let n = {
                    let mut p = port_rx.lock();
                    p.read(&mut buf).unwrap_or(0)
                };
                for &byte in &buf[..n] {
                    if let Some((t, payload)) = parser.feed(byte) {
                        if let Some(frame) = parse_frame(t, &payload) {
                            on_frame(frame);
                        }
                    }
                }
                if n == 0 {
                    thread::sleep(Duration::from_millis(1));
                }
            }
        });

        Ok(SerialConnection { port, running })
    }

    pub fn send(&self, data: &[u8]) {
        let _ = self.port.lock().write_all(data);
    }

    pub fn close(self) {
        self.running.store(false, Ordering::Relaxed);
    }
}

impl std::io::Write for SerialConnection {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        self.port.lock().write(buf)
    }
    fn flush(&mut self) -> std::io::Result<()> { Ok(()) }
}
```

- [ ] **Step 3: Build check**

```bash
cd sunshine_app && npm run tauri build -- --debug 2>&1 | grep "^error" | head -10
```
Expected: 0 errors.

- [ ] **Step 4: Commit**

```bash
git add sunshine_app/src-tauri/src/protocol.rs sunshine_app/src-tauri/src/serial.rs
git commit -m "feat(app): USB serial protocol parser + serial port manager"
```

---

## Task 3: Logging + Pipeline + Ring Buffer

**Files:**
- Create: `sunshine_app/src-tauri/src/logging.rs`
- Create: `sunshine_app/src-tauri/src/pipeline.rs`

- [ ] **Step 1: Create src-tauri/src/logging.rs**

```rust
//! .sun file writer — binary log format as specified in the system design.

use crate::ffi::{SunshineInput, SunshineState, SunshineVars, schema_version};
use std::fs::File;
use std::io::{BufWriter, Write, Seek, SeekFrom};
use std::mem::size_of;
use std::path::PathBuf;
use std::time::{SystemTime, UNIX_EPOCH};

const MAGIC: &[u8; 5]  = b"SHINE";
const FILE_FORMAT_VER: u16 = 1;
const HEADER_SIZE: u16 = 93;
const SOURCE_LIVE: u8 = 0;
const SOURCE_REPLAY: u8 = 1;
const SOURCE_SIM: u8 = 2;

pub struct LogWriter {
    writer:      BufWriter<File>,
    frame_count: u32,
    flush_every: u32,
    path:        PathBuf,
}

impl LogWriter {
    pub fn new(path: PathBuf, label: &str, source: u8) -> std::io::Result<Self> {
        let file = File::create(&path)?;
        let mut w = BufWriter::new(file);

        let now_ms = SystemTime::now()
            .duration_since(UNIX_EPOCH).unwrap().as_millis() as u64;

        // Header: 93 bytes
        w.write_all(MAGIC)?;
        w.write_all(&FILE_FORMAT_VER.to_le_bytes())?;
        w.write_all(&HEADER_SIZE.to_le_bytes())?;
        w.write_all(&schema_version().to_le_bytes())?;
        w.write_all(&(size_of::<SunshineInput>() as u16).to_le_bytes())?;
        w.write_all(&(size_of::<SunshineState>() as u16).to_le_bytes())?;
        w.write_all(&(size_of::<SunshineVars>() as u16).to_le_bytes())?;
        w.write_all(&now_ms.to_le_bytes())?;
        w.write_all(&[source])?;
        w.write_all(&[0u8])?;  // flags: logging_complete=0 until close()

        // 64-byte label (null-terminated, zero-padded)
        let mut label_buf = [0u8; 64];
        let bytes = label.as_bytes();
        let n = bytes.len().min(63);
        label_buf[..n].copy_from_slice(&bytes[..n]);
        w.write_all(&label_buf)?;

        Ok(LogWriter { writer: w, frame_count: 0, flush_every: 10, path })
    }

    pub fn write_frame(
        &mut self,
        frame_id: u32,
        state:    &SunshineState,
        inputs:   &[SunshineInput; 20],
        vars:     &SunshineVars,
    ) -> std::io::Result<()> {
        // frame_id (4B) + flags (1B) + state + inputs×20 + vars
        self.writer.write_all(&frame_id.to_le_bytes())?;
        self.writer.write_all(&[0x01u8])?;  // vars_valid = 1

        let state_bytes = unsafe {
            std::slice::from_raw_parts(state as *const _ as *const u8, size_of::<SunshineState>())
        };
        self.writer.write_all(state_bytes)?;

        for inp in inputs {
            let inp_bytes = unsafe {
                std::slice::from_raw_parts(inp as *const _ as *const u8, size_of::<SunshineInput>())
            };
            self.writer.write_all(inp_bytes)?;
        }

        let vars_bytes = unsafe {
            std::slice::from_raw_parts(vars as *const _ as *const u8, size_of::<SunshineVars>())
        };
        self.writer.write_all(vars_bytes)?;

        self.frame_count += 1;
        if self.frame_count % self.flush_every == 0 {
            self.writer.flush()?;
        }
        Ok(())
    }

    /// Set logging_complete flag and flush on clean close
    pub fn close(mut self) -> std::io::Result<()> {
        // flags byte is at offset 28 in the header
        self.writer.flush()?;
        let mut file = self.writer.into_inner()?;
        file.seek(SeekFrom::Start(28))?;
        file.write_all(&[0x01u8])?;  // logging_complete = 1
        Ok(())
    }

    pub fn path(&self) -> &PathBuf { &self.path }
    pub fn frame_count(&self) -> u32 { self.frame_count }
}

/// Generate a log file path: YYYY-MM-DD_HH-MM-SS[_label].sun
pub fn make_log_path(base_dir: &std::path::Path, label: &str) -> PathBuf {
    use std::time::{SystemTime, UNIX_EPOCH};
    let secs = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_secs();
    let dt   = format_datetime(secs);
    let name = if label.is_empty() {
        format!("{}.sun", dt)
    } else {
        let safe: String = label.chars().map(|c| if c.is_alphanumeric() || c == '_' { c } else { '_' }).collect();
        format!("{}_{}.sun", dt, safe)
    };
    base_dir.join(name)
}

fn format_datetime(unix_secs: u64) -> String {
    // Simple UTC formatter (no external crate)
    let s   = unix_secs;
    let sec = s % 60; let s = s / 60;
    let min = s % 60; let s = s / 60;
    let hr  = s % 24; let s = s / 24;
    // Days since 1970-01-01 → approx date (good enough for file names)
    let (y, m, d) = days_to_ymd(s as u32);
    format!("{:04}-{:02}-{:02}_{:02}-{:02}-{:02}", y, m, d, hr, min, sec)
}

fn days_to_ymd(mut days: u32) -> (u32, u32, u32) {
    let mut y = 1970u32;
    loop {
        let leap = (y % 4 == 0 && y % 100 != 0) || y % 400 == 0;
        let dy   = if leap { 366 } else { 365 };
        if days < dy { break; }
        days -= dy; y += 1;
    }
    let leap  = (y % 4 == 0 && y % 100 != 0) || y % 400 == 0;
    let mdays = [31u32, if leap {29} else {28}, 31,30,31,30,31,31,30,31,30,31];
    let mut m = 1u32;
    for &md in &mdays {
        if days < md { break; }
        days -= md; m += 1;
    }
    (y, m, days + 1)
}
```

- [ ] **Step 2: Create src-tauri/src/pipeline.rs**

This is the core data engine. For brevity, channels are indexed by a string key and stored in a flat time-series store.

```rust
//! Central data pipeline — drives brain_step(), holds ring buffer, serves graph data.

use crate::ffi::{SunshineInput, SunshineState, SunshineVars, brain_step, state_init};
use crate::logging::LogWriter;
use crate::protocol::TelemetryFrame;
use parking_lot::Mutex;
use std::collections::HashMap;
use std::sync::Arc;

// ── Ring buffer entry ─────────────────────────────────────────────────────────
#[derive(Clone, Default)]
pub struct DataPoint {
    pub time_us:   u64,
    pub input:     SunshineInput,
    pub real_vars: SunshineVars,   // 50Hz snapshots from robot
    pub rep_vars:  SunshineVars,   // 1kHz replayed
}

// 120s × 1000 Hz = 120_000 points
const RING_CAP: usize = 120_000;

pub struct Pipeline {
    ring:         Vec<DataPoint>,
    ring_head:    usize,
    ring_len:     usize,
    replay_state: SunshineState,
    pub logger:   Option<LogWriter>,
    pub source:   SourceKind,
    frame_count:  u32,
    // channel name → accessor function index (used in get_graph_data)
}

#[derive(Clone, Debug, PartialEq)]
pub enum SourceKind { None, Live, Replay, Simulation }

impl Pipeline {
    pub fn new() -> Self {
        let mut ring = Vec::with_capacity(RING_CAP);
        ring.resize(RING_CAP, DataPoint::default());
        let mut replay_state = SunshineState::default();
        state_init(&mut replay_state);
        Pipeline {
            ring, ring_head: 0, ring_len: 0,
            replay_state,
            logger: None,
            source: SourceKind::None,
            frame_count: 0,
        }
    }

    /// Ingest one 50 Hz telemetry frame (live or from .sun file).
    /// Runs brain_step() × 20 for the replayed 1 kHz series.
    pub fn ingest_frame(&mut self, frame: &TelemetryFrame, real_vars_snap: Option<&SunshineVars>) {
        let mut frame_inputs = frame.inputs;

        for (i, input) in frame_inputs.iter().enumerate() {
            let rep_vars = brain_step(input, &mut self.replay_state);

            let dp = DataPoint {
                time_us:   input.time_us as u64,
                input:     *input,
                real_vars: real_vars_snap.copied().unwrap_or_default(),
                rep_vars,
            };

            let idx = self.ring_head;
            self.ring[idx] = dp;
            self.ring_head = (self.ring_head + 1) % RING_CAP;
            if self.ring_len < RING_CAP { self.ring_len += 1; }
        }

        // Log at 50 Hz
        if let Some(logger) = self.logger.as_mut() {
            let vars_snap = real_vars_snap.copied().unwrap_or_default();
            let _ = logger.write_frame(
                self.frame_count,
                &frame.state,
                &frame.inputs,
                &vars_snap,
            );
        }
        self.frame_count += 1;
    }

    /// Reset replay state (e.g. when opening a new file or switching sources)
    pub fn reset_replay_state(&mut self, state: &SunshineState) {
        self.replay_state = *state;
    }

    /// Graph data query — returns at most max_points decimated (t_us, value) pairs.
    /// channel: e.g. "rep.est_theta", "real.est_omega", "input.accel_x"
    pub fn get_graph_data(
        &self,
        channel:    &str,
        start_us:   u64,
        end_us:     u64,
        max_points: u32,
    ) -> Vec<(u64, f32)> {
        if self.ring_len == 0 { return vec![]; }

        // Collect raw points in time range
        let accessor = channel_accessor(channel);
        let mut raw: Vec<(u64, f32)> = Vec::new();
        let start_idx = (self.ring_head + RING_CAP - self.ring_len) % RING_CAP;
        for i in 0..self.ring_len {
            let dp = &self.ring[(start_idx + i) % RING_CAP];
            if dp.time_us < start_us { continue; }
            if dp.time_us > end_us   { break; }
            raw.push((dp.time_us, accessor(dp)));
        }

        if raw.len() <= max_points as usize { return raw; }

        // Min/max envelope decimation
        let bucket = raw.len() / max_points as usize;
        let mut out = Vec::with_capacity(max_points as usize * 2);
        for chunk in raw.chunks(bucket) {
            if chunk.is_empty() { continue; }
            let (t0, _) = chunk[0];
            let (t1, _) = *chunk.last().unwrap();
            let min = chunk.iter().map(|&(_, v)| v).fold(f32::INFINITY, f32::min);
            let max = chunk.iter().map(|&(_, v)| v).fold(f32::NEG_INFINITY, f32::max);
            let t_mid = (t0 + t1) / 2;
            out.push((t0, min));
            out.push((t_mid, max));
        }
        out
    }
}

/// Map channel string → accessor closure
fn channel_accessor(channel: &str) -> fn(&DataPoint) -> f32 {
    match channel {
        "rep.est_theta"         => |dp: &DataPoint| dp.rep_vars.est_theta,
        "rep.est_omega"         => |dp| dp.rep_vars.est_omega,
        "rep.dshot_left"        => |dp| dp.rep_vars.dshot_cmd_left,
        "rep.dshot_right"       => |dp| dp.rep_vars.dshot_cmd_right,
        "rep.batt_voltage"      => |dp| dp.rep_vars.batt_voltage,
        "rep.erpm_left"         => |dp| dp.rep_vars.erpm_left,
        "rep.erpm_right"        => |dp| dp.rep_vars.erpm_right,
        "rep.mag_angle"         => |dp| dp.rep_vars.mag_angle,
        "rep.derot_i"           => |dp| dp.rep_vars.derot_i,
        "rep.derot_q"           => |dp| dp.rep_vars.derot_q,
        "rep.omega_from_accel"  => |dp| dp.rep_vars.omega_from_accel,
        "rep.centripetal_ms2"   => |dp| dp.rep_vars.centripetal_ms2,
        "real.est_theta"        => |dp| dp.real_vars.est_theta,
        "real.est_omega"        => |dp| dp.real_vars.est_omega,
        "real.dshot_left"       => |dp| dp.real_vars.dshot_cmd_left,
        "real.dshot_right"      => |dp| dp.real_vars.dshot_cmd_right,
        "input.accel_x"         => |dp| dp.input.accel_x as f32,
        "input.accel_y"         => |dp| dp.input.accel_y as f32,
        "input.accel_z"         => |dp| dp.input.accel_z as f32,
        "input.mag_x"           => |dp| dp.input.mag_x as f32,
        "input.mag_y"           => |dp| dp.input.mag_y as f32,
        "input.ctrl_x"          => |dp| dp.input.ctrl_x as f32,
        "input.ctrl_y"          => |dp| dp.input.ctrl_y as f32,
        "input.ctrl_throttle"   => |dp| dp.input.ctrl_throttle as f32,
        "input.rssi"            => |dp| dp.input.rssi as f32,
        "input.batt_offset"     => |dp| dp.input.batt_offset as f32,
        _                       => |_| 0.0,
    }
}
```

- [ ] **Step 3: Build check**

```bash
cd sunshine_app && npm run tauri build -- --debug 2>&1 | grep "^error" | head -10
```
Expected: 0 errors.

- [ ] **Step 4: Commit**

```bash
git add sunshine_app/src-tauri/src/logging.rs sunshine_app/src-tauri/src/pipeline.rs
git commit -m "feat(app): .sun log writer + pipeline ring buffer + graph decimation"
```

---

## Task 4: Replay + Simulation + Controls + Commands

**Files:**
- Create: `sunshine_app/src-tauri/src/replay.rs`
- Create: `sunshine_app/src-tauri/src/simulation.rs`
- Create: `sunshine_app/src-tauri/src/controls.rs`
- Create: `sunshine_app/src-tauri/src/commands.rs`

- [ ] **Step 1: Create src-tauri/src/replay.rs**

```rust
//! .sun file reader with backwards-compatible struct sizing.

use crate::ffi::{SunshineInput, SunshineState, SunshineVars, schema_version};
use crate::protocol::TelemetryFrame;
use std::fs::File;
use std::io::{Read, Seek, SeekFrom};
use std::mem::size_of;
use std::path::PathBuf;

#[derive(Debug, Clone)]
pub struct LogMetadata {
    pub path:           PathBuf,
    pub schema_version: u32,
    pub sizeof_input:   u16,
    pub sizeof_state:   u16,
    pub sizeof_vars:    u16,
    pub created_at_ms:  u64,
    pub source:         u8,
    pub label:          String,
    pub frame_count:    u32,  // estimated from file size
}

pub fn read_metadata(path: &PathBuf) -> std::io::Result<LogMetadata> {
    let mut f = File::open(path)?;
    let mut hdr = [0u8; 93];
    f.read_exact(&mut hdr)?;

    if &hdr[0..5] != b"SHINE" {
        return Err(std::io::Error::new(std::io::ErrorKind::InvalidData, "bad magic"));
    }
    let header_size   = u16::from_le_bytes(hdr[7..9].try_into().unwrap()) as u64;
    let schema_ver    = u32::from_le_bytes(hdr[9..13].try_into().unwrap());
    let sizeof_input  = u16::from_le_bytes(hdr[13..15].try_into().unwrap());
    let sizeof_state  = u16::from_le_bytes(hdr[15..17].try_into().unwrap());
    let sizeof_vars   = u16::from_le_bytes(hdr[17..19].try_into().unwrap());
    let created_at_ms = u64::from_le_bytes(hdr[19..27].try_into().unwrap());
    let source        = hdr[27];
    let label_end     = hdr[29..93].iter().position(|&b| b == 0).unwrap_or(64);
    let label         = String::from_utf8_lossy(&hdr[29..29+label_end]).to_string();

    let file_size     = f.seek(SeekFrom::End(0))?;
    let frame_size    = 5u64 + sizeof_state as u64 + sizeof_input as u64 * 20 + sizeof_vars as u64;
    let frame_count   = if frame_size > 0 { ((file_size - header_size) / frame_size) as u32 } else { 0 };

    Ok(LogMetadata { path: path.clone(), schema_version: schema_ver, sizeof_input,
                     sizeof_state, sizeof_vars, created_at_ms, source, label, frame_count })
}

/// Read one frame from file at current position.
/// Handles sizeof mismatches (append-only backwards compat).
pub fn read_frame(f: &mut File, meta: &LogMetadata) -> std::io::Result<(TelemetryFrame, SunshineVars)> {
    let mut id_flags = [0u8; 5];
    f.read_exact(&mut id_flags)?;
    let frame_id = u32::from_le_bytes(id_flags[0..4].try_into().unwrap());

    let state    = read_padded::<SunshineState>(f, meta.sizeof_state as usize)?;
    let mut inputs = [SunshineInput::default(); 20];
    for inp in inputs.iter_mut() {
        *inp = read_padded::<SunshineInput>(f, meta.sizeof_input as usize)?;
    }
    let vars = read_padded::<SunshineVars>(f, meta.sizeof_vars as usize)?;

    Ok((TelemetryFrame { frame_id: frame_id as u16, state, inputs }, vars))
}

fn read_padded<T: Default + Copy>(f: &mut File, file_size: usize) -> std::io::Result<T> {
    let code_size = size_of::<T>();
    let read_n    = code_size.min(file_size);
    let mut buf   = vec![0u8; file_size];
    f.read_exact(&mut buf)?;
    let mut obj   = T::default();
    unsafe {
        std::ptr::copy_nonoverlapping(
            buf.as_ptr(),
            &mut obj as *mut T as *mut u8,
            read_n,
        );
    }
    Ok(obj)
}
```

- [ ] **Step 2: Create src-tauri/src/simulation.rs**

```rust
//! Brushed DC motor + robot body physics simulation.
//! Produces SunshineInput at 1 kHz; consumes dshot_cmd from SunshineVars.

use crate::ffi::{SunshineInput, SunshineVars, f32_to_f16};
use std::f64::consts::PI;

// ── Physical constants (easily editable) ─────────────────────────────────────
const KV:           f64 = 1100.0;            // RPM/V
const KT:           f64 = 60.0 / (2.0*PI*KV); // N·m/A
const R_PHASE:      f64 = 0.075;             // Ω
const V_NOMINAL:    f64 = 8.4;               // V (full charge 2S)
const R_INTERNAL:   f64 = 0.008;             // Ω battery internal
const WHEEL_RADIUS: f64 = 0.022;             // m
const WHEEL_CENTER: f64 = 0.0405;            // m (from spin axis)
const MOI:          f64 = 1.214e-3;          // kg·m²
const IMU_RADIUS:   f64 = 0.011;             // m
const EARTH_FIELD:  f64 = 50.0;              // µT
const EARTH_ANGLE:  f64 = 0.0;              // rad (fixed Earth field direction)
const ADXL_SCALE:   f64 = 49e-3 * 9.81;     // m/s² per count
const MAG_SCALE:    f64 = 0.058;             // µT per count
const BATT_REF_V:   f64 = 7.6;
const BATT_SCALE:   f64 = 0.0205;

pub struct Simulation {
    body_omega:  f64,
    body_angle:  f64,
    omega_left:  f64,   // wheel angular velocity (rad/s)
    omega_right: f64,
    time_us:     u64,
}

impl Simulation {
    pub fn new() -> Self {
        Simulation { body_omega: 0.0, body_angle: 0.0,
                     omega_left: 0.0, omega_right: 0.0, time_us: 0 }
    }

    /// Advance one 1 ms tick. Returns a SunshineInput with simulated sensor values.
    pub fn tick(&mut self, last_vars: &SunshineVars) -> SunshineInput {
        let dt = 1e-3f64;
        self.time_us += 1000;

        // Wheel ω from body spin (body drives wheels through traction)
        let wheel_omega_body = self.body_omega * WHEEL_CENTER / WHEEL_RADIUS;

        // Motor model (per wheel) — use dshot_cmd from previous tick
        let v_term = self.terminal_voltage(last_vars.dshot_cmd_left, last_vars.dshot_cmd_right);
        let (torque_l, self_omega_l) = self.motor_tick(last_vars.dshot_cmd_left as f64 / 2047.0, self.omega_left,  v_term, dt);
        let (torque_r, self_omega_r) = self.motor_tick(last_vars.dshot_cmd_right as f64 / 2047.0, self.omega_right, v_term, dt);
        self.omega_left  = self_omega_l;
        self.omega_right = self_omega_r;

        // Body dynamics: net torque from both wheels
        let torque_body = (torque_l + torque_r) * WHEEL_CENTER / WHEEL_RADIUS;
        let alpha = torque_body / MOI;
        self.body_omega += alpha * dt;
        self.body_angle += self.body_omega * dt;

        // Sensor model
        let a_centripetal = self.body_omega.powi(2) * IMU_RADIUS;
        let a_tangential  = alpha * IMU_RADIUS;
        // IMU at 45° to radial: ax = (a_c - a_t)/√2, ay = (a_c + a_t)/√2
        let ax = (a_centripetal - a_tangential) / 2.0f64.sqrt();
        let ay = (a_centripetal + a_tangential) / 2.0f64.sqrt();
        let az = 9.81f64;

        // Magnetometer: Earth field rotated by negative body angle
        let mx = EARTH_FIELD * (EARTH_ANGLE - self.body_angle).cos();
        let my = EARTH_FIELD * (EARTH_ANGLE - self.body_angle).sin();

        // Battery voltage
        let i_total = self.motor_current(last_vars.dshot_cmd_left as f64 / 2047.0, self.omega_left, V_NOMINAL)
                    + self.motor_current(last_vars.dshot_cmd_right as f64 / 2047.0, self.omega_right, V_NOMINAL);
        let v_batt  = V_NOMINAL - i_total * R_INTERNAL;
        let batt_offset = ((v_batt - BATT_REF_V) / BATT_SCALE).round().clamp(-127.0, 127.0) as i8;

        SunshineInput {
            time_us:       self.time_us as u32,
            accel_x:       (ax / ADXL_SCALE).round().clamp(-32768.0, 32767.0) as i16,
            accel_y:       (ay / ADXL_SCALE).round().clamp(-32768.0, 32767.0) as i16,
            accel_z:       (az / ADXL_SCALE).round().clamp(-32768.0, 32767.0) as i16,
            mag_x:         (mx / MAG_SCALE).round().clamp(-32768.0, 32767.0) as i16,
            mag_y:         (my / MAG_SCALE).round().clamp(-32768.0, 32767.0) as i16,
            mag_z:         0,
            erpm_left:     f32_to_f16((self.omega_left  * 60.0 / (2.0 * PI as f64)) as f32),
            erpm_right:    f32_to_f16((self.omega_right * 60.0 / (2.0 * PI as f64)) as f32),
            batt_offset,
            ..SunshineInput::default()
        }
    }

    fn terminal_voltage(&self, cmd_l: f32, cmd_r: f32) -> f64 {
        let i_l = self.motor_current(cmd_l as f64 / 2047.0, self.omega_left,  V_NOMINAL);
        let i_r = self.motor_current(cmd_r as f64 / 2047.0, self.omega_right, V_NOMINAL);
        V_NOMINAL - (i_l + i_r) * R_INTERNAL
    }

    fn motor_current(&self, throttle: f64, omega: f64, v_term: f64) -> f64 {
        let v_motor  = throttle * v_term;
        let back_emf = omega / (KV * 2.0 * PI / 60.0);
        ((v_motor - back_emf) / R_PHASE).max(0.0)
    }

    fn motor_tick(&self, throttle: f64, omega: f64, v_term: f64, dt: f64) -> (f64, f64) {
        let current = self.motor_current(throttle, omega, v_term);
        let torque  = KT * current;
        // Simple wheel inertia model (treat wheel as point mass)
        let new_omega = (omega + (torque / (0.001 + 1e-6)) * dt).max(0.0);
        (torque, new_omega)
    }
}
```

- [ ] **Step 3: Create src-tauri/src/controls.rs**

```rust
//! Latest control state — shared between commands.rs and serial.rs.

use serde::{Deserialize, Serialize};

#[derive(Clone, Default, Debug, Serialize, Deserialize)]
pub struct ControlState {
    pub mode:          u8,
    pub ctrl_x:        i8,
    pub ctrl_y:        i8,
    pub ctrl_theta:    i8,
    pub ctrl_throttle: u8,
}
```

- [ ] **Step 4: Create src-tauri/src/commands.rs**

```rust
//! All Tauri command handlers (called from JavaScript via invoke()).

use crate::{AppState, serial::{SerialConnection, list_ports as serial_list},
            protocol::{ReceiverFrame, encode_ctrl, TelemetryFrame},
            pipeline::{SourceKind}, replay::{read_metadata, read_frame, LogMetadata},
            simulation::Simulation, logging::{LogWriter, make_log_path},
            ffi::{brain_step, state_init, SunshineVars, SunshineState}};
use tauri::{State, AppHandle, Emitter};
use parking_lot::Mutex;
use std::sync::Arc;
use serde::{Serialize, Deserialize};

#[tauri::command]
pub fn list_serial_ports() -> Vec<String> {
    serial_list()
}

#[tauri::command]
pub async fn connect_serial(
    port:     String,
    app:      AppHandle,
    state:    State<'_, AppState>,
) -> Result<(), String> {
    let pipeline = state.pipeline.clone();
    let app2     = app.clone();

    let conn = SerialConnection::open(&port, Arc::new(move |frame| {
        match frame {
            ReceiverFrame::Telemetry(telem) => {
                let mut pipe = pipeline.lock();
                // Compute real_vars from last replayed state snapshot (50 Hz only)
                pipe.ingest_frame(&telem, None);
                // Emit live_update to frontend
                let _ = app2.emit("live_update", build_live_update(&telem, &pipe));
            }
            ReceiverFrame::Status { code, message } => {
                let _ = app2.emit("source_status", serde_json::json!({
                    "kind": "Live", "code": code, "detail": message
                }));
            }
            ReceiverFrame::RxRssi { rssi } => {
                let _ = app2.emit("rx_rssi", rssi);
            }
            _ => {}
        }
    })).map_err(|e| e)?;

    // Store connection (simplified — in full impl use AppState field)
    Ok(())
}

#[tauri::command]
pub fn disconnect_serial(state: State<'_, AppState>) {
    // close serial connection
}

#[tauri::command]
pub fn open_replay(path: String) -> Result<serde_json::Value, String> {
    let path = std::path::PathBuf::from(&path);
    let meta = read_metadata(&path).map_err(|e| e.to_string())?;
    Ok(serde_json::json!({
        "label":        meta.label,
        "created_at_ms": meta.created_at_ms,
        "frame_count":  meta.frame_count,
        "schema_version": meta.schema_version,
    }))
}

#[tauri::command]
pub fn start_simulation(state: State<'_, AppState>, app: AppHandle) {
    let pipeline = state.pipeline.clone();
    let mut sim  = Simulation::new();
    let mut replay_state = SunshineState::default();
    state_init(&mut replay_state);
    let mut prev_vars = SunshineVars::default();

    std::thread::spawn(move || {
        loop {
            let input  = sim.tick(&prev_vars);
            let vars   = brain_step(&input, &mut replay_state);
            prev_vars  = vars;

            let telem  = crate::protocol::TelemetryFrame {
                frame_id: 0,
                state:    replay_state,
                inputs:   {
                    let mut arr = [input; 20];
                    arr  // simplified: 20 identical inputs per frame
                },
            };
            let mut pipe = pipeline.lock();
            pipe.ingest_frame(&telem, None);
            drop(pipe);
            std::thread::sleep(std::time::Duration::from_millis(20)); // 50 Hz
        }
    });
}

#[tauri::command]
pub fn stop_source(state: State<'_, AppState>) {
    state.pipeline.lock().source = SourceKind::None;
}

#[tauri::command]
pub fn set_mode(mode: u8, state: State<'_, AppState>) {
    state.controls.lock().mode = mode;
}

#[tauri::command]
pub fn set_controls(
    x: i8, y: i8, theta: i8, throttle: u8,
    state: State<'_, AppState>,
) {
    let mut ctrl = state.controls.lock();
    ctrl.ctrl_x        = x;
    ctrl.ctrl_y        = y;
    ctrl.ctrl_theta    = theta;
    ctrl.ctrl_throttle = throttle;
}

#[tauri::command]
pub fn enable_logging(label: String, state: State<'_, AppState>) -> Result<String, String> {
    let dir  = dirs::document_dir().unwrap_or_default().join("sunshine_logs");
    std::fs::create_dir_all(&dir).map_err(|e| e.to_string())?;
    let path = make_log_path(&dir, &label);
    let writer = LogWriter::new(path.clone(), &label, 0).map_err(|e| e.to_string())?;
    state.pipeline.lock().logger = Some(writer);
    Ok(path.to_string_lossy().to_string())
}

#[tauri::command]
pub fn disable_logging(state: State<'_, AppState>) {
    state.pipeline.lock().logger = None; // Drop closes and sets logging_complete flag
}

#[tauri::command]
pub fn get_graph_data(
    channel:    String,
    start_us:   u64,
    end_us:     u64,
    max_points: u32,
    state:      State<'_, AppState>,
) -> Vec<(u64, f32)> {
    state.pipeline.lock().get_graph_data(&channel, start_us, end_us, max_points)
}

fn build_live_update(telem: &TelemetryFrame, pipe: &crate::pipeline::Pipeline) -> serde_json::Value {
    // Pull the latest replayed vars from the last ring buffer entry
    serde_json::json!({
        "frame_id":   telem.frame_id,
        "est_theta":  telem.state.kf_theta,
        "est_omega":  telem.state.kf_omega,
        "mode":       telem.inputs[19].mode,
        "rssi":       telem.inputs[19].rssi,
        "batt_offset": telem.inputs[19].batt_offset,
    })
}
```

- [ ] **Step 5: Build check**

```bash
cd sunshine_app && npm run tauri build -- --debug 2>&1 | grep "^error" | head -20
```

- [ ] **Step 6: Commit**

```bash
git add sunshine_app/src-tauri/src/{replay,simulation,controls,commands}.rs
git commit -m "feat(app): replay reader, simulation engine, controls, command handlers"
```

---

## Task 5: React UI

**Files:**
- Create: `sunshine_app/src/types/sunshine.ts`
- Create: `sunshine_app/src/hooks/useAppState.ts`
- Create: `sunshine_app/src/hooks/useKeyboard.ts`
- Create: `sunshine_app/src/App.tsx`
- Create: `sunshine_app/src/components/*.tsx`

- [ ] **Step 1: Create src/types/sunshine.ts**

```typescript
export type Mode = 0 | 1 | 2; // DISABLED | TANK | MELTY
export const MODE_NAMES = ['DISABLED', 'TANK', 'MELTY'] as const;

export interface LiveUpdate {
  frame_id:    number;
  est_theta:   number;
  est_omega:   number;
  mode:        Mode;
  rssi:        number;
  batt_offset: number;
}

export interface SourceStatus {
  kind:   'Live' | 'Replay' | 'Sim' | 'Disconnected';
  code?:  number;
  detail: string;
}

export interface LogStatus {
  active:      boolean;
  path:        string;
  frame_count: number;
}

export const CHANNELS = {
  Inputs: [
    { key: 'input.accel_x',       label: 'Accel X',       unit: 'counts' },
    { key: 'input.accel_y',       label: 'Accel Y',       unit: 'counts' },
    { key: 'input.accel_z',       label: 'Accel Z',       unit: 'counts' },
    { key: 'input.mag_x',         label: 'Mag X',         unit: 'counts' },
    { key: 'input.mag_y',         label: 'Mag Y',         unit: 'counts' },
    { key: 'input.ctrl_x',        label: 'Ctrl X',        unit: '' },
    { key: 'input.ctrl_y',        label: 'Ctrl Y',        unit: '' },
    { key: 'input.ctrl_throttle', label: 'Throttle',      unit: '' },
    { key: 'input.rssi',          label: 'RSSI (brain)',  unit: 'dBm' },
    { key: 'input.batt_offset',   label: 'Batt Offset',   unit: 'LSB' },
  ],
  State: [
    { key: 'rep.est_theta',        label: 'θ (replayed)',      unit: 'rad' },
    { key: 'rep.est_omega',        label: 'ω (replayed)',      unit: 'rad/s' },
    { key: 'rep.mag_angle',        label: 'Mag angle',         unit: 'rad' },
    { key: 'rep.derot_i',          label: 'Derot I',           unit: 'µT' },
    { key: 'rep.derot_q',          label: 'Derot Q',           unit: 'µT' },
    { key: 'rep.dshot_left',       label: 'DShot L (rep)',     unit: '' },
    { key: 'rep.dshot_right',      label: 'DShot R (rep)',     unit: '' },
    { key: 'real.est_theta',       label: 'θ (real)',          unit: 'rad' },
    { key: 'real.est_omega',       label: 'ω (real)',          unit: 'rad/s' },
    { key: 'real.dshot_left',      label: 'DShot L (real)',    unit: '' },
    { key: 'real.dshot_right',     label: 'DShot R (real)',    unit: '' },
  ],
  Variables: [
    { key: 'rep.omega_from_accel', label: 'ω from accel',      unit: 'rad/s' },
    { key: 'rep.centripetal_ms2',  label: 'Centripetal',       unit: 'm/s²' },
    { key: 'rep.batt_voltage',     label: 'Battery',           unit: 'V' },
    { key: 'rep.erpm_left',        label: 'eRPM L',            unit: 'RPM' },
    { key: 'rep.erpm_right',       label: 'eRPM R',            unit: 'RPM' },
  ],
} as const;
```

- [ ] **Step 2: Create src/hooks/useKeyboard.ts**

```typescript
import { useEffect, useRef, useState } from 'react';
import { invoke } from '@tauri-apps/api/core';

const ALPHA = 0.03;  // ~1.8 Hz bandwidth at 60 Hz

export function useKeyboard(mode: number) {
  const target  = useRef({ x: 0, y: 0, theta: 0, throttle: 0 });
  const filtered = useRef({ x: 0, y: 0, theta: 0, throttle: 0 });
  const keys    = useRef(new Set<string>());
  const rafRef  = useRef<number>();

  useEffect(() => {
    const onKeyDown = (e: KeyboardEvent) => {
      if (e.repeat) return;
      keys.current.add(e.key);
    };
    const onKeyUp   = (e: KeyboardEvent) => keys.current.delete(e.key);
    window.addEventListener('keydown', onKeyDown);
    window.addEventListener('keyup',   onKeyUp);

    let frame = 0;
    const tick = () => {
      const t = target.current;
      // Update targets from key state
      t.x     = keys.current.has('a') ? -127 : keys.current.has('d') ?  127 : 0;
      t.y     = keys.current.has('w') ?  127 : keys.current.has('s') ? -127 : 0;
      t.theta = keys.current.has('ArrowLeft') ? -127 : keys.current.has('ArrowRight') ? 127 : 0;
      if (keys.current.has('ArrowUp'))   t.throttle = Math.min(255, t.throttle + 2);
      if (keys.current.has('ArrowDown')) t.throttle = Math.max(0,   t.throttle - 2);

      // Low-pass filter
      const f = filtered.current;
      f.x        += ALPHA * (t.x     - f.x);
      f.y        += ALPHA * (t.y     - f.y);
      f.theta    += ALPHA * (t.theta - f.theta);
      f.throttle  = t.throttle;

      // Send to backend every other frame (~30 Hz)
      if (frame++ % 2 === 0 && mode !== 0) {
        invoke('set_controls', {
          x:        Math.round(f.x),
          y:        Math.round(f.y),
          theta:    Math.round(f.theta),
          throttle: Math.round(f.throttle),
        });
      }

      rafRef.current = requestAnimationFrame(tick);
    };
    rafRef.current = requestAnimationFrame(tick);

    return () => {
      window.removeEventListener('keydown', onKeyDown);
      window.removeEventListener('keyup',   onKeyUp);
      if (rafRef.current) cancelAnimationFrame(rafRef.current);
    };
  }, [mode]);
}
```

- [ ] **Step 3: Create src/hooks/useAppState.ts**

```typescript
import { useState, useEffect, useCallback } from 'react';
import { listen } from '@tauri-apps/api/event';
import { invoke } from '@tauri-apps/api/core';
import { LiveUpdate, SourceStatus, Mode } from '../types/sunshine';

export function useAppState() {
  const [mode,            setModeState]   = useState<Mode>(0);
  const [liveUpdate,      setLiveUpdate]  = useState<LiveUpdate | null>(null);
  const [sourceStatus,    setSourceStatus] = useState<SourceStatus>({ kind: 'Disconnected', detail: '' });
  const [loggingActive,   setLogging]     = useState(false);
  const [logPath,         setLogPath]     = useState('');
  const [rxRssi,          setRxRssi]      = useState<number>(-127);

  useEffect(() => {
    const unsub1 = listen<LiveUpdate>('live_update',   e => setLiveUpdate(e.payload));
    const unsub2 = listen<SourceStatus>('source_status', e => setSourceStatus(e.payload));
    const unsub3 = listen<number>('rx_rssi',           e => setRxRssi(e.payload));
    return () => { unsub1.then(f=>f()); unsub2.then(f=>f()); unsub3.then(f=>f()); };
  }, []);

  const setMode = useCallback((m: Mode) => {
    setModeState(m);
    invoke('set_mode', { mode: m });
    if (m === 0) invoke('set_controls', { x:0, y:0, theta:0, throttle:0 });
  }, []);

  const enableLogging = useCallback(async (label: string) => {
    const path = await invoke<string>('enable_logging', { label });
    setLogging(true);
    setLogPath(path);
  }, []);

  const disableLogging = useCallback(() => {
    invoke('disable_logging');
    setLogging(false);
  }, []);

  return { mode, setMode, liveUpdate, sourceStatus, loggingActive, logPath,
           enableLogging, disableLogging, rxRssi };
}
```

- [ ] **Step 4: Create src/App.tsx**

```tsx
import React from 'react';
import { useAppState } from './hooks/useAppState';
import { useKeyboard } from './hooks/useKeyboard';
import Header from './components/Header';
import ConnectionPanel from './components/ConnectionPanel';
import GraphPanel from './components/GraphPanel';
import './App.css';

export default function App() {
  const state = useAppState();
  useKeyboard(state.mode);

  return (
    <div className="app">
      <Header {...state} />
      <div className="main-layout">
        <ConnectionPanel sourceStatus={state.sourceStatus} />
        <GraphPanel />
      </div>
    </div>
  );
}
```

- [ ] **Step 5: Create src/components/Header.tsx**

```tsx
import React, { useState } from 'react';
import ModeButtons from './ModeButtons';
import StatusBar from './StatusBar';
import LoggingControl from './LoggingControl';
import type { Mode, LiveUpdate } from '../types/sunshine';

interface Props {
  mode:           Mode;
  setMode:        (m: Mode) => void;
  liveUpdate:     LiveUpdate | null;
  rxRssi:         number;
  loggingActive:  boolean;
  logPath:        string;
  enableLogging:  (label: string) => void;
  disableLogging: () => void;
}

export default function Header(p: Props) {
  return (
    <header className="header">
      <ModeButtons mode={p.mode} setMode={p.setMode} />
      <StatusBar update={p.liveUpdate} rxRssi={p.rxRssi} />
      <LoggingControl active={p.loggingActive} path={p.logPath}
                      onEnable={p.enableLogging} onDisable={p.disableLogging} />
    </header>
  );
}
```

- [ ] **Step 6: Create src/components/ModeButtons.tsx**

```tsx
import React from 'react';
import type { Mode } from '../types/sunshine';

const MODES: { label: string; value: Mode; color: string }[] = [
  { label: 'DISABLED', value: 0, color: '#e74c3c' },
  { label: 'TANK',     value: 1, color: '#f39c12' },
  { label: 'MELTY',    value: 2, color: '#2ecc71' },
];

export default function ModeButtons({ mode, setMode }: { mode: Mode; setMode: (m: Mode) => void }) {
  return (
    <div className="mode-buttons">
      {MODES.map(m => (
        <button
          key={m.value}
          className={`mode-btn ${mode === m.value ? 'active' : ''}`}
          style={{ '--btn-color': m.color } as React.CSSProperties}
          onClick={() => setMode(m.value)}
        >
          {m.label}
        </button>
      ))}
    </div>
  );
}
```

- [ ] **Step 7: Create src/components/StatusBar.tsx**

```tsx
import React from 'react';
import type { LiveUpdate } from '../types/sunshine';

function battColor(offset: number): string {
  const v = 7.6 + offset * 0.0205;
  if (v >= 8.0) return '#2ecc71';
  if (v >= 7.4) return '#f39c12';
  if (v >= 7.0) return '#e67e22';
  return '#e74c3c';
}

export default function StatusBar({ update, rxRssi }: { update: LiveUpdate | null; rxRssi: number }) {
  const v = update ? (7.6 + update.batt_offset * 0.0205).toFixed(2) : '--';
  const omega_rpm = update ? (update.est_omega * 60 / (2 * Math.PI)).toFixed(0) : '--';
  const color = update ? battColor(update.batt_offset) : '#666';

  return (
    <div className="status-bar">
      <span className="status-item">
        <span className="status-label">BATT</span>
        <span className="status-value" style={{ color }}>{v} V</span>
      </span>
      <span className="status-item">
        <span className="status-label">RPM</span>
        <span className="status-value">{omega_rpm}</span>
      </span>
      <span className="status-item">
        <span className="status-label">RSSI(brain)</span>
        <span className="status-value">{update?.rssi ?? '--'} dBm</span>
      </span>
      <span className="status-item">
        <span className="status-label">RSSI(rx)</span>
        <span className="status-value">{rxRssi} dBm</span>
      </span>
    </div>
  );
}
```

- [ ] **Step 8: Create src/components/LoggingControl.tsx**

```tsx
import React, { useState } from 'react';

interface Props {
  active:     boolean;
  path:       string;
  onEnable:   (label: string) => void;
  onDisable:  () => void;
}

export default function LoggingControl({ active, path, onEnable, onDisable }: Props) {
  const [label, setLabel] = useState('');
  return (
    <div className="logging-control">
      {active ? (
        <>
          <span className="log-indicator active">● REC</span>
          <span className="log-path" title={path}>{path.split('/').pop()}</span>
          <button onClick={onDisable} className="btn-stop-log">Stop</button>
        </>
      ) : (
        <>
          <input
            className="log-label-input"
            placeholder="label (optional)"
            value={label}
            onChange={e => setLabel(e.target.value)}
            onKeyDown={e => e.key === 'Enter' && onEnable(label)}
          />
          <button onClick={() => onEnable(label)} className="btn-start-log">Log</button>
        </>
      )}
    </div>
  );
}
```

- [ ] **Step 9: Create src/components/ConnectionPanel.tsx**

```tsx
import React, { useState, useEffect } from 'react';
import { invoke } from '@tauri-apps/api/core';
import type { SourceStatus } from '../types/sunshine';

export default function ConnectionPanel({ sourceStatus }: { sourceStatus: SourceStatus }) {
  const [tab, setTab] = useState<'live' | 'replay' | 'sim'>('live');
  const [ports, setPorts] = useState<string[]>([]);
  const [port, setPort] = useState('');
  const [replayMeta, setReplayMeta] = useState<any>(null);

  useEffect(() => {
    invoke<string[]>('list_serial_ports').then(setPorts);
  }, [tab]);

  return (
    <div className="connection-panel">
      <div className="tab-bar">
        {(['live','replay','sim'] as const).map(t => (
          <button key={t} className={`tab ${tab===t?'active':''}`} onClick={() => setTab(t)}>
            {t.toUpperCase()}
          </button>
        ))}
      </div>

      {tab === 'live' && (
        <div className="tab-content">
          <select value={port} onChange={e => setPort(e.target.value)} className="port-select">
            <option value="">Select port…</option>
            {ports.map(p => <option key={p} value={p}>{p}</option>)}
          </select>
          <button onClick={() => invoke('connect_serial', { port })} disabled={!port} className="btn-connect">
            Connect
          </button>
          <div className={`conn-status ${sourceStatus.kind === 'Live' ? 'connected' : 'disconnected'}`}>
            {sourceStatus.detail || sourceStatus.kind}
          </div>
        </div>
      )}

      {tab === 'replay' && (
        <div className="tab-content">
          <button className="btn-file-pick" onClick={async () => {
            const { open } = await import('@tauri-apps/plugin-dialog');
            const path = await open({ filters: [{ name: 'Sun log', extensions: ['sun'] }] });
            if (typeof path === 'string') {
              const meta = await invoke('open_replay', { path });
              setReplayMeta(meta);
            }
          }}>Open .sun file</button>
          {replayMeta && (
            <div className="replay-meta">
              <div>Label: {replayMeta.label || '(none)'}</div>
              <div>Frames: {replayMeta.frame_count}</div>
              <div>Schema v{replayMeta.schema_version}</div>
            </div>
          )}
        </div>
      )}

      {tab === 'sim' && (
        <div className="tab-content">
          <button className="btn-connect" onClick={() => invoke('start_simulation')}>Start Simulation</button>
          <button className="btn-stop"    onClick={() => invoke('stop_source')}>Stop</button>
          <div className="sim-params">
            <div>KV: 1100 RPM/V</div>
            <div>MoI: 1.214×10⁻³ kg·m²</div>
            <div>Battery: 2S 8.4V</div>
          </div>
        </div>
      )}
    </div>
  );
}
```

- [ ] **Step 10: Create src/components/GraphPanel.tsx + UPlotCanvas.tsx + ChannelSelector.tsx**

```tsx
// src/components/ChannelSelector.tsx
import React from 'react';
import { CHANNELS } from '../types/sunshine';

interface Props {
  selected: string[];
  onToggle: (key: string) => void;
}

export default function ChannelSelector({ selected, onToggle }: Props) {
  return (
    <div className="channel-selector">
      {(Object.entries(CHANNELS) as [string, readonly {key:string;label:string;unit:string}[]][]).map(([group, chans]) => (
        <details key={group} open>
          <summary className="channel-group">{group}</summary>
          {chans.map(ch => (
            <label key={ch.key} className="channel-item">
              <input type="checkbox" checked={selected.includes(ch.key)} onChange={() => onToggle(ch.key)} />
              <span>{ch.label}</span>
              {ch.unit && <span className="unit">{ch.unit}</span>}
            </label>
          ))}
        </details>
      ))}
    </div>
  );
}
```

```tsx
// src/components/UPlotCanvas.tsx
import React, { useRef, useEffect, useCallback } from 'react';
import uPlot from 'uplot';
import 'uplot/dist/uPlot.min.css';
import { invoke } from '@tauri-apps/api/core';

interface Props {
  channels: string[];
  width:    number;
  height:   number;
}

const COLORS = ['#3498db','#e74c3c','#2ecc71','#f39c12','#9b59b6','#1abc9c','#e67e22','#34495e'];

export default function UPlotCanvas({ channels, width, height }: Props) {
  const divRef  = useRef<HTMLDivElement>(null);
  const uRef    = useRef<uPlot | null>(null);
  const viewRef = useRef({ startUs: 0, endUs: Date.now() * 1000 });

  const fetchAndDraw = useCallback(async () => {
    if (!uRef.current || channels.length === 0) return;
    const { startUs, endUs } = viewRef.current;
    const maxPts = Math.round(width);

    const series: number[][] = [[]];
    let times: number[] = [];

    for (const ch of channels) {
      const pts = await invoke<[number, number][]>('get_graph_data', {
        channel: ch, startUs, endUs, maxPoints: maxPts
      });
      if (times.length === 0) times = pts.map(([t]) => t / 1e6);
      series.push(pts.map(([, v]) => v));
    }
    series[0] = times;
    uRef.current.setData(series as any);
  }, [channels, width]);

  useEffect(() => {
    if (!divRef.current) return;

    const opts: uPlot.Options = {
      width,
      height,
      cursor: { drag: { x: true, y: false } },
      scales: { x: { time: true } },
      axes: [
        { stroke: '#888', grid: { stroke: '#333' } },
        { stroke: '#888', grid: { stroke: '#333' } },
      ],
      series: [
        {},
        ...channels.map((ch, i) => ({
          label:  ch.split('.').pop() ?? ch,
          stroke: COLORS[i % COLORS.length],
          width:  1.5,
        })),
      ],
    };

    uRef.current = new uPlot(opts, [[], ...channels.map(() => [])], divRef.current!);
    fetchAndDraw();

    return () => { uRef.current?.destroy(); uRef.current = null; };
  }, [channels, width, height]);

  // Ctrl+scroll to zoom, scroll to pan
  useEffect(() => {
    const el = divRef.current;
    if (!el) return;
    const onWheel = (e: WheelEvent) => {
      e.preventDefault();
      const { startUs, endUs } = viewRef.current;
      const span = endUs - startUs;
      if (e.ctrlKey) {
        // Zoom: shrink/expand span by 10% per tick
        const factor = e.deltaY > 0 ? 1.1 : 0.9;
        const mid    = (startUs + endUs) / 2;
        viewRef.current = { startUs: mid - span*factor/2, endUs: mid + span*factor/2 };
      } else {
        // Pan: shift by 5% per tick
        const shift = span * 0.05 * Math.sign(e.deltaY);
        viewRef.current = { startUs: startUs + shift, endUs: endUs + shift };
      }
      fetchAndDraw();
    };
    el.addEventListener('wheel', onWheel, { passive: false });
    return () => el.removeEventListener('wheel', onWheel);
  }, [fetchAndDraw]);

  return <div ref={divRef} className="uplot-wrap" />;
}
```

```tsx
// src/components/GraphPanel.tsx
import React, { useState, useRef } from 'react';
import ChannelSelector from './ChannelSelector';
import UPlotCanvas from './UPlotCanvas';

export default function GraphPanel() {
  const [selected, setSelected] = useState<string[]>(['rep.est_theta', 'rep.est_omega']);
  const containerRef = useRef<HTMLDivElement>(null);

  const toggle = (key: string) =>
    setSelected(prev => prev.includes(key) ? prev.filter(k => k !== key) : [...prev, key]);

  return (
    <div className="graph-panel" ref={containerRef}>
      <ChannelSelector selected={selected} onToggle={toggle} />
      <UPlotCanvas channels={selected} width={900} height={400} />
    </div>
  );
}
```

- [ ] **Step 11: Create App.css with dark theme matching mockup**

Fetch the mockup and extract CSS variables:
```bash
# Fetch the mockup HTML to extract the design tokens
curl "https://api.anthropic.com/v1/design/h/ER_Esv7z8Q_IkM8_wpP-5Q?open_file=Sunshine+Dashboard.html" \
  -o /tmp/sunshine_mockup.html 2>/dev/null || true
```

Apply the core dark theme. Replace `sunshine_app/src/App.css`:
```css
:root {
  --bg-primary:    #0d0d0d;
  --bg-secondary:  #141414;
  --bg-card:       #1a1a1a;
  --bg-elevated:   #222222;
  --text-primary:  #e8e8e8;
  --text-muted:    #888;
  --accent-green:  #2ecc71;
  --accent-red:    #e74c3c;
  --accent-yellow: #f39c12;
  --accent-blue:   #3498db;
  --border:        #2a2a2a;
  --border-bright: #3a3a3a;
}

* { box-sizing: border-box; margin: 0; padding: 0; }
body { background: var(--bg-primary); color: var(--text-primary);
       font-family: 'JetBrains Mono', 'Fira Code', monospace; font-size: 13px; }

.app { display: flex; flex-direction: column; height: 100vh; overflow: hidden; }

.header { display: flex; align-items: center; gap: 16px; padding: 8px 16px;
          background: var(--bg-secondary); border-bottom: 1px solid var(--border); }

.mode-buttons { display: flex; gap: 8px; }
.mode-btn { padding: 6px 18px; border: 2px solid var(--btn-color, #555);
            background: transparent; color: var(--btn-color, #aaa);
            border-radius: 4px; cursor: pointer; font: inherit; font-weight: 700;
            text-transform: uppercase; letter-spacing: 0.08em; transition: all 0.1s; }
.mode-btn.active { background: var(--btn-color); color: #000; }
.mode-btn:hover { opacity: 0.85; }

.status-bar { display: flex; gap: 20px; flex: 1; }
.status-item { display: flex; flex-direction: column; align-items: center; }
.status-label { font-size: 9px; color: var(--text-muted); text-transform: uppercase; letter-spacing: 0.1em; }
.status-value { font-size: 14px; font-weight: 600; }

.logging-control { display: flex; align-items: center; gap: 8px; }
.log-label-input { background: var(--bg-elevated); border: 1px solid var(--border-bright);
                   color: var(--text-primary); padding: 4px 8px; border-radius: 3px;
                   font: inherit; width: 140px; }
.btn-start-log, .btn-stop-log { padding: 4px 10px; border-radius: 3px; font: inherit;
                                 cursor: pointer; border: none; }
.btn-start-log { background: var(--accent-green); color: #000; }
.btn-stop-log  { background: var(--accent-red);   color: #fff; }
.log-indicator.active { color: var(--accent-red); animation: blink 1s step-end infinite; }
.log-path { color: var(--text-muted); font-size: 11px; max-width: 200px;
            overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
@keyframes blink { 50% { opacity: 0; } }

.main-layout { display: flex; flex: 1; overflow: hidden; }

.connection-panel { width: 240px; background: var(--bg-secondary); border-right: 1px solid var(--border);
                    padding: 12px; display: flex; flex-direction: column; gap: 8px; }
.tab-bar { display: flex; gap: 4px; margin-bottom: 8px; }
.tab { flex: 1; padding: 4px; background: var(--bg-elevated); border: 1px solid var(--border);
       color: var(--text-muted); cursor: pointer; font: inherit; border-radius: 3px; }
.tab.active { background: var(--accent-blue); color: #fff; border-color: var(--accent-blue); }
.tab-content { display: flex; flex-direction: column; gap: 8px; }
.port-select { background: var(--bg-elevated); border: 1px solid var(--border-bright);
               color: var(--text-primary); padding: 4px 8px; border-radius: 3px; font: inherit; width: 100%; }
.btn-connect { background: var(--accent-blue); color: #fff; border: none;
               padding: 6px; border-radius: 3px; cursor: pointer; font: inherit; font-weight: 600; }
.btn-stop { background: var(--accent-red); color: #fff; border: none;
            padding: 6px; border-radius: 3px; cursor: pointer; font: inherit; }
.conn-status { padding: 4px 8px; border-radius: 3px; font-size: 11px; text-align: center; }
.conn-status.connected    { background: rgba(46,204,113,0.15); color: var(--accent-green); }
.conn-status.disconnected { background: rgba(231,76,60,0.15);  color: var(--accent-red);   }
.replay-meta { font-size: 11px; color: var(--text-muted); display: flex; flex-direction: column; gap: 2px; }
.btn-file-pick { background: var(--bg-elevated); border: 1px solid var(--border-bright);
                 color: var(--text-primary); padding: 6px; border-radius: 3px; cursor: pointer; font: inherit; }
.sim-params { font-size: 11px; color: var(--text-muted); display: flex; flex-direction: column; gap: 2px; }

.graph-panel { flex: 1; display: flex; overflow: hidden; }

.channel-selector { width: 180px; overflow-y: auto; padding: 8px; background: var(--bg-secondary);
                    border-right: 1px solid var(--border); }
.channel-group { font-size: 10px; text-transform: uppercase; letter-spacing: 0.1em;
                 color: var(--text-muted); cursor: pointer; padding: 4px 0; }
.channel-item { display: flex; align-items: center; gap: 6px; padding: 2px 0;
                cursor: pointer; font-size: 11px; }
.channel-item input { accent-color: var(--accent-blue); }
.unit { color: var(--text-muted); font-size: 10px; margin-left: auto; }

.uplot-wrap { flex: 1; padding: 8px; background: var(--bg-primary); }
.uplot-wrap .u-wrap { background: var(--bg-primary) !important; }
.uplot-wrap .u-title { color: var(--text-primary) !important; }
```

- [ ] **Step 12: Final build + launch**

```bash
cd sunshine_app && npm run tauri dev
```
Expected:
- App window opens with dark theme
- Header shows DISABLED / TANK / MELTY buttons
- Connection panel shows Live / Replay / Sim tabs
- Graph panel shows channel selector and empty plot
- Press W → Ctrl Y ramps in channel selector if graphed
- Connect to receiver (after brain bringup 3) → data flows

- [ ] **Step 13: Commit**

```bash
git add sunshine_app/src/
git commit -m "feat(app): complete React UI — header, connection panel, graph panel with uPlot"
```

---

*End of sunshine_app plan.*

*All four plans are complete. Build order: `sunshine_core` → `sunshine_receiver` (parallel) → `sunshine_brain` → `sunshine_app`.*
