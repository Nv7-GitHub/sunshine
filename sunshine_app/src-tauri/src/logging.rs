use crate::ffi::{SunshineInput, SunshineState, schema_version};
use std::fs::File;
use std::io::{BufWriter, Write, Seek, SeekFrom};
use std::mem::size_of;
use std::path::PathBuf;
use std::time::{SystemTime, UNIX_EPOCH};

const MAGIC: &[u8; 5]  = b"SHINE";
// VER 3: two SunshineState snapshots per frame (start + midpoint → 100 Hz real
// state) and NO vars block (vars are a pure function of state+inputs; the host
// recomputes them). VER 2 logs (one state + a vars block) are still readable.
const FILE_FORMAT_VER: u16 = 3;
// Header layout (95 bytes), unchanged across VER 2/3:
//  [0..4]   MAGIC (5)     [5..6]   FILE_FORMAT_VER (2)   [7..8]   HEADER_SIZE (2)
//  [9..12]  schema_ver(4) [13..14] sizeof_input (2)       [15..16] sizeof_state (2)
//  [17..18] sizeof_vars(2)[19..26] created_at_ms (8)      [27]     source (1)
//  [28]     flags (1)     [29..92] label (64)              [93..94] num_inputs (2)
// VER 3 writes sizeof_vars = 0 to signal "no vars block"; states-per-frame is 2
// (implied by VER >= 3). VER 3 frame: frame_id(4) + flags(1) + state_start +
// state_mid + num_inputs×input.
const HEADER_SIZE: u16 = 95;

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

        w.write_all(MAGIC)?;
        w.write_all(&FILE_FORMAT_VER.to_le_bytes())?;
        w.write_all(&HEADER_SIZE.to_le_bytes())?;
        w.write_all(&schema_version().to_le_bytes())?;
        w.write_all(&(size_of::<SunshineInput>() as u16).to_le_bytes())?;
        w.write_all(&(size_of::<SunshineState>() as u16).to_le_bytes())?;
        w.write_all(&0u16.to_le_bytes())?;  // sizeof_vars = 0: VER 3 logs no vars block
        w.write_all(&now_ms.to_le_bytes())?;
        w.write_all(&[source])?;
        w.write_all(&[0u8])?;  // flags: logging_complete=0 until close()

        let mut label_buf = [0u8; 64];
        let bytes = label.as_bytes();
        let n = bytes.len().min(63);
        label_buf[..n].copy_from_slice(&bytes[..n]);
        w.write_all(&label_buf)?;
        // [93..94] num_inputs per frame (added in FILE_FORMAT_VER 2)
        use crate::protocol::INPUTS_PER_FRAME;
        w.write_all(&(INPUTS_PER_FRAME as u16).to_le_bytes())?;

        Ok(LogWriter { writer: w, frame_count: 0, flush_every: 10, path })
    }

    /// Write one VER 3 frame: frame_id + flags + state_start + state_mid + inputs.
    /// No vars block — vars are recomputed by the host from (state, inputs).
    pub fn write_frame(
        &mut self,
        frame_id:  u32,
        state:     &SunshineState,
        state_mid: &SunshineState,
        inputs:    &[SunshineInput],
    ) -> std::io::Result<()> {
        self.writer.write_all(&frame_id.to_le_bytes())?;
        self.writer.write_all(&[0x01u8])?;

        for st in [state, state_mid] {
            let bytes = unsafe {
                std::slice::from_raw_parts(st as *const _ as *const u8, size_of::<SunshineState>())
            };
            self.writer.write_all(bytes)?;
        }

        for inp in inputs {
            let inp_bytes = unsafe {
                std::slice::from_raw_parts(inp as *const _ as *const u8, size_of::<SunshineInput>())
            };
            self.writer.write_all(inp_bytes)?;
        }

        self.frame_count += 1;
        if self.frame_count % self.flush_every == 0 {
            self.writer.flush()?;
        }
        Ok(())
    }

    pub fn close(mut self) -> std::io::Result<()> {
        self.writer.flush()?;
        let mut file = self.writer.into_inner()?;
        file.seek(SeekFrom::Start(28))?;
        file.write_all(&[0x01u8])?;
        Ok(())
    }

    pub fn flush(&mut self) -> std::io::Result<()> {
        self.writer.flush()
    }

    pub fn path(&self) -> &PathBuf { &self.path }
    pub fn frame_count(&self) -> u32 { self.frame_count }
}

pub fn make_log_path(base_dir: &std::path::Path, label: &str) -> PathBuf {
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
    let s   = unix_secs;
    let sec = s % 60; let s = s / 60;
    let min = s % 60; let s = s / 60;
    let hr  = s % 24; let s = s / 24;
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
