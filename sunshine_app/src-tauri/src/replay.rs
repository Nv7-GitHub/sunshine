use crate::ffi::{SunshineInput, SunshineState, SunshineVars};
use crate::protocol::{TelemetryFrame, INPUTS_PER_FRAME};
use std::fs::File;
use std::io::{Read, Seek, SeekFrom};
use std::mem::size_of;
use std::path::PathBuf;

#[derive(Debug, Clone)]
pub struct LogMetadata {
    pub path:           PathBuf,
    pub header_size:    u64,
    pub schema_version: u32,
    pub sizeof_input:   u16,
    pub sizeof_state:   u16,
    pub sizeof_vars:    u16,
    pub num_inputs:     u16,  // inputs per frame; 6 for VER 2, defaulted to 20 for VER 1
    pub created_at_ms:  u64,
    pub source:         u8,
    pub label:          String,
    pub frame_count:    u32,
}

pub fn read_metadata(path: &PathBuf) -> std::io::Result<LogMetadata> {
    let mut f = File::open(path)?;
    // Read up to 95 bytes (VER-2 header); VER-1 files are 93 bytes
    let mut hdr = [0u8; 95];
    let bytes_read = f.read(&mut hdr)?;
    if bytes_read < 93 {
        return Err(std::io::Error::new(std::io::ErrorKind::InvalidData, "header too short"));
    }

    if &hdr[0..5] != b"SHINE" {
        return Err(std::io::Error::new(std::io::ErrorKind::InvalidData, "bad magic"));
    }
    let file_ver      = u16::from_le_bytes(hdr[5..7].try_into().unwrap());
    let header_size   = u16::from_le_bytes(hdr[7..9].try_into().unwrap()) as u64;
    let schema_ver    = u32::from_le_bytes(hdr[9..13].try_into().unwrap());
    let sizeof_input  = u16::from_le_bytes(hdr[13..15].try_into().unwrap());
    let sizeof_state  = u16::from_le_bytes(hdr[15..17].try_into().unwrap());
    let sizeof_vars   = u16::from_le_bytes(hdr[17..19].try_into().unwrap());
    let created_at_ms = u64::from_le_bytes(hdr[19..27].try_into().unwrap());
    let source        = hdr[27];
    let label_end     = hdr[29..93].iter().position(|&b| b == 0).unwrap_or(64);
    let label         = String::from_utf8_lossy(&hdr[29..29+label_end]).to_string();

    // VER 2 adds num_inputs at [93..94]; VER 1 always had 20 inputs per frame
    let num_inputs: u16 = if file_ver >= 2 && bytes_read >= 95 {
        u16::from_le_bytes(hdr[93..95].try_into().unwrap())
    } else {
        20
    };

    let file_size     = f.seek(SeekFrom::End(0))?;
    let frame_size    = 5u64 + sizeof_state as u64 + sizeof_input as u64 * num_inputs as u64 + sizeof_vars as u64;
    let frame_count   = if frame_size > 0 { ((file_size - header_size) / frame_size) as u32 } else { 0 };

    Ok(LogMetadata { path: path.clone(), header_size, schema_version: schema_ver,
                     sizeof_input, sizeof_state, sizeof_vars, num_inputs,
                     created_at_ms, source, label, frame_count })
}

pub fn read_frame(f: &mut File, meta: &LogMetadata) -> std::io::Result<(TelemetryFrame, SunshineVars)> {
    use crate::protocol::INPUTS_PER_FRAME;

    let mut id_flags = [0u8; 5];
    f.read_exact(&mut id_flags)?;
    let frame_id = u32::from_le_bytes(id_flags[0..4].try_into().unwrap());

    let state = read_padded::<SunshineState>(f, meta.sizeof_state as usize)?;

    // Read meta.num_inputs from file; fit into INPUTS_PER_FRAME slots (0-pad if file has fewer)
    let mut inputs = [SunshineInput::default(); INPUTS_PER_FRAME];
    let n = (meta.num_inputs as usize).min(INPUTS_PER_FRAME);
    for inp in inputs[..n].iter_mut() {
        *inp = read_padded::<SunshineInput>(f, meta.sizeof_input as usize)?;
    }
    // Skip any extra inputs from old files that had more than INPUTS_PER_FRAME
    for _ in n..meta.num_inputs as usize {
        let mut skip = vec![0u8; meta.sizeof_input as usize];
        f.read_exact(&mut skip)?;
    }

    let vars = read_padded::<SunshineVars>(f, meta.sizeof_vars as usize)?;

    Ok((TelemetryFrame { frame_id: frame_id as u16, state, inputs }, vars))
}

/// Returns the (first_input_time_us, last_input_time_us) for a log file.
pub fn log_time_range(meta: &LogMetadata) -> std::io::Result<(u64, u64)> {
    if meta.frame_count == 0 {
        return Ok((0, 0));
    }

    let mut f = File::open(&meta.path)?;

    f.seek(SeekFrom::Start(meta.header_size))?;
    let (first_frame, _) = read_frame(&mut f, meta)?;
    let start_us = first_frame.inputs[0].time_us as u64;

    let frame_size = 5u64
        + meta.sizeof_state as u64
        + meta.sizeof_input as u64 * meta.num_inputs as u64
        + meta.sizeof_vars as u64;
    let last_pos = meta.header_size + (meta.frame_count as u64 - 1) * frame_size;
    f.seek(SeekFrom::Start(last_pos))?;
    let (last_frame, _) = read_frame(&mut f, meta)?;
    let n = (meta.num_inputs as usize).min(INPUTS_PER_FRAME);
    let last_us = last_frame.inputs[n.saturating_sub(1)].time_us as u64;

    // Handle u32 hardware-clock wrap (sessions longer than ~4295 s)
    let end_us = if last_us < start_us { last_us + 0x1_0000_0000 } else { last_us };

    Ok((start_us, end_us))
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
