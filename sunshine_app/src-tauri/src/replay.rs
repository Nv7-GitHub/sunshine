use crate::ffi::{SunshineInput, SunshineState, SunshineVars};
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
    pub frame_count:    u32,
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
