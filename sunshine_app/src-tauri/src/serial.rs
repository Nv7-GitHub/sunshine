use crate::protocol::{FrameParser, ReceiverFrame, parse_frame};
use std::sync::{Arc, atomic::{AtomicBool, Ordering}};
use std::thread;
use std::time::Duration;
use parking_lot::Mutex;
use std::io::Write;

pub fn list_ports() -> Vec<String> {
    serialport::available_ports()
        .unwrap_or_default()
        .into_iter()
        .map(|p| p.port_name)
        .collect()
}

pub type FrameCallback = Arc<dyn Fn(ReceiverFrame) + Send + Sync>;

pub struct SerialConnection {
    port:    Arc<Mutex<Box<dyn serialport::SerialPort>>>,
    running: Arc<AtomicBool>,
}

impl Drop for SerialConnection {
    fn drop(&mut self) {
        self.running.store(false, Ordering::Relaxed);
    }
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
