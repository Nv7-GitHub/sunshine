use crate::protocol::{FrameParser, ReceiverFrame, parse_frame};
use std::sync::{Arc, atomic::{AtomicBool, Ordering}};
use std::thread;
use std::time::{Duration, Instant};
use parking_lot::Mutex;
use std::io::{self, Write};

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

        let port          = Arc::new(Mutex::new(port));
        let running       = Arc::new(AtomicBool::new(true));
        let port_rx       = port.clone();
        let running2      = running.clone();
        let port_name_str = port_name.to_string();

        thread::spawn(move || {
            let mut parser = FrameParser::new();
            let mut buf    = [0u8; 1024];
            while running2.load(Ordering::Relaxed) {
                let result = {
                    let mut p = port_rx.lock();
                    p.read(&mut buf)
                };
                match result {
                    Ok(0) => {
                        thread::sleep(Duration::from_millis(1));
                    }
                    Ok(n) => {
                        for &byte in &buf[..n] {
                            if let Some((t, payload)) = parser.feed(byte) {
                                if let Some(frame) = parse_frame(t, &payload) {
                                    on_frame(frame);
                                }
                            }
                        }
                    }
                    Err(e) if e.kind() == io::ErrorKind::TimedOut
                           || e.kind() == io::ErrorKind::WouldBlock => {
                        thread::sleep(Duration::from_millis(1));
                    }
                    Err(_) => {
                        // Port dropped — device reset or flash started.
                        on_frame(ReceiverFrame::SerialLost);

                        // Poll for the port to come back for up to 30 s.
                        let deadline = Instant::now() + Duration::from_secs(30);
                        let mut ok = false;
                        while running2.load(Ordering::Relaxed) && Instant::now() < deadline {
                            thread::sleep(Duration::from_millis(500));
                            if let Ok(new_port) = serialport::new(&port_name_str, 921600)
                                .timeout(Duration::from_millis(10))
                                .open()
                            {
                                *port_rx.lock() = new_port;
                                parser = FrameParser::new();
                                on_frame(ReceiverFrame::SerialReconnected);
                                ok = true;
                                break;
                            }
                        }
                        if !ok { break; }
                    }
                }
            }
        });

        Ok(SerialConnection { port, running })
    }

    pub fn send(&self, data: &[u8]) {
        let _ = self.port.lock().write_all(data);
    }

    // Spawns a background thread that calls data_fn every `interval` and writes
    // the result to the port. Stops automatically when this SerialConnection is
    // dropped (the running flag goes false).
    pub fn send_periodic(&self, interval: Duration, data_fn: impl Fn() -> Vec<u8> + Send + 'static) {
        let port    = self.port.clone();
        let running = self.running.clone();
        thread::spawn(move || {
            while running.load(Ordering::Relaxed) {
                let data = data_fn();
                let _ = port.lock().write_all(&data);
                thread::sleep(interval);
            }
        });
    }

    pub fn close(self) {
        self.running.store(false, Ordering::Relaxed);
    }
}
