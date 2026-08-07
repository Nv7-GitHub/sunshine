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
use std::sync::atomic::AtomicBool;
use parking_lot::Mutex;
use pipeline::Pipeline;
use controls::{ControlState, KeyTargets};
use serial::SerialConnection;

pub struct AppState {
    pub pipeline:     Arc<Mutex<Pipeline>>,
    pub controls:     Arc<Mutex<ControlState>>,
    pub sim_stop:     Arc<AtomicBool>,
    pub serial_conn:  Arc<Mutex<Option<SerialConnection>>>,
    pub key_targets:  Arc<Mutex<KeyTargets>>,
}

fn move_toward(cur: f32, target: f32, step: f32) -> f32 {
    if cur < target { (cur + step).min(target) }
    else if cur > target { (cur - step).max(target) }
    else { cur }
}

/// Native control-ramp loop. Mirrors the tuning that lived in useKeyboard.ts
/// (T_RISE 1.5 s, T_FALL 9 s, theta ~0.66 s, throttle 90 counts/s, 30 Hz send)
/// but runs in a std::thread so webview timer throttling can never again
/// freeze the ramps mid-press (measured stalls up to 8.6 s in the
/// Arewedoneyet log — the driver was pinned at partial deflection all
/// session). The webview now only reports key up/down events.
fn spawn_control_loop(controls: Arc<Mutex<ControlState>>,
                      serial: Arc<Mutex<Option<SerialConnection>>>,
                      keys: Arc<Mutex<KeyTargets>>) {
    std::thread::spawn(move || {
        const T_RISE: f32 = 1.5;
        const T_FALL: f32 = 1.5;  // was 9.0: a 9 s coast after release only made sense when the robot barely responded
        const RATE_THETA: f32 = (127.0 / 40.0) * 60.0;
        const RATE_THROTTLE: f32 = 90.0;
        const SEND_HZ: f32 = 30.0;
        let (mut fx, mut fy, mut fth, mut fthr) = (0f32, 0f32, 0f32, 0f32);
        let mut last = std::time::Instant::now();
        let mut send_acc = 0f32;
        loop {
            std::thread::sleep(std::time::Duration::from_millis(5));
            let now = std::time::Instant::now();
            let dt = (now - last).as_secs_f32().min(0.1);
            last = now;
            let k = *keys.lock();
            let mode = controls.lock().mode;
            if mode == 0 {
                // DISABLED is the panic stop: hard-zero the filter state so
                // re-arming never hands the robot stale deflection.
                fx = 0.0; fy = 0.0; fth = 0.0; fthr = 0.0;
            } else {
                let tx = k.x as f32 * 127.0;
                let ty = k.y as f32 * 127.0;
                let tth = k.theta as f32 * 127.0;
                let rate_for = |t: f32| if t == 0.0 { 127.0 / T_FALL } else { 127.0 / T_RISE };
                fx = move_toward(fx, tx, rate_for(tx) * dt);
                fy = move_toward(fy, ty, rate_for(ty) * dt);
                fth = move_toward(fth, tth, RATE_THETA * dt);
                if fth.abs() < 0.4 { fth = 0.0; }
                fthr = (fthr + k.thr as f32 * RATE_THROTTLE * dt).clamp(0.0, 255.0);
                // Unit-circle clamp so diagonals match cardinal magnitude.
                let mag = (fx * fx + fy * fy).sqrt();
                if mag > 127.0 { fx = fx / mag * 127.0; fy = fy / mag * 127.0; }
                let mut c = controls.lock();
                c.ctrl_x = fx.round() as i8;
                c.ctrl_y = fy.round() as i8;
                c.ctrl_theta = fth.round() as i8;
                c.ctrl_throttle = fthr.round() as u8;
            }
            send_acc += dt;
            if send_acc >= 1.0 / SEND_HZ {
                send_acc = 0.0;
                let (mode, x, y, th, thr) = {
                    let c = controls.lock();
                    (c.mode, c.ctrl_x, c.ctrl_y, c.ctrl_theta, c.ctrl_throttle)
                };
                if mode != 0 {
                    if let Some(conn) = serial.lock().as_ref() {
                        conn.send(&protocol::encode_ctrl(mode, x, y, th, thr));
                    }
                }
            }
        }
    });
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    let pipeline    = Arc::new(Mutex::new(Pipeline::new()));
    let controls    = Arc::new(Mutex::new(ControlState::default()));
    let sim_stop    = Arc::new(AtomicBool::new(false));
    let serial_conn = Arc::new(Mutex::new(None));
    let key_targets = Arc::new(Mutex::new(KeyTargets::default()));
    spawn_control_loop(controls.clone(), serial_conn.clone(), key_targets.clone());

    tauri::Builder::default()
        .plugin(tauri_plugin_dialog::init())
        .manage(AppState { pipeline, controls, sim_stop, serial_conn, key_targets })
        .invoke_handler(tauri::generate_handler![
            commands::list_serial_ports,
            commands::connect_serial,
            commands::disconnect_serial,
            commands::open_replay,
            commands::load_replay,
            commands::start_simulation,
            commands::stop_source,
            commands::set_mode,
            commands::set_controls,
            commands::set_key_targets,
            commands::enable_logging,
            commands::disable_logging,
            commands::get_graph_data,
            commands::get_graph_data_multi,
            commands::get_channel_snapshot,
        ])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
