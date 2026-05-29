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
use controls::ControlState;
use serial::SerialConnection;

pub struct AppState {
    pub pipeline:     Arc<Mutex<Pipeline>>,
    pub controls:     Arc<Mutex<ControlState>>,
    pub sim_stop:     Arc<AtomicBool>,
    pub serial_conn:  Arc<Mutex<Option<SerialConnection>>>,
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    let pipeline    = Arc::new(Mutex::new(Pipeline::new()));
    let controls    = Arc::new(Mutex::new(ControlState::default()));
    let sim_stop    = Arc::new(AtomicBool::new(false));
    let serial_conn = Arc::new(Mutex::new(None));

    tauri::Builder::default()
        .plugin(tauri_plugin_dialog::init())
        .manage(AppState { pipeline, controls, sim_stop, serial_conn })
        .invoke_handler(tauri::generate_handler![
            commands::list_serial_ports,
            commands::connect_serial,
            commands::disconnect_serial,
            commands::open_replay,
            commands::start_replay,
            commands::start_simulation,
            commands::stop_source,
            commands::set_mode,
            commands::set_controls,
            commands::enable_logging,
            commands::disable_logging,
            commands::get_graph_data,
            commands::get_channel_snapshot,
        ])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
