#![allow(unused_imports)]

use crate::{AppState,
            serial::{SerialConnection, list_ports as serial_list},
            protocol::{ReceiverFrame, encode_ctrl, TelemetryFrame},
            pipeline::SourceKind,
            replay::{read_metadata, read_frame},
            simulation::Simulation,
            logging::{LogWriter, make_log_path},
            ffi::{brain_step, state_init, SunshineInput, SunshineVars, SunshineState}};
use tauri::{State, AppHandle, Emitter};
use std::sync::Arc;
use std::sync::atomic::Ordering;
use serde::{Serialize, Deserialize};

#[tauri::command]
pub fn list_serial_ports() -> Vec<String> {
    serial_list()
}

#[tauri::command]
pub async fn connect_serial(
    port:  String,
    app:   AppHandle,
    state: State<'_, AppState>,
) -> Result<(), String> {
    let pipeline = state.pipeline.clone();
    let app2     = app.clone();

    let conn = SerialConnection::open(&port, Arc::new(move |frame| {
        match frame {
            ReceiverFrame::Telemetry(telem) => {
                let mut pipe = pipeline.lock();
                pipe.ingest_frame(&telem);
                let update = build_live_update(&telem);
                drop(pipe);
                let _ = app2.emit("live_update", update);
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

    *state.serial_conn.lock() = Some(conn);
    Ok(())
}

#[tauri::command]
pub fn disconnect_serial(state: State<'_, AppState>) {
    *state.serial_conn.lock() = None;
}

#[tauri::command]
pub fn open_replay(path: String) -> Result<serde_json::Value, String> {
    let path = std::path::PathBuf::from(&path);
    let meta = read_metadata(&path).map_err(|e| e.to_string())?;
    Ok(serde_json::json!({
        "label":            meta.label,
        "created_at_ms":    meta.created_at_ms,
        "frame_count":      meta.frame_count,
        "schema_version":   meta.schema_version,
    }))
}

#[tauri::command]
pub async fn start_simulation(state: State<'_, AppState>, app: AppHandle) -> Result<(), ()> {
    let pipeline  = state.pipeline.clone();
    let controls  = state.controls.clone();
    let stop_flag = state.sim_stop.clone();
    stop_flag.store(false, Ordering::Relaxed);

    let _ = app.emit("source_status", serde_json::json!({
        "kind": "Sim", "detail": "Running"
    }));

    tokio::spawn(async move {
        let mut sim          = Simulation::new();
        let mut replay_state = SunshineState::default();
        state_init(&mut replay_state);
        let mut prev_vars    = SunshineVars::default();

        loop {
            if stop_flag.load(Ordering::Relaxed) { break; }

            let (cx, cy, ct, cth, cmode) = {
                let ctrl = controls.lock();
                (ctrl.ctrl_x, ctrl.ctrl_y, ctrl.ctrl_theta, ctrl.ctrl_throttle, ctrl.mode)
            };

            let mut inputs = [SunshineInput::default(); 20];
            for slot in inputs.iter_mut() {
                let mut input = sim.tick(&prev_vars);
                input.ctrl_x        = cx;
                input.ctrl_y        = cy;
                input.ctrl_theta    = ct;
                input.ctrl_throttle = cth;
                input.mode          = cmode;
                let vars = brain_step(&input, &mut replay_state);
                prev_vars = vars;
                *slot = input;
            }

            let telem = TelemetryFrame {
                frame_id: 0,
                state:    replay_state,
                inputs,
            };
            {
                let mut pipe = pipeline.lock();
                pipe.ingest_frame(&telem);
            }
            let update = build_live_update(&telem);
            let _ = app.emit("live_update", update);

            tokio::time::sleep(tokio::time::Duration::from_millis(20)).await;
        }

        let _ = app.emit("source_status", serde_json::json!({
            "kind": "Disconnected", "detail": ""
        }));
    });
    Ok(())
}

#[tauri::command]
pub async fn start_replay(path: String, state: State<'_, AppState>, app: AppHandle) -> Result<(), String> {
    use crate::replay::{read_metadata, read_frame};
    use std::fs::File;
    use std::io::{Seek, SeekFrom};

    let path_buf = std::path::PathBuf::from(&path);
    let meta     = read_metadata(&path_buf).map_err(|e| e.to_string())?;

    let pipeline  = state.pipeline.clone();
    let stop_flag = state.sim_stop.clone();
    stop_flag.store(false, Ordering::Relaxed);

    let _ = app.emit("source_status", serde_json::json!({
        "kind": "Replay", "detail": "Playing"
    }));

    tokio::spawn(async move {
        let mut f = match File::open(&meta.path) {
            Ok(f) => f,
            Err(_) => return,
        };
        let _ = f.seek(SeekFrom::Start(93)); // skip fixed-size header

        for _ in 0..meta.frame_count {
            if stop_flag.load(Ordering::Relaxed) { break; }

            match read_frame(&mut f, &meta) {
                Ok((telem, _vars)) => {
                    let update = {
                        let mut pipe = pipeline.lock();
                        pipe.ingest_frame(&telem);
                        build_live_update(&telem)
                    };
                    let _ = app.emit("live_update", update);
                }
                Err(_) => break,
            }

            tokio::time::sleep(tokio::time::Duration::from_millis(20)).await;
        }

        let _ = app.emit("source_status", serde_json::json!({
            "kind": "Disconnected", "detail": ""
        }));
    });

    Ok(())
}

#[tauri::command]
pub fn stop_source(state: State<'_, AppState>) {
    state.pipeline.lock().source = SourceKind::None;
    state.sim_stop.store(true, Ordering::Relaxed);
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
    if let Some(logger) = state.pipeline.lock().logger.take() {
        let _ = logger.close();
    }
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

fn build_live_update(telem: &TelemetryFrame) -> serde_json::Value {
    let frame_id    = telem.frame_id;
    let kf_theta    = telem.state.kf_theta;
    let kf_omega    = telem.state.kf_omega;
    let last        = telem.inputs[19];
    let mode        = last.mode;
    let rssi        = last.rssi;
    let batt_offset = last.batt_offset;
    let time_us     = last.time_us;
    serde_json::json!({
        "frame_id":    frame_id,
        "est_theta":   kf_theta,
        "est_omega":   kf_omega,
        "mode":        mode,
        "rssi":        rssi,
        "batt_offset": batt_offset,
        "time_us":     time_us,
    })
}

#[tauri::command]
pub fn get_channel_snapshot(
    channels: Vec<String>,
    time_us:  Option<u64>,
    state:    State<'_, AppState>,
) -> Vec<Option<f32>> {
    state.pipeline.lock().get_channel_snapshot(&channels, time_us)
}
