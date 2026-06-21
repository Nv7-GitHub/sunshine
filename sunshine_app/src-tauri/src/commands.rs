#![allow(unused_imports)]

use crate::{AppState,
            serial::{SerialConnection, list_ports as serial_list},
            protocol::{ReceiverFrame, encode_ctrl, TelemetryFrame, INPUTS_PER_FRAME, STATUS_BRAIN_CONNECTED},
            pipeline::SourceKind,
            replay::{read_metadata, read_frame},
            simulation::Simulation,
            logging::{LogWriter, make_log_path},
            ffi::{brain_step, state_init, SunshineInput, SunshineVars, SunshineState}};
use tauri::{State, AppHandle, Emitter};
use std::sync::Arc;
use std::sync::atomic::Ordering;
use serde::{Serialize, Deserialize};

fn send_current_controls(state: &AppState) {
    let (mode, x, y, theta, throttle) = {
        let ctrl = state.controls.lock();
        (ctrl.mode, ctrl.ctrl_x, ctrl.ctrl_y, ctrl.ctrl_theta, ctrl.ctrl_throttle)
    };
    if let Some(conn) = state.serial_conn.lock().as_ref() {
        conn.send(&encode_ctrl(mode, x, y, theta, throttle));
    }
}

fn force_disabled(state: &AppState) {
    {
        let mut ctrl = state.controls.lock();
        ctrl.mode = 0;
        ctrl.ctrl_x = 0;
        ctrl.ctrl_y = 0;
        ctrl.ctrl_theta = 0;
        ctrl.ctrl_throttle = 0;
    }
    send_current_controls(state);
}

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
    let pipeline     = state.pipeline.clone();
    let app2         = app.clone();
    let port_name    = port.clone();
    let controls_fc  = state.controls.clone();

    // Drop any existing connection first so the port is released before we try
    // to open it again (e.g. after switching away from live to replay and back).
    *state.serial_conn.lock() = None;

    // Stop any running simulation loop — it ingests frames on its own tokio task
    // independently of `source`, so without this it would keep writing into the
    // same ring concurrently with the live reader thread we're about to start.
    state.sim_stop.store(true, Ordering::Relaxed);

    // Mark the source Live BEFORE opening the port, so frames arriving on the
    // reader thread are re-anchored from their transmitted state (ingest_frame).
    {
        let mut pipe = state.pipeline.lock();
        pipe.begin_live();
        pipe.set_history_log(None);
    }

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
                if code == STATUS_BRAIN_CONNECTED {
                    // Brain rebooted: clear the ring and reset epoch so timestamps
                    // don't go backward (which breaks graph queries), then force
                    // the app back to disabled since the brain always starts there.
                    pipeline.lock().begin_live();
                    {
                        let mut ctrl = controls_fc.lock();
                        ctrl.mode = 0;
                        ctrl.ctrl_x = 0;
                        ctrl.ctrl_y = 0;
                        ctrl.ctrl_theta = 0;
                        ctrl.ctrl_throttle = 0;
                    }
                    let _ = app2.emit("force_disabled", ());
                }
                let _ = app2.emit("source_status", serde_json::json!({
                    "kind": "Live", "code": code, "detail": message
                }));
            }
            ReceiverFrame::RxRssi { rssi } => {
                let _ = app2.emit("rx_rssi", rssi);
            }
            ReceiverFrame::SerialLost => {
                let _ = app2.emit("source_status", serde_json::json!({
                    "kind": "Live", "detail": "Serial lost — reconnecting…"
                }));
            }
            ReceiverFrame::SerialReconnected => {
                let _ = app2.emit("source_status", serde_json::json!({
                    "kind": "Live", "detail": format!("Connected · {}", port_name)
                }));
            }
            _ => {}
        }
    })).map_err(|e| e)?;

    // Send the current (DISABLED) control state every 200 ms so the receiver's
    // host-watchdog (1.5 s) never fires while the app is open.
    let controls2 = state.controls.clone();
    conn.send_periodic(std::time::Duration::from_millis(200), move || {
        let ctrl = controls2.lock();
        encode_ctrl(ctrl.mode, ctrl.ctrl_x, ctrl.ctrl_y, ctrl.ctrl_theta, ctrl.ctrl_throttle)
    });

    *state.serial_conn.lock() = Some(conn);
    force_disabled(&state);

    // Reflect the connection immediately. The receiver only sends a brain
    // status frame on *change*, so if the brain is already up we'd otherwise
    // never get a source_status and the UI would stay on "Disconnected".
    let _ = app.emit("source_status", serde_json::json!({
        "kind": "Live", "detail": format!("Connected · {}", port)
    }));
    Ok(())
}

#[tauri::command]
pub fn disconnect_serial(state: State<'_, AppState>, app: AppHandle) {
    force_disabled(&state);
    *state.serial_conn.lock() = None;
    {
        let mut pipe = state.pipeline.lock();
        pipe.source = SourceKind::None;
    }
    let _ = app.emit("source_status", serde_json::json!({
        "kind": "Disconnected", "detail": ""
    }));
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

/// Load a replay file: pre-compute all DataPoints (so graph queries are O(log n))
/// and return the full time range. Emits `replay_progress` (0.0–1.0) events while
/// building the cache so the UI can show a progress bar.
#[tauri::command]
pub async fn load_replay(path: String, state: State<'_, AppState>, app: AppHandle) -> Result<serde_json::Value, String> {
    let path_buf = std::path::PathBuf::from(&path);
    let meta     = read_metadata(&path_buf).map_err(|e| e.to_string())?;
    let (start_us, end_us) = crate::replay::log_time_range(&meta).map_err(|e| e.to_string())?;

    // Stop any live/sim producer — both ingest frames into the ring on their own
    // thread/task independently of `source`, so leaving one running while replay
    // is active would let it keep mutating shared pipeline state in the background.
    *state.serial_conn.lock() = None;
    state.sim_stop.store(true, Ordering::Relaxed);

    {
        let mut pipe = state.pipeline.lock();
        pipe.source = SourceKind::Replay;
        pipe.set_history_log(Some(meta.clone()));
        pipe.set_replay_cache(vec![]); // clear stale cache immediately
    }

    let _ = app.emit("source_status", serde_json::json!({
        "kind": "Replay", "detail": meta.label.clone()
    }));
    let _ = app.emit("replay_progress", 0.0f32);

    // Build the cache off the pipeline mutex in a blocking thread.
    let meta_clone    = meta.clone();
    let app_clone     = app.clone();
    let pipeline_arc  = state.pipeline.clone();

    tokio::task::spawn_blocking(move || {
        let cache = crate::pipeline::build_replay_cache(&meta_clone, |frac| {
            let _ = app_clone.emit("replay_progress", frac);
        });
        pipeline_arc.lock().set_replay_cache(cache);
    }).await.map_err(|e| e.to_string())?;

    let _ = app.emit("replay_progress", -1.0f32); // sentinel: loading done

    Ok(serde_json::json!({
        "label":          meta.label,
        "frame_count":    meta.frame_count,
        "schema_version": meta.schema_version,
        "start_us":       start_us,
        "end_us":         end_us,
    }))
}

#[tauri::command]
pub async fn start_simulation(state: State<'_, AppState>, app: AppHandle) -> Result<(), ()> {
    let pipeline  = state.pipeline.clone();
    let controls  = state.controls.clone();
    let stop_flag = state.sim_stop.clone();

    // Stop any live producer first — the serial reader thread ingests frames
    // independently of `source`, so leaving it connected would race the sim
    // loop's writes into the same ring.
    *state.serial_conn.lock() = None;
    stop_flag.store(false, Ordering::Relaxed);

    let _ = app.emit("source_status", serde_json::json!({
        "kind": "Sim", "detail": "Running"
    }));

    {
        let mut pipe = state.pipeline.lock();
        pipe.begin_sim();
    }

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

            let frame_initial_state = replay_state; // snapshot before this batch

            let mut inputs = [SunshineInput::default(); INPUTS_PER_FRAME];
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

            // Use the pre-batch state so hw_state == replayed state when viewed
            let telem = TelemetryFrame {
                frame_id: 0,
                state:    frame_initial_state,
                inputs,
            };
            {
                let mut pipe = pipeline.lock();
                pipe.ingest_frame(&telem);
            }
            let update = build_live_update(&telem);
            let _ = app.emit("live_update", update);

            // 6 inputs × 1 ms each → sleep 6 ms to run at 1:1 real-time
            tokio::time::sleep(tokio::time::Duration::from_millis(6)).await;
        }

        let _ = app.emit("source_status", serde_json::json!({
            "kind": "Disconnected", "detail": ""
        }));
    });
    Ok(())
}


#[tauri::command]
pub fn stop_source(state: State<'_, AppState>, app: AppHandle) {
    {
        let mut pipe = state.pipeline.lock();
        pipe.source = SourceKind::None;
        pipe.set_history_log(None); // clears replay_cache too
    }
    state.sim_stop.store(true, Ordering::Relaxed);
    force_disabled(&state);
    let _ = app.emit("source_status", serde_json::json!({ "kind": "Disconnected", "detail": "" }));
}

#[tauri::command]
pub fn set_mode(mode: u8, state: State<'_, AppState>) {
    {
        let mut ctrl = state.controls.lock();
        ctrl.mode = mode;
        if mode == 0 {
            ctrl.ctrl_x = 0;
            ctrl.ctrl_y = 0;
            ctrl.ctrl_theta = 0;
            ctrl.ctrl_throttle = 0;
        }
    }
    send_current_controls(&state);
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
    drop(ctrl);
    send_current_controls(&state);
}

#[tauri::command]
pub fn enable_logging(label: String, state: State<'_, AppState>) -> Result<String, String> {
    let dir  = dirs::document_dir().unwrap_or_default().join("sunshine_logs");
    std::fs::create_dir_all(&dir).map_err(|e| e.to_string())?;
    let path = make_log_path(&dir, &label);
    let mut writer = LogWriter::new(path.clone(), &label, 0).map_err(|e| e.to_string())?;
    let mut pipe = state.pipeline.lock();
    pipe.set_history_log(None);
    // Backfill any data already in the ring so the log starts with full history.
    pipe.backfill_log_from_ring(&mut writer);
    pipe.logger = Some(writer);
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

#[tauri::command]
pub fn get_graph_data_multi(
    channels:   Vec<String>,
    start_us:   u64,
    end_us:     u64,
    max_points: u32,
    state:      State<'_, AppState>,
) -> Vec<Vec<(u64, f32)>> {
    state.pipeline.lock().get_graph_data_multi(&channels, start_us, end_us, max_points)
}

fn build_live_update(telem: &TelemetryFrame) -> serde_json::Value {
    let frame_id    = telem.frame_id;
    let kf_theta    = telem.state.kf_theta;
    let kf_omega    = telem.state.kf_omega;
    let last        = telem.inputs[INPUTS_PER_FRAME - 1];
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
