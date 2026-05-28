#![allow(dead_code, unused_variables, unused_imports)]

use tauri::State;
use crate::AppState;
use serde_json::Value;

#[tauri::command]
pub fn list_serial_ports() -> Vec<String> {
    vec![]
}

#[tauri::command]
pub fn connect_serial(_state: State<AppState>, _port: String) -> Result<(), String> {
    Ok(())
}

#[tauri::command]
pub fn disconnect_serial(_state: State<AppState>) -> Result<(), String> {
    Ok(())
}

#[tauri::command]
pub fn open_replay(_state: State<AppState>, _path: String) -> Result<(), String> {
    Ok(())
}

#[tauri::command]
pub fn start_simulation(_state: State<AppState>) -> Result<(), String> {
    Ok(())
}

#[tauri::command]
pub fn stop_source(_state: State<AppState>) -> Result<(), String> {
    Ok(())
}

#[tauri::command]
pub fn set_mode(_state: State<AppState>, _mode: u8) -> Result<(), String> {
    Ok(())
}

#[tauri::command]
pub fn set_controls(_state: State<AppState>, _x: f32, _y: f32, _theta: f32, _throttle: f32) -> Result<(), String> {
    Ok(())
}

#[tauri::command]
pub fn enable_logging(_state: State<AppState>, _path: String) -> Result<(), String> {
    Ok(())
}

#[tauri::command]
pub fn disable_logging(_state: State<AppState>) -> Result<(), String> {
    Ok(())
}

#[tauri::command]
pub fn get_graph_data(_state: State<AppState>) -> Value {
    serde_json::json!({})
}
