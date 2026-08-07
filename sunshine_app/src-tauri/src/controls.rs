use serde::{Deserialize, Serialize};

#[derive(Clone, Default, Debug, Serialize, Deserialize)]
pub struct ControlState {
    pub mode:          u8,
    pub ctrl_x:        i8,
    pub ctrl_y:        i8,
    pub ctrl_theta:    i8,
    pub ctrl_throttle: u8,
}

/// Raw key directions (-1/0/+1 per axis) reported by the frontend on key
/// events. The RAMPS live in a native backend thread (lib.rs), NOT in the
/// webview: both requestAnimationFrame and setInterval get throttled by the
/// webview scheduler (measured: control updates at median 60 ms with stalls up
/// to 8.6 s — the robot spent a whole session pinned at partial deflection).
/// A std::thread is immune to that.
#[derive(Clone, Copy, Default, Debug)]
pub struct KeyTargets {
    pub x:     i8,
    pub y:     i8,
    pub theta: i8,
    pub thr:   i8,
}
