use serde::{Deserialize, Serialize};

#[derive(Clone, Default, Debug, Serialize, Deserialize)]
pub struct ControlState {
    pub mode:          u8,
    pub ctrl_x:        i8,
    pub ctrl_y:        i8,
    pub ctrl_theta:    i8,
    pub ctrl_throttle: u8,
}
