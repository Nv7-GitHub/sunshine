use crate::ffi::{SunshineInput, SunshineState, SunshineVars, brain_step, state_init};
use crate::logging::LogWriter;
use crate::protocol::TelemetryFrame;

#[derive(Clone, Default)]
pub struct DataPoint {
    pub time_us:         u64,
    pub input:           SunshineInput,
    pub hw_state:        SunshineState,  // received from hardware/sim/log at 50 Hz
    pub vars:            SunshineVars,   // always host-computed via brain_step
    pub rep_theta_offset: f32,           // theta_offset from replay_state after brain_step
}

const RING_CAP: usize = 120_000;

pub struct Pipeline {
    ring:          Vec<DataPoint>,
    ring_head:     usize,
    ring_len:      usize,
    replay_state:  SunshineState,
    pub logger:    Option<LogWriter>,
    pub source:    SourceKind,
    frame_count:   u32,
    hw_epoch_us:   u64,
    last_hw_us:    u32,
}

#[derive(Clone, Debug, PartialEq)]
pub enum SourceKind { None, Live, Replay, Simulation }

impl Pipeline {
    pub fn new() -> Self {
        let mut ring = Vec::with_capacity(RING_CAP);
        ring.resize(RING_CAP, DataPoint::default());
        let mut replay_state = SunshineState::default();
        state_init(&mut replay_state);
        Pipeline {
            ring, ring_head: 0, ring_len: 0,
            replay_state,
            logger: None,
            source: SourceKind::None,
            frame_count: 0,
            hw_epoch_us: 0,
            last_hw_us: 0,
        }
    }

    fn expand_hw_time(&mut self, hw: u32) -> u64 {
        if hw < self.last_hw_us && (self.last_hw_us - hw) > 0x8000_0000 {
            self.hw_epoch_us += 0x1_0000_0000;
        }
        self.last_hw_us = hw;
        self.hw_epoch_us + hw as u64
    }

    pub fn ingest_frame(&mut self, frame: &TelemetryFrame) {
        for input in frame.inputs.iter() {
            let time_us = self.expand_hw_time(input.time_us);
            let vars    = brain_step(input, &mut self.replay_state);

            let dp = DataPoint {
                time_us,
                input:    *input,
                hw_state: frame.state,
                vars,
                rep_theta_offset: self.replay_state.theta_offset,
            };

            let idx = self.ring_head;
            self.ring[idx] = dp;
            self.ring_head = (self.ring_head + 1) % RING_CAP;
            if self.ring_len < RING_CAP { self.ring_len += 1; }
        }

        if let Some(logger) = self.logger.as_mut() {
            let _ = logger.write_frame(
                self.frame_count,
                &frame.state,
                &frame.inputs,
                &SunshineVars::default(),
            );
        }
        self.frame_count += 1;
    }

    pub fn reset_replay_state(&mut self, state: &SunshineState) {
        self.replay_state = *state;
    }

    pub fn get_graph_data(
        &self,
        channel:    &str,
        start_us:   u64,
        end_us:     u64,
        max_points: u32,
    ) -> Vec<(u64, f32)> {
        if self.ring_len == 0 { return vec![]; }

        let accessor = channel_accessor(channel);
        let mut raw: Vec<(u64, f32)> = Vec::new();
        let start_idx = (self.ring_head + RING_CAP - self.ring_len) % RING_CAP;
        for i in 0..self.ring_len {
            let dp = &self.ring[(start_idx + i) % RING_CAP];
            if dp.time_us < start_us { continue; }
            if dp.time_us > end_us   { break; }
            raw.push((dp.time_us, accessor(dp)));
        }

        if raw.len() <= max_points as usize { return raw; }

        let bucket = raw.len() / max_points as usize;
        let mut out = Vec::with_capacity(max_points as usize * 2);
        for chunk in raw.chunks(bucket) {
            if chunk.is_empty() { continue; }
            let (t0, _) = chunk[0];
            let (t1, _) = *chunk.last().unwrap();
            let min = chunk.iter().map(|&(_, v)| v).fold(f32::INFINITY, f32::min);
            let max = chunk.iter().map(|&(_, v)| v).fold(f32::NEG_INFINITY, f32::max);
            let t_mid = (t0 + t1) / 2;
            out.push((t0, min));
            out.push((t_mid, max));
        }
        out
    }

    pub fn get_channel_snapshot(&self, channels: &[String], time_us: Option<u64>) -> Vec<Option<f32>> {
        if self.ring_len == 0 {
            return channels.iter().map(|_| None).collect();
        }

        let start_idx = (self.ring_head + RING_CAP - self.ring_len) % RING_CAP;

        let dp = match time_us {
            None => {
                let idx = (self.ring_head + RING_CAP - 1) % RING_CAP;
                &self.ring[idx]
            }
            Some(t) => {
                let mut best_idx = start_idx;
                let mut best_diff = u64::MAX;
                for i in 0..self.ring_len {
                    let idx = (start_idx + i) % RING_CAP;
                    let diff = self.ring[idx].time_us.abs_diff(t);
                    if diff < best_diff {
                        best_diff = diff;
                        best_idx = idx;
                    }
                }
                &self.ring[best_idx]
            }
        };

        channels.iter().map(|ch| {
            let v = channel_accessor(ch)(dp);
            if v.is_finite() { Some(v) } else { None }
        }).collect()
    }
}

const ADXL_SCALE_MS2: f32 = 49e-3 * 9.81;
const MAG_SCALE_UT:   f32 = 0.058;

fn channel_accessor(channel: &str) -> fn(&DataPoint) -> f32 {
    match channel {
        /* Hardware state — received from brain at 50 Hz */
        "hw.kf_theta"           => |dp: &DataPoint| dp.hw_state.kf_theta,
        "hw.kf_omega"           => |dp| dp.hw_state.kf_omega,
        "hw.theta_offset"       => |dp| dp.hw_state.theta_offset,
        "hw.dshot_left"         => |dp| dp.input.dshot_left_q as f32,
        "hw.dshot_right"        => |dp| dp.input.dshot_right_q as f32,
        /* Variables — always host-computed via brain_step */
        "rep.est_theta"         => |dp| dp.vars.est_theta,
        "rep.est_omega"         => |dp| dp.vars.est_omega,
        "rep.theta_offset"      => |dp| dp.rep_theta_offset,
        "rep.heading_deg"       => |dp| dp.vars.heading_deg,
        "rep.dshot_left"        => |dp| dp.vars.dshot_cmd_left,
        "rep.dshot_right"       => |dp| dp.vars.dshot_cmd_right,
        "rep.batt_voltage"      => |dp| dp.vars.batt_voltage,
        "rep.erpm_left"         => |dp| dp.vars.erpm_left,
        "rep.erpm_right"        => |dp| dp.vars.erpm_right,
        "rep.mag_angle"         => |dp| dp.vars.mag_angle,
        "rep.derot_i"           => |dp| dp.vars.derot_i,
        "rep.derot_q"           => |dp| dp.vars.derot_q,
        "rep.omega_from_accel"  => |dp| dp.vars.omega_from_accel,
        "rep.centripetal_ms2"   => |dp| dp.vars.centripetal_ms2,
        /* Raw sensor inputs */
        "input.accel_x"         => |dp| dp.input.accel_x as f32,
        "input.accel_y"         => |dp| dp.input.accel_y as f32,
        "input.accel_z"         => |dp| dp.input.accel_z as f32,
        "input.accel_x_ms2"     => |dp| dp.input.accel_x as f32 * ADXL_SCALE_MS2,
        "input.accel_y_ms2"     => |dp| dp.input.accel_y as f32 * ADXL_SCALE_MS2,
        "input.accel_z_ms2"     => |dp| dp.input.accel_z as f32 * ADXL_SCALE_MS2,
        "input.mag_x"           => |dp| dp.input.mag_x as f32,
        "input.mag_y"           => |dp| dp.input.mag_y as f32,
        "input.mag_magnitude"   => |dp| {
            let x = dp.input.mag_x as f32;
            let y = dp.input.mag_y as f32;
            let z = dp.input.mag_z as f32;
            (x*x + y*y + z*z).sqrt() * MAG_SCALE_UT
        },
        "input.erpm_left"       => |dp| dp.input.erpm_left as f32,
        "input.erpm_right"      => |dp| dp.input.erpm_right as f32,
        "input.ctrl_x"          => |dp| dp.input.ctrl_x as f32,
        "input.ctrl_y"          => |dp| dp.input.ctrl_y as f32,
        "input.ctrl_theta"      => |dp| dp.input.ctrl_theta as f32,
        "input.ctrl_throttle"   => |dp| dp.input.ctrl_throttle as f32,
        "input.rssi"            => |dp| dp.input.rssi as f32,
        "input.batt_offset"     => |dp| dp.input.batt_offset as f32,
        _                       => |_| 0.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::{TelemetryFrame, INPUTS_PER_FRAME};
    use crate::ffi::{SunshineInput, SunshineState};

    fn make_frame(inputs: [SunshineInput; INPUTS_PER_FRAME]) -> TelemetryFrame {
        TelemetryFrame { frame_id: 0, state: SunshineState::default(), inputs }
    }

    #[test]
    fn ingest_uses_hardware_timestamp() {
        let mut p = Pipeline::new();
        let hw_us: u32 = 5_000_000;
        let input = SunshineInput { time_us: hw_us, ctrl_x: 7, ..SunshineInput::default() };
        p.ingest_frame(&make_frame([input; INPUTS_PER_FRAME]));
        let data = p.get_graph_data("input.ctrl_x", hw_us as u64 - 1, hw_us as u64 + 1, 100);
        assert!(!data.is_empty(), "stored points must use hardware timestamp");
    }

    #[test]
    fn snapshot_empty_ring_returns_none() {
        let p = Pipeline::new();
        let vals = p.get_channel_snapshot(&["rep.est_theta".to_string()], None);
        assert_eq!(vals.len(), 1);
        assert!(vals[0].is_none(), "empty ring must return None");
    }

    #[test]
    fn snapshot_latest() {
        let mut p = Pipeline::new();
        let input = SunshineInput { ctrl_x: 42, ..SunshineInput::default() };
        p.ingest_frame(&make_frame([input; INPUTS_PER_FRAME]));
        let vals = p.get_channel_snapshot(&["input.ctrl_x".to_string()], None);
        assert_eq!(vals.len(), 1);
        assert_eq!(vals[0], Some(42.0));
    }

    #[test]
    fn snapshot_by_time_returns_closest() {
        let mut p = Pipeline::new();
        let hw_us: u32 = 10_000_000;
        let input = SunshineInput { time_us: hw_us, ctrl_x: 7, ..SunshineInput::default() };
        p.ingest_frame(&make_frame([input; INPUTS_PER_FRAME]));
        let later = hw_us as u64 + 1_000_000;
        let vals = p.get_channel_snapshot(&["input.ctrl_x".to_string()], Some(later));
        assert_eq!(vals[0], Some(7.0));
    }

    #[test]
    fn snapshot_unknown_channel_returns_zero() {
        let mut p = Pipeline::new();
        let input = SunshineInput::default();
        p.ingest_frame(&make_frame([input; INPUTS_PER_FRAME]));
        let vals = p.get_channel_snapshot(&["not.a.channel".to_string()], None);
        assert_eq!(vals[0], Some(0.0));
    }
}
