use crate::ffi::{SunshineInput, SunshineState, SunshineVars, brain_step, state_init};
use crate::logging::LogWriter;
use crate::protocol::TelemetryFrame;

#[derive(Clone, Default)]
pub struct DataPoint {
    pub time_us:   u64,
    pub input:     SunshineInput,
    pub real_vars: SunshineVars,
    pub rep_vars:  SunshineVars,
}

const RING_CAP: usize = 120_000;

pub struct Pipeline {
    ring:         Vec<DataPoint>,
    ring_head:    usize,
    ring_len:     usize,
    replay_state: SunshineState,
    pub logger:   Option<LogWriter>,
    pub source:   SourceKind,
    frame_count:  u32,
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
        }
    }

    pub fn ingest_frame(&mut self, frame: &TelemetryFrame, real_vars_snap: Option<&SunshineVars>) {
        use std::time::{SystemTime, UNIX_EPOCH};
        let now_us = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_micros() as u64;

        for input in frame.inputs.iter() {
            let rep_vars = brain_step(input, &mut self.replay_state);

            let dp = DataPoint {
                time_us: now_us,
                input:     *input,
                real_vars: real_vars_snap.copied().unwrap_or_default(),
                rep_vars,
            };

            let idx = self.ring_head;
            self.ring[idx] = dp;
            self.ring_head = (self.ring_head + 1) % RING_CAP;
            if self.ring_len < RING_CAP { self.ring_len += 1; }
        }

        if let Some(logger) = self.logger.as_mut() {
            let vars_snap = real_vars_snap.copied().unwrap_or_default();
            let _ = logger.write_frame(
                self.frame_count,
                &frame.state,
                &frame.inputs,
                &vars_snap,
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

    pub fn get_channel_snapshot(&self, channels: &[String], time_us: Option<u64>) -> Vec<f32> {
        if self.ring_len == 0 {
            return channels.iter().map(|_| f32::NAN).collect();
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

        channels.iter().map(|ch| channel_accessor(ch)(dp)).collect()
    }
}

fn channel_accessor(channel: &str) -> fn(&DataPoint) -> f32 {
    match channel {
        "rep.est_theta"         => |dp: &DataPoint| dp.rep_vars.est_theta,
        "rep.est_omega"         => |dp| dp.rep_vars.est_omega,
        "rep.dshot_left"        => |dp| dp.rep_vars.dshot_cmd_left,
        "rep.dshot_right"       => |dp| dp.rep_vars.dshot_cmd_right,
        "rep.batt_voltage"      => |dp| dp.rep_vars.batt_voltage,
        "rep.erpm_left"         => |dp| dp.rep_vars.erpm_left,
        "rep.erpm_right"        => |dp| dp.rep_vars.erpm_right,
        "rep.mag_angle"         => |dp| dp.rep_vars.mag_angle,
        "rep.derot_i"           => |dp| dp.rep_vars.derot_i,
        "rep.derot_q"           => |dp| dp.rep_vars.derot_q,
        "rep.omega_from_accel"  => |dp| dp.rep_vars.omega_from_accel,
        "rep.centripetal_ms2"   => |dp| dp.rep_vars.centripetal_ms2,
        "real.est_theta"        => |dp| dp.real_vars.est_theta,
        "real.est_omega"        => |dp| dp.real_vars.est_omega,
        "real.dshot_left"       => |dp| dp.real_vars.dshot_cmd_left,
        "real.dshot_right"      => |dp| dp.real_vars.dshot_cmd_right,
        "input.accel_x"         => |dp| dp.input.accel_x as f32,
        "input.accel_y"         => |dp| dp.input.accel_y as f32,
        "input.accel_z"         => |dp| dp.input.accel_z as f32,
        "input.mag_x"           => |dp| dp.input.mag_x as f32,
        "input.mag_y"           => |dp| dp.input.mag_y as f32,
        "input.ctrl_x"          => |dp| dp.input.ctrl_x as f32,
        "input.ctrl_y"          => |dp| dp.input.ctrl_y as f32,
        "input.ctrl_throttle"   => |dp| dp.input.ctrl_throttle as f32,
        "input.rssi"            => |dp| dp.input.rssi as f32,
        "input.batt_offset"     => |dp| dp.input.batt_offset as f32,
        _                       => |_| 0.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::TelemetryFrame;
    use crate::ffi::{SunshineInput, SunshineState};
    use std::time::{SystemTime, UNIX_EPOCH};

    fn make_frame(inputs: [SunshineInput; 20]) -> TelemetryFrame {
        TelemetryFrame { frame_id: 0, state: SunshineState::default(), inputs }
    }

    fn now_us() -> u64 {
        SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_micros() as u64
    }

    #[test]
    fn ingest_uses_wall_clock_timestamp() {
        let mut p = Pipeline::new();
        let before = now_us();
        let input = SunshineInput { time_us: 999, ..SunshineInput::default() };
        p.ingest_frame(&make_frame([input; 20]), None);
        let after = now_us();
        let data = p.get_graph_data("input.ctrl_x", before - 1, after + 1, 100);
        assert!(!data.is_empty(), "stored points must use wall-clock timestamp");
    }

    #[test]
    fn snapshot_empty_ring_returns_nan() {
        let p = Pipeline::new();
        let vals = p.get_channel_snapshot(&["rep.est_theta".to_string()], None);
        assert_eq!(vals.len(), 1);
        assert!(vals[0].is_nan(), "empty ring must return NaN");
    }

    #[test]
    fn snapshot_latest() {
        let mut p = Pipeline::new();
        let input = SunshineInput { ctrl_x: 42, ..SunshineInput::default() };
        p.ingest_frame(&make_frame([input; 20]), None);
        let vals = p.get_channel_snapshot(&["input.ctrl_x".to_string()], None);
        assert_eq!(vals.len(), 1);
        assert_eq!(vals[0], 42.0);
    }

    #[test]
    fn snapshot_by_time_returns_closest() {
        let mut p = Pipeline::new();
        let input = SunshineInput { ctrl_x: 7, ..SunshineInput::default() };
        p.ingest_frame(&make_frame([input; 20]), None);
        let after = now_us() + 1_000_000; // 1 second in the future
        // Only batch is in the past — it's still the closest point
        let vals = p.get_channel_snapshot(&["input.ctrl_x".to_string()], Some(after));
        assert_eq!(vals[0], 7.0);
    }

    #[test]
    fn snapshot_unknown_channel_returns_zero() {
        let mut p = Pipeline::new();
        let input = SunshineInput::default();
        p.ingest_frame(&make_frame([input; 20]), None);
        let vals = p.get_channel_snapshot(&["not.a.channel".to_string()], None);
        assert_eq!(vals[0], 0.0);
    }
}
