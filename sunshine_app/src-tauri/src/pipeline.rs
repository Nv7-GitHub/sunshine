use crate::ffi::{SunshineInput, SunshineState, SunshineVars, brain_step, f16_to_f32, state_init};
use crate::logging::LogWriter;
use crate::protocol::{TelemetryFrame, INPUTS_PER_FRAME};
use crate::replay::{LogMetadata, read_frame, read_metadata};
use std::fs::File;
use std::io::{Seek, SeekFrom};

#[derive(Clone, Default)]
pub struct DataPoint {
    pub time_us:    u64,
    pub input:      SunshineInput,
    // REAL series: the logged real state at the two per-frame snapshots (frame start
    // + midpoint → 100 Hz). `real_valid` is true ONLY at those two ticks; elsewhere
    // the real.* channels are omitted so the host draws straight lines between the
    // genuine 100 Hz samples (a coarse version of the replayed curve) instead of a
    // 1 kHz staircase-with-noise (which is what holding the state and re-stepping a
    // throwaway copy every tick produced). `real_vars` at a valid tick = one
    // brain_step from that snapshot's real state, so it's the true recorded vars.
    pub real_valid: bool,
    pub real_state: SunshineState,
    pub real_vars:  SunshineVars,
    // REPLAYED series: a filter free-running continuously from the first frame
    // (never re-anchored). Shows what the (possibly modified) host code produces
    // across the whole record — this is what you compare against the real series.
    pub rep_state:  SunshineState,
    pub rep_vars:   SunshineVars,
}

const RING_CAP: usize = 300_000;

pub struct Pipeline {
    ring:          Vec<DataPoint>,
    ring_head:     usize,
    ring_len:      usize,
    real_state:    SunshineState,   // re-anchored each frame (REAL)
    rep_state:     SunshineState,   // continuous free-run (REPLAYED)
    rep_seeded:    bool,            // rep_state seeded from the first frame yet?
    pub logger:    Option<LogWriter>,
    pub source:    SourceKind,
    history_log:   Option<LogMetadata>,
    /// Pre-computed DataPoints for Replay mode; populated by load_replay_cache.
    replay_cache:  Vec<DataPoint>,
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
        let mut real_state = SunshineState::default();
        state_init(&mut real_state);
        let mut rep_state = SunshineState::default();
        state_init(&mut rep_state);
        Pipeline {
            ring, ring_head: 0, ring_len: 0,
            real_state, rep_state, rep_seeded: false,
            logger: None,
            source: SourceKind::None,
            history_log: None,
            replay_cache: Vec::new(),
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
        // Per frame (see DataPoint):
        //  REAL     — the logged real state, HELD at each snapshot (frame start, then
        //             midpoint → 100 Hz). Shows the actual recorded state as-is (a
        //             staircase), never stepped/fabricated — so it reads distinctly
        //             from the replayed curve and stays honest when code changes.
        //  REPLAYED — seeded once from the first frame, then free-runs at 1 kHz. Lets
        //             a changed sunshine_step be validated across the whole record.
        self.real_state = frame.state;
        if !self.rep_seeded { self.rep_state = frame.state; self.rep_seeded = true; }

        let mid = INPUTS_PER_FRAME / 2;
        for (i, input) in frame.inputs.iter().enumerate() {
            if i == mid { self.real_state = frame.state_mid; }  // 100 Hz snapshot
            let time_us   = self.expand_hw_time(input.time_us);
            // REAL is a genuine sample only at the two snapshot ticks (frame start,
            // midpoint). There `self.real_state` is the logged snapshot state, so one
            // brain_step yields the true recorded vars. Elsewhere real_valid=false and
            // the real.* channels are skipped, so the plot connects the real samples
            // with straight lines (coarse) instead of a fabricated 1 kHz staircase.
            let real_valid = i == 0 || i == mid;
            let mut real_tmp = self.real_state;
            let real_vars = brain_step(input, &mut real_tmp);
            let rep_vars  = brain_step(input, &mut self.rep_state);

            let dp = DataPoint {
                time_us,
                input:      *input,
                real_valid,
                real_state: self.real_state,
                real_vars,
                rep_state:  self.rep_state,
                rep_vars,
            };

            let idx = self.ring_head;
            self.ring[idx] = dp;
            self.ring_head = (self.ring_head + 1) % RING_CAP;
            if self.ring_len < RING_CAP { self.ring_len += 1; }
        }

        // Log only the real state (start + midpoint) and the inputs — no vars.
        if let Some(logger) = self.logger.as_mut() {
            let _ = logger.write_frame(self.frame_count, &frame.state, &frame.state_mid, &frame.inputs);
        }
        self.frame_count += 1;
    }

    /// Begin a live session. Clears stale ring data and resets timestamp state
    /// so old frames from a previous session don't bleed into the new one.
    /// `ingest_frame` re-anchors `replay_state` from each packet's transmitted
    /// state while the source is Live (see `ingest_frame`).
    pub fn begin_live(&mut self) {
        self.source      = SourceKind::Live;
        self.ring_len    = 0;
        self.ring_head   = 0;
        self.hw_epoch_us = 0;
        self.last_hw_us  = 0;
        self.frame_count = 0;
        self.rep_seeded  = false;
        state_init(&mut self.real_state);
        state_init(&mut self.rep_state);
    }

    /// Begin a simulation session. Identical ring/timestamp reset as begin_live
    /// so stale ring entries from a previous sim/live session don't produce
    /// non-monotonic timestamps when the new sim restarts time_us from 0.
    pub fn begin_sim(&mut self) {
        self.source      = SourceKind::Simulation;
        self.ring_len    = 0;
        self.ring_head   = 0;
        self.hw_epoch_us = 0;
        self.last_hw_us  = 0;
        self.frame_count = 0;
        self.rep_seeded  = false;
        state_init(&mut self.real_state);
        state_init(&mut self.rep_state);
        self.replay_cache.clear();
        self.history_log = None;
    }

    pub fn set_history_log(&mut self, meta: Option<LogMetadata>) {
        if meta.is_none() { self.replay_cache.clear(); }
        self.history_log = meta;
    }

    pub fn set_replay_cache(&mut self, cache: Vec<DataPoint>) {
        self.replay_cache = cache;
    }

    fn ring_oldest_us(&self) -> Option<u64> {
        if self.ring_len == 0 { return None; }
        let idx = (self.ring_head + RING_CAP - self.ring_len) % RING_CAP;
        Some(self.ring[idx].time_us)
    }

    /// Write all ring-buffered data to the given LogWriter, grouped into
    /// frames of INPUTS_PER_FRAME.  Called when logging is enabled mid-session
    /// so the log file starts with a complete history.
    pub fn backfill_log_from_ring(&self, logger: &mut LogWriter) {
        if self.ring_len < INPUTS_PER_FRAME { return; }

        let start_idx = (self.ring_head + RING_CAP - self.ring_len) % RING_CAP;
        let mid = INPUTS_PER_FRAME / 2;
        // Only write complete frames; the last partial group will be captured by the next
        // ingest_frame call so we don't write zero-padded inputs with broken time_us=0.
        let complete_frames = self.ring_len / INPUTS_PER_FRAME;
        let mut frame_id: u32 = 0;

        for f in 0..complete_frames {
            let chunk_start = f * INPUTS_PER_FRAME;
            let mut inputs = [SunshineInput::default(); INPUTS_PER_FRAME];
            for i in 0..INPUTS_PER_FRAME {
                inputs[i] = self.ring[(start_idx + chunk_start + i) % RING_CAP].input;
            }
            // Real state at this frame's start and midpoint (reconstructed in the ring).
            let state_start = self.ring[(start_idx + chunk_start) % RING_CAP].real_state;
            let state_mid   = self.ring[(start_idx + chunk_start + mid) % RING_CAP].real_state;
            let _ = logger.write_frame(frame_id, &state_start, &state_mid, &inputs);
            frame_id += 1;
        }
    }

    pub fn get_graph_data(
        &mut self,
        channel:    &str,
        start_us:   u64,
        end_us:     u64,
        max_points: u32,
    ) -> Vec<(u64, f32)> {
        // Replay mode: always use the pre-computed cache. Return [] while the
        // cache is still being built (spawn_blocking hasn't stored it yet).
        // NEVER fall back to a file scan here — that would hold the pipeline
        // mutex for seconds and starve the cache-building thread trying to store
        // the finished cache (→ freeze).
        if self.source == SourceKind::Replay {
            return get_graph_data_from_slice(&self.replay_cache, channel, start_us, end_us, max_points);
        }

        // Live / sim: use in-memory ring for recent data.
        // Only fall back to the active log file when the ring has actually
        // WRAPPED (ring_len == RING_CAP) and the requested window starts before
        // the oldest retained sample — i.e. the data was genuinely evicted.
        // Without the wrap check this fires during the first ~5 min, and since
        // a logger is present it re-reads + re-replays the entire growing log on
        // every 10 Hz fetch → multi-second freeze.
        let oldest = self.ring_oldest_us();
        if let Some(oldest_t) = oldest {
            if start_us < oldest_t && self.ring_len == RING_CAP {
                let log_path = self.logger.as_ref().map(|l| l.path().clone());
                if let Some(path) = log_path {
                    if let Some(logger) = self.logger.as_mut() { let _ = logger.flush(); }
                    if let Ok(meta) = read_metadata(&path) {
                        if let Some(data) = self.get_graph_data_from_log_meta(&meta, channel, start_us, end_us, max_points) {
                            return data;
                        }
                    }
                }
            }
        }

        self.get_graph_data_from_ring(channel, start_us, end_us, max_points)
    }

    fn get_graph_data_from_ring(
        &self,
        channel:    &str,
        start_us:   u64,
        end_us:     u64,
        max_points: u32,
    ) -> Vec<(u64, f32)> {
        if self.ring_len == 0 { return vec![]; }
        let accessor = channel_accessor(channel);
        let is_real  = channel.starts_with("real.");
        let mut raw: Vec<(u64, f32)> = Vec::new();
        let start_idx = (self.ring_head + RING_CAP - self.ring_len) % RING_CAP;
        for i in 0..self.ring_len {
            let dp = &self.ring[(start_idx + i) % RING_CAP];
            if dp.time_us < start_us { continue; }
            if dp.time_us > end_us   { break; }
            if is_real && !dp.real_valid { continue; }   // real.* only at 100 Hz snapshots
            raw.push((dp.time_us, accessor(dp)));
        }

        downsample_min_max(raw, max_points)
    }

    fn get_graph_data_from_log_meta(
        &self,
        meta:       &LogMetadata,
        channel:    &str,
        start_us:   u64,
        end_us:     u64,
        max_points: u32,
    ) -> Option<Vec<(u64, f32)>> {
        let accessor = channel_accessor(channel);
        let is_real  = channel.starts_with("real.");
        let mut file = File::open(&meta.path).ok()?;
        file.seek(SeekFrom::Start(meta.header_size)).ok()?;

        let mut real_state = SunshineState::default(); state_init(&mut real_state);
        let mut rep_state  = SunshineState::default(); state_init(&mut rep_state);
        let mut rep_seeded = false;
        let mid = INPUTS_PER_FRAME / 2;
        let mut raw: Vec<(u64, f32)> = Vec::new();
        let mut epoch_us = 0u64;
        let mut last_hw_us = 0u32;

        for _ in 0..meta.frame_count {
            let (frame, _) = read_frame(&mut file, meta).ok()?;
            real_state = frame.state;                       // REAL: re-anchor each frame
            if !rep_seeded { rep_state = frame.state; rep_seeded = true; }  // REPLAYED: seed once
            for (i, input) in frame.inputs.iter().enumerate() {
                if i == mid { real_state = frame.state_mid; }   // 100 Hz re-anchor
                let hw = input.time_us;
                if hw < last_hw_us && (last_hw_us - hw) > 0x8000_0000 {
                    epoch_us += 0x1_0000_0000;
                }
                last_hw_us = hw;
                let time_us = epoch_us + hw as u64;
                let dp = step_point(time_us, input, i == 0 || i == mid, &real_state, &mut rep_state);

                if time_us < start_us { continue; }
                if time_us > end_us { return Some(downsample_min_max(raw, max_points)); }
                if is_real && !dp.real_valid { continue; }   // real.* only at 100 Hz snapshots
                raw.push((time_us, accessor(&dp)));
            }
        }

        Some(downsample_min_max(raw, max_points))
    }

    /// Fetch multiple channels for the same window under a single pipeline lock,
    /// so all returned series come from one consistent snapshot of the source
    /// (replay cache or ring). Querying each channel via a separate lock
    /// acquisition (one round-trip per channel) lets the source mutate between
    /// calls — e.g. a file swap or new live frames arriving — producing series
    /// of mismatched length/range that corrupt the chart when painted together.
    pub fn get_graph_data_multi(
        &mut self,
        channels:   &[String],
        start_us:   u64,
        end_us:     u64,
        max_points: u32,
    ) -> Vec<Vec<(u64, f32)>> {
        channels.iter()
            .map(|ch| self.get_graph_data(ch, start_us, end_us, max_points))
            .collect()
    }

    pub fn get_channel_snapshot(&self, channels: &[String], time_us: Option<u64>) -> Vec<Option<f32>> {
        // Replay mode: data lives in replay_cache, not the ring.
        if self.source == SourceKind::Replay {
            if self.replay_cache.is_empty() {
                return channels.iter().map(|_| None).collect();
            }
            let dp = match time_us {
                None => self.replay_cache.last().unwrap(),
                Some(t) => {
                    // replay_cache is sorted by time_us — use binary search.
                    let lo = self.replay_cache.partition_point(|dp| dp.time_us < t);
                    if lo == 0 {
                        &self.replay_cache[0]
                    } else if lo == self.replay_cache.len() {
                        self.replay_cache.last().unwrap()
                    } else {
                        let before = &self.replay_cache[lo - 1];
                        let after  = &self.replay_cache[lo];
                        if t - before.time_us <= after.time_us - t { before } else { after }
                    }
                }
            };
            return channels.iter().map(|ch| {
                let v = channel_accessor(ch)(dp);
                if v.is_finite() { Some(v) } else { None }
            }).collect();
        }

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

/// Build the DataPoint for one input. `real_valid` is true only at the two per-frame
/// snapshot ticks (frame start / midpoint); there `real_state` is the logged snapshot
/// state, so one throwaway brain_step gives the true recorded 100 Hz vars. At other
/// ticks real_valid=false and the real.* channels are skipped, so the plot draws
/// straight lines between the genuine 100 Hz samples (coarse) rather than a 1 kHz
/// staircase. `rep_state` free-runs (the 1 kHz recomputation) at every tick.
fn step_point(time_us: u64, input: &SunshineInput, real_valid: bool,
              real_state: &SunshineState, rep_state: &mut SunshineState) -> DataPoint {
    let mut real_tmp = *real_state;                 // derive vars without evolving the held state
    let real_vars = brain_step(input, &mut real_tmp);
    let rep_vars  = brain_step(input, rep_state);
    DataPoint {
        time_us,
        input:      *input,
        real_valid,
        real_state: *real_state,
        real_vars,
        rep_state:  *rep_state,
        rep_vars,
    }
}

/// Build the replay cache off the pipeline mutex. Calls `on_progress(0..=1)`
/// periodically so the caller can forward progress to the UI.
pub fn build_replay_cache(meta: &LogMetadata, mut on_progress: impl FnMut(f32)) -> Vec<DataPoint> {
    let mut file = match File::open(&meta.path) {
        Ok(f) => f,
        Err(_) => return vec![],
    };
    let _ = file.seek(SeekFrom::Start(meta.header_size));

    let mut real_state = SunshineState::default(); state_init(&mut real_state);
    let mut rep_state  = SunshineState::default(); state_init(&mut rep_state);
    let mut rep_seeded = false;
    let mid = INPUTS_PER_FRAME / 2;
    let mut epoch_us:   u64 = 0;
    let mut last_hw_us: u32 = 0;

    let total = meta.frame_count as usize;
    let cap   = total * meta.num_inputs as usize;
    let mut cache: Vec<DataPoint> = Vec::with_capacity(cap);

    for i in 0..total {
        let (frame, _) = match read_frame(&mut file, meta) {
            Ok(f) => f,
            Err(_) => break,
        };
        real_state = frame.state;                       // REAL: re-anchor each frame
        if !rep_seeded { rep_state = frame.state; rep_seeded = true; }  // REPLAYED: seed once
        for (j, input) in frame.inputs.iter().enumerate() {
            if j == mid { real_state = frame.state_mid; }   // 100 Hz re-anchor
            let hw = input.time_us;
            if hw < last_hw_us && (last_hw_us - hw) > 0x8000_0000 {
                epoch_us += 0x1_0000_0000;
            }
            last_hw_us = hw;
            let time_us = epoch_us + hw as u64;
            cache.push(step_point(time_us, input, j == 0 || j == mid, &real_state, &mut rep_state));
        }
        if i % 200 == 0 || i == total - 1 {
            on_progress((i + 1) as f32 / total as f32);
        }
    }
    cache
}

fn get_graph_data_from_slice(
    data:       &[DataPoint],
    channel:    &str,
    start_us:   u64,
    end_us:     u64,
    max_points: u32,
) -> Vec<(u64, f32)> {
    if data.is_empty() { return vec![]; }
    let accessor = channel_accessor(channel);
    let is_real  = channel.starts_with("real.");
    // Binary search to the first point >= start_us
    let lo = data.partition_point(|dp| dp.time_us < start_us);
    let raw: Vec<(u64, f32)> = data[lo..]
        .iter()
        .take_while(|dp| dp.time_us <= end_us)
        .filter(|dp| !is_real || dp.real_valid)      // real.* only at 100 Hz snapshots
        .map(|dp| (dp.time_us, accessor(dp)))
        .collect();
    downsample_min_max(raw, max_points)
}

fn downsample_min_max(raw: Vec<(u64, f32)>, max_points: u32) -> Vec<(u64, f32)> {
    if raw.len() <= max_points as usize { return raw; }
    if max_points == 0 { return vec![]; }

    let bucket = (raw.len() / max_points as usize).max(1);
    let mut out = Vec::with_capacity(max_points as usize * 2);
    for chunk in raw.chunks(bucket) {
        if chunk.is_empty() { continue; }
        let (t0, _) = chunk[0];
        let (t1, _) = *chunk.last().unwrap();
        // NaN-aware min/max. The obvious `fold(f32::INFINITY, f32::min)` is WRONG
        // here on both counts, because Rust's f32::min/max *ignore* NaN:
        //   - a MIXED bucket (some finite, some NaN) silently dropped the NaN, so a
        //     one-sample gap inside a bucket disappeared entirely;
        //   - an ALL-NaN bucket returned (+inf, -inf) rather than NaN.
        // The second one matters because serde_json maps NaN *and* ±inf to the SAME
        // JSON null (ser.rs serialize_f32), so an escaping +inf is indistinguishable
        // at the UI from a genuine gap — it would masquerade as "no measurement"
        // while actually meaning "the encoder overflowed". Non-finite values are
        // therefore never measurements and never emitted: we skip them on the way in
        // and emit an explicit NaN when the whole bucket was unmeasurable.
        // ±inf also arrives from pre-§4.1 logs, where sunshine_f32_to_f16 collapsed
        // NaN to 0x7C00; those become gaps too, which is the honest reading.
        let mut min = f32::INFINITY;
        let mut max = f32::NEG_INFINITY;
        let mut any_finite = false;
        for &(_, v) in chunk {
            if !v.is_finite() { continue; }
            any_finite = true;
            if v < min { min = v; }
            if v > max { max = v; }
        }
        let (min, max) = if any_finite { (min, max) } else { (f32::NAN, f32::NAN) };
        // An all-NaN bucket STILL pushes its two points. Every non-`real.*` channel
        // shares one downsampled time grid, and UPlotCanvas builds a UNION x-axis
        // from all selected channels' timestamps. Dropping the points here would
        // shorten this channel's grid, so the other channels' timestamps would enter
        // the union as extra columns where this channel is null — making a dense
        // channel read as sparse and, now that spanGaps is false for dense series,
        // punching spurious gaps into every OTHER trace. Point count per bucket must
        // stay at exactly 2.
        let t_mid = (t0 + t1) / 2;
        out.push((t0, min));
        out.push((t_mid, max));
    }
    out
}

const ADXL_SCALE_MS2: f32 = 49e-3 * 9.81;
const MAG_SCALE_UT:   f32 = 0.058;

fn channel_accessor(channel: &str) -> fn(&DataPoint) -> f32 {
    match channel {
        /* ── REAL: filter re-anchored to the logged real state (≈ the robot) ── */
        "real.kf_theta"          => |dp: &DataPoint| dp.real_state.kf_theta,
        "real.kf_omega"          => |dp| dp.real_state.kf_omega,
        "real.theta_offset"      => |dp| dp.real_state.theta_offset,
        "real.est_theta"         => |dp| dp.real_vars.est_theta,
        "real.est_omega"         => |dp| dp.real_vars.est_omega,
        "real.heading_deg"       => |dp| dp.real_vars.heading_deg,
        "real.mag_angle"         => |dp| dp.real_vars.mag_angle,
        "real.mag_x_filt"        => |dp| dp.real_vars.mag_x_filt,
        "real.mag_y_filt"        => |dp| dp.real_vars.mag_y_filt,
        "real.omega_from_accel"  => |dp| dp.real_vars.omega_from_accel,
        "real.centripetal_ms2"   => |dp| dp.real_vars.centripetal_ms2,
        "real.dshot_left"        => |dp| dp.real_vars.dshot_cmd_left,
        "real.dshot_right"       => |dp| dp.real_vars.dshot_cmd_right,
        "real.batt_voltage"      => |dp| dp.real_vars.batt_voltage,
        "real.erpm_left"         => |dp| dp.real_vars.erpm_left,
        "real.erpm_right"        => |dp| dp.real_vars.erpm_right,
        /* ── REPLAYED: filter free-running from the first frame (current code) ── */
        "rep.kf_theta"           => |dp| dp.rep_state.kf_theta,
        "rep.kf_omega"           => |dp| dp.rep_state.kf_omega,
        "rep.theta_offset"       => |dp| dp.rep_state.theta_offset,
        "rep.est_theta"          => |dp| dp.rep_vars.est_theta,
        "rep.est_omega"          => |dp| dp.rep_vars.est_omega,
        "rep.heading_deg"        => |dp| dp.rep_vars.heading_deg,
        "rep.mag_angle"          => |dp| dp.rep_vars.mag_angle,
        "rep.mag_x_filt"         => |dp| dp.rep_vars.mag_x_filt,
        "rep.mag_y_filt"         => |dp| dp.rep_vars.mag_y_filt,
        "rep.omega_from_accel"   => |dp| dp.rep_vars.omega_from_accel,
        "rep.centripetal_ms2"    => |dp| dp.rep_vars.centripetal_ms2,
        "rep.dshot_left"         => |dp| dp.rep_vars.dshot_cmd_left,
        "rep.dshot_right"        => |dp| dp.rep_vars.dshot_cmd_right,
        "rep.batt_voltage"       => |dp| dp.rep_vars.batt_voltage,
        "rep.erpm_left"          => |dp| dp.rep_vars.erpm_left,
        "rep.erpm_right"         => |dp| dp.rep_vars.erpm_right,
        /* ── VARIABLES: pure functions of the INPUTS only (no filter state), so the
         * real and replayed series are identical — one shared 1 kHz series, not a
         * real/replayed split. Sourced from rep_vars (free-runs at 1 kHz; these
         * fields are set at the TOP of sunshine_step before any state mutation, so
         * they equal the real values regardless of the replayed filter state). */
        "var.omega_from_accel"  => |dp| dp.rep_vars.omega_from_accel,
        "var.centripetal_ms2"   => |dp| dp.rep_vars.centripetal_ms2,
        "var.batt_voltage"      => |dp| dp.rep_vars.batt_voltage,
        "var.erpm_left"         => |dp| dp.rep_vars.erpm_left,
        "var.erpm_right"        => |dp| dp.rep_vars.erpm_right,
        "var.accel_x_ms2"       => |dp| dp.input.accel_x as f32 * ADXL_SCALE_MS2,
        "var.accel_y_ms2"       => |dp| dp.input.accel_y as f32 * ADXL_SCALE_MS2,
        "var.accel_z_ms2"       => |dp| dp.input.accel_z as f32 * ADXL_SCALE_MS2,
        /* Raw sensor inputs (shared — no real/replayed distinction) */
        "input.accel_x"         => |dp| dp.input.accel_x as f32,
        "input.accel_y"         => |dp| dp.input.accel_y as f32,
        "input.accel_z"         => |dp| dp.input.accel_z as f32,
        "input.mag_x"           => |dp| dp.input.mag_x as f32,
        "input.mag_y"           => |dp| dp.input.mag_y as f32,
        "input.mag_magnitude"   => |dp| {
            let x = dp.input.mag_x as f32;
            let y = dp.input.mag_y as f32;
            let z = dp.input.mag_z as f32;
            (x*x + y*y + z*z).sqrt() * MAG_SCALE_UT
        },
        /* erpm_* on the wire is an IEEE-754 float16 BIT PATTERN (sunshine_core.h:176),
         * not a count — `as f32` plots the encoding, which is meaningless and would
         * render the NaN "ESC not commutating" sentinel as a ~64000 spike (design
         * §4.3). sunshine_f16_to_f32 preserves NaN (utils.c keeps `mant << 13` when
         * exp == 31), so the gap survives the decode all the way to the plot. */
        "input.erpm_left"       => |dp| f16_to_f32(dp.input.erpm_left),
        "input.erpm_right"      => |dp| f16_to_f32(dp.input.erpm_right),
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
        TelemetryFrame { frame_id: 0, state: SunshineState::default(),
                         state_mid: SunshineState::default(), inputs }
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

    #[test]
    fn downsample_keeps_nan_in_a_mixed_bucket() {
        // A bucket holding both finite samples and a NaN must report the finite
        // min/max — f32::min/max ignoring NaN was never the bug on THIS side; the
        // bug was that the NaN vanished without trace. Here we only assert the
        // finite extremes survive and no infinity leaks out.
        let raw: Vec<(u64, f32)> = vec![
            (0, 1.0), (1, f32::NAN), (2, 3.0), (3, 2.0),
            (4, 9.0), (5, 9.0),      (6, 9.0), (7, 9.0),
        ];
        let out = downsample_min_max(raw, 2);
        assert!(out.iter().all(|&(_, v)| !v.is_infinite()),
                "±inf must never be emitted: {out:?}");
        assert_eq!(out[0].1, 1.0, "mixed bucket min must be the finite min");
        assert_eq!(out[1].1, 3.0, "mixed bucket max must be the finite max");
    }

    #[test]
    fn downsample_all_nan_bucket_yields_nan_not_infinity() {
        // The all-NaN bucket must yield NaN, not (+inf, -inf). Both serialize to
        // JSON null, so a test that only checks "renders as a gap" passes either
        // way — assert on is_nan/!is_infinite explicitly.
        let raw: Vec<(u64, f32)> = vec![
            (0, f32::NAN), (1, f32::NAN), (2, f32::NAN), (3, f32::NAN),
            (4, 5.0),      (5, 5.0),      (6, 5.0),      (7, 5.0),
        ];
        let out = downsample_min_max(raw, 2);
        assert!(out[0].1.is_nan() && out[1].1.is_nan(), "all-NaN bucket must be NaN");
        assert!(out.iter().all(|&(_, v)| !v.is_infinite()));
    }

    #[test]
    fn downsample_emits_two_points_per_bucket_even_when_all_nan() {
        // Point count must not depend on measurability: every non-real.* channel
        // shares one downsampled time grid, and UPlotCanvas unions those grids.
        // A short series here would inject foreign timestamps into every other
        // channel and punch spurious gaps into them.
        let n = 8usize;
        let nan_series: Vec<(u64, f32)> = (0..n).map(|i| (i as u64, f32::NAN)).collect();
        let fin_series: Vec<(u64, f32)> = (0..n).map(|i| (i as u64, i as f32)).collect();
        let a = downsample_min_max(nan_series, 2);
        let b = downsample_min_max(fin_series, 2);
        assert_eq!(a.len(), b.len());
        let ta: Vec<u64> = a.iter().map(|p| p.0).collect();
        let tb: Vec<u64> = b.iter().map(|p| p.0).collect();
        assert_eq!(ta, tb, "time grid must be identical regardless of NaN content");
    }

    #[test]
    fn input_erpm_is_decoded_from_float16_bits() {
        // input.erpm_* is a float16 bit pattern, not a count. 0x7E00 is the
        // "ESC not commutating" NaN; casting the bits would give 32256.
        let f = crate::ffi::f32_to_f16(1234.0);
        let dp = DataPoint { input: SunshineInput { erpm_left: f, erpm_right: 0x7E00,
                                                    ..SunshineInput::default() },
                             ..DataPoint::default() };
        let v = channel_accessor("input.erpm_left")(&dp);
        assert!((v - 1234.0).abs() < 1.0, "expected ~1234 RPM, got {v}");
        assert!(channel_accessor("input.erpm_right")(&dp).is_nan(),
                "a float16 NaN payload must decode to NaN, not a ~32256 spike");
    }

    #[test]
    fn graph_data_multi_returns_equal_length_series_for_same_window() {
        let mut p = Pipeline::new();
        for i in 0..5u32 {
            let input = SunshineInput { time_us: i * 1_000_000, ctrl_x: i as i8, ..SunshineInput::default() };
            p.ingest_frame(&make_frame([input; INPUTS_PER_FRAME]));
        }
        let channels = vec!["input.ctrl_x".to_string(), "input.ctrl_y".to_string()];
        let series = p.get_graph_data_multi(&channels, 0, 5_000_000, 1000);
        assert_eq!(series.len(), 2);
        // Same window, same underlying data -> every channel's series must be
        // the same length, otherwise the chart can't align them by index.
        assert_eq!(series[0].len(), series[1].len());
        assert!(!series[0].is_empty());
    }
}
