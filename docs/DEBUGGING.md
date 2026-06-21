# Debugging with Replay

Instructions for the `sunshine:replay-debug` Claude skill. Use this skill when you have a `.sun` log file and want to debug filter behavior, investigate unexpected robot behavior, or tune parameters using real recorded data.

---

## What the replay-debug skill does

The skill loads a `.sun` log file, replays all 1 kHz sensor frames through `sunshine_step()` (the same C code that ran on the robot), and lets you inspect any variable at any time. Because replay is deterministic — identical inputs + state → identical outputs — you can change tuning parameters and re-run to see exactly how the filter would have behaved.

---

## When to use it

- **Filter not converging:** LED sweeping, `est_theta` drifting, mag update not helping
- **Unexpected DShot outputs:** Robot not translating in expected direction, one ESC saturating
- **Spin-up anomalies:** `omega_from_accel` too high during spin-up, accel saturation flags firing unexpectedly
- **Investigating a specific event:** "At t=12.4s the robot suddenly changed direction — what happened to the filter?"
- **Parameter tuning without hardware:** Try different `KF_R_ACCEL`, `KF_R_MAG`, `DRIFT_AMPLITUDE` values on recorded data

---

## Log file location

Log files are written by the host app to:
```
~/Documents/sunshine_logs/YYYY-MM-DD_HH-MM-SS[_label].sun
```

The host app shows the current log file path in the logging status bar.

---

## Log file format (for reference)

**Current format is FILE_FORMAT_VER 2 (95-byte header).** VER 1 files (93-byte header, 20 inputs/frame) are rare and from early sessions.

```
Header (95 bytes, FILE_FORMAT_VER 2):
  magic[5]        = "SHINE"
  file_format     = 2 (uint16 LE)           ← was 1 in old files
  header_size     = 95 (uint16 LE)          ← was 93 in old files
  schema_version  = uint32 LE               (bumped when structs change)
  sizeof_input    = 29 (uint16 LE)
  sizeof_state    = 60 (uint16 LE)
  sizeof_vars     = 56 (uint16 LE)          ← was 52 in old files (heading_deg added)
  created_at_ms   = Unix timestamp ms (uint64 LE)
  source          = 0=live, 1=replay, 2=simulation (uint8)
  flags           = bit0=logging_complete (uint8)
  label[64]       = null-terminated UTF-8
  num_inputs      = uint16 LE               ← VER 2 ONLY: inputs per frame (now 20; early VER 2 logs were 6)

Frame (701 bytes at num_inputs=20, 50 Hz):
  frame_id         = uint32 LE, monotonic (gaps = dropped telemetry)
  frame_flags      = bit0=vars_valid (uint8)
  SunshineState    = 60 bytes (state at START of frame, before inputs)
  SunshineInput×20 = 580 bytes (20 consecutive 1 kHz inputs)  ← was ×6 in early VER 2 logs
  SunshineVars     = 56 bytes (computed after all 20 steps)
```

The brain now sends one 50 Hz telemetry packet per 20 inputs over **ESP-NOW v2**
(643-byte payload; the old 250-byte cap had forced 6-input/~167 Hz frames). The
larger per-packet buffer also cuts the telemetry packet rate ~3.3×, relieving the
ring-buffer backpressure that was dropping ~5% of 1 kHz inputs.

**Frame size formula:** `5 + sizeof_state + sizeof_input × num_inputs + sizeof_vars`
Always read `num_inputs` from the header (bytes [93..94]) — do NOT hardcode it.

**SunshineVars field order** (56 bytes packed):
```
float  omega_from_accel, derot_I, derot_Q, mag_angle, est_theta, est_omega,
       dshot_cmd_left, dshot_cmd_right, batt_voltage, erpm_left, erpm_right,
       centripetal_ms2;     ← 12 floats = 48 bytes
uint8  led_on, accel_saturated, mag_valid, loop_overrun;  ← 4 bytes
float  heading_deg;         ← 4 bytes (added in schema v2; was not in 52-byte vars)
```

---

## Channels available for plotting / inspection

All channels are available in the host app's channel selector when replaying. The same list is available to the replay-debug skill.

**Inputs (1 kHz, 20 per frame in current VER 2 files):**
- `inputs.time_us` — timestamp in µs
- `inputs.accel_x/y/z` — raw ADXL375 counts
- `inputs.mag_x/y/z` — raw LIS3MDL counts
- `inputs.erpm_left/right` — float16-encoded eRPM
- `inputs.rssi` — brain-side RSSI (dBm)
- `inputs.ctrl_x/y/theta` — driver inputs [-127, 127]
- `inputs.ctrl_throttle` — [0, 255]
- `inputs.batt_offset` — relative to 7.6V
- `inputs.dshot_left_q/right_q` — previous-tick DShot, quantised [0, 255]
- `inputs.mode` — 0=DISABLED, 1=TANK, 2=MELTY

**State (50 Hz, one per frame — the state at the START of the 20-input block):**
- `state.kf_theta` — raw Kalman angle (rad, unwrapped)
- `state.kf_omega` — raw Kalman omega (rad/s)
- `state.kf_P[0..3]` — covariance matrix elements
- `state.theta_offset` — heading reference offset (rad)

**Vars (50 Hz from file, 1 kHz from replay):**

In replay, `SunshineVars` is recomputed at 1 kHz (not just the 50 Hz snapshot in the file). The file's stored vars are available as `real.vars.*`; the recomputed vars are `replay.vars.*`.

- `vars.omega_from_accel` — rad/s (inflated during spin-up)
- `vars.derot_I/Q` — derotated LP-filtered mag components (µT)
- `vars.mag_angle` — atan2(Q,I) in rad
- `vars.est_theta/omega` — Kalman output
- `vars.dshot_cmd_left/right` — full-precision DShot command [0, 2047]
- `vars.batt_voltage` — actual voltage (V)
- `vars.erpm_left/right` — decoded from float16
- `vars.centripetal_ms2` — sqrt(ax²+ay²) × scale
- `vars.led_on` — 1 within ±3° of heading zero
- `vars.accel_saturated` — 1 when centripetal > 280g
- `vars.mag_valid` — 1 when omega > 4π rad/s
- `vars.loop_overrun` — 1 if 1 kHz loop exceeded budget
- `vars.heading_deg` — robot heading [0, 360), matches LED zero (added in schema v2)

---

## Common debugging scenarios

### Scenario 1: LED sweeping — theta not locking

**What to look at:**
1. `vars.mag_valid` — is it staying 1? If it drops, omega fell below 120 RPM.
2. `vars.est_omega` vs `vars.omega_from_accel` — does omega track correctly?
3. `vars.derot_I` and `vars.derot_Q` — are they near-constant DC? If they oscillate, the LP filter hasn't settled.
4. `state.kf_P[0]` — is the angle covariance decreasing? It should drop from 100 toward near-zero after the mag update engages.

**Try:** Set `KF_R_MAG` lower and replay. Does theta converge faster?

**What to expect from real vs. simulated mag data:**
- Real `inputs.mag_x/y`: large constant offset (~−95 µT X, ~+103 µT Y from motor hard-iron) plus a ~25 µT Earth-field sine wave. The LIS3MDL y-axis is physically inverted on the PCB, so `my = −E·sin(φ−θ)` (negated relative to the naive model). This is what makes the derotation algebra work — do not "fix" it.
- Sim `inputs.mag_x/y`: same convention with hard-iron and horizontal-only Earth field (25 µT, not 50 µT total).

### Scenario 2: omega_from_accel reads wrong

**What to look at:**
1. `vars.accel_saturated` — if this is 1 at unexpectedly low speeds, check IMU calibration or wiring.
2. `inputs.accel_x` and `inputs.accel_y` during steady spin — both should have similar magnitude if IMU is at 45°. If one dominates, IMU angle may be off.
3. `vars.centripetal_ms2` — at 500 RPM: `ω² × r = (52.4)² × 0.011 ≈ 30.2 m/s²`. Check if this matches.

### Scenario 3: DShot commands wrong in MELTY

**What to look at:**
1. `inputs.ctrl_x/y/theta` — are the driver inputs what you intended?
2. `vars.dshot_cmd_left` vs `vars.dshot_cmd_right` — plot together. In MELTY with forward input (ctrl_y > 0), the two signals should alternate: left high when pointing forward, right high when pointing backward.
3. `state.theta_offset` — if this has accumulated from left/right arrow presses, the heading reference is rotated.
4. `vars.est_theta` — check the angle was valid during the manoeuvre.

### Scenario 4: Accel saturation anomaly

**What to look at:**
1. `vars.centripetal_ms2` vs time — plot with the saturation threshold line at `280 × 9.81 = 2746.8 m/s²`
2. `vars.accel_saturated` — note how long it stays high
3. At what RPM does saturation occur? Expected: ~4800 RPM. If it saturates at lower speeds, check that the IMU is reading correctly (accel_z at rest should be ≈ +20 counts, not ≈ 0).

---

## Re-running with different parameters

To test a parameter change in replay:

1. Note the constant you want to change (e.g. `KF_R_MAG = 0.05` instead of `0.1`)
2. Edit `sunshine_core/include/sunshine_core.h`
3. Rebuild the app: `cd sunshine_app && pnpm tauri dev`
4. Reload the same `.sun` file in the Replay tab
5. Compare `replay.vars.est_theta` before and after

The replay always uses the currently compiled `sunshine_step()` — changing constants and recompiling is all that's needed.

---

## Reading the graph panel in replay

- **Solid lines:** Replayed series (1 kHz, full precision, recomputed)
- **Dotted/thin lines:** Real series (50 Hz, as stored in the file)
- **DShot:** Real = `inputs.dshot_left_q` (quantised 0–255, decoded). Replayed = `vars.dshot_cmd_left` (full 0–2047 float).
- **Zoom:** Ctrl+scroll
- **Pan:** Scroll
- **Time reference:** x-axis is `time_us` from `SunshineInput`, so it matches the robot's boot clock.

---

## Offline replay harness (`tools/replay/`) — CLI, no app required

For quick command-line analysis of a `.sun` file (CI, scripts, ad-hoc debugging)
without spinning up the Tauri app, use the standalone replay harness. It **links
the real `sunshine_core` sources** (no logic is reimplemented) and re-runs the
log's 1 kHz inputs through `sunshine_step()`, dumping every recomputed channel as
CSV at the full 1 kHz rate. This is the same code path the app's replay uses;
it's just a thin IO/glue layer so the parsing never has to be rewritten per task.

**Files:**
- `tools/replay/replay.c` — the harness (reads header-driven sizes + `num_inputs`,
  so it adapts to schema/format changes; unpacks fields from the packed on-disk
  offsets, never memcpy, so MSVC padding is irrelevant).
- `tools/replay/CMakeLists.txt` — cross-platform build (Windows MSVC / macOS /
  Linux). Also builds the `sunshine_core` unit tests on any toolchain.
- `tools/replay/msvc_compat.h` — force-included **only on MSVC** to strip GCC's
  `__attribute__((packed))` (native on gcc/clang).
- `tools/replay/analyze.py` — example analyses over the CSV (validate / gaps /
  precession). Reads only the CSV; reimplements no robot logic.

**Build (cross-platform — needs CMake + any C compiler):**
```bash
cd tools/replay
cmake -B build -S .          # configure (auto-detects MSVC / gcc / clang)
cmake --build build          # -> build/replay(.exe)  (+ test_* exes)
ctest --test-dir build       # run the sunshine_core unit tests
```
The `replay` binary lands in `build/` (or `build/Debug/` with the MSVC
multi-config generator). Examples below write to `/tmp`; on Windows use any
writable path.

**Run:**
```bash
# Continuous replay (seed state ONCE, free-run) — faithful 1 kHz trajectory:
build/replay LOG.sun > cont.csv
# Per-frame replay (re-seed from each frame's stored state) — for validation:
build/replay LOG.sun --reseed > reseed.csv
# Restrict to a time window (boot-clock microseconds):
build/replay LOG.sun --from-us 89628463 --to-us 90539463 > window.csv
```

CSV columns: `time_us, mode, kf_theta, kf_omega, omega_accel, mag_angle,
est_theta, est_omega, derot_I, derot_Q, heading_deg, led_on, mag_valid,
accel_sat, dshot_l, dshot_r, mag_x, mag_y, accel_x, accel_y, theta_offset,`
and (frame-end rows only) `stored_est_theta, stored_mag_angle, stored_led_on`.
The `stored_*` columns are the **real on-robot** values from the file — pair them
with `--reseed` to validate replay reproduces the robot bit-for-bit.

**`--reseed` vs continuous:** `--reseed` seeds state from each frame's logged
"state at start" and should reproduce the stored vars *exactly* (replay
determinism check). Continuous free-runs from the first frame; to keep that
faithful when the log has dropped 1 kHz inputs, the harness **dead-reckons the
filter across detected timestamp gaps** (the robot ran 1 kHz continuously — only
the telemetry link dropped frames). Use `analyze.py gaps` to see the drop rate.

**Example analyses:**
```bash
python ../analyze.py validate   reseed.csv  # replay == real? (~0 deg)   [path: tools/replay/analyze.py]
python ../analyze.py gaps        cont.csv    # dropped-input check
python ../analyze.py precession  cont.csv    # LED drift vs raw-mag truth
```

The `precession` check is a useful pattern: it derives **ground-truth spin rate
straight from the raw magnetometer** (the field vector rotates once per
revolution about its hard-iron centre), independent of the filter, and compares
it to the LED heading-reference rate. Any difference is how fast the LED dot
precesses. The same raw-mag trick gives a filter-independent reference for
`est_omega` and `omega_from_accel` accuracy.
