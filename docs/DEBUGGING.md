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

**Current format is FILE_FORMAT_VER 3 (95-byte header).** Readers must still
handle VER 2 (one state + a vars block) and VER 1 (93-byte header) for old logs.

```
Header (95 bytes, FILE_FORMAT_VER 3):
  magic[5]        = "SHINE"
  file_format     = 3 (uint16 LE)           ← 2 in older files
  header_size     = 95 (uint16 LE)          ← was 93 in VER 1
  schema_version  = uint32 LE               (bumped when structs change)
  sizeof_input    = 29 (uint16 LE)
  sizeof_state    = 44 (uint16 LE)          ← schema v3 (was 60); see SunshineState
  sizeof_vars     = 0  (uint16 LE)          ← VER 3: NO vars block (was 56 in VER 2)
  created_at_ms   = Unix timestamp ms (uint64 LE)
  source          = 0=live, 1=replay, 2=simulation (uint8)
  flags           = bit0=logging_complete (uint8)
  label[64]       = null-terminated UTF-8
  num_inputs      = uint16 LE               ← inputs per frame (20)

Frame (673 bytes at num_inputs=20, schema v3): VER 3 carries TWO states, no vars
  frame_id         = uint32 LE, monotonic (gaps = dropped telemetry)
  frame_flags      = uint8
  SunshineState    = 44 bytes  (REAL state at the START of the frame)
  SunshineState    = 44 bytes  (REAL state at the MIDPOINT input → 100 Hz state)
  SunshineInput×20 = 580 bytes (20 consecutive 1 kHz inputs)
```

Two state snapshots per 50 Hz frame give the **real filter state at 100 Hz**.
**Vars are NOT logged** — they are a pure function of (state, inputs); the host
recomputes them, both a *real* series (filter re-anchored to the logged state
each frame + midpoint) and a *replayed* series (filter free-running from the
first frame). The brain sends one 50 Hz packet per 20 inputs over **ESP-NOW v2**
(671-byte payload = 3 + 2×44 + 20×29; ESP-NOW v2 / IDF ≥ 5.4 is required for the
>250-byte payload).

**Frame size formula:** `5 + sizeof_state × num_states + sizeof_input × num_inputs + sizeof_vars`
where `num_states = 2` for VER ≥ 3 else 1, and `sizeof_vars = 0` for VER ≥ 3.
Always read `num_inputs` from the header (bytes [93..94]) — do NOT hardcode it.

**VER 2 (legacy) frame** for reference: `frame_id(4) + flags(1) + SunshineState(60)
+ SunshineInput×N + SunshineVars(sizeof_vars)` — one state, a trailing vars block.

**SunshineVars field order** (56 bytes packed) — recomputed by the host, **not
stored** in VER 3 logs (kept here as the struct reference):
```
float  omega_from_accel, mag_x_filt, mag_y_filt, mag_angle, est_theta, est_omega,
       dshot_cmd_left, dshot_cmd_right, batt_voltage, erpm_left, erpm_right,
       centripetal_ms2;     ← 12 floats = 48 bytes
uint8  led_on, accel_saturated, mag_valid, loop_overrun;  ← 4 bytes
float  heading_deg;         ← 4 bytes (added in schema v2)
```

---

## Channels available for plotting / inspection

The host app channel selector groups channels into **REAL** and **REPLAYED**
series (plus shared **Inputs**). Both series are host-computed via `sunshine_step`:

- **REAL** (`real.*`): a filter re-anchored to the logged real state at the start
  of every frame and again at its midpoint (100 Hz). Reproduces the robot's own
  1 kHz trajectory (given matching code).
- **REPLAYED** (`rep.*`): a filter free-running continuously from the first frame.
  Shows what the *current* `sunshine_step` produces across the whole record — diff
  it against `real.*` to see the effect of a code/tuning change.

**Inputs (1 kHz, shared — no real/replayed split):**
- `input.accel_x/y/z`, `input.accel_x/y/z_ms2` — raw ADXL375 counts / m·s⁻²
- `input.mag_x/y`, `input.mag_magnitude` — raw LIS3MDL counts / µT
- `input.erpm_left/right`, `input.ctrl_x/y/theta`, `input.ctrl_throttle`
- `input.rssi`, `input.batt_offset`

**REAL state + vars** (`real.*`) — and the identical set under **`rep.*`**:
- State: `kf_theta` (rad), `kf_omega` (rad/s), `theta_offset` (rad)
- Vars: `est_theta`, `est_omega`, `heading_deg`, `mag_angle`, `mag_x_filt`, `mag_y_filt`,
  `omega_from_accel`, `centripetal_ms2`, `dshot_left`, `dshot_right`,
  `batt_voltage`, `erpm_left`, `erpm_right`

So e.g. `real.dshot_left` vs `rep.dshot_left` compares the real motor command to
what the current code would output; `real.kf_theta` vs `rep.kf_theta` shows
heading-estimate divergence.

**Offline `replay.exe` CSV columns** (single series; see the harness section
below): `time_us, mode, ctrl_x, ctrl_y, ctrl_theta, ctrl_throttle,
input_dshot_l, input_dshot_r, input_dshot_l_q, input_dshot_r_q, kf_theta,
kf_omega, omega_accel, mag_angle, est_theta, est_omega, mag_x_filt, mag_y_filt,
heading_deg, led_on, mag_valid, accel_sat, dshot_l, dshot_r, mag_x, mag_y,
accel_x, accel_y, theta_offset` and (VER 2 logs only, at frame-end rows)
`stored_est_theta, stored_mag_angle, stored_led_on`.

---

## Common debugging scenarios

### Scenario 1: LED sweeping — theta not locking

**What to look at:**
1. `vars.mag_valid` — is it staying 1? If it drops, omega fell below the mag threshold (480 RPM).
2. `vars.est_omega` vs `vars.omega_from_accel` — does omega track correctly?
3. `vars.mag_x_filt` and `vars.mag_y_filt` — the band-passed Earth sine; their magnitude `sqrt(x²+y²)` should be a steady ~18–22 µT. If it collapses, the spin is below the mag threshold, or `omega_from_accel` is so far off that the spin frequency has fallen outside the ±33% tracking band.
4. `state.kf_P[0]` — is the angle covariance decreasing? It should drop from 100 toward near-zero after the mag update engages.

**Try:** Set `KF_R_MAG` lower and replay. Does theta converge faster?

**Heading PRECESSION (LED rotates slowly) — the band-pass must be centred on
`omega_from_accel`, NOT `kf_omega`.** Root cause history: an earlier *closed-loop*
synchronous demodulator derotated by `kf_theta`; a later revision used an
open-loop band-pass but still took its centre frequency from `kf_omega`. Because
the band-pass output feeds `kf_omega` back, the per-tick coefficient retuning from
that fed-back rate **parametrically false-locked the recovered heading at half the
true spin rate** (sim: true 201.7 rad/s → 108.6; `kf_theta` a curvy, not straight,
sawtooth). The fix (implemented): `mag_heading.c` centres the band-pass on
`omega_from_accel`, which is a direct accel measurement **independent of the
estimate**, breaking the loop. The accel is therefore trusted fully for the rate
again (`KF_R_ACCEL` always — the old `KF_R_ACCEL_LOCKED` down-weighting is gone),
so `kf_omega` tracks `omega_from_accel`. Measure with `analyze.py precession` (raw
mag = filter-independent ground truth): the LED rate should match the raw-mag rate,
and the band-passed field magnitude should be a steady ~18–22 µT.

**What to expect from real vs. simulated mag data:**
- Real `inputs.mag_x/y`: large constant offset (~−95 µT X, ~+103 µT Y from motor hard-iron) plus a ~25 µT Earth-field sine wave. The LIS3MDL y-axis is physically inverted on the PCB, so `my = −E·sin(φ−θ)` (negated relative to the naive model). This is what the `-my_hp` in the heading `atan2` accounts for (`mag_heading.c`) — do not "fix" it.
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

CSV columns: `time_us, mode, ctrl_x, ctrl_y, ctrl_theta, ctrl_throttle,
input_dshot_l, input_dshot_r, input_dshot_l_q, input_dshot_r_q, kf_theta,
kf_omega, omega_accel, mag_angle, est_theta, est_omega, mag_x_filt, mag_y_filt,
heading_deg, led_on, mag_valid, accel_sat, dshot_l, dshot_r, mag_x, mag_y,
accel_x, accel_y, theta_offset,` and (frame-end rows only)
`stored_est_theta, stored_mag_angle, stored_led_on`.
`input_dshot_l/r` are the robot-logged previous-tick DShot commands decoded from
the quantized `SunshineInput.dshot_*_q` fields; `dshot_l/r` are recomputed by the
current `sunshine_step()` for the current tick.
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
