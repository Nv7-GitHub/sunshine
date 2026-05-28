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

```
Header (93 bytes):
  magic[5]        = "SHINE"
  file_format     = 1 (uint16 LE)
  header_size     = 93 (uint16 LE)
  schema_version  = uint32 LE
  sizeof_input    = 29 (uint16 LE)
  sizeof_state    = 60 (uint16 LE)
  sizeof_vars     = 52 (uint16 LE)
  created_at_ms   = Unix timestamp ms (uint64 LE)
  source          = 0=live, 1=replay, 2=simulation (uint8)
  flags           = bit0=logging_complete (uint8)
  label[64]       = null-terminated UTF-8

Frame (697 bytes at schema v1, 50 Hz):
  frame_id        = uint32 LE, monotonic (gaps = dropped telemetry)
  frame_flags     = bit0=vars_valid (uint8)
  SunshineState   = 60 bytes (state at START of frame, before inputs)
  SunshineInput×20 = 580 bytes (20 consecutive 1 kHz inputs)
  SunshineVars    = 52 bytes (computed after all 20 steps — 50 Hz snapshot)
```

---

## Channels available for plotting / inspection

All channels are available in the host app's channel selector when replaying. The same list is available to the replay-debug skill.

**Inputs (1 kHz, 20 per frame):**
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

---

## Common debugging scenarios

### Scenario 1: LED sweeping — theta not locking

**What to look at:**
1. `vars.mag_valid` — is it staying 1? If it drops, omega fell below 120 RPM.
2. `vars.est_omega` vs `vars.omega_from_accel` — does omega track correctly?
3. `vars.derot_I` and `vars.derot_Q` — are they near-constant DC? If they oscillate, the LP filter hasn't settled.
4. `state.kf_P[0]` — is the angle covariance decreasing? It should drop from 100 toward near-zero after the mag update engages.

**Try:** Set `KF_R_MAG` lower and replay. Does theta converge faster?

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
