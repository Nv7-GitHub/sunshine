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

**On this machine that is literally `~/Documents/sunshine_logs`** — the logs are large and
machine-local, so they are outside the repo and nothing in the tree references one by
path. Anything you are asked to investigate ("the translation log", "the run that
bounced") is a file in there. The host app also shows the current log file path in the
logging status bar.

The name starts with a zero-padded sortable timestamp, so the newest log is the last one
lexicographically — but **select by mtime**, because a session that is still recording (or
that crashed) keeps its original name while the file keeps growing:

```bash
ls -t ~/Documents/sunshine_logs/*.sun | head -1           # newest
LOG=$(ls -t ~/Documents/sunshine_logs/*.sun | head -1)    # ...and use it
build/replay "$LOG" > cont.csv
```

If the newest log's header `flags` bit0 (`logging_complete`) is clear it was never closed
cleanly — normal while the app is still running. It replays fine; the harness reads whole
frames only, so a torn trailing frame is simply dropped.

---

## Log file format (for reference)

**Current format is FILE_FORMAT_VER 3 (95-byte header), schema v5.** `sizeof_state`
and `sizeof_input` are read from the header, so the file format is unchanged when a
struct grows — only the schema_version bumps. Readers must still handle VER 2 (one
state + a vars block) and VER 1 (93-byte header) for old logs, AND any older
sizeof_state (schema v4 = 52, v3 = 44, v2 = 60) — always use the header's sizeof,
never a constant. **Schema v5 also widened `batt_offset` int8→int16** (a mid-struct
resize), so the input reader is schema-aware: pre-v5 logs (`schema_version < 5`,
`sizeof_input` 29) use the old int8 layout with `mode`/`dshot_*_q` one byte earlier,
and the old int8 battery LSB is remapped to the v5 int16 scale (×20.5). See
`replay.rs::read_input` / `replay.c::unpack_input`.

```
Header (95 bytes, FILE_FORMAT_VER 3):
  magic[5]        = "SHINE"
  file_format     = 3 (uint16 LE)           ← 2 in older files
  header_size     = 95 (uint16 LE)          ← was 93 in VER 1
  schema_version  = uint32 LE               (bumped when structs change; 5 now)
  sizeof_input    = 30 (uint16 LE)          ← schema v5 (was 29; batt_offset int8→int16)
  sizeof_state    = 56 (uint16 LE)          ← schema v5 (was 52 in v4, 44 in v3, 60 in v2); see SunshineState
  sizeof_vars     = 0  (uint16 LE)          ← VER 3: NO vars block (was 56 in VER 2)
  created_at_ms   = Unix timestamp ms (uint64 LE)
  source          = 0=live, 1=replay, 2=simulation (uint8)
  flags           = bit0=logging_complete (uint8)
  label[64]       = null-terminated UTF-8
  num_inputs      = uint16 LE               ← inputs per frame (20)

Frame (717 bytes at num_inputs=20, schema v5): VER 3 carries TWO states, no vars
  frame_id         = uint32 LE, monotonic (gaps = dropped telemetry)
  frame_flags      = uint8
  SunshineState    = 56 bytes  (REAL state at the START of the frame)
  SunshineState    = 56 bytes  (REAL state at the MIDPOINT input → 100 Hz state)
  SunshineInput×20 = 600 bytes (20 consecutive 1 kHz inputs, 30 B each)
```

Two state snapshots per 50 Hz frame give the **real filter state at 100 Hz**.
**Vars are NOT logged** — they are a pure function of (state, inputs); the host
recomputes them for a *replayed* series (filter free-running at 1 kHz from the
first frame). The *real* series is emitted **only at the two 100 Hz snapshots per
frame** (frame start + midpoint) and drawn as straight lines between them — a
*coarse* version of the replayed curve, not a fabricated 1 kHz staircase (the old
held-and-re-stepped approach produced steps plus per-tick noise and matched the
replayed dot count; see `pipeline.rs` `real_valid`). The brain sends one 50 Hz
packet per 20 inputs over **ESP-NOW v2** (715-byte payload = 3 + 2×56 + 20×30 at
schema v5; ESP-NOW v2 / IDF ≥ 5.4 is required for the >250-byte payload).

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

The host app channel selector groups channels into **Inputs**, **Variables**,
**REAL**, and **REPLAYED**:

- **REAL** (`real.*`): the **logged real state**, sampled at the two 100 Hz
  snapshots per frame and drawn as straight lines between them (coarse). This is the
  ground truth the robot recorded; `real.*` vars at a snapshot = one `sunshine_step`
  from that snapshot's state. It stays honest when the code changes and reads
  distinctly (coarse) from the smooth 1 kHz replayed curve.
- **REPLAYED** (`rep.*`): a filter free-running continuously at 1 kHz from the first
  frame via the *current* `sunshine_step`. Diff it against `real.*` to see the
  effect of a code/tuning change, or to view the 1 kHz detail between snapshots.
- **VARIABLES** (`var.*`): quantities that are a pure function of the **inputs only**
  (no filter state), so real and replayed are identical — one shared 1 kHz series.

**Inputs (1 kHz, shared — raw sensor data):**
- `input.accel_x/y/z` — raw ADXL375 counts
- `input.mag_x/y`, `input.mag_magnitude` — raw LIS3MDL counts / µT
- `input.erpm_left/right` (raw f16), `input.ctrl_x/y/theta`, `input.ctrl_throttle`
- `input.rssi`, `input.batt_offset` (int16 LSB, schema v5)

**Variables (1 kHz, shared — derived from inputs, no real/replayed split):**
- `var.omega_from_accel` (rad/s), `var.centripetal_ms2` (m·s⁻²)
- `var.accel_x/y/z_ms2` (m·s⁻²) — raw counts × ADXL scale
- `var.batt_voltage` (V), `var.erpm_left/right` (RPM)

**REAL state + vars** (`real.*`) — and the identical set under **`rep.*`** (these
depend on filter STATE, so they differ between the recorded run and a re-run):
- State: `kf_theta` (rad), `kf_omega` (rad/s), `theta_offset` (rad)
- Vars: `heading_deg`, `mag_angle`, `mag_x_filt`, `mag_y_filt`, `dshot_left`,
  `dshot_right`

So e.g. `real.dshot_left` vs `rep.dshot_left` compares the real motor command to
what the current code would output; `real.kf_theta` vs `rep.kf_theta` shows
heading-estimate divergence. Battery / eRPM / ω-from-accel / centripetal moved to
`var.*` because they're input-only (identical real vs replayed).

**Offline `replay.exe` CSV columns** (single series; see the harness section
below): `time_us, mode, ctrl_x, ctrl_y, ctrl_theta, ctrl_throttle,
input_dshot_l, input_dshot_r, input_dshot_l_q, input_dshot_r_q, kf_theta,
kf_omega, omega_accel, mag_angle, est_theta, est_omega, mag_x_filt, mag_y_filt,
heading_deg, led_on, mag_valid, accel_sat, dshot_l, dshot_r,
erpm_left, erpm_right, mag_x, mag_y, accel_x, accel_y, theta_offset,
stored_kf_theta, stored_kf_omega,
stored_theta_offset` and (VER 2 logs only, at frame-end rows)
`stored_est_theta, stored_mag_angle, stored_led_on`.
`stored_kf_theta/omega/theta_offset` are sparse: in VER 3 logs they are present
only on the rows corresponding to the frame-start and midpoint state snapshots.

### Interpreting eRPM — NaN means "not measurable", not "stopped"

Bidirectional-DShot eRPM is derived from the ESC's **own commutation timing**. Below
~5 % duty AM32 turns the FETs off and lets the motor coast, so there is nothing left to
time and the ESC emits a decaying, meaningless period. The firmware therefore reports
**NaN** whenever the previous tick's duty was below the commutation threshold, and the
graph draws that as a **gap**. Read it as *"the ESC was not commutating"* — the wheel
may well have been spinning fast.

`0` now means what it says: a genuinely stopped wheel. **Logs recorded BEFORE this change
encode unmeasurable as 0, not NaN** — every undriven sample in them was written as a hard
0, which is why old eRPM traces "drop to zero" for long stretches. There is no way to
recover the distinction after the fact in such a log; read its zeros as "unknown".

That the zeros were never real is measurable two ways in
`2026-07-20_04-20-24_translation2.sun`:

- **Prevalence.** 77.6 % of samples with the command at neutral read zero, against 0.01 %
  of driven samples. Zeros track *the command*, not the wheel.
- **The coast-down at t = 402.9 s**, which settles it outright. The command drops to 0
  while `real.kf_omega` stays flat at −94 rad/s, so the flywheel — and therefore the
  wheels — provably cannot have slowed, yet `erpm_left` "falls" 18416 → 3312 over ~120 ms
  and throws a 25504 spike on the way down. The wheels were still turning at ~11 500 eRPM
  throughout. All of it is decode garbage from a coasting ESC.

Practical notes:

- **Gaps in `var.erpm_left/right` are expected** at neutral and during the low half of
  the MELTY drift wave. Long gaps *while the throttle is up* are not — check the DShot
  command (`real.dshot_left/right`) actually left the commutation threshold.
- **A flat line with no gaps at all** across a neutral stretch means you are looking at
  a pre-NaN log (or the fake-zero path came back).
- `var.erpm_*` is the sanitised value (range gate against a command-derived ceiling,
  direction-aware deviation gate, median-5). `input.erpm_*` is the raw decoded float16
  — use it to see what the ESC actually reported before filtering.
- eRPM is **telemetry only**; nothing in the Kalman or control path reads it, so a bad
  eRPM stream cannot itself have caused a behaviour you are chasing.

---

## Common debugging scenarios

### Scenario 1: LED sweeping — theta not locking

**FIRST check spin DIRECTION (schema v4 fix).** `omega_from_accel = √(a_c/r)` is an
unsigned MAGNITUDE — the accel cannot tell CW from CCW. When the robot is **inverted**
(flipped: same chassis spin, opposite world spin), the magnetometer sees the reversed
rotation but the accel doesn't, so a heading slaved only to the accel counter-rotates
against the field at ~2× the spin rate (a total smear). Fix (`brain.c` + `mag_heading.c`):
the spin SIGN is taken from the mag rotation sense (`state.spin_rate_lp`) and copied onto
`omega_from_accel`; `mag_valid` uses `|kf_omega|`. Symptom of a regression here: `kf_theta`
winding opposite to the raw-mag field. Verify in replay that the LED heading rate matches
the raw-mag winding for BOTH orientations (drive the robot inverted in a test log).

**What to look at:**
1. `vars.mag_valid` — is it staying 1? If it drops, omega fell below the mag threshold (480 RPM).
2. `vars.est_omega` vs `vars.omega_from_accel` — does omega track (same sign)? `state.spin_rate_lp` sign = mag-derived spin direction.
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

### Scenario 5: Wheel overspeed / the robot bounces while spinning

Symptom: in MELTY the robot hops vertically instead of sitting flat. One measurable
cause is the wheels spinning far faster than the ground demands: each landing dumps the
surplus wheel kinetic energy in as an impulsive traction spike, which throws the robot
back into the air.

**The reference figure: 123 eRPM per rad/s of body rate.** With no slip the wheel rate
is pure geometry, `ω_wheel = |ω_body| × WHEEL_CENTER_M / WHEEL_RADIUS_M`, and the ESC
reports electrical rpm, so

```
eRPM_no_slip = |ω_body| × (0.0405 / 0.022) × 60/(2π) × 7  =  123.06 × |ω_body|
```

**What to look at:**
1. Plot `var.erpm_left` and `real.kf_omega` together and divide: at 100 rad/s expect
   ~12 300 eRPM. A ratio much above 1.0 is overspeed — the evidence log
   `2026-07-20_04-20-24_translation2.sun` sat at a median 1.30 and a p95 of 2.27.
2. The ratio has a hard physical **floor at 1.0** (a wheel cannot roll slower than the
   ground unless it is braking). If the measured floor is not within a few percent of
   1.0, your `WHEEL_RADIUS_M` / `WHEEL_CENTER_M` / pole-pair constants are wrong — fix
   the geometry before reading anything else into the ratio.
3. `real.dshot_left/right` vs `rep.dshot_left/right` shows what the wheel-speed cap
   would have done to a run recorded before it existed.
4. `tools/replay/wheel_slip.py` reports commanded vs. capped DShot and the resulting
   slip distribution over a whole log — use it to confirm or adjust
   `WHEEL_SLIP_ALLOW_MS` against real data rather than argument.

**What the ratio should look like once the cap is in.** Note first what replay can and
cannot tell you here: replaying an old log through the capped `control.c` recomputes the
*command*, but the eRPM in that log is still the uncapped run's wheel response. A true
post-cap ratio distribution can only come from a log recorded with the cap live. (What
*is* measurable on an old log is the commanded no-load speed over rolling speed — that
modelled ratio falls from a 1.63 median / 5.51 p95 to 1.30 / 2.46 on the evidence log,
an ~83 % cut in squared excess wheel speed, the stored-energy proxy that lands as an
impulse on touchdown. It is a statement about the command, not about the wheel.)

What the cap does guarantee is a per-rate ceiling. Unloaded — airborne, the case that
matters — the wheel reaches the commanded no-load speed, which is rolling speed plus the
allowance, so:

```
ratio <= 1 + WHEEL_SLIP_ALLOW_MS / (|w_body| * WHEEL_CENTER_M)
```

| body rate | 50 rad/s | 100 rad/s | 150 rad/s |
|---|---|---|---|
| ceiling at allow = 1.0 m/s | 1.49 | 1.25 | 1.17 |

Two things legitimately sit above that line and neither is a fault: the drift wave rides
above the capped `base` (that asymmetry *is* translation), and below the mag-lock
threshold the cap holds a fixed ceiling while the rolling term keeps falling, so the ratio
at very low spin is large by construction — at 20 rad/s the ceiling is ~3.8. Judge the
ratio only above lock. A surviving 2×+ tail at 100 rad/s or more means the cap is not
being applied: check the mode is MELTY (TANK is uncapped by design) and check
`var.batt_voltage` per the note below.

At `WHEEL_SLIP_ALLOW_MS = 1.0` the cap binds on ~76 % of driven MELTY samples, and the
amount it removes shrinks with spin rate — median ~170 DShot counts below 50 rad/s, ~68
between 50 and 100, ~25 between 100 and 150 — because the rolling term grows while the
fixed allowance does
not. A cap that binds hard at high rate is the anomaly worth chasing, not one that binds
often at low rate.

**Remember the cap needs the battery.** The ESC is an open-loop voltage source, so
wheel speed is set by `duty × V_batt`; the pack moved 7.80–8.39 V inside a single run.
If `var.batt_voltage` is implausible (< 5 V or > 10 V) the cap fails open by design —
check it before concluding the cap is not working.

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
- `tools/replay/wheel_slip.py` — wheel overspeed and MELTY speed-cap report (see
  Scenario 5): measured-vs-no-slip eRPM ratio with its geometry regression guard,
  how often the cap binds and by how much, the overspeed distribution before and
  after, and an `--allow` sweep so `WHEEL_SLIP_ALLOW_MS` can be re-chosen against a
  real log without rebuilding firmware. Reads only the CSV; the **cap arithmetic is
  the one piece of robot logic it mirrors**, deliberately, because that is the thing
  under evaluation — `--vs-replay` diffs the mirror against the `dshot_l/r` the
  compiled `control.c` produced, and a non-zero median there means the two have
  drifted apart and the report is stale.

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
heading_deg, led_on, mag_valid, accel_sat, dshot_l, dshot_r,
erpm_left, erpm_right, mag_x, mag_y, accel_x, accel_y, theta_offset,
stored_kf_theta, stored_kf_omega,
stored_theta_offset,` and (frame-end rows only)
`stored_est_theta, stored_mag_angle, stored_led_on`.
`input_dshot_l/r` are the robot-logged previous-tick DShot commands decoded from
the quantized `SunshineInput.dshot_*_q` fields; `dshot_l/r` are recomputed by the
current `sunshine_step()` for the current tick.
`stored_kf_theta/omega/theta_offset` are the real on-robot state snapshots from
the log. For VER 3 they appear at the frame-start row and midpoint row only; other
rows are blank because the log carries state at 100 Hz, not 1 kHz.
The `stored_*` columns are the **real on-robot** values from the file — pair them
with `--reseed` to validate replay reproduces the robot bit-for-bit.

**`--reseed` vs continuous:** `--reseed` seeds state from each frame's logged
"state at start" and should reproduce the stored vars *exactly* (replay
determinism check). Continuous free-runs from the first frame; to keep that
faithful when the log has a hole in the 1 kHz inputs, the harness **dead-reckons
the filter across detected timestamp gaps**. Use `analyze.py gaps` to see the rate.

**A timestamp gap has TWO possible causes — check `frame_id` to tell them apart:**
- **Dropped telemetry** — `frame_id` jumps by >1 across the gap. The robot ran
  1 kHz fine; the ESP-NOW link lost whole frames. (Reliable unicast makes this rare.)
- **Robot nav-loop STALL** — `frame_id` is *contiguous* across the gap, but the time
  between the last input of one frame and the first of the next is >1 ms. The robot
  itself stopped sampling. A known cause: `Serial.printf` (USB-CDC) blocking the
  1 kHz loop when no host drains the TX FIFO — fixed with `Serial.setTxTimeoutMs(0)`
  + `if (Serial)` guards (main.cpp / nav_control.cpp). A stall means the *robot's*
  control/heading froze for that long, not just a plotting hole — investigate it.

**Example analyses:**
```bash
python ../analyze.py validate   reseed.csv  # replay == real? (~0 deg)   [path: tools/replay/analyze.py]
python ../analyze.py gaps        cont.csv    # dropped-input check
python ../analyze.py precession  cont.csv    # LED drift vs raw-mag truth
python ../wheel_slip.py          cont.csv    # wheel overspeed + speed-cap report
python ../wheel_slip.py          cont.csv --allow 0.5,0.75,1.0,1.5,2.0 --vs-replay
```

The `precession` check is a useful pattern: it derives **ground-truth spin rate
straight from the raw magnetometer** (the field vector rotates once per
revolution about its hard-iron centre), independent of the filter, and compares
it to the LED heading-reference rate. Any difference is how fast the LED dot
precesses. The same raw-mag trick gives a filter-independent reference for
`est_omega` and `omega_from_accel` accuracy.
