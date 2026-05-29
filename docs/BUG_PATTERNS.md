# Bug Patterns

Recurring categories of bugs found during the May 2026 code review. Use this as a checklist when auditing new code.

---

## 1. Physics / Domain Model Wrong

Code compiles and runs but the underlying model of the hardware is incorrect.

**Examples found:**
- TANK mode control formula: `L = fwd+turn, R = fwd-turn` — for tangential wheels, same-direction motors spin the body; opposite-direction motors translate it. The formula was backwards: W caused body spin instead of forward translation.
- `motor_tick` clamped `omega` to `≥ 0` with `.max(0.0)` — AM32 3D mode supports bidirectional operation; reverse commands always produced zero omega.
- `dshot_to_throttle(0.0)` returned `-1.048` (full reverse) — DShot values 0–47 are special commands / disarmed, not valid throttle. Missing `if d < DSHOT_MIN { return 0.0; }` boundary check.

**Where to look:**
- Any place a physical quantity is mapped, scaled, or clamped: check the hardware datasheet or DShot spec matches the code's assumptions.
- Motor direction / sign conventions: define positive once and audit every place it appears.
- Boundary conditions at neutral / zero: off-by-one errors are common at the boundary between forward, neutral, and reverse.

---

## 2. Data Model Confusion (What's computed where)

The codebase has three distinct data origins — hardware inputs, received state, and host-computed variables. Code that confuses these categories produces silently wrong or permanently-zero values.

**The model (from ARCHITECTURE.md):**
- `SunshineInput` — raw sensor frames from hardware, received at 1 kHz (50 Hz over the wire as 20-input batches)
- `SunshineState` — Kalman filter state from the brain, transmitted in every 50 Hz telemetry frame
- `SunshineVars` — **never telemetered**, always recomputed on host via `brain_step()`

**Examples found:**
- `frame.state` (the received `SunshineState`) was never stored in `DataPoint` — discarded in `ingest_frame`. The "State (real)" UI group was permanently zeros because it read `real_vars`, which was only populated for log-file replay.
- `real_vars` was labeled "real" but was zero for all live and simulation data. The distinction between "replayed" and "real" vars made no sense for the live use case.
- `heading_deg` was placed in the "State" UI group; it is host-computed from state, so it belongs in Variables.
- `rep.est_theta` / `rep.est_omega` were in "State (replayed)" — they are host-computed variables, not received state.
- Wall-clock timestamps were used instead of hardware timestamps (fixed in a8f687f: `now_us()` → `expand_hw_time(input.time_us)`).

**Where to look:**
- Any new channel added to the graph: ask "is this received from hardware, or computed on host?" Use `hw.*` for received state, `rep.*` for host-computed, `input.*` for raw sensor frames.
- Anywhere `DataPoint` fields are read: confirm the field is actually populated for the source you care about (live / sim / replay).
- Any new field on `SunshineVars`: verify it is not being confused with `SunshineState`.

---

## 3. Resource Lifecycle Bugs

Objects that hold OS resources (threads, file handles, sockets) dropped without cleanup.

**Examples found:**
- `connect_serial` assigned the returned `SerialConnection` to `let _conn` — Rust drops it immediately at end of scope, but the background reader thread continued running (held its own `Arc`). `disconnect_serial` was a documented no-op. Reconnecting leaked a thread every time.
- `LogWriter::close()` was never called — the `logging_complete` flag at byte offset 28 was never written to `1`, so every log file appeared incomplete to readers.
- Both cases shared the pattern of a method that **must** be called for correctness but had no enforcement.

**Where to look:**
- Any struct with a `close()` / `shutdown()` / `stop()` method: implement `Drop` to call it, or audit every code path that discards the struct.
- Any `let _x = expensive_constructor()` — the leading `_` signals intentional discard in Rust, which is almost never correct for resource-holding types.
- Async tasks spawned with `tokio::spawn`: confirm the stop signal is reachable from all exit paths.

---

## 4. Silent No-ops (Feature Stubs Never Wired Up)

Code that compiles, appears functional, but does nothing.

**Examples found:**
- `read_frame()` in `replay.rs` existed but no Tauri command ever called it — the replay tab showed metadata but never played frames back.
- `disconnect_serial` contained only a comment: `// Connection dropped when SerialConnection is not stored; extend AppState if needed`.
- The `real_vars_snap: Option<&SunshineVars>` parameter was passed as `None` everywhere for live and sim, making the entire `real_vars` path in the pipeline dead code for those sources.

**Where to look:**
- Every Tauri command in `commands.rs`: does it actually do what its name says? Verify the happy path end-to-end.
- Every `Option` parameter: check all call sites for `None` — if all callers pass `None`, the `Some` branch is dead.
- UI buttons/tabs: manually trace from click handler → invoke → command → effect. A button that calls an invoke with no backend handler fails silently.

---

## 5. Kalman Filter Initial Conditions and Thresholds

The filter has structural behaviors at initialization and at low-speed edge cases that are easy to miss without running it against real data.

**Examples found:**
- `kf_P[3]` (omega variance) initialized to `100.0` — std dev of 10 rad/s, far too high. First accelerometer measurement jumped omega to ~7.8 rad/s.
- `omega_from_accel` is computed as `sqrt(centripetal / r)`. Below ~7.8 rad/s the centripetal acceleration is less than 1 ADXL count, so `omega_from_accel = 0` and `kalman_update_omega` is skipped. With no drag in the predict step, `kf_omega` gets stuck at the last measured value until the robot spins up again.

**Where to look:**
- `sunshine_state_init`: initial `kf_P` values control how aggressively the filter trusts its first measurement.
- Any `if condition { kalman_update_*() }` guard: consider what happens to the estimate when the condition is permanently false (e.g., robot at rest, saturated accelerometer, spinning below mag threshold).
- Quantization thresholds: any sensor with integer counts has a minimum detectable signal. Compute it and document when the filter goes open-loop.

---

## General Checklist for New Code

1. **Physics**: Can you trace every sign, scale factor, and direction convention to a hardware spec or measured value?
2. **Data origin**: Is each channel tagged with what produces it (hardware-received vs host-computed)?
3. **Resources**: Does every resource-holding struct have a `Drop` impl or an audited caller that always cleans up?
4. **Feature completeness**: Does the UI action trace all the way to a real effect? Test by clicking it.
5. **Filter edge cases**: What does the filter output when the robot is at rest? Spinning below each threshold? At maximum speed?
