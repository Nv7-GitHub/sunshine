# Sunshine Filter + Telemetry Fixes Implementation Plan

> **For agentic workers:** Steps use checkbox (`- [ ]`) syntax for tracking. Inline execution (no subagents) per user request.

**Goal:** Fix the MELTY heading false-lock (sim), stop the 1 kHz telemetry input drops (brain), and make the offline/app replay robust to any residual drops — so simulated and real data behave correctly and `real` matches `replay`.

**Architecture:** Three independent fixes. (1) `mag_heading.c` centres its tracking band-pass on the **loop-independent** accel rate instead of the fed-back `kf_omega`, breaking the parametric feedback that false-locks at half the true spin rate. (2) The brain's telemetry SPSC ring is enlarged (and drops are counted + printed over USB) so transient Core-0/WiFi preemption no longer silently discards 1 kHz inputs. (3) The host replay free-run dead-reckons the heading across input-timestamp gaps so lossy logs replay faithfully.

**Tech Stack:** C (sunshine_core, verified via the CMake replay harness + ctest), C++/Arduino-ESP32 (sunshine_brain, PlatformIO), Rust (sunshine_app Tauri backend, cargo test).

## Global Constraints

- **Schema is APPEND-ONLY and unchanged here.** `sizeof(SunshineState)==44`, `sizeof(SunshineInput)==29`, `sizeof(SunshineVars)==56`, `SUNSHINE_SCHEMA_VERSION==3`. None of these fixes add/remove/reorder struct fields — verify the FFI `const _` asserts and `tools/replay` still parse VER 3 logs after.
- **No git operations this session** (user instruction). Do not branch/commit; leave changes in the working tree.
- **`omega_from_accel` and `accel_saturated` are set on `vars` BEFORE `mag_heading_step` is called** (`brain.c:21,35` then `:57`) — Task 1 relies on this ordering; do not reorder `brain.c`.
- Build core/replay with the VS-bundled CMake: `"/c/Program Files/Microsoft Visual Studio/18/Community/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe"`.

---

### Task 1: Break the mag-heading feedback false-lock (sim "bad results")

**Root cause (proven from `Simulation.sun`):** `mag_heading.c:42` sets the band-pass centre `fc = kf_omega/2π`, but `kf_omega` is driven by the heading that band-pass produces (`kalman_update_theta` cross-term + accel down-weighting). The per-tick coefficient retuning from this fed-back, oscillating `kf_omega` parametrically false-locks the recovered heading at **half** the true spin rate (true 201.7 rad/s → `kf_omega` 105, `mag_angle` rate 108.6). A *constant* `fc` recovers the true rate; centring on the **loop-independent** `omega_from_accel` recovers it in every tested window (200.9 / 114.2 / 362.0 vs true 201.7 / 115.1 / 363.7). This is the curvy-(not-straight)-sawtooth and `kf_omega`≠`omega_from_accel` the user saw.

**Files:**
- Modify: `sunshine_core/src/mag_heading.c:35-69` (fc source)
- Modify: `sunshine_core/include/sunshine_core.h:56-78` (band-pass comment block)
- Modify: `sunshine_core/src/brain.c:42-44` (comment only — note fc is now accel-driven)
- Test: `sunshine_core/test/test_mag_heading.c`
- Docs: `docs/DEBUGGING.md` (Scenario 1 note), `docs/FILTER_MATH.md` (Step 2 heading section)

**Interfaces:**
- Consumes: `SunshineVars.omega_from_accel` (f32, rad/s, 0 during saturation/zero), `SunshineVars.accel_saturated` (u8) — both already populated by `brain.c` before `mag_heading_step`.
- Produces: no signature change. `mag_heading_step(const SunshineInput*, SunshineState*, SunshineVars*)` unchanged.

- [ ] **Step 1: Write the failing test** — append to `test_mag_heading.c` before `TEST_RESULTS();`, and fix the existing spin test so it no longer relies on `kf_omega` for centring.

Replace the existing spin-test setup (lines 28-31) with:
```c
    sunshine_state_init(&s);
    memset(&in, 0, sizeof(in));
    memset(&v,  0, sizeof(v));
    float A = 22.0f, f = 20.0f;     /* µT, Hz */
    /* Fix: the band-pass centres on the LOOP-INDEPENDENT accel rate, NOT kf_omega.
       Set kf_omega deliberately WRONG (0) to prove the heading no longer depends
       on it; supply the true rate via omega_from_accel as brain.c does. */
    s.kf_omega          = 0.0f;
    v.accel_saturated   = 0;
    v.omega_from_accel  = 2.0f * 3.14159265f * f;
```
And add `memset(&v, 0, sizeof(v));` immediately after `sunshine_state_init(&s);` in the DC-blocked section (line 16-17) so the first calls read a defined `omega_from_accel==0` (→ kf_omega fallback, fc clamps to 8 Hz; DC is still blocked by the band-pass zero).

Also append a dedicated regression test:
```c
    /* Regression: a badly-wrong kf_omega must NOT drag the recovered heading rate
       (the old kf_omega-fed band-pass false-locked at half the true rate). */
    sunshine_state_init(&s);
    memset(&in, 0, sizeof(in));
    memset(&v,  0, sizeof(v));
    float Af = 22.0f, ff = 30.0f;          /* true spin 30 Hz */
    v.accel_saturated  = 0;
    v.omega_from_accel = 2.0f*3.14159265f*ff;
    float prevf = 0.0f, sumf = 0.0f; int Mf = 0;
    for (int n = 0; n < 4000; n++) {
        s.kf_omega = (n & 1) ? 50.0f : 250.0f;   /* oscillate kf_omega wildly each tick */
        float ph = 2.0f*3.14159265f*ff*n/1000.0f;
        in.mag_x = (int16_t)(Af*cosf(ph)/MAG_SCALE_UT);
        in.mag_y = (int16_t)(Af*sinf(ph)/MAG_SCALE_UT);
        mag_heading_step(&in, &s, &v);
        if (n > 2000) { sumf += wrap_pi(v.mag_angle - prevf); Mf++; }
        prevf = v.mag_angle;
    }
    float ratef = sumf / (Mf * 0.001f);
    ASSERT_NEAR(fabsf(ratef), 2.0f*3.14159265f*ff, 6.0f,
                "heading tracks true spin despite a wildly oscillating kf_omega");
```

- [ ] **Step 2: Run the test to verify it fails** (current code centres on `kf_omega`)

```bash
cd tools/replay
CMAKE="/c/Program Files/Microsoft Visual Studio/18/Community/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe"
"$CMAKE" --build build --config Debug --target test_mag_heading
./build/Debug/test_mag_heading.exe
```
Expected: FAIL on "heading tracks true spin despite a wildly oscillating kf_omega" (and the modified spin test), because fc currently follows `kf_omega`.

- [ ] **Step 3: Implement the fix** — in `mag_heading.c`, replace the fc computation (lines 39-43):

```c
    /* Band-pass centre = spin frequency. Use the LOOP-INDEPENDENT accelerometer
     * rate (omega_from_accel), NOT kf_omega: the heading this band-pass produces
     * feeds kf_omega, so centring on kf_omega closes a feedback loop whose per-tick
     * coefficient modulation parametrically FALSE-LOCKS the recovered heading at
     * half the true spin rate. omega_from_accel is measured straight from the
     * accelerometer (independent of the heading), so it breaks the loop. During
     * accel saturation it is 0 → fall back to kf_omega for those brief ticks (far
     * too short to develop a false lock). brain.c sets both fields before this. */
    float spin_omega = (!v->accel_saturated && v->omega_from_accel > 0.0f)
                       ? v->omega_from_accel : s->kf_omega;
    float fc = spin_omega * (0.5f / M_PI_F);
    if (fc < MAG_BP_MIN_FC_HZ) fc = MAG_BP_MIN_FC_HZ;
```

- [ ] **Step 4: Run the unit test to verify it passes**

```bash
cd tools/replay
"$CMAKE" --build build --config Debug --target test_mag_heading
./build/Debug/test_mag_heading.exe
```
Expected: all assertions PASS.

- [ ] **Step 5: Run the FULL core test suite (no regressions)**

```bash
cd tools/replay
"$CMAKE" --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```
Expected: all tests pass (kalman, brain, control, utils, mag_heading).

- [ ] **Step 6: Validate against the real sim log via replay**

```bash
cd tools/replay
./build/Debug/replay.exe ../../sunshine_brain/logs/2026-06-22_02-24-00_Simulation.sun > dbg/sim_fixed.csv
".venv python" dbg/c.py   # reuse: prints kf_omega vs omega_accel + mag_angle rate in the 19-23s window
```
Expected: in the 39%-throttle window, `kf_omega` mean ≈ **200** rad/s (was 105) and `mag_angle` rotation rate ≈ **200** (was 108.6); `kf_theta` is a straight (constant-rate) sawtooth. Use `python ../analyze.py precession dbg/sim_fixed.csv` to confirm LED rate ≈ raw-mag truth.

- [ ] **Step 7: Update the documentation** — `sunshine_core.h:56-78`, `brain.c:42-44`, `docs/DEBUGGING.md` Scenario 1, and `docs/FILTER_MATH.md` to state the band-pass centres on `omega_from_accel` (loop-independent), with the kf_omega-saturation fallback; remove claims that "the estimate only picks the band centre, so it cannot drift" (it could, via the loop).

---

### Task 2: Stop the brain dropping 1 kHz telemetry inputs

**Root cause (proven from `grounddata.sun`):** frame_ids are perfectly contiguous (zero frame loss) yet ~13% of 1 kHz inputs are missing, in ~20 ms holes. The nav loop busy-waits a true 1 kHz and runs `sunshine_step` on every tick (the logged real state is continuous — verified: 3.0° residual assuming continuous vs 34.4° assuming frozen). The loss is the **64-deep SPSC telem ring overflowing**: `telemetry_task` (Core 0, prio 5) shares Core 0 with the WiFi task (prio ~23) doing 500 Hz control RX + 50 Hz telem TX; a >64 ms WiFi/Core-0 preemption fills the ring and `RingBuffer::push` drops the input (returns false). The 167→50 Hz migration enlarged each TX (671 B, longer airtime/Core-0 hog per send) without enlarging the 64-deep ring (was 10× margin at 6 inputs/frame, only 3.2× at 20), so per-stall drops grew. Drops are invisible because `vars.loop_overrun` is never telemetered and nothing counts them.

**Files:**
- Modify: `sunshine_brain/src/telemetry.cpp:22` (ring depth) and add a drop counter + accessor
- Modify: `sunshine_brain/src/telemetry.h` (declare `telemetry_dropped_count()`)
- Modify: `sunshine_brain/src/nav_control.cpp:141-145,162-166` (count drops, print over USB in the existing PROF block)

**Interfaces:**
- Produces: `uint32_t telemetry_dropped_count(void);` — cumulative count of inputs `telemetry_push` could not enqueue.
- Consumes: existing `bool telemetry_push(const SunshineInput*, const SunshineState*)` return value (already returned, currently only sets `vars.loop_overrun`).

**Migration audit result (record in commit message / notes, no code needed):** Searched brain + receiver + host for 6-input / 167 Hz / 250-byte assumptions. The host (`protocol.rs`, `pipeline.rs`, `logging.rs`) and receiver (`espnow_rx.cpp`, `usb_bridge.cpp`) are all derived from `INPUTS_PER_FRAME`/`ESPNOW_TELEM_SIZE` and scale correctly. The only size that did **not** scale with the migration is this brain ring depth. The receiver's 2-slot last-value `telem_buf` double buffer (`espnow_rx.cpp:19`) holds only the newest frame; it did not drop here (frame_ids contiguous, USB keeps up at 50 Hz) but a lost frame now costs 20 inputs vs 6 — noted as a latent risk, left unchanged (no evidence of loss, changing it risks new bugs).

- [ ] **Step 1: Enlarge the ring** — `telemetry.cpp:22`:

```cpp
// Sized for transient Core-0/WiFi preemption: at 1 kHz, 256 entries = 256 ms of
// slack (observed worst-case stalls were ~40 ms). 256×sizeof(TelemetryEntry≈73B)
// ≈ 18 KB SRAM (ESP32-S3 has 512 KB). Was 64 (only 64 ms) — too small once the
// 50 Hz / 20-input framing made each TX a longer Core-0 hog. Power of 2 required.
static RingBuffer<TelemetryEntry, 256> telem_ring;
```

- [ ] **Step 2: Add a drop counter + accessor** — `telemetry.cpp`: add near the other statics, and update `telemetry_push`:

```cpp
static std::atomic<uint32_t> dropped_inputs{0};

bool telemetry_push(const SunshineInput *in, const SunshineState *state) {
    TelemetryEntry e;
    e.input = *in;
    e.state = *state;
    bool ok = telem_ring.push(e);
    if (!ok) dropped_inputs.fetch_add(1, std::memory_order_relaxed);
    return ok;
}

uint32_t telemetry_dropped_count(void) {
    return dropped_inputs.load(std::memory_order_relaxed);
}
```
Add `#include <atomic>` if not present (ring_buffer.h already pulls it, but include explicitly).

- [ ] **Step 3: Declare the accessor** — `telemetry.h`: add `uint32_t telemetry_dropped_count(void);` next to `telemetry_push`.

- [ ] **Step 4: Surface drops over USB** — `nav_control.cpp`, inside the existing `if (++prof_n % 500 == 0)` PROF print (line ~162), append the drop count so a dev with USB sees it without any UI:

```cpp
                Serial.printf("PROF us: adxl=%u mag=%u batt=%u ctrl=%u step=%u dshot=%u rest=%u | total=%u max=%u | telem_drop=%u\n",
                              adxl_us, mag_us, batt_us, ctrl_us, step_us, dshot_us, rest_us, elapsed, mx_total,
                              telemetry_dropped_count());
```
(Include `telemetry.h` is already included in nav_control.cpp.)

- [ ] **Step 5: Compile-check the firmware** (no hardware needed — just confirm it builds)

```bash
cd sunshine_brain
# PlatformIO via penv (see memory: invoke pio through penv exe, never bare `pio`)
~/.platformio/penv/Scripts/pio.exe run -e <env>   # adjust env name from platformio.ini
```
Expected: compiles clean. If the penv path differs, locate it first (`ls ~/.platformio/penv/Scripts/`). If the toolchain is unavailable in this environment, record that the change is source-correct and defer the on-device build to the user.

- [ ] **Step 6: Verify RAM headroom** — confirm the build's `.bss`/`DRAM` usage report still fits (the ring adds ~14 KB over the old 64-entry ring). Note the static RAM figure from the build output.

---

### Task 3: Make host replay robust to dropped inputs (real vs replay for lossy logs)

**Root cause:** the host's REPLAYED free-run steps every surviving input as a fixed 1 ms (`brain_step` → `DT=0.001`) and **ignores `input.time_us` gaps entirely** (`pipeline.rs` `build_replay_cache`, `ingest_frame`, `get_graph_data_from_log_meta`). Across each dropped-input hole the robot advanced its heading by the real elapsed time but replay advances only 1 ms, so the REPLAYED heading falls behind — 93° mean error on `grounddata.sun` vs 13° when the gap is dead-reckoned. (The REAL series re-anchors per frame, so it stays correct; this only fixes the free-running REPLAYED series.) Even after Task 2 reduces drops, RF noise guarantees some, so replay should ride across them.

**Files:**
- Modify: `sunshine_core/src/brain.c` — expose nothing new; `kalman_predict` is already a non-static symbol (used by `tools/replay`).
- Modify: `sunshine_app/src-tauri/src/ffi.rs` (add `kalman_predict` extern + safe wrapper)
- Modify: `sunshine_app/src-tauri/src/pipeline.rs` (`step_point` gains a gap-aware variant; the three free-run loops track previous hw time and dead-reckon `rep_state`)
- Test: `sunshine_app/src-tauri/src/pipeline.rs` (`#[cfg(test)]`)

**Interfaces:**
- Consumes: `kalman_predict(s: *mut SunshineState, dt: f32)` from sunshine_core (declared `void kalman_predict(SunshineState*, float)` in `brain.c`/`kalman.c`).
- Produces: `pub fn kalman_predict_state(state: &mut SunshineState, dt: f32)` in `ffi.rs`; `step_point` signature unchanged (REAL still re-anchored by caller); rep_state dead-reckon handled inside the loops.

- [ ] **Step 1: Write the failing test** — append to `pipeline.rs` tests: feed two frames whose inputs jump 20 ms between them and assert the free-run `rep_state.kf_theta` advances by ≈ `kf_omega × gap`, not by 1 ms.

```rust
    #[test]
    fn replay_deadreckons_rep_state_across_input_gaps() {
        // Two frames; second frame's inputs start 20 ms after the first ends.
        // rep_state spins at a fixed omega; across the gap the replayed heading
        // must advance by omega*gap (dead-reckoned), not by a single 1 ms step.
        let mut p = Pipeline::new();
        let omega = 50.0f32;
        let mut s = SunshineState::default(); state_init(&mut s); s.kf_omega = omega;
        // frame A: inputs at 0,1,..,19 ms
        let mut a = [SunshineInput::default(); INPUTS_PER_FRAME];
        for (i, inp) in a.iter_mut().enumerate() { inp.time_us = (i as u32)*1000; inp.mode = 0; }
        let fa = TelemetryFrame { frame_id: 0, state: s, state_mid: s, inputs: a };
        // frame B: inputs at 40,41,..,59 ms  (20 ms hole after A's last input at 19 ms)
        let mut b = [SunshineInput::default(); INPUTS_PER_FRAME];
        for (i, inp) in b.iter_mut().enumerate() { inp.time_us = 40_000 + (i as u32)*1000; inp.mode = 0; }
        let fb = TelemetryFrame { frame_id: 1, state: s, state_mid: s, inputs: b };
        p.begin_sim();              // resets, seeds rep_state from first frame
        p.ingest_frame(&fa);
        p.ingest_frame(&fb);
        // rep.kf_theta at the first input of frame B vs the last input of frame A
        let last_a  = p.get_channel_snapshot(&["rep.kf_theta".into()], Some(19_000))[0].unwrap();
        let first_b = p.get_channel_snapshot(&["rep.kf_theta".into()], Some(40_000))[0].unwrap();
        let adv = (first_b - last_a).abs();
        // continuous robot would advance ~omega*21ms ≈ 1.05 rad across the hole+step;
        // the old fixed-1ms replay advances only ~omega*1ms ≈ 0.05 rad.
        assert!(adv > 0.5, "rep_state must dead-reckon the gap (got {adv} rad)");
    }
```

- [ ] **Step 2: Run it to verify it fails**

```bash
cd sunshine_app/src-tauri
cargo test replay_deadreckons_rep_state_across_input_gaps -- --nocapture
```
Expected: FAIL (`adv` ≈ 0.05, replay ignores the gap).

- [ ] **Step 3: Expose `kalman_predict` in `ffi.rs`** — add to the `extern "C"` block and a wrapper:

```rust
extern "C" {
    // ... existing ...
    fn kalman_predict(s: *mut SunshineState, dt: f32);
}

/// Dead-reckon the filter forward by `dt` seconds (predict only, no measurement).
/// Used to advance the free-running replay across dropped-input timestamp gaps.
pub fn kalman_predict_state(state: &mut SunshineState, dt: f32) {
    unsafe { kalman_predict(state as *mut _, dt) }
}
```

- [ ] **Step 4: Dead-reckon `rep_state` across gaps in the three free-run loops** — `pipeline.rs`. Add a helper and call it before each `brain_step` on `rep_state` in `ingest_frame` (`:90-94`), `get_graph_data_from_log_meta` (`:279-291`), and `build_replay_cache` (`:421-429`). Track the previous input hardware time per loop.

Helper (module-level in pipeline.rs):
```rust
/// If the input timeline jumped (a dropped-input hole the robot ran through but
/// telemetry didn't capture), advance the free-running rep filter across it so its
/// heading doesn't fall behind. The robot used a fixed 1 ms DT per real tick; we
/// approximate the missing ticks with one predict of (gap - 1 ms). >1.5 ms = a gap;
/// cap at 100 ms to ignore session restarts / clock resets.
fn deadreckon_gap(rep_state: &mut SunshineState, prev_hw_us: &mut Option<u32>, now_hw_us: u32) {
    if let Some(prev) = *prev_hw_us {
        let gap = now_hw_us.wrapping_sub(prev);
        if gap > 1500 && gap < 100_000 {
            kalman_predict_state(rep_state, (gap - 1000) as f32 / 1e6);
        }
    }
    *prev_hw_us = Some(now_hw_us);
}
```
In each loop, declare `let mut rep_prev_hw: Option<u32> = None;` alongside the existing `rep_seeded`, and immediately before the `brain_step(input, &mut rep_state)` (or inside `step_point` — see below) call `deadreckon_gap(&mut rep_state, &mut rep_prev_hw, input.time_us);`. For `ingest_frame`, store `rep_prev_hw` as a `Pipeline` field (reset in `begin_live`/`begin_sim` to `None`) since it persists across frames; for the two batch loops it is a local. Do NOT dead-reckon `real_state` (it is re-anchored every frame).

Concretely for `ingest_frame`: add field `rep_prev_hw: Option<u32>` to `Pipeline`, init `None` in `new`/`begin_live`/`begin_sim`, and in the input loop:
```rust
            let real_vars = brain_step(input, &mut self.real_state);
            deadreckon_gap(&mut self.rep_state, &mut self.rep_prev_hw, input.time_us);
            let rep_vars  = brain_step(input, &mut self.rep_state);
```
For `get_graph_data_from_log_meta` and `build_replay_cache`, add `let mut rep_prev_hw: Option<u32> = None;` next to `rep_seeded` and call `deadreckon_gap(&mut rep_state, &mut rep_prev_hw, input.time_us);` right before the `step_point(...)` call (and have `step_point` keep stepping both as today — the dead-reckon happens just before it on `rep_state`).

- [ ] **Step 5: Run the test to verify it passes**

```bash
cd sunshine_app/src-tauri
cargo test replay_deadreckons_rep_state_across_input_gaps -- --nocapture
```
Expected: PASS (`adv` ≈ 1.0 rad).

- [ ] **Step 6: Run the full backend test suite (no regressions)**

```bash
cd sunshine_app/src-tauri
cargo test
```
Expected: all pipeline/protocol tests pass.

- [ ] **Step 7: End-to-end check against the real log** (matches the harness's dead-reckon result)

Confirm via the already-built harness that dead-reckoning brings continuous replay to ~13° of the logged real state (vs 93° without): `tools/replay/build/Debug/replay.exe grounddata.sun` already dead-reckons; the app now matches that behaviour. Note the number in the verification summary.

---

## Verification Summary (run after all tasks)

- [ ] `ctest --test-dir tools/replay/build -C Debug` — all core tests green (incl. new mag-heading regression).
- [ ] Replay `Simulation.sun`: `kf_omega` tracks `omega_from_accel` (~200 in the 39% window), `kf_theta` straight sawtooth, `analyze.py precession` ≈ 0 drift.
- [ ] `cargo test` in `sunshine_app/src-tauri` — green incl. the new gap dead-reckon test.
- [ ] Firmware compiles (or source-correctness recorded if toolchain unavailable); note static RAM headroom.
- [ ] Schema asserts intact: `sizeof` State/Input/Vars unchanged; `tools/replay` still parses both logs.

## Self-Review Notes

- **Spec coverage:** Sim false-lock → Task 1. Brain input drops + migration audit → Task 2. Real-vs-replay (dropped-input cause) → Task 2 (root) + Task 3 (replay robustness). "Make errors obvious" → Task 2 Step 4 (USB drop counter; UI work skipped per user since the fix is brain-side).
- **No schema change** in any task → FFI asserts and VER-3 log parsing remain valid.
- **Type consistency:** `telemetry_dropped_count` (Task 2) and `kalman_predict_state`/`deadreckon_gap` (Task 3) names used consistently across their steps.
