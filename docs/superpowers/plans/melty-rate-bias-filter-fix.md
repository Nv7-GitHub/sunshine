# MELTY Heading Rate-Bias Filter Fix — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: use superpowers:systematic-debugging (find/confirm root cause before fixing) and superpowers:executing-plans. **This repo: do NOT run git operations** (the human commits). Build/run details for the offline replay harness and the `.sun` log format are in `docs/DEBUGGING.md` — read it first; this plan assumes it.

**This plan is ONLY about fixing the MELTY heading LED precession (the navigation filter).** The telemetry format (50 Hz / ESP-NOW v2 / 100 Hz real state), the app's real-vs-replayed vars, dropped-frame display, and error handling are already implemented separately — do not touch them here.

**Goal:** In MELTY mode the heading LED (lit only within ±3° of the zero heading) should stay visually *stationary*. Today it slowly precesses (rotates ~1–2 rev/s while the robot spins ~40 rev/s). Make it lock.

---

## 1. What the robot is (minimum context)

Sunshine is a 1 lb melty-brain combat robot: the whole body spins (up to ~2500+ RPM) and wheel speeds are modulated per-revolution to translate. An ESP32-S3 ("brain") runs a 1 kHz navigation+control loop. The navigation filter (a Kalman filter in `sunshine_core/`, shared C code that also runs on the host for replay) estimates the body heading `θ` and spin rate `ω`:

- **Accelerometer (ADXL375, 11 mm from spin centre, 45° mount):** centripetal accel gives spin rate `ω_accel = sqrt(|a| / r)`. High bandwidth, always available, but **systematically biased** (see §2).
- **Magnetometer (LIS3MDL, 1 kHz):** the Earth field rotates once per body revolution. A synchronous demodulator (`derot_filter.c`) derotates the field by the estimated `θ` and low-pass filters it; `mag_angle = atan2(Q, I)` is an **absolute** heading reference, valid only while spinning (`ω > 4π rad/s ≈ 120 RPM`, `SUNSHINE_MAG_MIN_OMEGA`).
- The Kalman fuses them: accel → `ω`, mag → absolute `θ`. The LED lights when `wrap(kf_theta + theta_offset)` is within ±3°.

Key files (all in `sunshine_core/`): `src/kalman.c` (predict + two update steps), `src/brain.c` (`sunshine_step` orchestration + accel/mag gating), `src/derot_filter.c`, `include/sunshine_core.h` (state struct + tuning `#define`s, most `#ifndef`-guarded so they can be overridden with `-D`). The Kalman tuning constants: `KF_Q_THETA`, `KF_Q_OMEGA`, `KF_R_ACCEL` (accel measurement variance — smaller = stronger/more-trusted accel), `KF_R_MAG`.

The state struct `SunshineState` is **append-only** and shared with the Rust host via `sunshine_app/src-tauri/src/ffi.rs` (a mirrored `#[repr(C, packed)]` struct + a compile-time `size_of` assert) — **if you add/resize state fields you must update ffi.rs and bump `SUNSHINE_SCHEMA_VERSION`**, and the replay harness `tools/replay/replay.c` unpacks state by byte offset so it needs updating too.

---

## 2. Root cause (already investigated & confirmed — do not re-derive from scratch)

Confirmed from the two existing logs in `sunshine_brain/logs/` via the offline replay harness, using the **raw magnetometer as filter-independent ground truth** (the field vector rotates exactly once per revolution about its hard-iron centre; unwrap its angle to get true Ω):

1. **`omega_from_accel` is biased high by ~5–8%** vs the raw-mag true Ω (e.g. 262.6 vs 250.1 rad/s in one window). Both numbers come from *real logged inputs*, so the bias is real, not a replay artifact. Likely causes: the effective IMU radius differs from the assumed 11 mm (best-fit ~12.1–12.8 mm, and it **varies between windows** → not a single fixed constant), plus tangential acceleration from the MELTY drive ripple and accel offset.
2. **`kf_omega` locks onto that biased value during spin-up** (when the omega covariance `P_ωω` is still large, so the accel update dominates), then `P_ωω` collapses and **neither sensor strongly re-corrects ω**. The magnetometer corrects `θ` (angle) well — `derot |I,Q|` ≈ 18 µT vs ~22 µT ideal, a decent lock — but a persistent rate error of ~10 rad/s drifts faster than the mag's angle correction can null it → the LED precesses at roughly the residual bias.

So: **the LED precesses because the heading integrates a biased rate that the magnetometer's angle-only correction can't fully remove.** Precession scales with speed, which is why it shows up in fast MELTY but seemed fine during slow TANK nav-tuning.

### 2a. What was already tried and FAILED — do not repeat naively

A 3-state EKF augmenting the state with an accelerometer **scale factor** `s` (`ω_accel = ω·(1+s)`, mag makes `s` observable) was implemented and **passed an idealized unit test** (constant biased accel + clean mag → ω→true, s→bias) but **railed/diverged on the real logs**: positive feedback (scale↑ → corrected ω↓ → `kf_theta` rotates too slow → demod reference slips off the field → derot collapses → `mag_angle` garbage → scale↑) drove `s` to its clamp. Two compounding reasons it couldn't be tuned: (a) the only logs drop ~5% of 1 kHz inputs, injecting spurious ~80° mag innovations that poison `s`; (b) there is **no gap-free window longer than ~0.9 s** in those logs — far too short for a slow bias estimator to converge. This was reverted; `git`/the current tree is back to the original stable 2-state filter.

**Lesson:** any bias/scale state must be (i) tuned on *clean* data, (ii) stable against the demod-slip feedback loop (bound it tightly, adapt slowly, freeze it whenever the mag is invalid), and (iii) validated across multiple windows and both logs.

---

## 3. Prerequisite: capture a clean log (the real blocker)

The existing logs are unusable for tuning (dropped inputs + no long gap-free window). The telemetry was since moved to **50 Hz / ESP-NOW v2** specifically to stop the input drops, so a fresh capture is now possible.

- [ ] **Flash both boards** (already build clean): brain `production` env and the receiver. From the repo root, invoking pio via the penv exe (NOT bare `pio`):
  - `~/.platformio/penv/Scripts/pio.exe run -d sunshine_brain -e production -t upload`
  - `~/.platformio/penv/Scripts/pio.exe run -d sunshine_receiver -t upload`
- [ ] **Capture a log** with the host app (logging writes to `~/Documents/sunshine_logs/*.sun`): spin up in MELTY and hold steady for **≥30 s continuous**, ideally at 2–3 throttle levels (e.g. ~1500, ~2000, ~2500 RPM), plus the spin-up/down transitions. Label it clearly.
- [ ] **Verify it's clean:** build the replay harness (see DEBUGGING.md → "Offline replay harness") and run `python tools/replay/analyze.py gaps cont.csv`. The dropped-input count must be ≈ 0 (the old logs were ~5%). If drops persist, stop and fix telemetry before tuning — a poisoned log will mis-tune the filter.

Without this clean log, **do not attempt to tune** — you'll repeat the §2a failure.

---

## 4. The fix: let the magnetometer discipline the rate (chosen direction)

The human's decision: **keep the accelerometer powerful (it does a good job — this is sensor fusion), but allow moderate down-weighting so the absolute magnetometer governs the steady-state rate.** Rationale: with a 2-state `[θ, ω]` Kalman, a persistent mag *angle* innovation already feeds back into `ω` through the cross-covariance `P_θω` — that is the integral action of a type-2 PLL, which can drive `ω` to the true rate with zero steady-state error *if the loop has enough authority*. Today it doesn't, because the strong accel update (`KF_R_ACCEL=0.5`) re-pins `ω` to the biased value every tick and `P_ωω`/`P_θω` collapse.

Implement and A/B in replay, in this order (stop at the first that meets §5 criteria):

- [ ] **Approach A — increase the mag's rate authority (no new state; lowest risk).** Tune two existing constants in `sunshine_core/include/sunshine_core.h` (both `#ifndef`-guarded → override with `-D` for A/B without editing source):
  - Raise `KF_Q_OMEGA` (currently `1e-3`) so `P_ωω`/`P_θω` don't collapse and the mag keeps correcting `ω`. Sweep e.g. `1e-3 → 1e-2 → 1e-1`.
  - And/or moderately raise `KF_R_ACCEL` (currently `0.5`) **after lock** so the accel is a high-bandwidth prior, not the steady-state authority. Sweep e.g. `0.5 → 5 → 50`. *Caution: too high broke spin-up in a quick test (ω never reached the mag-valid threshold), so prefer gating — see Approach B's gate — or keep R_ACCEL low until `mag_valid`.*
  - Goal: find a `(KF_Q_OMEGA, KF_R_ACCEL)` pair where replayed precession → ~0 with stable lock AND spin-up still reaches `mag_valid`. Build per-variant: `cmake -B build-q -S . -DCMAKE_C_FLAGS="-DKF_Q_OMEGA=1e-2f -DKF_R_ACCEL=5.0f"` (or pass via the harness build), then run `analyze.py precession`.

- [ ] **Approach B — gated, bounded accel-bias state (only if A is insufficient across speeds).** Because the bias varies with speed, a learned correction may beat fixed retuning. Add an **additive ω-bias** state `b` (simpler/more stable than the scale factor that failed): `ω_accel` measured as `ω + b`, linear (no EKF). Make it safe:
  - Append `kf_omega_bias` (+ its covariance terms) to `SunshineState` (append-only; bump schema; update `ffi.rs` + `replay.c`).
  - **Freeze `b` whenever `!mag_valid`** (spin-up/spin-down) — zero its covariance coupling that tick (the §2a divergence happened when it updated without an absolute reference).
  - **Bound** `b` to a physically plausible range and use a **small** `Q` so it adapts over seconds, not ticks.
  - Keep `KF_R_ACCEL` reasonably strong (accel stays primary); `b` only soaks up the slow systematic component.

The detailed math for an augmented linear-bias Kalman: state `x=[θ,ω,b]`, predict `F=[[1,dt,0],[0,1,0],[0,0,1]]` with `Q=diag(Qθ,Qω,Qb)`; accel update `z=ω_accel`, `H=[0,1,1]`, `R=KF_R_ACCEL`; mag update `z=mag_angle`, `H=[1,0,0]`, `R=KF_R_MAG`. The mag's angle innovation makes `b` observable through `P_θb` (built via the predict `θ`-`ω` coupling and the accel update's `ω`-`b` coupling). This is standard gyro-bias estimation; the failure mode to guard is unobservability when the mag is off.

---

## 5. Validation (replay harness — see DEBUGGING.md for build/run)

For each candidate, rebuild the harness against the modified core and replay the **clean** log:

- [ ] **Precession → ~0:** `python tools/replay/analyze.py precession cont.csv` reports `LED PRECESSION` (= replayed heading-reference rate − raw-mag true Ω). **Pass: |precession| < 0.5 rad/s** across **≥3 windows and at the different throttle levels** (today it's ~+12 rad/s). The bias varies with speed, so a single window passing is NOT enough.
- [ ] **Lock quality:** `derot |I,Q|` improves toward the Earth-field magnitude (**> 20 µT**, up from ~18).
- [ ] **Spin-up still works:** over the full log there is no NaN/divergence, `ω` reaches `mag_valid`, and (Approach B) the bias state stays bounded and converges — not railed. Quick check: dump `kf_omega`/bias min/max from the CSV.
- [ ] **C unit tests pass:** `ctest --test-dir tools/replay/build` (the harness CMake also builds `sunshine_core`'s tests). Add a test asserting the fix's intended steady-state behaviour (e.g. biased-accel + clean-mag → ω converges to true, lock holds).
- [ ] **No regression to TANK / DISABLED:** spot-check those modes in the log behave sanely.

The raw-mag ground truth in `analyze.py precession` is filter-independent, so it stays valid no matter how you change the filter.

---

## 6. Workflow & guardrails

- This touches the **safety-critical 1 kHz control/filter loop**. Per the repo workflow: tag the current state, branch, keep changes minimal, and do not merge until §5 all pass. (The human runs git.)
- Prefer the smallest change that meets the criteria (Approach A before B).
- If 3+ tuning attempts each reveal a new failure, **stop and question the model** (systematic-debugging §Phase 4.5) rather than piling on — that's exactly how the §2a scale-EKF went wrong.
- Re-run the C unit tests after every core change; they build cross-platform via `tools/replay/CMakeLists.txt`.

## 7. Done when

The MELTY heading LED is visually stationary on hardware, and in replay the precession is < 0.5 rad/s across speeds with a stable lock (derot > 20 µT) and no spin-up regression, with the change validated on a freshly captured clean 50 Hz log.
