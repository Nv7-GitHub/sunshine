# MELTY Heading Rate-Bias Filter Fix — Validation & Tuning Plan

> **For agentic workers:** REQUIRED SUB-SKILL: `superpowers:systematic-debugging` and `superpowers:executing-plans`. **Do NOT run git operations** (the human commits). The `.sun` log format and how to build/run the offline replay harness (`tools/replay/`) + `analyze.py` are in `docs/DEBUGGING.md` — read it first; this plan assumes it.

**The fix is already implemented and validated against the existing (imperfect) logs.** This plan is to **confirm and fine-tune it on a freshly captured clean log**, and to escalate to a second stage only if needed. It concerns ONLY the navigation filter / MELTY heading-LED precession — not telemetry format, app, etc.

> **What's implemented now (read this — supersedes the "approaches" framing below):**
> 1. **Open-loop magnetometer heading** (`sunshine_core/src/mag_heading.c`): each mag axis is high-passed (2nd-order Butterworth, `MAG_HP_*`, fc=0.5 Hz) to strip the body-fixed DC (hard-iron + average ESC current), then `mag_angle = atan2(-my_hp, mx_hp)`. This is **open-loop** — independent of the estimate — so it cannot drift (replaced the old closed-loop synchronous demodulator). Full math in FILTER_MATH.md §Step 2.
> 2. **Trust the (now clean, absolute) mag**: `KF_R_MAG` lowered 0.1 → **0.01**.
> 3. **Accel rate down-weighting once locked**: `KF_R_ACCEL_LOCKED = 80` used when `mag_valid`, `KF_R_ACCEL = 0.5` during spin-up (`brain.c`).
>
> **Replay result on the old logs:** the high-passed field is a steady ~22 µT every window (clean), and precession on the longest window is ~+0.3 rev/s (was ~+2). Shorter windows scatter ±0.5 rev/s — but those are 0.3–0.9 s transient-contaminated chunks from gappy logs, so the **clean-log confirmation below is what matters**.
>
> **Remaining task:** capture a clean log (§3), confirm precession < 0.5 rad/s across all speeds, and fine-tune `KF_R_MAG` / `KF_R_ACCEL_LOCKED` (§ tuning). If a speed-dependent residual remains, escalate to the additive ω-bias state (§4). Schema is now v3, `SunshineState` = 44 bytes.

**Goal:** In MELTY the heading LED (lit within ±3° of zero heading) should look *stationary*. It used to precess ~2 rev/s while the robot spins ~40 rev/s. The implemented fix cut that to ~0.5 rev/s in replay; confirm it's visually stationary on hardware and fine-tune.

---

## 1. Minimum robot context

1 lb melty-brain combat robot: the whole body spins (~2000–2500 RPM) and wheel speeds modulate per-revolution to translate. An ESP32-S3 runs a 1 kHz nav+control loop. The navigation Kalman filter (`sunshine_core/`, shared C also run on the host for replay) estimates heading `θ` and spin rate `ω`:
- **Accelerometer (ADXL375, ~11 mm from spin centre):** centripetal accel → `ω_accel = sqrt(|a|/r)`. High-bandwidth, always available, but **biased ~5–8% high** (effective radius ≠ assumed 11 mm + tangential accel from drive ripple; the bias also *varies with speed*).
- **Magnetometer (LIS3MDL):** Earth field rotates once per body revolution; a synchronous demodulator (`derot_filter.c`) derotates by estimated `θ` → `mag_angle`, an **absolute** heading reference, valid only while spinning (`ω > SUNSHINE_MAG_MIN_OMEGA = 4π rad/s ≈ 120 RPM`).
- The Kalman fuses accel→`ω`, mag→absolute `θ`. LED lights when `wrap(kf_theta + theta_offset)` is within ±3°.

Files: `sunshine_core/src/{kalman,brain,derot_filter}.c`, `include/sunshine_core.h` (tuning `#define`s, `#ifndef`-guarded → overridable with `-D`). Tuning constants: `KF_Q_THETA`, `KF_Q_OMEGA`, `KF_R_ACCEL`, **`KF_R_ACCEL_LOCKED`** (new), `KF_R_MAG`.

---

## 2. Root cause (confirmed) and the implemented fix

**Root cause:** `ω_accel` is biased high. `kf_omega` locks onto that biased value during spin-up (when `P_ωω` is large and the accel update dominates); then `P_ωω` collapses and neither sensor strongly re-corrects `ω`. The magnetometer corrects the *angle* well (`derot |I,Q|` ≈ 20 µT of ~22) but a persistent ~10 rad/s rate error drifts faster than the angle correction can null → the LED precesses at roughly the residual bias. Confirmed against the raw magnetometer (filter-independent ground truth: the field vector rotates once per rev about its hard-iron centre).

**The fix (implemented):** keep the accelerometer as the high-bandwidth primary, but **down-weight it once the magnetometer is locked** so the absolute mag governs the steady-state rate (a type-2 PLL nulls the frequency error). Concretely, in `sunshine_core/`:
- `kalman_update_omega(s, omega_meas, r_accel)` now takes the measurement variance as an argument.
- `brain.c` computes `mag_valid` *before* the accel update and passes `KF_R_ACCEL_LOCKED` (weak) when locked, `KF_R_ACCEL` (strong) during spin-up so `ω` can still climb to the mag-valid threshold.
- Constants: `KF_R_ACCEL = 0.5` (spin-up, unchanged), **`KF_R_ACCEL_LOCKED = 80.0`** (locked), `KF_Q_OMEGA` raised `1e-3 → 1e-2` (so `ω` stays correctable by the mag — alone it does nothing; it only helps once the accel is down-weighted).
- Unit tests in `test_kalman.c` lock both behaviours (locked → mag pulls `ω` to truth despite biased accel; trusted → `ω` follows the biased accel for spin-up).

### Validation on the EXISTING logs (already done)
Using `tools/replay/replay.exe <log> | analyze.py precession` (the harness dead-reckons across logged input gaps so the filter is at steady-state by each gap-free window; precession = replayed heading rate − raw-mag true Ω):

| window | baseline precession | with fix |
|---|---|---|
| 0.91 s @ 2388 RPM (longest, most reliable) | +12.4 rad/s (+1.97 rev/s) | **+2.8 rad/s (+0.44 rev/s)** |
| other steady windows | +5…+6 rad/s | −0.5…−1 rev/s |
| 0.34 s @ 2259 RPM (transient) | +27 rad/s | +20 rad/s ⚠ |

Spin-up still reaches lock (`mag_valid` 99%+, `ω` 11→271 rad/s, no NaN/divergence). C unit tests 5/5 pass. So the fix is real and substantial (~4× reduction), and stable (no demod-collapse feedback like the abandoned approach in §5).

---

## 3. Confirm & fine-tune on a clean log (the main task)

The existing logs drop ~5% of inputs and have **no gap-free window > ~0.9 s**, so the per-window scatter above is partly measurement noise and partly the *speed-varying* bias. A clean 50 Hz log (the telemetry was moved to ESP-NOW v2 to stop the drops) will let you confirm and refine.

- [ ] **Flash both boards** (build clean; invoke pio via the penv exe, NOT bare `pio`):
  - `~/.platformio/penv/Scripts/pio.exe run -d sunshine_brain -e production -t upload`
  - `~/.platformio/penv/Scripts/pio.exe run -d sunshine_receiver -t upload`
- [ ] **Capture** a MELTY log (host app logs to `~/Documents/sunshine_logs/*.sun`): spin-up + **≥30 s steady at 2–3 throttle levels** (~1500/2000/2500 RPM) + a spin-down. Label it.
- [ ] **Confirm it's clean:** build the harness (DEBUGGING.md → "Offline replay harness"), then `python tools/replay/analyze.py gaps <log>.csv` → dropped inputs must be ≈ 0.
- [ ] **Measure precession:** `python tools/replay/analyze.py precession <log>.csv`. **Pass: |precession| < 0.5 rad/s across ALL steady windows and all speeds**, derot lock > 20 µT, no flagged outliers (other than genuine throttle transitions).
- [ ] **On the robot:** spin up in MELTY and look at the LED — it should be a near-stationary dot. (Heading flash is full brightness; idle is the dim breathe.)

### Fine-tuning `KF_R_ACCEL_LOCKED` (the main knob)
Override without editing source via the harness build flags (constant is `#ifndef`-guarded), e.g. configure a build with `-DCMAKE_C_FLAGS="-DKF_R_ACCEL_LOCKED=120.0f -DKF_Q_OMEGA=1e-2f"` and re-run `analyze.py precession`. Behaviour observed on existing data:
- **Higher `KF_R_ACCEL_LOCKED`** (weaker accel) → less precession, until it **overshoots negative** (~200 over-corrected to −1.8 rev/s and degraded the lock). The sweet spot was ~75–100; the longest window nulled at ~100.
- **Lower** → precession returns toward the biased-accel value.
- `KF_Q_OMEGA` alone does nothing — only matters paired with a down-weighted accel.
- Pick the value that minimizes |precession| across **all** speeds in the clean log (the optimum may differ from 80 once the data is clean). Keep `KF_R_ACCEL` (spin-up) at 0.5 — verify spin-up still reaches `mag_valid` after any change.
- Re-run `ctest --test-dir tools/replay/build` after every change.

---

## 4. Escalation (only if §3 can't hit < 0.5 rad/s across speeds)

If a single `KF_R_ACCEL_LOCKED` leaves a speed-dependent residual (because the accel bias varies with speed), add an **online ω-bias state** to absorb the slow systematic component:
- Append `kf_omega_bias` (+ its covariance terms) to `SunshineState` — **append-only**, so bump `SUNSHINE_SCHEMA_VERSION`, update the Rust mirror `sunshine_app/src-tauri/src/ffi.rs` (struct + `size_of` assert) and the harness `tools/replay/replay.c` (unpacks state by byte offset).
- Use an **additive** bias (linear: accel measures `ω + b`, `H=[0,1,1]`), NOT a scale factor (see §5).
- **Freeze `b` whenever `!mag_valid`** (zero its covariance coupling that tick) and **bound** it with a **small** `Q` so it adapts over seconds. The mag's angle innovation makes `b` observable via the cross-covariance.
- Validate exactly as §3, additionally confirming `b` converges and stays bounded (dump it from the replay CSV) and that it's stable across spin-up/down.

## 5. What NOT to do (already tried and failed)
A 3-state EKF with an accelerometer **scale factor** `s` (`ω_accel = ω·(1+s)`) passed an idealized unit test but **railed/diverged on real data**: positive feedback (scale↑ → corrected ω↓ → demod reference slips off the field → derot collapses → `mag_angle` garbage → scale↑) drove `s` to its clamp. The gaps + lack of a long convergence window made it untunable. The implemented §2 fix (down-weighting, no learned multiplicative state in the demod loop) avoids that feedback entirely. If you add a bias state (§4), keep it additive, gated to mag-valid, and slow — and watch for the same demod-slip instability.

## 6. Done when
The MELTY heading LED is visually stationary on hardware, and in replay on a clean log |precession| < 0.5 rad/s across all speeds with derot > 20 µT, no spin-up regression, and `ctest` green.
