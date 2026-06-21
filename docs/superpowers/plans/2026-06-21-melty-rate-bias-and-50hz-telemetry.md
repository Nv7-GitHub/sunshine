# Melty Rate-Bias Fusion + 50 Hz ESP-NOW v2 Telemetry — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. **This repo: do NOT run git. Replace "commit" with the stated "checkpoint" (rebuild + validate) at each task boundary.**

**Goal:** (A) Stop the MELTY heading LED from precessing by fusing out the accelerometer's angular-rate scale error using the magnetometer as the absolute reference, while keeping the accelerometer as the powerful primary rate source. (B) Move telemetry from 6-inputs/≈167 Hz frames to the README-intended 20-inputs/50 Hz frames using ESP-NOW v2 (1490-byte payloads), which also relieves the ring-buffer backpressure that currently drops ~5% of 1 kHz inputs.

**Architecture:**
- **Part A** augments the Kalman state from `[θ, ω]` to `[θ, ω, s]`, where `s` is the accelerometer angular-rate **scale error** (`ω_accel = ω·(1+s)`). The accel update stays strong (`KF_R_ACCEL` unchanged) so the accelerometer remains the high-bandwidth primary, but the magnetometer's absolute-angle update now observes and removes the slow scale error through the Kalman cross-covariance — textbook gyro-scale estimation. New state fields are **appended** (append-only rule) so old logs replay unchanged.
- **Part B** raises `INPUTS_PER_FRAME` 6→20 (frame 237→643 B), upgrades the receiver from espressif32@6.0.0 (IDF 4.4) to the same pioarduino IDF-5.x platform as the brain (ESP-NOW v2 needs IDF ≥5.4 on both ends), and widens the host/receiver buffers. The `.sun` header already carries `num_inputs`, so logging adapts automatically.

**Tech Stack:** C (sunshine_core, ESP32 firmware via PlatformIO/Arduino-ESP32), Rust (Tauri host), the cross-platform `tools/replay/` CMake harness for offline replay validation, Python+numpy for analysis.

---

> **STATUS (2026-06-21):**
> - **Part B (50 Hz / ESP-NOW v2): DONE & compiles** on both ESP32-S3 firmwares (brain `production` + receiver, the latter upgraded to the IDF-5 pioarduino platform). Host (`protocol.rs`) updated with a derive-from-structs frame size + a parse test; **host `cargo build`/`cargo test` and on-air bench verification are still pending** (no cargo/hardware in the dev environment).
> - **Part A (rate-bias fix): REVERTED to the original stable 2-state filter.** The scale-factor EKF (below) is sound in the idealised unit test but **railed/diverged on the only available logs** (positive feedback scale↔ω↔demod, plus those logs drop ~5% of inputs and have no gap-free window > ~0.9 s to converge in). Decision: ship Part B first to get clean 50 Hz data, then re-tune the rate-bias fix on fresh logs **with moderate accelerometer down-weighting** (KF_R_ACCEL) + bias modelling, hardware-in-the-loop. The Part A task steps below are retained as the design record for that future work.
> - Tooling: `tools/replay/` is now built via **CMake** (cross-platform), not PowerShell. It dead-reckons across logged telemetry gaps.

**Validation philosophy:** Part A is validated by **replay** (the `tools/replay/` harness re-runs real logged 1 kHz inputs through the actual `sunshine_step`) plus C unit tests. Ground truth for spin rate comes from the **raw magnetometer** (field rotation), independent of the filter. Part B firmware is validated by **compilation** (no hardware in the loop here) plus host-side Rust parse tests; the on-air behaviour must be bench-verified later.

---

## Pre-flight (once, before Part A)

- [ ] **P0.1: Tag the safe state** — *(no git per repo rule)*. Instead, snapshot the current replay baseline so we can prove improvement:

```bash
pwsh tools/replay/build.ps1
LOG="sunshine_brain/logs/2026-06-17_02-26-43_Spiritridge3.sun"
tools/replay/replay.exe "$LOG" > /tmp/baseline_cont.csv
.venv/Scripts/python.exe tools/replay/analyze.py precession /tmp/baseline_cont.csv | tee /tmp/baseline_precession.txt
```
Expected (record these as the "before"): precession ≈ **+12 rad/s** (≈ +1.9 rev/s) on the long window; derot |I,Q| ≈ **15–18 µT**.

- [ ] **P0.2: Add a test build script** so the C unit tests run under MSVC (no gcc here).

**Files:** Create `tools/replay/build_tests.ps1`

```powershell
# Build & run a sunshine_core C unit test under MSVC.
# Usage:  pwsh tools/replay/build_tests.ps1 kalman   (or: control / derot_filter / brain / utils)
param([Parameter(Mandatory=$true)][string]$which)
$ErrorActionPreference = "Stop"
$root   = Resolve-Path "$PSScriptRoot\..\.."
$core   = "$root\sunshine_core"
$vcvars = "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat"
$compat = "$PSScriptRoot\msvc_compat.h"
$out    = "$env:TEMP\test_$which.exe"
$srcs = @(
  "$core\test\test_$which.c",
  "$core\src\utils.c","$core\src\kalman.c","$core\src\derot_filter.c",
  "$core\src\control.c","$core\src\brain.c"
) -join '" "'
$cmd = "cl /nologo /O2 /I `"$core\include`" /I `"$core\test`" /FI`"$compat`" `"$srcs`" /Fe`"$out`" /Fo`"$env:TEMP\\`""
cmd /c "`"$vcvars`" >nul 2>&1 && $cmd"
if (-not (Test-Path $out)) { throw "build failed" }
& $out
```

- [ ] **P0.3: Confirm the harness builds the current tests** (sanity before changing anything):

Run: `pwsh tools/replay/build_tests.ps1 kalman`
Expected: prints `ok` lines and `N passed, 0 failed`.

> If `test_runner.h`'s `ASSERT_EQ` fails to compile under MSVC (`__typeof__`), it is only used by other test files; `test_kalman.c`/`test_control.c` use `ASSERT`/`ASSERT_NEAR` only. If a needed test uses it, replace `__typeof__(a)` with explicit types in that test.

---

# PART A — Magnetometer-fused accelerometer scale (fix LED precession)

### Task A1: Augment `SunshineState` with the scale state (append-only) + schema bump

**Files:**
- Modify: `sunshine_core/include/sunshine_core.h` (struct + schema version + new tuning constants)
- Modify: `sunshine_app/src-tauri/src/ffi.rs:26-35,60-64` (Rust mirror + size assert)

- [ ] **Step 1: Bump schema version**

In `sunshine_core/include/sunshine_core.h`, change:
```c
#define SUNSHINE_SCHEMA_VERSION  2U
```
to:
```c
#define SUNSHINE_SCHEMA_VERSION  3U
```

- [ ] **Step 2: Add scale tuning constants** (next to the other `#ifndef`-guarded KF constants):

```c
#ifndef KF_Q_SCALE
#define KF_Q_SCALE   1e-9f          /* scale-error random-walk: drifts very slowly */
#endif
#ifndef KF_P_SCALE_INIT
#define KF_P_SCALE_INIT  0.04f      /* initial scale variance: std 0.2 = ±20% prior */
#endif
#ifndef KF_SCALE_CLAMP
#define KF_SCALE_CLAMP   0.30f      /* |s| hard limit, guards against runaway */
#endif
```

- [ ] **Step 3: Append the new state fields** at the END of `SunshineState` (after `derot_lp_Q[4]`):

```c
typedef struct __attribute__((packed)) {
    float kf_theta;         /* Kalman angle estimate (rad, unwrapped)     */
    float kf_omega;         /* Kalman angular velocity estimate (rad/s)   */
    float kf_P[4];          /* 2×2 covariance of [θ,ω], row-major         */
    float theta_offset;     /* driver heading offset (rad)                */
    float derot_lp_I[4];    /* LP state for derotated I component         */
    float derot_lp_Q[4];    /* LP state for derotated Q component         */
    /* --- schema v3 (APPENDED) : accelerometer rate scale-error fusion --- */
    float kf_scale;         /* s: accel reads ω_accel = ω·(1+s)           */
    float kf_P_scale[3];    /* extra covariance [P_θs, P_ωs, P_ss]        */
} SunshineState;
/* static_assert(sizeof(SunshineState) == 76, ""); */
```

> Backward compat: old (60-byte) logs zero-pad `kf_scale=0`, `kf_P_scale={0,0,0}`. With `P_ss=0` the scale never updates, so the v3 filter **reduces exactly to the v2 two-state filter** on old logs — old logs replay identically.

- [ ] **Step 4: Update the Rust FFI mirror** `sunshine_app/src-tauri/src/ffi.rs`:

Replace the `SunshineState` struct (lines 26-35) with:
```rust
#[repr(C, packed)]
#[derive(Clone, Copy, Default, Debug)]
pub struct SunshineState {
    pub kf_theta:      f32,
    pub kf_omega:      f32,
    pub kf_p:          [f32; 4],
    pub theta_offset:  f32,
    pub derot_lp_i:    [f32; 4],
    pub derot_lp_q:    [f32; 4],
    pub kf_scale:      f32,
    pub kf_p_scale:    [f32; 3],
}
```
And update the size assert (line 62):
```rust
    assert!(size_of::<SunshineState>() == 76, "SunshineState size mismatch");
```

- [ ] **Step 5: Checkpoint** — `pwsh tools/replay/build.ps1` must still succeed (replay.c uses offset-based unpack; A2 updates it). Build is expected to pass; the new fields aren't read yet.

---

### Task A2: Teach the replay harness about v3 state (so it can validate the fix on old logs)

**Files:** Modify `tools/replay/replay.c`

The harness must (a) parse the new fields when present, and (b) when replaying an **old** (v2, 60-byte) log, keep `kf_scale=0` but seed `P_ss` from `KF_P_SCALE_INIT` so scale estimation is *exercised* on the historical inputs (proving the fix). Old logs have `sz_st=60`; v3 logs have `sz_st=76`.

- [ ] **Step 1: Update `unpack_state` to be size-aware:**

```c
static void unpack_state(const uint8_t *b, int sz_st, SunshineState *s){
    s->kf_theta     = rd_f32(b+0);
    s->kf_omega     = rd_f32(b+4);
    for (int i=0;i<4;i++) s->kf_P[i]      = rd_f32(b+8 +4*i);
    s->theta_offset = rd_f32(b+24);
    for (int i=0;i<4;i++) s->derot_lp_I[i]= rd_f32(b+28+4*i);
    for (int i=0;i<4;i++) s->derot_lp_Q[i]= rd_f32(b+44+4*i);
    if (sz_st >= 76) {                 /* v3 log: scale state present */
        s->kf_scale = rd_f32(b+60);
        for (int i=0;i<3;i++) s->kf_P_scale[i] = rd_f32(b+64+4*i);
    } else {                           /* v2 log: enable scale estimation */
        s->kf_scale = 0.0f;
        s->kf_P_scale[0] = 0.0f;       /* P_θs */
        s->kf_P_scale[1] = 0.0f;       /* P_ωs */
        s->kf_P_scale[2] = KF_P_SCALE_INIT; /* P_ss > 0 so s is observable */
    }
}
```

- [ ] **Step 2: Update the call site** (the `unpack_state(fb+5, &frame_state);` line) to pass `sz_st`:
```c
        SunshineState frame_state; unpack_state(fb+5, sz_st, &frame_state);
```

- [ ] **Step 3: Add `kf_scale` to the CSV.** Header line — append `,kf_scale` before `stored_est_theta`:
```c
           "mag_x,mag_y,accel_x,accel_y,theta_offset,kf_scale,"
           "stored_est_theta,stored_mag_angle,stored_led_on\n");
```
Row `printf` — add `st.kf_scale`. Change the main row format string tail from `...%.6f"` (theta_offset) to add one float:
```c
                   "%d,%d,%d,%d,%.6f,%.6f",
                   ... in.accel_x, in.accel_y, st.theta_offset, st.kf_scale);
```

- [ ] **Step 4: Checkpoint** — `pwsh tools/replay/build.ps1`; expect OK. (Filter math unchanged yet, so precession unchanged — verify the harness still runs and `kf_scale` column is present & 0.)
Run: `tools/replay/replay.exe "sunshine_brain/logs/2026-06-17_02-26-43_Spiritridge3.sun" --from-us 89628463 --to-us 89629463 | head -3`
Expected: rows print, `kf_scale` column = 0.000000.

---

### Task A3: Rewrite the Kalman as a 3-state `[θ, ω, s]` filter (TDD)

**Files:**
- Modify: `sunshine_core/src/kalman.c`
- Test: `sunshine_core/test/test_kalman.c`

**Math (EKF; only the accel update is nonlinear):**
- State `x=[θ,ω,s]`. Covariance `P` 3×3 symmetric, stored as `kf_P=[Pθθ,Pθω,Pωθ,Pωω]` + `kf_P_scale=[Pθs,Pωs,Pss]`.
- Predict `dt`: `θ+=ω·dt` (wrap); `ω,s` random-walk. `F=[[1,dt,0],[0,1,0],[0,0,1]]`, `Q=diag(KF_Q_THETA,KF_Q_OMEGA,KF_Q_SCALE)`.
- Accel update (measures `z=ω_accel`, `h=ω·(1+s)`): `H=[0,(1+s),ω]`, `R=KF_R_ACCEL`.
- Mag update (measures `z=θ`, `h=θ`): `H=[1,0,0]`, `R=KF_R_MAG`, innovation wrapped. This is what makes `s` observable.
- After every update, clamp `s∈[-KF_SCALE_CLAMP, KF_SCALE_CLAMP]`.

- [ ] **Step 1: Write failing tests.** Append to `sunshine_core/test/test_kalman.c` before `TEST_RESULTS();`:

```c
    /* ── schema v3: scale-error estimation ── */
    /* init: scale 0, P_ss > 0 (observable), cross terms 0 */
    sunshine_state_init(&s);
    ASSERT_NEAR(s.kf_scale, 0.0f, 1e-6f, "scale init = 0");
    ASSERT(s.kf_P_scale[2] > 0.0f, "P_ss init > 0");
    ASSERT_NEAR(s.kf_P_scale[0], 0.0f, 1e-6f, "P_theta_s init 0");

    /* predict leaves scale unchanged, grows P_ss slightly */
    s = make_state(0.0f, 100.0f);
    float pss0 = s.kf_P_scale[2];
    kalman_predict(&s, 0.001f);
    ASSERT_NEAR(s.kf_scale, 0.0f, 1e-6f, "predict: scale unchanged");
    ASSERT(s.kf_P_scale[2] >= pss0, "predict: P_ss non-decreasing");

    /* KEY behaviour: a constant biased accel rate + perfect absolute angle
       must drive scale to absorb the bias and omega to the TRUE rate.
       True omega = 100; accel over-reads by 8% => measures 108. Mag says the
       true angle advances at 100 rad/s. Filter should learn s≈+0.08, omega≈100. */
    sunshine_state_init(&s);
    s.kf_omega = 100.0f;
    float true_omega = 100.0f, accel_reads = 108.0f, theta_true = 0.0f;
    for (int i = 0; i < 60000; i++) {           /* 60 s @ 1 kHz */
        kalman_predict(&s, 0.001f);
        kalman_update_omega(&s, accel_reads);    /* biased accel */
        theta_true += true_omega * 0.001f;       /* ground-truth angle */
        kalman_update_theta(&s, theta_true);     /* absolute reference */
    }
    ASSERT_NEAR(s.kf_omega, 100.0f, 1.5f, "fused omega converges to TRUE rate");
    ASSERT_NEAR(s.kf_scale, 0.08f, 0.02f, "scale learns the +8%% accel bias");

    /* scale is clamped */
    sunshine_state_init(&s);
    s.kf_scale = 0.0f;
    for (int i = 0; i < 200000; i++) {           /* absurd 100% bias forever */
        kalman_predict(&s, 0.001f);
        kalman_update_omega(&s, 200.0f);
        float tt = 100.0f * 0.001f * i;
        kalman_update_theta(&s, remainderf(tt, 2.0f*3.14159265f));
    }
    ASSERT(s.kf_scale <= 0.30f + 1e-4f && s.kf_scale >= -0.30f - 1e-4f, "scale stays clamped");
```

- [ ] **Step 2: Run tests, verify they fail.**
Run: `pwsh tools/replay/build_tests.ps1 kalman`
Expected: FAIL on the new scale assertions (and possibly a compile error until A3-step3 lands). If it compiles, the "fused omega converges" / "scale learns" lines fail.

- [ ] **Step 3: Replace `sunshine_core/src/kalman.c` with the 3-state implementation:**

```c
/* src/kalman.c — 3-state EKF [theta, omega, scale]. scale s models the
 * accelerometer angular-rate scale error: omega_accel = omega*(1+s). The
 * magnetometer (absolute angle) makes s observable, removing the systematic
 * accel bias that otherwise precesses the heading. */
#include "sunshine_core.h"
#include <math.h>
#include <string.h>

#define M_PI_F 3.14159265f

static float wrap_to_pi(float a) { return remainderf(a, 2.0f * M_PI_F); }
static float clampf(float v, float lo, float hi){ return v<lo?lo:(v>hi?hi:v); }

void sunshine_state_init(SunshineState *s) {
    memset(s, 0, sizeof(*s));
    s->kf_P[0] = 100.0f;   /* angle uncertainty           */
    s->kf_P[3] = 1.0f;     /* omega ~ 1 rad/s std         */
    s->kf_scale = 0.0f;
    s->kf_P_scale[0] = 0.0f;              /* P_theta_s */
    s->kf_P_scale[1] = 0.0f;              /* P_omega_s */
    s->kf_P_scale[2] = KF_P_SCALE_INIT;   /* P_ss      */
}

/* Predict: F = [[1,dt,0],[0,1,0],[0,0,1]]; Q = diag(Qth,Qom,Qsc). */
void kalman_predict(SunshineState *s, float dt) {
    s->kf_theta = wrap_to_pi(s->kf_theta + s->kf_omega * dt);
    float Ptt=s->kf_P[0], Pto=s->kf_P[1], Poo=s->kf_P[3];
    float Pts=s->kf_P_scale[0], Pos=s->kf_P_scale[1], Pss=s->kf_P_scale[2];
    /* P' = F P F^T + Q. Row/col theta gain dt*omega-coupling. */
    s->kf_P[0] = Ptt + dt*(Pto + Pto) + dt*dt*Poo + KF_Q_THETA;
    s->kf_P[1] = Pto + dt*Poo;
    s->kf_P[2] = s->kf_P[1];                 /* keep symmetric */
    s->kf_P[3] = Poo + KF_Q_OMEGA;
    s->kf_P_scale[0] = Pts + dt*Pos;         /* P_theta_s */
    s->kf_P_scale[1] = Pos;                  /* P_omega_s */
    s->kf_P_scale[2] = Pss + KF_Q_SCALE;     /* P_ss      */
}

/* Accel update: z = omega_accel, h = omega*(1+s), H = [0, 1+s, omega]. */
void kalman_update_omega(SunshineState *s, float omega_meas) {
    float g  = 1.0f + s->kf_scale;
    float w  = s->kf_omega;
    float Ptt=s->kf_P[0], Pto=s->kf_P[1], Poo=s->kf_P[3];
    float Pts=s->kf_P_scale[0], Pos=s->kf_P_scale[1], Pss=s->kf_P_scale[2];
    /* PHt = P * H^T  (H = [0, g, w]) */
    float PHt0 = g*Pto + w*Pts;
    float PHt1 = g*Poo + w*Pos;
    float PHt2 = g*Pos + w*Pss;
    float S    = g*PHt1 + w*PHt2 + KF_R_ACCEL;   /* H*P*H^T + R */
    float inn  = omega_meas - g*w;
    float K0 = PHt0 / S, K1 = PHt1 / S, K2 = PHt2 / S;
    s->kf_theta += K0 * inn;
    s->kf_omega += K1 * inn;
    s->kf_scale  = clampf(s->kf_scale + K2 * inn, -KF_SCALE_CLAMP, KF_SCALE_CLAMP);
    /* P -= K * (H*P) ; H*P = [PHt0, PHt1, PHt2] (row) */
    s->kf_P[0] -= K0*PHt0;  s->kf_P[1] -= K0*PHt1;  s->kf_P_scale[0] -= K0*PHt2;
                            s->kf_P[3] -= K1*PHt1;  s->kf_P_scale[1] -= K1*PHt2;
                                                    s->kf_P_scale[2] -= K2*PHt2;
    s->kf_P[2] = s->kf_P[1];
}

/* Mag update: z = theta_meas, h = theta, H = [1,0,0]. Makes s observable. */
void kalman_update_theta(SunshineState *s, float theta_meas) {
    float Ptt=s->kf_P[0], Pto=s->kf_P[1], Pts=s->kf_P_scale[0];
    float S   = Ptt + KF_R_MAG;
    float inn = wrap_to_pi(theta_meas - s->kf_theta);
    float K0 = Ptt / S, K1 = Pto / S, K2 = Pts / S;
    s->kf_theta += K0 * inn;
    s->kf_omega += K1 * inn;
    s->kf_scale  = clampf(s->kf_scale + K2 * inn, -KF_SCALE_CLAMP, KF_SCALE_CLAMP);
    /* P -= K * (H*P) ; H*P = first row [Ptt, Pto, Pts] */
    s->kf_P[0] -= K0*Ptt;  s->kf_P[1] -= K0*Pto;  s->kf_P_scale[0] -= K0*Pts;
                           s->kf_P[3] -= K1*Pto;  s->kf_P_scale[1] -= K1*Pts;
                                                  s->kf_P_scale[2] -= K2*Pts;
    s->kf_P[2] = s->kf_P[1];
}
```

- [ ] **Step 4: Run tests, verify pass.**
Run: `pwsh tools/replay/build_tests.ps1 kalman`
Expected: all `ok`, `N passed, 0 failed` — including "fused omega converges to TRUE rate" and "scale learns the +8% accel bias".

- [ ] **Step 5: Run the other core tests** (no regressions in derot/control/brain that share state):
Run: `pwsh tools/replay/build_tests.ps1 control` then `... brain` then `... derot_filter`
Expected: all pass.

---

### Task A4: Gate scale estimation to when the magnetometer is valid

**Why:** During spin-up (`omega < SUNSHINE_MAG_MIN_OMEGA`) the mag is invalid, so `s` is unobservable. Without the mag, the accel update alone could drift `s` and `omega` together (degenerate). Freeze `s` until the mag is locked. The accel stays fully powerful for `theta`/`omega`.

**Files:** Modify `sunshine_core/src/brain.c:42-52`

- [ ] **Step 1: Pass mag validity into the accel update.** The simplest robust gate: only let the accel update move `s` once mag is valid. Implement by computing `mag_valid` *before* the accel update and using a guarded accel-update variant.

Replace the update ordering in `sunshine_step` (`brain.c`) so mag-validity is known first:
```c
    /* -- Kalman predict --------------------------------------------------- */
    kalman_predict(state, DT);

    /* mag becomes the absolute reference only above the min spin rate */
    vars->mag_valid = (state->kf_omega > SUNSHINE_MAG_MIN_OMEGA);

    /* -- Kalman update - accelerometer ------------------------------------ */
    /* Freeze scale-error learning until the mag can observe it (spin-up):
       temporarily zero the scale covariance row so K2 = 0 this tick. */
    if (!vars->accel_saturated && vars->omega_from_accel > 0.0f) {
        if (vars->mag_valid) {
            kalman_update_omega(state, vars->omega_from_accel);
        } else {
            float sp0 = state->kf_P_scale[0], sp1 = state->kf_P_scale[1];
            state->kf_P_scale[0] = 0.0f; state->kf_P_scale[1] = 0.0f;
            kalman_update_omega(state, vars->omega_from_accel);
            state->kf_P_scale[0] = sp0;  state->kf_P_scale[1] = sp1;
        }
    }

    /* -- Synchronous demodulation -> mag_angle ---------------------------- */
    derot_filter_step(in, state, vars);

    /* -- Kalman update - magnetometer ------------------------------------- */
    if (vars->mag_valid)
        kalman_update_theta(state, vars->mag_angle);
```
Remove the now-duplicated `vars->mag_valid = ...` line that was after `derot_filter_step` (the assignment now happens once, before the accel update).

- [ ] **Step 2: Build core tests still pass.**
Run: `pwsh tools/replay/build_tests.ps1 brain` and `... kalman`
Expected: all pass.

---

### Task A5: Validate the fix in replay (the real proof)

**Files:** none (uses `tools/replay/`)

- [ ] **Step 1: Rebuild harness with the new core:**
Run: `pwsh tools/replay/build.ps1`  → expect OK.

- [ ] **Step 2: Replay the long log and measure precession:**
```bash
LOG="sunshine_brain/logs/2026-06-17_02-26-43_Spiritridge3.sun"
tools/replay/replay.exe "$LOG" > /tmp/fixed_cont.csv
.venv/Scripts/python.exe tools/replay/analyze.py precession /tmp/fixed_cont.csv
```
**Pass criteria:**
- LED precession magnitude drops from ≈ +12 rad/s to **|precession| < 1.0 rad/s** on the gap-free window.
- `derot |I,Q|` improves toward Earth-field truth (**> 19 µT**, up from ~15).
- `kf_scale` (add a quick check) converges to roughly **+0.05…+0.09** (absorbing the ~+7.7% accel over-read) and is stable (not railed at the clamp).

- [ ] **Step 3: Verify spin-up + stability over the FULL log (no NaN / no divergence):**
```bash
.venv/Scripts/python.exe - <<'PY'
import csv, numpy as np
t=[];kf=[];om=[];sc=[]
for r in csv.DictReader(open('/tmp/fixed_cont.csv')):
    kf.append(float(r['kf_theta'])); om.append(float(r['kf_omega'])); sc.append(float(r['kf_scale']))
kf=np.array(kf);om=np.array(om);sc=np.array(sc)
print("NaN:", np.isnan(kf).any() or np.isnan(om).any() or np.isnan(sc).any())
print("omega range:", om.min(), om.max(), "| scale range:", sc.min(), sc.max())
PY
```
Expected: `NaN: False`; omega within physical range (≤ ~300 rad/s), scale within ±0.30.

- [ ] **Step 4: Repeat on the second log** `2026-06-17_01-54-20_Sprdg2.sun` (different session) — confirm no NaN/divergence and, if it has steady MELTY, reduced precession. Record results.

- [ ] **Step 5: Checkpoint A** — Part A complete when Steps 2–4 pass. Record before/after numbers in this file under each step.

> **If precession does NOT drop below 1 rad/s:** do not pile on fixes — return to systematic-debugging Phase 1. Likely suspects in order: (a) `KF_Q_SCALE` too small (scale can't adapt fast enough) — try `1e-8f`; (b) mag update too weak to make `s` observable — verify `kf_scale` is actually moving; (c) sign error in the accel `H` Jacobian — re-derive against the test in A3-step1.

---

# PART B — 50 Hz / 20-input telemetry over ESP-NOW v2

> Independent of Part A. Validated by compilation + host parse tests (no hardware here). On-air behaviour MUST be bench-verified before a match.

### Task B1: Brain — send 20 inputs/frame

**Files:** Modify `sunshine_brain/src/telemetry.cpp:22-30`

- [ ] **Step 1: Raise inputs-per-frame and the ring depth.**
```c
static RingBuffer<TelemetryEntry, 64> telem_ring;  // >= INPUTS_PER_FRAME + jitter margin

// ESP-NOW v2 payload max is 1490 bytes (IDF >= 5.4). 50 Hz frame:
// frame_id(2)+type(1)+SunshineState(76 @ schema v3)+20*SunshineInput(20*29=580) = 659 bytes
static constexpr int  INPUTS_PER_FRAME  = 20;
static constexpr int  FRAME_SIZE        = 2 + 1 + (int)sizeof(SunshineState) + INPUTS_PER_FRAME * (int)sizeof(SunshineInput);
static_assert(FRAME_SIZE <= 1490, "telemetry frame exceeds ESP-NOW v2 max payload");
```

> Note: with Part A, `sizeof(SunshineState)`=76, so FRAME_SIZE=659. Without Part A it is 643. Either way ≤1490. The `static_assert` also fails the build if the IDF lacks v2 (frame > 250 would still compile but fail on air — so also add the runtime guard in B2).

- [ ] **Step 2: Build the brain firmware** (catches IDF/ESP-NOW v2 availability):
Run (per memory note, invoke pio via penv exe):
```bash
sunshine_brain/.pio/penv/Scripts/pio.exe run -d sunshine_brain -e production
```
Expected: SUCCESS. If `esp_now_send` rejects >250 at runtime later, see B5 (verify `ESP_NOW_MAX_DATA_LEN_V2` exists in the IDF; bump platform if not).

---

### Task B2: Receiver — upgrade platform to IDF 5.x and accept 659-byte frames

**Files:**
- Modify: `sunshine_receiver/platformio.ini`
- Modify: `sunshine_receiver/include/protocol.h:13,27-29`
- Modify: `sunshine_receiver/src/espnow_rx.cpp:32-51,62-70` (IDF-5 recv callback signature)

- [ ] **Step 1: Match the brain's platform** in `sunshine_receiver/platformio.ini`:
```ini
platform = https://github.com/pioarduino/platform-espressif32/releases/download/55.03.38/platform-espressif32.zip
```
(keep board/framework/flags as-is).

- [ ] **Step 2: Update the telemetry payload size** in `sunshine_receiver/include/protocol.h`:
```c
// Brain sends 20 inputs/frame over ESP-NOW v2 (50 Hz). frame_id(2)+type(1)+
// SunshineState(76, schema v3)+20*29 = 659 bytes. (was 6 inputs/237 B on v1.)
static constexpr uint16_t ESPNOW_TELEM_SIZE  = 659;
```
Also update the comment on line 13 (`237 B payload` → `659 B payload`).

- [ ] **Step 3: Convert the recv callback to the IDF-5 signature** in `sunshine_receiver/src/espnow_rx.cpp`. Replace `on_espnow_recv` and its registration:
```c
#include <esp_now.h>   // ensure esp_now_recv_info_t is available

static void on_espnow_recv(const esp_now_recv_info_t *info,
                           const uint8_t *data, int len) {
    (void)info;
    if (len != (int)ESPNOW_TELEM_SIZE) return;
    if (data[2] != 0x01) return;
    int next = 1 - write_idx;
    memcpy(telem_buf[next], data, ESPNOW_TELEM_SIZE);
    write_idx = next;
    last_brain_frame_ms = (uint32_t)millis();
    brain_connected     = true;
    xSemaphoreGive(telem_sem);
}
```
IDF-5 exposes RSSI in `info->rx_ctrl->rssi`; capture it to retire the sniffer workaround (optional, keep behaviour minimal): add `espnow_rx_update_rssi(info->rx_ctrl->rssi);` inside the callback. Update the stale comment block at the top of the file (it references IDF 4.4).

- [ ] **Step 4: Build the receiver firmware:**
```bash
sunshine_receiver/.pio/penv/Scripts/pio.exe run -d sunshine_receiver
```
Expected: SUCCESS. Fix any other IDF-4→5 API breaks the compiler flags (likely none beyond the callback).

---

### Task B3: Receiver USB bridge — widen scratch/telemetry buffers

**Files:** Modify `sunshine_receiver/src/usb_bridge.cpp:63,150`

- [ ] **Step 1: The TX scratch and forward buffer already key off `ESPNOW_TELEM_SIZE`** (`tx_scratch[ESPNOW_TELEM_SIZE + 8]`, `telem_buf[ESPNOW_TELEM_SIZE]`), so they resize automatically with B2-step2. Confirm no hard-coded 237 remains:
Run: `grep -rn "237" sunshine_receiver/`
Expected: no matches (all sizes come from `ESPNOW_TELEM_SIZE`).

- [ ] **Step 2: Already built in B2-step4** — no separate action; this task is the grep verification.

---

### Task B4: Host — parse 20-input / 659-byte frames (TDD)

**Files:**
- Modify: `sunshine_app/src-tauri/src/protocol.rs:14-18,125-138`
- Test: `sunshine_app/src-tauri/src/protocol.rs` (add `#[cfg(test)]` module)

- [ ] **Step 1: Write a failing parse test.** Add to the bottom of `protocol.rs`:
```rust
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn telem_frame_is_659_bytes_and_parses_20_inputs() {
        assert_eq!(INPUTS_PER_FRAME, 20);
        assert_eq!(ESPNOW_TELEM_SIZE, 2 + 1 + 60 /*wait: v3 state is 76*/ + 20*29);
        let mut payload = vec![0u8; ESPNOW_TELEM_SIZE];
        payload[0] = 0x34; payload[1] = 0x12;          // frame_id = 0x1234
        let f = parse_telem(&payload);
        assert_eq!(f.frame_id, 0x1234);
        assert_eq!(f.inputs.len(), INPUTS_PER_FRAME);
    }
}
```
> The state size depends on whether Part A landed. If A is in: `sizeof(SunshineState)=76` ⇒ `ESPNOW_TELEM_SIZE = 2+1+76+580 = 659`. If running B standalone (A not applied): `=60` ⇒ `643`. Set the constant to match the actual `size_of::<SunshineState>()`; the test below computes it from the struct to stay correct either way.

Use this robust form for the test instead:
```rust
    #[test]
    fn telem_frame_size_matches_struct_and_parses_20_inputs() {
        use std::mem::size_of;
        assert_eq!(INPUTS_PER_FRAME, 20);
        let expect = 2 + 1 + size_of::<SunshineState>() + 20 * size_of::<SunshineInput>();
        assert_eq!(ESPNOW_TELEM_SIZE, expect, "frame size constant must match structs");
        let mut payload = vec![0u8; ESPNOW_TELEM_SIZE];
        payload[0] = 0x34; payload[1] = 0x12;
        let f = parse_telem(&payload);
        assert_eq!(f.frame_id, 0x1234);
        assert_eq!(f.inputs.len(), INPUTS_PER_FRAME);
    }
```

- [ ] **Step 2: Run it, watch it fail:**
Run: `cd sunshine_app/src-tauri && cargo test telem_frame_size`
Expected: FAIL (`INPUTS_PER_FRAME` is 6 / size mismatch).

- [ ] **Step 3: Update the constants** in `protocol.rs` (lines 14-18). Make the size derive from the structs so it can't drift:
```rust
use std::mem::size_of;
// Brain sends 20 inputs/frame over ESP-NOW v2 @ 50 Hz.
// 2 (frame_id) + 1 (type) + SunshineState + 20×SunshineInput.
pub const INPUTS_PER_FRAME:  usize = 20;
pub const ESPNOW_TELEM_SIZE: usize = 3 + size_of::<SunshineState>() + INPUTS_PER_FRAME * size_of::<SunshineInput>();
const MAX_USB_PAYLOAD_SIZE: usize = ESPNOW_TELEM_SIZE;
```

- [ ] **Step 4: Fix `parse_telem` offsets** to use the struct size for the input base (lines 125-138):
```rust
fn parse_telem(payload: &[u8]) -> TelemetryFrame {
    use std::mem::size_of;
    let frame_id = u16::from_le_bytes(payload[0..2].try_into().unwrap());
    let st = size_of::<SunshineState>();
    let state: SunshineState = unsafe {
        std::ptr::read_unaligned(payload[3..3+st].as_ptr() as *const SunshineState)
    };
    let mut inputs = [SunshineInput::default(); INPUTS_PER_FRAME];
    let isz = size_of::<SunshineInput>();
    for i in 0..INPUTS_PER_FRAME {
        let off = 3 + st + i * isz;
        inputs[i] = unsafe {
            std::ptr::read_unaligned(payload[off..off+isz].as_ptr() as *const SunshineInput)
        };
    }
    TelemetryFrame { frame_id, state, inputs }
}
```

- [ ] **Step 5: Run test, verify pass:**
Run: `cd sunshine_app/src-tauri && cargo test telem_frame_size`
Expected: PASS.

- [ ] **Step 6: Confirm logging writes `num_inputs=20`.** Check `sunshine_app/src-tauri/src/logging.rs` writes `INPUTS_PER_FRAME` into the header `num_inputs` field (it imports the constant). If it hard-codes 6 or 20, make it use `protocol::INPUTS_PER_FRAME`.
Run: `grep -n "num_inputs\|INPUTS_PER_FRAME\|sizeof_state\|size_of" sunshine_app/src-tauri/src/logging.rs`
Expected: header `num_inputs` = `INPUTS_PER_FRAME`; `sizeof_state` = `size_of::<SunshineState>()`. Fix if hard-coded.

---

### Task B5: Full host build + end-to-end constant audit

**Files:** none (verification)

- [ ] **Step 1: Build the whole host app** (compiles FFI against the v3 core too):
Run: `cd sunshine_app && pnpm tauri build --debug` (or `cargo build` in `src-tauri`)
Expected: SUCCESS, including the `size_of::<SunshineState>()==76` assert from A1-step4.

- [ ] **Step 2: Grep for stale frame-size assumptions across all three codebases:**
Run: `grep -rn "237\|= 6\b\|PER_FRAME.*6\|20\b.*input" sunshine_brain/src sunshine_receiver/src sunshine_app/src-tauri/src | grep -iv "test\|//"`
Expected: no remaining 237 / 6-input assumptions in live code paths.

- [ ] **Step 3: Run the full Rust test suite:**
Run: `cd sunshine_app/src-tauri && cargo test`
Expected: all pass (including replay round-trip tests; old `.sun` logs still load via `read_padded` zero-pad).

- [ ] **Step 4: Checkpoint B** — Part B complete. **Document clearly that on-air ESP-NOW v2 transfer and the 50 Hz cadence were NOT hardware-tested here and must be bench-verified** (flash both boards; confirm `frame_id` gap rate and the `analyze.py gaps` check on a freshly captured log shows the dropped-input rate collapses).

---

## Final self-review checklist (run after execution)

- [ ] Part A: precession < 1 rad/s in replay on Spiritridge3; derot |I,Q| > 19 µT; no NaN over both logs; `kf_scale` stable, not railed.
- [ ] Part A: all `sunshine_core` C tests pass under `build_tests.ps1`.
- [ ] Old v2 `.sun` logs still load and replay in the host app (backward compat via zero-pad).
- [ ] Part B: brain + receiver firmware compile on the IDF-5 platform; host app builds; `cargo test` green.
- [ ] Schema bumped to 3; `ffi.rs` mirror + size assert (76) updated; `replay.c` v3-aware.
- [ ] DEBUGGING.md log-format section updated (sizeof_state 60→76, num_inputs 6→20, schema 3) — add as a doc step during execution.
- [ ] No git commands were run.
