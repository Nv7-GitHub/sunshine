# Wheel-Speed Cap & eRPM Signal Integrity — Design

**Date:** 2026-08-02
**Status:** Approved for planning
**Evidence log:** `~/Documents/sunshine_logs/2026-07-20_04-20-24_translation2.sun`
(405 s total, 224 s in MELTY, schema v5 / FILE_FORMAT_VER 3)

---

## 1. Problem

The robot bounces vertically while spinning in MELTY. One contributing cause is
visible directly in the eRPM telemetry: the wheels spin up far faster than the
ground demands, then are dragged back down every time they touch. Each landing
dumps the stored wheel kinetic energy into an impulsive traction spike, which
throws the robot back into the air, and the cycle repeats.

This design addresses that one cause. It does not claim to be the only cause of
the bouncing.

A second, independent problem obstructs debugging: the logged eRPM stream is
untrustworthy — it reads zero for long stretches and occasionally spikes — so
the very signal needed to diagnose wheel behaviour cannot be read at face value.

---

## 2. Evidence

All numbers below are measured from the evidence log. **None of them are
compiled into the firmware.** They exist to justify the choice of round,
robot-independent default constants; see §3.6.

### 2.1 The wheels overspeed

With no slip, wheel angular rate is set by geometry:

```
w_wheel = |w_body| * WHEEL_CENTER_M / WHEEL_RADIUS_M
eRPM    = w_wheel * 60/(2*pi) * POLE_PAIRS  =  123.06 * |w_body|
```

Measured `erpm_left` divided by that expectation, over mag-valid spin:

| percentile | p5 | p25 | p50 | p75 | p95 |
|---|---|---|---|---|---|
| ratio | 1.05 | 1.17 | **1.30** | 1.55 | 2.27 |

The wheels typically run 30 % faster than the ground demands, and up to 2.3×.

### 2.2 The ESC is an open-loop voltage source

Predicted no-load eRPM is `duty * V_batt * KV * POLE_PAIRS`, where
`duty = (dshot - 1048) / 999`. Measured against that, using the **logged**
battery voltage:

| percentile | p5 | p25 | p50 | p75 | p95 |
|---|---|---|---|---|---|
| measured / nameplate no-load | 0.495 | 0.789 | **0.936** | 1.038 | 1.587 |

The wheels run at 94 % of the no-load speed their command implies — a median
deficit of **0.139 V** out of a median 2.489 V applied. That deficit is the
`I·R` drop under load, not an error in KV: pairing it with the median 1.15 A
derived independently from body acceleration (§2.4) gives R ≈ 0.12 Ω, matching
§2.4's estimate.

Two consequences:

- Wheel speed is set by `duty × V_batt`, so **battery voltage must appear in the
  cap**. Battery ranged 7.80–8.39 V within this single log — a 7 % speed swing
  for an unchanged DShot value.
- Because the wheels sit within 6 % of no-load speed, the motors are producing
  very little torque, which means very little traction force. Body acceleration
  over the 205–215 s spin-up was only **α ≈ 9 rad/s²**. The wheels were slipping
  uselessly, not driving the robot.

### 2.3 Drivetrain geometry is confirmed

A wheel cannot roll *slower* than the ground unless it is braking, so if the
geometry constants are right the measured ratio from §2.1 must have a hard floor
at 1.0. Over 113 k driven, mag-valid samples:

| percentile | 0.5 % | 1 % | 2 % | 5 % | 25 % | 50 % |
|---|---|---|---|---|---|---|
| left  | 0.885 | 0.974 | 1.012 | 1.055 | 1.169 | 1.298 |
| right | 0.178 | 0.912 | 0.973 | 1.026 | 1.155 | 1.283 |

The floor lands on 1.00 within ~2 %. A 10 % geometry error would have placed it
at 0.90 or 1.10. `WHEEL_RADIUS_M = 0.022`, `WHEEL_CENTER_M = 0.0405` and
7 pole pairs are therefore correct for this build.

### 2.4 Effective winding resistance

Deriving per-motor current from body acceleration
(`I = MOI*alpha*WHEEL_RADIUS_M / (2*WHEEL_CENTER_M*Kt)`) and slip voltage from
`duty*V_batt - rpm/KV`, over 37.6 k samples with α > 15 rad/s²:

| | p10 | p25 | p50 | p75 | p90 |
|---|---|---|---|---|---|
| R = ΔV/I (Ω) | 0.022 | 0.089 | **0.159** | 0.237 | 0.336 |

Roughly 2× the 0.075 Ω assumed in `simulation.rs`. This is an **upper bound** —
the estimate neglects bearing and tire drag, which understates the current truly
required and so overstates R. The independent aggregate cross-check in §2.2
(median 0.139 V deficit ÷ median 1.15 A) gives R ≈ 0.12 Ω, so the true value
lies somewhere in 0.12–0.16 Ω.

Slip voltage the robot actually used while accelerating: **median 0.197 V,
p90 0.431 V**.

### 2.5 eRPM zeros are unmeasurable samples, not stopped wheels

Zeros occur in **77.6 %** of samples where the command is at neutral, and in
**0.01 %** of driven samples. Bidirectional-DShot eRPM is derived from the ESC's
own commutation timing; at zero throttle AM32 turns the FETs off and coasts, so
there is nothing to time and it emits a decaying, meaningless period.

The decisive observation is at t = 402.9 s, where the command drops to 0 while
the body rate stays flat at −94 rad/s:

| t (s) | kf_omega | dshot | erpm_left |
|---|---|---|---|
| 402.90 | −94.0 | 1364 | 18416 |
| 402.91 | −94.7 | 0 | 15784 |
| 402.93 | −93.8 | 0 | 9864 |
| 402.95 | −99.1 | 0 | 5340 |
| 402.97 | −94.1 | 0 | 3408 |
| 402.98 | −94.8 | 0 | **25504** |
| 403.02 | −94.3 | 0 | 3312 |

Body ω is unchanged across the window, so the flywheel — and therefore the
wheels — cannot have slowed. The wheels are provably still turning at
~11 500 eRPM while the ESC reports ~3 300 with a 25 504 spike. **Zero means
"no measurement", not "stopped".**

Residual defects on *driven* samples: 0.07 % of decodes exceed 40 000 eRPM, with
single-tick jumps up to 58 000.

---

## 3. Design — the wheel-speed cap

### 3.1 Principle

Because the ESC is an open-loop voltage source (§2.2), the wheel's steady speed
is a known function of the command. Invert it: compute the DShot value whose
no-load speed equals the speed the ground demands, plus a fixed allowance for
slip, and clamp the commanded value to that.

### 3.2 Formulation

In `control.c`, MELTY branch only:

`v_batt` is `v->batt_voltage`, which `brain.c` already computes from
`in->batt_offset` before calling `control_step()`.

```
w_ref    = max(locked ? |s->kf_omega| : 0, SUNSHINE_MAG_MIN_OMEGA)   # rad/s, body
w_roll   = w_ref * WHEEL_CENTER_M / WHEEL_RADIUS_M                   # rad/s, wheel
w_cap    = w_roll + WHEEL_SLIP_ALLOW_MS / WHEEL_RADIUS_M
v_needed = (w_cap * 60/(2*pi)) / MOTOR_KV_RPM_PER_V
cap      = DSHOT_NEUTRAL + (v_needed / v_batt) * (DSHOT_MAX - DSHOT_NEUTRAL)
base     = min(base, max(cap, DSHOT_NEUTRAL))
```

`locked` is `v->mag_valid && fabsf(s->spin_rate_lp) > SUNSHINE_MAG_MIN_OMEGA` —
the exact condition `brain.c` already uses to decide whether to trust the mag
rate over the accel.

### 3.3 Why the reference rate is `max(..., SUNSHINE_MAG_MIN_OMEGA)`

Losing lock does not mean ω is unknown. Lock is *defined* by
`|kf_omega| > SUNSHINE_MAG_MIN_OMEGA`, so "not locked" carries the bound
`|omega| <= SUNSHINE_MAG_MIN_OMEGA`. The cap therefore never switches off; it
falls back to the threshold. Because `locked` implies
`|kf_omega| > SUNSHINE_MAG_MIN_OMEGA`, the single `max()` expression is
continuous across the lock boundary.

At 8.2 V the unlocked ceiling is ≈ 1194 DShot, which still permits 3.03 m/s of
slip at standstill (≈ 7.5 A, ≈ 2.96 N per wheel against a carpet friction limit
near 2.2 N). Spin-up from rest therefore remains fully traction-limited and is
not slowed by the cap, while the runaway is bounded at all times rather than
only after lock.

### 3.4 Why the rate must come from the magnetometer, not the accelerometer

`omega_from_accel` is corrupted precisely when the cap matters most: while the
robot is airborne or impacting, the accelerometer sees free-fall and impact
rather than centripetal acceleration, so the accel-derived rate spikes and would
*raise* the cap at the worst possible moment. `kf_omega` driven by the mag
rotation rate is immune to linear acceleration (see `brain.c` rate-update
comment) and is the only trustworthy reference during a bounce.

### 3.5 Why a fixed slip *speed*, not a percentage

Traction force comes from `I = (V_cmd - backEMF) / R`, so a fixed slip *voltage*
buys a fixed current and hence a fixed torque at any spin rate. A fixed slip
speed is a fixed slip voltage. Expressed as a percentage of rolling speed, the
1.0 m/s default is:

| body rate | 50 rad/s | 100 rad/s | 150 rad/s |
|---|---|---|---|
| margin | 50 % | 25 % | 17 % |

A flat 20 % would starve the robot at low spin while still permitting runaway at
high spin.

### 3.6 Constants

Added to the `Physical / sensor constants` block of `sunshine_core.h`, alongside
the existing `IMU_RADIUS_M`, and documented as **per-build parameters taken from
the mechanical design and motor spec sheet** — a different robot changes these
four values and nothing else:

| constant | value | source |
|---|---|---|
| `WHEEL_RADIUS_M` | 0.022 | mechanical design (confirmed §2.3) |
| `WHEEL_CENTER_M` | 0.0405 | mechanical design (confirmed §2.3) |
| `MOTOR_KV_RPM_PER_V` | 1100.0 | motor nameplate |
| `MOTOR_POLE_PAIRS` | 7 | motor spec (14 poles) |
| `WHEEL_SLIP_ALLOW_MS` | 1.0 | tuning knob, see below |

**No data-fitted value enters the firmware.** `MOTOR_KV_RPM_PER_V` is the 1100
nameplate. The 6 % speed deficit measured in §2.2 is *not* a KV error to be
corrected out — it is the `I·R` drop under load, which is exactly the slip doing
its job. The cap commands a **no-load** speed; the load then pulls the actual
wheel speed below it, and that difference is the torque-producing slip. Nothing
needs to be derated.

This gives the cap two clean guarantees:

- **Overspeed bound.** With the wheel unloaded — airborne, which is the case
  that matters — actual speed equals commanded no-load speed, so overspeed is
  bounded at exactly the allowance.
- **Torque budget.** With the wheel fully gripping, the entire allowance appears
  across the winding: `I_max = WHEEL_SLIP_ALLOW_MS / WHEEL_RADIUS_M * 60/(2*pi)
  / MOTOR_KV_RPM_PER_V / R`.

`WHEEL_SLIP_ALLOW_MS = 1.0` is justified, not fitted. It is 0.395 V of slip
voltage, which is **2× the median slip voltage (0.197 V) and about equal to the
p90 (0.431 V) the robot used while accelerating** (§2.4). It therefore preserves
essentially all acceleration the robot has ever demonstrated while removing the
1.3–2.3× overspeed. Against the upper-bound R from §2.4 it supports
α ≈ 66 rad/s², and against the §2.2 cross-checked R ≈ 0.12 Ω it supports
α ≈ 87 rad/s² — versus the 9 rad/s² actually achieved in the evidence log.

### 3.7 Scope of the clamp

The cap applies to `base` — the mean wheel command — and `headroom` is
recomputed from the capped base so the drift differential scales down with it.
The drift wave still rides above and below the capped base, so the driving wheel
briefly exceeds the cap while the other brakes; that asymmetry *is* the
translation mechanism and must not be clamped away.

The cap applies to MELTY only. TANK is direct driver control with no valid
rate reference.

### 3.8 Failure handling

If `v_batt` is implausible (< 5 V or > 10 V) the cap is skipped entirely
(fail-open). A stuck-low battery reading must not disarm the weapon mid-match.

### 3.9 Accepted consequence

A robot with a dead magnetometer never achieves lock and is therefore held at
the §3.3 unlocked ceiling, stalling out near 50–60 rad/s instead of spinning up
freely. A MELTY robot with no heading reference cannot be driven anyway, so this
is the right trade — but it converts "degraded but spinning" into "will not spin
up", which must be documented where bringup will see it (`BRINGUP.md`,
`DEBUGGING.md`), not left to be discovered in a match.

---

## 4. Design — eRPM signal integrity

### 4.1 Represent "unmeasurable" as NaN

When the ESC is not commutating, the eRPM field carries **NaN** rather than 0.
This costs no schema bump and no extra bytes — float16 already has NaN
encodings — and it restores the meaning of 0 to "a genuinely stopped wheel".
If eRPM is ever consumed by a control path, NaN fails loudly where a fake 0
fails silently.

`sunshine_f32_to_f16()` currently collapses NaN to +inf: it discards the
mantissa whenever `exp >= 31` (`utils.c:33`). It must preserve a non-zero
mantissa so NaN round-trips as NaN.

### 4.2 Brain-side filter hardening (`dshot.cpp`)

Three defects, only one of which the current `ErpmFilter` addresses:

1. **The escape hatch can flush downward.** `ERPM_ESCAPE = 6` flushes the median
   ring to a new level after 6 consecutive out-of-band decodes in *either*
   direction. Upward that is correct — spin-up is genuinely fast. Downward it is
   physically impossible: the flywheel cannot lose half its speed in 6 ms. Keep
   the fast escape for increases; require a sustained run on the order of 50 ms
   for decreases.
2. **The ceiling is a fixed 65 000**, so it only catches the +inf case. A
   command-derived ceiling is now available for free from the same model the cap
   uses — a wheel cannot exceed `duty * V_batt * KV * POLE_PAIRS` by more than a
   modest regen margin. This rejects the 25 504-while-coasting class of spike
   that 65 000 never will. It requires `dshot_send()` to receive the duty and
   battery voltage, both of which `nav_control.cpp` already holds.
3. **Undriven decodes must not enter the filter at all.** Below the commutation
   threshold, feed nothing and emit NaN. Today that garbage poisons the median
   ring and is then escaped-to.

### 4.3 Host / UI must render NaN as a gap

The UI is where the gap has to appear, but the backend currently destroys NaN
before it arrives. Both need the minimum change to deliver and draw it:

- **`pipeline.rs:488` — `downsample_min_max`.** Rust's `f32::min`/`f32::max`
  *ignore* NaN, so `fold(f32::INFINITY, f32::min)` silently drops NaN from a
  mixed bucket and returns `(+inf, -inf)` for an all-NaN bucket. Make it
  NaN-aware: an all-NaN bucket yields NaN; a mixed bucket yields the finite
  min/max; ±inf is never emitted.
- **`UPlotCanvas.tsx:105` — `spanGaps: true` is global.** The app already uses
  null-as-gap to mean "sparse series" for the coarse 100 Hz `real.*` channels and
  deliberately bridges across them. NaN needs to mean "no measurement, draw
  nothing", so the two must stop sharing one representation. `spanGaps` becomes
  per-series: `true` for sparse `real.*` channels, `false` for dense channels
  where a hole is a genuine absence of measurement.
- **`pipeline.rs:561` — `input.erpm_left/right`** currently plots the raw
  float16 *bit pattern* cast to f32. This is already meaningless and would
  render NaN as a ~64 000 spike. Decode it properly.

serde_json maps every non-finite f32 to `null`, which is the representation uPlot
consumes; the fix above ensures only true NaN reaches that path.

---

## 5. Verification

- **`sunshine_core` unit tests** (`test_control.c`): cap falls back to the
  threshold rate when unlocked; cap tracks `|kf_omega|` when locked; commanded
  no-load speed lands at rolling + allowance; cap scales inversely with
  `v_batt`; the drift wave still rides above the capped base; fail-open on an
  implausible battery reading; continuity across the lock boundary.
- **`f16` NaN round-trip test** — both directions, plus the existing finite
  values as a regression guard.
- **Replay over the evidence log** — a new `tools/replay/wheel_slip.py`
  reporting commanded vs. capped DShot and the resulting slip distribution, so
  the 1.0 m/s default can be confirmed or adjusted against real data rather than
  argument. `real.*` vs `rep.*` shows exactly what the cap would have done to the
  run that bounced.
- **UI check** — load the evidence log, plot `var.erpm_left`, confirm the
  undriven stretches render as gaps while the coarse `real.*` channels still
  bridge.

---

## 6. Documentation

`DEBUGGING.md` gains:

- the machine-local log path (`~/Documents/sunshine_logs`) and how to select the
  newest log;
- an **eRPM interpretation** subsection: NaN = ESC not commutating, not a
  stopped wheel, with the §2.5 coast-down evidence;
- a scenario entry for **wheel overspeed / bouncing**, including the no-slip
  reference figure of 123 eRPM per rad/s of body rate so the ratio is checkable
  at a glance.

`BRINGUP.md` gains the §3.9 dead-magnetometer consequence.

---

## 7. Explicitly out of scope

- **`simulation.rs` tire model.** `MAX_TIRE_FORCE = 25 N` is roughly 10× the
  real carpet friction limit (~2.2 N per wheel), so the simulation will not
  reproduce the bouncing.
- **Vertical dynamics.** The simulation has no vertical degree of freedom, so it
  cannot validate the bounce fix end to end — only the cap arithmetic.
- **The other causes of bouncing.** This design addresses wheel overspeed alone.

Both simulation items are real and both are larger than this change.
