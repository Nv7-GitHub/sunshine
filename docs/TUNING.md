# Sunshine Tuning Guide

Tuning guide for the Kalman filter and MELTY drift profile. Read `FILTER_MATH.md` first if you want to understand why these parameters exist.

All constants are in `sunshine_core/include/sunshine_core.h`. Change them and rebuild. You can override them with `-D` flags in `platformio.ini` to try values without editing the source file.

---

## Kalman Filter Tuning

The Kalman filter estimates two states: `θ` (absolute angle, rad) and `ω` (angular velocity, rad/s). Four constants control how much the filter trusts each measurement source vs. its own predictions.

### Parameter Reference

| Constant | Default | Units | What it controls |
|----------|---------|-------|-----------------|
| `KF_Q_THETA` | 1e-6 | rad²/step | How much the filter expects θ to drift on its own between steps |
| `KF_Q_OMEGA` | 1e-2 | rad²/s²/step | How much ω is expected to change between steps |
| `KF_R_ACCEL` | 0.5 | rad²/s² | How noisy the accelerometer omega measurement is |
| `KF_R_MAG` | 0.01 | rad² | How noisy the magnetometer angle measurement is |

**Rule of thumb:** Q values control process noise (model uncertainty). R values control measurement noise. Higher R = trust the measurement less. Higher Q = assume the model drifts more = update faster.

### Tuning Procedure

Work through these steps in order. Do each at bringup level 4 (nav filter on, DShot zeroed). Use the host app graph panel — plot the channels mentioned.

#### Step 1: Tune omega (ω) tracking

Plot `vars.omega_from_accel` and `vars.est_omega` simultaneously.

**Spin the robot at a steady speed** (hand-spin or with a bench drill — props off). After spin-up settles (~2 seconds):
- `omega_from_accel` and `est_omega` should track closely
- During spin-up, `omega_from_accel` reads high (tangential acceleration inflates it — see FILTER_MATH.md). This is expected.

**If `est_omega` lags `omega_from_accel` during steady state:**
- Decrease `KF_R_ACCEL` (trust accel more) or increase `KF_Q_OMEGA` (assume ω changes more)

**If `est_omega` is too noisy / jumpy:**
- Increase `KF_R_ACCEL` (trust accel less)

**Typical good values:** `KF_R_ACCEL = 0.3–1.0` (the accel is the rate sensor at all times), `KF_Q_OMEGA = 5e-3–1e-2`

#### Step 2: Verify mag threshold

Plot `vars.mag_valid`. It should become 1 (true) at around 480 RPM (16π ≈ 50.3 rad/s). If the robot never crosses this threshold, check that `est_omega` is tracking correctly (Step 1). If the LED never locks while spinning fast, also check `input.mag_x/y` for a dead/frozen sensor (zero variance = no data) before touching thresholds.

The threshold is defined by `SUNSHINE_MAG_MIN_OMEGA = 16π rad/s` in `sunshine_core.h`. It's set by the spin-tracking band-pass (`mag_heading.c`): the band's lower edge is ≈ 0.75·spin-freq, and below ~8 Hz spin that edge sinks toward the slow average-ESC-current band (which the 2nd-order skirt no longer rejects well) and the tangential-accel inflation of `est_omega` grows. Hard-iron DC is killed at any speed by the band-pass's zero at DC, so it isn't what sets the minimum.

#### Step 3: Tune angle (θ) tracking

Plot `vars.est_theta`. With the mag filter active (above 480 RPM), θ should converge to a stable value rather than drifting. The open-loop mag heading is absolute, so it should not drift; a slow creep means the mag isn't trusted enough (decrease `KF_R_MAG`).

Also watch the LED: it should appear as a stationary dot at a fixed heading once the filter converges (~3-5 seconds after crossing the mag threshold).

**If θ drifts slowly over 30+ seconds:**
- Decrease `KF_R_MAG` (trust mag more) or increase `KF_Q_THETA` (allow angle to correct faster)

**If θ jumps/oscillates:**
- Increase `KF_R_MAG` (trust mag less)
- Increase `KF_R_ACCEL` to reduce omega noise feeding into theta via the covariance matrix

**If filter never converges (θ keeps sweeping):**
- Check `mag_x_filt` and `mag_y_filt` — these are the band-passed Earth-field axes; they *oscillate* at the spin frequency, but their magnitude `sqrt(x²+y²)` should be a steady ~18–22 µT. If the magnitude collapses, `omega_from_accel` is so far off that the spin frequency has fallen outside the ±33% tracking band (or the spin is below the 480 RPM threshold) — fix `est_omega`/`omega_from_accel` first (Step 1).

**Typical good values:** `KF_R_MAG ≈ 0.01` (open-loop mag is a clean absolute reference, so trust it), `KF_Q_THETA = 1e-7–1e-5`.

#### Magnetometer band-pass centre LP — `MAG_BP_FC_LP_HZ` (default 1.5 Hz)

The mag band-pass is centred on the spin frequency taken from `omega_from_accel`.
That accel rate wobbles at the spin frequency **while translating** (linear body
acceleration adds a once-per-rev term), and retuning the band-pass from the raw
instantaneous value makes it *time-varying*, which injects wobble into `mag_angle`.
`MAG_BP_FC_LP_HZ` low-passes the centre so the filter stays quasi-LTI. Measured on
real logs it cut the `mag_angle` rate-error ~65%. It is re-seeded below the mag-valid
threshold, so a fast spin-up (even right after an impact stops the robot) carries no
lag across the stop.

- **Raise it** (e.g. 3–5 Hz) to track spin-up faster, at the cost of rejecting less
  translation wobble.
- **Lower it** for a steadier centre but a laggier response to real rate changes.
- `spinup_lag.py` shows the true spin rate stays inside the ±33% pass band ~98.7% of
  the mag-valid time at 1.5 Hz; the residual is brief impact glitches (cutoff-
  independent), so 1.5 Hz is a good default.

#### Angular-rate source: accel magnitude + mag sign (v5 mag-rate REVERTED)

The Kalman rate comes from the **accel magnitude** (`omega_from_accel = √(a_c/r)`)
with the **mag's sign** (`state->spin_rate_lp` — the accel can't sense CW/CCW).

The v5 design instead drove the rate from the mag's own rotation rate while the mag
was valid, reasoning it is unbiased by linear acceleration during translation. That
was **reverted on the 2026-08-15 steel-arena log**: every venue measured so far has
AC magnetic interference tones inside the spin band (a ~9.77 Hz harmonic comb at
19.5/29.3/39.1/48.8 Hz — the 102.4 ms WiFi-beacon period — plus 50/100 Hz mains-like
tones, 2–7 µT; both home logs and the arena show them). An in-band tone beats
against the Earth line and wobbles `mag_angle` at |f_spin − f_tone| (3–13 Hz).
The angle update tracks that wobble roughly 1:1, but a *rate* derived from the angle
DIFFERENTIATES it — multiplying the wobble by the beat frequency — so the mag-rate
path amplified it into ±10–20 rad/s of `kf_omega` wobble: the arena's
"locks, bounces all over, locks" LED. Measured with the replay bounce metric
(2 s quadratic-detrend residual of `kf_theta`):

| config | arena stationary p50/p90 | home stationary p50/p90 | home translating p50 |
|---|---|---|---|
| v5 mag rate | 25.7° / 143.5° | 22.2° / 44.9° | 24.6° |
| accel rate + mag sign | **11.7° / 26.4°** | 21.3° / 43.9° | 24.8° |

The v5 translation benefit does not reproduce (the interference exists at home too);
the accel's once-per-rev translation wobble sits at the spin frequency where the KF
attenuates it, and its +2–12% Jensen bias is cancelled by the strong mag angle
anchor (`KF_R_MAG = 0.01` — measured ~8° steady innovation offset). Do NOT "fix"
LED wobble by raising `KF_R_MAG` above ~0.05: the weakened anchor lets the accel
bias walk the LED 50–100° off the field reference. `spin_freq_lp` is still used
only to centre the band-pass (loop-independent).

#### Step 4: Pass/fail check

At 500+ RPM, run the robot for 30 seconds:
- LED must appear stationary (not sweeping)
- `est_theta` RMS error vs. visual heading: < 5° (0.087 rad)
- `est_omega` matches `omega_from_accel` within 10% during steady-state spin

---

## MELTY Drift Tuning

MELTY mode applies a differential DShot command that changes with robot angle. The left and right wheels receive equal-and-opposite offsets around the base spin command, so the body keeps spinning while the average world-frame force points in the commanded direction.

### Parameter Reference

| Constant | Default | What it controls |
|----------|---------|-----------------|
| `DRIFT_AMPLITUDE` | 0.40 | Max differential as a fraction of available symmetric DShot headroom. |
| `DRIFT_PLATEAU_WIDTH` | 0.35 | Fraction of full rotation spent at each +1 and -1 plateau. 0.35 gives two 126° plateaus and two 54° ramps. |
| `DRIFT_PHASE_OFFSET_RADS` | 0.0 | Fixed motor-timing offset between the LED/driver heading and the wheel-force waveform. |
| `DRIFT_PHASE_LEAD_S` | 0.018 | ESC/traction lag compensation. Added phase is `kf_omega * DRIFT_PHASE_LEAD_S`. **Measured, per-build** — `tools/replay/translation_lag.py`, procedure in BRINGUP.md Level 5 Step 4. |
| `THETA_RATE_RADS` | π rad/s | Heading offset rate per full left/right arrow deflection (ctrl_theta = ±127). |

### How the pulse works

At each tick, the robot's current angle relative to the commanded drive direction gives a `phase` value. A balanced bipolar trapezoid converts that phase to a differential multiplier:

```
+1.0     flat push
         ┌──────────────┐
 0.0  ───┘              └── ramps
                         ┌──────────────┐
-1.0                    flat pull/opposite side
      0                 pi              2pi
```

The waveform has zero mean over a revolution and satisfies `wave(phase + pi) = -wave(phase)`. That symmetry matters: the robot gets one push direction for half the cycle and the opposite wheel differential 180° later, instead of a one-sided bias that mostly loads one wheel.

`phase = robot_angle - drive_dir + DRIFT_PHASE_OFFSET_RADS + kf_omega * DRIFT_PHASE_LEAD_S`

`headroom = min(base - DSHOT_NEUTRAL, DSHOT_MAX - base)`

`diff = wave(phase) × drive_magnitude × DRIFT_AMPLITUDE × headroom`

`dshot_left  = base + diff`
`dshot_right = base - diff`

`headroom` prevents clipping. At low throttle there is little room above neutral; at very high throttle there is little room below max. Translation authority is strongest at moderate spin commands and intentionally fades near full throttle.

### Spin rate vs. translation authority (measured)

There are **two** reasons to translate at a *moderate* spin, not maximum:

1. **Headroom (control side):** the symmetric DShot headroom peaks at ~50% throttle
   and shrinks toward both extremes (above).
2. **Motor/ESC bandwidth (plant side):** the wheels can only change speed so fast.
   Measured from real logs (`tools/replay/erpm_bandwidth.py`), the achieved eRPM
   ripple per unit DShot ripple **falls as spin rises** (gain ~18 at 15 Hz spin →
   ~11 at 22 Hz), with a large phase lag. So at high RPM the once-per-rev modulation
   is increasingly attenuated *and* phase-shifted — translation gets weaker and its
   direction rotates with speed. Higher RPM buys gyroscopic stability but hurts
   translation authority; find the sweet spot experimentally.

### Translation force requires the wheel-speed cap (tire saturation)

Translation force is the *difference* between the two wheels' contact-patch friction
forces, and friction saturates at ~µN within a few tenths of a m/s of slip. Measured
on the pre-cap translation2 log: both wheels carried **+0.7 to +3.7 m/s of constant
forward slip**, and the drift-wave modulation (±0.1–0.9 m/s realized) never brought
either wheel near zero slip — so both tires sat pinned at saturated µN all
revolution and the differential force was ~zero *regardless of the commanded
waveform*. Symptom: motors audibly modulate, eRPM visibly modulates, robot barely
moves and wobbles inconsistently (the residual forces are normal-load fluctuations,
not the drift wave). The wheel-speed cap fixes this as a side effect of fixing the
bounce: it pins mean slip under `WHEEL_SLIP_ALLOW_MS` (5.0 m/s max), so the drift wave now
swings the retreating wheel down through zero slip into braking — one wheel
saturated forward, the other near-zero/braking = a real once-per-rev force
differential. Consequence: **do not "fix" weak translation by disabling or loosening
the cap** — that removes the very condition translation needs.

### `DRIFT_PHASE_LEAD_S` from the eRPM lag

The DShot→eRPM lag above turns into a heading-referenced phase error `omega × lag`
that grows with spin rate — exactly what `DRIFT_PHASE_LEAD_S` compensates.

**Measure it, don't guess it:** `tools/replay/translation_lag.py <log.sun>` cross-
correlates the logged DShot differential against the eRPM differential over every
translation window and prints the recommended constant (full procedure: BRINGUP.md
Level 5 Step 4). Unlike single-frequency demodulation (`erpm_bandwidth.py`) the
time-domain cross-correlation is not ambiguous modulo one rotation, and the script's
constant-offset residual check separates lag from a `DRIFT_PHASE_OFFSET_RADS` problem.

**Measured (2026-07-20 translation2 log):** a pure **time delay of ~20 ms** (18–24 ms
over 24 windows), identical for both spin directions, with **no constant offset** (so
`DRIFT_PHASE_OFFSET_RADS` stays 0 — the phase convention is correct). Subtracting the
~3 ms median-5 eRPM telemetry lag leaves **~17 ms of physical actuation delay** →
`DRIFT_PHASE_LEAD_S = 0.018f`. Uncompensated, this was a 110–150° force-direction
error at 1000–1300 RPM — the wheel-speed peak landed nearly opposite the commanded
direction, which is why the robot wobbled and barely translated. Confirm the residual
direction error on hardware (position cannot be replayed from logs) and refine with
the two-speed method in Step 3 below.

### Inverted operation flips the apparent translation direction (not a bug)

If the robot is **upside down**, its world-frame spin reverses (the schema-v4 spin-sign
recovery handles this, so `kf_omega` reads negative and the LED still tracks). But the
whole driver frame is mirrored, so "W" appears to drive *away* from the LED. This is
expected — verify direction **right-side up**. Tell inversion from the logs by
`input.accel_z` sign (≈ +20 counts upright, ≈ −20 inverted).

### Tuning Procedure

Do this at bringup Level 5 (production firmware, props on, open floor).

#### Step 1: Confirm LED is stationary

Do not begin drift tuning if the LED is sweeping. Fix Kalman tuning (Level 4) first.

#### Step 2: Set baseline throttle

In MELTY mode, bring throttle up slowly with arrow keys until the robot spins at a consistent speed with the LED appearing stationary. This is your tuning throttle. Hold it there throughout tuning.

#### Step 3: Test forward translation

Press W briefly. The robot should drift forward (toward the LED heading when W is pressed).

**If the robot barely moves:** First make sure throttle is not near max, because headroom shrinks there. If direction is repeatable but weak, increase `DRIFT_AMPLITUDE` (e.g. 0.40 → 0.55).

**If the robot moves sideways or backwards:** If the LED is stationary, tune `DRIFT_PHASE_OFFSET_RADS`. Start with 15-30° steps (`0.26f` to `0.52f` rad). If a positive change makes the direction worse, use the opposite sign.

**If the direction is correct at one RPM but wrong at another:** Tune `DRIFT_PHASE_LEAD_S`. A time lag turns into phase error as `omega * lag`; at 240 rad/s, 1 ms is about 14°. Estimate the needed lead from two speeds:

`DRIFT_PHASE_LEAD_S ≈ (offset_high - offset_low) / (omega_high - omega_low)`

**If the robot translates too aggressively and loses spin speed:** Decrease `DRIFT_AMPLITUDE`. The controller now scales by available DShot headroom, but large differential still modulates traction and spin energy.

#### Step 4: Test all four directions

Test N/S/E/W at the same throttle level. Pass if all four directions produce consistent, controllable translation in the correct direction.

#### Step 5: Tune waveform shape (optional)

If translation is jerky or inconsistent, adjust `DRIFT_PLATEAU_WIDTH`:
- Higher values, up to about `0.45`, give longer max-output dwell and behave more like a rectangle.
- Lower values, down toward about `0.25`, widen the ramps and are gentler for the ESC/wheel.
- Keep the waveform balanced; do not reintroduce unequal positive/negative dwell.

#### Step 6: Heading rate

`THETA_RATE_RADS` controls how fast the driver's heading reference rotates when holding left/right arrows. At the default (π rad/s), full deflection rotates the heading 180°/second.

Increase if the driver needs to re-orient heading quickly. Decrease if small arrow taps cause too much heading drift.

### Common Problems

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Robot spins in place, doesn't translate | No DShot headroom, phase very wrong, or too little amplitude | Use moderate throttle, tune `DRIFT_PHASE_OFFSET_RADS`, then increase `DRIFT_AMPLITUDE` |
| Translation direction wrong by a fixed angle | Motor timing phase offset | Tune `DRIFT_PHASE_OFFSET_RADS` |
| Translation direction changes with RPM | ESC/motor/traction lag | Tune `DRIFT_PHASE_LEAD_S` |
| Translation direction wrong by ~180° | Sign/geometry convention flipped | Try phase offset near ±π; then check motor/axis conventions |
| Robot wobbles during translation | Pulse too strong for this speed | Decrease `DRIFT_AMPLITUDE` |
| Translation only works at high RPM | Mag filter settling time too long | Check `KF_R_MAG`, ensure smooth spin-up |
| Translation disappears near full throttle | No symmetric DShot headroom remains | Use lower spin throttle; full throttle prioritizes spin energy |
