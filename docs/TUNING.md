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

Plot `vars.mag_valid`. It should become 1 (true) at around 480 RPM (16π ≈ 50.3 rad/s). If the robot never crosses this threshold, check that `est_omega` is tracking correctly (Step 1).

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
| `DRIFT_PHASE_LEAD_S` | 0.0 | ESC/traction lag compensation. Added phase is `kf_omega * DRIFT_PHASE_LEAD_S`. |
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
