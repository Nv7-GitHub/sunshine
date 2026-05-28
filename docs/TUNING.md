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
| `KF_Q_OMEGA` | 1e-3 | rad²/s²/step | How much ω is expected to change between steps |
| `KF_R_ACCEL` | 0.5 | rad²/s² | How noisy the accelerometer omega measurement is |
| `KF_R_MAG` | 0.1 | rad² | How noisy the magnetometer angle measurement is |

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

**Typical good values:** `KF_R_ACCEL = 0.3–1.0`, `KF_Q_OMEGA = 5e-4–5e-3`

#### Step 2: Verify mag threshold

Plot `vars.mag_valid`. It should become 1 (true) at around 120 RPM (4π ≈ 12.6 rad/s). If the robot never crosses this threshold, check that `est_omega` is tracking correctly (Step 1).

The threshold is defined by `SUNSHINE_MAG_MIN_OMEGA = 4π rad/s` in `sunshine_core.h`. Do not lower it below 2π (60 RPM) — at low speeds the motor/ESC magnetic field is not sufficiently attenuated by the 1 Hz LP filter.

#### Step 3: Tune angle (θ) tracking

Plot `vars.est_theta`. With the mag filter active (above 120 RPM), θ should converge to a stable value rather than drifting.

Also watch the LED: it should appear as a stationary dot at a fixed heading once the filter converges (~3-5 seconds after crossing the mag threshold).

**If θ drifts slowly over 30+ seconds:**
- Decrease `KF_R_MAG` (trust mag more) or increase `KF_Q_THETA` (allow angle to correct faster)

**If θ jumps/oscillates:**
- Increase `KF_R_MAG` (trust mag less)
- Increase `KF_R_ACCEL` to reduce omega noise feeding into theta via the covariance matrix

**If filter never converges (θ keeps sweeping):**
- Check `derot_I` and `derot_Q` — they should be near-constant DC values at steady spin speed. If they're oscillating, the derotation is using a bad theta estimate as input. Let the robot spin up to a consistent speed and wait for the LP filter to settle (3-5 seconds).

**Typical good values:** `KF_R_MAG = 0.05–0.3`, `KF_Q_THETA = 1e-7–1e-5`

#### Step 4: Pass/fail check

At 500+ RPM, run the robot for 30 seconds:
- LED must appear stationary (not sweeping)
- `est_theta` RMS error vs. visual heading: < 5° (0.087 rad)
- `est_omega` matches `omega_from_accel` within 10% during steady-state spin

---

## MELTY Drift Tuning

MELTY mode applies a differential DShot command that pulses as the robot rotates, biasing it toward the commanded direction. Three constants control the pulse shape; one controls heading rate.

### Parameter Reference

| Constant | Default | What it controls |
|----------|---------|-----------------|
| `DRIFT_AMPLITUDE` | 0.40 | Max differential as a fraction of base spin throttle. 0.40 = ±40% at full drive input. |
| `DRIFT_PULSE_WIDTH` | 0.25 | Fraction of full rotation where the differential is at its peak value. 0.25 = 90° of the 360°. |
| `DRIFT_RAMP_WIDTH` | 0.10 | Fraction of rotation used for the linear ramp between peak and trough. 0.10 = 36°. |
| `THETA_RATE_RADS` | π rad/s | Heading offset rate per full left/right arrow deflection (ctrl_theta = ±127). |

### How the pulse works

At each tick, the robot's current angle relative to the commanded drive direction gives a `phase` value. A trapezoidal wave converts phase to a differential multiplier:

```
+1.0  ┌────────┐
      │        │
  0.0 ┤        ├─── ramps ───┤
      │                       │
-1.0  └───────────────────────┘
      0        ←pulse_width→  π  ←ramp→  2π
```

`diff = trapezoid(phase) × drive_magnitude × DRIFT_AMPLITUDE × base_throttle`

`dshot_left  = base + diff`
`dshot_right = base - diff`

### Tuning Procedure

Do this at bringup Level 5 (production firmware, props on, open floor).

#### Step 1: Confirm LED is stationary

Do not begin drift tuning if the LED is sweeping. Fix Kalman tuning (Level 4) first.

#### Step 2: Set baseline throttle

In MELTY mode, bring throttle up slowly with arrow keys until the robot spins at a consistent speed with the LED appearing stationary. This is your tuning throttle. Hold it there throughout tuning.

#### Step 3: Test forward translation

Press W briefly. The robot should drift forward (toward the LED heading when W is pressed).

**If the robot barely moves:** Increase `DRIFT_AMPLITUDE` (e.g. 0.40 → 0.55).

**If the robot moves sideways or backwards:** The heading reference is off. Tune Kalman first. Or check `THETA_RATE_RADS` — the heading offset may have accumulated from left/right arrow presses.

**If the robot translates too aggressively and loses spin speed:** Decrease `DRIFT_AMPLITUDE`. At high drive magnitudes, the weaker side approaches neutral (1048) or below — if the pulse is too strong, one ESC brakes instead of driving.

#### Step 4: Test all four directions

Test N/S/E/W at the same throttle level. Pass if all four directions produce consistent, controllable translation in the correct direction.

#### Step 5: Tune pulse shape (optional)

If translation is jerky or inconsistent, adjust `DRIFT_PULSE_WIDTH` and `DRIFT_RAMP_WIDTH`:
- Wider `DRIFT_PULSE_WIDTH` (e.g. 0.35): longer push phase per rotation, smoother at lower RPM
- Wider `DRIFT_RAMP_WIDTH` (e.g. 0.15): softer transition, less abrupt force changes
- Narrower values: more precise heading control but requires accurate theta estimation

#### Step 6: Heading rate

`THETA_RATE_RADS` controls how fast the driver's heading reference rotates when holding left/right arrows. At the default (π rad/s), full deflection rotates the heading 180°/second.

Increase if the driver needs to re-orient heading quickly. Decrease if small arrow taps cause too much heading drift.

### Common Problems

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Robot spins in place, doesn't translate | LED is sweeping (theta not locked) | Fix Kalman tuning first |
| Translation direction wrong by ~180° | Heading reference flipped | Check IMU orientation; may need to negate `ctrl_x` or `ctrl_y` in control.c |
| Robot wobbles during translation | Pulse too strong for this speed | Decrease `DRIFT_AMPLITUDE` |
| Translation only works at high RPM | Mag filter settling time too long | Check `KF_R_MAG`, ensure smooth spin-up |
| One wheel braking during drift | `DRIFT_AMPLITUDE` too high | The weak side is crossing neutral; decrease amplitude or raise base throttle |
