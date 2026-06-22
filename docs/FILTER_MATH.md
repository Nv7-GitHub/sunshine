# Filter Chain — Math and Intuition

Plain-language explanation of the full signal processing chain in `sunshine_core`. All the math that goes from raw sensor counts to estimated angle and angular velocity.

---

## Overview

The robot spins. We want to know:
1. **How fast is it spinning?** → angular velocity ω (rad/s)
2. **What angle is it at right now?** → absolute angle θ (rad)

We have three sensors:
- **Accelerometer (ADXL375):** measures centripetal force → tells us ω (indirectly)
- **Magnetometer (LIS3MDL):** measures Earth's magnetic field → tells us θ (indirectly, after filtering)
- **DShot eRPM:** tells us wheel speed, but that's used for control feedback, not navigation

The processing pipeline:

```
Raw accel counts → centripetal_ms2 → omega_from_accel ──────────────────────────┐
                                                                                  ↓
                                                                          Kalman filter
                                                                                  ↑
Raw mag counts → derotate by θ estimate → I/Q components → 4th-order LP → mag_angle ┘
```

---

## Step 1: Centripetal Force → Angular Velocity

### Why centripetal gives us ω

When an object moves in a circle, it experiences centripetal acceleration directed toward the center:

```
a_centripetal = ω² × r
```

The IMU is mounted 11 mm from the robot's spin axis (`IMU_RADIUS_M = 0.011 m`). At angular velocity ω rad/s, the centripetal acceleration is `ω² × 0.011 m/s²`.

Rearranging: `ω = sqrt(a_centripetal / r)`

### The 45° mount

The IMU is physically rotated 45° relative to the radial direction. At 45°, centripetal acceleration (pointing inward radially) projects equally onto both `accel_x` and `accel_y`. This is intentional — it keeps both axes well within the ±200g range of the ADXL375 up to ~4800 RPM.

Raw centripetal magnitude:
```c
centripetal_ms2 = sqrt(accel_x² + accel_y²) * ADXL_SCALE_MS2
               = sqrt(accel_x² + accel_y²) * (49e-3 * 9.81)
```

Then:
```c
omega_from_accel = sqrt(centripetal_ms2 / IMU_RADIUS_M)
```

### The spin-up problem

During angular acceleration (spin-up or spin-down), the robot also produces **tangential acceleration** at the IMU. This adds to the centripetal vector:

```
ax = (a_centripetal - a_tangential) / sqrt(2)   (45° mount geometry)
ay = (a_centripetal + a_tangential) / sqrt(2)
```

When computing `sqrt(ax² + ay²)`, the tangential component doesn't cancel cleanly — it inflates the magnitude:

```
sqrt(ax² + ay²) = sqrt(a_centripetal² + a_tangential²) / sqrt(2) × sqrt(2)
                = sqrt(a_centripetal² + a_tangential²)
```

So during spin-up, `omega_from_accel` reads higher than true ω. This is the divergence you see in the graphs at spin-up. It's expected and harmless — the Kalman filter's covariance tracking limits how far the estimate can be pulled by the inflated reading.

### Saturation

At high speeds (> ~280g centripetal ≈ 4800 RPM), the ADXL375's ±200g range saturates. The code detects this:

```c
accel_saturated = centripetal_ms2 > (280.0f * 9.81f)
```

When saturated, the accel Kalman update is skipped entirely. The estimate coasts on the mag update and the model prediction.

---

## Step 2: Open-Loop Absolute Heading (Magnetometer)

### The problem: Earth's field rotates in the IMU frame

Earth's magnetic field is a fixed vector in the lab frame. As the robot spins, that vector appears to rotate in the IMU's frame of reference. The magnetometer reads a sinusoidal oscillation at exactly the body rotation frequency.

In the IMU frame, if the robot is at angle θ, the Earth field (amplitude B, direction φ_earth) reads:

```
mag_x = B × cos(φ_earth - θ)
mag_y = B × sin(φ_earth - θ)
```

So mag_x and mag_y oscillate at the rotation frequency. We want to extract φ_earth − θ (i.e., the absolute angle θ, given φ_earth is constant).

### The solution: high-pass each axis, then atan2

The body-fixed contributions to the magnetometer — hard-iron from the motor
magnets and PCB traces, plus the roughly-constant average ESC supply current —
are **DC in the spinning body frame** (they don't rotate with the body). Earth's
field rotates *with* the body, so it appears at the spin frequency. A high-pass
filter on each axis therefore removes the body-fixed offset and keeps the Earth
sine:

```c
float mx = mag_x * MAG_SCALE_UT;   // µT
float my = mag_y * MAG_SCALE_UT;
float mx_hp = biquad(mx, state.mag_hp_x, MAG_HP_*);   // 2nd-order Butterworth HP
float my_hp = biquad(my, state.mag_hp_y, MAG_HP_*);
```

The absolute heading is then just the angle of the high-passed vector:

```c
mag_angle = atan2(-my_hp, mx_hp);
```

(The `-my` matches the LIS3MDL y-axis inversion — see DEBUGGING.md. The absolute
declination offset `φ_earth` is absorbed by `theta_offset`, the driver's zero.)

**This is OPEN-LOOP** — it does not use the filter's own `θ` estimate, so
`mag_angle` is a true absolute reference and **cannot drift**. (The previous
design was a *closed-loop* synchronous demodulator that derotated the field by
`kf_theta` and low-pass filtered the result; its limited correction bandwidth let
a biased integration rate slowly precess the heading. That is why the heading LED
crept in MELTY.)

### Why high-pass, and the minimum speed

The HP cutoff is `fc = 0.5 Hz` (2nd-order Butterworth, `MAG_HP_*` in
`sunshine_core.h`). The Earth sine is at the spin frequency — 2 Hz at the 120 RPM
minimum, up to ~40 Hz in MELTY — well above `fc`, so it passes at ~unity gain.
The body-fixed DC and slow average-current drift are below `fc` and removed. ESC
*switching* noise is far above the spin band (kHz), so there is no
spin-frequency ESC content to separate. Below `SUNSHINE_MAG_MIN_OMEGA` (4π rad/s
≈ 120 RPM) the spin frequency approaches `fc`, DC leaks through, and the heading
is unreliable — so the mag Kalman update is gated off there.

### HP filter implementation

One 2nd-order Direct Form II biquad per axis. State is in
`SunshineState.mag_hp_x[2]` / `mag_hp_y[2]` (two delay elements per axis):
```
w[n] = x[n] - a1×w[n-1] - a2×w[n-2]
y[n] = b0×w[n] + b1×w[n-1] + b2×w[n-2]
```

The per-sample heading is noisy (the LIS3MDL runs in 1 kHz low-power mode); the
Kalman's mag update (`KF_R_MAG`, lowered to trust this clean absolute reference)
averages it down. Because the measurement is absolute and open-loop, that
denoising **cannot** introduce drift.

---

## Step 3: Kalman Filter

A 2-state linear Kalman filter estimates `[θ, ω]` at 1 kHz.

### State and model

State vector: `x = [θ (rad), ω (rad/s)]`

Motion model (constant angular velocity): angular velocity is assumed constant between steps, with θ integrating ω:

```
x[k+1] = F × x[k]

F = [[1, dt],
     [0,  1]]    where dt = 0.001 s
```

### Predict step (every tick)

```
x = F × x          →   θ += ω × dt
P = F × P × Fᵀ + Q →   P propagates with added process noise Q
```

Q is diagonal: `Q = diag(KF_Q_THETA, KF_Q_OMEGA)`. It models how much the true state drifts from the model prediction each tick.

After predict:
```
P[0,0] += P[1,0]×dt + P[0,1]×dt + P[1,1]×dt² + KF_Q_THETA
P[0,1] += P[1,1]×dt
P[1,0] += P[1,1]×dt
P[1,1] += KF_Q_OMEGA
```

### Update — Accelerometer (H = [0, 1], measures ω)

Innovation: `y = omega_from_accel - ω_estimated`

Kalman gain (simplified for H = [0,1]):
```
S = P[1,1] + KF_R_ACCEL
K = [P[0,1]/S, P[1,1]/S]
```

State and covariance update:
```
θ += K[0] × y
ω += K[1] × y
P -= K × H × P     (scalar × matrix)
```

This update is skipped when `accel_saturated = 1`.

### Update — Magnetometer (H = [1, 0], measures θ)

Innovation: `y = wrap_to_pi(mag_angle - θ_estimated)` (wrapped to [-π, π] to handle angle wraps)

Kalman gain (simplified for H = [1,0]):
```
S = P[0,0] + KF_R_MAG
K = [P[0,0]/S, P[1,0]/S]
```

State and covariance update:
```
θ += K[0] × y
ω += K[1] × y
P -= K × H × P
```

This update is skipped when `est_omega < SUNSHINE_MAG_MIN_OMEGA` (4π rad/s).

### Accelerometer down-weighting (heading-precession fix)

The accel-derived ω is biased a few percent (effective IMU radius ≠ the assumed
11 mm, plus tangential acceleration). If the accel is trusted strongly at all
times, `ω` locks onto that biased value and the heading creeps. So `brain.c`
passes `KF_R_ACCEL_LOCKED` (weak) to the omega update **once the mag is valid**,
letting the absolute mag govern the steady-state rate; `KF_R_ACCEL` (strong) is
used only during spin-up so `ω` can climb to the mag-valid threshold.

### No derotation feedback

Because Step 2 is now open-loop, the filter is genuinely linear: the mag update
is a normal absolute-angle measurement, not a function of the estimate. There is
no derot bootstrap and no closed-loop drift mode — the mag pins `θ` absolutely
and the accel supplies the high-bandwidth rate.

At startup the filter has high uncertainty (`P = diag(100, …)`). The accel update
locks in `ω` within a few seconds; the mag update corrects `θ` once the robot
passes 120 RPM and the high-pass has settled (~1 s).

---

## Summary: Signal at Each Stage

```
At 500 RPM steady spin, 50 µT Earth field:

accel_x, accel_y     ≈ ±100 counts (centripetal at 45°)
centripetal_ms2      ≈ 70 m/s²
omega_from_accel     ≈ 52 rad/s (500 RPM ≈ 52.4 rad/s) ✓

mag_x, mag_y         ≈ DC offset (hard-iron + avg current) + Earth sine
                       oscillating at 8.3 Hz (500 RPM), ~±380 counts (~22 µT)

After high-pass (per axis):
mag_x_filt, mag_y_filt ≈ the zero-centred Earth sine (DC offset removed)

mag_angle            = atan2(-mag_y_filt, mag_x_filt) = absolute heading
                       (open-loop; offset φ_earth absorbed by theta_offset)

Kalman output:
est_omega            ≈ 52.4 rad/s (matches omega_from_accel at steady state)
est_theta            = robot absolute angle, unwrapped (grows monotonically)
```
