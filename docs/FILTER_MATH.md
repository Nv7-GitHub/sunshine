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

## Step 2: Synchronous Demodulation (Magnetometer)

### The problem: Earth's field rotates in the IMU frame

Earth's magnetic field is a fixed vector in the lab frame. As the robot spins, that vector appears to rotate in the IMU's frame of reference. The magnetometer reads a sinusoidal oscillation at exactly the body rotation frequency.

In the IMU frame, if the robot is at angle θ, the Earth field (amplitude B, direction φ_earth) reads:

```
mag_x = B × cos(φ_earth - θ)
mag_y = B × sin(φ_earth - θ)
```

So mag_x and mag_y oscillate at the rotation frequency. We want to extract φ_earth − θ (i.e., the absolute angle θ, given φ_earth is constant).

### The solution: derotate then low-pass

**Step 1 — Derotate:** multiply mag_x/y by cos(θ) and sin(θ) (using the current Kalman θ estimate). This shifts the signal from "rotating at f_rot Hz" to DC:

```c
float theta = kf_theta + theta_offset;
float mx = mag_x * MAG_SCALE_UT;   // convert to µT
float my = mag_y * MAG_SCALE_UT;

float I_raw =  mx * cos(theta) + my * sin(theta);   // derotated I component
float Q_raw = -mx * sin(theta) + my * cos(theta);   // derotated Q component
```

If the derotation is perfect (θ exactly correct), `I_raw` and `Q_raw` are constant DC values proportional to `B × cos(φ_earth)` and `B × sin(φ_earth)`.

If θ is slightly off, `I_raw` and `Q_raw` oscillate at the error frequency. The low-pass filter in Step 2 removes this oscillation.

**Step 2 — Low-pass filter:** a 4th-order Butterworth filter (cutoff 1 Hz, sample rate 1 kHz) removes everything above 1 Hz, including motor electromagnetic interference (which appears at f_rot in the derotated frame) and the θ-error oscillation.

```
derot_I = lp4(I_raw, state.derot_lp_I)
derot_Q = lp4(Q_raw, state.derot_lp_Q)
```

**Step 3 — Extract angle:**

```c
mag_angle = atan2(derot_Q, derot_I)
```

This gives the Earth field angle in the robot's heading-offset frame. It's the input to the Kalman θ update.

### Why 1 Hz cutoff?

Motor electromagnetic interference appears in the magnetometer at the rotation frequency (f_rot). At 300 RPM = 5 Hz. The 1 Hz cutoff attenuates this by > 40 dB (4th-order Butterworth, one decade down from the corner frequency).

At 120 RPM (4π rad/s), f_rot = 2 Hz. The 1 Hz filter gives ~12 dB attenuation. Below 120 RPM, the motor interference is not attenuated enough to reliably extract the Earth field signal — this is why the mag Kalman update is disabled below `SUNSHINE_MAG_MIN_OMEGA`.

### LP filter implementation

Two cascaded Direct Form II biquads (4th-order total). State is stored in `SunshineState.derot_lp_I[4]` and `derot_lp_Q[4]` (four state variables per component: two per biquad section). The coefficients are in `sunshine_core.h` and were generated by `tools/gen_filter_coefficients.py`.

Biquad Direct Form II update:
```
w[n] = x[n] - a1×w[n-1] - a2×w[n-2]
y[n] = b0×w[n] + b1×w[n-1] + b2×w[n-2]
```

State `[w0, w1]` per section advances each tick.

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

### Interdependency: why the order matters

The Kalman filter uses its own θ estimate as input to the derotation step. If θ is wrong, the derotation produces a noisy `mag_angle`, which feeds back into the filter.

This is a nonlinear system handled with an Extended Kalman Filter approximation: we linearise around the current estimate. It works because:
1. The LP filter's 1 Hz bandwidth is much slower than the 1 kHz update rate — errors accumulate slowly
2. The accel omega update corrects ω quickly, keeping θ integration accurate enough for derotation
3. The whole system bootstraps: rough theta → good derot → better theta → better derot

At startup, the filter is initialised with high uncertainty (`P = diag(100, 100)`). The accel update locks in ω within a few seconds. The mag update then corrects θ once the robot reaches 120+ RPM.

---

## Summary: Signal at Each Stage

```
At 500 RPM steady spin, 50 µT Earth field:

accel_x, accel_y     ≈ ±100 counts (centripetal at 45°)
centripetal_ms2      ≈ 70 m/s²
omega_from_accel     ≈ 52 rad/s (500 RPM ≈ 52.4 rad/s) ✓

mag_x, mag_y         ≈ ±860 counts oscillating at 8.3 Hz (500 RPM)

After derotation:
I_raw, Q_raw         ≈ ±860 counts DC (if theta is correct)
                       + small ripple at 8.3 Hz from theta error
After LP filter:
derot_I, derot_Q     ≈ ±860 counts DC, ripple attenuated to < 1 count

mag_angle            = atan2(derot_Q, derot_I) = φ_earth − heading_offset

Kalman output:
est_omega            ≈ 52.4 rad/s (matches omega_from_accel at steady state)
est_theta            = robot absolute angle, unwrapped (grows monotonically)
```
