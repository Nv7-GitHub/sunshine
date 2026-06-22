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

### The solution: a spin-tracking band-pass, then atan2 of the two axes

Three things sit in the magnetometer signal: (a) the **Earth field**, at the spin
frequency `f_rot`; (b) a **body-fixed DC offset** (hard-iron + the roughly
constant average ESC current — DC because it doesn't rotate with the body); and
(c) **high-frequency tones** from the LIS3MDL's 1 kHz low-power-mode sampling — a
strong one at `f_s/6 ≈ 167 Hz`, measured ~7 µT. (ESC *switching* is far higher,
kHz, and irrelevant — there is no spin-frequency ESC content.)

To isolate the Earth sine we run a **2nd-order RBJ band-pass** (Robert
Bristow-Johnson's "audio EQ cookbook" biquad) on each axis, **centred on the spin
frequency** taken from `kf_omega`:

```c
float fc    = kf_omega / (2π);              // band centre = estimated spin freq (Hz)
float w0    = 2π·fc / 1000;                 // fs = 1 kHz
float alpha = sin(w0) / (2·MAG_BP_Q);       // MAG_BP_Q = 1.5  (constant Q)
// RBJ band-pass, constant 0 dB peak gain (then divide all by a0):
b0 =  alpha;  b1 = 0;  b2 = -alpha;
a0 =  1 + alpha;  a1 = -2·cos(w0);  a2 = 1 - alpha;
float mx_bp = biquad(mx, state.mag_hp_x, b0,b1,b2,a1,a2);   // coeffs recomputed each tick
float my_bp = biquad(my, state.mag_hp_y, b0,b1,b2,a1,a2);
```

Why this filter:
- **Transmission zero at DC** (`b0+b1+b2 = 0`) → hard-iron / average-current offset
  is killed *exactly*, at any spin rate.
- **Peak at `f_rot`** with a roll-off above → rejects the 167 Hz sampling tone,
  which a *fixed* filter cannot (167 Hz is only ~2.5–4× the 40–66 Hz spin).
- **Tracks `kf_omega`**, so the pass-band follows the spin as it changes.

**Constant-Q, not fixed-Hz bandwidth.** The accelerometer's spin-rate estimate is
off by a *fraction* (measured +2…+12% on the bench; combat adds linear
acceleration + impacts, so budget ~2× ≈ 30%). A fixed ±N Hz band would slide off
the true signal at high spin. With constant `Q`, the −3 dB half-bandwidth is a
fixed fraction of the centre: `fc/(2Q) = fc/3 ≈ ±33%` at `Q = 1.5`. So the band is
≈ `0.67·f_rot … 1.33·f_rot`, comfortably wider than the bias even in messy combat,
and the true Earth sine always stays in-band (worst case only −3 dB at the edge).
Lower `Q` = wider/more robust; higher `Q` = narrower/cleaner.

**Combining the two axes (atan2).** `mx_bp` and `my_bp` are *not* two independent
references — they are the **quadrature (cos / −sin) components of one rotating
vector**: `mx_bp ≈ E·cos(φ−θ)`, `my_bp ≈ −E·sin(φ−θ)`. `atan2` returns that
vector's angle, i.e. the absolute heading, using **both** axes every sample:

```c
mag_angle = atan2(-my_bp, mx_bp);
```

They are 90° apart (X is at its zero where Y is at its peak and vice-versa), which
is exactly what `atan2` needs to resolve a full, unambiguous 0–360° heading
continuously — strictly better than reading one axis' zero-crossings (which give
heading only twice per axis per rev). The `-my` matches the LIS3MDL y-axis
inversion (DEBUGGING.md) so heading increases with `kf_theta` (CCW positive).

**This is OPEN-LOOP** — the heading never uses the `θ` estimate (only the band
*centre* uses `kf_omega`, and mis-centring merely attenuates the signal, never
biases the angle), so it **cannot drift**. (The previous design was a *closed-loop*
synchronous demodulator that derotated by `kf_theta`; its limited correction
bandwidth let a biased rate slowly precess the heading — the original MELTY LED
creep.) The constant declination offset `φ_earth` and the band-pass's constant
group-delay phase are both absorbed by `theta_offset` (the driver zero).

### Implementation, minimum speed, and the soft-iron wobble

One 2nd-order Direct Form II biquad per axis; state in
`SunshineState.mag_hp_x[2]` / `mag_hp_y[2]`. The coefficients are recomputed each
tick from `kf_omega` (a few transcendental ops — negligible at 1 kHz; the spin
rate, hence the coeffs, changes slowly).

The mag update is gated off below `SUNSHINE_MAG_MIN_OMEGA = 16π rad/s` (8 Hz spin
= **480 RPM**). The band's lower edge is ≈ `0.75·f_rot`; below 8 Hz that edge sinks
toward the slow average-current band where the 2nd-order skirt (~6 dB/oct, not a
brick wall) no longer rejects it well, and the tangential-acceleration inflation
of `omega_from_accel` becomes large. (Hard-iron DC itself never constrains the
minimum — the band-pass zeros it at any speed.)

The per-sample heading is noisy (1 kHz low-power mode), so the Kalman trusts it
moderately (`KF_R_MAG = 0.01`) and averages it down; because the measurement is
absolute, that denoising cannot add drift. One real imperfection: the two mag axes
are not equal-gain (measured amplitude ratio ≈ 1.44 → the `(x,y)` locus is an
**ellipse**, not a circle), which gives `atan2` a **±~10° 2/rev error**. It is
**zero-mean**, so it does *not* move the LED (no drift) — it just adds a few
degrees of wobble to the otherwise-stationary lit point. A soft-iron calibration
(ellipse fit → rescale/de-skew the axes before `atan2`) would sharpen it but isn't
required.

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
