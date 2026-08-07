# Sunshine Bringup Guide

Step-by-step bringup instructions for all five levels. Work through them in order. Each level builds on the previous.

**Before you start:** Read `ARCHITECTURE.md` for the big picture.

---

## Platform Notes (Read First)

### Brain — pioarduino platform issue

The brain's `platformio.ini` uses the pioarduino `stable` release URL. As of 2026-05-26 it fails during `pio run` with:

```
PackageInstallError: MissingPackageManifestError on framework-arduinoespressif32-libs
```

**Workaround:** Pin the platform to a known-good pioarduino release. Edit `sunshine_brain/platformio.ini`:

```ini
# Replace the platform line in [env_base] with a pinned release, e.g.:
platform = https://github.com/pioarduino/platform-espressif32/releases/download/51.03.07/platform-espressif32.zip
```

Check [github.com/pioarduino/platform-espressif32/releases](https://github.com/pioarduino/platform-espressif32/releases) for the latest working release. Once pioarduino fixes the `stable` pointer, revert to the URL in the file.

### Receiver — IDF 4.4 (espressif32@6.0.0)

The receiver is built against `espressif32@6.0.0` (IDF 4.4). This affects two things:

1. **Receiver-side RSSI**: The IDF 4.4 `esp_now_recv_cb_t` signature does not expose RSSI. `espnow_rx_get_rssi()` returns -127. The `RX_RSSI` USB packet will always be -127 until the receiver is rebuilt on IDF 5.x. Brain-side RSSI (`inputs.rssi`) is unaffected — it reads fine.
2. **ESP-NOW callback signature**: The existing `espnow_rx.cpp` uses the IDF 4.x signature (`const uint8_t *mac_addr, const uint8_t *data, int len`). If you later migrate to IDF 5.x, change the callback to `const esp_now_recv_info_t *info, const uint8_t *data, int len` and read RSSI from `info->rx_ctrl->rssi`.

### DShot library

`sunshine_brain/platformio.ini` uses `derdoktor667/DShotRMT @ ^0.9.5` (not `qqqlab/DShotRMT_NEO` as originally planned — that package was unavailable). If DShot arming fails, check the library's examples for the correct arming procedure with AM32 in 3D mode.

---

## Porting to a New Robot

This codebase brings up **any two-wheel meltybrain that uses the same brain PCB** — different frame, wheels, motors, ESC ratings, mass. The port is: change the per-build constants below, then run Levels 1–5 in order. Everything else (schema, telemetry, replay, app) is robot-independent.

### Where the per-build constants live

| File | Constants | Consumed by |
|------|-----------|-------------|
| `sunshine_core/include/sunshine_core.h` | **The one file with every per-robot number.** Drivetrain block: `WHEEL_RADIUS_M`, `WHEEL_CENTER_M`, `MOTOR_KV_RPM_PER_V` (nameplate, do **not** derate — see comment there), `MOTOR_POLE_PAIRS`, `WHEEL_SLIP_ALLOW_MS`. Geometry: `IMU_RADIUS_M` (accel distance from spin axis). Measured: `DRIFT_PHASE_LEAD_S` (Level 5 Step 4 below). Plant/environment block (used by the host **simulation** only, but kept here so nothing per-robot lives anywhere else): `ROBOT_MASS_KG`, `ROBOT_MOI_KGM2`, `WHEEL_INERTIA_KGM2`, `MOTOR_R_PHASE_OHM`, `BATT_NOMINAL_V`, `BATT_R_INTERNAL_OHM`, `EARTH_FIELD_UT`, `EARTH_ANGLE_RAD`, `HARD_IRON_X/Y_UT`, `MAG_HF_TONE_*`, `SIM_*` model fudge factors. | Brain firmware, host app, replay tool, and unit tests all **link this one header**. The app's `build.rs` additionally **parses it and generates the Rust constants** for `simulation.rs` at build time — there is no hand-synced copy anywhere; change the header, rebuild, done. |
| `sunshine_brain/include/config.h` | Pins, `BATT_ADC_SCALE`, SPI wiring — **unchanged on the same PCB**. Per-build: `MOTOR_LEFT_INVERT` / `MOTOR_RIGHT_INVERT` (Level 2 Step 5), `ESPNOW_CHANNEL`. | Brain firmware only. |
| `sunshine_receiver/include/config.h` | `BRAIN_MAC` (Level 3 Step 1). | Receiver firmware only. |

Cosmetic only: the KV label in `sunshine_app/src/components/DriverStation.tsx`. The sensor scale factors (`ADXL_SCALE_MS2`, `MAG_SCALE_UT`, battery encoding) also appear in `pipeline.rs`/`StatusBar.tsx`, but they are properties of the PCB's sensors, not of the robot build — leave them alone.

### How to obtain the plant/environment numbers

- `ROBOT_MASS_KG` — kitchen scale.
- `ROBOT_MOI_KGM2` — CAD (preferred), or bifilar-pendulum measurement.
- `WHEEL_INERTIA_KGM2` — CAD, or `½·m_wheel·r²` as a first cut (includes motor rotor).
- `MOTOR_R_PHASE_OHM` — motor datasheet, or multimeter across two phases ÷ 2.
- `HARD_IRON_X_UT` / `HARD_IRON_Y_UT` — from any Level 3+ log: mean of raw `inputs.mag_x` / `inputs.mag_y` while spinning, × 0.058 µT/count. (The heading filter kills hard-iron by construction; the sim needs it only to *generate* realistic mag data.)
- `EARTH_FIELD_UT` — horizontal field strength for your location (NOAA calculator). Horizontal component **only**, not total.

### What "fully brought up" means

After Level 5, all of these work for the new robot with no further code changes: live driving (TANK + MELTY with translation), telemetry + logging, **replay** (`tools/replay/` links the same core, so `replay`, `analyze.py`, `translation_lag.py`, `erpm_bandwidth.py`, `wheel_slip.py`, `spinup_lag.py` are all robot-agnostic or read the new constants), and **simulation** (its constants are generated from the same header at build time). The unit tests (`build/ctest`) assert physical invariants against whatever constants are compiled, with one exception: the `DRIFT_PHASE_LEAD_S` band in `test_control.c` encodes this robot's measured delay — after Level 5 Step 4, retarget that band to the new measurement.

---

## Level 1 — Low-level Sensors

**Goal:** All three sensors init and read correctly.  
**Setup:** Brain board powered via USB, **no ESCs connected**.  
**Firmware environment:** `bringup_1_sensors`

### Step 1: Build and flash

```bash
cd sunshine_brain
pio run --environment bringup_1_sensors
pio run --target upload --environment bringup_1_sensors
```

If the build fails with `MissingPackageManifestError`, apply the pioarduino workaround above.

### Step 2: Open serial monitor

```bash
pio device monitor --baud 921600
```

Expected output: a CSV header line followed by comma-separated values at ~100 Hz:

```
accel_x,accel_y,accel_z,mag_x,mag_y,mag_z,batt_v
-1,0,20,-342,512,-198,8.35
...
```

### Step 3: Verify each sensor

| Sensor | Field(s) | Expected at rest | Action to verify |
|--------|----------|-----------------|-----------------|
| ADXL375 | `accel_z` | ≈ +20 counts (1g / 0.049g per count) | Shake board → all axes spike |
| LIS3MDL | `mag_x,y,z` | magnitude ≈ 860 counts (50 µT / 0.058 µT/count) | Rotate board → `mag_x` and `mag_y` trace a circle |
| Battery ADC | `batt_v` | Matches multimeter reading ±0.1V | Verify with multimeter |

### Pass criteria

- No LED error blink pattern
- `accel_z` within ±5 counts of 20 at rest
- mag magnitude `sqrt(x²+y²+z²)` between 700 and 1000 counts
- `batt_v` within 0.1V of multimeter

**Troubleshooting:**

- 1 blink → ADXL375 init failed. Check FSPI wiring: SCK=IO12, MOSI=IO11, MISO=IO13, CS=IO10.
- 2 blinks → LIS3MDL init failed. Check HSPI wiring: SCK=IO16, MOSI=IO15, MISO=IO17, CS=IO18.
- `accel_z ≈ 0` → ADXL375 returning zero. Check SPI mode (should be SPI_MODE3) and full-resolution flag.
- `batt_v ≈ 0` → ADC not reading. `PIN_BATT_ADC = 7` (GPIO7, ADC1_CH7). Check `analogReadResolution(12)` is called in setup.
- `batt_v` noisy / jumpy while spinning → expected: the ESCs inject a strong ~50 Hz (2×-spin) tone. `batt_read_v()` low-passes it (single-pole IIR at `BATT_LP_HZ`, default 6 Hz in `config.h`). Raise `BATT_LP_HZ` for snappier sag response, lower for a smoother trace. As of schema v5 `batt_offset` is **int16 at 1 mV/LSB** (was int8 at 20.5 mV), so telemetry no longer discretises the battery — the 12-bit ADC (~2.4 mV) and the LP are the only quantisers left. Plot it as `var.batt_voltage`.

---

## Level 2 — DShot & ESC

**Goal:** Bidirectional DShot 600 working; eRPM telemetry readable.  
**Setup:** ESCs connected to IO4 (left) and IO5 (right). **Props removed.** AM32 pre-flashed in 3D mode with correct motor direction settings.  
**Firmware environment:** `bringup_2_dshot`

### Step 1: Pre-configure AM32 ESCs

Before connecting, use the AM32 configurator to set:
- Mode: 3D
- Motor direction: verify left and right motors spin in the correct directions when commanded forward

### Step 2: Build and flash

```bash
pio run --target upload --environment bringup_2_dshot
pio device monitor --baud 921600
```

Expected: `BRINGUP 2: DShot test. l <val>, r <val>, s, t`

### Step 3: Interactive test

The serial interface accepts single-line commands:

| Command | Action |
|---------|--------|
| `l 1200` | Left ESC: DShot value 1200 (forward) |
| `r 1200` | Right ESC: DShot value 1200 (forward) |
| `l 900` | Left ESC: DShot value 900 (reverse in 3D mode) |
| `s` | Stop both (sends neutral 1048) |
| `t` | Print eRPM for both ESCs |

AM32 3D mode DShot mapping:
- 48–1047: reverse (1047 = full reverse)
- 1048: neutral / brake
- 1049–2047: forward (2047 = full forward)

### Step 4: Verify eRPM

1. Send `l 1300`, wait 1 second, send `t`
2. Expected: `eRPM L=<value> R=0` where value > 0
3. Repeat several times — success rate should be > 90% (occasional 0 reads are acceptable)
4. Calculate expected eRPM: `KV × V_battery × pole_pairs`. For 1100 KV motor at 8V with 14 pole pairs: 1100 × 8 × 7 ≈ 61,600 eRPM at full throttle.

### Step 5: Set motor directions for CCW spin

**Goal:** In MELTY mode both motors spin "forward" (DShot > 1048) and the robot body rotates **counter-clockwise viewed from above**. This is the single correct spin direction.

The wheels are tangentially mounted, so both motors spinning "forward" produces body rotation. Which direction depends on physical motor mounting. Determine and fix it now.

#### 5a — Identify the correct direction per motor

With props off, secure the robot so it can't move (hand or clamp). Send equal forward commands to both motors and observe which way the body wants to rotate:

```
s          (neutral both)
l 1300
r 1300
```

- If the body torques **CCW** (viewed from above): correct, no inversion needed.
- If the body torques **CW**: both motors are backwards. You can either:
  - Swap motor leads on both ESCs (or swap any two of the three motor phases), **or**
  - Set `MOTOR_LEFT_INVERT = true` and `MOTOR_RIGHT_INVERT = true` in `sunshine_brain/include/config.h`.

If one motor spins the wrong way relative to the other, invert only that motor's flag.

#### 5b — Verify TANK mode translation

While still in bringup level 2 (or reflash level 3 and use the host app), command both motors for "TANK forward" — left forward + right reverse:

```
l 1300
r 800
```

The robot should push forward in the LED-defined direction. If it goes backward, your Y-axis is inverted — set `MOTOR_LEFT_INVERT` / `MOTOR_RIGHT_INVERT` accordingly so that the motion matches the control intent.

#### 5c — Software inversion vs AM32 configuration

Prefer AM32 motor direction (via the AM32 configurator) over the software flags — AM32 changes are persistent and work across all bringup levels without reflashing. Use the `MOTOR_LEFT_INVERT` / `MOTOR_RIGHT_INVERT` flags in `config.h` only when rewiring or AM32 reconfiguration isn't convenient.

### Pass criteria

- ESCs arm without beeping an error sequence
- Both directions spin when commanded
- eRPM telemetry success rate > 90% (`t` command rarely returns 0 while motor is spinning)
- Values are plausible given battery voltage
- Both motors commanded "forward" torques the body CCW (viewed from above)

---

## Level 3 — Telemetry Link

**Goal:** Full brain→receiver→host pipeline live in host app.  
**Setup:** Brain + receiver both powered. Host machine running the app. **Props off.**  
**Firmware environment:** `bringup_3_telemetry`

### Step 1: Set the brain MAC address in the receiver

The receiver needs the brain's WiFi STA MAC address hardcoded. To find it:

1. Flash any bringup level to the brain (level 1 is fine)
2. In brain's `main.cpp` setup, temporarily add:
   ```cpp
   WiFi.mode(WIFI_STA);
   Serial.println(WiFi.macAddress());
   ```
3. Read the MAC from serial monitor, e.g. `A4:CF:12:34:56:78`
4. Edit `sunshine_receiver/include/config.h`:
   ```cpp
   static const uint8_t BRAIN_MAC[6] = {0xA4, 0xCF, 0x12, 0x34, 0x56, 0x78};
   ```
5. Similarly, set the receiver MAC in `sunshine_brain/include/config.h` (read the receiver's MAC the same way)
6. Remove the temporary WiFi lines from brain's `main.cpp`

### Step 2: Flash receiver

```bash
cd sunshine_receiver
pio run --target upload --environment receiver
```

### Step 3: Flash brain

```bash
cd sunshine_brain
pio run --target upload --environment bringup_3_telemetry
```

Note: motor outputs are zeroed in this environment even if controls are sent.

### Step 4: Launch host app and connect

```bash
cd sunshine_app
pnpm tauri dev
```

In the app:
1. Go to the **Live** tab in ConnectionPanel
2. Select the receiver's serial port from the dropdown
3. Click **Connect**
4. Both status indicators (receiver + brain) should turn green within ~5 seconds

The receiver's onboard RGB LED tracks this independently of the app: **red** (no brain) → **amber** (brain up, host silent) → **green** with cyan flicker (full pipeline live). See the *Receiver Status LED Reference* below.

### Step 5: Verify the full pipeline

Open the graph panel and plot the following channels. Expected behavior at rest on a table:

| Channel | Expected |
|---------|----------|
| `inputs.accel_z` | Steady ≈ +20 counts. Shake board → spikes. |
| `inputs.mag_x` + `inputs.mag_y` | Non-zero, sinusoidal if you rotate the board |
| `inputs.rssi` | Plausible negative dBm (e.g. -55 to -80). Note: `RX_RSSI` packet reads -127 on IDF 4.4 — this is expected. |
| `inputs.ctrl_y` | Zero at rest. Press W key → ramps up smoothly (LP filter visible), releases → decays to zero. |

Check frame integrity: in the status bar, `frame_id` should increment without large gaps. Occasional missed frames are acceptable; continuous gaps indicate an ESP-NOW range or channel issue.

### Step 6: 2-minute stability test

Let the system run for 2 minutes without touching anything. Pass if:
- No `BRAIN_DISCONNECTED` status events appear
- `frame_id` gaps are rare (< 5% of frames)
- Log file created on connect, frame count incrementing in log status bar

### Pass criteria

- Both status indicators green
- Sensor data visible and plausible
- Control inputs visible when keys pressed
- Stable for 2 minutes

**Troubleshooting:**

- Brain shows as disconnected: check MAC addresses in both config files. Check ESP-NOW channel (both must be channel 1). Check receiver has power.
- `inputs.rssi` is -127: expected on IDF 4.4 (see platform notes at top of this file).
- Frame gaps > 20%: try moving receiver closer to brain. Check for 2.4 GHz interference (channel 1 overlaps WiFi channel 1).
- Host app doesn't see serial port: receiver may need `ARDUINO_USB_CDC_ON_BOOT=1` in its build flags (it should already be set).

---

## Level 4 — Navigation Tuning

**Goal:** Kalman filter tuned, LED appears stationary when spinning, TANK mode working.  
**Setup:** Full robot assembled, **props off** initially.  
**Firmware environment:** `bringup_4_navigation`

In this environment **TANK mode drives the motors** so you can spin the robot to tune the filters. DISABLED and MELTY keep the motors zeroed (MELTY isn't tuned until Level 5). The Kalman and magnetometer-heading filters run in all modes. **Props off.**

### Step 1: Flash and connect

```bash
cd sunshine_brain && pio run --target upload --environment bringup_4_navigation
```

Connect host app (same as Level 3). Plot `vars.est_omega` and `vars.omega_from_accel`.

### Step 2: Verify omega sensing

Spin the robot slowly by hand (without motors). In the graph:
- `omega_from_accel` should rise as you spin faster
- During spin-up, `omega_from_accel` reads high (tangential acceleration adds to centripetal magnitude — this is expected and normal, documented in `FILTER_MATH.md`)

### Step 3: Verify magnetometer heading

Spin the robot above the mag threshold (~480 RPM) by hand or with light motor power (with props off it's safe). Plot `vars.mag_x_filt` and `vars.mag_y_filt` — the band-passed Earth-field axes. They *oscillate* at the spin frequency (they're the rotating Earth sine), but their magnitude `sqrt(x²+y²)` should settle to a steady ~18–22 µT within ~1 s. If the magnitude is near zero, the spin is below threshold or `est_omega` is far off (so the spin frequency is outside the ±33% tracking band — fix Step 2 first). Also plot `vars.mag_angle` (the absolute heading) — it should rotate smoothly with the body.

**Do not carry a marginal magnetometer forward into Level 5.** The MELTY wheel-speed cap
takes its rate reference from the mag-driven `kf_omega`, so a mag that never locks holds
MELTY at the unlocked ceiling and the robot will not spin up at all (see Level 5, Step 2).
A mag that is *intermittent* is worse to diagnose than one that is dead: the spin will
stall and release at random. Fix it here.

### Step 4: Tune Kalman parameters

See `TUNING.md` for the full tuning procedure. Constants are in `sunshine_core/include/sunshine_core.h`. Change them and rebuild — no re-flash needed if you use `-D` build flags in `platformio.ini` for rapid iteration.

Summary of what each parameter controls:

| Parameter | Effect |
|-----------|--------|
| `KF_Q_OMEGA` | Higher → faster omega tracking, more noise |
| `KF_R_ACCEL` | Accel ω weight (always); higher → less influence from accelerometer |
| `KF_Q_THETA` | Higher → faster angle tracking, more drift |
| `KF_R_MAG` | Higher → less influence from magnetometer |
| `MAG_BP_FC_LP_HZ` | Band-pass centre LP (default 1.5 Hz). Lower = steadier `mag_angle` (less translation-induced wobble); higher = tracks spin-up faster. See TUNING.md. |
| `MAG_SPIN_RATE_LP_HZ` | LP on the mag rotation rate that drives the Kalman rate during translation (default 3.2 Hz). Higher = snappier impact/spin-up response, noisier steady heading; lower = smoother, laggier on fast spin changes. See TUNING.md. |

**Check for `mag_angle` wobble during translation/impacts.** Plot `var.omega_from_accel`
and `real.mag_angle` while driving in TANK and bumping the robot. If `mag_angle` gets
jittery when the accel rate spikes, lower `MAG_BP_FC_LP_HZ`. (The LED itself should
stay solid — it's mag-anchored; this only affects the band-passed heading cleanliness.)

### Step 5: LED check

While spinning above the mag threshold (~480 RPM), the LED should appear as a stationary dot (a few degrees of wobble from the soft-iron ellipse is fine — see FILTER_MATH.md). If it *sweeps/precesses* around, the heading is drifting — the open-loop mag (band-pass centred on `omega_from_accel`) should prevent that, so trust the mag more (`KF_R_MAG` down). Confirm `omega_from_accel` is sane: if it is wildly wrong the band-pass mis-centres and the heading attenuates.

**If the LED spins fast / heading is wildly wrong ONLY in one orientation**, it's the spin-direction sign. `omega_from_accel` is an unsigned magnitude (the accel can't sense CW vs CCW), so the sign is taken from the magnetometer (`state.spin_rate_lp`). When the robot is **inverted** it spins the opposite way in the world frame; the mag catches this and `kf_omega` should flip sign within ~50 ms. Test BOTH orientations here (spin it upright, then flip it and spin again) — the LED must be stationary in both. If it's broken only when inverted, check that `mag_valid` uses `|kf_omega|` and that `spin_rate_lp` is taking the mag's sign (schema v4 fix).

### Pass criteria

- LED appears stationary (±5°) at 500+ RPM
- `est_theta` tracks with < 5° RMS error visible in graph
- No drift observed over 30 seconds of constant-speed spinning
- `est_omega` tracks `omega_from_accel` closely during steady-state spin (diverges during spin-up — expected)

---

## Level 5 — Full MELTY (Drift Tuning)

**Goal:** Robot translates reliably in commanded direction while spinning.  
**Setup:** Open floor, **props on**, safe area, full clearance.  
**Firmware environment:** `production` (bringup level 0)

**Only proceed to this level after Level 4 passes completely.** If the LED is not stationary, fix that first.

### Step 1: Flash production firmware

```bash
cd sunshine_brain && pio run --target upload --environment production
```

### Step 2: Confirm LED is stationary

At low spin throttle, verify the LED appears stationary at a fixed heading before attempting any translation. This confirms the Kalman filter is working.

> **A dead magnetometer now means MELTY WILL NOT SPIN UP — deliberate, not a fault.**
> MELTY commands pass through the wheel-speed cap (`melty_speed_cap()` in `control.c`),
> which clamps the mean wheel command to the no-load speed the *measured* body rate
> demands plus a fixed slip allowance. Its rate reference is the mag-driven `kf_omega`,
> and with no mag there is never a lock, so the reference falls back to
> `SUNSHINE_MAG_MIN_OMEGA` and the command sits at the unlocked ceiling (≈1194 DShot at
> 8.2 V) forever: the robot stalls out near 50–60 rad/s however far the throttle is
> pushed. The trade is intended — a MELTY robot with no heading reference is
> undriveable anyway — but it converts "degraded but spinning" into "will not spin up",
> so **suspect the mag first** when MELTY won't spin up while TANK is fine (the cap is
> MELTY-only, which is exactly why TANK still spins). Plot `real.kf_omega` against
> `input.ctrl_throttle`: `kf_omega` flat near the threshold while the throttle is high
> is this cap — not a motor, battery or ESC fault. Confirm with `real.mag_x_filt` /
> `real.mag_y_filt` (their magnitude collapses when the mag is dead), then go back to
> Level 4 Step 3 and fix the magnetometer.

### Step 3: Test translation in MELTY mode

In the host app:
1. Set mode to **MELTY** (green button)
2. Bring throttle up slowly with arrow keys until the robot is spinning steadily. Do not start near full throttle: MELTY translation uses the symmetric DShot headroom above and below the spin command, so very high throttle intentionally leaves little translation authority.
3. Press W briefly → robot should drift forward
4. Press A/D → robot should drift left/right
5. The direction the robot drifts should match the driver's reference frame (LED-defined forward = W)

> **Test right-side up.** If the robot is inverted, the world-frame spin reverses and
> the whole driver frame mirrors, so "W" appears to drive *away* from the LED — this is
> expected, not a phase bug (check `input.accel_z`: ≈ −20 counts means inverted). Only
> tune `DRIFT_PHASE_OFFSET_RADS` for a direction error seen **upright**.
>
> **Translate at a moderate spin, not maximum.** Two effects weaken translation at high
> RPM: shrinking DShot headroom *and* motor/ESC bandwidth (the wheels can't change speed
> fast enough — measured in the logs, and it also rotates the effective direction with
> speed). Bring throttle up only until the LED is steady and spin is stable.

### Step 4: Measure the force direction (`DRIFT_PHASE_OFFSET_RADS` + `DRIFT_PHASE_LEAD_S`)

**Do this before hand-tuning anything, and do it ON THE FLOOR.** The force
direction has a constant part (geometry / sign conventions — a motor-config or
wiring change can silently flip it 180°) and a spin-proportional part (the force
lag). **Neither can be trusted from eRPM analysis**: `translation_lag.py`
cross-correlates command against eRPM, which is wheel **speed** — speed is the
integral of torque and lags ~2× the force. On this robot that mistake set the
lead to 18 ms when the force lag is ~10 ms, rotating the force backwards by
`omega * 8 ms`, and it *self-conceals* (the eRPM check validates the wrong lead).
The tires ride the cap's slip bias so they are always kinetically sliding:
force is `mu*N*sign(slip)`, and its timing is set by the slip-**sign**
crossings, not the speed wave.

**The two-speed drift-direction test** (the only measurement that sees force):

1. Flat open floor. Spin at ~30% throttle, LED steady. Give short **taps** of W
   and note the drift direction relative to the LED as a clock position
   (LED = 12 o'clock). Repeat until confident.
2. Repeat at ~50% throttle. You now have two direction errors at two known spin
   rates (read `kf_omega` from the log the app saved).
3. The **constant** part of the error (extrapolated to zero spin) is
   `DRIFT_PHASE_OFFSET_RADS` (sign: positive offset moves the observed drift
   clockwise for a CCW-spinning robot). The **slope** (degrees per rad/s) is the
   lead error: `true_lead = compiled_lead − slope_in_rad_per_rad/s`.
4. **Pin the constant part with geometry before fitting.** The wheels push
   along the wheel-line tangent (90° from the wheel axis), and the driver
   convention W = "toward the light" is encoded as `dd = +90°` — so if the LED
   sits ON the wheel axis (this build), the stack-up is **exactly 0 or π**,
   nothing in between, and the drift test just resolves the binary and the
   lead: with the offset pinned, the two speeds must agree on a single lead
   (that agreement is your cross-check). If the LED sits elsewhere, its body
   angle relative to the wheel axis enters the constant — derive it, then
   verify. `tools/melty_sim.py` (stick-slip tire + motor model + the real
   control law) is available to sanity-check the fit against the robot's
   physics. Update the "PHASE LEAD" band in `test_control.c`, rebuild, reflash,
   re-run the taps: drift should land near 12 o'clock at both speeds.

Reference: on this robot the 2026-08-07 measurement was +90° at ω≈120 and +45°
at ω≈165 with lead 0.018/offset 0 → offset pinned at **π** (LED on the wheel
axis, "+diff" wheel side) → both speeds agree on a force lag of ~4–5 ms →
`DRIFT_PHASE_OFFSET_RADS = 3.14159265f`, `DRIFT_PHASE_LEAD_S = 0.005f`.
`translation_lag.py` remains useful only as a **speed-lag** measurement
(ESC/motor health, upper bound on force lag) — never set the lead from it.

### Step 5: Tune drift parameters

See `TUNING.md` for the full drift tuning procedure. Constants are in `sunshine_core/include/sunshine_core.h`:

| Parameter | Effect |
|-----------|--------|
| `DRIFT_AMPLITUDE` | Sets the wheel-speed swing: the translation **top-speed ceiling** and the wheel-rotor gyroscopic tilt kick — NOT low-speed force, which saturates at tire friction within a modest swing. Keep moderate; raise only for top speed, and only after phase is verified (a misaimed force "needs" huge amplitude, which is the tilt/strike spiral). |
| `DRIFT_PLATEAU_WIDTH` | Fraction of each rotation spent at each +1/-1 differential plateau. Higher is more rectangle-like; lower gives wider ramps. |
| `DRIFT_PHASE_OFFSET_RADS` | Fixed geometry/sign offset between the LED/driver heading and the wheel-force direction, **measured on-floor in Step 4**. Re-measure after ANY motor wiring, mounting, or `MOTOR_*_INVERT` change — a flip shows up here as ±180°. |
| `DRIFT_PHASE_LEAD_S` | Spin-proportional force-lag compensation, **measured on-floor in Step 4** (slip-sign lag, ~half the eRPM speed lag). Added phase is `kf_omega * DRIFT_PHASE_LEAD_S`. |
| `THETA_RATE_RADS` | How fast the driver heading reference rotates with left/right arrow. This rotates the LED reference, not motor timing. |

Tune in this order:

1. With Step 4's measured `DRIFT_PHASE_LEAD_S` in place, drive at a moderate spin throttle where the robot is stable but not near max.
2. If W still produces a consistent sideways/backwards drift **that is the same angle at different spin speeds**, adjust `DRIFT_PHASE_OFFSET_RADS` in 15-30 degree steps (`0.26f` to `0.52f` rad). Positive values advance the motor waveform in the code's CCW-positive phase convention; if the correction gets worse, use the opposite sign. (A direction error that *grows with spin speed* means the lead is off instead — re-measure Step 4, or nudge `DRIFT_PHASE_LEAD_S` by 1-2 ms: at 240 rad/s, `0.001f` is ~14° of phase.)
3. After direction is repeatable, raise or lower `DRIFT_AMPLITUDE`. If the robot still barely moves but the direction is correct, increase it. If spin speed collapses or it chatters, decrease it.
4. Adjust `DRIFT_PLATEAU_WIDTH` only after amplitude/phase are sane. Higher values dwell longer at max command and approach a rectangle; lower values widen the ramps and are gentler for the ESC/wheel.
5. After tuning, repeat the Step 4 two-speed taps — drift should stay near 12
   o'clock at both speeds. If you changed wheels/motors/wiring mid-tune, re-do
   Step 4 in full (the constant offset is exactly what such changes move).

### Step 6: The anti-tip swing clamp (why melties tip, sim-proven, per-robot)

**Mechanism** (established with the coupled 6-DOF simulation,
`tools/melty6dof.py`, which reproduces this robot's measured instant-tip from
pure contact physics): the tip driver is the **wheel-rotor gyroscopic
reaction**. The drift wave modulates the wheels — flywheels whose axles the
spinning chassis carries around — and the reaction is a world-frame tilting
moment ~ `I_w · ω · Δω_wheel`. It is NOT a motor-torque limit (more ESC current
makes it worse, not better), and it is phase-blind (in-sim, correcting the
phase constants alone changed nothing, while the same config with 13× lighter
wheels did not tip at all *and* translated 50% faster).

The sim-mapped safe envelope: the commanded no-load wheel-speed swing may not
exceed **ratio(ω) × body rate**, where the safe ratio *rises* with spin
(gyroscopic stiffness): `TIP_SWING_RATIO_LO` below `TIP_OMEGA_LO` ramping to
`TIP_SWING_RATIO_HI` above `TIP_OMEGA_HI`. Grid-validated on this build (ω
100–220, throttle 100–190, multiple roughness seeds): worst tilt 4.9° against
the 5.5° edge budget, zero edge contact. Translate at ω ≥ ~150 for the widest
envelope — high spin is the SAFE regime.

**Per-robot procedure:**
1. `WHEEL_INERTIA_KGM2` must be honest (CAD, including motor rotor/bell; a
   10 g rim at 22 mm is 4.8e-6 kg·m² by itself — rim mass dominates).
2. Update the physical constants in `tools/melty6dof.py` (they mirror this
   header) and re-run its calibration + grid; retune `TIP_SWING_RATIO_*` /
   `TIP_OMEGA_*` to the new no-tip frontier.
3. Speed levers, all confirmed in-sim: **lighter wheel/rotor assemblies**
   (directly shrinks the tip moment — the single biggest win), **grippier tire
   compound** (grip damps the tilt wobble AND drives harder: µ 0.12→0.5
   roughly doubled translation speed while *reducing* tilt), wider track /
   more mass (stance), higher spin while translating.

### Pass criteria

- Robot translates in the commanded direction at ≥ 3 of 4 compass points (N/S/E/W)
- No wheel slip causing uncontrolled spin-out
- LED remains stationary during translation inputs
- The inside wheel stays loaded during full translation (no edge ride / grinding)
- Robot can be steered to a target location reliably

---

## Receiver Status LED Reference

The receiver's onboard RGB LED (WS2812 on GPIO48 of the ESP32-S3-DevKitC-1) shows liveness and link state at a glance. All states use a slow "breathing" pulse so a steady, non-pulsing LED means the firmware has hung. Override the pin with `-DSTATUS_LED_PIN=<gpio>` in the receiver's `build_flags` if your board wires the LED elsewhere (some early revisions use GPIO38).

| LED | Meaning |
|-----|---------|
| Dim **white** breathe | Booting / idle — powered up, nothing connected yet |
| **Red** blink (fast, 150 ms) | Fatal error — ESP-NOW init failed (firmware halted) |
| **Red** breathe | Alive, but no brain telemetry arriving (brain off, out of range, or wrong MAC/channel) |
| **Amber** breathe | Brain link up, but the host app is silent (>1.5 s) — control is disabled / safe |
| **Green** breathe | Brain + host both live; a brighter **cyan flash** pulses on each telemetry frame forwarded to the host |

Quick bringup checks:
- Plug in the receiver with nothing else on → expect **red breathe** (waiting for brain).
- Power the brain → LED goes **amber** within ~200 ms once frames arrive.
- Connect the host app (Level 3, Step 4) → LED goes **green** and flickers cyan as telemetry flows. This mirrors the two green status indicators in the app.

## Reconnect Behaviour Reference

| Event | Effect |
|-------|--------|
| Receiver USB unplugged | Brain watchdog fires in 500ms → DISABLED. Host shows disconnected. |
| Receiver USB replugged | Receiver sends BRAIN_CONNECTED if brain frames arriving. Host opens new log file. |
| Brain loses power | Receiver: 10 missed frames (200ms) → BRAIN_DISCONNECTED. Host shows disconnected. |
| Brain reconnects | ESP-NOW MAC is hardcoded — no handshake needed. Telemetry resumes automatically. |
| Host app silent > 3s | Receiver watchdog forces DISABLED. Robot stops even if brain still running. |
