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
- `batt_v ≈ 0` → ADC not reading. `PIN_BATT_ADC = 39`. Check `analogReadResolution(12)` is called in setup.

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

In this environment **TANK mode drives the motors** so you can spin the robot to tune the filters. DISABLED and MELTY keep the motors zeroed (MELTY isn't tuned until Level 5). The Kalman and derotation filters run in all modes. **Props off.**

### Step 1: Flash and connect

```bash
cd sunshine_brain && pio run --target upload --environment bringup_4_navigation
```

Connect host app (same as Level 3). Plot `vars.est_omega` and `vars.omega_from_accel`.

### Step 2: Verify omega sensing

Spin the robot slowly by hand (without motors). In the graph:
- `omega_from_accel` should rise as you spin faster
- During spin-up, `omega_from_accel` reads high (tangential acceleration adds to centripetal magnitude — this is expected and normal, documented in `FILTER_MATH.md`)

### Step 3: Verify magnetometer derotation

Spin the robot at > 300 RPM by hand or with light motor power (with props off it's safe). Plot `vars.derot_I` and `vars.derot_Q`. They should be near-constant (DC values). If they oscillate, the derotation filter needs time to settle — wait ~3 seconds after reaching speed.

### Step 4: Tune Kalman parameters

See `TUNING.md` for the full tuning procedure. Constants are in `sunshine_core/include/sunshine_core.h`. Change them and rebuild — no re-flash needed if you use `-D` build flags in `platformio.ini` for rapid iteration.

Summary of what each parameter controls:

| Parameter | Effect |
|-----------|--------|
| `KF_Q_OMEGA` | Higher → faster omega tracking, more noise |
| `KF_R_ACCEL` | Higher → less influence from accelerometer |
| `KF_Q_THETA` | Higher → faster angle tracking, more drift |
| `KF_R_MAG` | Higher → less influence from magnetometer |

### Step 5: LED check

While spinning at > 300 RPM (above the 120 RPM mag threshold), the LED should appear as a stationary dot at a fixed heading. If it sweeps around, the theta estimate is drifting — tune `KF_R_MAG` down or `KF_Q_THETA` down.

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

### Step 3: Test translation in MELTY mode

In the host app:
1. Set mode to **MELTY** (green button)
2. Bring throttle up slowly with arrow keys until the robot is spinning steadily
3. Press W briefly → robot should drift forward
4. Press A/D → robot should drift left/right
5. The direction the robot drifts should match the driver's reference frame (LED-defined forward = W)

### Step 4: Tune drift parameters

See `TUNING.md` for the full drift tuning procedure. Constants are in `sunshine_core/include/sunshine_core.h`:

| Parameter | Effect |
|-----------|--------|
| `DRIFT_AMPLITUDE` | Overall translation strength. Increase if robot barely moves. |
| `DRIFT_PULSE_WIDTH` | Fraction of rotation at peak differential. Wider = more consistent push. |
| `DRIFT_RAMP_WIDTH` | Smoothness of the transition. Wider = smoother, less abrupt. |
| `THETA_RATE_RADS` | How fast the heading reference rotates with left/right arrow. |

### Pass criteria

- Robot translates in the commanded direction at ≥ 3 of 4 compass points (N/S/E/W)
- No wheel slip causing uncontrolled spin-out
- LED remains stationary during translation inputs
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
