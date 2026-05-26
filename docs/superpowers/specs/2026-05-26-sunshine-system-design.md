# Sunshine Combat Robot — System Design Spec
**Date:** 2026-05-26  
**Status:** Approved for implementation planning  
**Schema version:** 1 (initial)

---

## Table of Contents
1. [Overall Architecture & Module Boundaries](#1-overall-architecture--module-boundaries)
2. [IO Layer Data Model](#2-io-layer-data-model)
3. [Brain Firmware](#3-brain-firmware)
4. [Receiver Firmware & Telemetry Protocol](#4-receiver-firmware--telemetry-protocol)
5. [Log File Format](#5-log-file-format)
6. [Host App Architecture](#6-host-app-architecture)
7. [Bringup Plan](#7-bringup-plan)
8. [Deliverable Documents](#8-deliverable-documents)

---

## 1. Overall Architecture & Module Boundaries

### Repository Layout

```
sunshine/
├── sunshine_core/          # Pure C library — NO platform dependencies
│   ├── include/
│   │   └── sunshine_core.h # Public API + all struct definitions
│   ├── src/
│   │   ├── kalman.c        # 2-state Kalman filter (θ, ω)
│   │   ├── derot_filter.c  # Synchronous demodulation + 4th-order LP
│   │   ├── control.c       # DISABLED / TANK / MELTY control logic
│   │   └── brain.c         # sunshine_step() — top-level entry point
│   └── CMakeLists.txt      # Used by PlatformIO; cc crate uses source list directly
├── sunshine_brain/         # PlatformIO ESP32-S3 project
│   ├── include/
│   │   ├── bringup.h       # BRINGUP_LEVEL define + feature flags
│   │   └── config.h        # MAC addresses, pin definitions, tuning constants
│   ├── src/
│   │   ├── main.cpp
│   │   ├── sensors/        # ADXL375 + LIS3MDL SPI drivers
│   │   ├── dshot.cpp       # Bidirectional DShot 600 (DShotRMT_NEO)
│   │   ├── telemetry.cpp   # ESP-NOW TX/RX (Core 0)
│   │   └── nav_control.cpp # 1kHz loop (Core 1)
│   └── platformio.ini      # Multiple bringup environments + production
├── sunshine_receiver/      # PlatformIO ESP32-S3 project
│   ├── include/
│   │   └── config.h        # Brain MAC address, pin definitions
│   ├── src/
│   │   ├── main.cpp
│   │   ├── espnow_rx.cpp
│   │   └── usb_bridge.cpp
│   └── platformio.ini
├── sunshine_app/           # Tauri + React host app
│   ├── src-tauri/
│   │   ├── build.rs        # Compiles sunshine_core via cc crate
│   │   ├── src/
│   │   │   ├── main.rs
│   │   │   ├── ffi.rs          # sunshine_core C bindings + safe wrappers
│   │   │   ├── serial.rs       # USB serial port management
│   │   │   ├── protocol.rs     # USB frame encode/decode
│   │   │   ├── pipeline.rs     # Central data pipeline + ring buffer
│   │   │   ├── replay.rs       # Log file reader + replay engine
│   │   │   ├── simulation.rs   # Brushed DC motor + robot kinematics sim
│   │   │   ├── logging.rs      # .sun file writer
│   │   │   ├── controls.rs     # Latest control state management
│   │   │   └── commands.rs     # All Tauri command handlers
│   │   └── Cargo.toml
│   └── src/                # React frontend
│       ├── components/
│       │   ├── Header.tsx          # Mode buttons + status bar + logging control
│       │   ├── ConnectionPanel.tsx # Live / Replay / Simulation tabs
│       │   ├── GraphPanel.tsx      # uPlot canvas + channel selector
│       │   └── ...
│       └── ...
└── docs/
    ├── ARCHITECTURE.md     # Codebase map for future AI sessions
    ├── BRINGUP.md          # Step-by-step bringup instructions (human-readable)
    ├── TUNING.md           # Kalman + drift tuning guide for humans
    ├── FILTER_MATH.md      # Plain-language explanation of full filter chain
    ├── DEBUGGING.md        # Instructions for sunshine:replay-debug skill
    └── superpowers/specs/  # This document
```

### Data Flow

```
LIVE:       Brain sensors → sunshine_core (on ESP32) → ESP-NOW → Receiver → USB serial → Tauri → React
REPLAY:     .sun file → Tauri (replay.rs) → sunshine_core (FFI) → React
SIMULATION: Tauri (simulation.rs physics) → sunshine_core (FFI) → React
```

### Key Architectural Rules

1. **`sunshine_core` is a pure C island.** No ESP-IDF, no FreeRTOS, no OS calls. Only C99 standard library (math.h, stdint.h, stdbool.h). Compiles identically on ESP32 and desktop.
2. **State is explicit.** No global mutable state above the IO layer. Everything lives in `SunshineState`, passed into and out of `sunshine_step()`.
3. **The IO layer is the contract.** Nothing crosses it except through `SunshineInput` and `SunshineState`.
4. **Safety is structural.** `mode == DISABLED` zeroes DShot outputs unconditionally inside `sunshine_step()`, regardless of any other field. It cannot be bypassed.
5. **New fields are append-only.** Fields may only be added to the end of `SunshineInput`, `SunshineState`, or `SunshineVars`. Never insert, reorder, or resize existing fields. Bump `SUNSHINE_SCHEMA_VERSION` on every change with a comment.
6. **Simulation stays in Rust.** The physics model is only ever called from Tauri. No C simulation code.

---

## 2. IO Layer Data Model

### `SunshineInput` — 1kHz sensor frame (29 bytes packed)

```c
typedef struct __attribute__((packed)) {
    uint32_t time_us;        // Microseconds since boot
    int16_t  accel_x;        // ADXL375 raw counts (49 mg/LSB, ±200g)
                             // IMU mounted 45° to radial — centripetal splits
                             // equally across accel_x and accel_y
    int16_t  accel_y;        // See accel_x. Tangential accel also present during spinup.
    int16_t  accel_z;        // Vertical axis (~+20 counts = 1g at rest)
    int16_t  mag_x;          // LIS3MDL raw counts at ±16 Gauss (0.058 μT/LSB)
    int16_t  mag_y;
    int16_t  mag_z;
    uint16_t erpm_left;      // IEEE 754 float16 bits — left wheel eRPM
    uint16_t erpm_right;     // IEEE 754 float16 bits — right wheel eRPM
    int8_t   rssi;           // ESP-NOW RSSI at brain when receiving control packets (dBm)
    int8_t   ctrl_x;         // Driver input [-127, 127]  (A/D keys)
    int8_t   ctrl_y;         // Driver input [-127, 127]  (W/S keys)
    int8_t   ctrl_theta;     // Driver input [-127, 127]  (left/right arrow, rate)
    uint8_t  ctrl_throttle;  // Driver input [0, 255]     (up/down arrow, accumulates)
    int8_t   batt_offset;    // Battery voltage relative to 7.6V (0.0205 V/LSB, range 5.0–10.2V)
    uint8_t  dshot_left_q;   // DShot command applied on previous tick, quantized [0, 255]
    uint8_t  dshot_right_q;  // Same for right ESC
    uint8_t  mode;           // SUNSHINE_MODE_DISABLED=0, TANK=1, MELTY=2
} SunshineInput;             // sizeof = 29 bytes
```

**Notes:**
- `dshot_left_q`/`dshot_right_q` hold the command from the *previous* tick (what produced this tick's sensor readings). Zero on tick 0.
- At 4000 RPM, centripetal = 196.7g. Across the 45° mount: 139g per axis, well within ±200g. Effective saturation threshold: ~283g ≈ 4800 RPM (above motor max).
- `accel_x`/`accel_y` also carry tangential acceleration during angular acceleration events: `ax = (a_centripetal - a_tangential)/√2`, `ay = (a_centripetal + a_tangential)/√2`.

**Scale constants (in `sunshine_core.h`):**
```c
#define ADXL_SCALE_MS2     (49e-3f * 9.81f)   // m/s² per count
#define MAG_SCALE_UT       0.058f              // μT per count
#define BATT_SCALE_V       0.0205f             // V per count
#define BATT_OFFSET_REF_V  7.6f               // reference voltage
#define IMU_RADIUS_M       0.011f              // 11mm from center
```

---

### `SunshineState` — Filter history (44 bytes packed)

Everything with historical context. Initial conditions needed to deterministically reproduce all future outputs.

```c
typedef struct __attribute__((packed)) {
    // Kalman filter — 2-state [θ (rad), ω (rad/s)]
    float kf_theta;          // Estimated absolute angle (radians, unwrapped)
    float kf_omega;          // Estimated angular velocity (rad/s)
    float kf_P[4];           // 2×2 covariance matrix, row-major [P00,P01,P10,P11]

    // Driver heading offset (accumulated from ctrl_theta)
    float theta_offset;      // rad — rotates the driver's zero-heading reference

    // Synchronous demodulation LP filter states
    // 4th-order Butterworth LP, fc=1 Hz, fs=1kHz, implemented as 2 cascaded biquads
    // State layout per component: [s1_w0, s1_w1, s2_w0, s2_w1]
    float derot_lp_I[4];     // LP state for derotated I component of mag
    float derot_lp_Q[4];     // LP state for derotated Q component of mag
} SunshineState;             // sizeof = 60 bytes
                             // 4+4+16+4+16+16 = 60
                             // (kf_theta + kf_omega + kf_P[4] + theta_offset + derot_lp_I[4] + derot_lp_Q[4])
```

---

### `SunshineVars` — Derived variables (52 bytes, computed each step, never telemetered)

```c
typedef struct __attribute__((packed)) {
    float  omega_from_accel;    // rad/s — sqrt(sqrt(ax²+ay²)*ADXL_SCALE/IMU_RADIUS)
                                //   NOTE: inflated during spinup due to tangential accel
    float  derot_I;             // Derotated + LP-filtered mag I component (μT)
    float  derot_Q;             // Derotated + LP-filtered mag Q component (μT)
    float  mag_angle;           // atan2(derot_Q, derot_I), rad — Earth-field angle
    float  est_theta;           // = kf_theta (rad) — plottable angle estimate
    float  est_omega;           // = kf_omega (rad/s) — plottable omega estimate
    float  dshot_cmd_left;      // Computed DShot value [0, 2000], full precision
    float  dshot_cmd_right;
    float  batt_voltage;        // Actual voltage (V)
    float  erpm_left;           // Decoded from float16
    float  erpm_right;
    float  centripetal_ms2;     // sqrt(ax²+ay²) * ADXL_SCALE, m/s²
    uint8_t led_on;             // 1 when robot angle within ±3° of zero heading
    uint8_t accel_saturated;    // 1 when centripetal_ms2 > 280g equivalent
    uint8_t mag_valid;          // 1 when est_omega > SUNSHINE_MAG_MIN_OMEGA
    uint8_t loop_overrun;       // 1 when 1kHz tick exceeded 1000μs (hardware only)
} SunshineVars;                 // sizeof = 52 bytes
```

---

### C API

```c
// Lifecycle
void     sunshine_state_init(SunshineState *state);
    // Zero-initialises state, sets kf_P to high-uncertainty diagonal

// Main step — call at 1kHz
void     sunshine_step(const SunshineInput *in, SunshineState *state, SunshineVars *vars_out);
    // Pure except for *state mutation. Identical inputs + state → identical output.

// Serialisation (log files use sizeof() directly — structs are packed, no padding)
void     sunshine_input_serialize  (const SunshineInput *in,    uint8_t *buf);
void     sunshine_input_deserialize(const uint8_t *buf,         SunshineInput *in);
void     sunshine_state_serialize  (const SunshineState *state, uint8_t *buf);
void     sunshine_state_deserialize(const uint8_t *buf,         SunshineState *state);

// Schema
uint32_t sunshine_schema_version(void);   // Current: 1. Bump on every struct change.

// Unit conversion utilities
float    sunshine_accel_to_ms2(int16_t raw);    // raw * ADXL_SCALE_MS2
float    sunshine_mag_to_ut   (int16_t raw);    // raw * MAG_SCALE_UT
float    sunshine_batt_to_v   (int8_t  off);    // BATT_OFFSET_REF_V + off * BATT_SCALE_V
float    sunshine_f16_to_f32  (uint16_t half);
uint16_t sunshine_f32_to_f16  (float f);
```

---

### Schema Versioning

Log file header stores `schema_version`, `sizeof_input`, `sizeof_state`, `sizeof_vars`. When reading old files:
- `sizeof < current`: missing trailing fields default to zero/false
- `sizeof > current`: extra bytes skipped with `fseek`

No converter functions needed — the append-only rule handles it.

---

## 3. Brain Firmware

### Core Assignment

```
Core 0 — Telemetry task (FreeRTOS, priority 5)
  • ESP-NOW TX: 50 Hz state+inputs frames to receiver
  • ESP-NOW RX callback: stores latest control inputs + RSSI behind a mutex
  • Drains ring buffer produced by Core 1

Core 1 — Nav+Control task (FreeRTOS, priority 10, pinned)
  • 1kHz hard real-time loop
  • Pushes (SunshineInput, SunshineState) snapshots to ring buffer
  • Never blocks on telemetry
```

Ring buffer: 40 entries × (29+60) bytes = 3.6 KB. Absorbs 40ms of Core 0 scheduling jitter. Oldest entry overwritten if full — Core 1 never blocks.

### SPI Bus Assignment

| Bus | Peripheral | SCK | MOSI | MISO | CS |
|-----|-----------|-----|------|------|----|
| `FSPI` | ADXL375 (Adafruit library) | IO12 | IO11 | IO13 | IO10 |
| `HSPI` | LIS3MDL (Adafruit library) | IO16 | IO15 | IO17 | IO18 |

- ADXL375: ODR 1600 Hz, full-resolution mode, polled (no interrupt needed at 1kHz poll rate)
- LIS3MDL: ODR 1000 Hz (`LIS3MDL_DATARATE_1000_HZ`), ±16 Gauss, continuous conversion
- DShot: IO4 (left), IO5 (right) — bidirectional DShot 600 via DShotRMT_NEO, pull-ups on PCB
- Battery ADC: IO39, `V_bat = analogRead(39) * (3.3/4095.0) * 3.0`

### 1kHz Loop Structure

```
Every 1000 μs (enforced by busy-wait on micros()):

  t_start = micros()

  1. read_sensors()          → fill accel, mag, batt_offset, erpm fields in input
  2. copy_control_inputs()   → acquire mutex, copy ctrl_*, mode, rssi
  3. carry_prev_dshot()      → input.dshot_left_q  = quantize(prev_vars.dshot_cmd_left)
                               input.dshot_right_q = quantize(prev_vars.dshot_cmd_right)
  4. input.time_us = t_start

  5. sunshine_step(&input, &state, &vars)

  6. apply_outputs()
       send_dshot(LEFT,  vars.dshot_cmd_left)
       send_dshot(RIGHT, vars.dshot_cmd_right)
       digitalWrite(LED_PIN, vars.led_on)

  7. push_to_ring_buffer(&input, &state)

  8. elapsed = micros() - t_start
     if elapsed > 1000: overrun_count++; Serial.printf("OVERRUN %uus\n", elapsed)
     else: spin until t_start + 1000
```

### Kalman Filter

**State:** `x = [θ, ω]ᵀ`, dt = 0.001 s

**Predict (every tick):**
```
F = [[1, dt], [0, 1]]
x = F·x
P = F·P·Fᵀ + Q
```

**Update — accelerometer (every tick, skip if accel_saturated):**
```
centripetal = sqrt(accel_x² + accel_y²) * ADXL_SCALE_MS2
omega_meas  = sqrt(centripetal / IMU_RADIUS_M)
H = [0, 1];  innovation = omega_meas − H·x
S = H·P·Hᵀ + R_ACCEL;  K = P·Hᵀ / S
x += K·innovation;  P = (I − K·H)·P
```

**Update — magnetometer (only when mag_valid: est_omega > 4π rad/s ≈ 120 RPM):**
```
theta_meas = vars->mag_angle    (from synchronous demodulation output)
H = [1, 0];  innovation = wrap_to_pi(theta_meas − H·x)
S = H·P·Hᵀ + R_MAG;  K = P·Hᵀ / S
x += K·innovation;  P = (I − K·H)·P
```

**Tuning constants (in `sunshine_core.h`, documented for human tuning):**
```c
#define KF_Q_THETA   1e-6f   // Process noise: angle (rad²/step) — increase if angle drifts
#define KF_Q_OMEGA   1e-3f   // Process noise: omega (rad²/s²/step) — increase for faster tracking
#define KF_R_ACCEL   0.5f    // Accel noise (rad²/s²) — increase to reduce accel influence
#define KF_R_MAG     0.1f    // Mag noise (rad²) — increase to reduce mag influence
```

### Synchronous Demodulation (Magnetometer Filter)

Earth's field oscillates at body rotation frequency in the IMU frame. Derotating by the current Kalman theta shifts it to DC; a fixed LP filter then rejects motor/ESC offsets (which appear at f_rot in the derotated frame).

```c
// Derotate using current theta estimate
float theta = state->kf_theta + state->theta_offset;
float I_raw = input->mag_x * cosf(theta) + input->mag_y * sinf(theta);
float Q_raw = -input->mag_x * sinf(theta) + input->mag_y * cosf(theta);

// Apply 4th-order Butterworth LP (two cascaded biquad sections), fc=1 Hz, fs=1kHz
float I_filt = biquad_cascade(I_raw, state->derot_lp_I);
float Q_filt = biquad_cascade(Q_raw, state->derot_lp_Q);

vars->derot_I   = I_filt;
vars->derot_Q   = Q_filt;
vars->mag_angle = atan2f(Q_filt, I_filt);
```

**4th-order Butterworth LP coefficients** (fc=1 Hz, fs=1000 Hz, two cascaded Direct Form II biquads):
```c
// Section 1
#define LP4_S1_B0  9.886e-7f
#define LP4_S1_B1  1.977e-6f
#define LP4_S1_B2  9.886e-7f
#define LP4_S1_A1 -1.99112f
#define LP4_S1_A2  0.99115f
// Section 2
#define LP4_S2_B0  9.886e-7f
#define LP4_S2_B1  1.977e-6f
#define LP4_S2_B2  9.886e-7f
#define LP4_S2_A1 -1.99679f
#define LP4_S2_A2  0.99682f
```

**Minimum usable speed:** 120 RPM (4π rad/s). At this speed the motor offsets (at f_rot = 2 Hz) are attenuated ~40 dB below the Earth's field signal. Below this threshold Kalman angle update is skipped; filter continues running to maintain state.

**Effect during spinup:** `omega_from_accel` reads high (tangential acceleration adds to centripetal magnitude). Visible in graphs as divergence between `omega_from_accel` and `est_omega` during spin-up transients — this is expected and useful for filter tuning.

### Control Modes

**DISABLED:** `sunshine_step()` unconditionally sets `dshot_cmd_left = dshot_cmd_right = 0`. Structural — not a conditional check.

**TANK:**
```c
float fwd  = (ctrl_throttle / 127.5f) - 1.0f;    // −1..+1
float turn = ctrl_x / 127.0f;
dshot_cmd_left  = map_to_dshot(clampf(fwd + turn, -1, 1));
dshot_cmd_right = map_to_dshot(clampf(fwd - turn, -1, 1));
// map_to_dshot: ESC-specific, AM32 3D mode, finalised at Bringup Level 2
```

**MELTY:**
```c
// Heading offset (rate control)
state->theta_offset += (ctrl_theta / 127.0f) * THETA_RATE_RADS * dt;

// Base spin throttle (open-loop)
float base = (ctrl_throttle / 255.0f) * MAX_DSHOT_SPIN;

// Trapezoidal translation wave
float drive_dir = atan2f(ctrl_y, ctrl_x);
float drive_mag = sqrtf(ctrl_x*ctrl_x + ctrl_y*ctrl_y) / 127.0f;
float phase     = wrap_to_pi(state->kf_theta + state->theta_offset - drive_dir);
float diff      = trapezoid(phase, DRIFT_PULSE_WIDTH, DRIFT_RAMP_WIDTH)
                  * drive_mag * DRIFT_AMPLITUDE * base;

dshot_cmd_left  = clampf(base + diff, 0, MAX_DSHOT);
dshot_cmd_right = clampf(base - diff, 0, MAX_DSHOT);
```

**LED:** On when `|wrap_to_pi(kf_theta + theta_offset)| < 0.0524 rad` (±3°).

**Tuning constants:**
```c
#define DRIFT_PULSE_WIDTH   0.25f   // fraction of rotation at peak differential
#define DRIFT_RAMP_WIDTH    0.10f   // fraction for linear ramp transition
#define DRIFT_AMPLITUDE     0.40f   // max differential as fraction of base throttle
#define THETA_RATE_RADS     3.14f   // rad/s per full ctrl_theta deflection
#define MAX_DSHOT_SPIN      1500.0f // max DShot value used for body spin
```

### Error Handling & LED Patterns

On startup, each sensor `begin()` is checked. On failure:

| Blink count | Fault |
|-------------|-------|
| 1 | ADXL375 init failed |
| 2 | LIS3MDL init failed |
| 3 | DShot arming failed |
| 4+ | Multiple failures |

Pattern: N fast blinks (50ms on/off) → 1s off → repeat. Error also printed to USB serial at 1 Hz continuously — plug in USB any time to see the fault.

### Bringup Environments (platformio.ini)

```ini
[env:bringup_1_sensors]     build_flags = -DBRINGUP_LEVEL=1
[env:bringup_2_dshot]       build_flags = -DBRINGUP_LEVEL=2
[env:bringup_3_telemetry]   build_flags = -DBRINGUP_LEVEL=3
[env:bringup_4_navigation]  build_flags = -DBRINGUP_LEVEL=4
[env:production]            build_flags = -DBRINGUP_LEVEL=0
```

---

## 4. Receiver Firmware & Telemetry Protocol

### Task Structure

```
Core 0 — ESP-NOW RX callback
  • Validates incoming brain telemetry frames
  • Writes to a double-buffer (2 entries sufficient at 50 Hz)
  • Records receiver-side RSSI from ESP-NOW callback metadata

Core 1 — USB bridge + 500 Hz ESP-NOW TX
  • USB RX: any incoming frame from host resets `last_host_rx_us` watchdog;
            CTRL_PACKET updates `latest_ctrl` struct (mutex-protected)
  • USB TX: forwards brain telemetry frames to host
  • esp_timer @ 500 Hz: reads `latest_ctrl`, sends to brain via ESP-NOW
            If micros() - last_host_rx_us > 3,000,000: forces mode=DISABLED first
  • Heartbeat TX: 10 Hz STATUS packet to host
```

The 500 Hz ESP-NOW TX rate is generated entirely by the receiver's `esp_timer` — it does not depend on the host sending at any particular rate. The host can send at any rate; the receiver always rebroadcasts at 500 Hz with the latest known state.

### ESP-NOW Configuration

- Both devices: max TX power (`esp_wifi_set_max_tx_power(84)`)
- No ACK, no retransmit on either channel
- MAC addresses hardcoded in `config.h` on both devices
- Fixed channel 1 (no auto-switching)
- ESP-NOW v2.0 required — `platform = espressif32 @ >=6.0.0` in both `platformio.ini`
- Both brain and receiver are ESP32-S3 → v2.0 compatible, 1470-byte payload available
- 643-byte telemetry packet is well under the 1470-byte v2.0 limit

### Packet Formats

**ESP-NOW: Brain → Receiver (50 Hz, 635 bytes)**
```
Offset  Size  Field
0       2     frame_id (uint16 LE, monotonic)
2       1     type = 0x01
3       60    SunshineState (packed)
63      580   SunshineInput[20] (20 × 29 bytes, packed)
Total: 643 bytes
```

**ESP-NOW: Receiver → Brain (500 Hz, 8 bytes)**
```
Offset  Size  Field
0       2     seq_id (uint16 LE)
2       1     type = 0x02
3       1     mode
4       1     ctrl_x
5       1     ctrl_y
6       1     ctrl_theta
7       1     ctrl_throttle
Total: 8 bytes
```

### USB Serial Protocol (Receiver ↔ Host)

All frames: `[0xAA] [type: 1B] [payload_len: 2B LE] [payload: NB] [checksum: 1B XOR of payload]`

| Type | Direction | Payload | Rate |
|------|-----------|---------|------|
| `0x01` TELEM_FRAME | Receiver→Host | Full ESP-NOW telemetry payload (635 B) | 50 Hz |
| `0x02` CTRL_PACKET | Host→Receiver | `mode, ctrl_x, ctrl_y, ctrl_theta, ctrl_throttle` (5 B) | As sent by host |
| `0x03` STATUS | Both | `status_code (1B) + ascii message (≤32B)` | On event |
| `0x04` HEARTBEAT | Both | `timestamp_ms (4B)` | 10 Hz |
| `0x05` RX_RSSI | Receiver→Host | `rssi (int8, receiver-side reading of brain signal)` | 10 Hz |

**Status codes:**
```
0x00  OK
0x01  BRAIN_CONNECTED
0x02  BRAIN_DISCONNECTED    (>200ms since last brain frame — 10 missed at 50 Hz)
0x03  USB_LOGGING_STOPPED
0x04  INIT_ERROR
```

### Safety Watchdog Chain

```
Host app sends any USB packet → resets receiver's last_host_rx_us
    ↓ (if silent > 3 seconds)
Receiver forces mode=DISABLED in latest_ctrl (500 Hz timer callback)
    ↓
500 Hz timer sends DISABLED to brain via ESP-NOW
    ↓ (if brain receives no control packet for > 500ms)
Brain watchdog forces input.mode = DISABLED before sunshine_step()
    ↓
sunshine_step() DISABLED path → zero DShot outputs (structural)
```

Three independent watchdogs. Any broken link disables the robot within ≤3.5 seconds.

### Reconnect Handling

| Event | Brain | Receiver | Host |
|-------|-------|----------|------|
| Receiver USB unplugged | Continues; watchdog runs | Restarts on replug | Detects disconnect; pauses log |
| Receiver USB replugged | No effect | Sends BRAIN_CONNECTED if frames arriving | Opens **new** log file |
| Brain loses power | — | 10 missed frames → BRAIN_DISCONNECTED | Shows disconnected |
| Brain reconnects | Rejoins ESP-NOW (MAC hardcoded, no handshake) | Resumes forwarding | Opens new log file |

All multi-byte values: little-endian throughout (ESP32-S3 and macOS/ARM both LE).

---

## 5. Log File Format

### File Naming & Lifecycle

**Format:** `YYYY-MM-DD_HH-MM-SS[_label].sun`

- Label optional, set from host app UI (e.g. `match_1`, `practice_3`)
- **New file on every fresh connection** — never appended to
- Frames flushed to disk every 10 frames (200ms) — worst-case 200ms lost on crash
- `logging_complete` flag set on clean close
- Logging can be toggled from host app at any time; re-enabling creates a new file

### File Header (93 bytes)

```
Offset  Size  Field
0       5     magic = "SHINE"
5       2     file_format_version (uint16 LE) = 1
7       2     header_size (uint16 LE) = 93  — reader skips to here for first frame
9       4     schema_version (uint32 LE)
13      2     sizeof_input (uint16 LE)
15      2     sizeof_state (uint16 LE)
17      2     sizeof_vars  (uint16 LE)
19      8     created_at_ms (uint64 LE, Unix ms)
27      1     source: 0=live, 1=replay, 2=simulation
28      1     flags: bit0=logging_complete, bits1-7 reserved
29      64    label (null-terminated UTF-8, zero-padded)
Total: 93 bytes
```

### Frame Structure (~681 bytes at schema v1, 50 Hz)

```
Offset          Size          Field
0               4             frame_id (uint32 LE, monotonic — gaps = dropped telemetry)
4               1             frame_flags (bit0=vars_valid, bits1-7 reserved)
5               sizeof_state  SunshineState (state at START of frame, before any inputs)
5+S             sizeof_input × 20  SunshineInput[20]
5+S+20I         sizeof_vars   SunshineVars (computed after all 20 steps — 50 Hz snapshot)
```

At schema v1: 5 + 60 + 580 + 52 = **697 bytes/frame**  
At 50 Hz: ~35 KB/s | 5-min match: ~10.5 MB | 30-min session: ~63 MB

### Backwards Compatibility

```rust
fn read_struct<T: Default + AsBytesMut>(file: &mut File, file_size: usize) -> T {
    let mut obj = T::default();
    let read_bytes = file_size.min(size_of::<T>());
    file.read_exact(&mut obj.as_bytes_mut()[..read_bytes])?;
    if file_size > size_of::<T>() {
        file.seek(SeekFrom::Current((file_size - size_of::<T>()) as i64))?;
    }
    obj
}
```

### Real vs Replayed Data

| Data | Source | Resolution | In .sun file |
|------|--------|-----------|-------------|
| Real inputs | Robot telemetry | 1kHz | ✓ (20 per frame) |
| Real state | Robot telemetry | 50 Hz | ✓ (1 per frame) |
| Real vars | Computed at log time | 50 Hz | ✓ (1 per frame) |
| Replayed state | `sunshine_step()` replay | 1kHz | ✗ (computed live) |
| Replayed vars | `sunshine_step()` replay | 1kHz | ✗ (computed live) |

Graphing shows Real and Replayed as separate selectable series. Real is 50 Hz (thin/dotted), Replayed is 1kHz (solid). DShot: Real = `dshot_left_q` (quantized input), Replayed = `dshot_cmd_left` (full precision vars).

---

## 6. Host App Architecture

### Rust Backend Modules

```
src-tauri/src/
├── main.rs        — Tauri setup, app state init
├── ffi.rs         — sunshine_core C bindings + safe wrappers (only unsafe block)
├── serial.rs      — Serial port management + reconnect loop (serialport crate)
├── protocol.rs    — USB serial frame encode/decode
├── pipeline.rs    — Central data pipeline: owns ring buffer, drives live/replay/sim
├── replay.rs      — .sun file reader + replay loop
├── simulation.rs  — Brushed DC motor + robot kinematics physics model
├── logging.rs     — .sun file writer
├── controls.rs    — Latest control state + serialisation
└── commands.rs    — All Tauri command handlers (JS-callable)
```

### FFI Layer

`build.rs` compiles `sunshine_core/` via the `cc` crate. Rust structs use `#[repr(C, packed)]` to exactly match C layout.

```rust
extern "C" {
    fn sunshine_step(i: *const SunshineInput, s: *mut SunshineState, v: *mut SunshineVars);
    fn sunshine_state_init(s: *mut SunshineState);
    fn sunshine_schema_version() -> u32;
    fn sunshine_f16_to_f32(half: u16) -> f32;
    fn sunshine_f32_to_f16(f: f32) -> u16;
}

/// Safe wrapper — the only place `unsafe` appears for sunshine_core
pub fn brain_step(input: &SunshineInput, state: &mut SunshineState) -> SunshineVars { ... }
```

### Simulation Engine (`simulation.rs`)

Pure Rust, 1kHz driven by `pipeline.rs`. Produces `SunshineInput` each tick; DShot outputs from `SunshineVars` feed back as motor commands.

**Motor model (brushed DC, per wheel):**
```
KV      = 1100.0 RPM/V
Kt      = 60 / (2π × KV)  ≈ 8.68e-3 N·m/A    (standard brushless formula, SI)
R_phase = 0.075 Ω

back_emf = wheel_omega / (KV × 2π/60)
V_motor  = dshot_fraction × V_terminal
i_motor  = (V_motor - back_emf) / R_phase
torque   = Kt × i_motor
```

**Battery model (internal resistance only — no SOC drain):**
```
V_NOMINAL  = 8.4 V  (configurable constant)
R_INTERNAL = 8 mΩ
V_terminal = V_NOMINAL - (i_left + i_right) × R_INTERNAL
```

**Body dynamics:**
```
wheel_omega = body_omega × WHEEL_CENTER_R / WHEEL_RADIUS
           = body_omega × 40.5mm / 22mm

torque_body = (torque_left + torque_right) × WHEEL_CENTER_R / WHEEL_RADIUS
alpha       = torque_body / MOI          [MOI = 1.214e-3 kg·m²]
body_omega += alpha × dt
body_angle += body_omega × dt
```

**Sensor models:**
```
// Centripetal + tangential (tangential present during spinup)
a_centripetal = body_omega² × IMU_RADIUS
a_tangential  = alpha × IMU_RADIUS
ax = (a_centripetal - a_tangential) / √2      // 45° mount
ay = (a_centripetal + a_tangential) / √2
az = 9.81 m/s²  (gravity)
input.accel_{x,y,z} = (a{x,y,z} / ADXL_SCALE_MS2) as i16

// Magnetometer: ideal Earth field (50 μT, fixed direction φ_earth)
input.mag_x = (EARTH_FIELD_UT × cos(φ_earth - body_angle) / MAG_SCALE_UT) as i16
input.mag_y = (EARTH_FIELD_UT × sin(φ_earth - body_angle) / MAG_SCALE_UT) as i16
input.mag_z = 0

// Battery
input.batt_offset = round((V_terminal - 7.6) / 0.0205) as i8
```

Physical constants (in `simulation.rs`, easily editable):
```rust
const KV: f64           = 1100.0;
const R_PHASE: f64      = 0.075;
const V_NOMINAL: f64    = 8.4;
const R_INTERNAL: f64   = 0.008;
const WHEEL_RADIUS: f64 = 0.022;
const WHEEL_CENTER: f64 = 0.0405;
const MOI: f64          = 1.214e-3;
const IMU_RADIUS: f64   = 0.011;
const EARTH_FIELD: f64  = 50.0;   // μT
```

### Data Pipeline (`pipeline.rs`)

Ring buffer: last 120 seconds × 20 ticks/frame = 6000 frames × ~(29×20 + 44 + 52) = ~4 MB. Held in Rust, never pushed to frontend in bulk.

Three source modes share identical post-`brain_step()` code:

```
LIVE:       serial → protocol decode → TelemetryFrame{state, inputs[20]}
            → brain_step() × 20 → Vec<SunshineVars>
            → push 20 (input, vars) + state to ring buffer
            → logging.write_frame()  → emit live_update event

REPLAY:     replay.rs reads .sun frames → same brain_step() × 20 path
            (controllable speed: 1×, 5×, 10×, max)

SIMULATION: simulation.tick() → SunshineInput
            → brain_step() → SunshineVars
            → simulation.apply(vars.dshot_cmd_left, vars.dshot_cmd_right)
            → ring buffer + optional logging + events
```

### Graph Data Architecture

**Pull-based.** Frontend calls `get_graph_data` when viewport changes:

```
get_graph_data(
    channel:    String,   // e.g. "vars.est_theta", "inputs.accel_x"
    start_us:   u64,
    end_us:     u64,
    max_points: u32,      // viewport pixel width
    series:     Series,   // Real | Replayed
) → Vec<(u64, f32)>
```

Backend decimates using **min/max envelope** (preserves peaks). Result is at most `max_points` pairs. Called on: new data arriving (append latest to visible window) and user zoom/pan.

**Library:** uPlot (canvas-based, handles 1M+ points, ~40KB).

**Aesthetics:** Match the mockup exactly — dark theme, colored signal lines, minimal grid, units on y-axis labels, thin/dotted Real series vs solid Replayed series, Ctrl+scroll to zoom, scroll to pan. Extract CSS variables from the mockup HTML and apply directly to the Tauri WebView.

### Driver Controls

```typescript
// Target values from keys
onKeyDown('w') => target.y = 127;    onKeyUp('w') => target.y = 0
onKeyDown('s') => target.y = -127;   onKeyUp('s') => target.y = 0
onKeyDown('a') => target.x = -127;   onKeyUp('a') => target.x = 0
onKeyDown('d') => target.x = 127;    onKeyUp('d') => target.x = 0
onKeyDown('ArrowLeft')  => target.theta = -127;  onKeyUp => target.theta = 0
onKeyDown('ArrowRight') => target.theta = 127;   onKeyUp => target.theta = 0
onKeyDown('ArrowUp')   => throttle_delta = +THROTTLE_RATE
onKeyDown('ArrowDown') => throttle_delta = -THROTTLE_RATE

// Heavy LP filter at 60 Hz (requestAnimationFrame)
const ALPHA = 0.03   // ~1.8 Hz effective bandwidth
filtered.x     += ALPHA * (target.x     - filtered.x)
filtered.y     += ALPHA * (target.y     - filtered.y)
filtered.theta += ALPHA * (target.theta - filtered.theta)
filtered.throttle = clamp(filtered.throttle + throttle_delta, 0, 255)

// Send to Rust at ~30 Hz (every 2nd animation frame)
invoke('set_controls', { ...filtered })
```

Throttle resets to 0 on mode=DISABLED or source disconnect.

### Tauri Commands & Events

**Commands (JS → Rust):**
```
list_serial_ports() → Vec<String>
connect_serial(port) | disconnect_serial()
open_replay(path) → LogMetadata
start_simulation() | stop_source()
set_mode(mode: u8)
set_controls(x, y, theta, throttle)
enable_logging(label) → file_path | disable_logging()
get_graph_data(...) → Vec<(u64, f32)>
```

**Events (Rust → JS, 50 Hz):**
```
live_update  { est_theta, est_omega, batt_voltage, rssi, mode,
               brain_connected, receiver_connected, frame_id,
               loop_overrun_count, bandwidth_bps }
source_status { kind: Live|Replay|Sim|Disconnected, detail }
log_status    { active, path, frame_count }
error         { code, message }
```

### React Component Structure

```
App
├── Header
│   ├── ModeButtons        (DISABLED=red / TANK=yellow / MELTY=green, large)
│   ├── StatusBar          (RSSI, bandwidth, brain/receiver status, battery indicator)
│   └── LoggingControl     (toggle + label input)
├── ConnectionPanel
│   ├── LiveTab            (port selector, connect, receiver/brain status)
│   ├── ReplayTab          (file picker, play/pause, speed, scrub bar, file metadata)
│   └── SimulationTab      (start/stop, read-only sim parameters)
├── GraphPanel
│   ├── ChannelSelector    (grouped: Inputs / State / Variables; Real vs Replayed toggle)
│   └── uPlotCanvas
└── (keyboard listener on window)
```

Battery indicator colours (2S LiPo): ≥8.0V green, 7.4–8.0V yellow, 7.0–7.4V orange, <7.0V red (blinking).

---

## 7. Bringup Plan

Full step-by-step instructions with expected outputs and pass/fail criteria are in `BRINGUP.md`. This section is the structural overview.

### Level 1 — Low-level Sensors

**Goal:** All sensors init and read correctly over USB serial.  
**Setup:** Board powered via USB, no ESCs connected.  
**Env:** `bringup_1_sensors`  
**Output:** Labeled CSV at 100 Hz on USB serial (Arduino Serial Plotter compatible).

| Sensor | Expected at rest |
|--------|-----------------|
| ADXL375 | `accel_z ≈ +20` counts (1g); shake → all axes respond |
| LIS3MDL | Magnitude ≈ 860 counts (50 μT); rotate → `mag_x/y` traces circle |
| ADC | Maps to actual battery voltage ±0.1V (verify with multimeter) |

Pass: all sensors init, no LED error pattern, values sensible.

### Level 2 — DShot & ESC

**Goal:** Bidirectional DShot 600 working, eRPM telemetry readable.  
**Setup:** ESCs connected. **Props OFF.** AM32 pre-configured for 3D mode + correct directions.  
**Env:** `bringup_2_dshot`  
**Interface:** Simple USB serial command: `l <0-2000>`, `r <0-2000>`, `s`, `t` (print eRPM).

Pass: ESCs arm, both directions spin, eRPM matches expected RPM, success rate >90%.

### Level 3 — Telemetry Link

**Goal:** Full brain→receiver→host pipeline live in host app.  
**Setup:** Brain + receiver powered, host app running. Props OFF.  
**Env:** `bringup_3_telemetry` (motor outputs zeroed)

Checklist:
1. Host app: connect to receiver serial port → both status indicators green
2. Plot `inputs.accel_z` → stable ~1g; shake board → spikes visible
3. Rotate board → `inputs.mag_x/y` sinusoid visible
4. Check `inputs.rssi` → plausible negative dBm value
5. Press W key → `inputs.ctrl_y` ramps up (LP filter visible)
6. Walk receiver away → RSSI decreases
7. 2-minute stability test: no `BRAIN_DISCONNECTED` events, `frame_id` gaps rare

Pass: all of the above, log file contains clean data.

### Level 4 — Navigation Tuning

**Goal:** Kalman filter tuned, LED appears stationary when spinning, TANK mode working.  
**Setup:** Full robot, props OFF initially.  
**Env:** `bringup_4_navigation`

Tuning steps (detailed in `TUNING.md`):
1. Spin slowly → verify `omega_from_accel` responds; see spinup transient (expected)
2. Spin > 300 RPM → verify `derot_I`/`derot_Q` are near-constant (DC)
3. Tune `KF_R_ACCEL`, `KF_Q_OMEGA` for omega tracking quality
4. Tune `KF_R_MAG`, `KF_Q_THETA` for angle tracking quality
5. Verify LED appears stationary at a fixed heading

Pass: LED stationary ±5° at 500+ RPM; `est_theta` tracks with <5° RMS error; no drift over 30s.

### Level 5 — Drift Tuning (Full MELTY)

**Goal:** Robot translates reliably in commanded direction while spinning.  
**Setup:** Open floor, props ON, safe area.  
**Env:** `production`

1. Confirm LED is stationary (heading locked) — if not, return to Level 4
2. Apply small Y+ at low throttle → robot should drift forward
3. Tune `DRIFT_AMPLITUDE`, `DRIFT_PULSE_WIDTH`, `DRIFT_RAMP_WIDTH` per `TUNING.md`

Pass: translates in commanded direction at ≥3 of 4 compass points; no wheel slip; LED stationary during translation.

---

## 8. Deliverable Documents

| File | Contents |
|------|----------|
| `docs/ARCHITECTURE.md` | Codebase map for future AI sessions: module purposes, file locations, key interfaces, the append-only rule, schema versioning |
| `docs/BRINGUP.md` | Step-by-step human instructions for all 5 bringup levels with expected outputs and troubleshooting |
| `docs/TUNING.md` | Human-readable Kalman tuning guide + drift profile tuning with parameter tables |
| `docs/FILTER_MATH.md` | Plain-language + diagrams: centripetal→omega, sync demodulation, IQ low-pass, Kalman predict/update |
| `docs/DEBUGGING.md` | Instructions for the `sunshine:replay-debug` Claude skill |
| `docs/superpowers/specs/` | This document |

---

## Physical Constants Reference

| Parameter | Value | Source |
|-----------|-------|--------|
| Motor KV | 1100 RPM/V | Spec |
| Motor Kt | 60/(2π×1100) ≈ 8.68 mN·m/A | Calculated |
| Phase resistance | 75 mΩ | Spec |
| Wheel radius | 22 mm | Spec |
| Wheel center from axis | 40.5 mm | Spec |
| Robot MoI | 1213859 g·mm² = 1.214×10⁻³ kg·m² | Spec |
| IMU offset from center | 11 mm | PCB |
| IMU rotation | 45° to radial direction | PCB |
| Battery | 2S 650mAh 120C LiPo | Spec |
| Battery internal resistance | 8 mΩ | Estimated |
| Battery nominal voltage | 8.4V (full charge) | 2S LiPo |
| Max body RPM (estimated) | ~4000 RPM | Calculated (79.7% motor no-load) |
| Max centripetal @ IMU | 196.7g @ 4000 RPM | Calculated |
| Effective saturation RPM | ~4800 RPM | Calculated (45° mount) |
| Butterworth LP fc | 1 Hz | Design |
| Min usable mag speed | 120 RPM (4π rad/s) | Design |
| Earth magnetic field | 50 μT | Typical |
