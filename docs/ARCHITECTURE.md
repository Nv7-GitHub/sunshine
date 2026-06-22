# Sunshine Architecture

Codebase map for AI sessions and new contributors. Read this before touching anything.

---

## Repository Layout

```
sunshine/
├── sunshine_core/          # Pure C99 library — NO platform dependencies
│   ├── include/sunshine_core.h   # All structs, constants, full public API
│   └── src/
│       ├── utils.c         # float16, wrap_to_pi, clampf, unit conversions
│       ├── kalman.c        # 2-state Kalman filter (θ, ω): predict + two updates
│       ├── mag_heading.c   # Open-loop magnetometer heading (2nd-order HP + atan2)
│       ├── control.c       # DISABLED / TANK / MELTY logic, trapezoid wave, LED
│       └── brain.c         # sunshine_step() top-level, serialisation, state_init
├── sunshine_brain/         # PlatformIO ESP32-S3 brain firmware
│   ├── lib/sunshine_core/  # Symlinks to ../sunshine_core/ (PlatformIO lib)
│   ├── include/
│   │   ├── bringup.h       # BRINGUP_LEVEL compile-time feature gates
│   │   └── config.h        # Pin assignments, MAC addresses, timing constants
│   └── src/
│       ├── main.cpp        # setup/loop, error LED patterns, FreeRTOS task launch
│       ├── sensors/        # adxl375.cpp + lis3mdl.cpp — SPI driver wrappers
│       ├── dshot.cpp       # Bidirectional DShot 600, eRPM telemetry read
│       ├── ring_buffer.h   # Lock-free SPSC ring buffer (Core1→Core0, 40 entries)
│       ├── telemetry.cpp   # Core 0 task: ESP-NOW TX/RX, drain ring buffer
│       └── nav_control.cpp # Core 1: 1 kHz hard-real-time loop
├── sunshine_receiver/      # PlatformIO ESP32-S3 receiver firmware
│   ├── include/
│   │   ├── config.h        # Brain MAC, timing constants
│   │   ├── protocol.h      # USB frame format, encode/decode, FrameParser
│   │   └── led_status.h    # Onboard RGB status LED API
│   └── src/
│       ├── main.cpp        # WiFi+ESP-NOW init, task creation, LED init
│       ├── espnow_rx.cpp   # Core 0: ESP-NOW RX callback, double buffer
│       ├── usb_bridge.cpp  # Core 1: USB TX/RX, 500 Hz ESP-NOW TX, watchdog, LED drive
│       └── led_status.cpp  # Non-blocking WS2812 status LED (link/liveness)
├── sunshine_app/           # Tauri + React host application
│   ├── src-tauri/src/
│   │   ├── ffi.rs          # sunshine_core C bindings + safe wrappers (unsafe here only)
│   │   ├── serial.rs       # USB serial port management + reconnect loop
│   │   ├── protocol.rs     # USB frame encode/decode
│   │   ├── pipeline.rs     # Central data pipeline, ring buffer, drives all sources
│   │   ├── replay.rs       # .sun file reader + replay engine
│   │   ├── simulation.rs   # Brushed DC motor + robot kinematics physics model
│   │   ├── logging.rs      # .sun file writer
│   │   ├── controls.rs     # Latest control state management
│   │   └── commands.rs     # All Tauri command handlers (JS-callable)
│   └── src/
│       ├── components/Header.tsx          # Mode buttons, status bar, logging control
│       ├── components/ConnectionPanel.tsx # Live/Replay/Sim tabs
│       ├── components/GraphPanel.tsx      # uPlot canvas + channel selector
│       └── hooks/useKeyboard.ts          # Key → filtered control values → Rust
├── sunshine_pcb/           # KiCad PCB design files
├── tools/
│   └── gen_filter_coefficients.py        # Generates LP4 biquad coefficients
└── docs/
    ├── ARCHITECTURE.md     # This file
    ├── BRINGUP.md          # Step-by-step hardware bringup instructions
    ├── TUNING.md           # Kalman + drift tuning guide
    ├── FILTER_MATH.md      # Plain-language explanation of the full filter chain
    ├── DEBUGGING.md        # Instructions for sunshine:replay-debug Claude skill
    └── superpowers/
        ├── specs/          # System design spec (source of truth)
        └── plans/          # Implementation plans (per subsystem)
```

---

## Data Flow

```
LIVE:       Brain sensors
              → 1 kHz sunshine_step() on ESP32
              → ESP-NOW (~167 Hz, 6 inputs/frame, 237 B) → Receiver
                  (ESP-NOW 250-byte limit precludes 20 inputs/frame)
              → USB serial (237 B payload + 5 B framing) → Tauri serial.rs
              → pipeline.rs: brain_step() × 6 → ring buffer → live_update event
              → React GraphPanel

REPLAY:     .sun file → replay.rs reads frames
              → pipeline.rs: brain_step() × 6 (at controllable speed)
              → same ring buffer + events path as LIVE

SIMULATION: simulation.rs ticks at 1 kHz → SunshineInput
              → pipeline.rs: brain_step() → SunshineVars
              → simulation.rs applies dshot_cmd_left/right → next tick
              → same ring buffer + events path
```

---

## Module Responsibilities

### `sunshine_core` — Pure C island

**The single rule:** no ESP-IDF, no FreeRTOS, no OS calls. Only C99 standard library (`math.h`, `stdint.h`). Compiles identically on ESP32 and desktop.

**Entry point:** `sunshine_step(in, state, vars_out)` — pure function, no side effects except mutating `*state`. Identical inputs + state → identical output. This is what makes replay and simulation possible.

**Internal call sequence inside `sunshine_step()`:**
1. Decode raw inputs → `vars->centripetal_ms2`, `omega_from_accel`, battery, eRPM
2. `kalman_predict()` — dead-reckoning: θ += ω·dt, propagate covariance
3. `kalman_update_omega()` — accel measurement update (skip if saturated)
4. `mag_heading_step()` — open-loop high-pass + atan2 → mag_angle (absolute)
5. `kalman_update_theta()` — mag measurement update (skip if ω < 4π rad/s)
6. `control_step()` — DISABLED/TANK/MELTY → dshot_cmd_left/right, led_on

### `sunshine_brain` — Core assignment

- **Core 1** (priority 10, pinned): 1 kHz nav+control loop. Reads sensors, calls `sunshine_step()`, sends DShot, pushes to ring buffer. Never blocks on telemetry.
- **Core 0** (priority 5): Telemetry task. Drains ring buffer → assembles 50 Hz ESP-NOW frames (20 inputs + state) → sends to receiver. Receives 500 Hz control packets from receiver → stores behind mutex.

### `sunshine_receiver` — Bridge role

- **Core 0**: ESP-NOW RX callback stores brain telemetry in a double buffer, signals Core 1.
- **Core 1**: USB bridge loop — forwards telemetry frames to host, reads USB from host, forwards controls to brain. A FreeRTOS-independent `esp_timer` at 500 Hz sends the latest control packet to the brain regardless of host cadence.
- **Host watchdog**: If no USB packet from host for 3 seconds, forces `mode=DISABLED` before sending to brain.
- **Status LED**: The onboard WS2812 RGB LED (`led_status.cpp`, GPIO48 via `RGB_BUILTIN`) shows liveness and link state — driven each bridge tick from the existing brain-connected and host-watchdog signals. See the LED legend in `BRINGUP.md`.

### `sunshine_app` — Tauri backend

**`ffi.rs`** is the only file with `unsafe`. All `sunshine_core` C calls go through here.

**`pipeline.rs`** is the central hub — all three source modes (live/replay/sim) converge here after producing `SunshineInput`. It calls `brain_step()`, writes to the ring buffer, triggers logging, and emits `live_update` events to React.

**`simulation.rs`** implements brushed DC motor physics (KV=1100, R=75 mΩ), battery model (8.4V, 8 mΩ internal resistance), and robot body dynamics (MoI=1.214e-3 kg·m²). It feeds DShot outputs from `SunshineVars` back as motor commands each tick.

---

## IO Layer — The Contract

Nothing crosses the core/platform boundary except through these three structs. All defined in `sunshine_core.h` with `__attribute__((packed))`:

| Struct | Size | Direction | Description |
|--------|------|-----------|-------------|
| `SunshineInput` | 29 B | Hardware → core | 1 kHz sensor frame |
| `SunshineState` | 60 B | Core ↔ core | Filter history (Kalman + LP states) |
| `SunshineVars` | 56 B | Core → hardware | Derived variables, computed each step |

`SunshineVars` is never telemetered — it is always recomputed from `SunshineInput` + `SunshineState` via `sunshine_step()`. The log file stores the 50 Hz snapshot for display, but replay always recomputes at 1 kHz.

---

## Append-Only Rule

**Fields may only be added to the END of `SunshineInput`, `SunshineState`, or `SunshineVars`. Never insert, reorder, or resize existing fields.**

`SUNSHINE_SCHEMA_VERSION` (currently 2) must be bumped with a comment on every struct change. Log files store `sizeof_input`, `sizeof_state`, `sizeof_vars` in the header; readers handle mismatches:
- Reader's `sizeof < file's sizeof`: truncated read, extra bytes skipped
- Reader's `sizeof > file's sizeof`: missing trailing fields default to zero

This rule means old log files always remain readable without converter functions.

---

## Safety Architecture

Three independent watchdogs ensure the robot stops within ≤3.5 seconds of any broken link:

1. **Host app** → sends any USB packet → resets receiver's watchdog timer
2. **Receiver** → if silent > 3 s, forces `mode=DISABLED` in the 500 Hz control stream
3. **Brain** → if no control packet for 500 ms, forces `input.mode = DISABLED`
4. **`sunshine_step()`** → `DISABLED` mode unconditionally zeroes DShot outputs (structural — not a conditional check)

---

## Key Constants

All physical/tuning constants are in `sunshine_core/include/sunshine_core.h`. Firmware-specific constants (pin assignments, MAC addresses) are in `sunshine_brain/include/config.h` and `sunshine_receiver/include/config.h`.

| Constant | Value | Where |
|----------|-------|-------|
| `ADXL_SCALE_MS2` | 49e-3 × 9.81 | sunshine_core.h |
| `MAG_SCALE_UT` | 0.058 µT/count | sunshine_core.h |
| `IMU_RADIUS_M` | 0.011 m | sunshine_core.h |
| `SUNSHINE_MAG_MIN_OMEGA` | 4π rad/s (~120 RPM) | sunshine_core.h |
| `KF_Q_THETA/OMEGA` | 1e-6 / 1e-3 | sunshine_core.h |
| `KF_R_ACCEL/MAG` | 0.5 / 0.1 | sunshine_core.h |
| `DRIFT_PULSE_WIDTH` | 0.25 | sunshine_core.h |
| `DRIFT_AMPLITUDE` | 0.40 | sunshine_core.h |
| Loop interval | 1000 µs (1 kHz) | config.h (brain) |
| ESP-NOW TX rate (brain→rx) | ~167 Hz | Derived: 6 inputs/frame (250-byte ESP-NOW limit) |
| ESP-NOW TX rate (rx→brain) | 500 Hz | config.h (receiver) |
| Host watchdog | 3 s | config.h (receiver) |
| Brain watchdog | 500 ms | config.h (brain) |

---

## Known Platform Issues

**pioarduino `stable` URL**: The brain's `platformio.ini` references the pioarduino `stable` release URL. As of 2026-05-26 this URL fails with `MissingPackageManifestError`. See `BRINGUP.md` for the workaround.

**Receiver IDF version**: The receiver is built against `espressif32@6.0.0` (IDF 4.4). IDF 4.x uses a different `esp_now_recv_cb_t` signature (no `esp_now_recv_info_t`), so receiver-side RSSI is not available from the callback — `espnow_rx_get_rssi()` returns -127 until rebuilt on IDF 5.x. Brain-side RSSI (`inputs.rssi`) works normally.

**DShot library**: `platformio.ini` uses `derdoktor667/DShotRMT @ ^0.9.5` rather than `qqqlab/DShotRMT_NEO`. The API is similar but check the library examples if arming or eRPM telemetry behaves unexpectedly.
