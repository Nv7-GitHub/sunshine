# sunshine_brain Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the ESP32-S3 brain firmware: sensor drivers, bidirectional DShot 600, FreeRTOS two-core task structure, and the 1 kHz nav+control loop that calls `sunshine_step()`.

**Architecture:** Core 1 owns the 1 kHz hard-real-time loop (sensors → `sunshine_step()` → DShot + LED → ring buffer). Core 0 owns the telemetry task (drains ring buffer → 50 Hz ESP-NOW TX; receives 500 Hz control packets from receiver and stores them behind a mutex). Five `platformio.ini` bringup environments gate features at compile time so each subsystem can be tested in isolation before the next is added.

**Tech Stack:** PlatformIO, ESP32-S3, Arduino framework (esp-idf ≥ 6.0), Adafruit ADXL375 library, Adafruit LIS3MDL library, DShotRMT_NEO, ESP-NOW v2.0, FreeRTOS, sunshine_core (compiled as a library).

**Prerequisite:** `sunshine_core` plan complete — `sunshine_core/` directory with all sources must exist before Task 7.

---

## File Structure

```
sunshine_brain/
├── platformio.ini
├── lib/
│   └── sunshine_core/             # symlink or copy of ../sunshine_core/
│       ├── library.json
│       ├── include/sunshine_core.h
│       └── src/{utils,kalman,derot_filter,control,brain}.c
├── include/
│   ├── bringup.h                  # BRINGUP_LEVEL feature gates
│   └── config.h                   # pin assignments, MAC addresses, constants
└── src/
    ├── main.cpp                   # setup(), loop(), error LED patterns
    ├── sensors/
    │   ├── adxl375.h + adxl375.cpp   # ADXL375 SPI driver wrapper
    │   └── lis3mdl.h + lis3mdl.cpp   # LIS3MDL SPI driver wrapper
    ├── dshot.h + dshot.cpp            # DShotRMT_NEO wrapper, eRPM telemetry
    ├── ring_buffer.h                  # Lock-free(ish) 40-entry ring buffer
    ├── telemetry.h + telemetry.cpp    # Core 0 task: ESP-NOW TX/RX
    └── nav_control.h + nav_control.cpp  # Core 1: 1 kHz loop
```

---

## Task 1: PlatformIO Setup + bringup.h + config.h

**Files:**
- Create: `sunshine_brain/platformio.ini`
- Create: `sunshine_brain/include/bringup.h`
- Create: `sunshine_brain/include/config.h`

- [ ] **Step 1: Create platformio.ini**

```ini
[env_base]
platform    = https://github.com/pioarduino/platform-espressif32/releases/download/stable/platform-espressif32.zip
board       = esp32-s3-devkitc-1
framework   = arduino
monitor_speed  = 921600
upload_speed   = 921600
build_flags    =
    -DARDUINO_USB_CDC_ON_BOOT=1
    -DCORE_DEBUG_LEVEL=0
lib_deps =
    adafruit/Adafruit ADXL375 @ ^1.0.0
    adafruit/Adafruit LIS3MDL @ ^1.2.0
    qqqlab/DShotRMT_NEO @ ^1.0.0

[env:bringup_1_sensors]
extends    = env_base
build_flags = ${env_base.build_flags} -DBRINGUP_LEVEL=1

[env:bringup_2_dshot]
extends    = env_base
build_flags = ${env_base.build_flags} -DBRINGUP_LEVEL=2

[env:bringup_3_telemetry]
extends    = env_base
build_flags = ${env_base.build_flags} -DBRINGUP_LEVEL=3

[env:bringup_4_navigation]
extends    = env_base
build_flags = ${env_base.build_flags} -DBRINGUP_LEVEL=4

[env:production]
extends    = env_base
build_flags = ${env_base.build_flags} -DBRINGUP_LEVEL=0
```

> **Platform note:** The pioarduino `stable` URL currently fails with `MissingPackageManifestError` when `framework-arduinoespressif32-libs` tries to install from the official arduino-esp32 release tarball. If the brain project fails to build, pin the platform to a known-good release or wait for pioarduino to fix the `stable` pointer. The receiver was temporarily built against `espressif32@6.0.0` (IDF 4.4) as a workaround; once pioarduino is fixed both projects should use the URL above.

- [ ] **Step 2: Create include/bringup.h**

```cpp
#pragma once

// BRINGUP_LEVEL controls which features are compiled in.
// 0 = production (all features), 1-4 = incremental bringup.
//
// Level 1: sensors CSV on USB serial only (no ESP-NOW, no DShot)
// Level 2: + DShot interactive test (no telemetry)
// Level 3: + ESP-NOW telemetry, no navigation filter updates (zeros dshot)
// Level 4: + full navigation (kalman, derot filter), still zeros dshot for safety
// Level 0: production — all features, full control output

#ifndef BRINGUP_LEVEL
#define BRINGUP_LEVEL 0
#endif

// Feature gates derived from level
#if BRINGUP_LEVEL == 0 || BRINGUP_LEVEL >= 2
#  define FEATURE_DSHOT     1
#else
#  define FEATURE_DSHOT     0
#endif

#if BRINGUP_LEVEL == 0 || BRINGUP_LEVEL >= 3
#  define FEATURE_TELEMETRY 1
#else
#  define FEATURE_TELEMETRY 0
#endif

#if BRINGUP_LEVEL == 0 || BRINGUP_LEVEL >= 4
#  define FEATURE_NAVIGATION 1
#else
#  define FEATURE_NAVIGATION 0
#endif

// At levels 3-4 we always zero the dshot outputs for safety (no actual drive)
#if BRINGUP_LEVEL >= 3 && BRINGUP_LEVEL <= 4
#  define FORCE_DSHOT_ZERO 1
#else
#  define FORCE_DSHOT_ZERO 0
#endif
```

- [ ] **Step 3: Create include/config.h**

```cpp
#pragma once
#include <stdint.h>

// ── SPI pins ──────────────────────────────────────────────────────────────────
// FSPI → ADXL375
#define PIN_ADXL_CS    10
#define PIN_ADXL_MOSI  11
#define PIN_ADXL_SCK   12
#define PIN_ADXL_MISO  13
// INT pins (not used in polled mode, reserved)
#define PIN_ADXL_INT1   8
#define PIN_ADXL_INT2   9

// HSPI → LIS3MDL
#define PIN_MAG_CS      18
#define PIN_MAG_MOSI    15
#define PIN_MAG_SCK     16
#define PIN_MAG_MISO    17
#define PIN_MAG_DRDY    14
#define PIN_MAG_INT     21

// ── DShot ─────────────────────────────────────────────────────────────────────
#define PIN_DSHOT_LEFT   4   // S1
#define PIN_DSHOT_RIGHT  5   // S2

// ── Battery ADC ───────────────────────────────────────────────────────────────
#define PIN_BATT_ADC    39
// V = adc_raw * (3.3/4095) * (R_high+R_low)/R_low = adc_raw * (3.3/4095) * 3.0
static constexpr float BATT_ADC_SCALE = (3.3f / 4095.0f) * 3.0f;

// ── LED ───────────────────────────────────────────────────────────────────────
// LED is active-high (gate of low-side switch). Reuse whichever GPIO is
// available on your board. Fill in the actual pin.
#define PIN_LED          2   // CHANGE if your board uses a different pin

// ── Timing ────────────────────────────────────────────────────────────────────
static constexpr uint32_t LOOP_INTERVAL_US   = 1000;  // 1 kHz
static constexpr uint32_t TELEMETRY_FRAMES   = 20;    // inputs per 50 Hz frame
static constexpr uint32_t CTRL_WATCHDOG_MS   = 500;   // no ctrl → DISABLED

// ── ESP-NOW ───────────────────────────────────────────────────────────────────
static constexpr uint8_t ESPNOW_CHANNEL      = 1;
// Receiver MAC — fill in after running receiver and noting its WiFi STA MAC
static const uint8_t RECEIVER_MAC[6]         = {0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX};
```

- [ ] **Step 4: Create sunshine_core library.json**

PlatformIO needs a `library.json` to pick up the local source directory:

```bash
mkdir -p sunshine_brain/lib/sunshine_core
```

Create `sunshine_brain/lib/sunshine_core/library.json`:
```json
{
  "name": "sunshine_core",
  "version": "1.0.0",
  "build": {
    "srcDir": ".",
    "includeDir": "."
  }
}
```

Then create symlinks so PlatformIO sees the sources:
```bash
ln -s ../../../../sunshine_core/include/sunshine_core.h \
      sunshine_brain/lib/sunshine_core/sunshine_core.h
ln -s ../../../../sunshine_core/src/utils.c       sunshine_brain/lib/sunshine_core/utils.c
ln -s ../../../../sunshine_core/src/kalman.c      sunshine_brain/lib/sunshine_core/kalman.c
ln -s ../../../../sunshine_core/src/derot_filter.c sunshine_brain/lib/sunshine_core/derot_filter.c
ln -s ../../../../sunshine_core/src/control.c     sunshine_brain/lib/sunshine_core/control.c
ln -s ../../../../sunshine_core/src/brain.c       sunshine_brain/lib/sunshine_core/brain.c
```

- [ ] **Step 5: Add stub src/main.cpp and compile**

```cpp
// src/main.cpp (stub — replaced in later tasks)
#include "bringup.h"
#include "config.h"
#include <Arduino.h>
void setup() { Serial.begin(921600); }
void loop()  { delay(1000); }
```

```bash
cd sunshine_brain && pio run --environment bringup_1_sensors 2>&1 | tail -10
```
Expected: firmware.bin built, no errors.

- [ ] **Step 6: Commit**

```bash
git add sunshine_brain/
git commit -m "feat(brain): PlatformIO project scaffold, bringup gates, sunshine_core lib"
```

---

## Task 2: ADXL375 SPI Driver (Bringup Level 1)

**Files:**
- Create: `sunshine_brain/src/sensors/adxl375.h`
- Create: `sunshine_brain/src/sensors/adxl375.cpp`

- [ ] **Step 1: Create sensors/adxl375.h**

```cpp
#pragma once
#include <stdint.h>
#include <stdbool.h>

struct Adxl375Sample {
    int16_t x, y, z;   // raw counts, 49 mg/LSB
    bool    valid;
};

bool        adxl375_init(void);   // returns false on failure
Adxl375Sample adxl375_read(void);
```

- [ ] **Step 2: Create sensors/adxl375.cpp**

```cpp
#include "adxl375.h"
#include "../config.h"
#include <SPI.h>
#include <Adafruit_ADXL375.h>

static SPIClass            fspi(FSPI);
static Adafruit_ADXL375    adxl(12345, &fspi);  // ID unused by Adafruit lib

bool adxl375_init(void) {
    fspi.begin(PIN_ADXL_SCK, PIN_ADXL_MISO, PIN_ADXL_MOSI, PIN_ADXL_CS);
    if (!adxl.begin(PIN_ADXL_CS, &fspi)) return false;
    // ODR 1600 Hz, full-resolution mode (±200g, 49 mg/LSB)
    adxl.setDataRate(ADXL3XX_DATARATE_1600_HZ);
    adxl.setRange(ADXL375_RANGE_200_G);
    return true;
}

Adxl375Sample adxl375_read(void) {
    Adxl375Sample s;
    sensors_event_t event;
    adxl.getEvent(&event);
    // Adafruit returns m/s² — convert back to raw counts
    constexpr float ADXL_SCALE_MS2 = 49e-3f * 9.81f;
    s.x     = (int16_t)(event.acceleration.x / ADXL_SCALE_MS2);
    s.y     = (int16_t)(event.acceleration.y / ADXL_SCALE_MS2);
    s.z     = (int16_t)(event.acceleration.z / ADXL_SCALE_MS2);
    s.valid = true;
    return s;
}
```

**Note on Adafruit ADXL375 raw access:** The Adafruit library converts to m/s² internally. An alternative is to use the underlying `getX()/getY()/getZ()` raw methods if exposed, or to access the SPI registers directly. At bringup, verify the scaling is correct by checking that `accel_z ≈ +20` counts at rest (1g / 0.049g per count ≈ 20.4). If the Adafruit library version doesn't expose raw counts, add a direct SPI register read:

```cpp
// Fallback raw read (if Adafruit lib doesn't give raw counts directly)
// ADXL375 DATA registers: 0x32–0x37 (X0, X1, Y0, Y1, Z0, Z1)
static int16_t read_raw(uint8_t reg) {
    fspi.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE3));
    digitalWrite(PIN_ADXL_CS, LOW);
    fspi.transfer(0x80 | 0x40 | reg); // read + multi-byte
    uint8_t lo = fspi.transfer(0);
    uint8_t hi = fspi.transfer(0);
    digitalWrite(PIN_ADXL_CS, HIGH);
    fspi.endTransaction();
    return (int16_t)((hi << 8) | lo);
}
```

- [ ] **Step 3: Update main.cpp for bringup level 1 CSV output**

Replace `src/main.cpp`:
```cpp
#include "bringup.h"
#include "config.h"
#include "sensors/adxl375.h"
#include "sensors/lis3mdl.h"  // stub in Task 3 — include anyway
#include <Arduino.h>

static bool adxl_ok = false;

void setup() {
    Serial.begin(921600);
    pinMode(PIN_LED, OUTPUT);
    adxl_ok = adxl375_init();
    if (!adxl_ok) {
        Serial.println("ERROR: ADXL375 init failed");
        // Blink pattern 1 (1 fast blink → 1s off → repeat)
        while (true) {
            for (int i = 0; i < 1; i++) {
                digitalWrite(PIN_LED, HIGH); delay(50);
                digitalWrite(PIN_LED, LOW);  delay(50);
            }
            delay(1000);
        }
    }
    // Arduino Serial Plotter header
    Serial.println("accel_x,accel_y,accel_z");
}

void loop() {
#if BRINGUP_LEVEL == 1
    if (adxl_ok) {
        Adxl375Sample s = adxl375_read();
        Serial.printf("%d,%d,%d\n", s.x, s.y, s.z);
    }
    delay(10);  // ~100 Hz for Serial Plotter
#endif
}
```

- [ ] **Step 4: Flash bringup_1_sensors + verify**

```bash
cd sunshine_brain && pio run --target upload --environment bringup_1_sensors
pio device monitor --baud 921600
```
Expected:
- `accel_x,accel_y,accel_z` header line
- `accel_z ≈ +20` counts at rest (tolerance ±3)
- Shake board → all axes spike

- [ ] **Step 5: Commit**

```bash
git add sunshine_brain/src/sensors/adxl375.h sunshine_brain/src/sensors/adxl375.cpp \
        sunshine_brain/src/main.cpp
git commit -m "feat(brain): ADXL375 SPI driver + bringup-1 CSV output"
```

---

## Task 3: LIS3MDL Driver + Battery ADC (Bringup Level 1)

**Files:**
- Create: `sunshine_brain/src/sensors/lis3mdl.h`
- Create: `sunshine_brain/src/sensors/lis3mdl.cpp`

- [ ] **Step 1: Create sensors/lis3mdl.h**

```cpp
#pragma once
#include <stdint.h>
#include <stdbool.h>

struct MagSample {
    int16_t x, y, z;   // raw counts, 0.058 µT/LSB at ±16 Gauss
    bool    valid;
};

bool      lis3mdl_init(void);
MagSample lis3mdl_read(void);
float     batt_read_v(void);    // read battery voltage (V)
```

- [ ] **Step 2: Create sensors/lis3mdl.cpp**

```cpp
#include "lis3mdl.h"
#include "../config.h"
#include <SPI.h>
#include <Adafruit_LIS3MDL.h>
#include <Arduino.h>

static SPIClass        hspi(HSPI);
static Adafruit_LIS3MDL mag;

bool lis3mdl_init(void) {
    hspi.begin(PIN_MAG_SCK, PIN_MAG_MISO, PIN_MAG_MOSI, PIN_MAG_CS);
    if (!mag.begin_SPI(PIN_MAG_CS, &hspi)) return false;
    mag.setDataRate(LIS3MDL_DATARATE_1000_HZ);
    mag.setRange(LIS3MDL_RANGE_16_GAUSS);
    mag.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    mag.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
    return true;
}

MagSample lis3mdl_read(void) {
    MagSample s;
    mag.read();
    // Adafruit LIS3MDL exposes raw counts directly
    s.x     = mag.x;
    s.y     = mag.y;
    s.z     = mag.z;
    s.valid = true;
    return s;
}

float batt_read_v(void) {
    int raw = analogRead(PIN_BATT_ADC);
    return (float)raw * BATT_ADC_SCALE;
}
```

- [ ] **Step 3: Update main.cpp — add mag + batt to CSV output**

```cpp
#include "bringup.h"
#include "config.h"
#include "sensors/adxl375.h"
#include "sensors/lis3mdl.h"
#include <Arduino.h>

static bool adxl_ok = false;
static bool mag_ok  = false;

static void blink_error(int n) {
    while (true) {
        for (int i = 0; i < n; i++) {
            digitalWrite(PIN_LED, HIGH); delay(50);
            digitalWrite(PIN_LED, LOW);  delay(50);
        }
        delay(1000);
        Serial.printf("INIT ERROR: sensor fault (blink count %d)\n", n);
    }
}

void setup() {
    Serial.begin(921600);
    pinMode(PIN_LED, OUTPUT);
    analogReadResolution(12);

    adxl_ok = adxl375_init();
    mag_ok  = lis3mdl_init();

    int err_code = (!adxl_ok ? 1 : 0) + (!mag_ok ? 2 : 0);
    if (err_code) blink_error(err_code);

    Serial.println("accel_x,accel_y,accel_z,mag_x,mag_y,mag_z,batt_v");
}

void loop() {
#if BRINGUP_LEVEL == 1
    Adxl375Sample a = adxl375_read();
    MagSample     m = lis3mdl_read();
    float         v = batt_read_v();
    Serial.printf("%d,%d,%d,%d,%d,%d,%.3f\n",
                  a.x, a.y, a.z, m.x, m.y, m.z, v);
    delay(10);
#endif
}
```

- [ ] **Step 4: Flash and verify**

```bash
pio run --target upload --environment bringup_1_sensors && pio device monitor --baud 921600
```
Expected:
- `accel_z ≈ +20` at rest
- `mag` magnitude: sqrt(x²+y²+z²) ≈ 860 counts (50 µT / 0.058 µT/count)
- Rotate board → `mag_x`, `mag_y` trace a circle (±860 count amplitude)
- `batt_v` matches multimeter reading ±0.1V

- [ ] **Step 5: Commit**

```bash
git add sunshine_brain/src/sensors/lis3mdl.h sunshine_brain/src/sensors/lis3mdl.cpp \
        sunshine_brain/src/main.cpp
git commit -m "feat(brain): LIS3MDL SPI driver + battery ADC + bringup-1 complete"
```

---

## Task 4: DShot Driver (Bringup Level 2)

**Files:**
- Create: `sunshine_brain/src/dshot.h`
- Create: `sunshine_brain/src/dshot.cpp`

- [ ] **Step 1: Create src/dshot.h**

```cpp
#pragma once
#include <stdint.h>
#include <stdbool.h>

bool  dshot_init(void);                     // init both ESCs; returns false on error
void  dshot_send(uint16_t left, uint16_t right); // DShot values 0–2047; 0=disarm
float dshot_erpm_left(void);
float dshot_erpm_right(void);

// Quantise [0.0, 2047.0] → uint8 for SunshineInput
static inline uint8_t dshot_quantize(float v) {
    int q = (int)(v * (255.0f / 2047.0f) + 0.5f);
    return (uint8_t)(q < 0 ? 0 : q > 255 ? 255 : q);
}
```

- [ ] **Step 2: Create src/dshot.cpp**

```cpp
#include "dshot.h"
#include "config.h"
#include <DShotRMT.h>        // DShotRMT_NEO library

static DShotRMT dshot_left (PIN_DSHOT_LEFT,  RMT_CHANNEL_0);
static DShotRMT dshot_right(PIN_DSHOT_RIGHT, RMT_CHANNEL_1);

static float erpm_left_val  = 0.0f;
static float erpm_right_val = 0.0f;

bool dshot_init(void) {
    // DShot 600, bidirectional (DSHOT_BIDIRECTIONAL = true)
    bool ok_l = dshot_left .install(DSHOT600, true);
    bool ok_r = dshot_right.install(DSHOT600, true);
    if (!ok_l || !ok_r) return false;

    // AM32: 3D mode uses values 48–1047 (reverse) and 1049–2047 (forward).
    // Value 1048 = neutral/brake. Send neutral during arming (300ms).
    for (int i = 0; i < 300; i++) {
        dshot_left .send_dshot_value(1048, false);
        dshot_right.send_dshot_value(1048, false);
        delay(1);
    }
    return true;
}

void dshot_send(uint16_t left, uint16_t right) {
    // Receive eRPM telemetry from previous cycle before sending new command
    uint32_t telem_l = dshot_left .get_dshot_rpm();
    uint32_t telem_r = dshot_right.get_dshot_rpm();
    if (telem_l > 0) erpm_left_val  = (float)telem_l;
    if (telem_r > 0) erpm_right_val = (float)telem_r;

    dshot_left .send_dshot_value(left,  false);
    dshot_right.send_dshot_value(right, false);
}

float dshot_erpm_left(void)  { return erpm_left_val;  }
float dshot_erpm_right(void) { return erpm_right_val; }
```

**Note on DShotRMT_NEO API:** If the library API differs (method names, arming sequence), adjust accordingly. The key requirement is DShot 600 bidirectional on ESP32-S3 RMT channels. Check the library's examples for the exact arming procedure for AM32 in 3D mode.

- [ ] **Step 3: Add bringup level 2 interactive test to main.cpp**

Add to `src/main.cpp` after the existing setup/loop (inside `#if BRINGUP_LEVEL == 2` guard):

```cpp
// In setup(), after sensor init, add:
#if BRINGUP_LEVEL >= 2
    bool dshot_ok = dshot_init();
    if (!dshot_ok) blink_error(3);
    Serial.println("DShot ready. Commands: l <0-2047>, r <0-2047>, s (stop), t (print eRPM)");
#endif
```

```cpp
// Replace loop() entirely:
void loop() {
#if BRINGUP_LEVEL == 1
    Adxl375Sample a = adxl375_read();
    MagSample     m = lis3mdl_read();
    float         v = batt_read_v();
    Serial.printf("%d,%d,%d,%d,%d,%d,%.3f\n", a.x, a.y, a.z, m.x, m.y, m.z, v);
    delay(10);

#elif BRINGUP_LEVEL == 2
    static char cmd[32];
    static int  ci = 0;
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            cmd[ci] = '\0';
            if (cmd[0] == 'l') {
                int v = atoi(cmd + 2);
                dshot_send((uint16_t)v, 1048);
                Serial.printf("Left → %d\n", v);
            } else if (cmd[0] == 'r') {
                int v = atoi(cmd + 2);
                dshot_send(1048, (uint16_t)v);
                Serial.printf("Right → %d\n", v);
            } else if (cmd[0] == 's') {
                dshot_send(1048, 1048);
                Serial.println("Stop");
            } else if (cmd[0] == 't') {
                Serial.printf("eRPM L=%.0f R=%.0f\n",
                              dshot_erpm_left(), dshot_erpm_right());
            }
            ci = 0;
        } else if (ci < 31) {
            cmd[ci++] = c;
        }
    }
    dshot_send(1048, 1048);  // keep sending neutral if no command
    delay(1);
#endif
}
```

- [ ] **Step 4: Flash bringup_2_dshot + verify**

```bash
pio run --target upload --environment bringup_2_dshot && pio device monitor --baud 921600
```
Expected:
- `DShot ready.` message
- `l 1200` → left ESC spins forward; `s` → stops
- `t` → prints eRPM matching expected RPM (KV × battery_V, roughly)
- eRPM telemetry success rate > 90% (check with repeated `t` commands)

- [ ] **Step 5: Commit**

```bash
git add sunshine_brain/src/dshot.h sunshine_brain/src/dshot.cpp \
        sunshine_brain/src/main.cpp
git commit -m "feat(brain): DShot 600 bidirectional driver + bringup-2 interactive test"
```

---

## Task 5: ESP-NOW + Control Reception (Telemetry Foundation)

**Files:**
- Create: `sunshine_brain/src/telemetry.h`
- Create: `sunshine_brain/src/telemetry.cpp`

- [ ] **Step 1: Create src/telemetry.h**

```cpp
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <sunshine_core.h>

// Control inputs received from receiver (updated by Core 0 ESP-NOW callback)
struct CtrlInputs {
    uint8_t mode;
    int8_t  ctrl_x, ctrl_y, ctrl_theta;
    uint8_t ctrl_throttle;
    int8_t  rssi;
    uint32_t last_rx_ms;   // millis() at last reception
};

void    telemetry_init(void);

// Core 0 task entry point — pass to xTaskCreatePinnedToCore
void    telemetry_task(void *arg);

// Called from Core 1 to get latest control inputs (thread-safe)
CtrlInputs telemetry_get_ctrl(void);

// Called from Core 1 to queue a (SunshineInput, SunshineState) for TX
// Returns false if ring buffer full (oldest entry overwritten)
bool    telemetry_push(const SunshineInput *in, const SunshineState *state);
```

- [ ] **Step 2: Create src/telemetry.cpp**

```cpp
#include "telemetry.h"
#include "config.h"
#include "ring_buffer.h"
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <string.h>

// ── Control inputs (Core 0 writes, Core 1 reads) ──────────────────────────────
static CtrlInputs       latest_ctrl = {0};
static SemaphoreHandle_t ctrl_mutex;

// ── Telemetry ring buffer (Core 1 writes, Core 0 drains) ─────────────────────
// Each entry: SunshineInput (29B) + SunshineState (60B) = 89B × 40 = 3.56 KB
struct TelemetryEntry {
    SunshineInput  input;
    SunshineState  state;
};
static RingBuffer<TelemetryEntry, 40> telem_ring;

// ── ESP-NOW TX: accumulated frame ────────────────────────────────────────────
// Frame: frame_id(2) + type(1) + SunshineState(60) + SunshineInput[20](580)
static constexpr int ESPNOW_TELEM_SIZE = 643;
static uint8_t  tx_frame[ESPNOW_TELEM_SIZE];
static uint16_t tx_frame_id = 0;

// ── ESP-NOW RX callback (runs on Core 0) ──────────────────────────────────────
static void on_espnow_recv(const esp_now_recv_info_t *info,
                           const uint8_t *data, int len) {
    // Receiver → Brain: 8 bytes: seq_id(2) + type(1) + mode(1) + x(1) + y(1) + theta(1) + throttle(1)
    if (len != 8 || data[2] != 0x02) return;

    CtrlInputs c;
    c.mode          = data[3];
    c.ctrl_x        = (int8_t)data[4];
    c.ctrl_y        = (int8_t)data[5];
    c.ctrl_theta    = (int8_t)data[6];
    c.ctrl_throttle = data[7];
    c.rssi          = info->rx_ctrl->rssi;
    c.last_rx_ms    = (uint32_t)millis();

    xSemaphoreTake(ctrl_mutex, portMAX_DELAY);
    latest_ctrl = c;
    xSemaphoreGive(ctrl_mutex);
}

CtrlInputs telemetry_get_ctrl(void) {
    CtrlInputs c;
    xSemaphoreTake(ctrl_mutex, portMAX_DELAY);
    c = latest_ctrl;
    xSemaphoreGive(ctrl_mutex);
    return c;
}

bool telemetry_push(const SunshineInput *in, const SunshineState *state) {
    TelemetryEntry e;
    e.input = *in;
    e.state = *state;
    return telem_ring.push(e);
}

// ── Core 0 telemetry task ─────────────────────────────────────────────────────
// Drain ring buffer. Every 20 entries = one 50 Hz telemetry frame → ESP-NOW TX.

static esp_now_peer_info_t receiver_peer;
static int drain_count = 0;
static SunshineState first_state;  // state at start of current batch

void telemetry_task(void *) {
    for (;;) {
        TelemetryEntry entry;
        if (!telem_ring.pop(entry)) {
            vTaskDelay(1);  // ring empty — yield 1ms
            continue;
        }

        if (drain_count == 0) {
            // Capture state at start of the 20-input window
            first_state = entry.state;
            // Begin building frame
            tx_frame[0] = (uint8_t)(tx_frame_id & 0xFF);
            tx_frame[1] = (uint8_t)(tx_frame_id >> 8);
            tx_frame[2] = 0x01;  // type
            memcpy(tx_frame + 3, &first_state, sizeof(SunshineState));  // offset 3, 60B
        }

        // Copy input into frame at offset 63 + drain_count * 29
        memcpy(tx_frame + 63 + drain_count * sizeof(SunshineInput),
               &entry.input, sizeof(SunshineInput));
        drain_count++;

        if (drain_count == TELEMETRY_FRAMES) {
            // Send to receiver
            esp_now_send(receiver_peer.peer_addr, tx_frame, ESPNOW_TELEM_SIZE);
            tx_frame_id++;
            drain_count = 0;
        }
    }
}

void telemetry_init(void) {
    ctrl_mutex = xSemaphoreCreateMutex();

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_max_tx_power(84);

    esp_now_init();
    esp_now_register_recv_cb(on_espnow_recv);

    memset(&receiver_peer, 0, sizeof(receiver_peer));
    memcpy(receiver_peer.peer_addr, RECEIVER_MAC, 6);
    receiver_peer.channel = ESPNOW_CHANNEL;
    receiver_peer.encrypt = false;
    esp_now_add_peer(&receiver_peer);
}
```

- [ ] **Step 3: Commit**

```bash
git add sunshine_brain/src/telemetry.h sunshine_brain/src/telemetry.cpp
git commit -m "feat(brain): ESP-NOW telemetry task, control RX, ring buffer TX"
```

---

## Task 6: Ring Buffer

**Files:**
- Create: `sunshine_brain/src/ring_buffer.h`

- [ ] **Step 1: Create src/ring_buffer.h**

Lock-free single-producer single-consumer ring buffer (Core 1 pushes, Core 0 pops). Uses `volatile` indices — safe on this two-producer model.

```cpp
#pragma once
#include <stdint.h>
#include <stdbool.h>

template <typename T, uint32_t N>
class RingBuffer {
    static_assert((N & (N-1)) == 0, "N must be power of 2");
    T        buf[N];
    volatile uint32_t head = 0;  // written by producer
    volatile uint32_t tail = 0;  // written by consumer
public:
    // Push from producer (Core 1). If full, overwrites oldest entry.
    // Returns false if overwrite occurred (data loss warning).
    bool push(const T &item) {
        uint32_t h    = head;
        uint32_t next = (h + 1) & (N - 1);
        bool     full = (next == tail);
        buf[h & (N-1)] = item;
        __sync_synchronize();   // memory barrier before advancing head
        head = (h + 1) & (N - 1);
        if (full) tail = (tail + 1) & (N - 1);  // drop oldest
        return !full;
    }

    // Pop from consumer (Core 0). Returns false if empty.
    bool pop(T &out) {
        if (tail == head) return false;
        out  = buf[tail];
        __sync_synchronize();
        tail = (tail + 1) & (N - 1);
        return true;
    }

    uint32_t size(void) const {
        return (head - tail) & (N - 1);
    }
};
```

- [ ] **Step 2: Compile check**

```bash
cd sunshine_brain && pio run --environment bringup_3_telemetry 2>&1 | grep -c "error:"
```
Expected: `0`

- [ ] **Step 3: Commit**

```bash
git add sunshine_brain/src/ring_buffer.h
git commit -m "feat(brain): lock-free SPSC ring buffer for Core 1→Core 0 handoff"
```

---

## Task 7: 1 kHz Nav+Control Loop (Core 1)

**Files:**
- Create: `sunshine_brain/src/nav_control.h`
- Create: `sunshine_brain/src/nav_control.cpp`

- [ ] **Step 1: Create src/nav_control.h**

```cpp
#pragma once
void nav_control_init(void);
void nav_control_task(void *arg);  // pin to Core 1
```

- [ ] **Step 2: Create src/nav_control.cpp**

```cpp
#include "nav_control.h"
#include "config.h"
#include "bringup.h"
#include "sensors/adxl375.h"
#include "sensors/lis3mdl.h"
#include "dshot.h"
#include "telemetry.h"
#include <sunshine_core.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static SunshineState kf_state;
static SunshineVars  vars;
static uint32_t      overrun_count = 0;

void nav_control_init(void) {
    sunshine_state_init(&kf_state);
}

void nav_control_task(void *) {
    uint32_t t_next = micros();

    for (;;) {
        uint32_t t_start = micros();

        // ── 1. Build SunshineInput ──────────────────────────────────────────
        SunshineInput in = {};
        in.time_us = t_start;

        // Sensors
        Adxl375Sample a = adxl375_read();
        MagSample     m = lis3mdl_read();
        float         v = batt_read_v();

        in.accel_x = a.x;
        in.accel_y = a.y;
        in.accel_z = a.z;
        in.mag_x   = m.x;
        in.mag_y   = m.y;
        in.mag_z   = m.z;

        // Battery voltage → batt_offset
        float batt_offset_f = (v - 7.6f) / 0.0205f;
        in.batt_offset = (int8_t)(batt_offset_f < -127 ? -127 :
                                  batt_offset_f >  127 ?  127 :
                                  batt_offset_f);

        // eRPM as float16
        in.erpm_left  = sunshine_f32_to_f16(dshot_erpm_left());
        in.erpm_right = sunshine_f32_to_f16(dshot_erpm_right());

        // ── 2. Control inputs (from telemetry task) ─────────────────────────
        CtrlInputs ctrl = telemetry_get_ctrl();

        // Safety watchdog: if no control packet for 500ms, force DISABLED
        uint32_t now_ms = (uint32_t)millis();
        if (now_ms - ctrl.last_rx_ms > CTRL_WATCHDOG_MS) {
            in.mode = SUNSHINE_MODE_DISABLED;
        } else {
            in.mode = ctrl.mode;
        }
        in.ctrl_x        = ctrl.ctrl_x;
        in.ctrl_y        = ctrl.ctrl_y;
        in.ctrl_theta    = ctrl.ctrl_theta;
        in.ctrl_throttle = ctrl.ctrl_throttle;
        in.rssi          = ctrl.rssi;

        // ── 3. Previous DShot (from vars of last tick) ──────────────────────
        in.dshot_left_q  = dshot_quantize(vars.dshot_cmd_left);
        in.dshot_right_q = dshot_quantize(vars.dshot_cmd_right);

        // ── 4. sunshine_step ─────────────────────────────────────────────────
#if FEATURE_NAVIGATION
        sunshine_step(&in, &kf_state, &vars);
#else
        // Bringup 3: compute vars but zero dshot
        sunshine_step(&in, &kf_state, &vars);
        vars.dshot_cmd_left  = 0;
        vars.dshot_cmd_right = 0;
#endif

#if FORCE_DSHOT_ZERO
        vars.dshot_cmd_left  = 0;
        vars.dshot_cmd_right = 0;
#endif

        // ── 5. Apply outputs ─────────────────────────────────────────────────
#if FEATURE_DSHOT
        dshot_send((uint16_t)vars.dshot_cmd_left,
                   (uint16_t)vars.dshot_cmd_right);
#endif
        digitalWrite(PIN_LED, vars.led_on ? HIGH : LOW);

        // ── 6. Push to telemetry ring buffer ─────────────────────────────────
#if FEATURE_TELEMETRY
        vars.loop_overrun = (overrun_count > 0);
        telemetry_push(&in, &kf_state);
#endif

        // ── 7. Timing enforcement + overrun detection ────────────────────────
        uint32_t elapsed = micros() - t_start;
        if (elapsed > LOOP_INTERVAL_US) {
            overrun_count++;
            Serial.printf("OVERRUN: %u µs (count=%u)\n", elapsed, overrun_count);
        } else {
            overrun_count = 0;
            // Busy-wait until next tick
            t_next += LOOP_INTERVAL_US;
            while ((int32_t)(micros() - t_next) < 0) {}
        }
    }
}
```

- [ ] **Step 3: Commit**

```bash
git add sunshine_brain/src/nav_control.h sunshine_brain/src/nav_control.cpp
git commit -m "feat(brain): 1 kHz nav+control loop with sunshine_step integration"
```

---

## Task 8: Integration — main.cpp + Error Patterns + All Bringup Levels

**Files:**
- Modify: `sunshine_brain/src/main.cpp`

- [ ] **Step 1: Replace main.cpp with full integrated version**

```cpp
// src/main.cpp
// Sunshine Brain — main entry point
// Initialises all subsystems in order, handles init failures with LED patterns.
// Core 0: telemetry_task (FreeRTOS, priority 5)
// Core 1: nav_control_task (FreeRTOS, priority 10, pinned)

#include "bringup.h"
#include "config.h"
#include "sensors/adxl375.h"
#include "sensors/lis3mdl.h"
#include "dshot.h"
#include "telemetry.h"
#include "nav_control.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ── Error LED patterns ────────────────────────────────────────────────────────
// N fast blinks (50ms on/off) → 1s off → repeat
// Also prints error continuously to USB serial.
static void error_halt(int blink_count, const char *msg) {
    Serial.printf("FATAL INIT ERROR: %s (blink=%d)\n", msg, blink_count);
    while (true) {
        for (int i = 0; i < blink_count; i++) {
            digitalWrite(PIN_LED, HIGH); delay(50);
            digitalWrite(PIN_LED, LOW);  delay(50);
        }
        delay(1000);
        Serial.printf("ERROR: %s\n", msg);  // repeat for late USB attach
    }
}

void setup() {
    Serial.begin(921600);
    pinMode(PIN_LED, OUTPUT);
    analogReadResolution(12);

    // ── Level 1+ : init sensors ───────────────────────────────────────────────
    if (!adxl375_init()) error_halt(1, "ADXL375 init failed");
    if (!lis3mdl_init())  error_halt(2, "LIS3MDL init failed");

#if BRINGUP_LEVEL == 1
    // Bringup 1: CSV output only
    Serial.println("BRINGUP 1: accel_x,accel_y,accel_z,mag_x,mag_y,mag_z,batt_v");
    return;  // don't start tasks
#endif

    // ── Level 2+ : init DShot ─────────────────────────────────────────────────
#if FEATURE_DSHOT
    if (!dshot_init()) error_halt(3, "DShot arming failed");
#endif

#if BRINGUP_LEVEL == 2
    // Bringup 2: interactive DShot test only
    Serial.println("BRINGUP 2: DShot test. l <val>, r <val>, s, t");
    return;
#endif

    // ── Level 3+ : init telemetry ─────────────────────────────────────────────
#if FEATURE_TELEMETRY
    telemetry_init();
    xTaskCreatePinnedToCore(telemetry_task, "telemetry", 8192, nullptr, 5, nullptr, 0);
#endif

    // ── Level 3+ : start nav+control loop ────────────────────────────────────
    nav_control_init();
    xTaskCreatePinnedToCore(nav_control_task, "nav_ctrl", 8192, nullptr, 10, nullptr, 1);

    Serial.printf("Brain ready (bringup=%d)\n", BRINGUP_LEVEL);
}

void loop() {
    // Bringup 1 and 2 use loop(); production uses FreeRTOS tasks.
#if BRINGUP_LEVEL == 1
    Adxl375Sample a = adxl375_read();
    MagSample     m = lis3mdl_read();
    float         v = batt_read_v();
    Serial.printf("%d,%d,%d,%d,%d,%d,%.3f\n",
                  a.x, a.y, a.z, m.x, m.y, m.z, v);
    delay(10);

#elif BRINGUP_LEVEL == 2
    static char cmd[32];
    static int  ci = 0;
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            cmd[ci] = '\0'; ci = 0;
            if (cmd[0] == 'l') { int v = atoi(cmd+2); dshot_send((uint16_t)v, 1048); Serial.printf("L→%d\n",v); }
            else if (cmd[0] == 'r') { int v = atoi(cmd+2); dshot_send(1048,(uint16_t)v); Serial.printf("R→%d\n",v); }
            else if (cmd[0] == 's') { dshot_send(1048,1048); Serial.println("Stop"); }
            else if (cmd[0] == 't') { Serial.printf("eRPM L=%.0f R=%.0f\n",dshot_erpm_left(),dshot_erpm_right()); }
        } else if (ci < 31) { cmd[ci++] = c; }
    }
    dshot_send(1048, 1048);
    delay(1);

#else
    // Production / levels 3-4: all work in FreeRTOS tasks
    vTaskDelay(portMAX_DELAY);
#endif
}
```

- [ ] **Step 2: Build all environments**

```bash
cd sunshine_brain
pio run --environment bringup_1_sensors   2>&1 | grep -E "^(RAM|Flash|error)" | head -5
pio run --environment bringup_2_dshot     2>&1 | grep -E "^(RAM|Flash|error)" | head -5
pio run --environment bringup_3_telemetry 2>&1 | grep -E "^(RAM|Flash|error)" | head -5
pio run --environment bringup_4_navigation 2>&1 | grep -E "^(RAM|Flash|error)" | head -5
pio run --environment production          2>&1 | grep -E "^(RAM|Flash|error)" | head -5
```
Expected: all build successfully, `0` error lines.

- [ ] **Step 3: Flash bringup_3_telemetry + bringup test**

```bash
pio run --target upload --environment bringup_3_telemetry
```

With receiver running, open host app → Live tab → Connect. Verify:
1. Status indicators show both devices connected
2. `inputs.accel_z` plots ≈ +20 counts at rest
3. `inputs.mag_x` and `inputs.mag_y` are non-zero
4. `inputs.rssi` shows a plausible negative dBm value
5. Press W key → `inputs.ctrl_y` ramps up (LP filter visible in host app)
6. `frame_id` increments without large gaps

- [ ] **Step 4: Commit**

```bash
git add sunshine_brain/src/main.cpp
git commit -m "feat(brain): integrate all subsystems, full bringup level support"
```

---

## Post-Plan: Bringup Level 4 and Production

The remaining bringup levels require human-in-the-loop tuning documented in `docs/BRINGUP.md` and `docs/TUNING.md`. Refer to those documents once the host app and all firmware are running at bringup level 3.

**Level 4 checklist (navigation tuning, in `docs/TUNING.md`):**
1. Flash `bringup_4_navigation`
2. Spin robot slowly — verify `omega_from_accel` responds
3. Spin > 120 RPM — verify `derot_I`/`derot_Q` become near-constant (DC)
4. Tune `KF_R_ACCEL`, `KF_Q_OMEGA`, `KF_R_MAG`, `KF_Q_THETA` per TUNING.md
5. Verify LED appears stationary at a fixed heading

**Level 0 / production:**
1. Flash `production` environment
2. Verify MELTY mode translation in commanded direction

---

*End of brain plan. Next: `2026-05-26-sunshine-app.md`.*
