# sunshine_core Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the pure C99 library that implements the IO layer structs, Kalman filter, synchronous demodulation magnetometer filter, and all three control modes — testable on any desktop machine.

**Architecture:** Pure C99 with no platform dependencies. CMake for desktop builds and unit tests. PlatformIO pulls sources directly via `lib_deps` path. The single entry point is `sunshine_step()` — a pure function (no side effects except mutating `*state`).

**Tech Stack:** C99, CMake 3.16+, CTest, Python 3 (for generating filter coefficients), math.h.

---

## File Structure

```
sunshine_core/
├── CMakeLists.txt
├── include/
│   └── sunshine_core.h       # ALL struct definitions, constants, API declarations
├── src/
│   ├── utils.c               # float16, wrap_to_pi, clampf, unit conversions
│   ├── kalman.c              # Kalman predict + omega update + theta update
│   ├── derot_filter.c        # Biquad cascade + synchronous demodulation step
│   ├── control.c             # trapezoid(), DISABLED/TANK/MELTY logic
│   └── brain.c               # sunshine_step(), sunshine_state_init(), serialisation
└── test/
    ├── test_runner.h         # Minimal assertion macros (no external deps)
    ├── test_utils.c
    ├── test_kalman.c
    ├── test_derot_filter.c
    ├── test_control.c
    └── test_brain.c
```

---

## Task 1: CMake Build System + Test Runner

**Files:**
- Create: `sunshine_core/CMakeLists.txt`
- Create: `sunshine_core/test/test_runner.h`

- [ ] **Step 1: Create CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.16)
project(sunshine_core C)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CORE_SOURCES
    src/utils.c
    src/kalman.c
    src/derot_filter.c
    src/control.c
    src/brain.c
)

add_library(sunshine_core STATIC ${CORE_SOURCES})
target_include_directories(sunshine_core PUBLIC include)
target_link_libraries(sunshine_core m)
target_compile_options(sunshine_core PRIVATE -Wall -Wextra -Werror)

enable_testing()
foreach(t utils kalman derot_filter control brain)
    add_executable(test_${t} test/test_${t}.c)
    target_link_libraries(test_${t} sunshine_core)
    add_test(NAME ${t} COMMAND test_${t})
endforeach()
```

- [ ] **Step 2: Create test/test_runner.h**

```c
#pragma once
#include <stdio.h>
#include <math.h>
static int _pass = 0, _fail = 0;
#define ASSERT(cond, msg) \
    do { if(cond){_pass++;printf("  ok  %s\n",(msg));} \
         else{_fail++;printf(" FAIL %s  [%s:%d]\n",(msg),__FILE__,__LINE__);} } while(0)
#define ASSERT_NEAR(a, b, tol, msg) \
    ASSERT(fabsf((float)(a)-(float)(b)) < (float)(tol), msg)
#define ASSERT_EQ(a, b, msg) ASSERT((a)==(b), msg)
#define TEST_RESULTS() \
    do { printf("\n%d passed, %d failed\n", _pass, _fail); \
         return _fail > 0 ? 1 : 0; } while(0)
```

- [ ] **Step 3: Verify CMake configures cleanly (no sources yet — expected link errors)**

```bash
cd sunshine_core && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
```
Expected: configures without error (link errors come at build time, not configure time).

- [ ] **Step 4: Commit**

```bash
git add sunshine_core/CMakeLists.txt sunshine_core/test/test_runner.h
git commit -m "feat(core): add CMake build system and test runner"
```

---

## Task 2: sunshine_core.h — All Types and Constants

**Files:**
- Create: `sunshine_core/include/sunshine_core.h`

- [ ] **Step 1: Create the header**

```c
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ── Schema version ────────────────────────────────────────────────────────
 * Bump whenever ANY field is added, removed, reordered, or resized in
 * SunshineInput, SunshineState, or SunshineVars.
 * New fields MUST be appended at the END of the struct — never insert. */
#define SUNSHINE_SCHEMA_VERSION  1U

/* ── Control modes ─────────────────────────────────────────────────────── */
#define SUNSHINE_MODE_DISABLED  0U
#define SUNSHINE_MODE_TANK      1U
#define SUNSHINE_MODE_MELTY     2U

/* ── Physical / sensor constants ───────────────────────────────────────── */
#define ADXL_SCALE_MS2      (49e-3f * 9.81f)   /* m/s² per ADXL375 count  */
#define MAG_SCALE_UT        0.058f              /* µT per LIS3MDL count    */
#define BATT_OFFSET_REF_V   7.6f               /* reference voltage (V)   */
#define BATT_SCALE_V        0.0205f             /* V per batt_offset LSB   */
#define IMU_RADIUS_M        0.011f              /* 11 mm from spin centre  */
#define SUNSHINE_MAG_MIN_OMEGA  (4.0f * 3.14159265f)  /* ~120 RPM, rad/s  */

/* ── Kalman tuning (override with -D flag for tuning builds) ───────────── */
#ifndef KF_Q_THETA
#define KF_Q_THETA   1e-6f
#endif
#ifndef KF_Q_OMEGA
#define KF_Q_OMEGA   1e-3f
#endif
#ifndef KF_R_ACCEL
#define KF_R_ACCEL   0.5f
#endif
#ifndef KF_R_MAG
#define KF_R_MAG     0.1f
#endif

/* ── 4th-order LP Butterworth, fc=1 Hz, fs=1000 Hz, two cascaded biquads ─
 * Generated by tools/gen_filter_coefficients.py                           */
#define LP4_S1_B0   9.846e-6f
#define LP4_S1_B1   1.969e-5f
#define LP4_S1_B2   9.846e-6f
#define LP4_S1_A1  (-1.99515f)
#define LP4_S1_A2   0.99519f
#define LP4_S2_B0   9.807e-6f
#define LP4_S2_B1   1.961e-5f
#define LP4_S2_B2   9.807e-6f
#define LP4_S2_A1  (-1.98836f)
#define LP4_S2_A2   0.98842f

/* ── Control tuning ────────────────────────────────────────────────────── */
#define DRIFT_PULSE_WIDTH   0.25f   /* fraction of rotation at peak diff   */
#define DRIFT_RAMP_WIDTH    0.10f   /* fraction for linear ramp            */
#define DRIFT_AMPLITUDE     0.40f   /* max diff as fraction of base        */
#define THETA_RATE_RADS     3.14159265f  /* rad/s per full ctrl_theta      */
#define MAX_DSHOT_SPIN      1500.0f
#define DSHOT_NEUTRAL       1048.0f
#define DSHOT_MAX           2047.0f
#define DSHOT_MIN           48.0f

/* ── IO layer structs ──────────────────────────────────────────────────── */

/* SunshineInput: 1 kHz sensor frame, 29 bytes packed.
 * APPEND-ONLY: never insert, reorder, or resize existing fields. */
typedef struct __attribute__((packed)) {
    uint32_t time_us;
    int16_t  accel_x;       /* ADXL375 raw counts; IMU at 45° to radial   */
    int16_t  accel_y;       /* centripetal + tangential both split here    */
    int16_t  accel_z;       /* vertical (~+20 cnts = 1g at rest)          */
    int16_t  mag_x;         /* LIS3MDL raw counts at ±16 Gauss            */
    int16_t  mag_y;
    int16_t  mag_z;
    uint16_t erpm_left;     /* IEEE-754 float16 bits                      */
    uint16_t erpm_right;
    int8_t   rssi;          /* ESP-NOW RSSI at brain (dBm)                */
    int8_t   ctrl_x;        /* [-127, 127]                                */
    int8_t   ctrl_y;
    int8_t   ctrl_theta;
    uint8_t  ctrl_throttle; /* [0, 255]                                   */
    int8_t   batt_offset;   /* relative to 7.6 V, 0.0205 V/LSB           */
    uint8_t  dshot_left_q;  /* DShot cmd from PREVIOUS tick, quantised    */
    uint8_t  dshot_right_q;
    uint8_t  mode;          /* SUNSHINE_MODE_*                            */
} SunshineInput;
/* static_assert(sizeof(SunshineInput) == 29, ""); */

/* SunshineState: filter history, 60 bytes packed.
 * APPEND-ONLY rule applies here too. */
typedef struct __attribute__((packed)) {
    float kf_theta;         /* Kalman angle estimate (rad, unwrapped)     */
    float kf_omega;         /* Kalman angular velocity estimate (rad/s)   */
    float kf_P[4];          /* 2×2 covariance, row-major [P00,P01,P10,P11]*/
    float theta_offset;     /* driver heading offset (rad)                */
    float derot_lp_I[4];    /* LP state for derotated I component         */
                            /* [s1_w0, s1_w1, s2_w0, s2_w1]              */
    float derot_lp_Q[4];    /* LP state for derotated Q component         */
} SunshineState;
/* static_assert(sizeof(SunshineState) == 60, ""); */

/* SunshineVars: derived variables, never telemetered, 52 bytes. */
typedef struct {
    float  omega_from_accel;  /* rad/s, inflated during spinup            */
    float  derot_I;           /* derotated+filtered mag I (µT)            */
    float  derot_Q;           /* derotated+filtered mag Q (µT)            */
    float  mag_angle;         /* atan2(Q,I) rad                           */
    float  est_theta;         /* = kf_theta                               */
    float  est_omega;         /* = kf_omega                               */
    float  dshot_cmd_left;    /* [0, 2000], pre-quantisation              */
    float  dshot_cmd_right;
    float  batt_voltage;      /* actual voltage (V)                       */
    float  erpm_left;         /* decoded from float16                     */
    float  erpm_right;
    float  centripetal_ms2;   /* sqrt(ax²+ay²)*ADXL_SCALE_MS2            */
    bool   led_on;
    bool   accel_saturated;   /* centripetal > 280g equivalent            */
    bool   mag_valid;         /* est_omega > SUNSHINE_MAG_MIN_OMEGA       */
    bool   loop_overrun;      /* hardware only                            */
} SunshineVars;

/* ── Public API ────────────────────────────────────────────────────────── */
void     sunshine_state_init(SunshineState *state);
void     sunshine_step(const SunshineInput *in, SunshineState *state, SunshineVars *vars_out);

void     sunshine_input_serialize  (const SunshineInput *in,    uint8_t *buf);
void     sunshine_input_deserialize(const uint8_t *buf,         SunshineInput *in);
void     sunshine_state_serialize  (const SunshineState *state, uint8_t *buf);
void     sunshine_state_deserialize(const uint8_t *buf,         SunshineState *state);

uint32_t sunshine_schema_version(void);

float    sunshine_accel_to_ms2(int16_t raw);
float    sunshine_mag_to_ut   (int16_t raw);
float    sunshine_batt_to_v   (int8_t  off);
float    sunshine_f16_to_f32  (uint16_t half);
uint16_t sunshine_f32_to_f16  (float f);
```

- [ ] **Step 2: Verify size assertions hold**

Add a temporary `main.c` in `test/` that includes the header and checks sizes with a compile-time assert. C99 doesn't have `static_assert` — use the trick below, then delete after confirming.

```c
/* test/check_sizes.c — delete after confirming */
#include "sunshine_core.h"
typedef char assert_input_size [sizeof(SunshineInput)  == 29 ? 1 : -1];
typedef char assert_state_size [sizeof(SunshineState)  == 60 ? 1 : -1];
int main(void) { return 0; }
```

```bash
cd sunshine_core/build && cmake .. && make test_utils 2>&1 | head -5
# (will fail at link — that's OK, confirms sizes compile)
gcc -Iinclude test/check_sizes.c -o /tmp/check_sizes && echo "sizes OK"
```
Expected: `sizes OK`

- [ ] **Step 3: Commit**

```bash
git add sunshine_core/include/sunshine_core.h
git commit -m "feat(core): define SunshineInput, SunshineState, SunshineVars, constants"
```

---

## Task 3: utils.c — float16, wrap_to_pi, unit conversions

**Files:**
- Create: `sunshine_core/src/utils.c`
- Create: `sunshine_core/test/test_utils.c`

- [ ] **Step 1: Write the failing test**

```c
/* test/test_utils.c */
#include "test_runner.h"
#include "sunshine_core.h"
#include <string.h>

int main(void) {
    /* float16 round-trip */
    float vals[] = {0.0f, 1.0f, -1.0f, 3.14f, 0.001f, 65000.0f, -300.5f};
    for (int i = 0; i < 7; i++) {
        float rt = sunshine_f16_to_f32(sunshine_f32_to_f16(vals[i]));
        ASSERT_NEAR(rt, vals[i], fabsf(vals[i]) * 0.005f + 1e-5f,
                    "float16 round-trip within 0.5%");
    }
    ASSERT_EQ(sunshine_f32_to_f16(0.0f), 0, "zero encodes to 0");

    /* Unit conversions */
    ASSERT_NEAR(sunshine_accel_to_ms2(0),    0.0f,   0.001f, "accel 0 → 0 m/s²");
    ASSERT_NEAR(sunshine_accel_to_ms2(100),  100.0f * ADXL_SCALE_MS2, 0.001f, "accel scale");
    ASSERT_NEAR(sunshine_mag_to_ut(1000),    1000.0f * MAG_SCALE_UT,  0.001f, "mag scale");
    ASSERT_NEAR(sunshine_batt_to_v(0),       7.6f,   0.001f, "batt offset 0 → 7.6V");
    ASSERT_NEAR(sunshine_batt_to_v(127),     7.6f + 127*0.0205f, 0.001f, "batt max");
    ASSERT_NEAR(sunshine_batt_to_v(-127),    7.6f - 127*0.0205f, 0.001f, "batt min");

    TEST_RESULTS();
}
```

- [ ] **Step 2: Run to confirm failure**

```bash
cd sunshine_core/build && cmake .. -DCMAKE_BUILD_TYPE=Debug && make test_utils 2>&1 | tail -5
```
Expected: compile error — `sunshine_f16_to_f32` undefined.

- [ ] **Step 3: Implement utils.c**

```c
/* src/utils.c */
#include "sunshine_core.h"
#include <string.h>
#include <math.h>

float sunshine_f16_to_f32(uint16_t h) {
    uint32_t sign =  (h >> 15) & 0x1;
    uint32_t exp  =  (h >> 10) & 0x1F;
    uint32_t mant =   h        & 0x3FF;
    uint32_t f;
    if (exp == 0) {
        if (mant == 0) { f = sign << 31; }
        else {
            exp = 1;
            while (!(mant & 0x400)) { mant <<= 1; exp--; }
            mant &= 0x3FF;
            f = (sign << 31) | ((exp + 127 - 15) << 23) | (mant << 13);
        }
    } else if (exp == 31) {
        f = (sign << 31) | 0x7F800000u | (mant << 13);
    } else {
        f = (sign << 31) | ((exp + 127 - 15) << 23) | (mant << 13);
    }
    float r; memcpy(&r, &f, 4); return r;
}

uint16_t sunshine_f32_to_f16(float val) {
    uint32_t x; memcpy(&x, &val, 4);
    uint32_t sign = (x >> 31) & 0x1;
    int32_t  exp  = (int32_t)((x >> 23) & 0xFF) - 127 + 15;
    uint32_t mant = (x & 0x7FFFFFu) >> 13;
    if (exp <= 0)  return (uint16_t)(sign << 15);
    if (exp >= 31) return (uint16_t)((sign << 15) | 0x7C00u);
    return (uint16_t)((sign << 15) | ((uint32_t)exp << 10) | mant);
}

float sunshine_accel_to_ms2(int16_t raw) { return (float)raw * ADXL_SCALE_MS2; }
float sunshine_mag_to_ut   (int16_t raw) { return (float)raw * MAG_SCALE_UT;   }
float sunshine_batt_to_v   (int8_t  off) { return BATT_OFFSET_REF_V + (float)off * BATT_SCALE_V; }
uint32_t sunshine_schema_version(void)   { return SUNSHINE_SCHEMA_VERSION; }
```

- [ ] **Step 4: Run tests**

```bash
cd sunshine_core/build && make test_utils && ./test_utils
```
Expected: all `ok`, exit 0.

- [ ] **Step 5: Commit**

```bash
git add sunshine_core/src/utils.c sunshine_core/test/test_utils.c
git commit -m "feat(core): implement float16 conversion and unit helpers"
```

---

## Task 4: kalman.c — Predict + Two Update Steps

**Files:**
- Create: `sunshine_core/src/kalman.c`
- Create: `sunshine_core/test/test_kalman.c`

- [ ] **Step 1: Write the failing test**

```c
/* test/test_kalman.c */
#include "test_runner.h"
#include "sunshine_core.h"
#include <math.h>
#include <string.h>

/* expose internal functions for testing */
void kalman_predict      (SunshineState *s, float dt);
void kalman_update_omega (SunshineState *s, float omega_meas);
void kalman_update_theta (SunshineState *s, float theta_meas);

static SunshineState make_state(float theta, float omega) {
    SunshineState s;
    sunshine_state_init(&s);
    s.kf_theta = theta;
    s.kf_omega = omega;
    return s;
}

int main(void) {
    /* state_init sets P to large diagonal */
    SunshineState s;
    sunshine_state_init(&s);
    ASSERT(s.kf_P[0] > 1.0f && s.kf_P[3] > 1.0f, "P initialised to large values");
    ASSERT_NEAR(s.kf_P[1], 0.0f, 1e-6f, "P off-diagonal = 0");

    /* predict: theta advances by omega*dt */
    s = make_state(1.0f, 10.0f);
    kalman_predict(&s, 0.001f);
    ASSERT_NEAR(s.kf_theta, 1.0f + 10.0f * 0.001f, 1e-5f, "predict: theta += omega*dt");
    ASSERT_NEAR(s.kf_omega, 10.0f, 1e-5f, "predict: omega unchanged");

    /* predict: P grows (uncertainty increases without measurement) */
    sunshine_state_init(&s);
    s.kf_P[0] = 0.01f; s.kf_P[3] = 0.01f;
    float p00_before = s.kf_P[0];
    kalman_predict(&s, 0.001f);
    ASSERT(s.kf_P[0] > p00_before, "predict: P[0,0] grows");

    /* update omega: pulls kf_omega toward measurement */
    s = make_state(0.0f, 5.0f);
    s.kf_P[0] = 1.0f; s.kf_P[1] = 0.0f;
    s.kf_P[2] = 0.0f; s.kf_P[3] = 1.0f;
    kalman_update_omega(&s, 20.0f);
    ASSERT(s.kf_omega > 5.0f && s.kf_omega < 20.0f, "omega update: between prior and meas");

    /* update theta: pulls kf_theta toward measurement, handles wrap */
    s = make_state(3.1f, 0.0f);
    s.kf_P[0] = 1.0f; s.kf_P[1] = 0.0f;
    s.kf_P[2] = 0.0f; s.kf_P[3] = 1.0f;
    kalman_update_theta(&s, -3.1f);  /* equivalent angle, wrapped innovation */
    ASSERT_NEAR(s.kf_theta, 3.1f, 0.2f, "theta update: small correction for near-pi wrap");

    /* DC gain test: feed constant omega measurement repeatedly, should converge */
    sunshine_state_init(&s);
    for (int i = 0; i < 2000; i++) {
        kalman_predict(&s, 0.001f);
        kalman_update_omega(&s, 100.0f);
    }
    ASSERT_NEAR(s.kf_omega, 100.0f, 1.0f, "omega converges to constant measurement");

    TEST_RESULTS();
}
```

- [ ] **Step 2: Run to confirm failure**

```bash
cd sunshine_core/build && make test_kalman 2>&1 | tail -3
```
Expected: undefined reference to `kalman_predict`.

- [ ] **Step 3: Implement kalman.c**

```c
/* src/kalman.c */
#include "sunshine_core.h"
#include <math.h>
#include <string.h>

#define M_PI_F 3.14159265f

static float wrap_to_pi(float a) {
    while (a >  M_PI_F) a -= 2.0f * M_PI_F;
    while (a < -M_PI_F) a += 2.0f * M_PI_F;
    return a;
}

void sunshine_state_init(SunshineState *s) {
    memset(s, 0, sizeof(*s));
    s->kf_P[0] = 100.0f;   /* high initial angle uncertainty   */
    s->kf_P[3] = 100.0f;   /* high initial omega uncertainty   */
}

/* Predict step: F = [[1,dt],[0,1]] */
void kalman_predict(SunshineState *s, float dt) {
    s->kf_theta += s->kf_omega * dt;
    float p00 = s->kf_P[0], p01 = s->kf_P[1];
    float p10 = s->kf_P[2], p11 = s->kf_P[3];
    s->kf_P[0] = p00 + dt*(p10 + p01) + dt*dt*p11 + KF_Q_THETA;
    s->kf_P[1] = p01 + dt*p11;
    s->kf_P[2] = p10 + dt*p11;
    s->kf_P[3] = p11 + KF_Q_OMEGA;
}

/* Update with omega measurement: H = [0, 1] */
void kalman_update_omega(SunshineState *s, float omega_meas) {
    float inn   = omega_meas - s->kf_omega;
    float S_inv = 1.0f / (s->kf_P[3] + KF_R_ACCEL);
    float K0    = s->kf_P[1] * S_inv;   /* P[0,1]/S */
    float K1    = s->kf_P[3] * S_inv;   /* P[1,1]/S */
    s->kf_theta += K0 * inn;
    s->kf_omega += K1 * inn;
    float hP0 = s->kf_P[2], hP1 = s->kf_P[3]; /* H*P = second row */
    s->kf_P[0] -= K0 * hP0;
    s->kf_P[1] -= K0 * hP1;
    s->kf_P[2] -= K1 * hP0;
    s->kf_P[3] -= K1 * hP1;
}

/* Update with theta measurement: H = [1, 0] */
void kalman_update_theta(SunshineState *s, float theta_meas) {
    float inn   = wrap_to_pi(theta_meas - s->kf_theta);
    float S_inv = 1.0f / (s->kf_P[0] + KF_R_MAG);
    float K0    = s->kf_P[0] * S_inv;   /* P[0,0]/S */
    float K1    = s->kf_P[2] * S_inv;   /* P[1,0]/S */
    s->kf_theta += K0 * inn;
    s->kf_omega += K1 * inn;
    float hP0 = s->kf_P[0], hP1 = s->kf_P[1]; /* H*P = first row */
    s->kf_P[0] -= K0 * hP0;
    s->kf_P[1] -= K0 * hP1;
    s->kf_P[2] -= K1 * hP0;
    s->kf_P[3] -= K1 * hP1;
}
```

- [ ] **Step 4: Run tests**

```bash
cd sunshine_core/build && make test_kalman && ./test_kalman
```
Expected: all `ok`.

- [ ] **Step 5: Commit**

```bash
git add sunshine_core/src/kalman.c sunshine_core/test/test_kalman.c
git commit -m "feat(core): implement Kalman filter (predict + omega + theta updates)"
```

---

## Task 5: derot_filter.c — Synchronous Demodulation + 4th-order LP

**Files:**
- Create: `sunshine_core/src/derot_filter.c`
- Create: `sunshine_core/test/test_derot_filter.c`
- Create: `tools/gen_filter_coefficients.py`

- [ ] **Step 1: Generate and verify filter coefficients**

Create `tools/gen_filter_coefficients.py`:
```python
#!/usr/bin/env python3
"""Generate 4th-order LP Butterworth coefficients, fc=1 Hz, fs=1000 Hz."""
from scipy.signal import butter
import numpy as np

sos = butter(4, 1.0 / (1000.0 / 2.0), btype='low', output='sos')
print("/* Generated by tools/gen_filter_coefficients.py */")
for i, (b0,b1,b2,a0,a1,a2) in enumerate(sos, 1):
    assert abs(a0 - 1.0) < 1e-10, "a0 must be 1"
    print(f"/* Section {i} */")
    print(f"#define LP4_S{i}_B0  {b0:.6e}f")
    print(f"#define LP4_S{i}_B1  {b1:.6e}f")
    print(f"#define LP4_S{i}_B2  {b2:.6e}f")
    print(f"#define LP4_S{i}_A1 ({a1:.6f}f)")
    print(f"#define LP4_S{i}_A2  {a2:.6f}f")

# Verify DC gain = 1 for each section
for i, (b0,b1,b2,a0,a1,a2) in enumerate(sos, 1):
    dc = (b0+b1+b2)/(1+a1+a2)
    print(f"/* Section {i} DC gain: {dc:.8f} (should be 1.0) */")
```

```bash
pip install scipy numpy
python3 tools/gen_filter_coefficients.py
```

Expected output (copy the `#define` lines into `sunshine_core.h`, replacing the current LP4_* values):
```
#define LP4_S1_B0   9.8424e-07f     ← update these in sunshine_core.h
#define LP4_S1_A1  (-1.995154f)
...
```

Update the LP4_* constants in `sunshine_core/include/sunshine_core.h` with the output values.

- [ ] **Step 2: Write the failing test**

```c
/* test/test_derot_filter.c */
#include "test_runner.h"
#include "sunshine_core.h"
#include <math.h>
#include <string.h>

void derot_filter_step(const SunshineInput *in, SunshineState *s, SunshineVars *v);

int main(void) {
    SunshineState s;
    SunshineVars  v;
    SunshineInput in;
    memset(&in, 0, sizeof(in));
    sunshine_state_init(&s);

    /* Feed DC input — after settling, output should equal input (DC gain = 1) */
    float dc_val = 50.0f;  /* µT */
    int16_t raw = (int16_t)(dc_val / MAG_SCALE_UT);
    in.mag_x = raw; in.mag_y = 0; in.mag_z = 0;
    s.kf_theta = 0.0f;  /* derotation at angle 0: I_raw = mag_x, Q_raw = 0 */
    for (int i = 0; i < 5000; i++) {
        derot_filter_step(&in, &s, &v);
    }
    ASSERT_NEAR(v.derot_I, dc_val, 0.5f, "LP filter DC gain ≈ 1 after settling");
    ASSERT_NEAR(v.derot_Q, 0.0f,   0.1f, "LP Q output ≈ 0 for DC-only Q input");

    /* At fc (1 Hz) input to the filter, gain should be -3dB ≈ 0.707
     * This means feeding a 1 Hz sinusoid into the derotated domain.
     * We simulate this by feeding mag_x = A*cos(2π*1*n/1000) with angle=0.
     * After settling, RMS output / RMS input ≈ 0.707 */
    sunshine_state_init(&s);
    s.kf_theta = 0.0f;
    memset(&in, 0, sizeof(in));
    float amp = 50.0f;
    double rms_in = 0, rms_out = 0;
    int N = 3000;
    for (int n = 0; n < N; n++) {
        float sig = amp * cosf(2.0f * 3.14159f * 1.0f * n / 1000.0f);
        in.mag_x = (int16_t)(sig / MAG_SCALE_UT);
        derot_filter_step(&in, &s, &v);
        if (n > 2000) {  /* skip transient */
            rms_in  += sig * sig;
            rms_out += v.derot_I * v.derot_I;
        }
    }
    float gain = sqrtf((float)(rms_out / rms_in));
    ASSERT_NEAR(gain, 0.707f, 0.05f, "LP gain at fc ≈ -3dB (0.707)");

    /* mag_angle: at derot_I = X, derot_Q = Y, angle = atan2(Y,X) */
    sunshine_state_init(&s);
    s.kf_theta = 0.0f;
    in.mag_x = (int16_t)(30.0f / MAG_SCALE_UT);
    in.mag_y = (int16_t)(40.0f / MAG_SCALE_UT);
    for (int i = 0; i < 5000; i++) derot_filter_step(&in, &s, &v);
    ASSERT_NEAR(v.mag_angle, atan2f(40.0f, 30.0f), 0.05f, "mag_angle = atan2(Q,I)");

    TEST_RESULTS();
}
```

- [ ] **Step 3: Implement derot_filter.c**

```c
/* src/derot_filter.c */
#include "sunshine_core.h"
#include <math.h>

static float biquad(float x, float *w,
                    float b0, float b1, float b2,
                    float a1, float a2) {
    float wn = x - a1*w[0] - a2*w[1];
    float y  = b0*wn + b1*w[0] + b2*w[1];
    w[1] = w[0];
    w[0] = wn;
    return y;
}

static float lp4(float x, float *state) {
    /* state[0..1] = section 1, state[2..3] = section 2 */
    float y1 = biquad(x,       state,   LP4_S1_B0, LP4_S1_B1, LP4_S1_B2, LP4_S1_A1, LP4_S1_A2);
    float y2 = biquad(y1, state + 2,    LP4_S2_B0, LP4_S2_B1, LP4_S2_B2, LP4_S2_A1, LP4_S2_A2);
    return y2;
}

void derot_filter_step(const SunshineInput *in, SunshineState *s, SunshineVars *v) {
    float theta = s->kf_theta + s->theta_offset;
    float c  =  cosf(theta);
    float ss = -sinf(theta);   /* note: Q uses -sin for correct rotation */

    /* Convert to µT then derotate */
    float mx = (float)in->mag_x * MAG_SCALE_UT;
    float my = (float)in->mag_y * MAG_SCALE_UT;
    float I_raw =  c*mx - ss*my;   /* = mx*cos(θ) + my*sin(θ) */
    float Q_raw =  ss*mx + c*my;   /* = -mx*sin(θ) + my*cos(θ) */

    v->derot_I   = lp4(I_raw, s->derot_lp_I);
    v->derot_Q   = lp4(Q_raw, s->derot_lp_Q);
    v->mag_angle = atan2f(v->derot_Q, v->derot_I);
}
```

- [ ] **Step 4: Run tests**

```bash
cd sunshine_core/build && make test_derot_filter && ./test_derot_filter
```
Expected: all `ok` (settling tests may take a few seconds to compute).

- [ ] **Step 5: Commit**

```bash
git add sunshine_core/src/derot_filter.c sunshine_core/test/test_derot_filter.c \
        tools/gen_filter_coefficients.py
git commit -m "feat(core): implement synchronous demodulation + 4th-order LP filter"
```

---

## Task 6: control.c — DISABLED, TANK, MELTY

**Files:**
- Create: `sunshine_core/src/control.c`
- Create: `sunshine_core/test/test_control.c`

- [ ] **Step 1: Write the failing test**

```c
/* test/test_control.c */
#include "test_runner.h"
#include "sunshine_core.h"
#include <math.h>
#include <string.h>

void control_step(const SunshineInput *in, SunshineState *s, SunshineVars *v);

static SunshineInput make_input(uint8_t mode, uint8_t throttle,
                                 int8_t x, int8_t y, int8_t theta) {
    SunshineInput in; memset(&in, 0, sizeof(in));
    in.mode = mode; in.ctrl_throttle = throttle;
    in.ctrl_x = x; in.ctrl_y = y; in.ctrl_theta = theta;
    return in;
}

int main(void) {
    SunshineState s; SunshineVars v;
    sunshine_state_init(&s);

    /* DISABLED always zeroes outputs regardless of other inputs */
    SunshineInput in = make_input(SUNSHINE_MODE_DISABLED, 200, 100, 100, 50);
    control_step(&in, &s, &v);
    ASSERT_NEAR(v.dshot_cmd_left,  0.0f, 1e-6f, "DISABLED: left = 0");
    ASSERT_NEAR(v.dshot_cmd_right, 0.0f, 1e-6f, "DISABLED: right = 0");
    ASSERT_EQ(v.led_on, false, "DISABLED: LED off");

    /* TANK forward: throttle=255 → both wheels max forward */
    in = make_input(SUNSHINE_MODE_TANK, 255, 0, 0, 0);
    control_step(&in, &s, &v);
    ASSERT(v.dshot_cmd_left  > DSHOT_NEUTRAL, "TANK fwd: left > neutral");
    ASSERT(v.dshot_cmd_right > DSHOT_NEUTRAL, "TANK fwd: right > neutral");
    ASSERT_NEAR(v.dshot_cmd_left, v.dshot_cmd_right, 1.0f, "TANK straight: left=right");

    /* TANK reverse */
    in = make_input(SUNSHINE_MODE_TANK, 0, 0, 0, 0);
    control_step(&in, &s, &v);
    ASSERT(v.dshot_cmd_left  < DSHOT_NEUTRAL, "TANK rev: left < neutral");

    /* TANK turn right: ctrl_x > 0 → left faster than right */
    in = make_input(SUNSHINE_MODE_TANK, 200, 100, 0, 0);
    control_step(&in, &s, &v);
    ASSERT(v.dshot_cmd_left > v.dshot_cmd_right, "TANK turn right: left > right");

    /* MELTY: throttle>0, no translation → left≈right≈base */
    sunshine_state_init(&s);
    s.kf_theta = 0.0f;
    in = make_input(SUNSHINE_MODE_MELTY, 200, 0, 0, 0);
    control_step(&in, &s, &v);
    ASSERT_NEAR(v.dshot_cmd_left, v.dshot_cmd_right, 1.0f, "MELTY no-translation: left≈right");
    ASSERT(v.dshot_cmd_left > 0, "MELTY throttle: outputs > 0");

    /* MELTY: theta_offset accumulates with ctrl_theta */
    sunshine_state_init(&s);
    float offset_before = s.theta_offset;
    in = make_input(SUNSHINE_MODE_MELTY, 0, 0, 0, 127);
    control_step(&in, &s, &v);
    ASSERT(s.theta_offset != offset_before, "MELTY: theta_offset changes with ctrl_theta");

    /* LED: on at angle 0 ± 3 deg */
    sunshine_state_init(&s);
    s.kf_theta = 0.0f; s.theta_offset = 0.0f;
    in = make_input(SUNSHINE_MODE_MELTY, 100, 0, 0, 0);
    control_step(&in, &s, &v);
    ASSERT_EQ(v.led_on, true, "LED on at angle 0");

    s.kf_theta = 0.1f;   /* 5.7°, outside ±3° */
    control_step(&in, &s, &v);
    ASSERT_EQ(v.led_on, false, "LED off at 5.7°");

    TEST_RESULTS();
}
```

- [ ] **Step 2: Run to confirm failure**

```bash
cd sunshine_core/build && make test_control 2>&1 | tail -3
```

- [ ] **Step 3: Implement control.c**

```c
/* src/control.c */
#include "sunshine_core.h"
#include <math.h>

#define M_PI_F 3.14159265f

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static float wrap_to_pi(float a) {
    while (a >  M_PI_F) a -= 2.0f * M_PI_F;
    while (a < -M_PI_F) a += 2.0f * M_PI_F;
    return a;
}

/* Trapezoidal wave: +1 at |phase|<half_flat, linear ramp, -1 at bottom */
static float trapezoid(float phase, float pulse_width, float ramp_width) {
    float ap        = fabsf(phase);
    float half_flat = pulse_width * M_PI_F;
    float half_edge = (pulse_width + ramp_width) * M_PI_F;
    if (ap <= half_flat)  return  1.0f;
    if (ap >= half_edge)  return -1.0f;
    return 1.0f - 2.0f * (ap - half_flat) / (ramp_width * M_PI_F);
}

static float map_to_dshot(float v) {
    /* v in [-1, 1]: positive = forward, negative = reverse (AM32 3D mode) */
    if (v >= 0.0f) return DSHOT_NEUTRAL + v * (DSHOT_MAX - DSHOT_NEUTRAL);
    else           return DSHOT_NEUTRAL + v * (DSHOT_NEUTRAL - DSHOT_MIN);
}

void control_step(const SunshineInput *in, SunshineState *s, SunshineVars *v) {
    float robot_angle = s->kf_theta + s->theta_offset;
    v->led_on = fabsf(wrap_to_pi(robot_angle)) < (3.0f * M_PI_F / 180.0f);

    if (in->mode == SUNSHINE_MODE_DISABLED) {
        v->dshot_cmd_left  = 0.0f;
        v->dshot_cmd_right = 0.0f;
        return;
    }

    if (in->mode == SUNSHINE_MODE_TANK) {
        float fwd  = ((float)in->ctrl_throttle / 127.5f) - 1.0f;
        float turn = (float)in->ctrl_x / 127.0f;
        v->dshot_cmd_left  = map_to_dshot(clampf(fwd + turn, -1.0f, 1.0f));
        v->dshot_cmd_right = map_to_dshot(clampf(fwd - turn, -1.0f, 1.0f));
        return;
    }

    /* MELTY */
    s->theta_offset += ((float)in->ctrl_theta / 127.0f) * THETA_RATE_RADS * 0.001f;

    float base      = ((float)in->ctrl_throttle / 255.0f) * MAX_DSHOT_SPIN;
    float cx        = (float)in->ctrl_x;
    float cy        = (float)in->ctrl_y;
    float drive_dir = atan2f(cy, cx);
    float drive_mag = sqrtf(cx*cx + cy*cy) / 127.0f;
    drive_mag       = clampf(drive_mag, 0.0f, 1.0f);

    float phase = wrap_to_pi(robot_angle - drive_dir);
    float diff  = trapezoid(phase, DRIFT_PULSE_WIDTH, DRIFT_RAMP_WIDTH)
                  * drive_mag * DRIFT_AMPLITUDE * base;

    v->dshot_cmd_left  = clampf(base + diff, 0.0f, DSHOT_MAX);
    v->dshot_cmd_right = clampf(base - diff, 0.0f, DSHOT_MAX);
}
```

- [ ] **Step 4: Run tests**

```bash
cd sunshine_core/build && make test_control && ./test_control
```
Expected: all `ok`.

- [ ] **Step 5: Commit**

```bash
git add sunshine_core/src/control.c sunshine_core/test/test_control.c
git commit -m "feat(core): implement DISABLED/TANK/MELTY control with trapezoid wave"
```

---

## Task 7: brain.c — sunshine_step() + Serialisation

**Files:**
- Create: `sunshine_core/src/brain.c`
- Create: `sunshine_core/test/test_brain.c`

- [ ] **Step 1: Write the failing test**

```c
/* test/test_brain.c */
#include "test_runner.h"
#include "sunshine_core.h"
#include <string.h>
#include <math.h>

int main(void) {
    SunshineState s, s2;
    SunshineVars  v;
    SunshineInput in;
    memset(&in, 0, sizeof(in));
    sunshine_state_init(&s);

    /* schema version is positive */
    ASSERT(sunshine_schema_version() > 0, "schema version > 0");

    /* sunshine_step: DISABLED always gives zero DShot */
    in.mode = SUNSHINE_MODE_DISABLED;
    in.ctrl_throttle = 255;
    sunshine_step(&in, &s, &v);
    ASSERT_NEAR(v.dshot_cmd_left,  0.0f, 1e-5f, "step DISABLED → dshot_left=0");
    ASSERT_NEAR(v.dshot_cmd_right, 0.0f, 1e-5f, "step DISABLED → dshot_right=0");

    /* accel_saturated flag: centripetal > 280g threshold */
    in.mode = SUNSHINE_MODE_DISABLED;
    in.accel_x = 20000; in.accel_y = 20000;  /* far above max */
    sunshine_step(&in, &s, &v);
    ASSERT_EQ(v.accel_saturated, true, "accel_saturated when |accel| >> 280g");

    in.accel_x = 100; in.accel_y = 100;
    sunshine_step(&in, &s, &v);
    ASSERT_EQ(v.accel_saturated, false, "not saturated at low accel");

    /* mag_valid flag: valid only when est_omega > SUNSHINE_MAG_MIN_OMEGA */
    sunshine_state_init(&s);
    s.kf_omega = 1.0f;  /* below threshold */
    in.accel_x = 0; in.accel_y = 0;
    sunshine_step(&in, &s, &v);
    ASSERT_EQ(v.mag_valid, false, "mag invalid at low speed");

    sunshine_state_init(&s);
    s.kf_omega = SUNSHINE_MAG_MIN_OMEGA + 1.0f;
    sunshine_step(&in, &s, &v);
    ASSERT_EQ(v.mag_valid, true, "mag valid above threshold");

    /* Serialisation round-trip: SunshineInput */
    memset(&in, 0, sizeof(in));
    in.time_us = 12345; in.accel_x = -500; in.mag_y = 300;
    in.ctrl_throttle = 200; in.mode = SUNSHINE_MODE_MELTY;
    uint8_t buf[sizeof(SunshineInput)];
    SunshineInput in2;
    sunshine_input_serialize(&in, buf);
    sunshine_input_deserialize(buf, &in2);
    ASSERT_EQ(in2.time_us,       in.time_us,       "input serial: time_us");
    ASSERT_EQ(in2.accel_x,       in.accel_x,       "input serial: accel_x");
    ASSERT_EQ(in2.ctrl_throttle, in.ctrl_throttle, "input serial: throttle");
    ASSERT_EQ(in2.mode,          in.mode,           "input serial: mode");

    /* Serialisation round-trip: SunshineState */
    sunshine_state_init(&s);
    s.kf_theta = 1.23f; s.kf_omega = 45.6f; s.theta_offset = 0.5f;
    uint8_t sbuf[sizeof(SunshineState)];
    sunshine_state_serialize(&s, sbuf);
    sunshine_state_deserialize(sbuf, &s2);
    ASSERT_NEAR(s2.kf_theta,     s.kf_theta,     1e-5f, "state serial: kf_theta");
    ASSERT_NEAR(s2.kf_omega,     s.kf_omega,     1e-5f, "state serial: kf_omega");
    ASSERT_NEAR(s2.theta_offset, s.theta_offset, 1e-5f, "state serial: theta_offset");

    /* Determinism: same input + state → same output */
    sunshine_state_init(&s); sunshine_state_init(&s2);
    memset(&in, 0, sizeof(in));
    in.mode = SUNSHINE_MODE_TANK; in.ctrl_throttle = 100;
    sunshine_step(&in, &s,  &v);
    SunshineVars v2;
    sunshine_state_init(&s2);
    sunshine_step(&in, &s2, &v2);
    ASSERT_NEAR(v2.dshot_cmd_left, v.dshot_cmd_left,   1e-5f, "deterministic: left");
    ASSERT_NEAR(v2.dshot_cmd_right, v.dshot_cmd_right, 1e-5f, "deterministic: right");

    TEST_RESULTS();
}
```

- [ ] **Step 2: Run to confirm failure**

```bash
cd sunshine_core/build && make test_brain 2>&1 | tail -3
```

- [ ] **Step 3: Implement brain.c**

```c
/* src/brain.c */
#include "sunshine_core.h"
#include <string.h>
#include <math.h>

/* Forward declarations of internal functions */
void kalman_predict      (SunshineState *s, float dt);
void kalman_update_omega (SunshineState *s, float omega_meas);
void kalman_update_theta (SunshineState *s, float theta_meas);
void derot_filter_step   (const SunshineInput *in, SunshineState *s, SunshineVars *v);
void control_step        (const SunshineInput *in, SunshineState *s, SunshineVars *v);

#define ACCEL_SAT_THRESHOLD_MS2  (280.0f * 9.81f)   /* 280g */
#define DT                       0.001f

void sunshine_step(const SunshineInput *in, SunshineState *state, SunshineVars *vars) {
    /* ── Decode inputs ────────────────────────────────────────────────── */
    float ax = sunshine_accel_to_ms2(in->accel_x);
    float ay = sunshine_accel_to_ms2(in->accel_y);
    float centripetal = sqrtf(ax*ax + ay*ay);
    vars->centripetal_ms2 = centripetal;
    vars->accel_saturated = centripetal > ACCEL_SAT_THRESHOLD_MS2;
    vars->batt_voltage    = sunshine_batt_to_v(in->batt_offset);
    vars->erpm_left       = sunshine_f16_to_f32(in->erpm_left);
    vars->erpm_right      = sunshine_f16_to_f32(in->erpm_right);

    /* omega from centripetal: ω = sqrt(a_c / r) */
    if (centripetal > 0.0f && !vars->accel_saturated)
        vars->omega_from_accel = sqrtf(centripetal / IMU_RADIUS_M);
    else
        vars->omega_from_accel = 0.0f;

    /* ── Kalman predict ───────────────────────────────────────────────── */
    kalman_predict(state, DT);

    /* ── Kalman update — accelerometer ───────────────────────────────── */
    if (!vars->accel_saturated && vars->omega_from_accel > 0.0f)
        kalman_update_omega(state, vars->omega_from_accel);

    /* ── Synchronous demodulation → mag_angle ────────────────────────── */
    derot_filter_step(in, state, vars);

    /* ── Kalman update — magnetometer ────────────────────────────────── */
    vars->mag_valid = (state->kf_omega > SUNSHINE_MAG_MIN_OMEGA);
    if (vars->mag_valid)
        kalman_update_theta(state, vars->mag_angle);

    vars->est_theta = state->kf_theta;
    vars->est_omega = state->kf_omega;

    /* ── Control ─────────────────────────────────────────────────────── */
    control_step(in, state, vars);

    /* loop_overrun is set by the hardware layer, not here */
    vars->loop_overrun = false;
}

/* Serialisation: memcpy suffices because structs are __attribute__((packed))
 * and both sides are little-endian (ESP32-S3 + macOS/x86/ARM). */
void sunshine_input_serialize  (const SunshineInput *in,    uint8_t *buf) { memcpy(buf, in,    sizeof(SunshineInput)); }
void sunshine_input_deserialize(const uint8_t *buf,         SunshineInput *in) { memcpy(in, buf, sizeof(SunshineInput)); }
void sunshine_state_serialize  (const SunshineState *state, uint8_t *buf) { memcpy(buf, state, sizeof(SunshineState)); }
void sunshine_state_deserialize(const uint8_t *buf,         SunshineState *state) { memcpy(state, buf, sizeof(SunshineState)); }
```

- [ ] **Step 4: Run all tests**

```bash
cd sunshine_core/build && make && ctest --output-on-failure
```
Expected:
```
Test project .../sunshine_core/build
    Start 1: utils
1/5 Test #1: utils ............  Passed
    Start 2: kalman
2/5 Test #2: kalman ...........  Passed
    ...
5/5 Test #5: brain ............  Passed
100% tests passed, 0 tests failed
```

- [ ] **Step 5: Commit**

```bash
git add sunshine_core/src/brain.c sunshine_core/test/test_brain.c
git commit -m "feat(core): implement sunshine_step() entry point and serialisation"
```

---

*End of Plan 1. Proceed to `2026-05-26-sunshine-receiver.md` next (no sunshine_core dependency), then `2026-05-26-sunshine-brain.md`, then `2026-05-26-sunshine-app.md`.*
