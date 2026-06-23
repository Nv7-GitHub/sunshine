#pragma once
#include <stdint.h>
#include <stddef.h>

/* ── Schema version ────────────────────────────────────────────────────────
 * Bump whenever ANY field is added, removed, reordered, or resized in
 * SunshineInput, SunshineState, or SunshineVars.
 * New fields MUST be appended at the END of the struct — never insert. */
#define SUNSHINE_SCHEMA_VERSION  3U

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
#define ADXL_MAX_COUNTS     4082                /* ±200 g / 49 mg·LSB⁻¹  */
/* Min spin for the mag heading. The tracking band-pass is centred on the spin
 * frequency; its half-width is a FRACTION of that frequency (constant-Q), so the
 * lower band edge is ~0.75·fc. Below the minimum, that edge sinks toward the
 * slow-ESC-current band and the 2nd-order skirt (only ~6 dB/oct) no longer
 * rejects it well, and at low ω the tangential-accel inflation of omega_accel is
 * large — so the mag is gated off. 16π rad/s = 8 Hz spin = 480 RPM (lower edge
 * ≈ 6 Hz). (Hard-iron DC itself is killed exactly by the band-pass's zero at DC.) */
#define SUNSHINE_MAG_MIN_OMEGA  (16.0f * 3.14159265f)  /* ~480 RPM, rad/s  */

/* ── Kalman tuning (override with -D flag for tuning builds) ───────────── */
#ifndef KF_Q_THETA
#define KF_Q_THETA   1e-6f
#endif
#ifndef KF_Q_OMEGA
#define KF_Q_OMEGA   1e-2f
#endif
/* Accelerometer omega-measurement variance. The accel (ω = √(a_c/r)) is the rate
 * sensor at all times. Since the heading is recovered open-loop (mag_heading.c
 * band-passes the Earth sine at the accel-derived spin frequency, independent of
 * the estimate), the accel can't drag the heading into precession, so it is
 * trusted fully and kf_omega tracks omega_from_accel. */
#ifndef KF_R_ACCEL
#define KF_R_ACCEL   0.5f
#endif
#ifndef KF_R_MAG
#define KF_R_MAG     0.01f          /* open-loop mag heading is a clean absolute reference */
#endif

/* ── Magnetometer tracking band-pass (open-loop absolute heading) ──────────
 * The Earth field appears at the spin frequency; the body-fixed offset
 * (hard-iron + avg ESC current) is at DC; and 1 kHz low-power-mode sampling adds
 * tones well above the spin band (a strong one at fs/6 ≈ 167 Hz). A 2nd-order
 * RBJ band-pass CENTRED ON THE SPIN FREQUENCY (from omega_from_accel — a direct
 * accel measurement, NOT the heading-coupled kf_omega; see mag_heading.c) isolates
 * the Earth sine: it has a transmission zero at DC (kills hard-iron) and rolls off
 * above, rejecting the HF sampling tones — which a fixed filter can't do because
 * the 167 Hz tone is only ~2.5–4× the 40–66 Hz spin. heading = atan2 of the
 * band-passed axes (open-loop → cannot drift).
 *
 * The bandwidth is a FRACTION of the centre frequency (constant Q), NOT a fixed
 * Hz: the accelerometer's spin-rate error is fractional, so a fixed ±N Hz band
 * would lose the signal at high spin. Bench bias was +2..+12% with ~3% per-sample
 * noise; combat adds linear acceleration + impacts, so we budget conservatively
 * (~2× that ≈ 30%) and set half-bandwidth = fc/(2·MAG_BP_Q) ≈ 33% of fc at Q=1.5.
 * That keeps the true spin in the band even when the accel rate is biased, at the
 * cost of slightly weaker HF-tone rejection (a 4th-order band-pass would sharpen
 * it if needed). Coeffs are recomputed each tick from omega_from_accel (cheap). With
 * constant Q the group delay is a CONSTANT heading offset that theta_offset (the
 * driver zero) absorbs — no speed-dependent shift. Tunable: higher Q = narrower
 * = cleaner but riskier; lower Q = wider = more robust. */
#define MAG_BP_Q          1.5f    /* half-BW = fc/(2Q) ≈ 33% of spin freq          */
#define MAG_BP_MIN_FC_HZ  8.0f    /* clamp centre to the mag-valid speed (480 RPM) */

/* ── Control tuning ────────────────────────────────────────────────────── */
#define DRIFT_PLATEAU_WIDTH 0.35f   /* fraction of rotation at each +/- peak diff */
#define DRIFT_AMPLITUDE     0.40f   /* max diff as fraction of available headroom */
#define DRIFT_PHASE_OFFSET_RADS 0.0f /* fixed motor timing offset, rad             */
#define DRIFT_PHASE_LEAD_S  0.0f    /* ESC/traction lag compensation, seconds     */
#define THETA_RATE_RADS     3.14159265f  /* rad/s per full ctrl_theta              */
#define MAX_DSHOT_SPIN      DSHOT_MAX
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

/* SunshineState: filter history, 44 bytes packed.
 * APPEND-ONLY rule applies here too. */
typedef struct __attribute__((packed)) {
    float kf_theta;         /* Kalman angle estimate (rad, unwrapped)     */
    float kf_omega;         /* Kalman angular velocity estimate (rad/s)   */
    float kf_P[4];          /* 2×2 covariance, row-major [P00,P01,P10,P11]*/
    float theta_offset;     /* driver heading offset (rad)                */
    float mag_hp_x[2];      /* mag_x high-pass biquad state (2nd order)   */
    float mag_hp_y[2];      /* mag_y high-pass biquad state               */
} SunshineState;
/* static_assert(sizeof(SunshineState) == 44, ""); */

/* SunshineVars: derived variables, never telemetered, 56 bytes packed.
 * APPEND-ONLY: never insert, reorder, or resize existing fields. */
typedef struct __attribute__((packed)) {
    float   omega_from_accel;  /* rad/s, inflated during spinup            */
    float   mag_x_filt;        /* high-passed mag_x (µT); Earth-field sine  */
    float   mag_y_filt;        /* high-passed mag_y (µT)                    */
    float   mag_angle;         /* open-loop absolute heading atan2 (rad)    */
    float   est_theta;         /* = kf_theta                               */
    float   est_omega;         /* = kf_omega                               */
    float   dshot_cmd_left;    /* [0, 2047], pre-quantisation              */
    float   dshot_cmd_right;
    float   batt_voltage;      /* actual voltage (V)                       */
    float   erpm_left;         /* decoded from float16                     */
    float   erpm_right;
    float   centripetal_ms2;   /* sqrt(ax²+ay²)*ADXL_SCALE_MS2            */
    uint8_t led_on;            /* 1 when within ±3° of zero heading        */
    uint8_t accel_saturated;   /* 1 when centripetal > 280g equivalent     */
    uint8_t mag_valid;         /* 1 when est_omega > SUNSHINE_MAG_MIN_OMEGA*/
    uint8_t loop_overrun;      /* 1 when 1kHz tick exceeded 1000µs (HW)   */
    float   heading_deg;       /* robot heading [0, 360), matches LED zero */
} SunshineVars;

/* ── Public API ────────────────────────────────────────────────────────── */
#ifdef __cplusplus
extern "C" {
#endif

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

#ifdef __cplusplus
}
#endif
