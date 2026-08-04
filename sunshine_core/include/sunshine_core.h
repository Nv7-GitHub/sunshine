#pragma once
#include <stdint.h>
#include <stddef.h>

/* ── Schema version ────────────────────────────────────────────────────────
 * Bump whenever ANY field is added, removed, reordered, or resized in
 * SunshineInput, SunshineState, or SunshineVars.
 * New fields MUST be appended at the END of the struct — never insert.
 *
 * v5: batt_offset widened int8 → int16 (finer battery resolution) and
 *     SunshineState.spin_freq_lp appended (band-pass centre LP, see mag_heading.c).
 *     batt_offset is the ONE historical exception to append-only (a mid-struct
 *     resize); the host log reader remaps it per schema_version (replay.rs
 *     read_input) so all pre-v5 logs still parse. Live/wire format follows the
 *     current struct on both ends (brain + app rebuilt together). */
#define SUNSHINE_SCHEMA_VERSION  5U

/* ── Control modes ─────────────────────────────────────────────────────── */
#define SUNSHINE_MODE_DISABLED  0U
#define SUNSHINE_MODE_TANK      1U
#define SUNSHINE_MODE_MELTY     2U

/* ── Physical / sensor constants ───────────────────────────────────────── */
#define ADXL_SCALE_MS2      (49e-3f * 9.81f)   /* m/s² per ADXL375 count  */
#define MAG_SCALE_UT        0.058f              /* µT per LIS3MDL count    */
#define BATT_OFFSET_REF_V   7.6f               /* reference voltage (V)   */
/* batt_offset is int16 (schema v5): 1 mV/LSB, so telemetry no longer discretises
 * the battery (the 12-bit ADC ≈ 2.4 mV/LSB + the 6 Hz LP in batt_read_v are now
 * the only quantisers). ±32.7 V range is far beyond the 2S span; the brain clamps.
 * BATT_SCALE_V_LEGACY is the v4 int8 step, kept only so the host can decode the
 * batt_offset in pre-v5 logs (replay.rs remaps old int8 → new int16 LSBs). */
#define BATT_SCALE_V        0.001f              /* V per batt_offset LSB (int16)   */
#define BATT_SCALE_V_LEGACY 0.0205f             /* v4 int8 step, for old-log decode */
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
/* ── Drivetrain / motor, for the MELTY wheel-speed cap (control.c) ─────────
 * PER-BUILD parameters, read off the mechanical design and the motor nameplate.
 * They are NOT fitted to any log: a different robot changes exactly these values
 * and nothing else in the control path.
 *
 * Why they exist: the AM32 ESC is an open-loop VOLTAGE SOURCE — it applies a duty
 * cycle, it does not close a speed loop — so a wheel's steady no-load speed is a
 * known function of (duty × V_batt). That is what makes the relation invertible,
 * and the cap in control.c is exactly that inverse: given the wheel speed the
 * ground demands, solve for the DShot value whose no-load speed matches it.
 *
 * The failure mode being prevented: without the cap the wheels run 1.3–2.3× faster
 * than the rolling speed the body rate demands. The surplus is stored kinetic
 * energy in the wheels, and every time the robot touches down it is dumped into an
 * impulsive traction spike that throws the robot back into the air — the vertical
 * bounce.
 *
 * MOTOR_KV_RPM_PER_V is the 1100 NAMEPLATE and must NOT be derated to match the
 * ~6% speed deficit seen under load. That deficit is the I·R drop across the
 * winding, i.e. the slip DOING ITS JOB: the cap commands a NO-LOAD speed and the
 * load then pulls the actual speed below it, and that difference is the
 * torque-producing slip. Deriving KV from loaded data would cancel the very margin
 * the cap is supposed to grant.
 *
 * MOTOR_POLE_PAIRS is unused by the cap arithmetic. It exists for the eRPM model
 * (eRPM = mechanical rpm × pole pairs) that the brain-side eRPM filter uses to
 * build its command-derived ceiling.
 *
 * WHEEL_SLIP_ALLOW_MS is THE tuning knob, and it is a fixed slip SPEED, not a
 * percentage of rolling speed. Traction force is I = (V_cmd − backEMF)/R, so a
 * fixed slip speed is a fixed slip VOLTAGE and therefore a fixed current — a fixed
 * available torque at any spin rate. A flat percentage instead would starve the
 * robot of torque at low spin and still permit the runaway at high spin. */
#define WHEEL_RADIUS_M      0.022f   /* wheel rolling radius, m              */
#define WHEEL_CENTER_M      0.0405f  /* wheel contact patch to spin axis, m  */
#define MOTOR_KV_RPM_PER_V  1100.0f  /* nameplate, rpm/V (no-load)           */
#define MOTOR_POLE_PAIRS    7        /* 14-pole motor -> 7 pole pairs        */
#define WHEEL_SLIP_ALLOW_MS 1.0f     /* allowed slip at the contact patch, m/s */

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
/* Band-pass CENTRE low-pass (schema v5). The centre was previously retuned every
 * tick from the INSTANTANEOUS omega_from_accel. While translating, linear body
 * acceleration adds a once-per-rev component to the accel, so omega_from_accel
 * wobbles at the spin frequency — and a band-pass whose coefficients wobble tick-
 * to-tick is TIME-VARYING, which injects heading wobble (the LTI "mis-centre only
 * attenuates" argument holds only for a FIXED filter). Measured on real logs: the
 * recovered-heading rate error was 37–100 rad/s. Low-passing the centre to ~1.5 Hz
 * makes the filter quasi-LTI and cuts that to 3–5 rad/s (~90%), because the true
 * spin rate changes slowly (~sub-Hz) while the corruption is at the ~15-25 Hz spin
 * band. spinup_lag.py: the true rate stays inside the ±33% pass band ~98.7% of the
 * mag-valid time; the ~1.3% is brief impact glitches, cutoff-independent. The LP is
 * re-seeded below the mag threshold (mag_heading.c) so a fast spin-up carries no
 * lag over a stop. Higher = tracks spin-up faster but rejects less wobble; lower =
 * smoother heading but laggier centre. Loop-independent (uses only the accel rate),
 * so it does NOT reintroduce the kf_omega false-lock. Tunable at bringup Level 4. */
#define MAG_BP_FC_LP_HZ   1.5f    /* band-pass centre LP cutoff, Hz (nav loop = 1 kHz) */
/* LP cutoff for the SIGNED mag rotation rate (spin_rate_lp). Since schema v5 this is
 * the Kalman's rate measurement whenever the mag is valid (mag_heading.c / brain.c) —
 * it is unbiased by linear acceleration, so it fixes the translation heading swings
 * the accel magnitude caused. Trade-off knob: higher = snappier tracking of fast spin
 * changes (impact slowdown, spin-up) but a noisier steady heading; lower = smoother
 * but laggier on transients (the mag_angle update still re-anchors, so the lag is
 * momentary). 3.2 Hz preserves the pre-v5 coefficient (0.02 @ 1 kHz). Bringup Level 4. */
#define MAG_SPIN_RATE_LP_HZ 3.2f  /* SIGNED mag-rate LP cutoff, Hz */

/* ── Control tuning ────────────────────────────────────────────────────── */
#define DRIFT_PLATEAU_WIDTH 0.35f   /* fraction of rotation at each +/- peak diff */
#define DRIFT_AMPLITUDE     0.40f   /* max diff as fraction of available headroom */
#define DRIFT_PHASE_OFFSET_RADS 0.0f /* fixed motor timing offset, rad             */
/* ESC/traction lag compensation. PER-BUILD: measured, not designed — re-run
 * tools/replay/translation_lag.py on a translation log for any new robot
 * (procedure: BRINGUP.md Level 5). On the 2026-07-20 translation2 log the
 * DShot→eRPM differential is a pure ~20 ms TIME delay (18–24 ms across 24
 * windows, BOTH spin directions; the lock-in phase fits omega·tau with ~0
 * constant term, so DRIFT_PHASE_OFFSET_RADS stays 0). Minus ~3 ms median-5
 * eRPM telemetry lag → ~17 ms physical. Uncompensated this rotated the
 * translation force 110–150° at 1000–1300 RPM, which is why the robot wobbled
 * instead of translating. Signed kf_omega handles inverted operation. */
#define DRIFT_PHASE_LEAD_S  0.018f  /* ESC/traction lag compensation, seconds     */
#define THETA_RATE_RADS     3.14159265f  /* rad/s per full ctrl_theta              */
#define MAX_DSHOT_SPIN      DSHOT_MAX
#define DSHOT_NEUTRAL       1048.0f
#define DSHOT_MAX           2047.0f
#define DSHOT_MIN           48.0f

/* ── IO layer structs ──────────────────────────────────────────────────── */

/* SunshineInput: 1 kHz sensor frame, 30 bytes packed (schema v5).
 * APPEND-ONLY: never insert, reorder, or resize existing fields. The lone
 * exception is batt_offset, widened int8→int16 at v5 (see SUNSHINE_SCHEMA_VERSION);
 * the host reader remaps pre-v5 int8 batt per schema_version. */
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
    int16_t  batt_offset;   /* v5: relative to 7.6 V, 0.001 V/LSB (int16) */
    uint8_t  dshot_left_q;  /* DShot cmd from PREVIOUS tick, quantised    */
    uint8_t  dshot_right_q;
    uint8_t  mode;          /* SUNSHINE_MODE_*                            */
} SunshineInput;
/* static_assert(sizeof(SunshineInput) == 30, ""); */

/* SunshineState: filter history, 52 bytes packed.
 * APPEND-ONLY rule applies here too. */
typedef struct __attribute__((packed)) {
    float kf_theta;         /* Kalman angle estimate (rad, unwrapped)     */
    float kf_omega;         /* Kalman angular velocity estimate (rad/s)   */
    float kf_P[4];          /* 2×2 covariance, row-major [P00,P01,P10,P11]*/
    float theta_offset;     /* driver heading offset (rad)                */
    float mag_hp_x[2];      /* mag_x high-pass biquad state (2nd order)   */
    float mag_hp_y[2];      /* mag_y high-pass biquad state               */
    /* Spin-direction recovery (schema v4). The accel gives only |ω| (centripetal
     * magnitude), so kf_omega's SIGN must come from the magnetometer, which sees
     * the true rotation sense — otherwise the heading counter-rotates when the
     * robot is inverted (flip → same chassis spin, opposite world spin). */
    float mag_ang_prev;     /* previous mag_angle, for its rotation-rate sign  */
    float spin_rate_lp;     /* low-passed SIGNED mag rotation rate (rad/s)      */
    /* Band-pass CENTRE frequency LP (schema v5). Low-passed UNSIGNED spin rate
     * (rad/s) used to centre the mag band-pass — smooths the once-per-rev
     * translation corruption of omega_from_accel. See mag_heading.c / MAG_BP_FC_LP_HZ. */
    float spin_freq_lp;
} SunshineState;
/* static_assert(sizeof(SunshineState) == 56, ""); */

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
float    sunshine_batt_to_v   (int16_t off);   /* v5: int16, 0.001 V/LSB */
float    sunshine_f16_to_f32  (uint16_t half);
uint16_t sunshine_f32_to_f16  (float f);

#ifdef __cplusplus
}
#endif
