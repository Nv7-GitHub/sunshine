/* src/brain.c */
#include "sunshine_core.h"
#include <string.h>
#include <math.h>

/* Forward declarations of internal functions */
void kalman_predict      (SunshineState *s, float dt);
void kalman_update_omega (SunshineState *s, float omega_meas, float r_accel);
void kalman_update_theta (SunshineState *s, float theta_meas);
void mag_heading_step    (const SunshineInput *in, SunshineState *s, SunshineVars *v);
void control_step        (const SunshineInput *in, SunshineState *s, SunshineVars *v);

#define ACCEL_SAT_THRESHOLD_MS2  (280.0f * 9.81f)   /* 280g */
#define DT                       0.001f

void sunshine_step(const SunshineInput *in, SunshineState *state, SunshineVars *vars) {
    /* -- Decode inputs ---------------------------------------------------- */
    float ax = sunshine_accel_to_ms2(in->accel_x);
    float ay = sunshine_accel_to_ms2(in->accel_y);
    float centripetal = sqrtf(ax*ax + ay*ay);
    vars->centripetal_ms2 = centripetal;
    /* Saturated if EITHER individual axis clips (e.g. during an impact that is
     * not centripetal) OR the centripetal magnitude approaches the ±200 g limit.
     * The 45° mounting makes the centripetal clip point 200g*√2≈283g, so 280g
     * is a conservative vector-magnitude guard.  Per-axis check catches hits. */
    vars->accel_saturated = (in->accel_x >=  ADXL_MAX_COUNTS || in->accel_x <= -ADXL_MAX_COUNTS ||
                             in->accel_y >=  ADXL_MAX_COUNTS || in->accel_y <= -ADXL_MAX_COUNTS ||
                             centripetal  >  ACCEL_SAT_THRESHOLD_MS2);
    vars->batt_voltage    = sunshine_batt_to_v(in->batt_offset);
    vars->erpm_left       = sunshine_f16_to_f32(in->erpm_left);
    vars->erpm_right      = sunshine_f16_to_f32(in->erpm_right);

    /* omega from centripetal: w = sqrt(a_c / r) */
    if (centripetal > 0.0f && !vars->accel_saturated)
        vars->omega_from_accel = sqrtf(centripetal / IMU_RADIUS_M);
    else
        vars->omega_from_accel = 0.0f;

    /* -- Kalman predict --------------------------------------------------- */
    kalman_predict(state, DT);

    /* The mag is the absolute reference only above the min spin rate. Use |kf_omega|
     * so it stays valid when the robot spins the other way (inverted): kf_omega is
     * SIGNED (see the accel update below), and a correct negative rate must not gate
     * the mag off. */
    vars->mag_valid = (fabsf(state->kf_omega) > SUNSHINE_MAG_MIN_OMEGA);

    /* -- Open-loop magnetometer absolute heading -> mag_angle ------------- */
    /* Runs BEFORE the rate update so state->spin_rate_lp (the SIGNED, LP-smoothed
     * mag rotation rate) and spin_freq_lp are fresh this tick. */
    mag_heading_step(in, state, vars);

    /* -- Kalman update - angular rate ------------------------------------- */
    /* The rate measurement is the ACCEL magnitude √(a_c/r) with the mag's SIGN
     * (spin_rate_lp — the accel can't sense CW vs CCW; schema v4).
     *
     * The v5 design drove the rate from the mag's own rotation rate
     * (spin_rate_lp) while mag-valid, reasoning that it is unbiased by linear
     * acceleration during translation. REVERTED (2026-08-15 steel-arena log):
     * every venue measured so far has AC magnetic interference tones inside the
     * spin band (a ~9.77 Hz harmonic comb at 19.5/29.3/39.1/48.8 Hz — the 102.4 ms
     * WiFi-beacon period — plus 50/100 Hz, 2–7 µT). These beat against the Earth
     * line and wobble mag_angle at |f_spin − f_tone| (3–13 Hz); DIFFERENTIATING
     * that wobble into a rate multiplies it by the beat frequency, so the mag-rate
     * path amplified it into ±10–20 rad/s of kf_omega wobble — the LED "bounces,
     * locks, bounces" in the arena (25.7° median wander stationary). The accel
     * rate is immune to magnetic interference; its once-per-rev translation wobble
     * is at the spin frequency (KF-attenuated) and its +2–12% bias is cancelled by
     * the strong mag angle anchor (KF_R_MAG=0.01 → measured 8° steady offset).
     * Measured on real logs (tools/replay + bounce metric): arena stationary
     * wander 25.7°→11.7° median (p90 143°→26°); home logs unchanged, translating
     * windows unchanged or better in BOTH venues — the v5 benefit never
     * materialised because the interference exists at home too. */
    if (!vars->accel_saturated && vars->omega_from_accel > 0.0f) {
        float signed_omega = (state->spin_rate_lp < 0.0f)
                             ? -vars->omega_from_accel : vars->omega_from_accel;
        kalman_update_omega(state, signed_omega, KF_R_ACCEL);
    }

    /* -- Kalman update - magnetometer (absolute heading reference) -------- */
    if (vars->mag_valid)
        kalman_update_theta(state, vars->mag_angle);

    vars->est_theta = state->kf_theta;
    vars->est_omega = state->kf_omega;

    /* -- Control ---------------------------------------------------------- */
    control_step(in, state, vars);

    /* loop_overrun is set by the hardware layer, not here */
    vars->loop_overrun = 0;
}

/* Serialisation: memcpy suffices because structs are __attribute__((packed))
 * and both sides are little-endian (ESP32-S3 + macOS/x86/ARM). */
void sunshine_input_serialize(const SunshineInput *in, uint8_t *buf) {
    memcpy(buf, in, sizeof(SunshineInput));
}

void sunshine_input_deserialize(const uint8_t *buf, SunshineInput *in) {
    memcpy(in, buf, sizeof(SunshineInput));
}

void sunshine_state_serialize(const SunshineState *state, uint8_t *buf) {
    memcpy(buf, state, sizeof(SunshineState));
}

void sunshine_state_deserialize(const uint8_t *buf, SunshineState *state) {
    memcpy(state, buf, sizeof(SunshineState));
}
