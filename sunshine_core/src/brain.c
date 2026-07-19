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

    /* -- Kalman update - accelerometer ------------------------------------ */
    /* The accelerometer is the rate sensor at all times: |ω| = √(a_c/r). But the
       accel gives only the MAGNITUDE — it cannot sense CW vs CCW. The spin SIGN
       comes from the magnetometer (state->spin_rate_lp, updated in mag_heading.c),
       which sees the true rotation sense and so reverses correctly when the robot is
       inverted. Copy that sign onto the accel rate before the update, so kf_omega —
       and hence the predicted kf_theta — winds the correct way in either orientation.
       Before the mag has locked (spin_rate_lp≈0) the sign defaults to +, matching the
       old behaviour; once the mag is valid it sets the true sign within ~50 ms.
       The heading is recovered open-loop by mag_heading.c (band-pass centred on this
       accel rate, independent of the estimate), so the accel can't precess it. */
    if (!vars->accel_saturated && vars->omega_from_accel > 0.0f) {
        float signed_omega = (state->spin_rate_lp < 0.0f)
                             ? -vars->omega_from_accel : vars->omega_from_accel;
        kalman_update_omega(state, signed_omega, KF_R_ACCEL);
    }

    /* -- Open-loop magnetometer absolute heading -> mag_angle ------------- */
    mag_heading_step(in, state, vars);

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
