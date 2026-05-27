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
    /* -- Decode inputs ---------------------------------------------------- */
    float ax = sunshine_accel_to_ms2(in->accel_x);
    float ay = sunshine_accel_to_ms2(in->accel_y);
    float centripetal = sqrtf(ax*ax + ay*ay);
    vars->centripetal_ms2 = centripetal;
    vars->accel_saturated = centripetal > ACCEL_SAT_THRESHOLD_MS2;
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

    /* -- Kalman update - accelerometer ------------------------------------ */
    if (!vars->accel_saturated && vars->omega_from_accel > 0.0f)
        kalman_update_omega(state, vars->omega_from_accel);

    /* -- Synchronous demodulation -> mag_angle ---------------------------- */
    derot_filter_step(in, state, vars);

    /* -- Kalman update - magnetometer ------------------------------------- */
    vars->mag_valid = (state->kf_omega > SUNSHINE_MAG_MIN_OMEGA);
    if (vars->mag_valid)
        kalman_update_theta(state, vars->mag_angle);

    vars->est_theta = state->kf_theta;
    vars->est_omega = state->kf_omega;

    /* -- Control ---------------------------------------------------------- */
    control_step(in, state, vars);

    /* loop_overrun is set by the hardware layer, not here */
    vars->loop_overrun = false;
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
