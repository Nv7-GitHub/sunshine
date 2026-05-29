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
    s->kf_P[0] = 100.0f;   /* high initial angle uncertainty           */
    s->kf_P[3] = 1.0f;    /* prior: std dev ≈ 1 rad/s (~9.5 RPM)      */
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
