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
    /* P[0] (angle) = 100 (large); P[3] (omega) = 1 (std dev 1 rad/s, not zero) */
    ASSERT(s.kf_P[0] > 1.0f && s.kf_P[3] > 0.0f, "P initialised to non-zero positive values");
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
