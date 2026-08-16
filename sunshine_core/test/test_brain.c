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
    ASSERT_NEAR(v.dshot_cmd_left,  0.0f, 1e-5f, "step DISABLED -> dshot_left=0");
    ASSERT_NEAR(v.dshot_cmd_right, 0.0f, 1e-5f, "step DISABLED -> dshot_right=0");

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

    /* Regression (steel-arena bounce): an AC interference tone NEAR the spin
     * frequency (in the band-pass) beats against the Earth line and wobbles
     * mag_angle at |f_spin - f_tone|. The Kalman rate must come from the ACCEL,
     * not from differentiating the mag angle — the v5 mag-rate path amplified
     * the beat into ±10-20 rad/s of kf_omega wobble (LED bouncing). With the
     * accel rate, kf_omega must stay near the true rate with small ripple. */
    {
        sunshine_state_init(&s);
        memset(&in, 0, sizeof(in));
        in.mode = SUNSHINE_MODE_MELTY;
        float w  = 2.0f * 3.14159265f * 28.0f;      /* true spin 28 Hz */
        float ac = w * w * IMU_RADIUS_M;
        in.accel_x = (int16_t)(ac / ADXL_SCALE_MS2 * 0.7071f);
        in.accel_y = (int16_t)(ac / ADXL_SCALE_MS2 * 0.7071f);
        float wsum = 0.0f, wsq = 0.0f; int M = 0;
        for (int i = 0; i < 8000; i++) {
            float ti = i * 0.001f;
            float ph = w * ti;                       /* Earth line, 22 µT   */
            float pt = 2.0f * 3.14159265f * 29.3f * ti;  /* interferer, 7 µT */
            in.mag_x = (int16_t)((-95.0f + 22.0f*cosf(ph) + 7.0f*cosf(pt)) / MAG_SCALE_UT);
            in.mag_y = (int16_t)((103.0f - 22.0f*sinf(ph) - 7.0f*sinf(pt)) / MAG_SCALE_UT);
            sunshine_step(&in, &s, &v);
            if (i >= 5000) { wsum += s.kf_omega; wsq += s.kf_omega*s.kf_omega; M++; }
        }
        float mean = wsum / M, var = wsq / M - mean*mean;
        float ripple = sqrtf(var > 0 ? var : 0);
        ASSERT_NEAR(mean, w, 0.05f * w, "kf_omega tracks true rate under interference");
        ASSERT(ripple < 2.0f, "kf_omega ripple < 2 rad/s with in-band mag interferer");
    }

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

    /* Determinism: same input + state -> same output */
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
