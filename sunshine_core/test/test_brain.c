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
