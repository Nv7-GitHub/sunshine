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

    /* TANK forward: W key (ctrl_y=127) → tangential wheels: left fwd, right rev */
    in = make_input(SUNSHINE_MODE_TANK, 0, 0, 127, 0);
    control_step(&in, &s, &v);
    ASSERT(v.dshot_cmd_left  > DSHOT_NEUTRAL, "TANK fwd: left > neutral (forward)");
    ASSERT(v.dshot_cmd_right < DSHOT_NEUTRAL, "TANK fwd: right < neutral (reverse, tangential drive)");

    /* TANK reverse: S key (ctrl_y=-127) */
    in = make_input(SUNSHINE_MODE_TANK, 0, 0, -127, 0);
    control_step(&in, &s, &v);
    ASSERT(v.dshot_cmd_left  < DSHOT_NEUTRAL, "TANK rev: left < neutral");

    /* TANK neutral: no keys → both at neutral */
    in = make_input(SUNSHINE_MODE_TANK, 0, 0, 0, 0);
    control_step(&in, &s, &v);
    ASSERT_NEAR(v.dshot_cmd_left,  DSHOT_NEUTRAL, 1.0f, "TANK neutral: left = neutral");
    ASSERT_NEAR(v.dshot_cmd_right, DSHOT_NEUTRAL, 1.0f, "TANK neutral: right = neutral");

    /* TANK turn right: A/D (ctrl_x > 0) → both wheels drive the body spin */
    in = make_input(SUNSHINE_MODE_TANK, 0, 100, 0, 0);
    control_step(&in, &s, &v);
    ASSERT_NEAR(v.dshot_cmd_left, v.dshot_cmd_right, 1.0f, "TANK turn right: left≈right");
    ASSERT(v.dshot_cmd_left > DSHOT_NEUTRAL, "TANK turn right: both forward of neutral");

    /* MELTY: throttle>0, no translation → left≈right≈base */
    sunshine_state_init(&s);
    s.kf_theta = 0.0f;
    in = make_input(SUNSHINE_MODE_MELTY, 200, 0, 0, 0);
    control_step(&in, &s, &v);
    ASSERT_NEAR(v.dshot_cmd_left, v.dshot_cmd_right, 1.0f, "MELTY no-translation: left≈right");
    ASSERT(v.dshot_cmd_left > DSHOT_NEUTRAL, "MELTY throttle: outputs forward of neutral");

    /* MELTY low throttle must not fall into the bidirectional reverse range. */
    in = make_input(SUNSHINE_MODE_MELTY, 13, 0, 0, 0);
    control_step(&in, &s, &v);
    ASSERT(v.dshot_cmd_left >= DSHOT_NEUTRAL, "MELTY low throttle: left not reverse");
    ASSERT(v.dshot_cmd_right >= DSHOT_NEUTRAL, "MELTY low throttle: right not reverse");

    /* MELTY zero throttle means stopped while enabled, not disarmed/special DShot. */
    in = make_input(SUNSHINE_MODE_MELTY, 0, 0, 0, 0);
    control_step(&in, &s, &v);
    ASSERT_NEAR(v.dshot_cmd_left, DSHOT_NEUTRAL, 1.0f, "MELTY zero throttle: left neutral");
    ASSERT_NEAR(v.dshot_cmd_right, DSHOT_NEUTRAL, 1.0f, "MELTY zero throttle: right neutral");

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
