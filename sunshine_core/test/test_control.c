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

static float melty_halfdiff_at(float phase, uint8_t throttle) {
    SunshineState s;
    SunshineVars v;
    sunshine_state_init(&s);
    s.kf_theta = phase;
    SunshineInput in = make_input(SUNSHINE_MODE_MELTY, throttle, 127, 0, 0);
    control_step(&in, &s, &v);
    return 0.5f * (v.dshot_cmd_left - v.dshot_cmd_right);
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

    /* LED must fire EVERY revolution even at high spin. At 250 rad/s the heading
       advances ~14.3°/tick — larger than the ±3° (6°) window — so a fixed point-
       test would step over the window between 1 kHz samples and skip whole revs
       (the dot visibly vanishes). The window must widen to ≥ half the per-tick
       sweep so a sample always lands in it. */
    in = make_input(SUNSHINE_MODE_MELTY, 200, 0, 0, 0);
    const float DT_TEST = 0.001f, TWO_PI = 2.0f * 3.14159265f;
    /* Sweep a range of high spin rates; at each, the LED must never stay dark for
       more than ~1 revolution (the real "dot doesn't vanish" requirement — robust
       to where the per-tick samples fall relative to the window). */
    for (float omega = 110.0f; omega <= 349.0f; omega += 17.0f) {
        sunshine_state_init(&s);
        s.kf_omega = omega; s.theta_offset = 0.0f;
        float rev_ticks = TWO_PI / (omega * DT_TEST);     /* 1 kHz ticks per revolution */
        int last_on = -1, max_gap = 0;
        float ph = 0.0f;
        for (int i = 0; i < 8000; i++) {
            s.kf_theta = ph;
            control_step(&in, &s, &v);
            if (v.led_on) { if (last_on >= 0 && i - last_on > max_gap) max_gap = i - last_on; last_on = i; }
            ph += omega * DT_TEST;
            if (ph >= TWO_PI) ph -= TWO_PI;
        }
        ASSERT(last_on >= 0 && (float)max_gap < 1.5f * rev_ticks,
               "LED never dark for more than ~1 revolution at high spin");
    }

    /* MELTY drift waveform must be balanced over one rotation. A non-zero mean
       differential mostly loads one wheel/body side instead of producing a clean
       world-frame translation vector. */
    {
        const int N = 720;
        float sum = 0.0f;
        for (int i = 0; i < N; i++) {
            float phase = -3.14159265f + TWO_PI * ((float)i + 0.5f) / (float)N;
            sum += melty_halfdiff_at(phase, 100);
        }
        ASSERT_NEAR(sum / (float)N, 0.0f, 2.0f, "MELTY drift waveform has zero mean");
    }

    /* Opposite sides of the spin must command opposite differentials. This keeps
       the useful first harmonic while avoiding one-sided random-walk loading. */
    {
        const int N = 180;
        float max_abs_sum = 0.0f;
        for (int i = 0; i < N; i++) {
            float phase = -3.14159265f + TWO_PI * ((float)i + 0.5f) / (float)N;
            float a = melty_halfdiff_at(phase, 100);
            float b = melty_halfdiff_at(phase + 3.14159265f, 100);
            float e = fabsf(a + b);
            if (e > max_abs_sum) max_abs_sum = e;
        }
        ASSERT(max_abs_sum < 2.0f, "MELTY drift waveform is half-wave antisymmetric");
    }

    /* Differential authority must scale to the available symmetric headroom.
       At high spin throttle the old base-scaled diff clipped a side to DSHOT_MAX,
       distorting the waveform and hiding the requested profile from the ESC. */
    {
        sunshine_state_init(&s);
        s.kf_theta = 0.0f;
        in = make_input(SUNSHINE_MODE_MELTY, 240, 127, 0, 0);
        control_step(&in, &s, &v);
        ASSERT(v.dshot_cmd_left < DSHOT_MAX - 1.0f, "MELTY high throttle drift does not clip high side");
        ASSERT(v.dshot_cmd_right > DSHOT_NEUTRAL + 1.0f, "MELTY high throttle drift does not clip low side");
    }

    TEST_RESULTS();
}
