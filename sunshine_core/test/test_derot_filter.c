/* test/test_derot_filter.c */
#include "test_runner.h"
#include "sunshine_core.h"
#include <math.h>
#include <string.h>

void derot_filter_step(const SunshineInput *in, SunshineState *s, SunshineVars *v);

int main(void) {
    SunshineState s;
    SunshineVars  v;
    SunshineInput in;
    memset(&in, 0, sizeof(in));
    sunshine_state_init(&s);

    /* Feed DC input — after settling, output should equal input (DC gain = 1) */
    float dc_val = 50.0f;  /* µT */
    int16_t raw = (int16_t)(dc_val / MAG_SCALE_UT);
    in.mag_x = raw; in.mag_y = 0; in.mag_z = 0;
    s.kf_theta = 0.0f;  /* derotation at angle 0: I_raw = mag_x, Q_raw = 0 */
    for (int i = 0; i < 5000; i++) {
        derot_filter_step(&in, &s, &v);
    }
    ASSERT_NEAR(v.derot_I, dc_val, 0.5f, "LP filter DC gain ≈ 1 after settling");
    ASSERT_NEAR(v.derot_Q, 0.0f,   0.1f, "LP Q output ≈ 0 for DC-only Q input");

    /* At fc (1 Hz) input to the filter, gain should be -3dB ≈ 0.707 */
    sunshine_state_init(&s);
    s.kf_theta = 0.0f;
    memset(&in, 0, sizeof(in));
    float amp = 50.0f;
    double rms_in = 0, rms_out = 0;
    int N = 3000;
    for (int n = 0; n < N; n++) {
        float sig = amp * cosf(2.0f * 3.14159f * 1.0f * n / 1000.0f);
        in.mag_x = (int16_t)(sig / MAG_SCALE_UT);
        derot_filter_step(&in, &s, &v);
        if (n > 2000) {  /* skip transient */
            rms_in  += sig * sig;
            rms_out += v.derot_I * v.derot_I;
        }
    }
    float gain = sqrtf((float)(rms_out / rms_in));
    ASSERT_NEAR(gain, 0.707f, 0.05f, "LP gain at fc ≈ -3dB (0.707)");

    /* mag_angle: at derot_I = X, derot_Q = Y, angle = atan2(Y,X) */
    sunshine_state_init(&s);
    s.kf_theta = 0.0f;
    in.mag_x = (int16_t)(30.0f / MAG_SCALE_UT);
    in.mag_y = (int16_t)(40.0f / MAG_SCALE_UT);
    for (int i = 0; i < 5000; i++) derot_filter_step(&in, &s, &v);
    ASSERT_NEAR(v.mag_angle, atan2f(40.0f, 30.0f), 0.05f, "mag_angle = atan2(Q,I)");

    TEST_RESULTS();
}
