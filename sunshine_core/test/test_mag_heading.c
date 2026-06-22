/* test/test_mag_heading.c — open-loop magnetometer heading (HP + atan2) */
#include "test_runner.h"
#include "sunshine_core.h"
#include <math.h>
#include <string.h>

void mag_heading_step(const SunshineInput *in, SunshineState *s, SunshineVars *v);

static float wrap_pi(float a) { return remainderf(a, 2.0f * 3.14159265f); }

int main(void) {
    SunshineState s;
    SunshineVars  v;
    SunshineInput in;
    memset(&in, 0, sizeof(in));
    sunshine_state_init(&s);
    memset(&v, 0, sizeof(v));   /* omega_from_accel==0 → kf_omega(0) fallback, fc clamps */

    /* DC is blocked: the band-pass has a zero at DC, so a constant field → ~0. */
    in.mag_x = (int16_t)(50.0f / MAG_SCALE_UT);
    in.mag_y = 0; in.mag_z = 0;
    for (int i = 0; i < 20000; i++) mag_heading_step(&in, &s, &v);
    ASSERT_NEAR(v.mag_x_filt, 0.0f, 1.0f, "band-pass zero at DC blocks mag_x offset");
    ASSERT_NEAR(v.mag_y_filt, 0.0f, 1.0f, "band-pass zero at DC blocks mag_y offset");

    /* A spinning field in the tracking band: atan2 recovers a heading rotating at
       the spin rate. Feed mx=A cos(2πf t), my=A sin(2πf t) at f=20 Hz and set
       kf_omega so the band-pass centres on f. */
    sunshine_state_init(&s);
    memset(&in, 0, sizeof(in));
    memset(&v,  0, sizeof(v));
    float A = 22.0f, f = 20.0f;     /* µT, Hz */
    /* Fix: the band-pass centres on the LOOP-INDEPENDENT accel rate, NOT kf_omega.
       Set kf_omega deliberately WRONG (0) to prove the heading no longer depends
       on it; supply the true rate via omega_from_accel as brain.c does. */
    s.kf_omega          = 0.0f;
    v.accel_saturated   = 0;
    v.omega_from_accel  = 2.0f * 3.14159265f * f;
    float prev = 0.0f, sumrate = 0.0f, amp = 0.0f;
    int N = 4000, M = 0;
    for (int n = 0; n < N; n++) {
        float ph = 2.0f * 3.14159265f * f * n / 1000.0f;
        in.mag_x = (int16_t)(A * cosf(ph) / MAG_SCALE_UT);
        in.mag_y = (int16_t)(A * sinf(ph) / MAG_SCALE_UT);
        mag_heading_step(&in, &s, &v);
        if (n > 2000) {             /* after HP settles */
            sumrate += wrap_pi(v.mag_angle - prev);
            amp     += sqrtf(v.mag_x_filt*v.mag_x_filt + v.mag_y_filt*v.mag_y_filt);
            M++;
        }
        prev = v.mag_angle;
    }
    float rate = sumrate / (M * 0.001f);   /* rad/s */
    ASSERT_NEAR(fabsf(rate), 2.0f*3.14159265f*f, 3.0f, "heading rotates at the spin rate");
    ASSERT_NEAR(amp / M, A, 2.0f, "HP passes the spin-frequency field at ~unity gain");

    /* Regression: the band-pass must centre on omega_from_accel, NOT kf_omega.
       Set kf_omega to a WRONG constant (half the true rate). The old code centred
       on kf_omega → the true-frequency field lands an octave above the band → the
       passed amplitude collapses (and the heading is corrupted). With the fix the
       band centres on omega_from_accel → unity gain + correct rate regardless. */
    sunshine_state_init(&s);
    memset(&in, 0, sizeof(in));
    memset(&v,  0, sizeof(v));
    float Af = 22.0f, ff = 30.0f;          /* true spin 30 Hz */
    s.kf_omega         = 2.0f*3.14159265f*(ff*0.5f);  /* deliberately wrong: 15 Hz */
    v.accel_saturated  = 0;
    v.omega_from_accel = 2.0f*3.14159265f*ff;          /* truth fed via accel */
    float prevf = 0.0f, sumf = 0.0f, ampf = 0.0f; int Mf = 0;
    for (int n = 0; n < 4000; n++) {
        float ph = 2.0f*3.14159265f*ff*n/1000.0f;
        in.mag_x = (int16_t)(Af*cosf(ph)/MAG_SCALE_UT);
        in.mag_y = (int16_t)(Af*sinf(ph)/MAG_SCALE_UT);
        mag_heading_step(&in, &s, &v);
        if (n > 2000) {
            sumf += wrap_pi(v.mag_angle - prevf);
            ampf += sqrtf(v.mag_x_filt*v.mag_x_filt + v.mag_y_filt*v.mag_y_filt);
            Mf++;
        }
        prevf = v.mag_angle;
    }
    float ratef = sumf / (Mf * 0.001f);
    ASSERT_NEAR(fabsf(ratef), 2.0f*3.14159265f*ff, 4.0f,
                "heading rate tracks omega_from_accel, not a wrong kf_omega");
    ASSERT_NEAR(ampf / Mf, Af, 3.0f,
                "band-pass centred on omega_from_accel → unity gain despite wrong kf_omega");

    TEST_RESULTS();
}
