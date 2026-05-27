/* src/derot_filter.c */
#include "sunshine_core.h"
#include <math.h>

static float biquad(float x, float *w,
                    float b0, float b1, float b2,
                    float a1, float a2) {
    float wn = x - a1*w[0] - a2*w[1];
    float y  = b0*wn + b1*w[0] + b2*w[1];
    w[1] = w[0];
    w[0] = wn;
    return y;
}

static float lp4(float x, float *state) {
    /* state[0..1] = section 1, state[2..3] = section 2 */
    float y1 = biquad(x,       state,     LP4_S1_B0, LP4_S1_B1, LP4_S1_B2, LP4_S1_A1, LP4_S1_A2);
    float y2 = biquad(y1, state + 2,      LP4_S2_B0, LP4_S2_B1, LP4_S2_B2, LP4_S2_A1, LP4_S2_A2);
    return y2;
}

void derot_filter_step(const SunshineInput *in, SunshineState *s, SunshineVars *v) {
    float theta = s->kf_theta + s->theta_offset;
    float c  =  cosf(theta);
    float ss = -sinf(theta);   /* note: Q uses -sin for correct rotation */

    /* Convert to µT then derotate */
    float mx = (float)in->mag_x * MAG_SCALE_UT;
    float my = (float)in->mag_y * MAG_SCALE_UT;
    float I_raw =  c*mx - ss*my;   /* = mx*cos(θ) + my*sin(θ) */
    float Q_raw =  ss*mx + c*my;   /* = -mx*sin(θ) + my*cos(θ) */

    v->derot_I   = lp4(I_raw, s->derot_lp_I);
    v->derot_Q   = lp4(Q_raw, s->derot_lp_Q);
    v->mag_angle = atan2f(v->derot_Q, v->derot_I);
}
