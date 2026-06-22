/* src/mag_heading.c — open-loop magnetometer absolute heading.
 *
 * The body spins, so the Earth field appears in the body frame as a sine at the
 * spin frequency, on top of (a) a body-fixed DC offset (hard-iron + average ESC
 * current) and (b) high-frequency tones from the LIS3MDL's 1 kHz low-power-mode
 * sampling (a strong one near fs/6 ≈ 167 Hz). To isolate the Earth sine we run a
 * 2nd-order RBJ band-pass on each axis, CENTRED ON THE SPIN FREQUENCY taken from
 * kf_omega and ±MAG_BP_HALF_BW_HZ wide. The band-pass has a transmission zero at
 * DC (kills hard-iron regardless of speed) and rolls off above the band
 * (rejecting the HF sampling tones). The heading is atan2 of the band-passed
 * axes — OPEN-LOOP (the estimate only picks the band centre, never the angle),
 * so it cannot drift; mis-centering only attenuates the signal, and the ±3 Hz
 * band (3× the accel rate uncertainty) keeps the true spin inside it.
 *
 * Coeffs are recomputed each tick from kf_omega (a few transcendental ops — fine
 * at 1 kHz; the spin rate changes slowly so the coeffs do too). Filter state is
 * SunshineState.mag_hp_x / mag_hp_y (two biquad delay elements per axis).
 */
#include "sunshine_core.h"
#include <math.h>

#define M_PI_F 3.14159265f

/* Direct-Form-II biquad with pre-normalised coefficients (a0 = 1). */
static float biquad(float x, float *w,
                    float b0, float b1, float b2,
                    float a1, float a2) {
    float wn = x - a1*w[0] - a2*w[1];
    float y  = b0*wn + b1*w[0] + b2*w[1];
    w[1] = w[0];
    w[0] = wn;
    return y;
}

void mag_heading_step(const SunshineInput *in, SunshineState *s, SunshineVars *v) {
    float mx = (float)in->mag_x * MAG_SCALE_UT;
    float my = (float)in->mag_y * MAG_SCALE_UT;

    /* Band-pass centre = estimated spin frequency (Hz), clamped to the min so the
     * coeffs stay sane below the mag-valid threshold (the mag update is gated off
     * there anyway by brain.c). */
    float fc = s->kf_omega * (0.5f / M_PI_F);
    if (fc < MAG_BP_MIN_FC_HZ) fc = MAG_BP_MIN_FC_HZ;

    /* RBJ band-pass (constant 0 dB peak): zero at DC, peak at fc, constant Q. */
    float w0    = 2.0f * M_PI_F * fc / 1000.0f;
    float cw    = cosf(w0), sw = sinf(w0);
    float alpha = sw / (2.0f * MAG_BP_Q);
    float a0    = 1.0f + alpha;
    float b0    =  alpha / a0;        /* b1 = 0 */
    float b2    = -alpha / a0;
    float a1    = -2.0f * cw / a0;
    float a2    = (1.0f - alpha) / a0;

    float mx_bp = biquad(mx, s->mag_hp_x, b0, 0.0f, b2, a1, a2);
    float my_bp = biquad(my, s->mag_hp_y, b0, 0.0f, b2, a1, a2);

    /* Band-passed Earth components; |(x,y)| ≈ Earth horizontal field (~22 µT),
     * a lock/health indicator. */
    v->mag_x_filt = mx_bp;
    v->mag_y_filt = my_bp;

    /* Open-loop absolute heading: angle of the (mx, -my) Earth-field vector.
     * atan2 COMBINES both axes into one unambiguous 360° heading (they are the
     * cos/sin components of the same rotating vector — see FILTER_MATH.md). The
     * -my matches the LIS3MDL y-axis inversion (DEBUGGING.md) so heading increases
     * with kf_theta (CCW positive). The band-pass's constant group-delay phase and
     * the declination offset are both constant, absorbed by theta_offset. */
    v->mag_angle = atan2f(-my_bp, mx_bp);
}
