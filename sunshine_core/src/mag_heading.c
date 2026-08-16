/* src/mag_heading.c — open-loop magnetometer absolute heading.
 *
 * The body spins, so the Earth field appears in the body frame as a sine at the
 * spin frequency, on top of (a) a body-fixed DC offset (hard-iron + average ESC
 * current) and (b) high-frequency tones from the LIS3MDL's 1 kHz low-power-mode
 * sampling (a strong one near fs/6 ≈ 167 Hz). To isolate the Earth sine we run a
 * 2nd-order RBJ band-pass on each axis, CENTRED ON THE SPIN FREQUENCY taken from
 * omega_from_accel (a direct accelerometer measurement, NOT kf_omega). The band-
 * pass has a transmission zero at DC (kills hard-iron regardless of speed) and
 * rolls off above the band (rejecting the HF sampling tones). The heading is atan2
 * of the band-passed axes — fully OPEN-LOOP: neither the angle nor the band centre
 * uses the estimate, so it cannot drift. This independence is the whole point:
 * centring the band on kf_omega (which the band-pass output feeds back into) closes
 * a loop whose per-tick coefficient modulation parametrically FALSE-LOCKS at half
 * the true spin rate. omega_from_accel is loop-independent, so it breaks that loop;
 * mis-centering from the accel's few-% bias only attenuates the signal (a fixed
 * band-pass is LTI → preserves frequency), never biases the angle.
 *
 * Coeffs are recomputed each tick from omega_from_accel (a few transcendental ops —
 * fine at 1 kHz; the spin rate changes slowly so the coeffs do too). Filter state is
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

    /* Band-pass centre = spin frequency. Use the LOOP-INDEPENDENT accelerometer
     * rate (omega_from_accel), NOT kf_omega: the heading this band-pass produces
     * sets kf_theta, which feeds kf_omega back via the Kalman update's cross-
     * covariance, so centring on kf_omega closes a feedback loop whose per-tick coeff
     * modulation parametrically FALSE-LOCKS the recovered heading at half the true
     * spin rate. omega_from_accel is measured straight from the accelerometer
     * (independent of the heading), so it breaks the loop. During accel saturation
     * it is 0 → fall back to |kf_omega| for those brief ticks. brain.c sets
     * omega_from_accel + accel_saturated on v before calling this. */
    float raw_spin = (!v->accel_saturated && v->omega_from_accel > 0.0f)
                     ? v->omega_from_accel : fabsf(s->kf_omega);

    /* Low-pass the CENTRE (see MAG_BP_FC_LP_HZ). omega_from_accel wobbles at the spin
     * frequency while translating (linear accel adds a once-per-rev term); retuning
     * the band-pass from that instantaneous value made the filter time-varying and
     * injected heading wobble. The true spin rate changes slowly, so a slow LP of it
     * keeps the filter quasi-LTI and kills the wobble. It stays loop-independent
     * (only the accel rate feeds it), so no kf_omega false-lock returns.
     *
     * Re-seed while BELOW the mag-valid threshold: there the band-pass output is
     * unused (brain.c gates the mag update off), so hold spin_freq_lp at the current
     * rate. A fast spin-up — including right after an impact stops the robot — then
     * begins tracking from the true rate with NO lag carried across the stop; the LP
     * only smooths once already spinning. */
    if (s->spin_freq_lp < SUNSHINE_MAG_MIN_OMEGA) {
        /* Seed / re-seed only when the SMOOTHED rate is genuinely sub-threshold
         * (robot slow or stopped) — NOT on the instantaneous raw value, which dips
         * below the threshold on the once-per-rev troughs while translating. Keying
         * the re-seed on raw_spin would re-seed to those trough values every rev and
         * destroy the smoothing (measured: it made kf_omega jumpier, not smoother). */
        s->spin_freq_lp = raw_spin;
    } else {
        /* LP rejects both the once-per-rev troughs AND impact spikes: at a=~0.009 a
         * lone 3× spike or 0.5× trough moves the centre by <2%, so it stays smooth. */
        float a = 1.0f - expf(-2.0f * M_PI_F * MAG_BP_FC_LP_HZ / 1000.0f);
        s->spin_freq_lp += a * (raw_spin - s->spin_freq_lp);
    }
    float fc = s->spin_freq_lp * (0.5f / M_PI_F);
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

    /* SIGNED mag rotation rate — used ONLY for the spin SIGN the accel can't
     * sense (schema v4; the sign reverses when the robot is inverted). It briefly
     * (v5) also served as the Kalman's rate measurement, but that was reverted:
     * in-band AC interference tones beat against the Earth line, and
     * differentiating the resulting mag_angle wobble amplified it into large
     * kf_omega wobble (see brain.c rate-update comment). Low-passed at
     * MAG_SPIN_RATE_LP_HZ so the sign is robust to per-tick noise. Only
     * integrated while the mag is valid; mag_ang_prev still advances every tick
     * so the first valid delta is a true one-tick step. */
    float dma = v->mag_angle - s->mag_ang_prev;
    while (dma >  M_PI_F) dma -= 2.0f * M_PI_F;
    while (dma < -M_PI_F) dma += 2.0f * M_PI_F;
    s->mag_ang_prev = v->mag_angle;
    if (v->mag_valid) {
        float a = 1.0f - expf(-2.0f * M_PI_F * MAG_SPIN_RATE_LP_HZ / 1000.0f);
        s->spin_rate_lp += a * (dma * 1000.0f - s->spin_rate_lp);
    }
}
