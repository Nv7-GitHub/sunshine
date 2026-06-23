/* src/control.c */
#include "sunshine_core.h"
#include <math.h>

#define M_PI_F 3.14159265f

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static float wrap_to_pi(float a) {
    /* O(1) wrap to [-pi, pi] — see kalman.c. The old iterative version's cost
     * scaled with |a|, so once kf_theta grew large this ran thousands of loops
     * every tick and blew the 1 kHz budget. */
    return remainderf(a, 2.0f * M_PI_F);
}

/* Balanced bipolar trapezoid for MELTY translation.
 * The + and - plateaus are 180° apart and equal area, so the differential has
 * zero mean and wave(phase+pi) == -wave(phase). DRIFT_PLATEAU_WIDTH is the
 * fraction of a full rotation spent on each plateau; the two ramps fill the
 * rest of the cycle to preserve half-wave symmetry. */
static float drift_wave(float phase) {
    float p = wrap_to_pi(phase);
    float plateau = clampf(DRIFT_PLATEAU_WIDTH, 0.0f, 0.49f);
    float half_flat = plateau * M_PI_F;
    float ramp = M_PI_F - 2.0f * half_flat;
    float ap = fabsf(p);

    if (ap <= half_flat) return 1.0f;
    if (ap >= M_PI_F - half_flat) return -1.0f;
    if (ramp <= 1e-6f) return p >= 0.0f ? -1.0f : 1.0f;

    if (p > 0.0f)
        return 1.0f - 2.0f * (p - half_flat) / ramp;
    return 1.0f + 2.0f * (p + half_flat) / ramp;
}

static float map_to_dshot(float v) {
    /* v in [-1, 1] → AM32 3D DShot value.
     * Forward [1048..2047]: v=0 → 1048 (stopped), v=+1 → 2047 (max fwd).
     * Reverse [48..1047]:   v→0⁻ → 48 (min rev),  v=-1 → 1047 (max rev). */
    if (v >= 0.0f) return DSHOT_NEUTRAL + v * (DSHOT_MAX    - DSHOT_NEUTRAL);
    else           return DSHOT_MIN     + (-v) * (DSHOT_NEUTRAL - 1.0f - DSHOT_MIN);
}

void control_step(const SunshineInput *in, SunshineState *s, SunshineVars *v) {
    float robot_angle = s->kf_theta + s->theta_offset;
    float wrapped     = wrap_to_pi(robot_angle);
    float hd          = wrapped * (180.0f / M_PI_F);
    v->heading_deg    = hd < 0.0f ? hd + 360.0f : hd;
    /* LED beacon: lit within ±LED_HALF_ARC of the zero heading. At high spin the
     * heading advances more than the 6° window per 1 kHz tick (e.g. ~14° at 250
     * rad/s), so a fixed ±3° point-test would step OVER the window between samples
     * and the dot would vanish for whole revolutions. Widen the half-window to at
     * least half the per-tick sweep (ω·dt/2) so a sample always lands in it: the
     * dot is then never narrower than one tick (the 1 kHz limit) and never skipped.
     * At low spin it stays the crisp ±3°. */
    float led_half = 3.0f * M_PI_F / 180.0f;
    float sweep_half = 0.6f * fabsf(s->kf_omega) * 0.001f;   /* >½·ω·dt (dt=1 ms) + margin */
    if (led_half < sweep_half) led_half = sweep_half;
    v->led_on = fabsf(wrapped) < led_half;

    if (in->mode == SUNSHINE_MODE_DISABLED) {
        v->dshot_cmd_left  = 0.0f;
        v->dshot_cmd_right = 0.0f;
        v->led_on = 0;
        return;
    }

    if (in->mode == SUNSHINE_MODE_TANK) {
        /* W/S (ctrl_y) controls fwd/rev; A/D (ctrl_x) controls turning */
        float fwd  = clampf((float)in->ctrl_y  / 127.0f, -1.0f, 1.0f);
        float turn = clampf((float)in->ctrl_x  / 127.0f, -1.0f, 1.0f);
        v->dshot_cmd_left  = map_to_dshot(clampf(fwd + turn, -1.0f, 1.0f));
        v->dshot_cmd_right = map_to_dshot(clampf(turn - fwd, -1.0f, 1.0f));
        return;
    }

    /* MELTY */
    /* Driver heading offset is an angle — keep it wrapped so it can't grow
     * unbounded when the driver holds a turn (precision + keeps cos/sin args
     * small). Only used as (kf_theta + theta_offset), so wrapping is invisible. */
    s->theta_offset = wrap_to_pi(s->theta_offset
                       + ((float)in->ctrl_theta / 127.0f) * THETA_RATE_RADS * 0.001f);

    /* MELTY spin is unipolar: outputs stay in [DSHOT_NEUTRAL..DSHOT_MAX].
     * motor_cmd() in nav_control inverts these into the ESC reverse zone
     * [48..1047], so both motors always spin CCW (forward = CCW per BRINGUP). */
    float spin_frac = clampf((float)in->ctrl_throttle / 255.0f, 0.0f, 1.0f);
    float spin_span = spin_frac * (MAX_DSHOT_SPIN - DSHOT_NEUTRAL);
    float cx        = (float)in->ctrl_x;
    float cy        = (float)in->ctrl_y;
    float drive_dir = atan2f(cy, cx);
    float drive_mag = sqrtf(cx*cx + cy*cy) / 127.0f;
    drive_mag       = clampf(drive_mag, 0.0f, 1.0f);

    float base = DSHOT_NEUTRAL + spin_span;
    float headroom = fminf(base - DSHOT_NEUTRAL, DSHOT_MAX - base);
    if (headroom < 0.0f) headroom = 0.0f;

    float phase = wrap_to_pi(robot_angle - drive_dir
                             + DRIFT_PHASE_OFFSET_RADS
                             + s->kf_omega * DRIFT_PHASE_LEAD_S);
    float diff = drift_wave(phase) * drive_mag * DRIFT_AMPLITUDE * headroom;

    v->dshot_cmd_left  = clampf(base + diff, DSHOT_NEUTRAL, DSHOT_MAX);
    v->dshot_cmd_right = clampf(base - diff, DSHOT_NEUTRAL, DSHOT_MAX);
}
