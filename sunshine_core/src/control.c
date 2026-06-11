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

/* Trapezoidal wave: +1 at |phase|<half_flat, linear ramp, -1 at bottom */
static float trapezoid(float phase, float pulse_width, float ramp_width) {
    float ap        = fabsf(phase);
    float half_flat = pulse_width * M_PI_F;
    float half_edge = (pulse_width + ramp_width) * M_PI_F;
    if (ap <= half_flat)  return  1.0f;
    if (ap >= half_edge)  return -1.0f;
    return 1.0f - 2.0f * (ap - half_flat) / (ramp_width * M_PI_F);
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
    v->led_on         = fabsf(wrapped) < (3.0f * M_PI_F / 180.0f);

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

    float phase = wrap_to_pi(robot_angle - drive_dir);
    float diff  = trapezoid(phase, DRIFT_PULSE_WIDTH, DRIFT_RAMP_WIDTH)
                  * drive_mag * DRIFT_AMPLITUDE * spin_span;

    v->dshot_cmd_left  = clampf(DSHOT_NEUTRAL + spin_span + diff, DSHOT_NEUTRAL, DSHOT_MAX);
    v->dshot_cmd_right = clampf(DSHOT_NEUTRAL + spin_span - diff, DSHOT_NEUTRAL, DSHOT_MAX);
}
