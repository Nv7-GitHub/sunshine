/* src/control.c */
#include "sunshine_core.h"
#include <math.h>

#define M_PI_F 3.14159265f

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static float wrap_to_pi(float a) {
    while (a >  M_PI_F) a -= 2.0f * M_PI_F;
    while (a < -M_PI_F) a += 2.0f * M_PI_F;
    return a;
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
    /* v in [-1, 1]: positive = forward, negative = reverse (AM32 3D mode) */
    if (v >= 0.0f) return DSHOT_NEUTRAL + v * (DSHOT_MAX - DSHOT_NEUTRAL);
    else           return DSHOT_NEUTRAL + v * (DSHOT_NEUTRAL - DSHOT_MIN);
}

void control_step(const SunshineInput *in, SunshineState *s, SunshineVars *v) {
    float robot_angle = s->kf_theta + s->theta_offset;
    v->led_on = fabsf(wrap_to_pi(robot_angle)) < (3.0f * M_PI_F / 180.0f);

    if (in->mode == SUNSHINE_MODE_DISABLED) {
        v->dshot_cmd_left  = 0.0f;
        v->dshot_cmd_right = 0.0f;
        v->led_on = false;
        return;
    }

    if (in->mode == SUNSHINE_MODE_TANK) {
        float fwd  = ((float)in->ctrl_throttle / 127.5f) - 1.0f;
        float turn = (float)in->ctrl_x / 127.0f;
        v->dshot_cmd_left  = map_to_dshot(clampf(fwd + turn, -1.0f, 1.0f));
        v->dshot_cmd_right = map_to_dshot(clampf(fwd - turn, -1.0f, 1.0f));
        return;
    }

    /* MELTY */
    s->theta_offset += ((float)in->ctrl_theta / 127.0f) * THETA_RATE_RADS * 0.001f;

    float base      = ((float)in->ctrl_throttle / 255.0f) * MAX_DSHOT_SPIN;
    float cx        = (float)in->ctrl_x;
    float cy        = (float)in->ctrl_y;
    float drive_dir = atan2f(cy, cx);
    float drive_mag = sqrtf(cx*cx + cy*cy) / 127.0f;
    drive_mag       = clampf(drive_mag, 0.0f, 1.0f);

    float phase = wrap_to_pi(robot_angle - drive_dir);
    float diff  = trapezoid(phase, DRIFT_PULSE_WIDTH, DRIFT_RAMP_WIDTH)
                  * drive_mag * DRIFT_AMPLITUDE * base;

    v->dshot_cmd_left  = clampf(base + diff, 0.0f, DSHOT_MAX);
    v->dshot_cmd_right = clampf(base - diff, 0.0f, DSHOT_MAX);
}
