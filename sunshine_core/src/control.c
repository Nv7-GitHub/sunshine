/* src/control.c */
#include "sunshine_core.h"
#include <math.h>

#define M_PI_F 3.14159265f

/* Wheel-speed cap policy (control policy, deliberately NOT in the header next to
 * the per-build physical parameters — a different robot does not retune these).
 * Outside the plausibility window the battery reading is not believable (a stuck
 * ADC, an unpopulated divider, a garbage frame), and the cap is SKIPPED ENTIRELY.
 * The cap divides by v_batt, so a stuck-low reading would compute a near-zero
 * ceiling and silently disarm the weapon mid-match. It must therefore fail OPEN,
 * never closed: a wrong battery reading costs us the overspeed bound, a closed
 * failure costs us the fight. */
#define RADS_TO_RPM     (60.0f / (2.0f * M_PI_F))
#define CAP_VBATT_MIN_V 5.0f
#define CAP_VBATT_MAX_V 10.0f

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

/* MELTY wheel-speed cap. Returns `base` clamped to the DShot value whose no-load
 * wheel speed equals the speed the ground demands plus a fixed slip allowance.
 * Without it the wheels run 1.3–2.3× the rolling speed, and each landing dumps
 * that surplus kinetic energy in as an impulsive traction spike (the bounce).
 *
 * The rate reference is the mag-driven kf_omega and NEVER omega_from_accel. The
 * accel is corrupted precisely when the cap matters most: airborne it sees
 * free-fall, and on touchdown it sees the impact, so the accel-derived rate spikes
 * and would RAISE the cap at the exact instant the wheels are about to hit. The
 * mag rotation rate measures rotation directly and is immune to linear
 * acceleration, so it is the only trustworthy reference during a bounce.
 *
 * The reference is max(..., SUNSHINE_MAG_MIN_OMEGA) rather than "switch the cap
 * off when unlocked", because losing lock does not mean ω is unknown. Lock is
 * DEFINED as |ω| > SUNSHINE_MAG_MIN_OMEGA, so "not locked" itself carries the
 * bound |ω| ≤ SUNSHINE_MAG_MIN_OMEGA — the cap never switches off, it falls back
 * to the threshold, and the runaway stays bounded even before lock. And because
 * locked implies |kf_omega| > the threshold, this single max() is CONTINUOUS
 * across the lock boundary: no step in commanded speed the instant the mag locks.
 *
 * v_batt must appear because the ESC is an open-loop voltage source: speed is set
 * by duty × V_batt, and the pack moved 7.80–8.39 V inside one run — a 7% speed
 * swing for an unchanged DShot value. A cap in pure DShot units would drift with
 * the battery over a match.
 *
 * Accepted consequence: a robot with a DEAD magnetometer never locks and so is
 * held at the unlocked ceiling forever (≈1194 DShot at 8.2 V) — it will not spin
 * up. That is the right trade, since a MELTY robot with no heading reference is
 * undriveable anyway, but it converts "degraded but spinning" into "will not spin
 * up" and is therefore documented in BRINGUP.md so bringup sees it first. */
/* drive_mag: post-fade deflection (scales the slip un-bias).
 * drive_raw: pre-fade deflection — the governor must see the stick even when
 * the spin is above the rolloff band (where post-fade drive is zero), or it
 * could never brake the robot down into the band. */
static float melty_speed_cap(const SunshineState *s, const SunshineVars *v, float base,
                             float drive_mag, float drive_raw) {
    /* Written as a positive-range test so a NaN battery also fails open. */
    if (!(v->batt_voltage >= CAP_VBATT_MIN_V && v->batt_voltage <= CAP_VBATT_MAX_V))
        return base;

    /* Same lock condition brain.c uses to decide whether to trust the mag rate. */
    int locked = v->mag_valid && fabsf(s->spin_rate_lp) > SUNSHINE_MAG_MIN_OMEGA;

    /* Translation slip un-bias. The slip allowance exists to give spin-up torque,
     * but during translation it is a FORCE DEAD ZONE: tire friction saturates
     * within a few tenths of m/s of slip, so with both wheels biased +1 m/s
     * forward the drift wave's first ~1 m/s of differential moves neither tire
     * off forward saturation and produces ~zero force (measured on the
     * 2026-08-06 logs: light presses modulated the motors audibly but barely
     * translated; only deep-saturation amplitudes translated, which is also
     * what bounced the robot). Scaling the allowance down with drive_mag centres
     * the wave near ZERO slip at full stick — advancing wheel pushes, retreating
     * wheel genuinely brakes, both in the controllable friction range — so
     * translation force is proportional to the stick from the first counts.
     * DRIFT_UNBIAS_FRAC < 1 keeps a spin-maintenance remnant of the allowance
     * at full stick (see sunshine_core.h: removing all of it starves the spin
     * against drag and the robot parks at the fade equilibrium). */
    float allow_ms = WHEEL_SLIP_ALLOW_MS * (1.0f - DRIFT_UNBIAS_FRAC * drive_mag);

    float w_ref    = fmaxf(locked ? fabsf(s->kf_omega) : 0.0f, SUNSHINE_MAG_MIN_OMEGA);
    /* Translation spin governor — see DRIFT_TRANSLATE_OMEGA_RADS. A held stick
     * blends the reference down to the translating band, so the cap actively
     * brakes the robot to a speed where the wheels can realize the wave. */
    w_ref         -= drive_raw * fmaxf(0.0f, w_ref - DRIFT_TRANSLATE_OMEGA_RADS);
    float w_cap    = w_ref * (WHEEL_CENTER_M / WHEEL_RADIUS_M)   /* rolling rate    */
                     + allow_ms / WHEEL_RADIUS_M;                /* + slip allowance */
    float v_needed = (w_cap * RADS_TO_RPM) / MOTOR_KV_RPM_PER_V;
    float cap      = DSHOT_NEUTRAL + (v_needed / v->batt_voltage) * (DSHOT_MAX - DSHOT_NEUTRAL);

    return fminf(base, fmaxf(cap, DSHOT_NEUTRAL));
}

void control_step(const SunshineInput *in, SunshineState *s, SunshineVars *v) {
    float robot_angle = s->kf_theta + s->theta_offset;
    float wrapped     = wrap_to_pi(robot_angle);
    float hd          = wrapped * (180.0f / M_PI_F);
    v->heading_deg    = hd < 0.0f ? hd + 360.0f : hd;
    /* LED beacon (MELTY; TANK overrides this to solid below): lit within
     * ±LED_HALF_ARC of the zero heading. At high spin the
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
        /* The heading beacon is a MELTY-only construct: it marks the robot's
         * "forward" once per revolution so the driver can steer a spinning body.
         * TANK does not spin, so the same test just tracks wherever the (noisy,
         * possibly unlocked) mag heading happens to sit relative to zero — the
         * LED then stutters on and off at random as the robot is driven around.
         * Hold it solid instead: in TANK the LED means "armed", which reads
         * unambiguously against the DISABLED breathe and the MELTY strobe. */
        v->led_on = 1;
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
    float drive_raw = drive_mag;   /* pre-fade, for the spin governor */
    /* Low-spin translation fade — see DRIFT_OMEGA_FADE_* in sunshine_core.h.
     * Applied BEFORE the cap so a faded drive also restores the full spin-up
     * slip allowance (the un-bias scales by the post-fade drive_mag). */
    drive_mag *= clampf((fabsf(s->kf_omega) - DRIFT_OMEGA_FADE_LO)
                        / (DRIFT_OMEGA_FADE_HI - DRIFT_OMEGA_FADE_LO), 0.0f, 1.0f);
    /* High-spin rolloff — plant-bandwidth gate, see DRIFT_OMEGA_ROLLOFF_* in
     * sunshine_core.h. Above the band the wheels can't realize the wave, so
     * commanding it buys tilt and zero force. */
    drive_mag *= clampf((DRIFT_OMEGA_ROLLOFF_HI - fabsf(s->kf_omega))
                        / (DRIFT_OMEGA_ROLLOFF_HI - DRIFT_OMEGA_ROLLOFF_LO), 0.0f, 1.0f);

    float base = DSHOT_NEUTRAL + spin_span;
    /* Cap the MEAN wheel command before headroom is derived from it, so the drift
     * differential scales down together with the capped base. Note the drift wave
     * still rides above AND below the capped base, so the driving wheel briefly
     * exceeds the cap while the other brakes — that asymmetry IS the translation
     * mechanism and must not be clamped away. */
    base = melty_speed_cap(s, v, base, drive_mag, drive_raw);
    float headroom = fminf(base - DSHOT_NEUTRAL, DSHOT_MAX - base);
    if (headroom < 0.0f) headroom = 0.0f;

    float phase = wrap_to_pi(robot_angle - drive_dir
                             + DRIFT_PHASE_OFFSET_RADS
                             + s->kf_omega * DRIFT_PHASE_LEAD_S);
    float diff = drift_wave(phase) * drive_mag * DRIFT_AMPLITUDE * headroom;

    v->dshot_cmd_left  = clampf(base + diff, DSHOT_NEUTRAL, DSHOT_MAX);
    v->dshot_cmd_right = clampf(base - diff, DSHOT_NEUTRAL, DSHOT_MAX);
}
