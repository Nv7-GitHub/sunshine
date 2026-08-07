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

static float melty_halfdiff_omega(float phase, float omega, uint8_t throttle) {
    SunshineState s;
    SunshineVars v;
    sunshine_state_init(&s);
    s.kf_theta = phase;
    s.kf_omega = omega;
    /* Zeroed vars = 0 V battery, which is implausible, so the wheel-speed cap
       fails open and these waveform assertions keep their pre-cap values. Do NOT
       "fix" this by supplying a battery — the cap has its own tests below. */
    memset(&v, 0, sizeof(v));
    SunshineInput in = make_input(SUNSHINE_MODE_MELTY, throttle, 127, 0, 0);
    control_step(&in, &s, &v);
    return 0.5f * (v.dshot_cmd_left - v.dshot_cmd_right);
}

static float melty_halfdiff_at(float phase, uint8_t throttle) {
    /* Waveform-shape probe. Translation fades out below DRIFT_OMEGA_FADE_HI and
       rolls off above DRIFT_OMEGA_ROLLOFF_LO, so probe inside the full-authority
       band and cancel the omega*lead advance so `phase` is still the wave's own
       argument. */
    const float W = 95.0f;
    return melty_halfdiff_omega(phase - W * DRIFT_PHASE_LEAD_S, W, throttle);
}

/* One MELTY control_step with the full cap-relevant state made explicit. */
static SunshineVars melty_run(uint8_t throttle, int8_t cx, float kf_omega,
                              float spin_rate_lp, uint8_t mag_valid,
                              float batt, float theta) {
    SunshineState s;
    SunshineVars v;
    sunshine_state_init(&s);
    s.kf_theta     = theta;
    s.kf_omega     = kf_omega;
    s.spin_rate_lp = spin_rate_lp;
    memset(&v, 0, sizeof(v));
    v.mag_valid    = mag_valid;
    v.batt_voltage = batt;
    SunshineInput in = make_input(SUNSHINE_MODE_MELTY, throttle, cx, 0, 0);
    control_step(&in, &s, &v);
    return v;
}

/* Inverse of the cap's forward model: the no-load wheel rate (rad/s) a DShot
   command implies at a given battery voltage. Tests assert this PHYSICAL
   invariant rather than hard-coded DShot numbers, so no magic cap value gets
   baked into the test and a geometry change stays a one-line edit. */
static float cmd_to_wheel_rads(float cmd, float batt) {
    float duty = (cmd - DSHOT_NEUTRAL) / (DSHOT_MAX - DSHOT_NEUTRAL);
    return duty * batt * MOTOR_KV_RPM_PER_V * (2.0f * 3.14159265f / 60.0f);
}

int main(void) {
    SunshineState s; SunshineVars v;
    sunshine_state_init(&s);
    /* control_step now READS v (batt_voltage, mag_valid) for the wheel-speed cap,
       so these must be initialised or every MELTY assertion below is
       nondeterministic. 0 V is implausible → the cap fails open → all pre-existing
       expected values are unchanged. Deliberate; do not supply a battery here. */
    memset(&v, 0, sizeof(v));

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

    /* TANK holds the LED solid. The heading beacon is meaningless without a spin,
       so applying it here made the LED stutter at random as the robot drove. Sweep
       headings that straddle the beacon window: all must stay lit. */
    for (float th = -3.0f; th <= 3.0f; th += 0.05f) {
        s.kf_theta = th; s.theta_offset = 0.0f;
        in = make_input(SUNSHINE_MODE_TANK, 0, 40, 60, 0);
        control_step(&in, &s, &v);
        ASSERT_EQ(v.led_on, true, "TANK: LED solid at every heading");
    }
    sunshine_state_init(&s);

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
        s.kf_omega = 95.0f;    /* inside the translation authority band so drift is live */
        in = make_input(SUNSHINE_MODE_MELTY, 240, 127, 0, 0);
        control_step(&in, &s, &v);
        ASSERT(v.dshot_cmd_left < DSHOT_MAX - 1.0f, "MELTY high throttle drift does not clip high side");
        ASSERT(v.dshot_cmd_right > DSHOT_NEUTRAL + 1.0f, "MELTY high throttle drift does not clip low side");
    }

    /* ── MELTY actuation-lag phase lead ────────────────────────────────────
       Measured by tools/replay/translation_lag.py on the 2026-07-20 translation2
       log: the wheel-speed response (eRPM differential) lags the DShot
       differential by a pure TIME delay of ~20 ms (18–24 ms across 24 windows,
       BOTH spin directions; lock-in phase fits omega*tau with no constant
       offset). Minus ~3 ms of median-5 eRPM telemetry lag → ~17 ms physical.
       Uncompensated, that is a 110–150° force-direction error at 1000–1300 RPM —
       translation mostly cancels. DRIFT_PHASE_LEAD_S must compensate it. The
       band below is generous because the delay is per-build (motor/ESC/wheel
       inertia): re-measure per BRINGUP.md Level 5 and retarget on a new robot. */
    {
        /* (a) The compiled lead must be in the measured band. The FORCE lag is
              ~4-5 ms (two-speed drift test with the offset pinned at the
              geometrically derived pi), NOT the ~20 ms wheel-SPEED lag —
              speed integrates torque. */
        ASSERT(DRIFT_PHASE_LEAD_S >= 0.001f && DRIFT_PHASE_LEAD_S <= 0.009f,
               "PHASE LEAD: compensates the ~4-5 ms force lag, not the speed lag");

        /* (b) Sign/wiring: at spin rate W the wave must be ADVANCED by exactly
              W*LEAD. Both probes spin inside the translation authority band
              (omega=0 would fade the drift to zero; 135+ rolls it off), so
              compare W against W2 with the phase pre-shifted by (W-W2)*LEAD:
              both then evaluate the wave at theta + W*LEAD, and a retard (sign
              flip) breaks the match. */
        const float W = 95.0f, W2 = 100.0f;     /* rad/s, in-band spin rates */
        for (float th = -3.0f; th <= 3.0f; th += 0.37f) {
            float spinning = melty_halfdiff_omega(th, W, 100);
            float advanced = melty_halfdiff_omega(th + (W - W2) * DRIFT_PHASE_LEAD_S,
                                                  W2, 100);
            ASSERT_NEAR(spinning, advanced, 1.0f,
                        "PHASE LEAD: wave advanced by omega*lead (sign correct)");
        }

        /* (c) Negative spin (inverted robot) must advance the other way:
              halfdiff(0.5, -W) evaluates the wave at 0.5 - W*LEAD, which equals
              a positive-spin probe pre-retarded by 2*W*LEAD. */
        float neg  = melty_halfdiff_omega(0.5f, -W, 100);
        float nexp = melty_halfdiff_omega(0.5f - 2.0f * W * DRIFT_PHASE_LEAD_S, W, 100);
        ASSERT_NEAR(neg, nexp, 1.0f,
                    "PHASE LEAD: negative spin advances in the opposite direction");
    }

    /* ── MELTY wheel-speed cap ─────────────────────────────────────────────
       The cap inverts the ESC's open-loop voltage-source model to command a
       no-load wheel speed of "rolling speed + fixed slip allowance", which bounds
       the 1.3–2.3x overspeed that dumps stored wheel energy into the ground on
       every landing. Assertions are on the commanded no-load WHEEL RATE, not on
       DShot counts, so the geometry is what is under test. */
    {
        const float GEOM   = WHEEL_CENTER_M / WHEEL_RADIUS_M;   /* wheel rad/s per body rad/s */
        const float ALLOW  = WHEEL_SLIP_ALLOW_MS / WHEEL_RADIUS_M; /* allowance, wheel rad/s  */
        const float TOL    = 2.0f;   /* rad/s; ~2 DShot counts at 8 V */

        /* (1) Losing lock does not switch the cap off — lock is DEFINED as
              |omega| > threshold, so "unlocked" itself bounds omega by the
              threshold, and the cap falls back to exactly that rate. */
        SunshineVars u = melty_run(255, 0, 0.0f, 0.0f, 0, 8.0f, 0.0f);
        ASSERT_NEAR(cmd_to_wheel_rads(u.dshot_cmd_left, 8.0f),
                    SUNSHINE_MAG_MIN_OMEGA * GEOM + ALLOW, TOL,
                    "CAP unlocked: falls back to the mag-threshold rate, not off");
        ASSERT(u.dshot_cmd_left < DSHOT_MAX - 200.0f,
               "CAP unlocked: actually bites at full throttle");

        /* (2) Locked, the cap tracks |kf_omega| — the mag-driven rate, never the
              accel (which sees free-fall and impact exactly when the cap matters). */
        SunshineVars l100 = melty_run(255, 0, 100.0f, 100.0f, 1, 8.0f, 0.0f);
        ASSERT_NEAR(cmd_to_wheel_rads(l100.dshot_cmd_left, 8.0f),
                    100.0f * GEOM + ALLOW, TOL,
                    "CAP locked: tracks |kf_omega| at 100 rad/s");
        ASSERT(l100.dshot_cmd_left > u.dshot_cmd_left,
               "CAP locked at 100 rad/s permits more than the unlocked floor");

        /* (3) Commanded speed = rolling + allowance: the slope in body rate is
              pure geometry and the allowance is a constant offset (a percentage
              margin would make the slope depend on the allowance). */
        SunshineVars l80  = melty_run(255, 0,  80.0f,  80.0f, 1, 8.0f, 0.0f);
        SunshineVars l140 = melty_run(255, 0, 140.0f, 140.0f, 1, 8.0f, 0.0f);
        ASSERT_NEAR(cmd_to_wheel_rads(l140.dshot_cmd_left, 8.0f)
                    - cmd_to_wheel_rads(l80.dshot_cmd_left, 8.0f),
                    60.0f * GEOM, TOL,
                    "CAP: slope is pure geometry, allowance is a constant offset");

        /* (4) Inverse in v_batt: the ESC is a voltage source, so the same wheel
              speed needs proportionally less DShot as the pack rises. The pack
              moved 7.80-8.39 V inside one run — 7% of speed at fixed DShot. */
        SunshineVars b75 = melty_run(255, 0, 0.0f, 0.0f, 0, 7.5f, 0.0f);
        SunshineVars b90 = melty_run(255, 0, 0.0f, 0.0f, 0, 9.0f, 0.0f);
        ASSERT_NEAR((b75.dshot_cmd_left - DSHOT_NEUTRAL) * 7.5f,
                    (b90.dshot_cmd_left - DSHOT_NEUTRAL) * 9.0f, 1.0f,
                    "CAP: (cmd - neutral) * v_batt is invariant");
        ASSERT(b75.dshot_cmd_left > b90.dshot_cmd_left,
               "CAP: lower battery needs a higher DShot for the same wheel speed");

        /* (5) The drift wave rides above and below the CAPPED base: one wheel
              legitimately exceeds the cap while the other brakes. That asymmetry
              IS translation and must not be clamped away. theta cancels the
              omega*DRIFT_PHASE_LEAD_S advance so the wave sits on its + plateau
              regardless of the compiled lead. */
        SunshineVars tr = melty_run(255, 127, 100.0f, 100.0f, 1, 8.0f,
                                    -DRIFT_PHASE_OFFSET_RADS - 100.0f * DRIFT_PHASE_LEAD_S);
        ASSERT(tr.dshot_cmd_left > tr.dshot_cmd_right,
               "CAP: drift differential survives the cap");
        ASSERT(tr.dshot_cmd_left > l100.dshot_cmd_left - ALLOW / 2.0f,
               "CAP: driving wheel rides above the translating base");

        /* (5b) Translation slip un-bias: at full stick the allowance must drop
              to its spin-maintenance remnant (1 - DRIFT_UNBIAS_FRAC) so the
              wave straddles near-zero slip. Measured 2026-08-06: the full
              +1 m/s bias kept both tires saturated forward for most of the
              wave — a 1 m/s differential dead zone. Measured 2026-08-07:
              removing ALL of it starved the spin (~0.5 m/s of slip just pays
              drag) and the robot parked at the fade equilibrium. */
        ASSERT_NEAR(cmd_to_wheel_rads(0.5f * (tr.dshot_cmd_left + tr.dshot_cmd_right), 8.0f),
                    100.0f * GEOM + ALLOW * (1.0f - DRIFT_UNBIAS_FRAC), TOL,
                    "UNBIAS: full stick keeps only the spin-maintenance remnant");

        /* (5c) Partial stick keeps a proportional share of the allowance. */
        SunshineVars half = melty_run(255, 64, 100.0f, 100.0f, 1, 8.0f,
                                      -DRIFT_PHASE_OFFSET_RADS - 100.0f * DRIFT_PHASE_LEAD_S);
        ASSERT_NEAR(cmd_to_wheel_rads(0.5f * (half.dshot_cmd_left + half.dshot_cmd_right), 8.0f),
                    100.0f * GEOM + ALLOW * (1.0f - DRIFT_UNBIAS_FRAC * 64.0f / 127.0f), TOL,
                    "UNBIAS: allowance scales down with stick deflection");

        /* (5d) Low-spin translation fade: below DRIFT_OMEGA_FADE_LO the drift
              must be OFF (the measured failure: a collapse leaves the robot
              trapped at 55-60 rad/s, drive held, cap braking the wheels — the
              fade lets it spin back up instead), ramping to full authority by
              DRIFT_OMEGA_FADE_HI. */
        SunshineVars f_lo  = melty_run(255, 127, 55.0f, 55.0f, 1, 8.0f,
                                       -DRIFT_PHASE_OFFSET_RADS - 55.0f * DRIFT_PHASE_LEAD_S);
        SunshineVars f_mid = melty_run(255, 127, 72.0f, 72.0f, 1, 8.0f,
                                       -DRIFT_PHASE_OFFSET_RADS - 72.0f * DRIFT_PHASE_LEAD_S);
        SunshineVars f_hi  = melty_run(255, 127, 95.0f, 95.0f, 1, 8.0f,
                                       -DRIFT_PHASE_OFFSET_RADS - 95.0f * DRIFT_PHASE_LEAD_S);
        float d_lo  = 0.5f * (f_lo.dshot_cmd_left  - f_lo.dshot_cmd_right);
        float d_mid = 0.5f * (f_mid.dshot_cmd_left - f_mid.dshot_cmd_right);
        float d_hi  = 0.5f * (f_hi.dshot_cmd_left  - f_hi.dshot_cmd_right);
        ASSERT_NEAR(d_lo, 0.0f, 0.5f, "FADE: no translation below DRIFT_OMEGA_FADE_LO");
        ASSERT(d_mid > 2.0f,          "FADE: partial authority inside the fade band");
        ASSERT(d_hi > d_mid,          "FADE: full authority above DRIFT_OMEGA_FADE_HI");

        /* (5f) High spin keeps full translation authority: a rolloff + spin
              governor briefly forced translation below ~1000 RPM, built on an
              eRPM-attenuation reading that tire grip confounds. Melty practice
              translates at 2K+ RPM; the drift must stay live there. */
        SunshineVars hi_spin = melty_run(255, 127, 165.0f, 165.0f, 1, 8.0f,
                                         -DRIFT_PHASE_OFFSET_RADS - 165.0f * DRIFT_PHASE_LEAD_S);
        float d_hispin = 0.5f * (hi_spin.dshot_cmd_left - hi_spin.dshot_cmd_right);
        ASSERT(d_hispin > 2.0f, "HIGH SPIN: translation authority is not rolled off");
        ASSERT_NEAR(cmd_to_wheel_rads(0.5f * (hi_spin.dshot_cmd_left + hi_spin.dshot_cmd_right), 8.0f),
                    165.0f * GEOM + ALLOW * (1.0f - DRIFT_UNBIAS_FRAC), TOL,
                    "HIGH SPIN: cap tracks the actual spin (no governor pull-down)");
        /* With translation faded out, the full spin-up slip allowance returns. */
        ASSERT_NEAR(cmd_to_wheel_rads(0.5f * (f_lo.dshot_cmd_left + f_lo.dshot_cmd_right), 8.0f),
                    55.0f * GEOM + ALLOW, TOL,
                    "FADE: faded translation restores the spin-up allowance");

        /* (6) Fail-open: an implausible battery reading skips the cap entirely.
              A stuck-low or garbage reading must never disarm the weapon. */
        SunshineVars f0  = melty_run(255, 0, 0.0f, 0.0f, 0,  0.0f, 0.0f);
        SunshineVars f12 = melty_run(255, 0, 0.0f, 0.0f, 0, 12.0f, 0.0f);
        ASSERT_NEAR(f0.dshot_cmd_left,  DSHOT_MAX, 1.0f,
                    "CAP fail-open: 0 V battery reading skips the cap");
        ASSERT_NEAR(f12.dshot_cmd_left, DSHOT_MAX, 1.0f,
                    "CAP fail-open: 12 V battery reading skips the cap");

        /* (7) Continuity across the lock boundary: locked implies
              |kf_omega| > threshold, so the single max() has no step at lock. */
        SunshineVars just_locked = melty_run(255, 0,
                                             SUNSHINE_MAG_MIN_OMEGA + 0.01f,
                                             SUNSHINE_MAG_MIN_OMEGA + 0.01f,
                                             1, 8.0f, 0.0f);
        ASSERT_NEAR(just_locked.dshot_cmd_left, u.dshot_cmd_left, 1.0f,
                    "CAP: continuous across the lock boundary");

        /* (8) Inverted robot: kf_omega is signed, the cap uses |kf_omega|, so a
              negative body rate must not be throttled to the unlocked floor. */
        SunshineVars pos = melty_run(255, 0,  120.0f,  120.0f, 1, 8.0f, 0.0f);
        SunshineVars neg = melty_run(255, 0, -120.0f, -120.0f, 1, 8.0f, 0.0f);
        ASSERT_NEAR(neg.dshot_cmd_left, pos.dshot_cmd_left, 1.0f,
                    "CAP: inverted spin (negative omega) caps identically");
    }

    TEST_RESULTS();
}
