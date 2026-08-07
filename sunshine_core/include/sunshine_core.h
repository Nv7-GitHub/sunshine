#pragma once
#include <stddef.h>
#include <stdint.h>

/* ── Schema version ────────────────────────────────────────────────────────
 * Bump whenever ANY field is added, removed, reordered, or resized in
 * SunshineInput, SunshineState, or SunshineVars.
 * New fields MUST be appended at the END of the struct — never insert.
 *
 * v5: batt_offset widened int8 → int16 (finer battery resolution) and
 *     SunshineState.spin_freq_lp appended (band-pass centre LP, see
 * mag_heading.c). batt_offset is the ONE historical exception to append-only (a
 * mid-struct resize); the host log reader remaps it per schema_version
 * (replay.rs read_input) so all pre-v5 logs still parse. Live/wire format
 * follows the current struct on both ends (brain + app rebuilt together). */
/* v6: SunshineState gains wob_hp / wob_env / wob_ref (appended) — the wobble
 *     damper's accel-z envelope filter states (see WOBBLE_* and control.c).
 *
 * A schema bump changes the TELEMETRY FRAME SIZE, and THREE artifacts compile
 * it from this header: the brain, the app, AND the receiver
 * (sunshine_receiver/src/espnow_rx.cpp drops frames whose length mismatches
 * its compiled ESPNOW_TELEM_SIZE — symptom: controls still work, telemetry
 * dead, red LED). Rebuild and flash ALL THREE from the same commit. */
#define SUNSHINE_SCHEMA_VERSION 6U

/* ── Control modes ─────────────────────────────────────────────────────── */
#define SUNSHINE_MODE_DISABLED 0U
#define SUNSHINE_MODE_TANK 1U
#define SUNSHINE_MODE_MELTY 2U

/* ── Physical / sensor constants ───────────────────────────────────────── */
#define ADXL_SCALE_MS2 (49e-3f * 9.81f) /* m/s² per ADXL375 count  */
#define MAG_SCALE_UT 0.058f             /* µT per LIS3MDL count    */
#define BATT_OFFSET_REF_V 7.6f          /* reference voltage (V)   */
/* batt_offset is int16 (schema v5): 1 mV/LSB, so telemetry no longer
 * discretises the battery (the 12-bit ADC ≈ 2.4 mV/LSB + the 6 Hz LP in
 * batt_read_v are now the only quantisers). ±32.7 V range is far beyond the 2S
 * span; the brain clamps. BATT_SCALE_V_LEGACY is the v4 int8 step, kept only so
 * the host can decode the batt_offset in pre-v5 logs (replay.rs remaps old int8
 * → new int16 LSBs). */
#define BATT_SCALE_V 0.001f         /* V per batt_offset LSB (int16)   */
#define BATT_SCALE_V_LEGACY 0.0205f /* v4 int8 step, for old-log decode */
#define IMU_RADIUS_M 0.011f         /* 11 mm from spin centre  */
#define ADXL_MAX_COUNTS 4082        /* ±200 g / 49 mg·LSB⁻¹  */
/* Min spin for the mag heading. The tracking band-pass is centred on the spin
 * frequency; its half-width is a FRACTION of that frequency (constant-Q), so
 * the lower band edge is ~0.75·fc. Below the minimum, that edge sinks toward
 * the slow-ESC-current band and the 2nd-order skirt (only ~6 dB/oct) no longer
 * rejects it well, and at low ω the tangential-accel inflation of omega_accel
 * is large — so the mag is gated off. 16π rad/s = 8 Hz spin = 480 RPM (lower
 * edge ≈ 6 Hz). (Hard-iron DC itself is killed exactly by the band-pass's zero
 * at DC.) */
#define SUNSHINE_MAG_MIN_OMEGA (16.0f * 3.14159265f) /* ~480 RPM, rad/s  */
/* ── Drivetrain / motor, for the MELTY wheel-speed cap (control.c) ─────────
 * PER-BUILD parameters, read off the mechanical design and the motor nameplate.
 * They are NOT fitted to any log: a different robot changes exactly these
 * values and nothing else in the control path.
 *
 * Why they exist: the AM32 ESC is an open-loop VOLTAGE SOURCE — it applies a
 * duty cycle, it does not close a speed loop — so a wheel's steady no-load
 * speed is a known function of (duty × V_batt). That is what makes the relation
 * invertible, and the cap in control.c is exactly that inverse: given the wheel
 * speed the ground demands, solve for the DShot value whose no-load speed
 * matches it.
 *
 * The failure mode being prevented: without the cap the wheels run 1.3–2.3×
 * faster than the rolling speed the body rate demands. The surplus is stored
 * kinetic energy in the wheels, and every time the robot touches down it is
 * dumped into an impulsive traction spike that throws the robot back into the
 * air — the vertical bounce.
 *
 * MOTOR_KV_RPM_PER_V is the 1100 NAMEPLATE and must NOT be derated to match the
 * ~6% speed deficit seen under load. That deficit is the I·R drop across the
 * winding, i.e. the slip DOING ITS JOB: the cap commands a NO-LOAD speed and
 * the load then pulls the actual speed below it, and that difference is the
 * torque-producing slip. Deriving KV from loaded data would cancel the very
 * margin the cap is supposed to grant.
 *
 * MOTOR_POLE_PAIRS is unused by the cap arithmetic. It exists for the eRPM
 * model (eRPM = mechanical rpm × pole pairs) that the brain-side eRPM filter
 * uses to build its command-derived ceiling.
 *
 * WHEEL_SLIP_ALLOW_MS is THE tuning knob, and it is a fixed slip SPEED, not a
 * percentage of rolling speed. Traction force is I = (V_cmd − backEMF)/R, so a
 * fixed slip speed is a fixed slip VOLTAGE and therefore a fixed current — a
 * fixed available torque at any spin rate. A flat percentage instead would
 * starve the robot of torque at low spin and still permit the runaway at high
 * spin. */
#define WHEEL_RADIUS_M 0.022f      /* wheel rolling radius, m              */
#define WHEEL_CENTER_M 0.0405f     /* wheel contact patch to spin axis, m  */
#define MOTOR_KV_RPM_PER_V 1100.0f /* nameplate, rpm/V (no-load)           */
#define MOTOR_POLE_PAIRS 7         /* 14-pole motor -> 7 pole pairs        */
#define WHEEL_SLIP_ALLOW_MS 3.0f   /* allowed slip at the contact patch, m/s */

/* ── Robot plant / environment (per-build; used by the host SIMULATION only) ──
 * The C core does not read these, but they live here so ALL per-robot numbers
 * are in one file: the app's build.rs parses this header and generates the Rust
 * constants for simulation.rs at build time — there is no second copy to keep
 * in sync. Sources: mass from a scale; MOI/wheel inertia from CAD; R_phase from
 * the motor datasheet (or phase-to-phase resistance / 2); hard-iron and the mag
 * HF tone measured from a real log (see BRINGUP.md "Porting to a New Robot");
 * Earth field = HORIZONTAL component for your location (NOAA calculator) — the
 * robot spins in a horizontal plane, so the vertical component never rotates.
 */
#define ROBOT_MASS_KG 0.454f
#define ROBOT_MOI_KGM2 1.214e-3f /* body yaw inertia                  */
#define WHEEL_INERTIA_KGM2 6.40744019e-6f /* wheel + motor rotor, each */
/* MEASURED FROM LOGS 2026-08-07 (ledger method: commanded-minus-realized slip
 * surplus 0.99 V at delivered 1.08 A during the allowance-3.0 spin-up), and
 * independently confirmed by (a) the wave-following gain it predicts
 * (tau = I_w*R/(Kt*Ke) ~ 76 ms -> gain 0.27 at spin frequency — matching the
 * measured 0.2-0.3 "plant attenuation") and (b) the brake-decel magnitude.
 * The old 0.075 was likely the HIGH-KV variant's datasheet row — an 1100 KV
 * rewind has ~turns^2 more resistance. That single error manufactured the
 * phantom 10x force expectations, the "ESC bandwidth" and "current limit"
 * mysteries, and the original cap sizing. Confirm against the motor spec when
 * known; measure phase-to-phase / 2 if ever unsoldered. */
#define MOTOR_R_PHASE_OHM 0.92f
#define BATT_NOMINAL_V 8.4f /* sim pack voltage (2S full-ish)    */
#define BATT_R_INTERNAL_OHM 0.008f
#define EARTH_FIELD_UT 25.0f /* horizontal component ONLY, µT     */
#define EARTH_ANGLE_RAD 0.0f /* field azimuth (arbitrary ref)     */
/* Hard-iron: body-frame bias from motor magnets/PCB traces, measured as the
 * mean of raw mag_x/mag_y over a spinning log × MAG_SCALE_UT. The heading
 * band-pass kills it by construction; the sim needs it only to GENERATE
 * realistic raw mag data. */
#define HARD_IRON_X_UT -95.0f
#define HARD_IRON_Y_UT 103.0f
/* LIS3MDL 1 kHz low-power-mode sampling artifact: a tone at fs/6 ≈ 167 Hz,
 * measured ~7 µT on X and ~2 µT on Y (spiritridge log). Modelled so the sim
 * exercises the band-pass's HF rejection. */
#define MAG_HF_TONE_HZ 166.6667f
#define MAG_HF_TONE_X_UT 7.0f
#define MAG_HF_TONE_Y_UT 2.0f
/* SIM_*: lumped plant-model fudge factors (not measurable robot properties) —
 * retune only if sim spin-up/translation feel diverges from real logs. */
#define SIM_BODY_DRAG 0.05f       /* body spin drag, 1/s               */
#define SIM_TRANSLATION_DRAG 2.0f /* rolling/carpet loss, 1/s          */
#define SIM_TIRE_DAMPING 10.0f    /* N per m/s of longitudinal slip    */
#define SIM_MAX_TIRE_FORCE 25.0f  /* crude traction limit per wheel, N */
#define SIM_WHEEL_DRAG 2.0e-6f    /* wheel bearing/air drag torque coeff */

/* ── Kalman tuning (override with -D flag for tuning builds) ───────────── */
#ifndef KF_Q_THETA
#define KF_Q_THETA 1e-6f
#endif
#ifndef KF_Q_OMEGA
#define KF_Q_OMEGA 1e-2f
#endif
/* Accelerometer omega-measurement variance. The accel (ω = √(a_c/r)) is the
 * rate sensor at all times. Since the heading is recovered open-loop
 * (mag_heading.c band-passes the Earth sine at the accel-derived spin
 * frequency, independent of the estimate), the accel can't drag the heading
 * into precession, so it is trusted fully and kf_omega tracks omega_from_accel.
 */
#ifndef KF_R_ACCEL
#define KF_R_ACCEL 0.5f
#endif
#ifndef KF_R_MAG
#define KF_R_MAG 0.01f /* open-loop mag heading is a clean absolute reference \
                        */
#endif

/* ── Magnetometer tracking band-pass (open-loop absolute heading) ──────────
 * The Earth field appears at the spin frequency; the body-fixed offset
 * (hard-iron + avg ESC current) is at DC; and 1 kHz low-power-mode sampling
 * adds tones well above the spin band (a strong one at fs/6 ≈ 167 Hz). A
 * 2nd-order RBJ band-pass CENTRED ON THE SPIN FREQUENCY (from omega_from_accel
 * — a direct accel measurement, NOT the heading-coupled kf_omega; see
 * mag_heading.c) isolates the Earth sine: it has a transmission zero at DC
 * (kills hard-iron) and rolls off above, rejecting the HF sampling tones —
 * which a fixed filter can't do because the 167 Hz tone is only ~2.5–4× the
 * 40–66 Hz spin. heading = atan2 of the band-passed axes (open-loop → cannot
 * drift).
 *
 * The bandwidth is a FRACTION of the centre frequency (constant Q), NOT a fixed
 * Hz: the accelerometer's spin-rate error is fractional, so a fixed ±N Hz band
 * would lose the signal at high spin. Bench bias was +2..+12% with ~3%
 * per-sample noise; combat adds linear acceleration + impacts, so we budget
 * conservatively
 * (~2× that ≈ 30%) and set half-bandwidth = fc/(2·MAG_BP_Q) ≈ 33% of fc at
 * Q=1.5. That keeps the true spin in the band even when the accel rate is
 * biased, at the cost of slightly weaker HF-tone rejection (a 4th-order
 * band-pass would sharpen it if needed). Coeffs are recomputed each tick from
 * omega_from_accel (cheap). With constant Q the group delay is a CONSTANT
 * heading offset that theta_offset (the driver zero) absorbs — no
 * speed-dependent shift. Tunable: higher Q = narrower = cleaner but riskier;
 * lower Q = wider = more robust. */
#define MAG_BP_Q 1.5f /* half-BW = fc/(2Q) ≈ 33% of spin freq          */
#define MAG_BP_MIN_FC_HZ \
  8.0f /* clamp centre to the mag-valid speed (480 RPM) */
/* Band-pass CENTRE low-pass (schema v5). The centre was previously retuned
 * every tick from the INSTANTANEOUS omega_from_accel. While translating, linear
 * body acceleration adds a once-per-rev component to the accel, so
 * omega_from_accel wobbles at the spin frequency — and a band-pass whose
 * coefficients wobble tick- to-tick is TIME-VARYING, which injects heading
 * wobble (the LTI "mis-centre only attenuates" argument holds only for a FIXED
 * filter). Measured on real logs: the recovered-heading rate error was 37–100
 * rad/s. Low-passing the centre to ~1.5 Hz makes the filter quasi-LTI and cuts
 * that to 3–5 rad/s (~90%), because the true spin rate changes slowly (~sub-Hz)
 * while the corruption is at the ~15-25 Hz spin band. spinup_lag.py: the true
 * rate stays inside the ±33% pass band ~98.7% of the mag-valid time; the ~1.3%
 * is brief impact glitches, cutoff-independent. The LP is re-seeded below the
 * mag threshold (mag_heading.c) so a fast spin-up carries no lag over a stop.
 * Higher = tracks spin-up faster but rejects less wobble; lower = smoother
 * heading but laggier centre. Loop-independent (uses only the accel rate), so
 * it does NOT reintroduce the kf_omega false-lock. Tunable at bringup Level 4.
 */
#define MAG_BP_FC_LP_HZ \
  1.5f /* band-pass centre LP cutoff, Hz (nav loop = 1 kHz) */
/* LP cutoff for the SIGNED mag rotation rate (spin_rate_lp). Since schema v5
 * this is the Kalman's rate measurement whenever the mag is valid
 * (mag_heading.c / brain.c) — it is unbiased by linear acceleration, so it
 * fixes the translation heading swings the accel magnitude caused. Trade-off
 * knob: higher = snappier tracking of fast spin changes (impact slowdown,
 * spin-up) but a noisier steady heading; lower = smoother but laggier on
 * transients (the mag_angle update still re-anchors, so the lag is
 * momentary). 3.2 Hz preserves the pre-v5 coefficient (0.02 @ 1 kHz). Bringup
 * Level 4. */
#define MAG_SPIN_RATE_LP_HZ 3.2f /* SIGNED mag-rate LP cutoff, Hz */

/* ── Control tuning ────────────────────────────────────────────────────── */
/* 0.25 → 90° ramps (~10 ms at combat spin). The old 0.35 commanded a full wheel
 * reversal in ~6 ms against the measured ~20 ms actuation lag: the plant
 * rounded the edges off anyway (no force gained), while the harmonic content
 * that did get through showed as elevated 2x/3x-rev vibration lines in accel_z
 * during translation (2026-08-06 logs). Wider ramps deliver the same
 * fundamental with less per-edge wheel-KE dump and gyroscopic kick. */
#define DRIFT_PLATEAU_WIDTH \
  0.25f /* fraction of rotation at each +/- peak diff */
/* With the force DIRECTION fixed (see DRIFT_PHASE_OFFSET_RADS / _LEAD_S),
 * amplitude no longer needs to compensate for a misaimed force. What it buys:
 * the wheel-speed swing sets the translation TOP-SPEED ceiling (the advancing
 * wheel must out-run rolling by the robot's ground speed) — low-speed force
 * saturates at tire friction within a modest swing regardless. What it costs:
 * the same swing sets the wheel-rotor gyroscopic tilt kick (the amplitude
 * escalation while the force was misaimed is what drove the edge strikes).
 * 0.30 ≈ ±1.4 m/s ceiling at translating spin. Raise toward 0.45 for top
 * speed once direction is verified clean; drop toward 0.20 if any edge
 * strikes return. */
/* Resized for the MEASURED winding resistance (0.92 ohm, not 0.075): force is
 * current and current is surplus-voltage / R, so meaningful force needs
 * multi-volt swings — 0.60 of the FULL DShot span commands rail-slamming
 * excursions (advancing wheel toward full duty, retreating toward stop;
 * the output clamp shapes the asymmetry) independent of cruise throttle —
 * the classic melty modulation the reference robots use on identical
 * motors/cells. Old throttle-span basis at 40% throttle delivered only
 * +/-1.8 V (2.5 A, ~0.9 N electrical, the measured slow scoot).
 * differential at driving throttles -> ~1.5-2.5 N net after the measured 0.27
 * plant gain (vs the 0.3-0.45 N measured under the old sizing). */
/* 0.60 -> 1.00 (2026-08-07 Post_debug log): the delivered translation force,
 * measured ABSOLUTELY via ground-frame accel demod (calibrated 1g = ~78
 * counts), is only 0.3-0.4 N — an order below the friction ceiling — and the
 * plant realizes just ~±1-1.5 m/s of wheel-speed swing from the 0.60 command
 * (tau = I_w*R/KtKe ~ 78 ms vs the 25-30 Hz rev). Terminal translation speed
 * against spin-scrub lateral drag is ~(F/muN)*|v_slip| ~ 0.2 m/s — the
 * measured "snail pace". Reference melties slam rail-to-rail; amplitude is
 * the one direct multiplier on realized swing that remains. Expect more
 * ground contact while translating (the tilt bill scales with the same
 * swing); the working regime is mid-spin holds, not high spin. */
#define DRIFT_AMPLITUDE 1.00f /* max diff as fraction of FULL DShot span */
/* DERIVED from the build geometry and confirmed by the 2026-08-07 two-speed
 * drift test. The wheels push along the TANGENT of the wheel line (90 deg from
 * the wheel axis); the LED sits ON the wheel axis; and the driver convention is
 * W = toward the light with W encoded as dd = +90 deg. The two 90s stack to
 * exactly 0 or 180 depending on which physical side the firmware's "+diff"
 * wheel sits — no other term contributes (mag zero and filter delays shift the
 * LED and the wave identically and cancel). The binary resolves to 180: with
 * offset 0, W pushed exactly AWAY from the light. Pinning 180 here makes the
 * on-floor readings (+90 deg err at omega~120, +45 at ~165, compiled lead
 * 0.018) internally consistent with a single force lag of ~4-5 ms (see
 * DRIFT_PHASE_LEAD_S). If the "+diff" wheel side ever changes (rewiring,
 * remount, invert flip), this flips between pi and 0 — re-run the drift test.
 */
#define DRIFT_PHASE_OFFSET_RADS \
  3.14159265f /* wheel-side geometry: pi, not 0    */
/* Low-spin translation fade (2026-08-06 logs). Two measured reasons translation
 * must not run at low spin: (1) the collapse trap — an edge-strike slowdown
 * drops kf_omega, the cap follows it down and (with ESC complementary-PWM
 * braking) actively brakes the wheels, parking the robot at 55–60 rad/s for as
 * long as the stick is held, where recovery is throttled by the same cap; (2)
 * the actuation phase lag grows at low spin (wheel-diff lag measured 50–80° at
 * 60–100 rad/s vs ~0–10° at 145–165), so what translation remains is weak and
 * crabbed anyway. Fading drive_mag to zero below FADE_LO breaks the trap: the
 * drift stops, the full slip allowance returns, and the robot spins back up
 * through the band. FADE_LO sits above the 50.3 rad/s mag floor (where the trap
 * parks); FADE_HI is below the ~90+ rad/s range where translation demonstrably
 * works. */
#define DRIFT_OMEGA_FADE_LO \
  60.0f /* rad/s: translation fully off below        */
#define DRIFT_OMEGA_FADE_HI \
  85.0f /* rad/s: full translation authority above   */
/* NOTE (2026-08-07): a high-spin rolloff + spin governor briefly lived here,
 * built on reading the eRPM attenuation at 20+ Hz spin as an ESC/plant
 * bandwidth limit. That reading is confounded: a GRIPPING tire pins wheel
 * speed near rolling regardless of command, so speed attenuation can mean
 * force is being transmitted, not that the plant can't respond. Other melties
 * translate at 2K+ RPM; forcing this robot below ~1000 RPM to translate was
 * treating the symptom. Removed pending the on-floor force-direction test
 * (TUNING.md Level 5 Step 3, two-speed method). */
/* Fraction of the slip allowance the un-bias removes at full stick. 1.0 (remove
 * all of it) was tried on the 2026-08-07 Nosliphopefully log and creates a spin
 * STARVATION equilibrium: holding the pack's drag needs ~0.5 m/s of slip, so
 * with zero bias a held stick bleeds spin until the omega-fade partially
 * restores the allowance — the robot parked at ~75 rad/s (fade ~0.6, measured
 * mean slip +0.53 m/s) for as long as W was held. Keeping (1 - FRAC) of the
 * allowance at full stick pays that drag bill while staying negligible against
 * the multi-m/s wave swing, so no force dead zone returns. Raise FRAC toward 1
 * for purer zero-slip translation (more spin bleed); lower it toward 0 to
 * favour spin (bias creep returns). 0.7: with only ~30% plant gain the REALIZED
 * swing is ~±1 m/s, so the remaining bias must sit well under that for the
 * braking half to exist at all. */
/* SUPERSEDED COUPLING FIX (2026-08-07 concrete log): the bias used to be
 * ALLOW*(1 - FRAC*drive), which couples the TRANSLATION bias to the SPIN-UP
 * allowance knob — raising WHEEL_SLIP_ALLOW_MS for faster spin-up silently
 * raised the translating bias past the realized wave swing, so slip never
 * crossed zero and the differential force was EXACTLY zero (both tires push
 * forward all revolution: pure spin torque; the robot then "translates" only
 * via wobble/contact chaos — the measured only-moves-while-wobbling symptom).
 * The bias while translating is now a FIXED remnant, independent of the
 * allowance: full stick -> DRIFT_TRANSLATE_BIAS_MS, stick released -> full
 * WHEEL_SLIP_ALLOW_MS, linear in between. */
/* DEAD-ZONE WIDTH FIX (2026-08-07 fixed_ log): "linear in between" left a huge
 * force dead zone across most of the stick — at 30% deflection the bias was
 * still ~2.1 m/s, and the eRPM-vs-rolling slip measurement showed the tires in
 * braking slip only 6-12% of each rev (vs ~25-30% at full stick): sound
 * changes, zero force, zero motion — the exact tap-W symptom. Worse, anything
 * that scales drive_mag down (wobble damper, low-spin fade) silently dragged
 * the bias back UP mid-press, so shed oscillations also swung the slip bias
 * (measured +0.7..+1.8 m/s at full stick when the target was 0.05) and
 * scrambled the force direction. The un-bias now completes by
 * DRIFT_UNBIAS_FULL_STICK of deflection: any deliberate press translates on
 * the same near-zero bias as full stick, and drive_mag shedding only shrinks
 * the wave amplitude, never the bias. */
#define DRIFT_UNBIAS_FULL_STICK 0.30f /* drive_mag at which bias reaches remnant */
/* Sim-swept 2026-08-07 with the coast-down-measured tire friction (mu~0.9,
 * NOT the earlier 0.1 misread): bias vs full-stick translation —
 *   0.00 -> 0.93 m/s but 5.8 deg tilt (over the 5.5 edge budget) and -14
 *           rad/s^2 spin drain;
 *   0.05 -> 0.71 m/s, 4.6 deg, spin near-neutral;   <- knee, shipped
 *   0.15 -> 0.41 m/s;  0.40+ -> dead zone (0.26-0.30).
 * MORE bias = SLOWER translation (it shrinks the braking half and pushes both
 * tires forward); it is spin maintenance, not translation authority. */
#define DRIFT_TRANSLATE_BIAS_MS 0.05f /* m/s: slip bias at full deflection */
/* ── Anti-tip swing clamp (the strike mechanism, sim-proven) ────────────────
 * MECHANISM, proven in the coupled 6-DOF simulation (tools/melty6dof.py): the
 * tip is driven by the WHEEL-ROTOR gyroscopic reaction — the drift wave
 * modulates flywheels whose axles the spinning chassis carries around, and the
 * reaction is a tilting moment ~ I_w * omega * dOmega_wheel. In-sim ablation:
 * same config with feather wheels (I_w/13) does not tip AND translates 50%
 * faster; phase correction alone changes nothing (the tip is phase-blind, as
 * the driver argued). The safe commanded swing grows ~linearly with spin
 * (higher spin = gyroscopically stiffer): clamp the NO-LOAD wheel-speed swing
 * to a ratio of the body rate, ramped with omega. Grid-validated (omega
 * 100-220, throttle 100-190, seeds): worst tilt 4.9 deg vs the 5.5 deg edge
 * budget, zero edge contact. On the current low-grip urethane this allows
 * ~0.8 m/s translation; a grippier tire compound both stabilizes tilt further
 * and roughly doubles speed in-sim (grip damps the wobble AND drives harder).
 */
/* Ratios rescaled x2.5 from the sim calibration (0.45/0.75): the sim's wheels
 * realize ~0.8 of commanded swing while the REAL plant realizes only 0.2-0.3
 * (measured, eRPM vs command at spin frequency), and the tilt driver is the
 * REALIZED momentum swing. The sim-validated realized envelope therefore maps
 * to a ~2.5x larger COMMANDED swing on the real robot. Re-derive if the plant
 * gain changes (lighter wheels, ESC settings). */
/* Opened ~2x again for the measured R: the tilt driver is REALIZED wheel-
 * momentum swing, and with tau ~76 ms the wheels realize only ~0.27 of the
 * commanded swing at spin frequency — the prior ratios (sized at an assumed
 * 0.2-0.3 gain against a sim plant that realized far more) were strangling
 * the force. Realized swing at these caps stays at the previously-validated
 * envelope. */
/* Opened 2.0/2.8 -> 3.0/4.2 with the amplitude change above: the old caps
 * bound deep presses to ~220-250 commanded counts, and at that swing the
 * robot BOTH failed to translate AND still struck the ground when held
 * (Post_debug: press-time big-impulse rate 10-18/1000 samples in EVERY omega
 * band — there is no strike-free press regime under the old clamp, so it was
 * paying the tilt bill without buying the force). The only session with
 * visible translation (Moreamplitudeagain, omega ~130, 2-4.6 s holds) ran
 * comparable tilt rates and survived by riding the contact. These caps admit
 * the full-span wave at mid spin and up. */
#define TIP_SWING_RATIO_LO 3.0f /* cmd swing/omega cap below TIP_OMEGA_LO   */
#define TIP_SWING_RATIO_HI 4.2f /* cmd swing/omega cap above TIP_OMEGA_HI   */
#define TIP_OMEGA_LO 125.0f      /* rad/s: ramp start                       */
#define TIP_OMEGA_HI 170.0f      /* rad/s: ramp end (full ratio)            */
/* ── Closed-loop wobble damper (schema v6) ─────────────────────────────────
 * Every measured ground strike is preceded by the same signature: the accel-z
 * vibration envelope swelling well above its riding-clean level (idle ~90-155
 * counts across floors; pre-strike windows 215-430 — the sub-synchronous
 * ~0.5x-rev whirl mode ringing up). No feed-forward constant can prevent it
 * because none can see it; the robot can, at 1 kHz. The damper: high-pass
 * accel-z (DC/tilt removed), rectify into a fast envelope (wob_env), learn the
 * riding-clean reference (wob_ref) ONLY while the drive stick is released, and
 * shed translation authority as env/ref rises through the ratio band — full
 * force while riding clean, proportional backoff only while the measured
 * wobble grows, automatic recovery as it damps. Thresholds from the measured
 * ratios (thrash/idle ≈ 2.3-4.5x). WOBBLE_REF_MIN keeps the ratio sane on an
 * unusually quiet floor. */
/* THRESHOLDS RAISED (2026-08-07 fixed_ log): the 1.6-2.6 band sat INSIDE the
 * vibration band of healthy hard translation. Reconstructing the damper state
 * offline from accel-z showed the press-time env/ref ratio median at 1.7 —
 * pinned to the old shed onset — with the damper oscillating drive_mag between
 * 0.2 and 0.6 every ~0.3 s at full stick. That made it a translation GOVERNOR
 * with a limit cycle: force builds -> normal translating vibration crosses 1.6
 * -> force shed -> vibration falls -> force returns. Net force measured 0.15 N
 * (shed) vs 1.2-1.4 N (free) with the direction smeared by the cycling — the
 * drunken-stagger, only-moves-while-grinding session. Measured bands: idle
 * spin p75 = 1.06 (max 1.95), coherent clean translation 1.1-3.1, strike
 * thrash up to 5.4. The band now starts ABOVE everything translation does
 * (3.5) and sheds fully only at genuine-thrash levels (5.0). */
#define WOBBLE_HP_HZ 3.0f     /* accel-z DC-tracking cutoff (pass the wobble)   */
#define WOBBLE_ENV_HZ 2.5f    /* fast envelope LP: ~60-100 ms reaction          */
#define WOBBLE_REF_HZ 0.15f   /* riding-clean reference LP (learn while idle)   */
#define WOBBLE_RATIO_LO 3.5f  /* env/ref where shedding starts                  */
#define WOBBLE_RATIO_HI 5.0f  /* env/ref where translation is fully shed        */
#define WOBBLE_REF_MIN 40.0f  /* counts: reference floor                        */
/* FORCE-lag compensation — NOT the wheel-speed lag. History: 0.018 was set from
 * translation_lag.py's DShot→eRPM cross-correlation (~20 ms), but eRPM is wheel
 * SPEED, and speed is the INTEGRAL of torque — compensating force with the
 * speed lag rotated the force backwards by omega*(18-10) ms. The tires ride
 * the cap's slip bias, so they are ALWAYS kinetically sliding: contact force is
 * mu*N*sign(slip), and the force wave's timing is set by where the slip SIGN
 * crosses zero — which lags roughly half the speed wave. Calibrated 2026-08-07
 * by a stick-slip first-principles simulation (tools-side melty_sim) fitted to
 * the on-floor two-speed drift-direction observations (+90 deg at ~120 rad/s,
 * +45 at ~165, taps, lead 0.018/offset 0): with DRIFT_PHASE_OFFSET_RADS pinned
 * at the geometrically derived pi, the two observations agree on a single
 * force lag of tau ~ 4-5 ms (120: 4.9 ms, 165: 3.7 ms — the internal agreement
 * is the cross-check). DO NOT re-derive from eRPM cross-correlation (speed
 * domain); re-verify only with the on-floor drift-direction test. */
#define DRIFT_PHASE_LEAD_S \
  0.005f /* force (slip-sign) lag compensation, s      */
#define THETA_RATE_RADS 3.14159265f /* rad/s per full ctrl_theta */
#define MAX_DSHOT_SPIN DSHOT_MAX
#define DSHOT_NEUTRAL 1048.0f
#define DSHOT_MAX 2047.0f
#define DSHOT_MIN 48.0f

/* ── IO layer structs ──────────────────────────────────────────────────── */

/* SunshineInput: 1 kHz sensor frame, 30 bytes packed (schema v5).
 * APPEND-ONLY: never insert, reorder, or resize existing fields. The lone
 * exception is batt_offset, widened int8→int16 at v5 (see
 * SUNSHINE_SCHEMA_VERSION); the host reader remaps pre-v5 int8 batt per
 * schema_version. */
typedef struct __attribute__((packed)) {
  uint32_t time_us;
  int16_t accel_x; /* ADXL375 raw counts; IMU at 45° to radial   */
  int16_t accel_y; /* centripetal + tangential both split here    */
  int16_t accel_z; /* vertical (~+20 cnts = 1g at rest)          */
  int16_t mag_x;   /* LIS3MDL raw counts at ±16 Gauss            */
  int16_t mag_y;
  int16_t mag_z;
  uint16_t erpm_left; /* IEEE-754 float16 bits                      */
  uint16_t erpm_right;
  int8_t rssi;   /* ESP-NOW RSSI at brain (dBm)                */
  int8_t ctrl_x; /* [-127, 127]                                */
  int8_t ctrl_y;
  int8_t ctrl_theta;
  uint8_t ctrl_throttle; /* [0, 255]                                   */
  int16_t batt_offset;   /* v5: relative to 7.6 V, 0.001 V/LSB (int16) */
  uint8_t dshot_left_q;  /* DShot cmd from PREVIOUS tick, quantised    */
  uint8_t dshot_right_q;
  uint8_t mode; /* SUNSHINE_MODE_*                            */
} SunshineInput;
/* static_assert(sizeof(SunshineInput) == 30, ""); */

/* SunshineState: filter history, 52 bytes packed.
 * APPEND-ONLY rule applies here too. */
typedef struct __attribute__((packed)) {
  float kf_theta;     /* Kalman angle estimate (rad, unwrapped)     */
  float kf_omega;     /* Kalman angular velocity estimate (rad/s)   */
  float kf_P[4];      /* 2×2 covariance, row-major [P00,P01,P10,P11]*/
  float theta_offset; /* driver heading offset (rad)                */
  float mag_hp_x[2];  /* mag_x high-pass biquad state (2nd order)   */
  float mag_hp_y[2];  /* mag_y high-pass biquad state               */
  /* Spin-direction recovery (schema v4). The accel gives only |ω| (centripetal
   * magnitude), so kf_omega's SIGN must come from the magnetometer, which sees
   * the true rotation sense — otherwise the heading counter-rotates when the
   * robot is inverted (flip → same chassis spin, opposite world spin). */
  float mag_ang_prev; /* previous mag_angle, for its rotation-rate sign  */
  float spin_rate_lp; /* low-passed SIGNED mag rotation rate (rad/s)      */
  /* Band-pass CENTRE frequency LP (schema v5). Low-passed UNSIGNED spin rate
   * (rad/s) used to centre the mag band-pass — smooths the once-per-rev
   * translation corruption of omega_from_accel. See mag_heading.c /
   * MAG_BP_FC_LP_HZ. */
  float spin_freq_lp;
  /* Wobble damper states (schema v6, see WOBBLE_* and control.c): accel-z
   * DC-tracking low-pass (wob_hp), fast rectified envelope (wob_env), and the
   * slow riding-clean reference envelope (wob_ref, learned only while the
   * drive stick is released). */
  float wob_hp;
  float wob_env;
  float wob_ref;
} SunshineState;
/* static_assert(sizeof(SunshineState) == 68, ""); */

/* SunshineVars: derived variables, never telemetered, 56 bytes packed.
 * APPEND-ONLY: never insert, reorder, or resize existing fields. */
typedef struct __attribute__((packed)) {
  float omega_from_accel; /* rad/s, inflated during spinup            */
  float mag_x_filt;       /* high-passed mag_x (µT); Earth-field sine  */
  float mag_y_filt;       /* high-passed mag_y (µT)                    */
  float mag_angle;        /* open-loop absolute heading atan2 (rad)    */
  float est_theta;        /* = kf_theta                               */
  float est_omega;        /* = kf_omega                               */
  float dshot_cmd_left;   /* [0, 2047], pre-quantisation              */
  float dshot_cmd_right;
  float batt_voltage; /* actual voltage (V)                       */
  float erpm_left;    /* decoded from float16                     */
  float erpm_right;
  float centripetal_ms2;   /* sqrt(ax²+ay²)*ADXL_SCALE_MS2            */
  uint8_t led_on;          /* 1 when within ±3° of zero heading        */
  uint8_t accel_saturated; /* 1 when centripetal > 280g equivalent     */
  uint8_t mag_valid;       /* 1 when est_omega > SUNSHINE_MAG_MIN_OMEGA*/
  uint8_t loop_overrun;    /* 1 when 1kHz tick exceeded 1000µs (HW)   */
  float heading_deg;       /* robot heading [0, 360), matches LED zero */
} SunshineVars;

/* ── Public API ────────────────────────────────────────────────────────── */
#ifdef __cplusplus
extern "C" {
#endif

void sunshine_state_init(SunshineState* state);
void sunshine_step(const SunshineInput* in, SunshineState* state,
                   SunshineVars* vars_out);

void sunshine_input_serialize(const SunshineInput* in, uint8_t* buf);
void sunshine_input_deserialize(const uint8_t* buf, SunshineInput* in);
void sunshine_state_serialize(const SunshineState* state, uint8_t* buf);
void sunshine_state_deserialize(const uint8_t* buf, SunshineState* state);

uint32_t sunshine_schema_version(void);

float sunshine_accel_to_ms2(int16_t raw);
float sunshine_mag_to_ut(int16_t raw);
float sunshine_batt_to_v(int16_t off); /* v5: int16, 0.001 V/LSB */
float sunshine_f16_to_f32(uint16_t half);
uint16_t sunshine_f32_to_f16(float f);

#ifdef __cplusplus
}
#endif
