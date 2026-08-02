#!/usr/bin/env python
"""Would the MELTY wheel-speed cap have helped, and is 1.0 m/s the right allowance?

The wheels spin up far faster than the ground demands, then get dragged back down
on every touchdown; each landing dumps the stored wheel energy into an impulsive
traction spike. The cap inverts the ESC's open-loop voltage->speed model and
clamps the mean wheel command to the DShot whose NO-LOAD speed equals the speed
the ground demands plus a fixed slip allowance.

This script reads ONLY replay.exe's CSV. It reimplements no robot logic except
the cap arithmetic itself (design §3.2), which is exactly the thing under
evaluation: it lives here so WHEEL_SLIP_ALLOW_MS can be swept over the real log
without rebuilding firmware. Everything else — kf_omega, mag_valid, spin_rate_lp,
batt_voltage, the DShot the pre-cap firmware actually sent — comes from the CSV.

Reference identity, so the output is checkable by eye (design §6):
    eRPM = |w_body| * WHEEL_CENTER_M/WHEEL_RADIUS_M * 60/(2*pi) * POLE_PAIRS
         = 123.06 * |w_body|      at zero slip
Section 2 prints the measured ratio against that; it MUST have a hard floor at
1.00 (a wheel cannot roll slower than the ground unless it is braking). If it
does not, the geometry constants below have drifted out of sync with the
firmware and the rest of the report is void.

Usage:
  replay.exe LOG.sun > cont.csv
  python wheel_slip.py cont.csv [--allow 0.5,0.75,1.0,1.5,2.0] [--vs-replay]
"""
import sys, csv, numpy as np

# ---- MIRROR of sunshine_core.h. These are NOT authoritative: the firmware's
# values are. A mismatch here silently evaluates a different cap than the robot
# ships, and the §2.1/§2.3 ratio regression guard in section 2 is the only
# automatic detector of it — do not drop that check as redundant.
WHEEL_RADIUS_M       = 0.022        # mechanical design
WHEEL_CENTER_M       = 0.0405       # mechanical design (wheel centre from spin axis)
MOTOR_KV_RPM_PER_V   = 1100.0       # motor nameplate
MOTOR_POLE_PAIRS     = 7            # motor spec (14 poles)
DSHOT_NEUTRAL        = 1048.0       # DShot value at zero throttle
DSHOT_MAX            = 2047.0       # DShot full scale
MAG_MIN_OMEGA        = 16.0 * np.pi # SUNSHINE_MAG_MIN_OMEGA, ~480 RPM
ERPM_PER_RADS        = WHEEL_CENTER_M / WHEEL_RADIUS_M * 60/(2*np.pi) * MOTOR_POLE_PAIRS
DEFAULT_ALLOW_MS     = 1.0          # WHEEL_SLIP_ALLOW_MS the build ships

DRIVEN_MARGIN = 20.0   # DShot counts above neutral before eRPM is measurable at all


def load(p):
    r = csv.reader(open(p)); h = next(r); rows = list(r); i = {x: j for j, x in enumerate(h)}
    return lambda n: np.array([np.nan if x[i[n]] == '' else float(x[i[n]]) for x in rows]), len(rows)


def pct(x, ps=(5, 25, 50, 75, 95)):
    x = x[np.isfinite(x)]
    if x.size == 0: return [np.nan]*len(ps)
    return [np.percentile(x, p) for p in ps]


def select(c, n):
    """The sample population, defined exactly once. A report over an unstated
    population is not evidence, so every clause is counted and printed."""
    mode = c('mode'); t = c('time_us')
    # replay dead-reckons across logged telemetry gaps; those samples are
    # extrapolations, not measurements. Same continuity test analyze.py uses.
    cont = np.ones(n, bool); d = np.diff(t)
    cont[:-1] &= (d >= 900) & (d <= 1100); cont[-1] = False
    melty  = mode == 2                                  # the cap is MELTY-only (§3.7)
    # eRPM is unmeasurable at neutral: at zero throttle AM32 coasts with the FETs
    # off, so there is no commutation to time (§2.5 — zero in 77.6% of neutral
    # samples vs 0.01% of driven ones). Undriven samples are not wheel speeds.
    driven = (c('input_dshot_l') > DSHOT_NEUTRAL + DRIVEN_MARGIN)
    finite = np.isfinite(c('erpm_left')) & np.isfinite(c('erpm_right'))
    m = melty & cont & driven & finite
    print(f"population: {m.sum()} of {n} samples ({100*m.sum()/max(n,1):.1f}%)")
    for name, clause in [("not MELTY", ~melty), ("timestamp gap", ~cont),
                         ("undriven (<=neutral+%d)" % DRIVEN_MARGIN, ~driven),
                         ("eRPM not finite (NaN dropout)", ~finite)]:
        print(f"  dropped by {name:32s} {100*clause.mean():5.1f}%")
    return m, cont


def cap_dshot(kf_omega, mag_valid, spin_rate_lp, v_batt, allow_ms):
    """Vectorised transcription of design §3.2, in that exact order so it can be
    diffed against control.c line by line.

    The single max() in w_ref is what makes the cap CONTINUOUS across the lock
    boundary (§3.3): lock is defined by |kf_omega| > MAG_MIN_OMEGA, so "not
    locked" itself carries the bound |omega| <= MAG_MIN_OMEGA. The cap never
    switches off; it falls back to the threshold rate.
    """
    locked   = (mag_valid != 0) & (np.abs(spin_rate_lp) > MAG_MIN_OMEGA)
    w_ref    = np.maximum(np.where(locked, np.abs(kf_omega), 0.0), MAG_MIN_OMEGA)
    w_roll   = w_ref * WHEEL_CENTER_M / WHEEL_RADIUS_M
    w_cap    = w_roll + allow_ms / WHEEL_RADIUS_M
    v_needed = w_cap * 60/(2*np.pi) / MOTOR_KV_RPM_PER_V
    cap      = DSHOT_NEUTRAL + (v_needed / v_batt) * (DSHOT_MAX - DSHOT_NEUTRAL)
    cap      = np.maximum(cap, DSHOT_NEUTRAL)
    # Fail-open on an implausible battery reading (§3.8): a stuck-low sense must
    # not disarm the weapon mid-match. +inf rather than dropping the sample, so
    # the fail-open path is visibly COUNTED in the report instead of silently
    # shrinking the population.
    bad_v = (v_batt < 5.0) | (v_batt > 10.0) | ~np.isfinite(v_batt)
    cap = np.where(bad_v, np.inf, cap)
    return cap, locked, bad_v


def main():
    args = [a for a in sys.argv[1:]]
    vs_replay = '--vs-replay' in args
    if vs_replay: args.remove('--vs-replay')
    allows = [0.5, 0.75, 1.0, 1.5, 2.0]
    if '--allow' in args:
        i = args.index('--allow'); allows = [float(x) for x in args[i+1].split(',')]
        del args[i:i+2]
    if not args: print(__doc__); sys.exit(2)
    path = args[0]

    c, n = load(path)
    try:
        vb_all = c('batt_voltage'); srlp_all = c('spin_rate_lp')
    except KeyError:
        sys.exit("this CSV predates the batt_voltage/spin_rate_lp columns; "
                 "rebuild replay and regenerate it")

    print(f"###### {path}: MELTY wheel-speed cap ######")
    m, cont = select(c, n)
    if m.sum() < 100: sys.exit("too few samples to report on")

    kfom  = c('kf_omega')[m]
    mval  = c('mag_valid')[m]
    srlp  = srlp_all[m]
    vb    = vb_all[m]
    cmd_l = c('input_dshot_l')[m]     # the DShot the PRE-CAP firmware actually sent
    cmd_r = c('input_dshot_r')[m]     # (= "real.*"); reimplements nothing.
    # Note: input_dshot_* is the previous tick's command, quantised at
    # 2047/255 ~= 8.03 counts (see dequantize_dshot in replay.c). Both are
    # negligible against cap reductions of hundreds of counts.
    el    = c('erpm_left')[m]
    er    = c('erpm_right')[m]

    # ---- battery context. This is the evidence for "battery voltage must appear
    # in the cap" (§2.2) and the visible consumer of the new replay.c column.
    W_REF = 100.0   # rad/s body, a fixed reference so only v_batt varies
    caps_ref = [DSHOT_NEUTRAL + ((W_REF*WHEEL_CENTER_M/WHEEL_RADIUS_M
                                  + DEFAULT_ALLOW_MS/WHEEL_RADIUS_M)*60/(2*np.pi)
                                 / MOTOR_KV_RPM_PER_V / v) * (DSHOT_MAX-DSHOT_NEUTRAL)
                for v in (np.nanmin(vb), np.nanmedian(vb), np.nanmax(vb))]
    print(f"\nbattery over the population: min={np.nanmin(vb):.2f} med={np.nanmedian(vb):.2f} "
          f"max={np.nanmax(vb):.2f} V")
    print(f"  cap DShot at |w_body|={W_REF:.0f} rad/s, allow={DEFAULT_ALLOW_MS} m/s: "
          f"{caps_ref[0]:.0f} / {caps_ref[1]:.0f} / {caps_ref[2]:.0f}  "
          f"(spread {caps_ref[0]-caps_ref[2]:.0f} counts across the pack sag)")

    # ---- 1. commanded vs capped DShot ------------------------------------
    print("\n--- 1. commanded vs capped DShot (commanded = logged pre-cap firmware output)")
    print(f"  {'allow':>6} {'binds%':>7} {'med drop':>9} {'p90 drop':>9} "
          f"{'med cmd':>8} {'med cap':>8} {'unlocked%':>10} {'failopen%':>10}")
    for a in allows:
        cap, locked, bad_v = cap_dshot(kfom, mval, srlp, vb, a)
        capped = np.minimum(cmd_l, cap)
        drop = cmd_l - capped
        binds = drop > 0.5
        print(f"  {a:6.2f} {100*binds.mean():6.1f}% {np.median(drop[binds]) if binds.any() else 0:9.0f} "
              f"{np.percentile(drop[binds],90) if binds.any() else 0:9.0f} "
              f"{np.median(cmd_l):8.0f} {np.median(capped):8.0f} "
              f"{100*(~locked).mean():9.1f}% {100*bad_v.mean():9.1f}%")

    # ---- 2. measured slip distribution ------------------------------------
    # v_wheel: actual tyre speed. v_roll: the speed the ground demands.
    print("\n--- 2. measured slip distribution (m/s at the tyre), pre-cap")
    v_roll = np.abs(kfom) * WHEEL_CENTER_M
    v_wl = el / MOTOR_POLE_PAIRS * (2*np.pi/60) * WHEEL_RADIUS_M
    v_wr = er / MOTOR_POLE_PAIRS * (2*np.pi/60) * WHEEL_RADIUS_M
    print(f"  {'':>6} {'p5':>7} {'p25':>7} {'p50':>7} {'p75':>7} {'p95':>7}")
    for name, vw in (("left", v_wl), ("right", v_wr)):
        print(f"  {name:>6} " + " ".join(f"{q:7.2f}" for q in pct(vw - v_roll)))

    # REGRESSION GUARD, not new analysis: this dimensionless ratio must reproduce
    # the design's measured table (p5..p95 = 1.05/1.17/1.30/1.55/2.27) and, above
    # all, must have its floor at ~1.00. A wheel cannot roll slower than the
    # ground unless it is braking, so a floor away from 1.00 means the geometry
    # constants (or the replay) have drifted and this report means nothing.
    print("\n  ratio erpm / (123.06 * |kf_omega|)  [expect floor ~1.00, p50 ~1.30]")
    spinning = np.abs(kfom) > MAG_MIN_OMEGA
    ratios = {}
    for name, e in (("left", el), ("right", er)):
        r = e[spinning] / (ERPM_PER_RADS * np.abs(kfom[spinning]))
        ratios[name] = r
        lo = pct(r, (0.5, 1, 2, 5, 25, 50))
        print(f"  {name:>6} " + " ".join(f"{q:7.3f}" for q in lo) + "   (0.5/1/2/5/25/50 %ile)")
    # Tolerance is +-5%, not +-2%. The discriminating claim in §2.3 is that "a 10%
    # geometry error would have placed [the floor] at 0.90 or 1.10", so 5% is the
    # tightest band that still separates a correct build from the error this guard
    # exists to catch. It must also not fire on the design's OWN evidence: §2.3's
    # right-wheel 2%ile is 0.973, which a +-2% band would have rejected. The floor
    # is a percentile of a noisy ratio, so its exact value moves with the sample
    # population (this script gates on MELTY+driven+continuous, §2.3 gated on
    # driven+mag-valid); only a gross shift means the constants are wrong.
    floor = np.nanmin([np.percentile(r[np.isfinite(r)], 2) for r in ratios.values()])
    if not (0.95 <= floor <= 1.05):
        print(f"  !! REGRESSION: 2%ile floor is {floor:.3f}, not 1.00 +-5%. The geometry")
        print(f"     constants in this file no longer match the firmware, or the replay")
        print(f"     has drifted. THE REST OF THIS REPORT IS VOID.")
    else:
        print(f"  ok: 2%ile floor {floor:.3f} lands on 1.00 within 5% — geometry confirmed")

    # ---- 3. slip under the cap + acceleration preservation ------------------
    # (a) is an UPPER BOUND on post-cap slip, not a simulation — the design
    # explicitly rules out simulating it (no vertical DOF, wrong tyre model).
    # It is a bound because the cap commands a NO-LOAD speed and load can only
    # pull the wheel BELOW it, so post-cap wheel speed <= v_roll + allow, and
    # also <= whatever the wheel actually managed unclamped.
    print("\n--- 3a. slip if the cap had been applied (upper bound, m/s)")
    print(f"  {'allow':>6} {'p5':>7} {'p25':>7} {'p50':>7} {'p75':>7} {'p95':>7}")
    for a in allows:
        bounded = np.minimum(v_wl, v_roll + a) - v_roll
        print(f"  {a:6.2f} " + " ".join(f"{q:7.2f}" for q in pct(bounded)))

    # (b) how much of the acceleration the robot actually demonstrated would the
    # cap have touched? alpha by central difference over ~50 ms, gated on the
    # same continuity mask so a dead-reckoned gap cannot manufacture a ramp.
    HALF = 25   # samples; ~50 ms window at 1 kHz
    om_full = c('kf_omega')
    alpha = np.full(n, np.nan)
    alpha[HALF:n-HALF] = (om_full[2*HALF:] - om_full[:n-2*HALF]) / (2*HALF*1e-3)
    win_ok = np.convolve(cont.astype(float), np.ones(2*HALF+1), 'same') == (2*HALF+1)
    accel = m & win_ok & np.isfinite(alpha) & (np.abs(alpha) > 15.0)   # matches §2.4's population
    print(f"\n--- 3b. acceleration preserved  (n={accel.sum()} samples with |alpha| > 15 rad/s^2)")
    # The quantity here is SLIP VOLTAGE (duty*V_batt - rpm/KV), not tyre slip
    # speed, and the distinction is the whole point of the section. Tyre slip
    # (wheel speed - ground speed) measures OVERSPEED, which is what the cap
    # removes; comparing it to the allowance just restates section 1's binds%
    # and says nothing about lost acceleration. Torque is what acceleration
    # costs, and torque is I = (V_cmd - backEMF)/R, i.e. slip VOLTAGE (§3.5).
    # §3.6 justifies the 1.0 m/s default by exactly this comparison: it is
    # 0.395 V, ~2x the median slip voltage the robot used while accelerating.
    # When the wheel fully grips (actual speed == rolling speed) the entire
    # allowance appears across the winding, so allow -> V is the conversion
    # below and a sample whose used slip voltage is under it was never limited.
    if accel.sum() < 50:
        print("  too few accelerating samples to judge")
    else:
        sub   = accel[m]                   # re-index onto the selected population
        duty  = (cmd_l - DSHOT_NEUTRAL) / (DSHOT_MAX - DSHOT_NEUTRAL)
        v_slip = (duty*vb - (el/MOTOR_POLE_PAIRS)/MOTOR_KV_RPM_PER_V)[sub]
        v_slip = v_slip[np.isfinite(v_slip)]
        med = np.median(v_slip)
        print("  slip voltage used while accelerating (V): " +
              " ".join(f"p{p}={np.percentile(v_slip,p):.3f}" for p in (10, 25, 50, 75, 90)))
        for a in allows:
            va = a / WHEEL_RADIUS_M * 60/(2*np.pi) / MOTOR_KV_RPM_PER_V
            print(f"  allow {a:4.2f} m/s = {va:5.3f} V = {va/med:4.2f}x the median used; "
                  f"{100*np.mean(v_slip < va):5.1f}% of accelerating samples were under it")
    # No pass/fail bar here on purpose. The design does not assert a percentile
    # threshold the allowance must clear — it argues from the MULTIPLE of the
    # median (§3.6). A hard bar invented here would fail the shipped 1.0 m/s
    # default for disagreeing with a rule the design never made.
    print("\nverdict: read the multiple above against §3.6 — the default 1.0 m/s is "
          "justified as ~2x the median slip voltage used while accelerating, not by "
          "clearing a percentile bar.")

    # ---- 4. optional cross-check of this file's Python against the real C -----
    if vs_replay:
        # dshot_l here is the REPLAYED, cap-applied output of the real control.c
        # ("rep.*"). Restricted to samples with no drift differential
        # (hypot(ctrl_x,ctrl_y)==0), because §3.7 deliberately lets the drift wave
        # ride ABOVE the capped base — that asymmetry is the translation
        # mechanism, not a discrepancy.
        rep = c('dshot_l')[m]
        drive_mag = np.hypot(c('ctrl_x')[m], c('ctrl_y')[m])
        cap, _, _ = cap_dshot(kfom, mval, srlp, vb, DEFAULT_ALLOW_MS)
        mine = np.minimum(cmd_l, cap)
        sel = (drive_mag == 0) & (cap < cmd_l) & np.isfinite(rep)
        print(f"\n--- 4. python cap vs replayed control.c  (n={sel.sum()} binding, no-translation)")
        if sel.sum() == 0:
            print("  no binding no-translation samples — is this a PRE-cap replay binary?")
        else:
            d = np.abs(rep[sel] - mine[sel])
            print(f"  |rep.dshot_l - min(cmd, cap)|: median={np.median(d):.1f}  max={d.max():.1f} counts")
            print("  (non-trivial disagreement => control.c and the design have diverged)")


if __name__ == '__main__':
    main()
