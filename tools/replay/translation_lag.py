#!/usr/bin/env python
"""Measure the DShot->wheel actuation delay and derive DRIFT_PHASE_LEAD_S.

The MELTY drift wave only translates the robot if the wheel-speed differential
peaks when the wheels point along the commanded direction. Any delay between
the DShot command and the wheel actually changing speed rotates the force by
omega * delay — at 1000+ RPM a ~15 ms delay is a >100 degree error and the
robot wobbles instead of translating.

This script measures that delay from a log, robot-agnostically (no physical
constants involved — it is pure signal analysis between the logged command
differential and the logged eRPM differential, so it works unchanged on any
robot/motor/ESC combination):

  1. finds translation windows (MELTY, spinning, drive stick held);
  2. cross-correlates cmd diff (dshot_l - dshot_r) against response diff
     (erpm_left - erpm_right) in the TIME domain. Unlike single-frequency
     demodulation (erpm_bandwidth.py) this is NOT ambiguous modulo one
     rotation period — the search is restricted to [0, 0.9 * period), where
     the first correlation peak is the physical delay;
  3. checks the residual CONSTANT phase offset (should be ~0; a large one
     means a phase-convention problem -> DRIFT_PHASE_OFFSET_RADS, not lead);
  4. subtracts the eRPM telemetry filter lag (median-5 in dshot.cpp, ~3 ms)
     and prints the recommended DRIFT_PHASE_LEAD_S.

Usage:
  translation_lag.py LOG.sun          (runs tools/replay/build/replay itself)
  translation_lag.py cont.csv         (pre-generated replay CSV)

Record the log per BRINGUP.md Level 5 "Characterize the actuation delay":
moderate spin throttle, hold each of W/A/S/D for ~2-3 s. Bouncing/airborne
time is fine — windows are analysed independently and the median is robust.
"""
import sys, os, csv, subprocess, tempfile
import numpy as np

ERPM_TELEM_LAG_S = 0.003   # median-5 eRPM filter + 1-tick log alignment (dshot.cpp)
OMEGA_MIN  = 60.0          # rad/s: analyse only real spin (delay >~ period is aliased below this)
DRIVE_MIN  = 30.0          # |ctrl| counts: stick actually held
MIN_WIN    = 400           # ms: minimum window
MAX_LAG_S  = 0.060         # search ceiling (physical delays are well below this)


def load_csv(path):
    r = csv.reader(open(path)); h = next(r); rows = list(r)
    i = {x: j for j, x in enumerate(h)}
    return (lambda n: np.array([np.nan if x[i[n]] == '' else float(x[i[n]]) for x in rows])), len(rows)


def replay_to_csv(sun_path):
    """Run the replay binary on a .sun log, return the CSV path."""
    here = os.path.dirname(os.path.abspath(__file__))
    for cand in (os.path.join(here, 'build', 'replay'),
                 os.path.join(here, 'build', 'replay.exe'),
                 os.path.join(here, 'build', 'Release', 'replay.exe')):
        if os.path.exists(cand):
            out = tempfile.NamedTemporaryFile(suffix='.csv', delete=False)
            subprocess.run([cand, sun_path], stdout=out, check=True)
            out.close()
            return out.name
    sys.exit("replay binary not found — build it first (see tools/replay/CMakeLists.txt)")


def interp_nan(x):
    x = np.asarray(x, float).copy()
    good = np.isfinite(x)
    if good.sum() < 10: return None
    idx = np.arange(len(x))
    x[~good] = np.interp(idx[~good], idx[good], x[good])
    return x


def main():
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(2)
    path = sys.argv[1]
    if path.endswith('.sun'):
        path = replay_to_csv(path)
    c, n = load_csv(path)

    t = c('time_us') / 1e6
    mode, kfom = c('mode'), c('kf_omega')
    cx, cy = c('ctrl_x'), c('ctrl_y')
    # Use the LOGGED commands (what the robot actually sent), not the replayed
    # ones, so the measurement is independent of any host-side constant changes.
    dl, dr = c('input_dshot_l'), c('input_dshot_r')
    el, er = c('erpm_left'), c('erpm_right')
    th = c('kf_theta') + c('theta_offset')
    dd = np.arctan2(cy, cx)

    ok = (mode == 2) & (np.abs(kfom) > OMEGA_MIN) & (np.hypot(cx, cy) > DRIVE_MIN)
    dt = np.diff(t * 1e6)
    ok[1:] &= (dt > 500) & (dt < 1500)          # gap-free 1 kHz only
    idx = np.where(ok)[0]
    blocks = [b for b in np.split(idx, np.where(np.diff(idx) > 50)[0] + 1)
              if len(b) >= MIN_WIN]
    if not blocks:
        sys.exit("no translation windows found — record per BRINGUP.md Level 5 "
                 "(MELTY, spinning, drive stick held >0.4 s)")

    print(f"{len(blocks)} translation windows (MELTY, |omega|>{OMEGA_MIN:.0f} rad/s, stick held)")
    print(f"{'win':>4} {'dur_s':>6} {'omega':>6} {'delay_ms':>9} {'peak_r':>7} {'offset_deg':>11}")
    results = []
    for bi, g in enumerate(blocks):
        om = kfom[g].mean()                      # signed body rate
        a = interp_nan(dl[g] - dr[g])
        e = interp_nan(el[g] - er[g])
        if a is None or e is None: continue
        a -= a.mean(); e -= e.mean()
        # xcorr, lag restricted to [0, min(MAX_LAG, 0.9*period)) — first peak
        # inside one rotation is the physical delay, no aliasing possible.
        period_ms = 1000.0 * 2 * np.pi / max(abs(om), 1e-6)
        L = int(min(MAX_LAG_S * 1000, 0.9 * period_ms))
        if len(a) <= 2 * L + 10: continue
        cc = np.array([np.corrcoef(a[:len(a)-L], e[k:len(e)-L+k])[0, 1]
                       for k in range(L + 1)])
        k = int(np.argmax(cc))
        if cc[k] < 0.3: continue                 # no coherent response
        # constant-offset residual: lock-in phase of response at the drift-wave
        # phase, minus what the measured pure delay predicts. Should be ~0.
        ph = th[g] - dd[g]
        s0 = e - e.mean()
        I = 2 * np.mean(s0 * np.cos(ph)); Q = 2 * np.mean(s0 * np.sin(ph))
        resid = np.degrees((np.arctan2(Q, I) - om * k / 1000.0 + np.pi) % (2 * np.pi) - np.pi)
        results.append((k, cc[k], resid))
        print(f"[{bi:2d}] {t[g][-1]-t[g][0]:6.1f} {om:+6.0f} {k:9d} {cc[k]:7.2f} {resid:+11.0f}")

    if not results:
        sys.exit("windows found but no coherent cmd->eRPM response "
                 "(check eRPM telemetry health at Level 2)")
    d = np.array(results)
    delay_ms = np.median(d[:, 0])
    offset = np.median(d[:, 2])
    lead = delay_ms / 1000.0 - ERPM_TELEM_LAG_S
    print(f"\nmedian delay          = {delay_ms:.0f} ms over {len(d)} windows")
    print(f"eRPM telemetry lag    = {ERPM_TELEM_LAG_S*1000:.0f} ms (subtracted)")
    print(f"=> DRIFT_PHASE_LEAD_S = {lead:.3f}f   (sunshine_core.h)")
    print(f"median offset residual = {offset:+.0f} deg "
          f"({'OK, leave DRIFT_PHASE_OFFSET_RADS = 0' if abs(offset) < 25 else 'LARGE — investigate phase convention / DRIFT_PHASE_OFFSET_RADS'})")


if __name__ == '__main__':
    main()
