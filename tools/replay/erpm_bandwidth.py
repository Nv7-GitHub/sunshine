#!/usr/bin/env python
"""Measure the DShot->eRPM transfer at the SPIN frequency vs spin rate.
The MELTY drift wave modulates each wheel's DShot once per revolution; the wheel
(motor+ESC+inertia) responds with a lagged, attenuated eRPM ripple. This quantifies:
  - GAIN  = |eRPM ripple| / |DShot ripple| at fc  (relative; falls if the wheel
            can't keep up -> "wheels won't change speed as much at high RPM")
  - LAG   = phase(eRPM) - phase(DShot) at fc, in ms and deg
The lag in ms -> DRIFT_PHASE_LEAD_S; the deg lag = omega*lag grows with spin rate.
Robust to eRPM spike/zero dropouts (median-clip). Usage: erpm_bandwidth.py cont.csv"""
import sys, csv, numpy as np

def load(p):
    r = csv.reader(open(p)); h = next(r); rows = list(r); i = {x: j for j, x in enumerate(h)}
    return lambda n: np.array([np.nan if x[i[n]] == '' else float(x[i[n]]) for x in rows]), len(rows)

def demod(sig, fc, t):
    """Complex amplitude of sig at fc (Hz) over the window (t in us)."""
    ts = (t - t[0]) / 1e6
    ph = 2 * np.pi * fc * ts
    ref = np.exp(-1j * ph)
    return 2.0 * np.mean((sig - np.mean(sig)) * ref)

def declip(x):
    """Kill bidirectional-DShot eRPM dropouts (0 or saturated) via median window."""
    x = x.copy()
    med = np.median(x[np.isfinite(x)])
    bad = ~np.isfinite(x) | (x <= 1.0) | (x > 8 * max(med, 1))
    x[bad] = np.nan
    # linear-interp over the bad samples
    idx = np.arange(len(x)); good = ~np.isnan(x)
    if good.sum() < 10: return None
    x[~good] = np.interp(idx[~good], idx[good], x[good])
    return x

c, n = load(sys.argv[1]); lab = sys.argv[2] if len(sys.argv) > 2 else sys.argv[1]
t = c('time_us'); mode = c('mode'); kfom = c('kf_omega'); oacc = c('omega_accel')
cx = c('ctrl_x'); cy = c('ctrl_y'); dl = c('dshot_l'); el = c('erpm_left')
xy = np.hypot(cx, cy)
d = np.diff(t); cont = np.ones(n, bool); cont[:-1] &= (d >= 900) & (d <= 1100); cont[-1] = False
translating = (mode == 2) & (np.abs(kfom) > 40) & cont & (xy > 5)
idx = np.where(translating)[0]
blks = [b for b in np.split(idx, np.where(np.diff(idx) > 1)[0] + 1) if len(b) >= 1500]
blks.sort(key=len, reverse=True)
print(f"###### {lab}: DShot->eRPM transfer at spin freq ({len(blks)} translating windows) ######")
print(f"  {'dur':>5} {'fc_Hz':>6} {'|dshot|':>8} {'|erpm|':>8} {'gain':>7} {'lag_ms':>7} {'lag_deg':>8}")
rows = []
for b in blks[:14]:
    dur = (t[b][-1] - t[b][0]) / 1e6
    fc = np.nanmean(oacc[b]) / (2 * np.pi)
    if fc < 8: continue
    er = declip(el[b])
    if er is None: continue
    D = demod(dl[b], fc, t[b]); E = demod(er, fc, t[b])
    if abs(D) < 1e-3: continue
    gain = abs(E) / abs(D)
    dphase = np.angle(E) - np.angle(D)          # rad; eRPM relative to DShot
    dphase = (dphase + np.pi) % (2 * np.pi) - np.pi
    lag_ms = -dphase / (2 * np.pi * fc) * 1000  # positive lag = eRPM behind
    lag_deg = np.degrees(-dphase)
    rows.append((fc, gain, lag_ms))
    print(f"  {dur:5.1f} {fc:6.1f} {abs(D):8.1f} {abs(E):8.1f} {gain:7.2f} {lag_ms:7.1f} {lag_deg:8.1f}")
if rows:
    rows = np.array(rows); rows = rows[rows[:, 0].argsort()]
    print(f"\n  gain vs fc (is it dropping at high fc? => bandwidth limit):")
    lo = rows[rows[:, 0] < np.median(rows[:, 0])]; hi = rows[rows[:, 0] >= np.median(rows[:, 0])]
    if len(lo) and len(hi):
        print(f"    low fc  ~{lo[:,0].mean():.1f} Hz: gain={lo[:,1].mean():.2f}  lag={lo[:,2].mean():.1f} ms")
        print(f"    high fc ~{hi[:,0].mean():.1f} Hz: gain={hi[:,1].mean():.2f}  lag={hi[:,2].mean():.1f} ms")
    print(f"  median lag = {np.median(rows[:,2]):.1f} ms  -> DRIFT_PHASE_LEAD_S candidate")
