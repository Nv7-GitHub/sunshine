#!/usr/bin/env python
"""Analyse replay.exe CSV output.

This does NOT reimplement any robot logic — it only reads the CSV that
replay.exe (which links the real sunshine_core) emits. Three checks:

  1. validate   — reseed CSV: replayed est_theta vs stored (real) est_theta.
                  Should be ~0 deg. Non-zero => replay engine / float drift bug.
  2. gaps       — continuous CSV: 1 kHz input timestamp continuity. Big gaps
                  (>1.5 ms) mean dropped inputs => continuous replay can't track.
  3. precession — continuous CSV over a gap-free MELTY window: compares the LED
                  heading-reference rate to the TRUE spin rate measured straight
                  from the raw magnetometer (field rotation about its hard-iron
                  centre). Non-zero difference = the LED precesses by that much.

Usage:
  replay.exe LOG.sun --reseed > reseed.csv
  replay.exe LOG.sun           > cont.csv
  python analyze.py validate   reseed.csv
  python analyze.py gaps       cont.csv
  python analyze.py precession cont.csv
"""
import sys, csv, numpy as np

def load(path):
    with open(path) as f:
        r = csv.reader(f); hdr = next(r); rows = list(r)
    idx = {h: i for i, h in enumerate(hdr)}
    def col(name):
        i = idx[name]
        return np.array([np.nan if row[i] == '' else float(row[i]) for row in rows])
    return col
def wrap(x): return (x + np.pi) % (2*np.pi) - np.pi
def phasor_rate(t, ang):
    """Mean angular rate, alias-free for <pi/step (good to ~500 rev/s @1kHz)."""
    return np.sum(wrap(np.diff(ang))) / (t[-1] - t[0])

def steady_melty_block(col, omega_min=200.0, gap_free=False):
    t = col('time_us'); mode = col('mode'); kfom = col('kf_omega')
    ok = (mode == 2) & (kfom > omega_min)
    if gap_free:
        dt = np.diff(t); g = (dt >= 900) & (dt <= 1100)
        ok[:-1] &= g; ok[-1] = False
    idx = np.where(ok)[0]
    if len(idx) < 50: return None
    blocks = np.split(idx, np.where(np.diff(idx) > 1)[0] + 1)
    return max(blocks, key=len)

def cmd_validate(path):
    c = load(path); rep = c('est_theta'); st = c('stored_est_theta'); mode = c('mode')
    fe = ~np.isnan(st)
    d = np.degrees(np.abs(wrap(rep[fe] - st[fe]))); mm = mode[fe]
    print("RESEED validation — replayed vs real (stored) est_theta:")
    for m, name in [(0, 'DISABLED'), (1, 'TANK'), (2, 'MELTY')]:
        s = mm == m
        if s.sum(): print(f"  {name}: n={s.sum()} mean={d[s].mean():.3f} max={d[s].max():.3f} deg")
    print("  (expect ~0; non-zero => replay/float-determinism bug)")

def cmd_gaps(path):
    c = load(path); t = c('time_us').astype(np.int64); dt = np.diff(t)
    dt = dt[(dt > 0) & (dt < 1e8)]  # drop the single uint32 micros() wrap
    print("CONTINUOUS input timestamp continuity (1 kHz expected):")
    print(f"  samples={len(t)}  ~1ms steps={100*np.mean(np.abs(dt-1000)<=4):.1f}%")
    big = dt > 1500
    print(f"  gaps >1.5ms: {big.sum()} (lost ~{int((dt[big]/1000 - 1).sum())} inputs)")
    print("  (each gap = dropped 1 kHz inputs => continuous replay diverges there)")

def precession_of(c, g):
    """(true_Omega, led_rate, precession, derot_uT, earth_uT) over index block g."""
    t = c('time_us')[g] / 1e6
    mx, my = c('mag_x')[g], c('mag_y')[g]
    ox, oy = mx.mean(), my.mean()
    # Body field rotates at ±Omega about its hard-iron centre → |rate| = true spin.
    Om_true = abs(phasor_rate(t, np.arctan2(my - oy, mx - ox)))
    r_led   = abs(phasor_rate(t, wrap(c('kf_theta')[g] + c('theta_offset')[g])))
    derot   = np.hypot(c('mag_x_filt')[g], c('mag_y_filt')[g]).mean()
    earth   = np.hypot(mx - ox, my - oy).mean() * 0.058
    return Om_true, r_led, r_led - Om_true, derot, earth

def cmd_precession(path):
    """Report LED precession (heading rate − raw-mag ground-truth Ω) across the
    longest gap-free MELTY windows. The bias varies with speed, so a single
    window is not enough — check that ALL are small."""
    c = load(path)
    t = c('time_us'); mode = c('mode'); kfom = c('kf_omega')
    ok = (mode == 2) & (kfom > 150)
    dt = np.diff(t); ok[:-1] &= (dt >= 900) & (dt <= 1100); ok[-1] = False
    idx = np.where(ok)[0]
    if len(idx) < 50:
        print("no gap-free steady MELTY window found"); return
    blocks = [b for b in np.split(idx, np.where(np.diff(idx) > 1)[0] + 1) if len(b) >= 300]
    blocks.sort(key=len, reverse=True)
    print("LED precession = heading-ref rate - raw-mag true Omega  (target |.| < 0.5 rad/s):")
    for b in blocks[:6]:
        Om, rled, prec, derot, earth = precession_of(c, b)
        flag = "  <-- transient/outlier?" if abs(prec) > 8 else ""
        print(f"  {len(b)/1000:.2f}s @ {Om*60/2/np.pi:4.0f} RPM:  precession = {prec:+6.2f} rad/s "
              f"({prec/2/np.pi:+.2f} rev/s)  derot {derot:.1f}/{earth:.0f}uT{flag}")

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print(__doc__); sys.exit(2)
    {'validate': cmd_validate, 'gaps': cmd_gaps, 'precession': cmd_precession}[sys.argv[1]](sys.argv[2])
