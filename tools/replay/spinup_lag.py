#!/usr/bin/env python
"""Does LP-ing the band-pass centre lag fast spin-ups enough to break the mag?
The constant-Q band-pass (half-width fc/(2Q)=~33% of centre) still passes the Earth
sine while the TRUE spin freq stays within ~[0.67,1.33]*centre. On a rising spin-up
the LP centre lags below the truth; it must stay > 0.75*f_true to keep the sine in
band. We: (1) find spin-up ramps, measure max d(spin_freq)/dt (Hz/s); (2) simulate
1-pole LPs at several cutoffs, report worst-case band error (f_true-fc_lp)/f_true and
how long it exceeds the 25% margin (= seconds of degraded heading per spin-up).
Also re-checks translation-wobble rejection at each cutoff. Usage: spinup_lag.py cont.csv"""
import sys, csv, numpy as np

def load(p):
    r = csv.reader(open(p)); h = next(r); rows = list(r); i = {x: j for j, x in enumerate(h)}
    return lambda n: np.array([np.nan if x[i[n]] == '' else float(x[i[n]]) for x in rows]), len(rows)

def lp1(x, fc, fs=1000.0):
    a = 1 - np.exp(-2*np.pi*fc/fs); y = np.empty_like(x); acc = x[0]
    for n in range(len(x)):
        acc += a*(x[n]-acc); y[n] = acc
    return y

c, n = load(sys.argv[1]); lab = sys.argv[2] if len(sys.argv) > 2 else sys.argv[1]
t = c('time_us'); mode = c('mode'); oacc = c('omega_accel'); kfom = c('kf_omega')
sat = c('accel_sat')
fs_hz = oacc/(2*np.pi)                       # instantaneous spin FREQ (Hz) from accel
dt = np.gradient(t)/1e6
# clean: only where dt sane
good = np.isfinite(fs_hz) & (dt > 5e-4) & (dt < 2e-3)
# spin-up ramps: mag is only USED above 480 RPM (8 Hz). Rate of change of spin freq:
dfdt = np.gradient(fs_hz, t/1e6)             # Hz per second
dfdt[~good] = 0
valid = fs_hz > 8.0                          # region where the mag/band-pass matters
print(f"###### {lab}: spin-up dynamics ######")
print(f"  spin-freq (mag-valid, >8Hz): median={np.median(fs_hz[valid]):.1f} Hz  max={np.nanmax(fs_hz[valid]):.1f} Hz")
rise = dfdt[valid & (dfdt > 0)]
print(f"  RISING d(spinFreq)/dt (Hz/s): median={np.median(rise):.0f}  p90={np.percentile(rise,90):.0f}  p99={np.percentile(rise,99):.0f}  max={rise.max():.0f}")

print(f"\n  LP cutoff | worst band-err | % time band-err>25% (mag valid) | est degraded-time/min")
CUT = [1.5, 3.0, 5.0, 8.0]
for fc_lp in CUT:
    cen = lp1(fs_hz, fc_lp)                   # LP-centred band centre (Hz)
    # band error on the RISING side that pushes the true freq out of band:
    # need cen > 0.75*f_true; err = (f_true-cen)/f_true ; >0.25 means out of band
    err = (fs_hz - cen)/np.maximum(fs_hz, 1e-6)
    m = valid & good
    out = m & (err > 0.25)
    frac = out.sum()/max(m.sum(),1)
    worst = err[m].max() if m.sum() else 0
    # seconds per minute of run where heading is degraded
    degper_min = frac*60
    print(f"   {fc_lp:4.1f} Hz  | {worst:+13.2f} | {100*frac:6.2f}%                        | {degper_min:5.1f} s/min")

# translation-wobble rejection check at each cutoff (reuse band-pass demod idea):
def rbj_tv(x, fc_arr, Q=1.5, fs=1000.0):
    y=np.zeros_like(x); w=[0.0,0.0]
    for i in range(len(x)):
        fc=max(fc_arr[i],8.0); w0=2*np.pi*fc/fs; cw=np.cos(w0); sw=np.sin(w0); a=sw/(2*Q)
        a0=1+a; b0=a/a0; b2=-a/a0; a1=-2*cw/a0; a2=(1-a)/a0
        wn=x[i]-a1*w[0]-a2*w[1]; y[i]=b0*wn+b2*w[1]; w[1]=w[0]; w[0]=wn
    return y
def rbj_fixed(x, fc, Q=1.5, fs=1000.0):
    w0=2*np.pi*fc/fs; cw=np.cos(w0); sw=np.sin(w0); a=sw/(2*Q)
    a0=1+a; b0=a/a0; b2=-a/a0; a1=-2*cw/a0; a2=(1-a)/a0
    y=np.zeros_like(x); w=[0.0,0.0]
    for i in range(len(x)):
        wn=x[i]-a1*w[0]-a2*w[1]; y[i]=b0*wn+b2*w[1]; w[1]=w[0]; w[0]=wn
    return y
def wrap(x): return (x+np.pi)%(2*np.pi)-np.pi
magx=c('mag_x')*0.058; magy=c('mag_y')*0.058; cx=c('ctrl_x'); cy=c('ctrl_y')
xy=np.hypot(cx,cy)
cont=np.ones(n,bool); d=np.diff(t); cont[:-1]&=(d>=900)&(d<=1100); cont[-1]=False
trans=(mode==2)&(np.abs(kfom)>40)&cont&(xy>5)
idx=np.where(trans)[0]; blks=[b for b in np.split(idx,np.where(np.diff(idx)>1)[0]+1) if len(b)>=1500]
blks.sort(key=len,reverse=True)
if blks:
    print(f"\n  translation heading-wobble (rate-err std, rad/s) vs LP cutoff [{min(6,len(blks))} windows]:")
    hdr="   window "+" ".join(f"{fc:>6.1f}Hz" for fc in CUT)+"   perc-tick"
    print(hdr)
    for b in blks[:6]:
        fcmean=np.nanmean(oacc[b])/(2*np.pi)
        ref=np.arctan2(-rbj_fixed(magy[b],fcmean), rbj_fixed(magx[b],fcmean))
        warm=400; row=[]
        for fc_lp in CUT:
            cen=lp1(oacc[b],fc_lp)*(0.5/np.pi)
            h=np.arctan2(-rbj_tv(magy[b],cen), rbj_tv(magx[b],cen))
            dh=wrap(np.diff(h[warm:])); df=wrap(np.diff(ref[warm:])); dd=(t[b][warm+1:]-t[b][warm:-1])/1e6
            ok=(np.abs(dh)<0.6)&(np.abs(df)<0.6)&(dd>0)
            row.append(np.std((dh[ok]-df[ok])/dd[ok]))
        cenpt=oacc[b]*(0.5/np.pi)
        hp=np.arctan2(-rbj_tv(magy[b],cenpt), rbj_tv(magx[b],cenpt))
        dh=wrap(np.diff(hp[warm:])); df=wrap(np.diff(ref[warm:])); dd=(t[b][warm+1:]-t[b][warm:-1])/1e6
        ok=(np.abs(dh)<0.6)&(np.abs(df)<0.6)&(dd>0); ptc=np.std((dh[ok]-df[ok])/dd[ok])
        print(f"   {(t[b][-1]-t[b][0])/1e6:5.1f}s  "+" ".join(f"{v:8.2f}" for v in row)+f"   {ptc:8.2f}")
