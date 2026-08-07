#!/usr/bin/env python
"""First-principles planar melty simulation from the robot's actual constants.

Physics per wheel:
  motor: V = duty*Vb;  I = (V - Ke*w_wheel)/R;  torque = Kt*I
  tire (stick-slip Coulomb):
    contact speed error  s = w_wheel*r_w - v_contact_ground
    if |s| < eps and |motor torque| <= mu*N*r_w:  GRIP  -> wheel speed follows
        ground kinematics, contact force = motor torque / r_w (instantaneous!)
    else: SLIDE -> force = -mu*N*sign(s) on wheel (reaction +mu*N*sign(s) on
        ground->robot), wheel accelerates by (torque - f*r_w)/I_w
Body: M, I_z; wheels at +/-WHEEL_CENTER on body x-axis, driving along body y.
Control: the actual control.c law (cap, unbias, amplitude, trapezoid, phase
constants) + an injected HARDWARE mapping offset H (rad) representing any
sign/geometry error between dshot_cmd_l/r and the physical wheels.
"""
import numpy as np
import sys

# ── constants from sunshine_core.h ──
WR, WCn = 0.022, 0.0405
KV = 1100.0; KE = 1.0/(KV*2*np.pi/60); KT = KE
Rm = 0.075
IW = 6.40744019e-6
M, IZ = 0.454, 1.214e-3
VB = 7.6
MU = 0.8
G = 9.81
DSHOT_N, DSHOT_MAX = 1048.0, 2047.0
MAG_MIN = 16*np.pi
PLATEAU = 0.25
FADE_LO, FADE_HI = 60.0, 85.0

def drift_wave(p):
    p = np.mod(p+np.pi, 2*np.pi)-np.pi
    hf = PLATEAU*np.pi
    ramp = np.pi - 2*hf
    ap = abs(p)
    if ap <= hf: return 1.0
    if ap >= np.pi-hf: return -1.0
    return 1.0 - 2*(p-hf)/ramp if p > 0 else 1.0 + 2*(p+hf)/ramp

def simulate(omega0, LEAD, OFFSET, H, AMP=0.45, UNBIAS=0.7, ALLOW=1.0,
             T=2.5, drive_dir=np.pi/2, dt=1e-4):
    th = 0.0; om = omega0
    vx = vy = 0.0
    ww = np.array([om*WCn/WR + ALLOW*0.3/WR]*2)   # wheel speeds, near rolling
    N = M*G/2
    logv = []
    for k in range(int(T/dt)):
        t = k*dt
        # ── control at 1 kHz ──
        if k % 10 == 0:
            drive = 1.0 if t > 0.2 else 0.0
            dm = drive*min(max((abs(om)-FADE_LO)/(FADE_HI-FADE_LO),0),1)
            allow = ALLOW*(1-UNBIAS*dm)
            w_ref = max(abs(om), MAG_MIN)
            w_cap = w_ref*WCn/WR + allow/WR
            v_need = w_cap/(KV*2*np.pi/60)
            cap = DSHOT_N + v_need/VB*(DSHOT_MAX-DSHOT_N)
            base = min(DSHOT_MAX, max(cap, DSHOT_N))
            head = min(base-DSHOT_N, DSHOT_MAX-base)
            ph = th - drive_dir + OFFSET + om*LEAD
            diff = drift_wave(ph)*dm*AMP*head
            duty = np.array([(base+diff-DSHOT_N)/999.0, (base-diff-DSHOT_N)/999.0]).clip(0,1)
        # ── plant ──
        # wheel geometry: wheels on body x-axis at +/-WC, tangential = body y
        cs, sn = np.cos(th), np.sin(th)
        Fx = Fy = tau = 0.0
        # hardware offset H: rotates which physical angle the diff pattern hits
        # (equivalent to rotating wheel line by -H relative to assumed)
        for i,sgn in enumerate((+1,-1)):
            # wheel position in world (with hardware offset H applied)
            a = th + H + (0 if sgn>0 else np.pi)
            px, py = WCn*np.cos(a), WCn*np.sin(a)
            # tangential direction (direction wheel drives the ground contact)
            tx, ty = -np.sin(a), np.cos(a)
            # ground-contact velocity of that point on the robot
            vcx = vx - om*py; vcy = vy + om*px
            v_along = vcx*tx + vcy*ty
            V = duty[i]*VB
            Im = (V - KE*ww[i])/Rm
            tq = KT*Im
            s = ww[i]*WR - v_along
            if abs(s) < 0.02 and abs(tq) <= MU*N*WR:
                f = tq/WR                       # GRIP: force = motor torque/r
                ww[i] = v_along/WR              # wheel follows ground
            else:
                f = MU*N*np.sign(s)             # SLIDE
                ww[i] += (tq - f*WR)/IW*dt
                ww[i] = max(ww[i], 0)
            Fx += f*tx; Fy += f*ty
            tau += px*(f*ty) - py*(f*tx)
        vx += Fx/M*dt; vy += Fy/M*dt
        vx *= (1-2.0*dt); vy *= (1-2.0*dt)      # SIM_TRANSLATION_DRAG
        om += tau/IZ*dt; om *= (1-0.05*dt)      # SIM_BODY_DRAG
        th += om*dt
        if k % 100 == 0: logv.append((t, vx, vy, om))
    logv = np.array(logv)
    # steady drift direction/speed over last 1 s
    m = logv[:,0] > T-1.0
    vxm, vym = logv[m,1].mean(), logv[m,2].mean()
    sp = np.hypot(vxm, vym)
    ang = np.degrees(np.arctan2(vym, vxm))
    err = (ang - np.degrees(drive_dir) + 180) % 360 - 180
    return sp, err, logv[m,3].mean()

H_FIT = np.radians(210.0)
print("=== A) as-flashed constants (LEAD=0.018, OFFSET=0) + hardware offset H ===")
print("   reproduce the observed clock test?  (observed: +90 deg @ ~w120, +45 @ ~w165)")
for H in [0.0, H_FIT]:
    for om0 in (120.0, 165.0):
        sp, err, omf = simulate(om0, 0.018, 0.0, H)
        print(f"  H={np.degrees(H):4.0f}deg w0={om0:3.0f}: drift err={err:+6.0f} deg  speed={sp:5.2f} m/s  w_end={omf:5.0f}")
print("\n=== B) corrected constants (LEAD=0.002, OFFSET=-2.62) with H=210 present ===")
for om0 in (100.0, 120.0, 165.0, 190.0):
    sp, err, omf = simulate(om0, 0.002, -2.62, H_FIT)
    print(f"  w0={om0:3.0f}: drift err={err:+6.0f} deg  speed={sp:5.2f} m/s  w_end={omf:5.0f}")
