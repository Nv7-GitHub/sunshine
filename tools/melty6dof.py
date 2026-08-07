#!/usr/bin/env python
"""Full coupled 6-DOF melty simulation.

Rigid body (quaternion attitude, body-frame Euler equations, dynamic spin),
two driven wheels with motor electrodynamics and slip-dependent (Stribeck)
tire friction resolved as 2-D contact forces, unilateral vertical tire
springs, an edge ring with unilateral contact + kinetic friction, per-wheel
ground roughness, exact wheel-rotor gyroscopic reaction (d/dt of wheel spin
momentum carried on the rotating chassis), and the firmware's actual MELTY
control law with configurable constants.

Everything emerges from contact forces — no hand-inserted "mechanism" moments.
"""
import math
import numpy as np

# ── robot constants (sunshine_core.h) ──
M=0.454; G=9.81
IS=1.214e-3; IT=0.7e-3
IW=6.40744019e-6
WC=0.0405; RW=0.022
KV_SI=1100.0*2*math.pi/60.0; KE=1.0/KV_SI; KT=KE; RM=0.075
VB=7.6
HCG=0.021                 # CG height above floor (level, tires uncompressed)
Z_AXLE=RW-HCG             # wheel axle height in body frame (rel CG): 0.001
R_EDGE=0.060; CLEAR=0.0058
Z_RING=-(HCG-CLEAR)       # ring plane height in body frame: -0.0152
KT_TIRE=15000.0; CT_TIRE=30.0
KT_EDGE=30000.0; CT_EDGE=30.0
MU_EDGE=0.35              # edge-vs-asphalt kinetic friction (calibrate: drain)
BODY_DRAG=1.5e-5          # N*m*s/rad about spin (aero+bearing, calib: top speed)
# Stribeck tire friction: mu(s) = MU_C + (MU_S-MU_C)*exp(-s/S_STRIBECK)
MU_S=1.1; MU_C=0.12; S_STRIBECK=0.25
S_REG=0.03                # regularization slip (m/s) for smooth force at s->0
DSHOT_N=1048.0; DSHOT_MAX=2047.0
MAG_MIN=16*math.pi

def mu_of(s):
    return MU_C + (MU_S-MU_C)*math.exp(-s/S_STRIBECK)

def qmul(a,b):
    w1,x1,y1,z1=a; w2,x2,y2,z2=b
    return (w1*w2-x1*x2-y1*y2-z1*z2, w1*x2+x1*w2+y1*z2-z1*y2,
            w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2)

def qrot(q,v):
    # rotate vector v by quaternion q
    w,x,y,z=q
    t=(2*(y*v[2]-z*v[1]), 2*(z*v[0]-x*v[2]), 2*(x*v[1]-y*v[0]))
    return (v[0]+w*t[0]+y*t[2]-z*t[1],
            v[1]+w*t[1]+z*t[0]-x*t[2],
            v[2]+w*t[2]+x*t[1]-y*t[0])

def qconj(q): return (q[0],-q[1],-q[2],-q[3])

def drift_wave(p, plateau):
    p = math.remainder(p, 2*math.pi)
    hf = plateau*math.pi
    ramp = math.pi - 2*hf
    ap = abs(p)
    if ap <= hf: return 1.0
    if ap >= math.pi-hf: return -1.0
    return 1.0-2*(p-hf)/ramp if p>0 else 1.0+2*(p+hf)/ramp

def simulate(cfg, T=4.0, dt=4e-5, seed=2, log_dec=50):
    rng = np.random.default_rng(seed)
    # config: control constants
    LEAD=cfg.get('lead',0.005); OFFS=cfg.get('offset',math.pi)
    AMP=cfg.get('amp',0.30); PLATEAU=cfg.get('plateau',0.25)
    UNBIAS=cfg.get('unbias',0.7); ALLOW=cfg.get('allow',1.0)
    FADE_LO=cfg.get('fade_lo',60.0); FADE_HI=cfg.get('fade_hi',85.0)
    DIFF_BASIS=cfg.get('diff_basis','span')   # 'span' (new) or 'headroom' (old)
    TIP_CLAMP=cfg.get('tip_clamp',None)       # None or (frac_total, cg_h, plant_gain)
    THR=cfg.get('throttle',128)/255.0
    DRIVE_T0=cfg.get('drive_t0',1.2)          # when W is pressed
    SIGMA_R=cfg.get(' sigma_r',cfg.get('sigma_r',0.10e-3)); TAU_R=0.002
    OM0=cfg.get('om0',150.0)
    # state
    q=(1.0,0,0,0)                # body->world
    Om=[0.0,0.0,OM0]             # body angular velocity
    pos=[0.0,0.0,HCG-M*G/(2*KT_TIRE)]
    vel=[0.0,0.0,0.0]
    ww=[OM0*WC/RW+0.3/RW, OM0*WC/RW+0.3/RW]
    g_r=[0.0,0.0]
    duty=[0.3,0.3]
    n=int(T/dt); ctrl_every=max(1,int(0.001/dt))
    out={'t':[],'tilt':[],'om':[],'v':[],'az':[],'edgeN':[],'wwd':[]}
    edge_time=0.0; t_first_edge=None
    for i in range(n):
        t=i*dt
        # world axes of body
        ex=qrot(q,(1,0,0)); ey=qrot(q,(0,1,0)); ez=qrot(q,(0,0,1))
        tilt=math.acos(max(-1,min(1,ez[2])))
        om_spin=Om[2]                    # body-z spin rate
        # yaw angle for control (project body x)
        psi=math.atan2(ex[1],ex[0])
        # ── control @1 kHz ──
        if i%ctrl_every==0:
            drive = 1.0 if t>DRIVE_T0 else 0.0
            dm=drive*min(max((abs(om_spin)-FADE_LO)/(FADE_HI-FADE_LO),0.0),1.0)
            allow=ALLOW*(1-UNBIAS*dm)
            w_cap=max(abs(om_spin),MAG_MIN)*WC/RW+allow/RW
            cap=DSHOT_N+(w_cap/KV_SI)/VB*(DSHOT_MAX-DSHOT_N)
            spin_span=THR*(DSHOT_MAX-DSHOT_N)
            base=min(DSHOT_MAX,max(min(DSHOT_N+spin_span,cap),DSHOT_N))
            ph=psi-math.pi/2+OFFS+om_spin*LEAD     # drive dir = +y (W)
            if DIFF_BASIS=='span':
                basis=spin_span
            else:
                basis=max(0.0,min(base-DSHOT_N,DSHOT_MAX-base))
            diff=drift_wave(ph,PLATEAU)*dm*AMP*basis
            if TIP_CLAMP is not None:
                frac,cgh,pg=TIP_CLAMP
                budget=frac*M*G*WC
                rpc=(VB/(DSHOT_MAX-DSHOT_N))*KV_SI
                npc=pg*IW*abs(om_spin)*rpc
                if npc>1e-9:
                    dmax=budget/npc
                    diff=max(-dmax,min(dmax,diff))
            l=max(DSHOT_N,min(DSHOT_MAX,base+diff))
            r=max(DSHOT_N,min(DSHOT_MAX,base-diff))
            duty=[(l-DSHOT_N)/(DSHOT_MAX-DSHOT_N),(r-DSHOT_N)/(DSHOT_MAX-DSHOT_N)]
            # roughness OU (per control tick is fine, fast dynamics not needed)
            k_ou=0.001/TAU_R
            for j in range(2):
                g_r[j]+= -g_r[j]*k_ou + SIGMA_R*math.sqrt(2*k_ou)*rng.standard_normal()
        # ── forces ──
        F=[0.0,0.0,-M*G]; TAU_W=[0.0,0.0,0.0]   # world force, world torque about CG
        ww_dot=[0.0,0.0]
        for j,sx in enumerate((1.0,-1.0)):
            # wheel axle point (body): (sx*WC, 0, Z_AXLE); contact under it
            pb=(sx*WC,0.0,Z_AXLE)
            pw=qrot(q,pb); pw=(pw[0]+pos[0],pw[1]+pos[1],pw[2]+pos[2])
            # contact height: wheel bottom = axle z - RW (assume wheel stays ~vertical)
            h=pw[2]-RW-g_r[j]
            # rolling (tangential) direction: body +/-y rotated, horizontal proj
            tb=(0.0,sx*1.0,0.0)
            tw=qrot(q,tb); tn=math.hypot(tw[0],tw[1])
            tw=(tw[0]/tn,tw[1]/tn) if tn>1e-6 else (0.0,1.0)
            lw=(-tw[1],tw[0])                      # lateral dir
            # contact point velocity (chassis)
            rc=(pw[0]-pos[0],pw[1]-pos[1],pw[2]-RW-pos[2])
            Ow=qrot(q,(Om[0],Om[1],Om[2]))
            vc=(vel[0]+Ow[1]*rc[2]-Ow[2]*rc[1],
                vel[1]+Ow[2]*rc[0]-Ow[0]*rc[2],
                vel[2]+Ow[0]*rc[1]-Ow[1]*rc[0])
            if h<0:
                N=-KT_TIRE*h - CT_TIRE*vc[2]
                if N<0: N=0.0
            else:
                N=0.0
            if N>0:
                # slip: chassis contact velocity + wheel surface velocity
                s_long=(vc[0]*tw[0]+vc[1]*tw[1]) - ww[j]*RW
                s_lat = vc[0]*lw[0]+vc[1]*lw[1]
                s=math.hypot(s_long,s_lat)
                mu=mu_of(s)
                f=mu*N*s/math.sqrt(s*s+S_REG*S_REG)
                fl=-f*(s_long/max(s,1e-9)); fla=-f*(s_lat/max(s,1e-9))
                Fx=fl*tw[0]+fla*lw[0]; Fy=fl*tw[1]+fla*lw[1]
                F[0]+=Fx; F[1]+=Fy; F[2]+=N
                TAU_W[0]+=rc[1]*N - rc[2]*Fy
                TAU_W[1]+=rc[2]*Fx - rc[0]*N
                TAU_W[2]+=rc[0]*Fy - rc[1]*Fx
                tire_tq=fl*RW                       # torque on wheel from ground
            else:
                tire_tq=0.0
            V=duty[j]*VB
            ww_dot[j]=(KT*(V-KE*ww[j])/RM + tire_tq)/IW
        # edge ring: lowest point in direction of tilt
        if tilt>1e-4:
            # horizontal direction of downhill: -ez horizontal component
            dx,dy=ez[0],ez[1]; dn=math.hypot(dx,dy)
            ux,uy=(-dx/dn,-dy/dn) if dn>1e-9 else (1.0,0.0)
            # ring point (body frame direction closest to downhill): approximate
            # world position: CG + R_EDGE*(u) + Z_RING*ez
            pe=(pos[0]+R_EDGE*ux+Z_RING*ez[0],
                pos[1]+R_EDGE*uy+Z_RING*ez[1],
                pos[2]+R_EDGE*(ux*0+uy*0)+Z_RING*ez[2] - R_EDGE*dn)  # z drop from tilt
            he=pe[2]
        else:
            he=1.0
        if he<0:
            rc=(pe[0]-pos[0],pe[1]-pos[1],pe[2]-pos[2])
            Ow=qrot(q,(Om[0],Om[1],Om[2]))
            vce=(vel[0]+Ow[1]*rc[2]-Ow[2]*rc[1],
                 vel[1]+Ow[2]*rc[0]-Ow[0]*rc[2],
                 vel[2]+Ow[0]*rc[1]-Ow[1]*rc[0])
            Ne=-KT_EDGE*he - CT_EDGE*vce[2]
            if Ne<0: Ne=0.0
            sp=math.hypot(vce[0],vce[1])
            if sp>1e-6:
                Fex=-MU_EDGE*Ne*vce[0]/sp; Fey=-MU_EDGE*Ne*vce[1]/sp
            else:
                Fex=Fey=0.0
            F[0]+=Fex; F[1]+=Fey; F[2]+=Ne
            TAU_W[0]+=rc[1]*Ne - rc[2]*Fey
            TAU_W[1]+=rc[2]*Fex - rc[0]*Ne
            TAU_W[2]+=rc[0]*Fey - rc[1]*Fex
            edge_time+=dt
            if t_first_edge is None and t>0.2: t_first_edge=t
            edgeN=Ne
        else:
            edgeN=0.0
        # wheel-rotor gyroscopic reaction: -d/dt(sum IW*ww_j*a_j), a=body x*(+/-1)
        Ow=qrot(q,(Om[0],Om[1],Om[2]))
        for j,sx in enumerate((1.0,-1.0)):
            aw=qrot(q,(sx,0.0,0.0))
            dLdt=(IW*ww_dot[j]*aw[0]+IW*ww[j]*(Ow[1]*aw[2]-Ow[2]*aw[1]),
                  IW*ww_dot[j]*aw[1]+IW*ww[j]*(Ow[2]*aw[0]-Ow[0]*aw[2]),
                  IW*ww_dot[j]*aw[2]+IW*ww[j]*(Ow[0]*aw[1]-Ow[1]*aw[0]))
            TAU_W[0]-=dLdt[0]; TAU_W[1]-=dLdt[1]; TAU_W[2]-=dLdt[2]
        TAU_W[2]-=BODY_DRAG*Om[2]*abs(Om[2])**0.0  # linear drag term
        # ── integrate ──
        tb=qrot(qconj(q),tuple(TAU_W))
        Om[0]+=dt*(tb[0]-(Om[1]*IS*Om[2]-Om[2]*IT*Om[1]))/IT
        Om[1]+=dt*(tb[1]-(Om[2]*IT*Om[0]-Om[0]*IS*Om[2]))/IT
        Om[2]+=dt*(tb[2]-(Om[0]*IT*Om[1]-Om[1]*IT*Om[0]))/IS
        Ow=qrot(q,tuple(Om))
        dq=qmul((0.0,0.5*Ow[0]*dt,0.5*Ow[1]*dt,0.5*Ow[2]*dt),q)
        q=(q[0]+dq[0],q[1]+dq[1],q[2]+dq[2],q[3]+dq[3])
        qn=math.sqrt(sum(c*c for c in q)); q=tuple(c/qn for c in q)
        for k in range(3):
            vel[k]+=dt*F[k]/M; pos[k]+=dt*vel[k]
        for j in range(2):
            ww[j]+=dt*ww_dot[j]
            if ww[j]<0: ww[j]=0.0
        if i%log_dec==0:
            out['t'].append(t); out['tilt'].append(tilt)
            out['om'].append(Om[2]); out['v'].append(math.hypot(vel[0],vel[1]))
            out['az'].append(F[2]/M-G); out['edgeN'].append(edgeN)
    for k in out: out[k]=np.array(out[k])
    return out, edge_time, t_first_edge

def report(name,cfg,T=4.0):
    o,et,tfe=simulate(cfg,T=T)
    m=o['t']>cfg.get('drive_t0',1.2)
    pre=(o['t']>0.4)&(o['t']<cfg.get('drive_t0',1.2))
    print(f"{name:<44} tilt_pre={np.degrees(o['tilt'][pre].max()):5.2f}d "
          f"tilt_max={np.degrees(o['tilt'][m].max() if m.any() else 0):5.2f}d "
          f"v_max={o['v'].max():4.2f} om {o['om'][0]:.0f}->{o['om'][-1]:5.0f} "
          f"edge%={100*et/T:4.1f} t_edge={tfe if tfe else float('nan'):.2f}")
    return o

if __name__=="__main__":
    print("=== calibration: OLD firmware constants (Aug-6 era) ===")
    old=dict(lead=0.018, offset=0.0, amp=0.60, plateau=0.35, unbias=0.0,
             diff_basis='headroom', fade_lo=-1e9, fade_hi=-1e9+1, om0=150.0,
             throttle=128, drive_t0=1.2)
    report("A idle only (drive never)", {**old,'drive_t0':99.0}, T=3.0)
    report("B OLD constants, hold W @ om0=150", old)
    print("=== current build (6e7e14d) ===")
    cur=dict(lead=0.005, offset=math.pi, amp=0.30, plateau=0.25, unbias=0.7,
             diff_basis='span', om0=150.0, throttle=128, drive_t0=1.2,
             tip_clamp=(0.75*(1-0.15),0.021,0.5))
    report("C current constants, hold W", cur)
