import { useEffect, useRef } from 'react';
import type { RefObject } from 'react';
import { invoke } from '@tauri-apps/api/core';
import type { Mode } from '../types/sunshine';

export interface InputState {
  x:        number;
  y:        number;
  theta:    number;
  throttle: number;
}

// ── X/Y shaping: "joystick without a joystick" ───────────────────────────────
// Goal: feather WASD the way you would in a game — spam the key on and off to
// hold a partial speed — on a robot that has no analogue axis. X/Y therefore run
// through a first-order lag, NOT the linear slew this used to use: a slew
// integrates at a fixed rate while a key is down and decays at a fixed rate once
// it is up, so there is no level it can settle on and pulsing W only sawtooths
// between 0 and a spike. A lag converges on the fraction of time the key is held.
//
// The ramp is LINEAR — a fixed counts/second, the same at 10% deflection as at
// 100%. An exponential lag was tried and is wrong for this: its rate is
// proportional to the current value, so it bleeds off fast at speed and crawls
// near zero, which reads as the control changing character with how fast you are
// already going.
//
// The two rates are ASYMMETRIC, and that asymmetry is the whole design. Ripple
// while tapping is just how far the value falls in the GAP between taps:
//
//     ripple ≈ 127 · t_gap / T_FALL
//
// T_RISE barely enters it. Games get away with near-symmetric rates because
// people spam at 3–5 Hz, where t_gap is ~200 ms; at a deliberate ~0.5 Hz tap the
// gap is ~1.8 s, so T_FALL has to be very long — note this is the opposite of the
// original tuning, which decayed FASTER than it rose (0.30 s vs 0.66 s) and so
// dumped the whole axis between taps.
//
// The second number that matters is the balance duty — the fraction of time held
// at which the level neither climbs nor sinks. It must match how you actually tap:
//
//     d* = T_RISE / (T_RISE + T_FALL)     → 1.5/10.5 ≈ 14%, i.e. 150 ms every 1 s
//
// Tap harder than d* and it walks up, softer and it sinks. Be honest about what
// this is: a linear ramp INTEGRATES, so it has no attracting mid-level — it only
// truly parks exactly at d*. What makes it usable is that the drift near d* is
// slow, so it behaves like a joystick you push rather than a level you select:
//
//     duty 10% (100 ms / 1 s)  →  −1.4 counts/s   (sinks, 90 s across range)
//     duty 14% (140 ms / 1 s)  →   0.0 counts/s   (holds)
//     duty 15% (150 ms / 1 s)  →  +0.6 counts/s   (200 s across range)
//     duty 25% (250 ms / 1 s)  →  +6.6 counts/s   (19 s across range)
//
// Ripple is independent of all that — it is just the gap decay, ~12 counts (9%)
// at a 0.85 s gap. Substituting T_FALL out gives the one relation worth knowing:
//
//     ripple = 127 · d* · T_tap_period / T_RISE
//
// so a FASTER tap rate is what buys a faster ramp: halving the tap period halves
// the ripple, which can then be spent on halving T_RISE. Tuning for 1 Hz instead
// of 0.5 Hz is exactly why T_RISE could drop from 3 s to 1.5 s here.
//
// For comparison the original tuning had d* = 69% and shed 783 counts over the
// same gap — it dumped the axis to zero between every tap, which is the ripple
// that started this.
//
// SAFETY: releasing everything is no longer how you stop — from full deflection a
// clean release takes T_FALL to reach zero. Stop with the OPPOSITE key, which
// ramps at the fast T_RISE rate (a non-zero target always uses T_RISE, including
// a reversal), or Enter → DISABLED, which is instant. The robot's own
// CTRL_WATCHDOG_MS also forces DISABLED 500 ms after the link drops.
//
// TUNING: T_FALL alone sets ripple — raise it for less. T_RISE sets how much one
// tap is worth. Both together set d*, so changing either shifts the tap rate that
// holds a constant level.
const T_RISE = 1.5;               // s, 0 → full deflection while held
const T_FALL = 9;                 // s, full deflection → 0 while released
const RATE_XY_UP   = 127 / T_RISE;  // counts/second
const RATE_XY_DOWN = 127 / T_FALL;  // counts/second

// Rates below are counts/SECOND. They used to be counts/frame with 60 fps assumed,
// which silently ran everything at 2× on a 120 Hz display — the reason "how slow"
// was not reproducible between machines.
const RATE_THETA    = (127 / 40) * 60; // heading trim: unchanged, ~0.66 s to full
const RATE_THROTTLE = 1.5 * 60;        // arrow up/down throttle integrator
const SEND_HZ       = 30;              // control packet rate, also display-independent

// Linear ramp toward a target at `rate` counts/second, clamped so it never overshoots.
function moveToward(current: number, target: number, rate: number, dt: number): number {
  const step = rate * dt;
  if (current < target) return Math.min(target, current + step);
  if (current > target) return Math.max(target, current - step);
  return current;
}

export function useKeyboard(mode: Mode, setMode: (m: Mode) => void): RefObject<InputState> {
  const target   = useRef<InputState>({ x: 0, y: 0, theta: 0, throttle: 0 });
  const filtered = useRef<InputState>({ x: 0, y: 0, theta: 0, throttle: 0 });
  const keys     = useRef(new Set<string>());

  useEffect(() => {
    // Key state goes STRAIGHT to the backend on every event — the ramps run in
    // a native Rust thread (spawn_control_loop) that the webview scheduler
    // cannot throttle (measured stalls up to 8.6 s froze the old in-webview
    // ramps mid-press). The local tick below is a display mirror only.
    const pushKeys = () => {
      const k = keys.current;
      invoke('set_key_targets', {
        x:     k.has('KeyA') ? -1 : k.has('KeyD') ? 1 : 0,
        y:     k.has('KeyW') ?  1 : k.has('KeyS') ? -1 : 0,
        theta: k.has('ArrowLeft') ? -1 : k.has('ArrowRight') ? 1 : 0,
        thr:   k.has('ArrowUp') ? 1 : k.has('ArrowDown') ? -1 : 0,
      });
    };
    const onDown = (e: KeyboardEvent) => {
      if (!e.repeat) { keys.current.add(e.code); pushKeys(); }
      if (e.code === 'ArrowUp' || e.code === 'ArrowDown' ||
          e.code === 'ArrowLeft' || e.code === 'ArrowRight') {
        e.preventDefault();
      }
      if (e.code === 'Enter') {
        setMode(0);
      }
    };
    const onUp = (e: KeyboardEvent) => { keys.current.delete(e.code); pushKeys(); };
    // A window that loses focus mid-hold never sees the keyup, so the key would
    // stay latched in the set and hold the target pinned at full deflection.
    const onBlur = () => { keys.current.clear(); pushKeys(); };
    window.addEventListener('keydown', onDown);
    window.addEventListener('keyup',   onUp);
    window.addEventListener('blur',    onBlur);

    // The control tick MUST NOT ride requestAnimationFrame: the webview can
    // throttle/starve rAF independently of visible UI smoothness (measured in
    // the 2026-08-07 commitW log: control updates at 57 ms instead of 33, with
    // multi-second stalls freezing the ramps mid-press — "keyboard feels
    // sluggish" while the UI looks fine). A fixed timer keeps control feel
    // independent of render scheduling; dt still comes from real elapsed time,
    // so the ramp tuning is unchanged.
    let lastMs   = 0;
    let sendAcc  = 0;

    const tick = (nowMs: number) => {
      // Real elapsed time, so the tuning above means the same thing on a 60 Hz and
      // a 120 Hz display. Clamped: a backgrounded window can hand back a multi-second
      // gap, which would otherwise step the filter straight to full deflection.
      const dt = lastMs === 0 ? 0 : Math.min(0.1, (nowMs - lastMs) / 1000);
      lastMs = nowMs;

      const k = keys.current;
      const t = target.current;

      t.x     = k.has('KeyA') ? -127 : k.has('KeyD') ?  127 : 0;
      t.y     = k.has('KeyW') ?  127 : k.has('KeyS') ? -127 : 0;
      t.theta = k.has('ArrowLeft') ? -127 : k.has('ArrowRight') ? 127 : 0;

      if (k.has('ArrowUp'))   t.throttle = Math.min(255, t.throttle + RATE_THROTTLE * dt);
      if (k.has('ArrowDown')) t.throttle = Math.max(0,   t.throttle - RATE_THROTTLE * dt);

      // Linear ramp. Each axis picks its rate from whether a key is DOWN, not from
      // which way the value is moving: a reversal is a press and gets the fast rate,
      // so braking with the opposite key stays quick.
      const rateFor = (target: number) => (target === 0 ? RATE_XY_DOWN : RATE_XY_UP);

      const f = filtered.current;
      f.x     = moveToward(f.x, t.x, rateFor(t.x), dt);
      f.y     = moveToward(f.y, t.y, rateFor(t.y), dt);
      f.theta = moveToward(f.theta, t.theta, RATE_THETA, dt);
      f.throttle = t.throttle;

      // No deadband snap needed on X/Y: a linear ramp clamps exactly onto its
      // target, unlike the exponential tail this replaced.
      if (Math.abs(f.theta) < 0.4) f.theta = 0;

      // Clamp the filtered XY output to the unit circle so diagonals (W+D etc.)
      // always have the same magnitude as cardinal directions.  Doing this on the
      // filtered values (not the target) correctly handles direction transitions —
      // normalising the target instead leaves the filter overshooting during changes.
      const fmag = Math.sqrt(f.x * f.x + f.y * f.y);
      if (fmag > 127) {
        f.x = (f.x / fmag) * 127;
        f.y = (f.y / fmag) * 127;
      }

      // DISABLED is the panic stop, so it must clear the filter state outright.
      // The slow TAU_RELEASE means a coasting X/Y would otherwise still be parked
      // at half deflection a few seconds later, and re-arming would hand the robot
      // that stale translation on the very first packet.
      if (mode === 0) {
        t.throttle = 0;
        f.throttle = 0;
        f.x = 0; f.y = 0; f.theta = 0;
      }

      // No sends from the webview: the backend ramp thread owns transmission.
      // This tick only mirrors the ramps for the on-screen control display.
      sendAcc += dt;
      if (sendAcc >= 1 / SEND_HZ) sendAcc = 0;

    };

    const timerId = window.setInterval(() => tick(performance.now()), 8);
    return () => {
      window.removeEventListener('keydown', onDown);
      window.removeEventListener('keyup',   onUp);
      window.removeEventListener('blur',    onBlur);
      window.clearInterval(timerId);
    };
  }, [mode]);

  return filtered;
}
