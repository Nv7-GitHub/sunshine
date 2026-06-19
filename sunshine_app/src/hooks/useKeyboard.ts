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

// Linear ramp rates (units/frame) — derived from old exponential τ (83 and 50 frames).
// Key held → ramps up to ±127 at this rate; released → decays back to 0 at same rate.
const RATE_XY    = 127 / 83;   // ≈ 1.53 / frame
const RATE_THETA = 127 / 50;   // ≈ 2.54 / frame

function moveToward(current: number, target: number, rate: number): number {
  if (current < target) return Math.min(target, current + rate);
  if (current > target) return Math.max(target, current - rate);
  return current;
}

export function useKeyboard(mode: Mode, setMode: (m: Mode) => void): RefObject<InputState> {
  const target   = useRef<InputState>({ x: 0, y: 0, theta: 0, throttle: 0 });
  const filtered = useRef<InputState>({ x: 0, y: 0, theta: 0, throttle: 0 });
  const keys     = useRef(new Set<string>());

  useEffect(() => {
    const onDown = (e: KeyboardEvent) => {
      if (!e.repeat) keys.current.add(e.code);
      if (e.code === 'ArrowUp' || e.code === 'ArrowDown' ||
          e.code === 'ArrowLeft' || e.code === 'ArrowRight') {
        e.preventDefault();
      }
      if (e.code === 'Enter') {
        setMode(0);
      }
    };
    const onUp = (e: KeyboardEvent) => keys.current.delete(e.code);
    window.addEventListener('keydown', onDown);
    window.addEventListener('keyup',   onUp);

    let frame = 0;
    let rafId: number;

    const tick = () => {
      const k = keys.current;
      const t = target.current;

      t.x     = k.has('KeyA') ? -127 : k.has('KeyD') ?  127 : 0;
      t.y     = k.has('KeyW') ?  127 : k.has('KeyS') ? -127 : 0;
      t.theta = k.has('ArrowLeft') ? -127 : k.has('ArrowRight') ? 127 : 0;

      if (k.has('ArrowUp'))   t.throttle = Math.min(255, t.throttle + 1.5);
      if (k.has('ArrowDown')) t.throttle = Math.max(0,   t.throttle - 1.5);

      const f = filtered.current;
      f.x     = moveToward(f.x,     t.x,     RATE_XY);
      f.y     = moveToward(f.y,     t.y,     RATE_XY);
      f.theta = moveToward(f.theta, t.theta, RATE_THETA);
      f.throttle = t.throttle;

      if (Math.abs(f.x)     < 0.4) f.x     = 0;
      if (Math.abs(f.y)     < 0.4) f.y     = 0;
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

      if (mode === 0) {
        t.throttle = 0;
        f.throttle = 0;
      }

      if (frame++ % 2 === 0 && mode !== 0) {
        invoke('set_controls', {
          x:        Math.round(f.x),
          y:        Math.round(f.y),
          theta:    Math.round(f.theta),
          throttle: Math.round(f.throttle),
        });
      }

      rafId = requestAnimationFrame(tick);
    };

    rafId = requestAnimationFrame(tick);
    return () => {
      window.removeEventListener('keydown', onDown);
      window.removeEventListener('keyup',   onUp);
      cancelAnimationFrame(rafId);
    };
  }, [mode]);

  return filtered;
}
