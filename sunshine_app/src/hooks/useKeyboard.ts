import { useEffect, useRef } from 'react';
import { invoke } from '@tauri-apps/api/core';

const ALPHA = 0.03;

export function useKeyboard(mode: number) {
  const target   = useRef({ x: 0, y: 0, theta: 0, throttle: 0 });
  const filtered = useRef({ x: 0, y: 0, theta: 0, throttle: 0 });
  const keys     = useRef(new Set<string>());
  const rafRef   = useRef<number | undefined>(undefined);

  useEffect(() => {
    const onKeyDown = (e: KeyboardEvent) => { if (!e.repeat) keys.current.add(e.key); };
    const onKeyUp   = (e: KeyboardEvent) => keys.current.delete(e.key);
    window.addEventListener('keydown', onKeyDown);
    window.addEventListener('keyup',   onKeyUp);

    let frame = 0;
    const tick = () => {
      const t = target.current;
      t.x     = keys.current.has('a') ? -127 : keys.current.has('d') ?  127 : 0;
      t.y     = keys.current.has('w') ?  127 : keys.current.has('s') ? -127 : 0;
      t.theta = keys.current.has('ArrowLeft') ? -127 : keys.current.has('ArrowRight') ? 127 : 0;
      if (keys.current.has('ArrowUp'))   t.throttle = Math.min(255, t.throttle + 2);
      if (keys.current.has('ArrowDown')) t.throttle = Math.max(0,   t.throttle - 2);

      const f = filtered.current;
      f.x        += ALPHA * (t.x     - f.x);
      f.y        += ALPHA * (t.y     - f.y);
      f.theta    += ALPHA * (t.theta - f.theta);
      f.throttle  = t.throttle;

      if (frame++ % 2 === 0 && mode !== 0) {
        invoke('set_controls', {
          x:        Math.round(f.x),
          y:        Math.round(f.y),
          theta:    Math.round(f.theta),
          throttle: Math.round(f.throttle),
        });
      }
      rafRef.current = requestAnimationFrame(tick);
    };
    rafRef.current = requestAnimationFrame(tick);

    return () => {
      window.removeEventListener('keydown', onKeyDown);
      window.removeEventListener('keyup',   onKeyUp);
      if (rafRef.current) cancelAnimationFrame(rafRef.current);
    };
  }, [mode]);
}
