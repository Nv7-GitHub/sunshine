import React from 'react';
import type { Mode } from '../types/sunshine';

const MODES: { label: string; value: Mode; color: string }[] = [
  { label: 'DISABLED', value: 0, color: '#e74c3c' },
  { label: 'TANK',     value: 1, color: '#f39c12' },
  { label: 'MELTY',    value: 2, color: '#2ecc71' },
];

export default function ModeButtons({ mode, setMode }: { mode: Mode; setMode: (m: Mode) => void }) {
  return (
    <div className="mode-buttons">
      {MODES.map(m => (
        <button
          key={m.value}
          className={`mode-btn ${mode === m.value ? 'active' : ''}`}
          style={{ '--btn-color': m.color } as React.CSSProperties}
          onClick={() => setMode(m.value)}
        >
          {m.label}
        </button>
      ))}
    </div>
  );
}
