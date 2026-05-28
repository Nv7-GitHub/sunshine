import { useRef, useState, useEffect } from 'react';
import { CHANNELS } from '../types/sunshine';
import UPlotCanvas from './UPlotCanvas';

const SERIES_COLORS = [
  'oklch(.82 .14 168)',
  'oklch(.82 .14 82)',
  'oklch(.78 .12 240)',
  'oklch(.78 .14 300)',
  'oklch(.78 .14 20)',
  'oklch(.85 .10 200)',
];

type ChanEntry = { key: string; label: string; unit: string };
const ALL_CHANNELS = Object.entries(CHANNELS).flatMap(
  ([group, chans]) => (chans as readonly ChanEntry[]).map(ch => ({ ...ch, group }))
);

interface Props {
  selected:     string[];
  onToggle:     (key: string) => void;
  headTimeUs:   number;
  requestLive:  number;
  onCursorMove: (us: number | null) => void;
  onLiveChange: (live: boolean) => void;
}

export default function GraphPanel({ selected, onToggle, headTimeUs, requestLive, onCursorMove, onLiveChange }: Props) {
  const wrapRef = useRef<HTMLDivElement>(null);
  const [size, setSize] = useState({ w: 600, h: 400 });

  useEffect(() => {
    const ro = new ResizeObserver(entries => {
      const r = entries[0].contentRect;
      setSize({ w: Math.max(100, Math.floor(r.width)), h: Math.max(80, Math.floor(r.height)) });
    });
    if (wrapRef.current) ro.observe(wrapRef.current);
    return () => ro.disconnect();
  }, []);

  return (
    <div className="center-panel">
      <div className="graph-head">
        <div className="graph-title">
          <b>Live plot</b>
          <span style={{ color: 'var(--text-3)' }}>{selected.length} series</span>
        </div>
        <div className="graph-tools">
          <span style={{ fontSize: 10, color: 'var(--text-4)', letterSpacing: '.06em', marginRight: 2 }}>SCROLL TO ZOOM · CTRL+SCROLL TO PAN</span>
        </div>
      </div>

      <div ref={wrapRef} className="uplot-wrap">
        <UPlotCanvas
          channels={selected}
          width={size.w}
          height={size.h}
          headTimeUs={headTimeUs}
          requestLive={requestLive}
          onCursorMove={onCursorMove}
          onLiveChange={onLiveChange}
        />
      </div>

      <div className="chips-strip">
        {selected.length === 0 && (
          <span className="chip chip-empty">Select variables on the left to plot them →</span>
        )}
        {selected.map((key, i) => {
          const ch = ALL_CHANNELS.find(c => c.key === key);
          if (!ch) return null;
          return (
            <span key={key} className="chip">
              <span className="chip-dot" style={{ background: SERIES_COLORS[i % SERIES_COLORS.length] }} />
              <span className="chip-group">{ch.group}/</span>
              <span>{ch.label}</span>
              {ch.unit && <span className="var-unit" style={{ marginLeft: 2, color: 'var(--text-4)', fontSize: 10 }}>{ch.unit}</span>}
              <button className="chip-x" onClick={() => onToggle(key)}>×</button>
            </span>
          );
        })}
      </div>
    </div>
  );
}
