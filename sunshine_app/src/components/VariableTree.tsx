import { useState, useEffect, useRef } from 'react';
import { invoke } from '@tauri-apps/api/core';
import { CHANNELS } from '../types/sunshine';

const SERIES_COLORS = [
  'oklch(.82 .14 168)',
  'oklch(.82 .14 82)',
  'oklch(.78 .12 240)',
  'oklch(.78 .14 300)',
  'oklch(.78 .14 20)',
  'oklch(.85 .10 200)',
];

type ChanEntry = { key: string; label: string; unit: string };
const TOTAL = Object.values(CHANNELS).reduce((n, g) => n + g.length, 0);
const ALL_KEYS: string[] = Object.values(CHANNELS).flatMap(g =>
  (g as readonly ChanEntry[]).map(ch => ch.key)
);

interface Props {
  selected:  string[];
  onToggle:  (key: string) => void;
  cursorUs:  number | null;
}

export default function VariableTree({ selected, onToggle, cursorUs }: Props) {
  const [query, setQuery]       = useState('');
  const [collapsed, setCollapsed] = useState<Record<string, boolean>>({});
  const [values, setValues]     = useState<Map<string, number | null>>(new Map());
  const cursorUsRef             = useRef(cursorUs);
  cursorUsRef.current           = cursorUs;

  const fetchSnapshot = async (timeUs: number | null) => {
    try {
      const result = await invoke<(number | null)[]>('get_channel_snapshot', {
        channels: ALL_KEYS,
        timeUs,
      });
      setValues(new Map(ALL_KEYS.map((k, i) => [k, result[i]])));
    } catch { /* backend not ready */ }
  };

  // 10 Hz poll when cursor is not active
  useEffect(() => {
    const id = setInterval(() => {
      if (cursorUsRef.current === null) fetchSnapshot(null);
    }, 100);
    return () => clearInterval(id);
  }, []);

  // Immediate fetch whenever cursor changes
  useEffect(() => {
    fetchSnapshot(cursorUs);
  }, [cursorUs]);

  const q      = query.toLowerCase();
  const toggle = (g: string) => setCollapsed(c => ({ ...c, [g]: !c[g] }));

  const fmt = (v: number | null | undefined): string => {
    if (v == null || isNaN(v)) return '—';
    if (Math.abs(v) >= 1000)  return v.toFixed(0);
    if (Math.abs(v) >= 10)    return v.toFixed(2);
    return v.toFixed(4);
  };

  return (
    <div className="left-rail">
      <div className="rail-head">
        <span className="rail-title">Channels</span>
        <span className="rail-count mono">{selected.length}/{TOTAL}</span>
      </div>

      <div className="var-search">
        <input
          placeholder="Search…"
          value={query}
          onChange={e => setQuery(e.target.value)}
        />
      </div>

      <div className="var-tree">
        {(Object.entries(CHANNELS) as [string, readonly ChanEntry[]][]).map(([group, chans]) => {
          const filtered = chans.filter(ch =>
            !q || ch.label.toLowerCase().includes(q) || group.toLowerCase().includes(q)
          );
          if (!filtered.length) return null;
          const isCollapsed = collapsed[group] && !q;

          return (
            <div key={group} className="var-group">
              <div
                className={`var-group-header ${isCollapsed ? 'collapsed' : ''}`}
                onClick={() => toggle(group)}
              >
                <svg className="caret" viewBox="0 0 9 9" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round">
                  <path d="M2 3 L4.5 6 L7 3" />
                </svg>
                <span>{group}</span>
                <span className="var-group-count">{filtered.length}</span>
              </div>

              {!isCollapsed && filtered.map(ch => {
                const isSel  = selected.includes(ch.key);
                const selIdx = selected.indexOf(ch.key);
                const color  = isSel ? SERIES_COLORS[selIdx % SERIES_COLORS.length] : undefined;
                const val    = values.get(ch.key);
                return (
                  <div
                    key={ch.key}
                    className={`var-row ${isSel ? 'selected' : ''}`}
                    onClick={() => onToggle(ch.key)}
                  >
                    <span className="var-swatch" style={color ? { background: color } : {}} />
                    <span className="var-name">{ch.label}</span>
                    <span className="var-val">
                      {fmt(val)}{ch.unit && <span className="var-unit">{ch.unit}</span>}
                    </span>
                  </div>
                );
              })}
            </div>
          );
        })}
      </div>
    </div>
  );
}
