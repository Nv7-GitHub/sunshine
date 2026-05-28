import { useState } from 'react';
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

interface Props {
  selected: string[];
  onToggle: (key: string) => void;
}

export default function VariableTree({ selected, onToggle }: Props) {
  const [query, setQuery] = useState('');
  const [collapsed, setCollapsed] = useState<Record<string, boolean>>({});

  const q = query.toLowerCase();
  const toggle = (g: string) => setCollapsed(c => ({ ...c, [g]: !c[g] }));

  return (
    <div className="left-rail">
      <div className="rail-head">
        <span className="rail-title">Variables</span>
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
                return (
                  <div
                    key={ch.key}
                    className={`var-row ${isSel ? 'selected' : ''}`}
                    onClick={() => onToggle(ch.key)}
                  >
                    <span className="var-swatch" style={color ? { background: color } : {}} />
                    <span className="var-name">{ch.label}</span>
                    <span className="var-val">
                      —{ch.unit && <span className="var-unit">{ch.unit}</span>}
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
