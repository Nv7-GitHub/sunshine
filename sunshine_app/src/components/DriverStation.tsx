import { useState, useEffect, useRef, type RefObject } from 'react';
import { invoke } from '@tauri-apps/api/core';
import { open as openDialog } from '@tauri-apps/plugin-dialog';
import type { Mode, SourceStatus, LiveUpdate } from '../types/sunshine';
import type { InputState } from '../hooks/useKeyboard';

const MODE_LABELS = ['DISABLED', 'TANK', 'MELTY'] as const;

function battVoltage(offset: number): number {
  return 7.6 + offset * 0.0205;
}
function battClass(v: number): string {
  if (v >= 7.8) return 'batt-good';
  if (v >= 7.2) return 'batt-warn';
  return 'batt-bad';
}
function battPct(v: number): number {
  return Math.max(0, Math.min(100, (v - 6.0) / 2.4 * 100));
}
function battLabel(v: number): string {
  if (v >= 7.8) return 'Nominal';
  if (v >= 7.2) return 'Watch';
  return 'Low';
}

/* ── Controls visualization ── */
function ControlsViz({ inputRef }: { inputRef: RefObject<InputState> }) {
  const [st, setSt] = useState<InputState>({ x: 0, y: 0, theta: 0, throttle: 0 });
  const trailRef    = useRef<{ x: number; y: number }[]>([]);

  useEffect(() => {
    let raf: number;
    const tick = () => {
      const { x, y, theta, throttle } = inputRef.current;
      const nx = x / 127;
      const ny = y / 127;
      const jx = 50 + nx * 40;
      const jy = 50 - ny * 40;
      const trail = trailRef.current;
      trail.push({ x: jx, y: jy });
      if (trail.length > 30) trail.shift();
      setSt({ x, y, theta, throttle });
      raf = requestAnimationFrame(tick);
    };
    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
  }, [inputRef]);

  const nx = st.x / 127;
  const ny = st.y / 127;
  const nt = st.theta / 127;
  const throttlePct = (st.throttle / 255) * 100;

  const jx = 50 + nx * 40;
  const jy = 50 - ny * 40;

  const thetaFillPct  = Math.abs(nt) * 50;
  const thetaFillSide = nt >= 0 ? 'right' : 'left';

  return (
    <>
      <div className="controls-grid">
        {/* X/Y joystick */}
        <div className="joy-pad">
          <svg className="joy-grid" viewBox="0 0 100 100" preserveAspectRatio="none">
            <line className="axis" x1="50" y1="5" x2="50" y2="95" />
            <line className="axis" x1="5" y1="50" x2="95" y2="50" />
            <line x1="25" y1="5" x2="25" y2="95" />
            <line x1="75" y1="5" x2="75" y2="95" />
            <line x1="5" y1="25" x2="95" y2="25" />
            <line x1="5" y1="75" x2="95" y2="75" />
            {trailRef.current.map((pt, i) => {
              const count   = trailRef.current.length;
              const age     = count - 1 - i;
              const opacity = 0.5 - age * (0.45 / 29);
              const radius  = 4 - age * (2 / 29);
              return (
                <circle key={i} cx={pt.x} cy={pt.y} r={Math.max(2, radius)} fill="var(--accent)" opacity={Math.max(0.05, opacity)} />
              );
            })}
          </svg>
          <div className="joy-ring" />
          <div className="joy-dot" style={{ left: `${jx}%`, top: `${jy}%` }} />
          <div className="joy-labels">
            <span className="jt">+Y</span>
            <span className="jb">−Y</span>
            <span className="jl">−X</span>
            <span className="jr">+X</span>
          </div>
          <div className="joy-readout mono">
            x<span className="v"> {nx.toFixed(2)}</span>{' '}y<span className="v"> {ny.toFixed(2)}</span>
          </div>
        </div>

        {/* Throttle */}
        <div className="vbar">
          <span className="vbar-label">THR</span>
          <div className="vbar-track">
            <div className="vbar-fill" style={{ height: `${throttlePct}%` }} />
          </div>
          <span className="vbar-val mono">{Math.round(throttlePct)}<span style={{ color: 'var(--text-4)', fontSize: '8px' }}>%</span></span>
        </div>
      </div>

      {/* Theta horizontal slider */}
      <div className="hslider">
        <span className="hslider-label">θ</span>
        <div className="hslider-track">
          <div className="hslider-center-dot" />
          <div
            className={`hslider-fill hslider-fill-${thetaFillSide}`}
            style={{ width: `${thetaFillPct}%` }}
          />
        </div>
        <span className="hslider-val mono">{nt >= 0 ? '+' : ''}{nt.toFixed(2)}</span>
      </div>
    </>
  );
}

interface Props {
  mode:           Mode;
  setMode:        (m: Mode) => void;
  sourceStatus:   SourceStatus;
  liveUpdate:     LiveUpdate | null;
  inputRef:       RefObject<InputState>;
  loadReplay:     (path: string) => Promise<void>;
  stopSource:     () => void;
}

export default function DriverStation({ mode, setMode, sourceStatus, liveUpdate, inputRef, loadReplay, stopSource }: Props) {
  const [tab, setTab]               = useState<'live' | 'replay' | 'sim'>('live');
  const [ports, setPorts]           = useState<string[]>([]);
  const [port, setPort]             = useState('');
  const [replayPath, setReplayPath] = useState('');
  const [replayMeta, setReplayMeta] = useState<{ label?: string; frame_count: number; schema_version: number } | null>(null);

  useEffect(() => {
    invoke<string[]>('list_serial_ports').then(setPorts).catch(() => setPorts([]));
  }, [tab]);

  const isLive       = sourceStatus.kind === 'Live';
  const isConnected  = sourceStatus.kind !== 'Disconnected';

  const batt  = liveUpdate ? battVoltage(liveUpdate.batt_offset) : null;
  const bClass = batt !== null ? battClass(batt) : 'batt-good';

  return (
    <div className="right-rail">
      <div className="right-head">
        <span className="rail-title">Driver Station</span>
        <span className="rail-count mono">{sourceStatus.kind}</span>
      </div>

      {/* ── Mode selection ── */}
      <div className="ds-section">
        <div className="ds-head"><span>Mode</span></div>
        <div className="mode-grid">
          {([0, 1, 2] as Mode[]).map(m => (
            <button
              key={m}
              className={`mode-card ${mode === m ? `mc-active-${m}` : ''}`}
              onClick={() => setMode(m)}
            >
              <span className="mc-label">{MODE_LABELS[m]}</span>
            </button>
          ))}
        </div>
      </div>

      {/* ── Battery ── */}
      <div className="ds-section">
        <div className="ds-head">
          <span>Battery</span>
          <span className="ds-head-sub">{batt !== null ? battLabel(batt) : '--'}</span>
        </div>
        <div className={`batt-display ${bClass}`}>
          <div>
            <div className="batt-num mono">
              {batt !== null ? batt.toFixed(2) : '--'}<span className="u">V</span>
            </div>
          </div>
          <div style={{ flex: 1 }}>
            <div className="batt-bar-wrap">
              <div className="batt-bar-fill" style={{ width: batt !== null ? `${battPct(batt)}%` : '0%' }} />
            </div>
          </div>
        </div>
      </div>

      {/* ── Connection ── */}
      <div className="ds-section">
        <div className="ds-head">
          <span>Connection</span>
          {isConnected && (
            <span style={{ width: 6, height: 6, borderRadius: '50%', background: 'var(--good)', boxShadow: '0 0 8px var(--good)', display: 'inline-block', animation: 'pulse 1.6s infinite' }} />
          )}
        </div>
        <div className="tab-row">
          {(['live', 'replay', 'sim'] as const).map(t => (
            <button key={t} className={`tab-btn ${tab === t ? 'active' : ''}`} onClick={() => setTab(t)}>
              {t}
            </button>
          ))}
        </div>

        {tab === 'live' && (
          <>
            <select className="conn-select" value={port} onChange={e => setPort(e.target.value)} disabled={isLive}>
              <option value="">Select port…</option>
              {ports.map(p => <option key={p} value={p}>{p}</option>)}
            </select>
            {isLive ? (
              <button className="conn-btn danger" onClick={() => invoke('disconnect_serial')}>
                Disconnect
              </button>
            ) : (
              <button className="conn-btn" onClick={() => invoke('connect_serial', { port })} disabled={!port}>
                Connect
              </button>
            )}
            <div className={`conn-status ${isLive ? 'connected' : 'disconnected'}`}>
              {isLive ? '● ' : '○ '}{isLive ? (sourceStatus.detail || 'Connected') : 'Disconnected'}
            </div>
          </>
        )}

        {tab === 'replay' && (
          <div className="tab-body">
            <button
              className={`file-btn${replayPath ? ' has-file' : ''}`}
              onClick={async () => {
                let selected: string | null = null;
                try {
                  selected = await openDialog({ filters: [{ name: 'Sunshine Log', extensions: ['sun'] }] }) as string | null;
                } catch { return; }
                if (!selected) return;
                setReplayPath(selected);
                try {
                  const meta = await invoke<{ label?: string; frame_count: number; schema_version: number }>(
                    'open_replay', { path: selected }
                  );
                  setReplayMeta(meta);
                } catch {
                  setReplayMeta(null);
                }
              }}
            >
              <svg viewBox="0 0 16 16"><path d="M2 4.5A1.5 1.5 0 013.5 3h3.086a1.5 1.5 0 011.06.44L8.5 4.293A1.5 1.5 0 009.56 4.5H12.5A1.5 1.5 0 0114 6v5.5A1.5 1.5 0 0112.5 13h-9A1.5 1.5 0 012 11.5V4.5z" stroke="currentColor" stroke-width="1.2" fill="none" stroke-linejoin="round"/></svg>
              <span>{replayPath ? replayPath.split('/').pop() ?? replayPath : 'Select .sun file…'}</span>
            </button>

            {replayMeta && (
              <>
                <div className="meta-rows">
                  {replayMeta.label ? (
                    <div className="meta-row"><span>Label</span><span>{replayMeta.label}</span></div>
                  ) : null}
                  <div className="meta-row">
                    <span>Frames</span>
                    <span>{replayMeta.frame_count.toLocaleString()}</span>
                  </div>
                  <div className="meta-row">
                    <span>Schema</span>
                    <span>v{replayMeta.schema_version}</span>
                  </div>
                </div>
                {sourceStatus.kind === 'Replay' ? (
                  <button className="source-btn source-btn-stop" onClick={stopSource}>Close Replay</button>
                ) : (
                  <button className="source-btn source-btn-start" onClick={async () => {
                    try { await loadReplay(replayPath); } catch { /* ignore */ }
                  }}>Load</button>
                )}
              </>
            )}
          </div>
        )}

        {tab === 'sim' && (
          <div className="tab-body">
            {sourceStatus.kind === 'Sim' ? (
              <button className="source-btn source-btn-stop" onClick={stopSource}>Stop</button>
            ) : (
              <button className="source-btn source-btn-start" onClick={() => { invoke('start_simulation'); setMode(1); }}>
                Start Simulation
              </button>
            )}
            <div className="meta-rows">
              <div className="meta-row"><span>KV</span><span>1100 RPM/V</span></div>
              <div className="meta-row"><span>MoI</span><span>1.214×10⁻³ kg·m²</span></div>
              <div className="meta-row"><span>Battery</span><span>2S · 8.4 V</span></div>
            </div>
          </div>
        )}
      </div>

      {/* ── Controls visualization ── */}
      <div className="ds-section">
        <div className="ds-head">
          <span>Controls</span>
          <span className="ds-head-sub">WASD · arrows</span>
        </div>
        <ControlsViz inputRef={inputRef} />
      </div>
    </div>
  );
}
