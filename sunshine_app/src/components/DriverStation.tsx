import { useState, useEffect, type RefObject } from 'react';
import { invoke } from '@tauri-apps/api/core';
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

  useEffect(() => {
    let raf: number;
    const tick = () => {
      const { x, y, theta, throttle } = inputRef.current;
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

  const thetaBarH = Math.abs(nt) * 50;
  const thetaBase = nt >= 0 ? 50 : 50 - thetaBarH;

  return (
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

      {/* Theta */}
      <div className="vbar">
        <span className="vbar-label">θ</span>
        <div className="vbar-track">
          <div
            className="vbar-fill theta-fill"
            style={{ height: `${thetaBarH}%`, bottom: `${thetaBase}%` }}
          />
        </div>
        <span className="vbar-val mono">{nt >= 0 ? '+' : ''}{nt.toFixed(2)}</span>
      </div>
    </div>
  );
}

interface Props {
  mode:         Mode;
  setMode:      (m: Mode) => void;
  sourceStatus: SourceStatus;
  liveUpdate:   LiveUpdate | null;
  inputRef:     RefObject<InputState>;
}

export default function DriverStation({ mode, setMode, sourceStatus, liveUpdate, inputRef }: Props) {
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
            <select className="conn-select" value={port} onChange={e => setPort(e.target.value)}>
              <option value="">Select port…</option>
              {ports.map(p => <option key={p} value={p}>{p}</option>)}
            </select>
            <button className="conn-btn" onClick={() => invoke('connect_serial', { port })} disabled={!port}>
              Connect
            </button>
            <div className={`conn-status ${isLive ? 'connected' : 'disconnected'}`}>
              {sourceStatus.detail || sourceStatus.kind}
            </div>
          </>
        )}

        {tab === 'replay' && (
          <>
            <input
              className="conn-input"
              placeholder="Path to .sun file…"
              value={replayPath}
              onChange={e => setReplayPath(e.target.value)}
            />
            <button className="conn-btn" onClick={async () => {
              if (!replayPath) return;
              const meta = await invoke<{ label?: string; frame_count: number; schema_version: number }>('open_replay', { path: replayPath });
              setReplayMeta(meta);
            }}>
              Open
            </button>
            {replayMeta && (
              <div className="replay-info">
                <span>Label: {replayMeta.label || '(none)'}</span>
                <span>Frames: {replayMeta.frame_count}</span>
                <span>Schema v{replayMeta.schema_version}</span>
              </div>
            )}
          </>
        )}

        {tab === 'sim' && (
          <>
            <button className="conn-btn" onClick={() => invoke('start_simulation')}>Start Simulation</button>
            <button className="conn-btn danger" onClick={() => invoke('stop_source')}>Stop</button>
            <div className="sim-info">
              <span>KV: 1100 RPM/V</span>
              <span>MoI: 1.214×10⁻³ kg·m²</span>
              <span>Battery: 2S 8.4V</span>
            </div>
          </>
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
