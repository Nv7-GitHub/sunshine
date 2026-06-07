import { useState, useEffect } from 'react';
import { invoke } from '@tauri-apps/api/core';
import type { SourceStatus } from '../types/sunshine';

export default function ConnectionPanel({ sourceStatus }: { sourceStatus: SourceStatus }) {
  const [tab, setTab]               = useState<'live' | 'replay' | 'sim'>('live');
  const [ports, setPorts]           = useState<string[]>([]);
  const [port, setPort]             = useState('');
  const [replayMeta, setReplayMeta] = useState<{ label?: string; frame_count: number; schema_version: number } | null>(null);
  const [replayPath, setReplayPath] = useState('');
  const isLive = sourceStatus.kind === 'Live';

  useEffect(() => {
    invoke<string[]>('list_serial_ports').then(setPorts).catch(() => setPorts([]));
  }, [tab]);

  return (
    <div className="connection-panel">
      <div className="tab-bar">
        {(['live','replay','sim'] as const).map(t => (
          <button key={t} className={`tab ${tab===t?'active':''}`} onClick={() => setTab(t)}>
            {t.toUpperCase()}
          </button>
        ))}
      </div>

      {tab === 'live' && (
        <div className="tab-content">
          <select value={port} onChange={e => setPort(e.target.value)} className="port-select" disabled={isLive}>
            <option value="">Select port…</option>
            {ports.map(p => <option key={p} value={p}>{p}</option>)}
          </select>
          {isLive ? (
            <button onClick={() => invoke('disconnect_serial')} className="btn-stop">
              Disconnect
            </button>
          ) : (
            <button onClick={() => invoke('connect_serial', { port })} disabled={!port} className="btn-connect">
              Connect
            </button>
          )}
          <div className={`conn-status ${isLive ? 'connected' : 'disconnected'}`}>
            {isLive ? '● ' : '○ '}{sourceStatus.detail || (isLive ? 'Connected' : 'Disconnected')}
          </div>
        </div>
      )}

      {tab === 'replay' && (
        <div className="tab-content">
          <input
            className="log-label-input"
            placeholder="Path to .sun file"
            value={replayPath}
            onChange={e => setReplayPath(e.target.value)}
          />
          <button className="btn-connect" onClick={async () => {
            if (!replayPath) return;
            const meta = await invoke<{ label?: string; frame_count: number; schema_version: number }>('open_replay', { path: replayPath });
            setReplayMeta(meta);
          }}>Open</button>
          {replayMeta && (
            <div className="replay-meta">
              <div>Label: {replayMeta.label || '(none)'}</div>
              <div>Frames: {replayMeta.frame_count}</div>
              <div>Schema v{replayMeta.schema_version}</div>
            </div>
          )}
        </div>
      )}

      {tab === 'sim' && (
        <div className="tab-content">
          <button className="btn-connect" onClick={() => invoke('start_simulation')}>Start Simulation</button>
          <button className="btn-stop"    onClick={() => invoke('stop_source')}>Stop</button>
          <div className="sim-params">
            <div>KV: 1100 RPM/V</div>
            <div>MoI: 1.214×10⁻³ kg·m²</div>
            <div>Battery: 2S 8.4V</div>
          </div>
        </div>
      )}
    </div>
  );
}
