import { useState } from 'react';
import type { LiveUpdate, Mode } from '../types/sunshine';

const MODE_LABELS = ['DISABLED', 'TANK', 'MELTY'] as const;

interface Props {
  mode:             Mode;
  liveUpdate:       LiveUpdate | null;
  rxRssi:           number;
  loggingActive:    boolean;
  logPath:          string;
  isGraphLive:      boolean;
  onGoLive:         () => void;
  onEnableLogging:  (label: string) => void;
  onDisableLogging: () => void;
}

function battVoltage(offset: number): number {
  return 7.6 + offset * 0.0205;
}

export default function TopBar({ mode, liveUpdate, rxRssi, loggingActive, logPath, isGraphLive, onGoLive, onEnableLogging, onDisableLogging }: Props) {
  const [label, setLabel] = useState('');

  const batt   = liveUpdate ? battVoltage(liveUpdate.batt_offset).toFixed(2) : '--';
  const omega   = liveUpdate ? (liveUpdate.est_omega * 60 / (2 * Math.PI)).toFixed(0) : '--';
  const rssiB   = liveUpdate?.rssi ?? '--';

  return (
    <header className="topbar">
      <div className="brand">
        <div className="brand-dot" />
        <span className="brand-name">SUNSHINE</span>
        <span className="brand-sub">/ telemetry</span>
      </div>

      <div className={`mode-pill m${mode}`}>
        <span className="led" />
        <span>{MODE_LABELS[mode]}</span>
      </div>

      <div className="top-meta">
        <span className="top-stat">
          <span className="k">BATT</span>
          <span className="v">{batt} V</span>
        </span>
        <span className="top-stat">
          <span className="k">SPIN</span>
          <span className="v">{omega} RPM</span>
        </span>
        <span className="top-stat">
          <span className="k">RSSI·BOT</span>
          <span className="v">{rssiB} dBm</span>
        </span>
        <span className="top-stat">
          <span className="k">RSSI·RX</span>
          <span className="v">{rxRssi} dBm</span>
        </span>
      </div>

      <button
        className={`live-btn ${isGraphLive ? 'live-btn-live' : 'live-btn-paused'}`}
        onClick={onGoLive}
        disabled={isGraphLive}
      >
        <span className="live-btn-dot" />
        {isGraphLive ? 'LIVE' : 'PAUSED'}
      </button>

      <div className="top-spacer" />

      <div className="logging-bar">
        {loggingActive ? (
          <>
            <span className="log-rec">
              <span className="log-rec-dot" />
              REC
            </span>
            <span className="log-path" title={logPath}>{logPath.split('/').pop()}</span>
            <button className="log-btn stop" onClick={onDisableLogging}>Stop</button>
          </>
        ) : (
          <>
            <input
              className="log-input"
              placeholder="label (optional)"
              value={label}
              onChange={e => setLabel(e.target.value)}
              onKeyDown={e => { if (e.key === 'Enter') { onEnableLogging(label); setLabel(''); } }}
            />
            <button className="log-btn start" onClick={() => { onEnableLogging(label); setLabel(''); }}>Log</button>
          </>
        )}
      </div>
    </header>
  );
}
