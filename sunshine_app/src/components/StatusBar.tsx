import type { LiveUpdate } from '../types/sunshine';

function battColor(offset: number): string {
  const v = 7.6 + offset * 0.0205;
  if (v >= 8.0) return '#2ecc71';
  if (v >= 7.4) return '#f39c12';
  if (v >= 7.0) return '#e67e22';
  return '#e74c3c';
}

export default function StatusBar({ update, rxRssi }: { update: LiveUpdate | null; rxRssi: number }) {
  const v         = update ? (7.6 + update.batt_offset * 0.0205).toFixed(2) : '--';
  const omega_rpm = update ? (update.est_omega * 60 / (2 * Math.PI)).toFixed(0) : '--';
  const color     = update ? battColor(update.batt_offset) : '#666';

  return (
    <div className="status-bar">
      <span className="status-item">
        <span className="status-label">BATT</span>
        <span className="status-value" style={{ color }}>{v} V</span>
      </span>
      <span className="status-item">
        <span className="status-label">RPM</span>
        <span className="status-value">{omega_rpm}</span>
      </span>
      <span className="status-item">
        <span className="status-label">RSSI(brain)</span>
        <span className="status-value">{update?.rssi ?? '--'} dBm</span>
      </span>
      <span className="status-item">
        <span className="status-label">RSSI(rx)</span>
        <span className="status-value">{rxRssi} dBm</span>
      </span>
    </div>
  );
}
