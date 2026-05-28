import { useState } from 'react';

interface Props {
  active:    boolean;
  path:      string;
  onEnable:  (label: string) => void;
  onDisable: () => void;
}

export default function LoggingControl({ active, path, onEnable, onDisable }: Props) {
  const [label, setLabel] = useState('');
  return (
    <div className="logging-control">
      {active ? (
        <>
          <span className="log-indicator active">● REC</span>
          <span className="log-path" title={path}>{path.split('/').pop()}</span>
          <button onClick={onDisable} className="btn-stop-log">Stop</button>
        </>
      ) : (
        <>
          <input
            className="log-label-input"
            placeholder="label (optional)"
            value={label}
            onChange={e => setLabel(e.target.value)}
            onKeyDown={e => e.key === 'Enter' && onEnable(label)}
          />
          <button onClick={() => onEnable(label)} className="btn-start-log">Log</button>
        </>
      )}
    </div>
  );
}
