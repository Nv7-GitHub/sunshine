import { CHANNELS } from '../types/sunshine';

interface Props {
  selected: string[];
  onToggle: (key: string) => void;
}

export default function ChannelSelector({ selected, onToggle }: Props) {
  return (
    <div className="channel-selector">
      {(Object.entries(CHANNELS) as [string, readonly { key: string; label: string; unit: string }[]][]).map(([group, chans]) => (
        <details key={group} open>
          <summary className="channel-group">{group}</summary>
          {chans.map(ch => (
            <label key={ch.key} className="channel-item">
              <input type="checkbox" checked={selected.includes(ch.key)} onChange={() => onToggle(ch.key)} />
              <span>{ch.label}</span>
              {ch.unit && <span className="unit">{ch.unit}</span>}
            </label>
          ))}
        </details>
      ))}
    </div>
  );
}
