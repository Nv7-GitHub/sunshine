import { useState } from 'react';
import ChannelSelector from './ChannelSelector';
import UPlotCanvas from './UPlotCanvas';

export default function GraphPanel() {
  const [selected, setSelected] = useState<string[]>(['rep.est_theta', 'rep.est_omega']);

  const toggle = (key: string) =>
    setSelected(prev => prev.includes(key) ? prev.filter(k => k !== key) : [...prev, key]);

  return (
    <div className="graph-panel">
      <ChannelSelector selected={selected} onToggle={toggle} />
      <UPlotCanvas channels={selected} width={900} height={400} />
    </div>
  );
}
