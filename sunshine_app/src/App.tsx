import { useState } from 'react';
import { useAppState } from './hooks/useAppState';
import { useKeyboard } from './hooks/useKeyboard';
import TopBar from './components/TopBar';
import VariableTree from './components/VariableTree';
import GraphPanel from './components/GraphPanel';
import DriverStation from './components/DriverStation';
import './App.css';

const DEFAULT_CHANNELS = ['rep.est_theta', 'rep.est_omega'];

export default function App() {
  const state    = useAppState();
  const inputRef = useKeyboard(state.mode);
  const [selected,  setSelected]  = useState<string[]>(DEFAULT_CHANNELS);
  const [cursorUs,  setCursorUs]  = useState<number | null>(null);

  const toggle = (key: string) =>
    setSelected(prev => prev.includes(key) ? prev.filter(k => k !== key) : [...prev, key]);

  return (
    <div className="app">
      <TopBar
        mode={state.mode}
        liveUpdate={state.liveUpdate}
        rxRssi={state.rxRssi}
        loggingActive={state.loggingActive}
        logPath={state.logPath}
        onEnableLogging={state.enableLogging}
        onDisableLogging={state.disableLogging}
      />
      <VariableTree selected={selected} onToggle={toggle} cursorUs={cursorUs} />
      <GraphPanel   selected={selected} onToggle={toggle} onCursorMove={setCursorUs} />
      <DriverStation
        mode={state.mode}
        setMode={state.setMode}
        sourceStatus={state.sourceStatus}
        liveUpdate={state.liveUpdate}
        inputRef={inputRef}
      />
    </div>
  );
}
