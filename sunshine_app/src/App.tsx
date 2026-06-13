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
  const inputRef = useKeyboard(state.mode, state.setMode);
  const [selected,    setSelected]    = useState<string[]>(DEFAULT_CHANNELS);
  const [cursorUs,    setCursorUs]    = useState<number | null>(null);
  const [isGraphLive, setIsGraphLive] = useState(true);
  const [liveRequest, setLiveRequest] = useState(0);

  const toggle = (key: string) =>
    setSelected(prev => prev.includes(key) ? prev.filter(k => k !== key) : [...prev, key]);

  // Use 0 when there's no active live source so the RAF extrapolation is disabled.
  const headTimeUs = (state.sourceStatus.kind === 'Replay' || state.sourceStatus.kind === 'Disconnected')
    ? 0 : (state.liveUpdate?.time_us ?? 0);

  return (
    <div className="app">
      <TopBar
        mode={state.mode}
        liveUpdate={state.liveUpdate}
        rxRssi={state.rxRssi}
        loggingActive={state.loggingActive}
        logPath={state.logPath}
        isGraphLive={isGraphLive}
        onGoLive={() => setLiveRequest(r => r + 1)}
        onEnableLogging={state.enableLogging}
        onDisableLogging={state.disableLogging}
      />
      <VariableTree selected={selected} onToggle={toggle} cursorUs={cursorUs} />
      <GraphPanel
        selected={selected}
        onToggle={toggle}
        headTimeUs={headTimeUs}
        requestLive={liveRequest}
        replayRange={state.replayRange}
        onCursorMove={setCursorUs}
        onLiveChange={setIsGraphLive}
      />
      <DriverStation
        mode={state.mode}
        setMode={state.setMode}
        sourceStatus={state.sourceStatus}
        liveUpdate={state.liveUpdate}
        inputRef={inputRef}
        loadReplay={state.loadReplay}
        stopSource={state.stopSource}
      />
      {state.replayProgress !== null && (
        <div style={{
          position: 'fixed', bottom: 0, left: 0, right: 0,
          height: 3, background: 'var(--bg-2)', zIndex: 9999,
        }}>
          <div style={{
            height: '100%',
            width: `${state.replayProgress * 100}%`,
            background: 'var(--accent)',
            transition: 'width 80ms linear',
          }} />
        </div>
      )}
    </div>
  );
}
