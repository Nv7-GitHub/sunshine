import ModeButtons from './ModeButtons';
import StatusBar from './StatusBar';
import LoggingControl from './LoggingControl';
import type { Mode, LiveUpdate } from '../types/sunshine';

interface Props {
  mode:           Mode;
  setMode:        (m: Mode) => void;
  liveUpdate:     LiveUpdate | null;
  rxRssi:         number;
  loggingActive:  boolean;
  logPath:        string;
  enableLogging:  (label: string) => void;
  disableLogging: () => void;
}

export default function Header(p: Props) {
  return (
    <header className="header">
      <ModeButtons mode={p.mode} setMode={p.setMode} />
      <StatusBar update={p.liveUpdate} rxRssi={p.rxRssi} />
      <LoggingControl active={p.loggingActive} path={p.logPath}
                      onEnable={p.enableLogging} onDisable={p.disableLogging} />
    </header>
  );
}
