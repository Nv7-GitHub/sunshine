import { useAppState } from './hooks/useAppState';
import { useKeyboard } from './hooks/useKeyboard';
import Header from './components/Header';
import ConnectionPanel from './components/ConnectionPanel';
import GraphPanel from './components/GraphPanel';
import './App.css';

export default function App() {
  const state = useAppState();
  useKeyboard(state.mode);

  return (
    <div className="app">
      <Header {...state} />
      <div className="main-layout">
        <ConnectionPanel sourceStatus={state.sourceStatus} />
        <GraphPanel />
      </div>
    </div>
  );
}
