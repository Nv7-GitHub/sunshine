import { useState, useEffect, useCallback } from 'react';
import { listen } from '@tauri-apps/api/event';
import { invoke } from '@tauri-apps/api/core';
import { LiveUpdate, SourceStatus, Mode } from '../types/sunshine';

export function useAppState() {
  const [mode,          setModeState]    = useState<Mode>(0);
  const [liveUpdate,    setLiveUpdate]   = useState<LiveUpdate | null>(null);
  const [sourceStatus,  setSourceStatus] = useState<SourceStatus>({ kind: 'Disconnected', detail: '' });
  const [loggingActive, setLogging]      = useState(false);
  const [logPath,       setLogPath]      = useState('');
  const [rxRssi,        setRxRssi]       = useState<number>(-127);

  useEffect(() => {
    const unsub1 = listen<LiveUpdate>('live_update',    e => setLiveUpdate(e.payload)).catch(() => {});
    const unsub2 = listen<SourceStatus>('source_status', e => setSourceStatus(e.payload)).catch(() => {});
    const unsub3 = listen<number>('rx_rssi',            e => setRxRssi(e.payload)).catch(() => {});
    return () => {
      unsub1.then(f => typeof f === 'function' && f()).catch(() => {});
      unsub2.then(f => typeof f === 'function' && f()).catch(() => {});
      unsub3.then(f => typeof f === 'function' && f()).catch(() => {});
    };
  }, []);

  const setMode = useCallback((m: Mode) => {
    setModeState(m);
    invoke('set_mode', { mode: m });
    if (m === 0) invoke('set_controls', { x:0, y:0, theta:0, throttle:0 });
  }, []);

  const enableLogging = useCallback(async (label: string) => {
    const path = await invoke<string>('enable_logging', { label });
    setLogging(true);
    setLogPath(path);
  }, []);

  const disableLogging = useCallback(() => {
    invoke('disable_logging');
    setLogging(false);
  }, []);

  return { mode, setMode, liveUpdate, sourceStatus, loggingActive, logPath,
           enableLogging, disableLogging, rxRssi };
}
