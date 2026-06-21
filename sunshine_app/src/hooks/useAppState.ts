import { useState, useEffect, useCallback, useRef } from 'react';
import { listen } from '@tauri-apps/api/event';
import { invoke } from '@tauri-apps/api/core';
import { LiveUpdate, SourceStatus, Mode } from '../types/sunshine';

export interface ReplayRange { startUs: number; endUs: number }

// Turn any thrown value (string from a Rust command, JS Error, etc.) into a
// readable message for the UI.
function errMsg(e: unknown): string {
  if (typeof e === 'string') return e;
  if (e instanceof Error)    return e.message;
  try { return JSON.stringify(e); } catch { return String(e); }
}

export function useAppState() {
  const [mode,           setModeState]    = useState<Mode>(0);
  const [liveUpdate,     setLiveUpdate]   = useState<LiveUpdate | null>(null);
  const [sourceStatus,   setSourceStatus] = useState<SourceStatus>({ kind: 'Disconnected', detail: '' });
  const [loggingActive,  setLogging]      = useState(false);
  const [logPath,        setLogPath]      = useState('');
  const [rxRssi,         setRxRssi]       = useState<number>(-127);
  const [replayRange,    setReplayRange]  = useState<ReplayRange | null>(null);
  // null = not loading; 0–1 = loading fraction
  const [replayProgress, setReplayProgress] = useState<number | null>(null);
  // Surfaced error message (null = no error). Shown as a dismissible banner.
  const [error,          setError]        = useState<string | null>(null);
  // Cumulative dropped telemetry frames in the current live session (frame_id gaps).
  const [droppedFrames,  setDroppedFrames] = useState(0);
  const lastFrameId = useRef<number | null>(null);

  const reportError = useCallback((e: unknown) => setError(errMsg(e)), []);
  const clearError  = useCallback(() => setError(null), []);

  useEffect(() => {
    const unsub1 = listen<LiveUpdate>('live_update', e => {
      setLiveUpdate(e.payload);
      // Detect dropped frames from frame_id gaps (u16, wraps). Sim sends a
      // constant frame_id (0) so it never trips this.
      const id = e.payload.frame_id;
      if (lastFrameId.current !== null) {
        const gap = (id - lastFrameId.current) & 0xFFFF;
        if (gap > 1 && gap < 0x8000) setDroppedFrames(d => d + (gap - 1));
      }
      lastFrameId.current = id;
    }).catch(() => {});
    const unsub2 = listen<SourceStatus>('source_status', e => {
      setSourceStatus(e.payload);
      // New/ended session → reset the drop counter so old gaps don't linger.
      if (e.payload.kind === 'Disconnected') {
        lastFrameId.current = null;
        setDroppedFrames(0);
      }
    }).catch(() => {});
    const unsub3 = listen<number>('rx_rssi',              e => setRxRssi(e.payload)).catch(() => {});
    const unsub4 = listen<number>('replay_progress',      e => {
      const v = e.payload;
      setReplayProgress(v < 0 ? null : v); // -1 sentinel = done
    }).catch(() => {});
    const unsub5 = listen('force_disabled', () => {
      setModeState(0);
      // Brain rebooted: frame_id restarts, so re-baseline the drop detector.
      lastFrameId.current = null;
      setDroppedFrames(0);
    }).catch(() => {});
    return () => {
      unsub1.then(f => typeof f === 'function' && f()).catch(() => {});
      unsub2.then(f => typeof f === 'function' && f()).catch(() => {});
      unsub3.then(f => typeof f === 'function' && f()).catch(() => {});
      unsub4.then(f => typeof f === 'function' && f()).catch(() => {});
      unsub5.then(f => typeof f === 'function' && f()).catch(() => {});
    };
  }, []);

  const setMode = useCallback((m: Mode) => {
    setModeState(m);
    invoke('set_mode', { mode: m });
    if (m === 0) invoke('set_controls', { x:0, y:0, theta:0, throttle:0 });
  }, []);

  const enableLogging = useCallback(async (label: string) => {
    try {
      const path = await invoke<string>('enable_logging', { label });
      setLogging(true);
      setLogPath(path);
    } catch (e) { reportError(e); }
  }, [reportError]);

  const disableLogging = useCallback(() => {
    invoke('disable_logging').catch(reportError);
    setLogging(false);
  }, [reportError]);

  const loadReplay = useCallback(async (path: string) => {
    try {
      const result = await invoke<{ start_us: number; end_us: number }>('load_replay', { path });
      setReplayRange({ startUs: result.start_us, endUs: result.end_us });
      setLiveUpdate(null); // clear stale live data
    } catch (e) { reportError(e); }
  }, [reportError]);

  const stopSource = useCallback(async () => {
    try {
      await invoke('stop_source');
      setReplayRange(null);
    } catch (e) { reportError(e); }
  }, [reportError]);

  return { mode, setMode, liveUpdate, sourceStatus, loggingActive, logPath,
           enableLogging, disableLogging, rxRssi, replayRange, replayProgress,
           loadReplay, stopSource, error, clearError, reportError, droppedFrames };
}
