import { useRef, useEffect, useCallback } from 'react';
import uPlot from 'uplot';
import 'uplot/dist/uPlot.min.css';
import { invoke } from '@tauri-apps/api/core';

interface Props {
  channels:      string[];
  width:         number;
  height:        number;
  headTimeUs:    number;
  requestLive:   number;
  onCursorMove?: (us: number | null) => void;
  onLiveChange?: (live: boolean) => void;
}

const COLORS = [
  'oklch(.82 .14 168)',
  'oklch(.82 .14 82)',
  'oklch(.78 .12 240)',
  'oklch(.78 .14 300)',
  'oklch(.78 .14 20)',
  'oklch(.85 .10 200)',
];

const AXIS_COLOR = 'rgba(236,236,238,.28)';
const GRID_COLOR = 'rgba(255,255,255,.05)';
const TICK_COLOR = 'rgba(255,255,255,.05)';

const WINDOW_US = 30 * 1_000_000;

function fmtElapsed(_u: uPlot, vals: (number | null | undefined)[]): string[] {
  return vals.map(v => {
    if (v == null) return '';
    const s = v / 1_000_000;
    return s < 60 ? s.toFixed(1) + 's' : (s / 60).toFixed(2) + 'm';
  });
}

function buildOpts(
  channels:      string[],
  width:         number,
  height:        number,
  onCursorMove?: (us: number | null) => void,
): uPlot.Options {
  return {
    width:  Math.max(100, width),
    height: Math.max(80,  height),
    cursor: { drag: { x: false, y: false } },
    scales: { x: { time: false } },
    axes: [
      {
        stroke: AXIS_COLOR,
        grid:   { stroke: GRID_COLOR, width: 1 },
        ticks:  { stroke: TICK_COLOR },
        font:   '10px JetBrains Mono, monospace',
        values: fmtElapsed,
      },
      {
        stroke: AXIS_COLOR,
        grid:   { stroke: GRID_COLOR, width: 1 },
        ticks:  { stroke: TICK_COLOR },
        font:   '10px JetBrains Mono, monospace',
        size:   54,
      },
    ],
    series: [
      {},
      ...channels.map((ch, i) => ({
        label:  ch.split('.').pop() ?? ch,
        stroke: COLORS[i % COLORS.length],
        width:  1.5,
      })),
    ],
    hooks: {
      setCursor: [
        (u: uPlot) => {
          if (!onCursorMove) return;
          const idx = u.cursor.idx;
          if (idx !== null && idx !== undefined && u.data[0]?.[idx] !== undefined) {
            onCursorMove(u.data[0][idx]);
          }
        },
      ],
    },
  };
}

export default function UPlotCanvas({ channels, width, height, headTimeUs, requestLive, onCursorMove, onLiveChange }: Props) {
  const divRef      = useRef<HTMLDivElement>(null);
  const uRef        = useRef<uPlot | null>(null);
  const viewRef     = useRef({ startUs: 0, endUs: 0, live: true });
  const fetchRef    = useRef<() => void>(() => {});
  const headRef     = useRef(headTimeUs);
  const liveChgRef  = useRef(onLiveChange);
  headRef.current    = headTimeUs;
  liveChgRef.current = onLiveChange;

  const setLive = useCallback((live: boolean) => {
    if (viewRef.current.live !== live) {
      viewRef.current.live = live;
      liveChgRef.current?.(live);
    }
  }, []);

  const fetchAndDraw = useCallback(async () => {
    if (!uRef.current) return;

    const head = headRef.current;
    if (viewRef.current.live && head > 0) {
      viewRef.current.endUs   = head;
      viewRef.current.startUs = Math.max(0, head - WINDOW_US);
    }

    const { startUs, endUs } = viewRef.current;
    if (channels.length === 0) {
      uRef.current.setData([[], ...channels.map(() => [])] as uPlot.AlignedData);
      return;
    }

    const maxPts = Math.round(Math.max(100, width));
    const times: number[] = [];
    const series: number[][] = [times];

    for (let ci = 0; ci < channels.length; ci++) {
      try {
        const pts = await invoke<[number, number][]>('get_graph_data', {
          channel: channels[ci], startUs, endUs, maxPoints: maxPts,
        });
        if (ci === 0) pts.forEach(([t]) => times.push(t));
        series.push(pts.map(([, v]) => v));
      } catch {
        series.push(times.map(() => NaN));
      }
    }

    if (uRef.current) {
      uRef.current.setData(series as uPlot.AlignedData);
      if (startUs < endUs) {
        uRef.current.setScale('x', { min: startUs, max: endUs });
      }
    }
  }, [channels, width]);

  useEffect(() => { fetchRef.current = fetchAndDraw; }, [fetchAndDraw]);

  // (re)create uPlot when channels change
  useEffect(() => {
    if (!divRef.current) return;
    uRef.current?.destroy();

    const opts = buildOpts(channels, width, height, onCursorMove);
    const data = [[], ...channels.map(() => [])] as uPlot.AlignedData;
    uRef.current = new uPlot(opts, data, divRef.current!);
    fetchAndDraw();

    return () => { uRef.current?.destroy(); uRef.current = null; };
  }, [channels]); // eslint-disable-line react-hooks/exhaustive-deps

  // resize without recreating
  useEffect(() => {
    if (uRef.current && width > 0 && height > 0) {
      uRef.current.setSize({ width: Math.max(100, width), height: Math.max(80, height) });
    }
  }, [width, height]);

  // jump back to live when requestLive bumps
  useEffect(() => {
    if (requestLive === 0) return;
    setLive(true);
    fetchRef.current();
  }, [requestLive, setLive]);

  // live refresh at 10 Hz
  useEffect(() => {
    const id = setInterval(() => fetchRef.current(), 100);
    return () => clearInterval(id);
  }, []);

  // scroll: plain scroll = zoom around cursor, ctrl/meta = pan
  useEffect(() => {
    const el = divRef.current;
    if (!el) return;

    const onWheel = (e: WheelEvent) => {
      e.preventDefault();
      if (!uRef.current) return;

      const { startUs, endUs } = viewRef.current;
      const span = endUs - startUs;
      const raw  = e.deltaY;
      const norm = Math.sign(raw) * Math.min(Math.abs(raw), 80);

      if (e.ctrlKey || e.metaKey) {
        // pan
        const shift    = span * norm * 0.012;
        const head     = headRef.current;
        const newEnd   = head > 0 ? Math.min(head, endUs + shift) : endUs + shift;
        const newStart = newEnd - span;
        const isLive   = head > 0 && newEnd >= head - 500_000;
        viewRef.current = { startUs: newStart, endUs: newEnd, live: isLive };
        setLive(isLive);
      } else {
        // zoom around cursor — use uPlot's posToVal for exact time at mouse
        const rect   = el.getBoundingClientRect();
        const px     = e.clientX - rect.left;
        const anchor = uRef.current.posToVal(px, 'x');
        const factor = 1 + norm * 0.006;
        const newSpan = Math.max(2_000_000, Math.min(600_000_000, span * factor));
        const ratio   = span > 0 ? (anchor - startUs) / span : 0.5;
        const clamped = Math.max(0, Math.min(1, ratio));
        const newStart = anchor - clamped * newSpan;
        const newEnd   = anchor + (1 - clamped) * newSpan;
        const head     = headRef.current;
        const isLive   = head > 0 && newEnd >= head - 500_000;
        viewRef.current = { startUs: newStart, endUs: newEnd, live: isLive };
        setLive(isLive);
      }

      fetchRef.current();
    };

    el.addEventListener('wheel', onWheel, { passive: false });
    return () => el.removeEventListener('wheel', onWheel);
  }, [setLive]);

  return (
    <div
      ref={divRef}
      style={{ width: '100%', height: '100%', background: 'transparent' }}
      onMouseLeave={() => onCursorMove?.(null)}
    />
  );
}
