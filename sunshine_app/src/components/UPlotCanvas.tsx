import { useRef, useEffect, useCallback } from 'react';
import uPlot from 'uplot';
import 'uplot/dist/uPlot.min.css';
import { invoke } from '@tauri-apps/api/core';

interface Props {
  channels:      string[];
  channelUnits:  string[];
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

function scaleKey(unit: string): string {
  return unit ? `u_${unit.replace(/[^a-zA-Z0-9]/g, '_')}` : 'u_dim';
}

function buildOpts(
  channels:      string[],
  channelUnits:  string[],
  width:         number,
  height:        number,
  onCursorMove?: (us: number | null) => void,
): uPlot.Options {
  // Collect unique units in first-seen order
  const unitOrder: string[] = [];
  for (const u of channelUnits) {
    if (!unitOrder.includes(u)) unitOrder.push(u);
  }

  const scales: uPlot.Options['scales'] = { x: { time: false } };
  for (const u of unitOrder) scales[scaleKey(u)] = {};

  const yAxes: uPlot.Axis[] = unitOrder.map((u, i) => ({
    scale:  scaleKey(u),
    side:   i % 2 === 0 ? 3 : 1,   // alternate left / right
    stroke: AXIS_COLOR,
    grid:   i === 0 ? { stroke: GRID_COLOR, width: 1 } : { show: false },
    ticks:  { stroke: TICK_COLOR },
    font:   '10px JetBrains Mono, monospace',
    size:   54,
    label:  u || undefined,
  }));

  return {
    width:  Math.max(100, width),
    height: Math.max(80,  height),
    cursor: { drag: { x: false, y: false }, sync: { key: '' } },
    scales,
    axes: [
      {
        stroke: AXIS_COLOR,
        grid:   { stroke: GRID_COLOR, width: 1 },
        ticks:  { stroke: TICK_COLOR },
        font:   '10px JetBrains Mono, monospace',
        values: fmtElapsed,
      },
      ...yAxes,
    ],
    series: [
      {},
      ...channels.map((ch, i) => ({
        label:  ch.split('.').pop() ?? ch,
        stroke: COLORS[i % COLORS.length],
        width:  1.5,
        scale:  scaleKey(channelUnits[i] ?? ''),
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

// Clamp to valid u64 range and round to integer
function toUs(v: number): number {
  return Math.max(0, Math.round(v));
}

export default function UPlotCanvas({ channels, channelUnits, width, height, headTimeUs, requestLive, onCursorMove, onLiveChange }: Props) {
  const divRef     = useRef<HTMLDivElement>(null);
  const uRef       = useRef<uPlot | null>(null);
  const viewRef    = useRef({ startUs: 0, endUs: 0, live: true });
  const fetchRef   = useRef<() => void>(() => {});
  const headRef    = useRef(headTimeUs);
  const liveChgRef = useRef(onLiveChange);
  headRef.current    = headTimeUs;
  liveChgRef.current = onLiveChange;

  const prevLiveRef = useRef(true);
  const notifyLive = useCallback((live: boolean) => {
    viewRef.current.live = live;
    if (prevLiveRef.current !== live) {
      prevLiveRef.current = live;
      liveChgRef.current?.(live);
    }
  }, []);

  const fetchAndDraw = useCallback(async () => {
    const u = uRef.current;
    if (!u) return;

    const head = headRef.current;
    if (viewRef.current.live && head > 0) {
      const currentSpan = viewRef.current.endUs - viewRef.current.startUs;
      const span = currentSpan > 0 ? currentSpan : WINDOW_US;
      viewRef.current.endUs   = head;
      viewRef.current.startUs = Math.max(0, head - span);
    }

    const startUs = toUs(viewRef.current.startUs);
    const endUs   = toUs(viewRef.current.endUs);

    if (channels.length === 0) {
      u.setData([[], ...channels.map(() => [])] as uPlot.AlignedData);
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

    if (!uRef.current) return;
    uRef.current.setData(series as uPlot.AlignedData);
    if (startUs < endUs) {
      uRef.current.setScale('x', { min: startUs, max: endUs });
    }
  }, [channels, width]);

  useEffect(() => { fetchRef.current = fetchAndDraw; }, [fetchAndDraw]);

  // (re)create uPlot when channels change
  useEffect(() => {
    if (!divRef.current) return;
    uRef.current?.destroy();

    const opts = buildOpts(channels, channelUnits, width, height, onCursorMove);
    uRef.current = new uPlot(opts, [[], ...channels.map(() => [])] as uPlot.AlignedData, divRef.current);
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
    prevLiveRef.current = true;
    viewRef.current.live = true;
    liveChgRef.current?.(true);
    fetchRef.current();
  }, [requestLive]);

  // live refresh at 10 Hz
  useEffect(() => {
    const id = setInterval(() => fetchRef.current(), 100);
    return () => clearInterval(id);
  }, []);

  // scroll: attach imperatively to the div so { passive: false } is effective
  useEffect(() => {
    const el = divRef.current;
    if (!el) return;

    const onWheel = (e: WheelEvent) => {
      if (!uRef.current) return;
      e.preventDefault();

      const head = headRef.current;
      // Bootstrap view if empty
      if (viewRef.current.startUs === 0 && viewRef.current.endUs === 0) {
        const end = head > 0 ? head : WINDOW_US;
        viewRef.current = { startUs: Math.max(0, end - WINDOW_US), endUs: end, live: head > 0 };
      }

      const { startUs, endUs } = viewRef.current;
      const span = Math.max(endUs - startUs, 1);
      const norm = Math.sign(e.deltaY) * Math.min(Math.abs(e.deltaY), 80);

      if (e.ctrlKey || e.metaKey) {
        // pan
        const shift    = span * norm * 0.015;
        const newEnd   = head > 0 ? Math.min(head, endUs + shift) : endUs + shift;
        const newStart = Math.max(0, newEnd - span);
        const isLive   = head > 0 && toUs(newEnd) >= head - 500_000;
        viewRef.current = { startUs: newStart, endUs: newEnd, live: isLive };
        notifyLive(isLive);
      } else {
        // zoom around cursor — use u.over rect so the axis gutter doesn't skew frac
        const overRect = uRef.current.over.getBoundingClientRect();
        const frac   = Math.max(0, Math.min(1, (e.clientX - overRect.left) / overRect.width));
        const anchor = startUs + frac * span;
        const factor = 1 + norm * 0.008;
        const newSpan  = Math.max(1_000, span * factor);
        const newStart = Math.max(0, anchor - frac * newSpan);
        const newEnd   = anchor + (1 - frac) * newSpan;
        // re-enter live when zooming out to the live head
        if (head > 0 && newEnd >= head) {
          viewRef.current = { startUs: Math.max(0, head - WINDOW_US), endUs: head, live: true };
          notifyLive(true);
        } else {
          viewRef.current = { startUs: newStart, endUs: newEnd, live: false };
          notifyLive(false);
        }
      }

      fetchRef.current();
    };

    el.addEventListener('wheel', onWheel, { passive: false });
    return () => el.removeEventListener('wheel', onWheel);
  }, [notifyLive]);

  return (
    <div
      ref={divRef}
      style={{ width: '100%', height: '100%', background: 'transparent' }}
      onMouseLeave={() => onCursorMove?.(null)}
    />
  );
}
