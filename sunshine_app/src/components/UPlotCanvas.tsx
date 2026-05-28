import { useRef, useEffect, useCallback } from 'react';
import uPlot from 'uplot';
import 'uplot/dist/uPlot.min.css';
import { invoke } from '@tauri-apps/api/core';

interface Props {
  channels:      string[];
  width:         number;
  height:        number;
  onCursorMove?: (us: number | null) => void;
}

const COLORS = [
  'oklch(.82 .14 168)',
  'oklch(.82 .14 82)',
  'oklch(.78 .12 240)',
  'oklch(.78 .14 300)',
  'oklch(.78 .14 20)',
  'oklch(.85 .10 200)',
];

const AXIS_COLOR  = 'rgba(236,236,238,.28)';
const GRID_COLOR  = 'rgba(255,255,255,.05)';
const TICK_COLOR  = 'rgba(255,255,255,.05)';

const WINDOW_US = 30 * 1_000_000; // 30 second default window

function buildOpts(
  channels:      string[],
  width:         number,
  height:        number,
  onCursorMove?: (us: number | null) => void,
): uPlot.Options {
  return {
    width:  Math.max(100, width),
    height: Math.max(80,  height),
    cursor: { drag: { x: true, y: false } },
    scales: { x: { time: true } },
    axes: [
      {
        stroke: AXIS_COLOR,
        grid:   { stroke: GRID_COLOR, width: 1 },
        ticks:  { stroke: TICK_COLOR },
        font:   '10px JetBrains Mono, monospace',
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
            onCursorMove(u.data[0][idx] * 1e6);
          }
        },
      ],
    },
  };
}

export default function UPlotCanvas({ channels, width, height, onCursorMove }: Props) {
  const divRef  = useRef<HTMLDivElement>(null);
  const uRef    = useRef<uPlot | null>(null);
  const viewRef = useRef({ startUs: Date.now() * 1000 - WINDOW_US, endUs: Date.now() * 1000, live: true });
  const fetchRef = useRef<() => void>(() => {});

  const fetchAndDraw = useCallback(async () => {
    if (!uRef.current) return;

    const now = Date.now() * 1000;
    if (viewRef.current.live) {
      viewRef.current.endUs   = now;
      viewRef.current.startUs = now - WINDOW_US;
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
        if (ci === 0) pts.forEach(([t]) => times.push(t / 1e6));
        series.push(pts.map(([, v]) => v));
      } catch {
        series.push(times.map(() => NaN));
      }
    }

    if (uRef.current) {
      uRef.current.setData(series as uPlot.AlignedData);
    }
  }, [channels, width]);

  // store fetchAndDraw in a ref so the interval always calls the latest version
  useEffect(() => { fetchRef.current = fetchAndDraw; }, [fetchAndDraw]);

  // (re)create uPlot when channels or size changes
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

  // live data refresh at 10 Hz
  useEffect(() => {
    const id = setInterval(() => fetchRef.current(), 100);
    return () => clearInterval(id);
  }, []);

  // scroll: zoom & pan
  useEffect(() => {
    const el = divRef.current;
    if (!el) return;

    const onWheel = (e: WheelEvent) => {
      e.preventDefault();

      const { startUs, endUs } = viewRef.current;
      const span = endUs - startUs;

      // normalize delta — trackpads send small fractional values, wheels send ±100+
      const raw = e.deltaY;
      const norm = Math.sign(raw) * Math.min(Math.abs(raw), 80);

      if (e.ctrlKey || e.metaKey) {
        // zoom centered on cursor
        const factor = 1 + norm * 0.006;
        const rect = el.getBoundingClientRect();
        const ratio = Math.max(0, Math.min(1, (e.clientX - rect.left) / rect.width));
        const anchor = startUs + ratio * span;
        const newSpan = Math.max(2_000_000, Math.min(300_000_000, span * factor));
        viewRef.current = {
          startUs: anchor - ratio * newSpan,
          endUs:   anchor + (1 - ratio) * newSpan,
          live:    false,
        };
      } else {
        // pan (horizontal scroll or plain scroll = time shift)
        const shift = span * norm * 0.012;
        const nowUs = Date.now() * 1000;
        const newEnd   = Math.min(nowUs, endUs + shift);
        const newStart = newEnd - span;
        const isLive   = newEnd >= nowUs - 500_000;
        viewRef.current = { startUs: newStart, endUs: newEnd, live: isLive };
      }

      fetchRef.current();
    };

    el.addEventListener('wheel', onWheel, { passive: false });
    return () => el.removeEventListener('wheel', onWheel);
  }, []);

  return (
    <div
      ref={divRef}
      style={{ width: '100%', height: '100%', background: 'transparent' }}
      onMouseLeave={() => onCursorMove?.(null)}
    />
  );
}
