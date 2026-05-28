import { useRef, useEffect, useCallback } from 'react';
import uPlot from 'uplot';
import 'uplot/dist/uPlot.min.css';
import { invoke } from '@tauri-apps/api/core';

interface Props {
  channels: string[];
  width:    number;
  height:   number;
}

const COLORS = ['#3498db','#e74c3c','#2ecc71','#f39c12','#9b59b6','#1abc9c','#e67e22','#34495e'];

export default function UPlotCanvas({ channels, width, height }: Props) {
  const divRef  = useRef<HTMLDivElement>(null);
  const uRef    = useRef<uPlot | null>(null);
  const viewRef = useRef({ startUs: 0, endUs: Date.now() * 1000 });

  const fetchAndDraw = useCallback(async () => {
    if (!uRef.current || channels.length === 0) return;
    const { startUs, endUs } = viewRef.current;
    const maxPts = Math.round(width);

    const times: number[] = [];
    const series: number[][] = [times];

    for (let ci = 0; ci < channels.length; ci++) {
      const pts = await invoke<[number, number][]>('get_graph_data', {
        channel: channels[ci], startUs, endUs, maxPoints: maxPts
      });
      if (ci === 0) {
        pts.forEach(([t]) => times.push(t / 1e6));
      }
      series.push(pts.map(([, v]) => v));
    }

    uRef.current.setData(series as uPlot.AlignedData);
  }, [channels, width]);

  useEffect(() => {
    if (!divRef.current) return;

    const opts: uPlot.Options = {
      width,
      height,
      cursor: { drag: { x: true, y: false } },
      scales: { x: { time: true } },
      axes: [
        { stroke: '#888', grid: { stroke: '#333' } },
        { stroke: '#888', grid: { stroke: '#333' } },
      ],
      series: [
        {},
        ...channels.map((ch, i) => ({
          label:  ch.split('.').pop() ?? ch,
          stroke: COLORS[i % COLORS.length],
          width:  1.5,
        })),
      ],
    };

    uRef.current = new uPlot(opts, [[], ...channels.map(() => [])] as uPlot.AlignedData, divRef.current!);
    fetchAndDraw();

    return () => { uRef.current?.destroy(); uRef.current = null; };
  }, [channels, width, height]);

  useEffect(() => {
    const el = divRef.current;
    if (!el) return;
    const onWheel = (e: WheelEvent) => {
      e.preventDefault();
      const { startUs, endUs } = viewRef.current;
      const span = endUs - startUs;
      if (e.ctrlKey) {
        const factor = e.deltaY > 0 ? 1.1 : 0.9;
        const mid    = (startUs + endUs) / 2;
        viewRef.current = { startUs: mid - span*factor/2, endUs: mid + span*factor/2 };
      } else {
        const shift = span * 0.05 * Math.sign(e.deltaY);
        viewRef.current = { startUs: startUs + shift, endUs: endUs + shift };
      }
      fetchAndDraw();
    };
    el.addEventListener('wheel', onWheel, { passive: false });
    return () => el.removeEventListener('wheel', onWheel);
  }, [fetchAndDraw]);

  return <div ref={divRef} className="uplot-wrap" />;
}
