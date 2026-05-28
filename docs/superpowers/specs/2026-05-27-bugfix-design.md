# Sunshine App — Bug Fix Design
**Date:** 2026-05-27

## Overview

Twelve confirmed bugs found across the Rust backend and React frontend. Grouped into five areas: simulation, graph/variable-tree data pipeline, controls visualization, top-bar labels, and replay file picker.

---

## 1. Simulation (Backend)

### 1a. Events not delivered — use tokio::spawn

`start_simulation` uses `std::thread::spawn`. Tauri v2 event emission (`app.emit`) may require a tokio runtime context internally. The `let _ = app.emit(...)` silently drops any error. Fix: replace with `tokio::spawn` so the loop runs inside the tokio runtime already present in the Tauri process.

### 1b. No `source_status` event

The frontend `sourceStatus` stays `Disconnected` forever. Fix: emit `{kind:"Sim", detail:"Running"}` immediately when the sim starts, and `{kind:"Disconnected", detail:""}` when it stops.

### 1c. Controls not wired into SunshineInput

The simulation builds `SunshineInput` with `..SunshineInput::default()` for all control fields (`ctrl_x`, `ctrl_y`, `ctrl_theta`, `ctrl_throttle`, `mode`). The brain always runs in DISABLED mode with zero inputs. Fix: clone `state.controls: Arc<Mutex<ControlState>>` and move it into the task. Each tick, lock controls and copy `ctrl_x/y/theta/throttle` and `mode` into the `SunshineInput`.

### 1d. Simulation cannot be stopped

`stop_source` only sets `pipeline.source = SourceKind::None` — it does not stop the simulation task. Fix: add `sim_stop: Arc<AtomicBool>` to `AppState`. `start_simulation` sets it `false` before spawning; the loop checks it each tick and exits on `true`. `stop_source` sets it `true` and emits `Disconnected`.

---

## 2. Graph & Variable Tree Data Pipeline

### 2a. Graph always empty — timestamp mismatch (root cause)

`pipeline::ingest_frame` stores `DataPoint { time_us: input.time_us as u64, ... }`. The hardware `time_us` is a local counter starting near 0 (wraps every ~71 min). The frontend queries `get_graph_data` with `startUs = Date.now() * 1000 ≈ 1.748×10¹⁵ µs`. Every stored point satisfies `dp.time_us < start_us`, so all are filtered. **This affects live, replay, and simulation equally — the graph is always empty.**

Fix: in `pipeline::ingest_frame`, ignore `input.time_us` for the ring buffer timestamp. Instead compute:
```rust
use std::time::{SystemTime, UNIX_EPOCH};
let now_us = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_micros() as u64;
```
Use `now_us` as `DataPoint.time_us`. The hardware timestamp is still available inside `DataPoint.input.time_us` for other uses.

### 2b. VariableTree always shows hardcoded `—`

`VariableTree.tsx` renders `—` unconditionally in `var-val`. No live values are ever shown.

**Required behavior:**
- Default: show the latest value from the ring buffer for each channel.
- While cursor is over the graph: show the value at the cursor's x-position (time).
- Cursor leaves graph but window is zoomed in: revert to latest live value.

**Design:**

*New backend command* `get_channel_snapshot(channels: Vec<String>, time_us: Option<u64>) -> Vec<f32>`:
- `time_us = None` → return the value at the most recent ring buffer point for each channel.
- `time_us = Some(t)` → scan backward from ring head to find the point with `time_us` closest to `t`, return its value per channel.
- Returns `f32::NAN` for unknown channels or empty ring. Returns a parallel `Vec<f32>` in the same order as `channels`.

*Shared cursor state:* `App.tsx` holds `cursorUs: number | null`. `UPlotCanvas` receives `onCursorMove: (us: number | null) => void`. Uses uPlot's `cursor.move` hook to call it with the x-axis value converted back to microseconds (`x * 1e6`, since uPlot stores time in seconds), and fires `null` on `mouseleave`. App passes `cursorUs` down to `VariableTree`.

*VariableTree polling:* polls `get_channel_snapshot` at 10 Hz (same interval as graph). When `cursorUs` changes, fires an immediate query. Stores `Map<string, number>` of values. Renders the value (or `—` if NaN/missing) with appropriate precision in `var-val`.

---

## 3. Controls Visualization

### 3a. Theta — replace vertical bar with horizontal slider

The vertical `vbar` for theta is confusing for a bidirectional value. Replace with a horizontal slider element positioned **below the joystick pad** (spanning the same width as the joystick).

Layout: a thin horizontal track (full-width of the joystick) with a filled indicator that slides left (−) to right (+). Center = 0. The indicator width is proportional to `|theta|`. A small circle marks the center-zero. Numeric readout sits below the track.

CSS: add `.hslider`, `.hslider-track`, `.hslider-fill-left`, `.hslider-fill-right` classes. Remove the theta `vbar` from the grid.

Grid change: `controls-grid` changes from `1fr 44px 44px` (joystick + throttle + theta-vbar) to `1fr 44px` (joystick + throttle). Theta slider spans full width below.

### 3b. Joystick history trail

Add a fading trail of the last 30 joystick positions. Store them in a `useRef<{x:number,y:number}[]>` (circular buffer). The RAF loop appends the current position each frame. Render them as SVG `<circle>` elements in the `.joy-grid` SVG, with opacity linearly decreasing from newest to oldest (range: 0.5 → 0.05). Radius: 4px decreasing to 2px.

### 3c. Throttle visualizer lag

Remove `transition: height .08s linear` from `.vbar-fill` in `App.css`. The RAF loop in `ControlsViz` already runs at 60 fps — the CSS transition creates the illusion of lag because it smooths changes instead of tracking the true value. Without the transition, the bar snaps to the exact current value each frame.

---

## 4. Top Bar Labels

| Current | Replace with | Reason |
|---------|-------------|--------|
| `ω` | `SPIN` | `ω` (U+03C9) renders identically to Ω (ohm) in most fonts at small sizes; `SPIN` is unambiguous |
| `RSSI·B` | `RSSI·BOT` | `B` was undocumented; `BOT` makes clear it is the robot brain's RSSI, contrasted with `RSSI·RX` (receiver) |

Value formats stay the same.

---

## 5. Replay File Picker

Add `@tauri-apps/plugin-dialog` (JS) and `tauri-plugin-dialog` (Rust) to enable the native file open dialog.

Replace the free-text path input in the replay tab with:
- A short read-only text field showing the selected filename (or placeholder `No file selected`)
- A `Browse…` button that calls `open({ filters: [{ name: 'Sunshine Log', extensions: ['sun'] }] })`
- On selection, populate `replayPath` state and call `open_replay` automatically

Keep the existing metadata display panel.

---

## Component / File Change Map

| File | Changes |
|------|---------|
| `src-tauri/src/pipeline.rs` | Use `SystemTime::now()` for `DataPoint.time_us`; add `get_point_snapshot` method |
| `src-tauri/src/commands.rs` | `start_simulation` → tokio task, emit source_status, wire controls, check stop flag; add `get_channel_snapshot` command; `stop_source` sets stop flag |
| `src-tauri/src/lib.rs` | Add `sim_stop: Arc<AtomicBool>` to `AppState` |
| `src-tauri/Cargo.toml` | Add `tauri-plugin-dialog` |
| `src-tauri/src/main.rs` or `lib.rs` | Register `tauri-plugin-dialog` |
| `package.json` | Add `@tauri-apps/plugin-dialog` |
| `src/hooks/useAppState.ts` | No changes needed |
| `src/components/TopBar.tsx` | Rename `ω` → `SPIN`, `RSSI·B` → `RSSI·BOT` |
| `src/components/DriverStation.tsx` | Replace theta vbar with horizontal slider; add joystick trail; remove CSS transition workaround |
| `src/components/VariableTree.tsx` | Accept `cursorUs` prop; poll `get_channel_snapshot`; display live values |
| `src/components/UPlotCanvas.tsx` | Wire cursor move hook; call `onCursorMove` callback |
| `src/components/GraphPanel.tsx` | Accept + pass `onCursorMove` |
| `src/App.tsx` | Add `cursorUs` state; wire cursor callbacks |
| `src/App.css` | Remove throttle bar CSS transition; add hslider styles |
| `src/types/sunshine.ts` | No changes needed |
