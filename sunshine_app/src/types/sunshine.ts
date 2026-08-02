export type Mode = 0 | 1 | 2;
export const MODE_NAMES = ['DISABLED', 'TANK', 'MELTY'] as const;

export interface LiveUpdate {
  frame_id:    number;
  est_theta:   number;
  est_omega:   number;
  mode:        Mode;
  rssi:        number;
  batt_offset: number;
  time_us:     number;
}

export interface SourceStatus {
  kind:   'Live' | 'Replay' | 'Sim' | 'Disconnected';
  code?:  number;
  detail: string;
}

export interface LogStatus {
  active:      boolean;
  path:        string;
  frame_count: number;
}

// Channels fall into four groups:
//   Inputs    — raw sensor data below the IO layer (1 kHz, shared).
//   Variables — pure functions of the INPUTS only (no filter state), so real and
//               replayed are identical: ONE shared 1 kHz series (battery, eRPM,
//               ω-from-accel, centripetal, accel in m/s²). Not sent over telemetry.
//   Real /    — quantities that depend on the filter STATE, so they differ between
//   Replayed    the recorded run and a re-run. REAL = the logged 100 Hz state
//               snapshots (drawn as coarse lines between the two samples/frame).
//               REPLAYED = filter free-running at 1 kHz from the first frame.
// Each real.X has a matching rep.X so the two can be compared directly.
const REAL_REP_FIELDS = [
  { suffix: 'kf_theta',         label: 'θ',             unit: 'rad' },
  { suffix: 'kf_omega',         label: 'ω',             unit: 'rad/s' },
  { suffix: 'theta_offset',     label: 'θ offset',      unit: 'rad' },
  { suffix: 'heading_deg',      label: 'Heading',       unit: '°' },
  { suffix: 'dshot_left',       label: 'DShot L',       unit: '' },
  { suffix: 'dshot_right',      label: 'DShot R',       unit: '' },
  { suffix: 'mag_angle',        label: 'Mag angle',     unit: 'rad' },
  { suffix: 'mag_x_filt',       label: 'Mag X (filt)',  unit: 'µT' },
  { suffix: 'mag_y_filt',       label: 'Mag Y (filt)',  unit: 'µT' },
] as const;

const realRep = (prefix: 'real' | 'rep') =>
  REAL_REP_FIELDS.map(f => ({ key: `${prefix}.${f.suffix}`, label: f.label, unit: f.unit }));

export const CHANNELS = {
  Inputs: [
    { key: 'input.accel_x',       label: 'Accel X',       unit: 'counts' },
    { key: 'input.accel_y',       label: 'Accel Y',       unit: 'counts' },
    { key: 'input.accel_z',       label: 'Accel Z',       unit: 'counts' },
    { key: 'input.mag_x',         label: 'Mag X',         unit: 'counts' },
    { key: 'input.mag_y',         label: 'Mag Y',         unit: 'counts' },
    { key: 'input.mag_magnitude', label: 'Mag |B|',       unit: 'µT' },
    // Wire format is a float16 BIT PATTERN, not a count; the backend now decodes
    // it (pipeline.rs channel_accessor), so the unit is RPM and these land on the
    // same axis as var.erpm_*. "raw" means straight off the ESC — unfiltered and
    // NaN where the ESC was not commutating — not undecoded.
    { key: 'input.erpm_left',     label: 'eRPM L (raw)',  unit: 'RPM' },
    { key: 'input.erpm_right',    label: 'eRPM R (raw)',  unit: 'RPM' },
    { key: 'input.ctrl_x',        label: 'Ctrl X',        unit: '' },
    { key: 'input.ctrl_y',        label: 'Ctrl Y',        unit: '' },
    { key: 'input.ctrl_theta',    label: 'Ctrl θ',        unit: '' },
    { key: 'input.ctrl_throttle', label: 'Throttle',      unit: '' },
    { key: 'input.rssi',          label: 'RSSI (brain)',  unit: 'dBm' },
    { key: 'input.batt_offset',   label: 'Batt Offset',   unit: 'LSB' },
  ],
  // Variables: pure functions of the inputs (no filter state) → one shared 1 kHz
  // series (identical for real and replayed). Each distinct unit gets its own axis.
  Variables: [
    { key: 'var.omega_from_accel', label: 'ω from accel', unit: 'rad/s' },
    { key: 'var.centripetal_ms2',  label: 'Centripetal',  unit: 'm/s²' },
    { key: 'var.accel_x_ms2',      label: 'Accel X',      unit: 'm/s²' },
    { key: 'var.accel_y_ms2',      label: 'Accel Y',      unit: 'm/s²' },
    { key: 'var.accel_z_ms2',      label: 'Accel Z',      unit: 'm/s²' },
    { key: 'var.batt_voltage',     label: 'Battery',      unit: 'V' },
    { key: 'var.erpm_left',        label: 'eRPM L',       unit: 'RPM' },
    { key: 'var.erpm_right',       label: 'eRPM R',       unit: 'RPM' },
  ],
  Real:     realRep('real'),
  Replayed: realRep('rep'),
} as const;
