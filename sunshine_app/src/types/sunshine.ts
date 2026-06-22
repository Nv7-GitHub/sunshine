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

// Channels are grouped Inputs / Real / Replayed. State has separate real vs
// replayed values, and so do the variables (vars are recomputed by the host from
// state+inputs). REAL = filter re-anchored to the logged real state each frame
// (+ midpoint, 100 Hz) → reproduces the robot. REPLAYED = filter free-running
// from the first frame → what the current code does across the whole record.
// Each real.X has a matching rep.X so the two series can be compared directly.
const REAL_REP_FIELDS = [
  { suffix: 'kf_theta',         label: 'θ',             unit: 'rad' },
  { suffix: 'kf_omega',         label: 'ω',             unit: 'rad/s' },
  { suffix: 'theta_offset',     label: 'θ offset',      unit: 'rad' },
  { suffix: 'heading_deg',      label: 'Heading',       unit: '°' },
  { suffix: 'dshot_left',       label: 'DShot L',       unit: '' },
  { suffix: 'dshot_right',      label: 'DShot R',       unit: '' },
  { suffix: 'omega_from_accel', label: 'ω from accel',  unit: 'rad/s' },
  { suffix: 'centripetal_ms2',  label: 'Centripetal',   unit: 'm/s²' },
  { suffix: 'mag_angle',        label: 'Mag angle',     unit: 'rad' },
  { suffix: 'mag_x_filt',       label: 'Mag X (filt)',  unit: 'µT' },
  { suffix: 'mag_y_filt',       label: 'Mag Y (filt)',  unit: 'µT' },
  { suffix: 'erpm_left',        label: 'eRPM L',        unit: 'RPM' },
  { suffix: 'erpm_right',       label: 'eRPM R',        unit: 'RPM' },
  { suffix: 'batt_voltage',     label: 'Battery',       unit: 'V' },
] as const;

const realRep = (prefix: 'real' | 'rep') =>
  REAL_REP_FIELDS.map(f => ({ key: `${prefix}.${f.suffix}`, label: f.label, unit: f.unit }));

export const CHANNELS = {
  Inputs: [
    { key: 'input.accel_x',       label: 'Accel X',       unit: 'counts' },
    { key: 'input.accel_y',       label: 'Accel Y',       unit: 'counts' },
    { key: 'input.accel_z',       label: 'Accel Z',       unit: 'counts' },
    { key: 'input.accel_x_ms2',   label: 'Accel X',       unit: 'm/s²' },
    { key: 'input.accel_y_ms2',   label: 'Accel Y',       unit: 'm/s²' },
    { key: 'input.accel_z_ms2',   label: 'Accel Z',       unit: 'm/s²' },
    { key: 'input.mag_x',         label: 'Mag X',         unit: 'counts' },
    { key: 'input.mag_y',         label: 'Mag Y',         unit: 'counts' },
    { key: 'input.mag_magnitude', label: 'Mag |B|',       unit: 'µT' },
    { key: 'input.erpm_left',     label: 'eRPM L (raw)',  unit: 'counts' },
    { key: 'input.erpm_right',    label: 'eRPM R (raw)',  unit: 'counts' },
    { key: 'input.ctrl_x',        label: 'Ctrl X',        unit: '' },
    { key: 'input.ctrl_y',        label: 'Ctrl Y',        unit: '' },
    { key: 'input.ctrl_theta',    label: 'Ctrl θ',        unit: '' },
    { key: 'input.ctrl_throttle', label: 'Throttle',      unit: '' },
    { key: 'input.rssi',          label: 'RSSI (brain)',  unit: 'dBm' },
    { key: 'input.batt_offset',   label: 'Batt Offset',   unit: 'LSB' },
  ],
  Real:     realRep('real'),
  Replayed: realRep('rep'),
} as const;
