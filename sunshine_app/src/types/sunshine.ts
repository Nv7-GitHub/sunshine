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

export const CHANNELS = {
  Inputs: [
    { key: 'input.accel_x',       label: 'Accel X',       unit: 'counts' },
    { key: 'input.accel_y',       label: 'Accel Y',       unit: 'counts' },
    { key: 'input.accel_z',       label: 'Accel Z',       unit: 'counts' },
    { key: 'input.mag_x',         label: 'Mag X',         unit: 'counts' },
    { key: 'input.mag_y',         label: 'Mag Y',         unit: 'counts' },
    { key: 'input.erpm_left',     label: 'eRPM L',        unit: 'counts' },
    { key: 'input.erpm_right',    label: 'eRPM R',        unit: 'counts' },
    { key: 'input.ctrl_x',        label: 'Ctrl X',        unit: '' },
    { key: 'input.ctrl_y',        label: 'Ctrl Y',        unit: '' },
    { key: 'input.ctrl_throttle', label: 'Throttle',      unit: '' },
    { key: 'input.rssi',          label: 'RSSI (brain)',  unit: 'dBm' },
    { key: 'input.batt_offset',   label: 'Batt Offset',   unit: 'LSB' },
  ],
  'State (replayed)': [
    { key: 'rep.est_theta',        label: 'θ',           unit: 'rad' },
    { key: 'rep.est_omega',        label: 'ω',           unit: 'rad/s' },
    { key: 'rep.mag_angle',        label: 'Mag angle',   unit: 'rad' },
    { key: 'rep.derot_i',          label: 'Derot I',     unit: 'µT' },
    { key: 'rep.derot_q',          label: 'Derot Q',     unit: 'µT' },
    { key: 'rep.dshot_left',       label: 'DShot L',     unit: '' },
    { key: 'rep.dshot_right',      label: 'DShot R',     unit: '' },
  ],
  'State (real)': [
    { key: 'real.est_theta',       label: 'θ',           unit: 'rad' },
    { key: 'real.est_omega',       label: 'ω',           unit: 'rad/s' },
    { key: 'real.dshot_left',      label: 'DShot L',     unit: '' },
    { key: 'real.dshot_right',     label: 'DShot R',     unit: '' },
  ],
  Variables: [
    { key: 'rep.omega_from_accel', label: 'ω from accel',  unit: 'rad/s' },
    { key: 'rep.centripetal_ms2',  label: 'Centripetal',   unit: 'm/s²' },
    { key: 'rep.erpm_left',        label: 'eRPM L',        unit: 'RPM' },
    { key: 'rep.erpm_right',       label: 'eRPM R',        unit: 'RPM' },
    { key: 'input.accel_x_ms2',    label: 'Accel X',       unit: 'm/s²' },
    { key: 'input.accel_y_ms2',    label: 'Accel Y',       unit: 'm/s²' },
    { key: 'input.accel_z_ms2',    label: 'Accel Z',       unit: 'm/s²' },
    { key: 'input.mag_magnitude',  label: 'Mag |B|',       unit: 'µT' },
    { key: 'rep.batt_voltage',     label: 'Battery',       unit: 'V' },
  ],
} as const;
