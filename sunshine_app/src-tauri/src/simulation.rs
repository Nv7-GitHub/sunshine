use crate::ffi::{SunshineInput, SunshineVars, f32_to_f16};
use std::f64::consts::PI;

const KV:           f64 = 1100.0;
const KT:           f64 = 60.0 / (2.0*PI*KV);
const R_PHASE:      f64 = 0.075;
const V_NOMINAL:    f64 = 8.4;
const R_INTERNAL:   f64 = 0.008;
const WHEEL_RADIUS: f64 = 0.022;
const WHEEL_CENTER: f64 = 0.0405;
const MOI:          f64 = 1.214e-3;
const WHEEL_INERTIA:f64 = 1.0e-6;
const IMU_RADIUS:   f64 = 0.011;
const EARTH_FIELD:  f64 = 50.0;
const EARTH_ANGLE:  f64 = 0.0;
const ADXL_SCALE:   f64 = 49e-3 * 9.81;
const ADXL_MAX_G:   f64 = 200.0;           // ADXL375 physical ±200g range
const ADXL_MAX_CNT: f64 = ADXL_MAX_G / (49e-3);  // ≈ 4082 counts
const MAG_SCALE:    f64 = 0.058;
const BATT_REF_V:   f64 = 7.6;
const BATT_SCALE:   f64 = 0.0205;
const POLE_PAIRS:   f64 = 7.0;             // 14-pole motor → 7 pole pairs

pub struct Simulation {
    body_omega:  f64,
    body_angle:  f64,
    omega_left:  f64,
    omega_right: f64,
    time_us:     u64,
}

const DSHOT_NEUTRAL:  f64 = 1048.0;
const DSHOT_MAX:      f64 = 2047.0;
const DSHOT_MIN:      f64 = 48.0;
const BODY_DRAG:      f64 = 0.05;  // lumped body spin drag (1/s)
const ROLLING_RATIO:  f64 = WHEEL_CENTER / WHEEL_RADIUS;
const EFFECTIVE_MOI:  f64 = MOI + 2.0 * WHEEL_INERTIA * ROLLING_RATIO * ROLLING_RATIO;

fn dshot_to_throttle(dshot: f32) -> f64 {
    let d = dshot as f64;
    if d < DSHOT_MIN { return 0.0; }  // 0 = disarmed, 1–47 = special commands → coast
    if d >= DSHOT_NEUTRAL {
        (d - DSHOT_NEUTRAL) / (DSHOT_MAX - DSHOT_NEUTRAL)
    } else {
        -((DSHOT_NEUTRAL - d) / (DSHOT_NEUTRAL - DSHOT_MIN))
    }
}

impl Simulation {
    pub fn new() -> Self {
        Simulation { body_omega: 0.0, body_angle: 0.0,
                     omega_left: 0.0, omega_right: 0.0, time_us: 0 }
    }

    pub fn tick(&mut self, last_vars: &SunshineVars) -> SunshineInput {
        let dt = 1e-3f64;
        self.time_us += 1000;

        self.sync_wheels_to_body();

        let v_term = self.terminal_voltage(last_vars.dshot_cmd_left, last_vars.dshot_cmd_right);
        let torque_l = self.motor_torque(dshot_to_throttle(last_vars.dshot_cmd_left),  self.omega_left,  v_term);
        let torque_r = self.motor_torque(dshot_to_throttle(last_vars.dshot_cmd_right), self.omega_right, v_term);

        let torque_body = (torque_l + torque_r) * ROLLING_RATIO;
        let alpha = torque_body / EFFECTIVE_MOI;
        self.body_omega += alpha * dt;
        self.body_omega *= 1.0 - BODY_DRAG * dt;
        self.body_angle += self.body_omega * dt;
        self.sync_wheels_to_body();

        let a_centripetal = self.body_omega.powi(2) * IMU_RADIUS;
        let a_tangential  = alpha * IMU_RADIUS;
        let ax = (a_centripetal - a_tangential) / 2.0f64.sqrt();
        let ay = (a_centripetal + a_tangential) / 2.0f64.sqrt();
        let az = 9.81f64;

        let mx = EARTH_FIELD * (EARTH_ANGLE - self.body_angle).cos();
        let my = EARTH_FIELD * (EARTH_ANGLE - self.body_angle).sin();

        let i_total = self.supply_current(last_vars.dshot_cmd_left, last_vars.dshot_cmd_right, V_NOMINAL);
        let v_batt  = V_NOMINAL - i_total * R_INTERNAL;
        let batt_offset = ((v_batt - BATT_REF_V) / BATT_SCALE).round().clamp(-127.0, 127.0) as i8;

        SunshineInput {
            time_us:       self.time_us as u32,
            accel_x:       (ax / ADXL_SCALE).round().clamp(-ADXL_MAX_CNT, ADXL_MAX_CNT) as i16,
            accel_y:       (ay / ADXL_SCALE).round().clamp(-ADXL_MAX_CNT, ADXL_MAX_CNT) as i16,
            accel_z:       (az / ADXL_SCALE).round().clamp(-32768.0, 32767.0) as i16,
            mag_x:         (mx / MAG_SCALE).round().clamp(-32768.0, 32767.0) as i16,
            mag_y:         (my / MAG_SCALE).round().clamp(-32768.0, 32767.0) as i16,
            mag_z:         0,
            erpm_left:     f32_to_f16((self.omega_left  * 60.0 / (2.0 * PI) * POLE_PAIRS) as f32),
            erpm_right:    f32_to_f16((self.omega_right * 60.0 / (2.0 * PI) * POLE_PAIRS) as f32),
            batt_offset,
            ..SunshineInput::default()
        }
    }

    fn terminal_voltage(&self, cmd_l: f32, cmd_r: f32) -> f64 {
        V_NOMINAL - self.supply_current(cmd_l, cmd_r, V_NOMINAL) * R_INTERNAL
    }

    fn supply_current(&self, cmd_l: f32, cmd_r: f32, v_term: f64) -> f64 {
        let i_l = self.motor_current(dshot_to_throttle(cmd_l), self.omega_left,  v_term);
        let i_r = self.motor_current(dshot_to_throttle(cmd_r), self.omega_right, v_term);
        (i_l + i_r).max(0.0)
    }

    fn motor_current(&self, throttle: f64, omega: f64, v_term: f64) -> f64 {
        let v_motor  = throttle * v_term;
        let back_emf = omega / (KV * 2.0 * PI / 60.0);
        // Allow negative current: back-EMF brakes the motor when throttle is low
        (v_motor - back_emf) / R_PHASE
    }

    fn motor_torque(&self, throttle: f64, omega: f64, v_term: f64) -> f64 {
        KT * self.motor_current(throttle, omega, v_term)
    }

    fn sync_wheels_to_body(&mut self) {
        let wheel_omega = self.body_omega * ROLLING_RATIO;
        self.omega_left = wheel_omega;
        self.omega_right = wheel_omega;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ffi::{f16_to_f32, SunshineVars};

    #[test]
    fn melty_full_throttle_can_saturate_accelerometer() {
        let mut sim = Simulation::new();
        let cmd = SunshineVars {
            dshot_cmd_left: DSHOT_MAX as f32,
            dshot_cmd_right: DSHOT_MAX as f32,
            ..SunshineVars::default()
        };

        let mut input = SunshineInput::default();
        for _ in 0..10_000 {
            input = sim.tick(&cmd);
        }

        let ax = input.accel_x as f64 * ADXL_SCALE;
        let ay = input.accel_y as f64 * ADXL_SCALE;
        let centripetal = (ax * ax + ay * ay).sqrt();
        let erpm_left = f16_to_f32(input.erpm_left);

        assert!(erpm_left > 55_000.0);
        assert!(centripetal > 280.0 * 9.81);
        assert!(input.accel_x.abs() >= ADXL_MAX_CNT as i16 ||
                input.accel_y.abs() >= ADXL_MAX_CNT as i16);
    }
}
