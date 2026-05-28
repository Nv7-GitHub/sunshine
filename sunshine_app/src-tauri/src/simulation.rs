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
const IMU_RADIUS:   f64 = 0.011;
const EARTH_FIELD:  f64 = 50.0;
const EARTH_ANGLE:  f64 = 0.0;
const ADXL_SCALE:   f64 = 49e-3 * 9.81;
const MAG_SCALE:    f64 = 0.058;
const BATT_REF_V:   f64 = 7.6;
const BATT_SCALE:   f64 = 0.0205;

pub struct Simulation {
    body_omega:  f64,
    body_angle:  f64,
    omega_left:  f64,
    omega_right: f64,
    time_us:     u64,
}

impl Simulation {
    pub fn new() -> Self {
        Simulation { body_omega: 0.0, body_angle: 0.0,
                     omega_left: 0.0, omega_right: 0.0, time_us: 0 }
    }

    pub fn tick(&mut self, last_vars: &SunshineVars) -> SunshineInput {
        let dt = 1e-3f64;
        self.time_us += 1000;

        let v_term = self.terminal_voltage(last_vars.dshot_cmd_left, last_vars.dshot_cmd_right);
        let (torque_l, self_omega_l) = self.motor_tick(last_vars.dshot_cmd_left as f64 / 2047.0, self.omega_left,  v_term, dt);
        let (torque_r, self_omega_r) = self.motor_tick(last_vars.dshot_cmd_right as f64 / 2047.0, self.omega_right, v_term, dt);
        self.omega_left  = self_omega_l;
        self.omega_right = self_omega_r;

        let torque_body = (torque_l + torque_r) * WHEEL_CENTER / WHEEL_RADIUS;
        let alpha = torque_body / MOI;
        self.body_omega += alpha * dt;
        self.body_angle += self.body_omega * dt;

        let a_centripetal = self.body_omega.powi(2) * IMU_RADIUS;
        let a_tangential  = alpha * IMU_RADIUS;
        let ax = (a_centripetal - a_tangential) / 2.0f64.sqrt();
        let ay = (a_centripetal + a_tangential) / 2.0f64.sqrt();
        let az = 9.81f64;

        let mx = EARTH_FIELD * (EARTH_ANGLE - self.body_angle).cos();
        let my = EARTH_FIELD * (EARTH_ANGLE - self.body_angle).sin();

        let i_total = self.motor_current(last_vars.dshot_cmd_left as f64 / 2047.0, self.omega_left, V_NOMINAL)
                    + self.motor_current(last_vars.dshot_cmd_right as f64 / 2047.0, self.omega_right, V_NOMINAL);
        let v_batt  = V_NOMINAL - i_total * R_INTERNAL;
        let batt_offset = ((v_batt - BATT_REF_V) / BATT_SCALE).round().clamp(-127.0, 127.0) as i8;

        SunshineInput {
            time_us:       self.time_us as u32,
            accel_x:       (ax / ADXL_SCALE).round().clamp(-32768.0, 32767.0) as i16,
            accel_y:       (ay / ADXL_SCALE).round().clamp(-32768.0, 32767.0) as i16,
            accel_z:       (az / ADXL_SCALE).round().clamp(-32768.0, 32767.0) as i16,
            mag_x:         (mx / MAG_SCALE).round().clamp(-32768.0, 32767.0) as i16,
            mag_y:         (my / MAG_SCALE).round().clamp(-32768.0, 32767.0) as i16,
            mag_z:         0,
            erpm_left:     f32_to_f16((self.omega_left  * 60.0 / (2.0 * PI)) as f32),
            erpm_right:    f32_to_f16((self.omega_right * 60.0 / (2.0 * PI)) as f32),
            batt_offset,
            ..SunshineInput::default()
        }
    }

    fn terminal_voltage(&self, cmd_l: f32, cmd_r: f32) -> f64 {
        let i_l = self.motor_current(cmd_l as f64 / 2047.0, self.omega_left,  V_NOMINAL);
        let i_r = self.motor_current(cmd_r as f64 / 2047.0, self.omega_right, V_NOMINAL);
        V_NOMINAL - (i_l + i_r) * R_INTERNAL
    }

    fn motor_current(&self, throttle: f64, omega: f64, v_term: f64) -> f64 {
        let v_motor  = throttle * v_term;
        let back_emf = omega / (KV * 2.0 * PI / 60.0);
        ((v_motor - back_emf) / R_PHASE).max(0.0)
    }

    fn motor_tick(&self, throttle: f64, omega: f64, v_term: f64, dt: f64) -> (f64, f64) {
        let current = self.motor_current(throttle, omega, v_term);
        let torque  = KT * current;
        let new_omega = (omega + (torque / (0.001 + 1e-6)) * dt).max(0.0);
        (torque, new_omega)
    }
}
