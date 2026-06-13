use crate::ffi::{SunshineInput, SunshineVars, f32_to_f16};
use std::f64::consts::PI;

const KV:           f64 = 1100.0;
const KT:           f64 = 60.0 / (2.0*PI*KV);
const R_PHASE:      f64 = 0.075;
const V_NOMINAL:    f64 = 8.4;
const R_INTERNAL:   f64 = 0.008;
const WHEEL_RADIUS: f64 = 0.022;
const WHEEL_CENTER: f64 = 0.0405;
const MASS:         f64 = 0.454;
const MOI:          f64 = 1.214e-3;
const WHEEL_INERTIA:f64 = 6.407_440_19e-6;
const IMU_RADIUS:      f64 = 0.011;
// EARTH_FIELD is the horizontal component only — the robot spins in a horizontal plane so
// the vertical component is constant regardless of heading and creates no rotating signal.
// Total field ≈ 50 µT; at ~60° magnetic inclination (typical US), horizontal = 50×cos(60°) = 25 µT.
// Confirmed by spiritridge.sun: settled derot_I/Q magnitude ≈ 25 µT.
const EARTH_FIELD:     f64 = 25.0;          // µT, horizontal component only
const EARTH_ANGLE:     f64 = 0.0;           // Earth field azimuth (rad, arbitrary reference)
// Hard-iron: constant body-frame bias from motor magnets and PCB traces.
// The derotation LP filter rejects this (it becomes AC after derotation) but it is visible
// in raw sensor data as the large offset between sim and real. Values from spiritridge.sun.
const HARD_IRON_X:     f64 = -95.0;         // µT
const HARD_IRON_Y:     f64 =  103.0;        // µT
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
    vel_x:       f64,
    vel_y:       f64,
    omega_left:  f64,
    omega_right: f64,
    time_us:     u64,
}

const DSHOT_NEUTRAL:  f64 = 1048.0;
const DSHOT_MAX:      f64 = 2047.0;
const DSHOT_MIN:      f64 = 48.0;
const BODY_DRAG:      f64 = 0.05;  // lumped body spin drag (1/s)
const TRANSLATION_DRAG:f64 = 2.0;   // lumped rolling/carpet loss (1/s)
const TIRE_DAMPING:   f64 = 10.0;  // N per (m/s) of longitudinal slip
const MAX_TIRE_FORCE: f64 = 25.0;  // crude traction limit per wheel
const WHEEL_DRAG:     f64 = 2.0e-6;

fn dshot_to_throttle(dshot: f32) -> f64 {
    let d = dshot as f64;
    if d < DSHOT_MIN { return 0.0; }  // 0 = disarmed, 1–47 = special commands → coast
    if d >= DSHOT_NEUTRAL {
        // 1048 → 0.0 (slowest fwd), 2047 → 1.0 (fastest fwd)
        (d - DSHOT_NEUTRAL) / (DSHOT_MAX - DSHOT_NEUTRAL)
    } else {
        // 48 → 0.0 (slowest rev), 1047 → -1.0 (fastest rev)
        -((d - DSHOT_MIN) / (DSHOT_NEUTRAL - 1.0 - DSHOT_MIN))
    }
}

impl Simulation {
    pub fn new() -> Self {
        Simulation { body_omega: 0.0, body_angle: 0.0, vel_x: 0.0, vel_y: 0.0,
                     omega_left: 0.0, omega_right: 0.0, time_us: 0 }
    }

    pub fn tick(&mut self, last_vars: &SunshineVars) -> SunshineInput {
        let dt = 1e-3f64;
        self.time_us += 1000;

        let v_term = self.terminal_voltage(last_vars.dshot_cmd_left, last_vars.dshot_cmd_right);
        let torque_l = self.motor_torque(dshot_to_throttle(last_vars.dshot_cmd_left),  self.omega_left,  v_term);
        let torque_r = self.motor_torque(dshot_to_throttle(last_vars.dshot_cmd_right), self.omega_right, v_term);

        let (_radial_x, _radial_y, tangent_x, tangent_y) = self.body_axes();
        let v_tangent = self.vel_x * tangent_x + self.vel_y * tangent_y;
        let spin_surface = self.body_omega * WHEEL_CENTER;

        let contact_l = spin_surface + v_tangent;
        let contact_r = spin_surface - v_tangent;
        let slip_l = self.omega_left * WHEEL_RADIUS - contact_l;
        let slip_r = self.omega_right * WHEEL_RADIUS - contact_r;
        let force_l = (slip_l * TIRE_DAMPING).clamp(-MAX_TIRE_FORCE, MAX_TIRE_FORCE);
        let force_r = (slip_r * TIRE_DAMPING).clamp(-MAX_TIRE_FORCE, MAX_TIRE_FORCE);

        let alpha_l = (torque_l - force_l * WHEEL_RADIUS - WHEEL_DRAG * self.omega_left) / WHEEL_INERTIA;
        let alpha_r = (torque_r - force_r * WHEEL_RADIUS - WHEEL_DRAG * self.omega_right) / WHEEL_INERTIA;
        self.omega_left += alpha_l * dt;
        self.omega_right += alpha_r * dt;

        let torque_body = (force_l + force_r) * WHEEL_CENTER;
        let alpha = torque_body / MOI;
        self.body_omega += alpha * dt;
        self.body_omega *= 1.0 - BODY_DRAG * dt;
        self.body_angle += self.body_omega * dt;

        let force_translation = force_l - force_r;
        let accel_world_x = (force_translation / MASS) * tangent_x - TRANSLATION_DRAG * self.vel_x;
        let accel_world_y = (force_translation / MASS) * tangent_y - TRANSLATION_DRAG * self.vel_y;
        self.vel_x += accel_world_x * dt;
        self.vel_y += accel_world_y * dt;

        let a_centripetal = self.body_omega.powi(2) * IMU_RADIUS;
        let a_tangential  = alpha * IMU_RADIUS;
        let (radial_x, radial_y, tangent_x, tangent_y) = self.body_axes();
        let accel_radial = accel_world_x * radial_x + accel_world_y * radial_y;
        let accel_tangent = accel_world_x * tangent_x + accel_world_y * tangent_y;
        let body_x = a_centripetal + accel_radial;
        let body_y = a_tangential + accel_tangent;
        let ax = (body_x - body_y) / 2.0f64.sqrt();
        let ay = (body_x + body_y) / 2.0f64.sqrt();
        let az = 9.81f64;

        // The derotation algorithm (derot_filter.c) applies R(-θ) to [mx, my], which gives
        // DC output for Earth's field only when the sensor y-axis is negated relative to the
        // naive geometric model.  The LIS3MDL is physically mounted with its y-axis inverted,
        // so my = -E·sin(φ−θ).  Without the minus sign the derotated signal oscillates at 2ω
        // and the LP filter kills it → heading Kalman update receives atan2(0,0)=0 every tick.
        let phi_minus_theta = EARTH_ANGLE - self.body_angle;
        let mx = EARTH_FIELD * phi_minus_theta.cos() + HARD_IRON_X;
        let my = -EARTH_FIELD * phi_minus_theta.sin() + HARD_IRON_Y;

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

    fn body_axes(&self) -> (f64, f64, f64, f64) {
        let radial_x = self.body_angle.cos();
        let radial_y = self.body_angle.sin();
        let tangent_x = -radial_y;
        let tangent_y = radial_x;
        (radial_x, radial_y, tangent_x, tangent_y)
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

    #[test]
    fn tank_differential_commands_create_linear_accel() {
        let mut sim = Simulation::new();
        let cmd = SunshineVars {
            dshot_cmd_left: DSHOT_MAX as f32,
            dshot_cmd_right: DSHOT_MIN as f32,
            ..SunshineVars::default()
        };

        let mut input = SunshineInput::default();
        for _ in 0..200 {
            input = sim.tick(&cmd);
        }

        let ax = input.accel_x as f64 * ADXL_SCALE;
        let ay = input.accel_y as f64 * ADXL_SCALE;
        assert!((ax * ax + ay * ay).sqrt() > 2.0);
        assert!(sim.vel_x.abs() > 0.01 || sim.vel_y.abs() > 0.01);
    }

    #[test]
    fn melty_differential_commands_change_wheel_erpms_independently() {
        let mut sim = Simulation::new();
        let cmd = SunshineVars {
            dshot_cmd_left: DSHOT_MAX as f32,
            dshot_cmd_right: DSHOT_NEUTRAL as f32,
            ..SunshineVars::default()
        };

        let mut input = SunshineInput::default();
        for _ in 0..200 {
            input = sim.tick(&cmd);
        }

        let erpm_left = f16_to_f32(input.erpm_left);
        let erpm_right = f16_to_f32(input.erpm_right);
        assert!((erpm_left - erpm_right).abs() > 100.0);
    }
}
