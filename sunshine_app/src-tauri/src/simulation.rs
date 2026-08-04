use crate::ffi::{SunshineInput, SunshineVars, f32_to_f16};
use std::f64::consts::PI;

// ALL physical/plant constants come from sunshine_core/include/sunshine_core.h —
// build.rs parses the header and generates this file, so there is no second copy
// to keep in sync. New robot => edit the header only (see BRINGUP.md "Porting to
// a New Robot"). Rationale for each value lives next to its #define.
#[allow(dead_code)]
mod core_consts {
    include!(concat!(env!("OUT_DIR"), "/core_constants.rs"));
}
use core_consts::*;

// Torque constant, derived from the nameplate KV (N·m per A).
const KT: f64 = 60.0 / (2.0 * PI * MOTOR_KV_RPM_PER_V);

pub struct Simulation {
    body_omega:  f64,
    body_angle:  f64,
    vel_x:       f64,
    vel_y:       f64,
    omega_left:  f64,
    omega_right: f64,
    time_us:     u64,
}

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
        let spin_surface = self.body_omega * WHEEL_CENTER_M;

        let contact_l = spin_surface + v_tangent;
        let contact_r = spin_surface - v_tangent;
        let slip_l = self.omega_left * WHEEL_RADIUS_M - contact_l;
        let slip_r = self.omega_right * WHEEL_RADIUS_M - contact_r;
        let force_l = (slip_l * SIM_TIRE_DAMPING).clamp(-SIM_MAX_TIRE_FORCE, SIM_MAX_TIRE_FORCE);
        let force_r = (slip_r * SIM_TIRE_DAMPING).clamp(-SIM_MAX_TIRE_FORCE, SIM_MAX_TIRE_FORCE);

        let alpha_l = (torque_l - force_l * WHEEL_RADIUS_M - SIM_WHEEL_DRAG * self.omega_left) / WHEEL_INERTIA_KGM2;
        let alpha_r = (torque_r - force_r * WHEEL_RADIUS_M - SIM_WHEEL_DRAG * self.omega_right) / WHEEL_INERTIA_KGM2;
        self.omega_left += alpha_l * dt;
        self.omega_right += alpha_r * dt;

        let torque_body = (force_l + force_r) * WHEEL_CENTER_M;
        let alpha = torque_body / ROBOT_MOI_KGM2;
        self.body_omega += alpha * dt;
        self.body_omega *= 1.0 - SIM_BODY_DRAG * dt;
        self.body_angle += self.body_omega * dt;

        let force_translation = force_l - force_r;
        let accel_world_x = (force_translation / ROBOT_MASS_KG) * tangent_x - SIM_TRANSLATION_DRAG * self.vel_x;
        let accel_world_y = (force_translation / ROBOT_MASS_KG) * tangent_y - SIM_TRANSLATION_DRAG * self.vel_y;
        self.vel_x += accel_world_x * dt;
        self.vel_y += accel_world_y * dt;

        let a_centripetal = self.body_omega.powi(2) * IMU_RADIUS_M;
        let a_tangential  = alpha * IMU_RADIUS_M;
        let (radial_x, radial_y, tangent_x, tangent_y) = self.body_axes();
        let accel_radial = accel_world_x * radial_x + accel_world_y * radial_y;
        let accel_tangent = accel_world_x * tangent_x + accel_world_y * tangent_y;
        let body_x = a_centripetal + accel_radial;
        let body_y = a_tangential + accel_tangent;
        let ax = (body_x - body_y) / 2.0f64.sqrt();
        let ay = (body_x + body_y) / 2.0f64.sqrt();
        let az = 9.81f64;

        // The open-loop heading (mag_heading.c) takes atan2(-my_hp, mx_hp). For that to give
        // the true heading, the sensor y-axis must be negated relative to the naive geometric
        // model. The LIS3MDL is physically mounted with its y-axis inverted, so we generate
        // my = -E·sin(φ−θ) here to match the real hardware.
        let phi_minus_theta = EARTH_ANGLE_RAD - self.body_angle;
        // LP-mode HF sampling artifact (fs/6 tone); see consts above.
        let hf = (2.0 * PI * MAG_HF_TONE_HZ * (self.time_us as f64 * 1e-6)).sin();
        let mx = EARTH_FIELD_UT * phi_minus_theta.cos() + HARD_IRON_X_UT + MAG_HF_TONE_X_UT * hf;
        let my = -EARTH_FIELD_UT * phi_minus_theta.sin() + HARD_IRON_Y_UT + MAG_HF_TONE_Y_UT * hf;

        let i_total = self.supply_current(last_vars.dshot_cmd_left, last_vars.dshot_cmd_right, BATT_NOMINAL_V);
        let v_batt  = BATT_NOMINAL_V - i_total * BATT_R_INTERNAL_OHM;
        let batt_offset = ((v_batt - BATT_OFFSET_REF_V) / BATT_SCALE_V).round().clamp(-32768.0, 32767.0) as i16;

        SunshineInput {
            time_us:       self.time_us as u32,
            accel_x:       (ax / ADXL_SCALE_MS2).round().clamp(-ADXL_MAX_COUNTS, ADXL_MAX_COUNTS) as i16,
            accel_y:       (ay / ADXL_SCALE_MS2).round().clamp(-ADXL_MAX_COUNTS, ADXL_MAX_COUNTS) as i16,
            accel_z:       (az / ADXL_SCALE_MS2).round().clamp(-32768.0, 32767.0) as i16,
            mag_x:         (mx / MAG_SCALE_UT).round().clamp(-32768.0, 32767.0) as i16,
            mag_y:         (my / MAG_SCALE_UT).round().clamp(-32768.0, 32767.0) as i16,
            mag_z:         0,
            erpm_left:     f32_to_f16((self.omega_left  * 60.0 / (2.0 * PI) * MOTOR_POLE_PAIRS) as f32),
            erpm_right:    f32_to_f16((self.omega_right * 60.0 / (2.0 * PI) * MOTOR_POLE_PAIRS) as f32),
            batt_offset,
            ..SunshineInput::default()
        }
    }

    fn terminal_voltage(&self, cmd_l: f32, cmd_r: f32) -> f64 {
        BATT_NOMINAL_V - self.supply_current(cmd_l, cmd_r, BATT_NOMINAL_V) * BATT_R_INTERNAL_OHM
    }

    fn supply_current(&self, cmd_l: f32, cmd_r: f32, v_term: f64) -> f64 {
        let i_l = self.motor_current(dshot_to_throttle(cmd_l), self.omega_left,  v_term);
        let i_r = self.motor_current(dshot_to_throttle(cmd_r), self.omega_right, v_term);
        (i_l + i_r).max(0.0)
    }

    fn motor_current(&self, throttle: f64, omega: f64, v_term: f64) -> f64 {
        let v_motor  = throttle * v_term;
        let back_emf = omega / (MOTOR_KV_RPM_PER_V * 2.0 * PI / 60.0);
        // Allow negative current: back-EMF brakes the motor when throttle is low
        (v_motor - back_emf) / MOTOR_R_PHASE_OHM
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

        let ax = input.accel_x as f64 * ADXL_SCALE_MS2;
        let ay = input.accel_y as f64 * ADXL_SCALE_MS2;
        let centripetal = (ax * ax + ay * ay).sqrt();
        let erpm_left = f16_to_f32(input.erpm_left);

        assert!(erpm_left > 55_000.0);
        assert!(centripetal > 280.0 * 9.81);
        assert!(input.accel_x.abs() >= ADXL_MAX_COUNTS as i16 ||
                input.accel_y.abs() >= ADXL_MAX_COUNTS as i16);
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

        let ax = input.accel_x as f64 * ADXL_SCALE_MS2;
        let ay = input.accel_y as f64 * ADXL_SCALE_MS2;
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
