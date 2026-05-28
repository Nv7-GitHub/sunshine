use std::mem::size_of;

#[repr(C, packed)]
#[derive(Clone, Copy, Default, Debug)]
pub struct SunshineInput {
    pub time_us:       u32,
    pub accel_x:       i16,
    pub accel_y:       i16,
    pub accel_z:       i16,
    pub mag_x:         i16,
    pub mag_y:         i16,
    pub mag_z:         i16,
    pub erpm_left:     u16,
    pub erpm_right:    u16,
    pub rssi:          i8,
    pub ctrl_x:        i8,
    pub ctrl_y:        i8,
    pub ctrl_theta:    i8,
    pub ctrl_throttle: u8,
    pub batt_offset:   i8,
    pub dshot_left_q:  u8,
    pub dshot_right_q: u8,
    pub mode:          u8,
}

#[repr(C, packed)]
#[derive(Clone, Copy, Default, Debug)]
pub struct SunshineState {
    pub kf_theta:      f32,
    pub kf_omega:      f32,
    pub kf_p:          [f32; 4],
    pub theta_offset:  f32,
    pub derot_lp_i:    [f32; 4],
    pub derot_lp_q:    [f32; 4],
}

/// SunshineVars: 12 floats + 4 u8 flags = 48 + 4 = 52 bytes packed
#[repr(C, packed)]
#[derive(Clone, Copy, Default, Debug)]
pub struct SunshineVars {
    pub omega_from_accel:  f32,
    pub derot_i:           f32,
    pub derot_q:           f32,
    pub mag_angle:         f32,
    pub est_theta:         f32,
    pub est_omega:         f32,
    pub dshot_cmd_left:    f32,
    pub dshot_cmd_right:   f32,
    pub batt_voltage:      f32,
    pub erpm_left:         f32,
    pub erpm_right:        f32,
    pub centripetal_ms2:   f32,
    pub led_on:            u8,
    pub accel_saturated:   u8,
    pub mag_valid:         u8,
    pub loop_overrun:      u8,
}

const _: () = {
    assert!(size_of::<SunshineInput>() == 29, "SunshineInput size mismatch");
    assert!(size_of::<SunshineState>() == 60, "SunshineState size mismatch");
    assert!(size_of::<SunshineVars>()  == 52, "SunshineVars size mismatch");
};

extern "C" {
    fn sunshine_step(i: *const SunshineInput, s: *mut SunshineState, v: *mut SunshineVars);
    fn sunshine_state_init(s: *mut SunshineState);
    fn sunshine_schema_version() -> u32;
    fn sunshine_f16_to_f32(half: u16) -> f32;
    fn sunshine_f32_to_f16(f: f32) -> u16;
}

pub fn brain_step(input: &SunshineInput, state: &mut SunshineState) -> SunshineVars {
    let mut vars = SunshineVars::default();
    unsafe { sunshine_step(input as *const _, state as *mut _, &mut vars as *mut _) }
    vars
}

pub fn state_init(state: &mut SunshineState) {
    unsafe { sunshine_state_init(state as *mut _) }
}

pub fn schema_version() -> u32 {
    unsafe { sunshine_schema_version() }
}

pub fn f16_to_f32(half: u16) -> f32 {
    unsafe { sunshine_f16_to_f32(half) }
}

pub fn f32_to_f16(f: f32) -> u16 {
    unsafe { sunshine_f32_to_f16(f) }
}
