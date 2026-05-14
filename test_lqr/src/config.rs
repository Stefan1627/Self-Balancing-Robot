// src/config.rs

pub const CONTROL_PERIOD_MS: u64 = 5;
pub const DT_S: f32 = 0.005;

// TODO: measure total assembled robot mass.
pub const MASS_KG: f32 = 1.20;

// 80 mm wheel diameter -> 40 mm radius.
pub const WHEEL_RADIUS_M: f32 = 0.040;

// Most NEMA17 steppers are 1.8 deg/step.
// That means 360 / 1.8 = 200 full steps/rev.
pub const FULL_STEPS_PER_REV: f32 = 200.0;

// You confirmed DRV8825 is set to 1/16 microstepping.
pub const MICROSTEPS: f32 = 16.0;

// Start conservative.
pub const MAX_SPEED_MPS: f32 = 0.25;
pub const MAX_ACCEL_MPS2: f32 = 0.6;
pub const MAX_FORCE_N: f32 = MASS_KG * MAX_ACCEL_MPS2;

pub const FALL_LIMIT_RAD: f32 = 0.50;

pub const MIN_STEP_HZ: u32 = 2;
pub const MAX_STEP_HZ: u32 = 12_000;

// Set after measuring pitch while robot is physically upright.
pub const THETA_ZERO_RAD: f32 = 0.0;

pub const THETA_SIGN: f32 = 1.0;
pub const MOTOR_SIGN: f32 = 1.0;

// Replace with output from tools/lqr_gain.py.
pub const LQR_K: [f32; 4] = [
    0.0,
    0.0,
    0.0,
    0.0,
];