// src/config.rs

pub const CONTROL_PERIOD_MS: u64 = 5;
pub const DT_S: f32 = 0.005;

// TODO: measure total assembled robot mass.
pub const MASS_KG: f32 = 1.35;

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

pub const FALL_LIMIT_RAD: f32 = 0.70;

pub const MIN_STEP_HZ: u32 = 50;
pub const MAX_STEP_HZ: u32 = 12_000;

// Maximum step-frequency increase per control tick (5 ms).
// test_full uses 200 Hz per 10 ms; this is equivalent.
// Prevents stepper stalls from sudden frequency jumps.
pub const ACCEL_STEP_HZ: u32 = 100;

// Set after measuring pitch while robot is physically upright.
pub const THETA_ZERO_RAD: f32 = 0.0;

pub const THETA_SIGN: f32 = 1.0;
pub const MOTOR_SIGN: f32 = 1.0;

// --- Motor direction inversion ---
// If pressing Forward makes the robot go BACKWARD, flip MOTOR_SIGN to -1.0.
// If one wheel spins the wrong way relative to the other, set the corresponding
// flag to `true`. This is common when a motor is physically mounted mirrored.
pub const INVERT_LEFT_MOTOR: bool = false;
pub const INVERT_RIGHT_MOTOR: bool = false;

// How often the telemetry loop sends a line to the phone (milliseconds).
pub const TELEMETRY_PERIOD_MS: u64 = 100;

// Replace with output from tools/lqr_gain.py.
pub const LQR_K: [f32; 4] = [
    -3.0698523f32,
    -0.2540903f32,
    0.00646826f32,
    0.64917966f32,
];
