// src/config.rs

pub const CONTROL_PERIOD_MS: u64 = 5;
pub const DT_S: f32 = 0.005;

// Total assembled robot mass.
pub const MASS_KG: f32 = 1.35;

// 80 mm wheel diameter -> 40 mm radius.
pub const WHEEL_RADIUS_M: f32 = 0.040;

// Most NEMA17 steppers are 1.8 deg/step.
// That means 360 / 1.8 = 200 full steps/rev.
pub const FULL_STEPS_PER_REV: f32 = 200.0;

// DRV8825 currently configured for 1/8 microstepping.
pub const MICROSTEPS: f32 = 8.0;

// Aggressive limits.
// If the motors buzz or skip steps, reduce MAX_ACCEL_MPS2 first.
pub const MAX_SPEED_MPS: f32 = 0.90;
pub const MAX_ACCEL_MPS2: f32 = 6.0;
pub const MAX_FORCE_N: f32 = MASS_KG * MAX_ACCEL_MPS2;

pub const FALL_LIMIT_RAD: f32 = 0.70;

pub const MIN_STEP_HZ: u32 = 20;
pub const MAX_STEP_HZ: u32 = 12_000;

// Maximum step-frequency increase per control tick.
// Control tick = 5 ms.
// Higher value gives faster wheel acceleration, but can cause stepper stalls.
pub const ACCEL_STEP_HZ: u32 = 1000;

// Set after measuring the balancing angle while robot is physically upright.
pub const THETA_ZERO_RAD: f32 = 0.0;

pub const THETA_SIGN: f32 = 1.0;
pub const MOTOR_SIGN: f32 = 1.0;

// Extra acceleration added when the robot begins to tilt.
// This makes the first response more aggressive.
pub const TILT_ACCEL_BOOST_GAIN: f32 = 30.0;

// Ignore very tiny tilt to avoid jitter around upright.
pub const TILT_ACCEL_BOOST_DEADBAND_RAD: f32 = 0.015;

// Motor direction inversion.
// If one wheel spins the wrong way relative to the other, flip its flag.
pub const INVERT_LEFT_MOTOR: bool = false;
pub const INVERT_RIGHT_MOTOR: bool = false;

// How often the telemetry loop sends a line to the phone.
pub const TELEMETRY_PERIOD_MS: u64 = 100;

// Replace with output from tools/lqr_gain.py when you retune.
// Current gains are your aggressive gain set.
pub const LQR_K: [f32; 4] = [
    -20.260053f32,
    -1.3820483f32,
    0.0010499381f32,
    1.6614389f32,
];
