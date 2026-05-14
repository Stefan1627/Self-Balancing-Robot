// src/motor.rs

use crate::config::{
    FULL_STEPS_PER_REV, MAX_STEP_HZ, MICROSTEPS, MIN_STEP_HZ, WHEEL_RADIUS_M,
};
use crate::lqr::clampf;

pub enum MotorCommand {
    StopPulses,
    Run {
        forward: bool,
        step_hz: u32,
    },
}

pub fn velocity_to_motor_command(v_mps: f32) -> MotorCommand {
    let wheel_circumference_m = 2.0 * core::f32::consts::PI * WHEEL_RADIUS_M;
    let rev_per_s = v_mps.abs() / wheel_circumference_m;
    let step_hz_f = rev_per_s * FULL_STEPS_PER_REV * MICROSTEPS;

    let step_hz = clampf(step_hz_f, 0.0, MAX_STEP_HZ as f32) as u32;

    if step_hz < MIN_STEP_HZ {
        MotorCommand::StopPulses
    } else {
        MotorCommand::Run {
            forward: v_mps >= 0.0,
            step_hz,
        }
    }
}