// src/lqr.rs

use crate::config::{
    DT_S, FALL_LIMIT_RAD, LQR_K, MASS_KG, MAX_ACCEL_MPS2, MAX_FORCE_N, MAX_SPEED_MPS,
    MOTOR_SIGN, THETA_SIGN, THETA_ZERO_RAD, TILT_ACCEL_BOOST_DEADBAND_RAD,
    TILT_ACCEL_BOOST_GAIN,
};

#[inline]
pub fn clampf(x: f32, lo: f32, hi: f32) -> f32 {
    if x < lo {
        lo
    } else if x > hi {
        hi
    } else {
        x
    }
}

pub struct LqrController {
    last_theta_rad: f32,
    theta_dot_rad_s: f32,
    x_m: f32,
    v_mps: f32,
    initialized: bool,
}

impl LqrController {
    pub const fn new() -> Self {
        Self {
            last_theta_rad: 0.0,
            theta_dot_rad_s: 0.0,
            x_m: 0.0,
            v_mps: 0.0,
            initialized: false,
        }
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    pub fn step(&mut self, measured_theta_rad: f32, v_ref_mps: f32) -> Option<f32> {
        let theta_rad = THETA_SIGN * (measured_theta_rad - THETA_ZERO_RAD);

        if theta_rad.abs() > FALL_LIMIT_RAD {
            self.reset();
            return None;
        }

        let theta_dot_raw = if self.initialized {
            (theta_rad - self.last_theta_rad) / DT_S
        } else {
            0.0
        };

        self.last_theta_rad = theta_rad;
        self.initialized = true;

        // Derivative low-pass filter.
        // Faster than the previous 0.35 value.
        // If the robot becomes noisy or oscillatory, reduce to 0.45.
        const D_ALPHA: f32 = 0.55;
        self.theta_dot_rad_s += D_ALPHA * (theta_dot_raw - self.theta_dot_rad_s);

        let e_theta = theta_rad;
        let e_theta_dot = self.theta_dot_rad_s;
        let e_x = self.x_m;
        let e_v = self.v_mps - v_ref_mps;

        // LQR law:
        //   u = -Kx
        let force_n = -(
            LQR_K[0] * e_theta
                + LQR_K[1] * e_theta_dot
                + LQR_K[2] * e_x
                + LQR_K[3] * e_v
        );

        let force_n = clampf(force_n, -MAX_FORCE_N, MAX_FORCE_N);

        let mut accel_mps2 = force_n / MASS_KG;

        // Nonlinear early-tilt acceleration boost.
        // This makes the wheels accelerate harder as soon as the body starts falling.
        let theta_abs = theta_rad.abs();

        if theta_abs > TILT_ACCEL_BOOST_DEADBAND_RAD {
            let boost = TILT_ACCEL_BOOST_GAIN
                * (theta_abs - TILT_ACCEL_BOOST_DEADBAND_RAD);

            accel_mps2 += boost * theta_rad.signum();
        }

        // Final acceleration clamp.
        // This keeps LQR + boost inside the physical acceleration limit.
        accel_mps2 = clampf(accel_mps2, -MAX_ACCEL_MPS2, MAX_ACCEL_MPS2);

        self.v_mps += accel_mps2 * DT_S;
        self.v_mps = clampf(self.v_mps, -MAX_SPEED_MPS, MAX_SPEED_MPS);

        // Without encoders, this is commanded position, not measured position.
        // Keep LQR_K[2] near zero until you add encoders.
        self.x_m += self.v_mps * DT_S;

        Some(MOTOR_SIGN * self.v_mps)
    }
}