// src/commands.rs

use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU8, Ordering};

pub static BALANCE_ENABLE: AtomicBool = AtomicBool::new(false);

// Desired forward velocity in mm/s, used by LQR when balancing.
pub static V_REF_MM_S: AtomicI32 = AtomicI32::new(0);

// Direct-drive motor command, used when balance is OFF.
// 0 = stop, 1 = forward, 2 = backward.
pub static MOTOR_CMD: AtomicU8 = AtomicU8::new(0);

pub fn handle_bluetooth_byte(byte: u8) {
    match byte {
        b'E' => {
            // Enable balance mode and clear direct-drive command.
            MOTOR_CMD.store(0, Ordering::Relaxed);
            BALANCE_ENABLE.store(true, Ordering::Relaxed);
        }
        b'X' => {
            // E-stop: disable balance and stop everything.
            BALANCE_ENABLE.store(false, Ordering::Relaxed);
            V_REF_MM_S.store(0, Ordering::Relaxed);
            MOTOR_CMD.store(0, Ordering::Relaxed);
        }
        b'F' => {
            if BALANCE_ENABLE.load(Ordering::Relaxed) {
                // Balanced mode: set velocity reference for LQR.
                V_REF_MM_S.store(120, Ordering::Relaxed);
            } else {
                // Direct-drive mode.
                MOTOR_CMD.store(1, Ordering::Relaxed);
            }
        }
        b'B' => {
            if BALANCE_ENABLE.load(Ordering::Relaxed) {
                // Balanced mode: set velocity reference for LQR.
                V_REF_MM_S.store(-120, Ordering::Relaxed);
            } else {
                // Direct-drive mode.
                MOTOR_CMD.store(2, Ordering::Relaxed);
            }
        }
        b'S' => {
            if BALANCE_ENABLE.load(Ordering::Relaxed) {
                // Balanced mode: zero velocity reference, keep balancing.
                V_REF_MM_S.store(0, Ordering::Relaxed);
            } else {
                // Direct-drive mode: stop motors.
                MOTOR_CMD.store(0, Ordering::Relaxed);
            }
        }
        _ => {}
    }
}

pub fn balance_enabled() -> bool {
    BALANCE_ENABLE.load(Ordering::Relaxed)
}

pub fn velocity_reference_mps() -> f32 {
    V_REF_MM_S.load(Ordering::Relaxed) as f32 * 0.001
}

pub fn motor_cmd() -> u8 {
    MOTOR_CMD.load(Ordering::Relaxed)
}

pub fn disable_balance() {
    BALANCE_ENABLE.store(false, Ordering::Relaxed);
    V_REF_MM_S.store(0, Ordering::Relaxed);
    MOTOR_CMD.store(0, Ordering::Relaxed);
}