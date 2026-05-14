// src/commands.rs

use core::sync::atomic::{AtomicBool, AtomicI32, Ordering};

pub static BALANCE_ENABLE: AtomicBool = AtomicBool::new(false);

// Desired forward velocity in mm/s.
pub static V_REF_MM_S: AtomicI32 = AtomicI32::new(0);

pub fn handle_bluetooth_byte(byte: u8) {
    match byte {
        b'E' => {
            BALANCE_ENABLE.store(true, Ordering::Relaxed);
        }
        b'X' => {
            BALANCE_ENABLE.store(false, Ordering::Relaxed);
            V_REF_MM_S.store(0, Ordering::Relaxed);
        }
        b'F' => {
            BALANCE_ENABLE.store(true, Ordering::Relaxed);
            V_REF_MM_S.store(120, Ordering::Relaxed);
        }
        b'B' => {
            BALANCE_ENABLE.store(true, Ordering::Relaxed);
            V_REF_MM_S.store(-120, Ordering::Relaxed);
        }
        b'S' => {
            // Stop translation, but keep balancing.
            V_REF_MM_S.store(0, Ordering::Relaxed);
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

pub fn disable_balance() {
    BALANCE_ENABLE.store(false, Ordering::Relaxed);
    V_REF_MM_S.store(0, Ordering::Relaxed);
}