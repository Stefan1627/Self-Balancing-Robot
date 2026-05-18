// src/telemetry.rs

use core::fmt::Write;
use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, Ordering};

// ---------------------------------------------------------------------------
// Shared telemetry state: written by control loop, read by UART loop.
// ---------------------------------------------------------------------------

pub static TELEM_YAW_BITS: AtomicU32 = AtomicU32::new(0);
pub static TELEM_PITCH_BITS: AtomicU32 = AtomicU32::new(0);
pub static TELEM_ROLL_BITS: AtomicU32 = AtomicU32::new(0);

pub static TELEM_AX: AtomicI32 = AtomicI32::new(0);
pub static TELEM_AY: AtomicI32 = AtomicI32::new(0);
pub static TELEM_AZ: AtomicI32 = AtomicI32::new(0);

pub static TELEM_V_CMD_BITS: AtomicU32 = AtomicU32::new(0);
pub static TELEM_V_REF_BITS: AtomicU32 = AtomicU32::new(0);

pub static TELEM_BALANCED: AtomicBool = AtomicBool::new(false);

#[allow(clippy::too_many_arguments)]
pub fn update_telem(
    yaw_rad: f32,
    pitch_rad: f32,
    roll_rad: f32,
    ax: i16,
    ay: i16,
    az: i16,
    v_cmd_mps: f32,
    v_ref_mps: f32,
    balanced: bool,
) {
    TELEM_YAW_BITS.store(yaw_rad.to_bits(), Ordering::Relaxed);
    TELEM_PITCH_BITS.store(pitch_rad.to_bits(), Ordering::Relaxed);
    TELEM_ROLL_BITS.store(roll_rad.to_bits(), Ordering::Relaxed);

    TELEM_AX.store(ax as i32, Ordering::Relaxed);
    TELEM_AY.store(ay as i32, Ordering::Relaxed);
    TELEM_AZ.store(az as i32, Ordering::Relaxed);

    TELEM_V_CMD_BITS.store(v_cmd_mps.to_bits(), Ordering::Relaxed);
    TELEM_V_REF_BITS.store(v_ref_mps.to_bits(), Ordering::Relaxed);

    TELEM_BALANCED.store(balanced, Ordering::Relaxed);
}

pub struct Telemetry {
    pub yaw_rad: f32,
    pub pitch_rad: f32,
    pub roll_rad: f32,
    pub ax: i32,
    pub ay: i32,
    pub az: i32,
    pub v_cmd_mps: f32,
    pub v_ref_mps: f32,
    pub balanced: bool,
}

pub fn read_telem() -> Telemetry {
    Telemetry {
        yaw_rad: f32::from_bits(TELEM_YAW_BITS.load(Ordering::Relaxed)),
        pitch_rad: f32::from_bits(TELEM_PITCH_BITS.load(Ordering::Relaxed)),
        roll_rad: f32::from_bits(TELEM_ROLL_BITS.load(Ordering::Relaxed)),

        ax: TELEM_AX.load(Ordering::Relaxed),
        ay: TELEM_AY.load(Ordering::Relaxed),
        az: TELEM_AZ.load(Ordering::Relaxed),

        v_cmd_mps: f32::from_bits(TELEM_V_CMD_BITS.load(Ordering::Relaxed)),
        v_ref_mps: f32::from_bits(TELEM_V_REF_BITS.load(Ordering::Relaxed)),

        balanced: TELEM_BALANCED.load(Ordering::Relaxed),
    }
}

// ---------------------------------------------------------------------------
// UART string buffer.
// ---------------------------------------------------------------------------

pub struct UartBuf {
    buf: [u8; 128],
    len: usize,
}

impl UartBuf {
    pub const fn new() -> Self {
        Self {
            buf: [0; 128],
            len: 0,
        }
    }

    pub fn as_slice(&self) -> &[u8] {
        &self.buf[..self.len]
    }

    pub fn clear(&mut self) {
        self.len = 0;
    }
}

impl Write for UartBuf {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let remaining = self.buf.len() - self.len;

        if bytes.len() > remaining {
            return Err(core::fmt::Error);
        }

        self.buf[self.len..self.len + bytes.len()].copy_from_slice(bytes);
        self.len += bytes.len();

        Ok(())
    }
}