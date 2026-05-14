Below is the concrete project refactor I would do. The goal is to move from your current **open-loop motor ramp** to a **fixed-rate local balance controller** running on the STM32.

The article’s runtime implementation is simple: compute the LQR gains offline, then in the embedded app build an error vector from measured states and take the dot product with the constant gain vector. It specifically says Python is used only to generate matrices/gains, while the compiled Rust app only uses those gains at runtime. 

---

# 1. Files to create

Create this structure:

```text
project-root/
├── Cargo.toml
├── src/
│   ├── main.rs
│   ├── config.rs
│   ├── lqr.rs
│   ├── motor.rs
│   ├── commands.rs
│   └── telemetry.rs          optional
└── tools/
    └── lqr_gain.py
```

Minimum required new files:

```text
src/config.rs
src/lqr.rs
src/motor.rs
src/commands.rs
tools/lqr_gain.py
```

`telemetry.rs` is optional, but I strongly recommend separating it so you do not accidentally block the balance loop with UART formatting.

---

# 2. Modify `Cargo.toml`

You can keep your existing dependencies. Add `micromath` only if you want lightweight embedded math helpers. For the first version, you can avoid it.

Recommended `Cargo.toml` change:

```toml
[dependencies]
embassy-stm32 = { version = "0.4.0", features = ["defmt", "stm32u545re", "unstable-pac", "memory-x", "time-driver-tim4", "exti", "chrono"] }
embassy-embedded-hal = { version = "0.5.0", features = ["defmt"] }
embassy-sync = { version = "0.7.2", features = ["defmt"] }
embassy-executor = { version = "0.9.0", features = ["arch-cortex-m", "executor-thread", "defmt"] }
embassy-futures = "0.1.2"
embassy-time = { version = "0.5.0", features = ["defmt", "defmt-timestamp-uptime", "tick-hz-32_768"] }

defmt = "1.0.1"
defmt-rtt = "1.1.0"

cortex-m = { version = "0.7.7", features = ["inline-asm", "critical-section-single-core"] }
cortex-m-rt = "0.7.5"
panic-probe = { version = "1.0.0", features = ["print-defmt"] }

mpu6050-dmp = "0.6.1"
libm = "0.2"
embedded-hal = "0.2.7"
```

No mandatory new Rust dependency is needed.

For the offline Python script, install these on your PC:

```bash
pip install numpy scipy
```

---

# 3. Create `tools/lqr_gain.py`

This file is **not** for the STM32. It runs on your laptop and prints the gain vector that you paste into `src/config.rs`.

The article uses Python/control analysis to produce the gain vector, then the Rust runtime only consumes the constants. 

Create:

```python
# tools/lqr_gain.py

import numpy as np
from scipy.signal import cont2discrete
from scipy.linalg import solve_discrete_are

# ============================================================
# MEASURE THESE ON YOUR REAL ROBOT
# ============================================================

g = 9.81

# Total robot mass in kg.
m = 1.20

# Distance from wheel axle to center of gravity in meters.
# Measure with battery and all electronics mounted.
l = 0.080

# Firmware control loop period.
dt = 0.005  # 200 Hz

# ============================================================
# STATE MODEL
#
# State vector:
#   x = [theta, theta_dot, position, velocity]
#
# theta:
#   pitch angle from vertical, radians
#
# input:
#   approximate horizontal force / effort
# ============================================================

A = np.array([
    [0.0,   1.0, 0.0, 0.0],
    [g/l,   0.0, 0.0, 0.0],
    [0.0,   0.0, 0.0, 1.0],
    [-g,    0.0, 0.0, 0.0],
])

B = np.array([
    [0.0],
    [-1.0 / (m * l * l)],
    [0.0],
    [1.0 / m],
])

# Penalize states.
# Increase Q[0,0] for stronger pitch correction.
# Increase Q[1,1] for more pitch-rate damping.
# Keep Q[2,2] near zero until you have wheel encoders.
# Increase Q[3,3] for tighter velocity control.
Q = np.diag([
    2.0,    # theta
    0.05,   # theta_dot
    0.0,    # position
    1.0,    # velocity
])

# Penalize control effort.
# Higher R = softer, less aggressive.
# Lower R = more aggressive.
R = np.array([[2.0]])

C = np.eye(4)
D = np.zeros((4, 1))

Ad, Bd, _, _, _ = cont2discrete((A, B, C, D), dt)

P = solve_discrete_are(Ad, Bd, Q, R)
K = np.linalg.inv(Bd.T @ P @ Bd + R) @ (Bd.T @ P @ Ad)

print("Paste this into src/config.rs:")
print("pub const LQR_K: [f32; 4] = [")
for v in K.ravel():
    print(f"    {float(v):.8}f32,")
print("];")
```

Run:

```bash
python tools/lqr_gain.py
```

Then paste the printed constants into `src/config.rs`.

---

# 4. Create `src/config.rs`

This file centralizes all constants you will tune.

```rust
// src/config.rs

// ============================================================
// Control timing
// ============================================================

pub const CONTROL_PERIOD_MS: u64 = 5;
pub const DT_S: f32 = 0.005;

// ============================================================
// Robot physical parameters
// ============================================================

// TODO: measure your robot.
pub const MASS_KG: f32 = 1.20;

// TODO: measure wheel radius.
pub const WHEEL_RADIUS_M: f32 = 0.035;

// Usually 200 for 1.8 degree steppers.
pub const FULL_STEPS_PER_REV: f32 = 200.0;

// Must match your DRV8825 MODE pin wiring.
// If MODE0/MODE1/MODE2 are floating, DRV8825 defaults to full-step.
// Set this to 1.0 unless you actually wired microstepping.
pub const MICROSTEPS: f32 = 1.0;

// ============================================================
// Safety and command limits
// ============================================================

pub const MAX_SPEED_MPS: f32 = 0.35;
pub const MAX_ACCEL_MPS2: f32 = 1.0;
pub const MAX_FORCE_N: f32 = MASS_KG * MAX_ACCEL_MPS2;

pub const FALL_LIMIT_RAD: f32 = 0.50; // about 28.6 degrees

pub const MIN_STEP_HZ: u32 = 2;
pub const MAX_STEP_HZ: u32 = 12_000;

// ============================================================
// Calibration
// ============================================================

// Set this after measuring pitch when robot is physically upright.
pub const THETA_ZERO_RAD: f32 = 0.0;

// Flip one of these during sign testing if needed.
pub const THETA_SIGN: f32 = 1.0;
pub const MOTOR_SIGN: f32 = 1.0;

// ============================================================
// LQR gains
//
// Replace these with output from tools/lqr_gain.py.
// State order:
//   [theta, theta_dot, x, x_dot]
// ============================================================

pub const LQR_K: [f32; 4] = [
    0.0,
    0.0,
    0.0,
    0.0,
];
```

Important: leave `LQR_K` as zero only to compile. The robot will not balance until you paste real gains.

---

# 5. Create `src/lqr.rs`

This is the embedded controller. It does no allocation and uses only `f32`.

```rust
// src/lqr.rs

use crate::config::{
    DT_S, FALL_LIMIT_RAD, LQR_K, MASS_KG, MAX_FORCE_N, MAX_SPEED_MPS, MOTOR_SIGN, THETA_SIGN,
    THETA_ZERO_RAD,
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
        // Increase alpha for faster derivative, decrease for smoother derivative.
        const D_ALPHA: f32 = 0.35;
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

        let accel_mps2 = force_n / MASS_KG;

        self.v_mps += accel_mps2 * DT_S;
        self.v_mps = clampf(self.v_mps, -MAX_SPEED_MPS, MAX_SPEED_MPS);

        // Without encoders, this is commanded position, not measured position.
        // Keep LQR_K[2] near zero until you add encoders.
        self.x_m += self.v_mps * DT_S;

        Some(MOTOR_SIGN * self.v_mps)
    }
}
```

---

# 6. Create `src/motor.rs`

This handles velocity-to-step-frequency conversion.

```rust
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
```

---

# 7. Create `src/commands.rs`

This replaces your current `MOTOR_CMD`.

```rust
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
```

Phone command convention:

```text
E = enable balance
X = emergency stop / disable balance
F = forward
B = backward
S = zero velocity, keep balancing
```

This is different from your current code where `S` disables the motors. For a balancing robot, `S` should usually mean “stand still,” not “turn off.”

---

# 8. Optional: create `src/telemetry.rs`

Keep telemetry short and slow.

```rust
// src/telemetry.rs

use core::fmt::Write;

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
```

---

# 9. Modify `src/main.rs`

## 9.1 Add modules at the top

At the top of `main.rs`, add:

```rust
mod commands;
mod config;
mod lqr;
mod motor;
mod telemetry;
```

## 9.2 Replace imports

Remove this:

```rust
use core::sync::atomic::{AtomicU8, Ordering};
```

Replace with:

```rust
use core::sync::atomic::Ordering;
```

Then add:

```rust
use embassy_futures::join::join;
use embassy_time::{Delay, Duration, Ticker, Timer};

use commands::{
    balance_enabled, disable_balance, handle_bluetooth_byte, velocity_reference_mps,
};
use lqr::LqrController;
use motor::{velocity_to_motor_command, MotorCommand};
```

You currently import:

```rust
use embassy_futures::join::join3;
use embassy_time::{Delay, Duration, Timer};
```

Replace with:

```rust
use embassy_futures::join::join;
use embassy_time::{Delay, Duration, Ticker, Timer};
```

## 9.3 Delete the old global `MOTOR_CMD`

Delete:

```rust
static MOTOR_CMD: AtomicU8 = AtomicU8::new(0);
```

Command state is now in `src/commands.rs`.

## 9.4 Delete the local `UartBuf` from `main.rs`

If you created `telemetry.rs`, delete this whole block from `main.rs`:

```rust
struct UartBuf {
    buf: [u8; 128],
    len: usize,
}

impl UartBuf {
    fn new() -> Self { Self { buf: [0; 128], len: 0 } }
    fn as_slice(&self) -> &[u8] { &self.buf[..self.len] }
    fn clear(&mut self) { self.len = 0; }
}

impl core::fmt::Write for UartBuf {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let remaining = self.buf.len() - self.len;
        if bytes.len() > remaining { return Err(core::fmt::Error); }
        self.buf[self.len..self.len + bytes.len()].copy_from_slice(bytes);
        self.len += bytes.len();
        Ok(())
    }
}
```

## 9.5 Keep peripheral initialization mostly unchanged

Keep:

```rust
let mut en_motors = Output::new(p.PA4, Level::High, Speed::Low);
let mut dir_left = Output::new(p.PA1, Level::Low, Speed::VeryHigh);
let mut dir_right = Output::new(p.PA3, Level::Low, Speed::VeryHigh);
```

Keep your PWM setup:

```rust
let ch2 = PwmPin::new(p.PB3, embassy_stm32::gpio::OutputType::PushPull);
let ch3 = PwmPin::new(p.PA2, embassy_stm32::gpio::OutputType::PushPull);

let mut pwm = SimplePwm::new(
    p.TIM2,
    None,
    Some(ch2),
    Some(ch3),
    None,
    Hertz(2000),
    Default::default(),
);
```

Keep UART and I2C initialization.

## 9.6 Replace the three loops

Delete these entire blocks:

```rust
let motor_loop = async { ... };
let receiver_loop = async { ... };
let telemetry_loop = async { ... };

join3(motor_loop, receiver_loop, telemetry_loop).await;
```

Replace them with this:

```rust
let receiver_loop = async {
    let mut buf = [0u8; 1];

    loop {
        if rx.read(&mut buf).await.is_ok() {
            handle_bluetooth_byte(buf[0]);
        }
    }
};

let control_loop = async {
    let mut ticker = Ticker::every(Duration::from_millis(config::CONTROL_PERIOD_MS));
    let mut lqr = LqrController::new();

    loop {
        ticker.next().await;

        if !balance_enabled() {
            pwm.disable(Channel::Ch2);
            pwm.disable(Channel::Ch3);
            en_motors.set_high();
            lqr.reset();
            continue;
        }

        let mut latest_pitch_rad: Option<f32> = None;

        if let Ok(mut len) = sensor.get_fifo_count() {
            let mut fifo_buf = [0u8; 28];

            while len >= 28 {
                if sensor.read_fifo(&mut fifo_buf).is_ok() {
                    if let Some(quat) = Quaternion::from_bytes(&fifo_buf[..16]) {
                        let quat = quat.normalize();
                        let ypr = YawPitchRoll::from(quat);

                        // Verify the mpu6050-dmp crate unit.
                        // If ypr.pitch is already radians, remove this conversion.
                        const DEG_TO_RAD: f32 = core::f32::consts::PI / 180.0;
                        latest_pitch_rad = Some((ypr.pitch as f32) * DEG_TO_RAD);
                    }
                }

                len -= 28;
            }
        }

        let Some(theta_rad) = latest_pitch_rad else {
            continue;
        };

        let v_ref_mps = velocity_reference_mps();

        let Some(v_cmd_mps) = lqr.step(theta_rad, v_ref_mps) else {
            pwm.disable(Channel::Ch2);
            pwm.disable(Channel::Ch3);
            en_motors.set_high();
            disable_balance();
            continue;
        };

        match velocity_to_motor_command(v_cmd_mps) {
            MotorCommand::StopPulses => {
                // Keep drivers enabled for holding torque, but stop motion.
                en_motors.set_low();
                pwm.disable(Channel::Ch2);
                pwm.disable(Channel::Ch3);
            }
            MotorCommand::Run { forward, step_hz } => {
                en_motors.set_low();

                if forward {
                    dir_left.set_high();
                    dir_right.set_low();
                } else {
                    dir_left.set_low();
                    dir_right.set_high();
                }

                embassy_time::block_for(Duration::from_micros(2));

                pwm.set_frequency(Hertz(step_hz));

                let max_duty = pwm.get_max_duty();
                pwm.set_duty(Channel::Ch2, max_duty / 2);
                pwm.set_duty(Channel::Ch3, max_duty / 2);

                pwm.enable(Channel::Ch2);
                pwm.enable(Channel::Ch3);
            }
        }
    }
};

join(control_loop, receiver_loop).await;
```

---

# 10. Two important modifications before testing

## 10.1 Confirm the pitch unit

You currently print:

```rust
ypr.pitch
```

Before using the LQR loop, confirm whether `YawPitchRoll::from(quat)` returns degrees or radians.

Your code currently prints values as human-readable angles, but the LQR model must use radians. The article also explicitly mentions converting the controller inputs to meters/s and radians before feeding them into the LQR implementation. 

Test with the board tilted about 90 degrees:

```text
If printed pitch ≈ 90.0  -> degrees, keep DEG_TO_RAD.
If printed pitch ≈ 1.57  -> radians, remove DEG_TO_RAD.
```

## 10.2 Confirm your DRV8825 microstepping

In `config.rs`:

```rust
pub const MICROSTEPS: f32 = 1.0;
```

Only set it to `16.0`, `32.0`, etc. if you wired the DRV8825 `M0/M1/M2` pins accordingly. Otherwise your step frequency conversion will be wrong.

---

# 11. What your final `main.rs` should conceptually contain

After the refactor, `main.rs` should only do this:

```text
1. initialize STM32 clocks
2. initialize motor pins and PWM
3. initialize UART
4. initialize I2C + MPU6050 DMP
5. run two async loops:
   - receiver_loop
   - control_loop
```

It should no longer contain:

```text
- open-loop acceleration ramp
- cruise-control motor loop
- high-rate UART telemetry loop
- AtomicU8 MOTOR_CMD
- LQR math mixed directly into main.rs
```

---

# 12. Build/test order

Do this in sequence:

```text
1. Create the new files.
2. Paste the modified main.rs loop structure.
3. Keep LQR_K = [0, 0, 0, 0] and run cargo check.
4. Run tools/lqr_gain.py on your PC.
5. Paste the generated LQR_K into src/config.rs.
6. Verify pitch units.
7. Lift robot off the ground.
8. Send E.
9. Tilt robot forward manually.
10. Wheels must spin forward.
11. If not, flip THETA_SIGN or MOTOR_SIGN, not both.
12. Only then test on the floor with low MAX_ACCEL_MPS2 and low MAX_SPEED_MPS.
```

---

# 13. Critical warning

With no wheel encoders, your `x` and `x_dot` are only estimates from commanded step frequency. That is acceptable for a first prototype, but it will fail if the steppers skip steps. For a more robust robot, add encoders and replace this part in `LqrController`:

```rust
self.x_m += self.v_mps * DT_S;
```

with measured wheel position/velocity.

For now, keep the position gain low or zero:

```rust
Q[2,2] = 0.0
```

and therefore expect:

```rust
LQR_K[2]
```

to be small or not very important.
