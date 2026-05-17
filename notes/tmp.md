Below is a complete context message you can copy into a new chat, a README, or project notes.

---

# Self-balancing robot project context

I am building a **two-wheeled self-balancing robot** using a **STM32 Nucleo-U545RE-Q** development board. The firmware must be written in **Rust**, using a `#![no_std]`, `#![no_main]` embedded setup with the **Embassy** async runtime.

The hardware I currently have:

```text
MCU board:       STM32 Nucleo-U545RE-Q
IMU:             MPU6050
Bluetooth:       HC-06 UART module
Motor drivers:   2x Pololu DRV8825 stepper drivers
Motors:          2x NEMA17 stepper motors, model 42BYGHW609
Wheels:          80 mm diameter wheels
Microstepping:   DRV8825 configured for 1/16 microstepping
Motor type:      likely 1.8 degree stepper, so 200 full steps/rev
```

I already have basic functionality working:

```text
1. STM32 can communicate with phone through HC-06.
2. STM32 can read MPU6050 data.
3. STM32 can control the two stepper motors through DRV8825 STEP/DIR.
4. Pitch from the MPU6050 DMP prints about 1.57 when tilted 90 degrees, so pitch is already in radians.
5. Motor correction direction has been tested and is correct.
6. The robot now responds faster after increasing the controller aggressiveness, but I want still more wheel acceleration/aggression.
```

The uploaded reference article is **“LQR Control of a Self Balancing Robot”**. Its important idea is that Python is used offline to compute the LQR gain matrix, but the embedded Rust runtime only needs to build the state/error vector and apply the constant gain vector with a dot product. The article’s controller uses state variables around pitch angle, pitch rate, position, and velocity, and it uses weighting matrices `Q` and `R` to decide how strongly to penalize state error versus control effort. 

The article also makes clear that LQR is based on a linearized model around the upright equilibrium, so it is only valid close to vertical. A hard fall cutoff is required in firmware.

---

# Original firmware state

The original firmware was a single `main.rs` containing:

```rust
#![no_std]
#![no_main]
```

It used:

```rust
embassy-stm32
embassy-executor
embassy-time
embassy-futures
defmt
defmt-rtt
panic-probe
mpu6050-dmp
embedded-hal
libm
```

The original program did the following:

```text
1. Initialized motor enable and DIR pins.
2. Used TIM2 PWM channels for STEP pulses.
3. Used USART3 for HC-06.
4. Used I2C1 for MPU6050.
5. Initialized MPU6050 DMP.
6. Ran three async loops with join3:
   - motor_loop
   - receiver_loop
   - telemetry_loop
```

The original motor control was **open-loop**:

```text
Phone sends F/B/S.
Firmware ramps PWM frequency to a fixed speed.
Motors keep running at that speed until command changes.
Pitch is only printed as telemetry.
Pitch is not used for motor control.
```

That is not enough for self-balancing because the motors must respond continuously to pitch error.

---

# Required architecture change

The firmware should be refactored from:

```text
motor_loop      -> open-loop ramp
receiver_loop   -> Bluetooth command parser
telemetry_loop  -> MPU read and UART print
```

to:

```text
receiver_loop   -> Bluetooth command parser only

control_loop    -> fixed-rate balance loop
                -> owns MPU6050
                -> owns PWM and motor pins
                -> reads latest pitch
                -> computes LQR
                -> updates stepper frequency and direction
```

The phone should **not** run the balance algorithm. The STM32 should run the controller locally. The phone should only send high-level commands:

```text
E = enable balance
X = emergency stop / disable balance
F = forward velocity command
B = backward velocity command
S = zero forward velocity, keep balancing
```

For a balancing robot, `S` should not disable the motors. It should mean “stand still / zero velocity.” Emergency stop should be `X`.

---

# Project file structure

The recommended structure is:

```text
project-root/
├── Cargo.toml
├── src/
│   ├── main.rs
│   ├── config.rs
│   ├── lqr.rs
│   ├── motor.rs
│   ├── commands.rs
│   └── telemetry.rs
└── tools/
    └── lqr_gain.py
```

The required files are:

```text
src/config.rs
src/lqr.rs
src/motor.rs
src/commands.rs
tools/lqr_gain.py
```

`src/telemetry.rs` is optional but recommended.

---

# `Cargo.toml`

The current dependency set can remain basically unchanged:

```toml
[package]
name = "Self-Balancing-Robot"
version = "0.1.0"
edition = "2024"

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

No additional Rust crate is strictly required for the first LQR version.

For the offline Python script, install:

```bash
pip install numpy scipy
```

---

# Important physical constants

The robot has **80 mm wheels**, so:

```text
wheel diameter = 80 mm
wheel radius   = 40 mm = 0.040 m
```

The DRV8825 is configured for **1/16 microstepping**.

Assuming the NEMA17 motors are standard 1.8 degree motors:

```text
full steps/rev = 200
microsteps     = 16
effective microsteps/rev = 200 * 16 = 3200
```

Wheel circumference:

```text
circumference = 2 * pi * 0.040 = 0.2513 m
```

Step-rate conversion:

```text
step_hz = velocity_mps * 3200 / 0.2513
```

Approximate values:

```text
0.10 m/s -> 1273 steps/s
0.20 m/s -> 2546 steps/s
0.25 m/s -> 3183 steps/s
0.35 m/s -> 4456 steps/s
0.50 m/s -> 6366 steps/s
0.60 m/s -> 7639 steps/s
```

So a `MAX_STEP_HZ` of `12_000` is enough for current test speeds.

---

# `src/config.rs`

The current recommended config after discovering the wheel size and microstepping is:

```rust
// src/config.rs

pub const CONTROL_PERIOD_MS: u64 = 5;
pub const DT_S: f32 = 0.005;

// TODO: replace with real assembled robot mass.
pub const MASS_KG: f32 = 1.20;

// 80 mm diameter wheel -> 40 mm radius.
pub const WHEEL_RADIUS_M: f32 = 0.040;

// Most NEMA17 motors are 1.8 degree/step.
pub const FULL_STEPS_PER_REV: f32 = 200.0;

// DRV8825 is configured for 1/16 microstepping.
pub const MICROSTEPS: f32 = 16.0;

// These started conservative, then were increased.
pub const MAX_SPEED_MPS: f32 = 0.45;
pub const MAX_ACCEL_MPS2: f32 = 1.5;
pub const MAX_FORCE_N: f32 = MASS_KG * MAX_ACCEL_MPS2;

// Fall cutoff. Can be reduced for early testing.
pub const FALL_LIMIT_RAD: f32 = 0.50;

// Very tiny step rates do not produce useful correction.
// Raising this can make the robot feel more decisive near upright.
pub const MIN_STEP_HZ: u32 = 30;
pub const MAX_STEP_HZ: u32 = 12_000;

// Must be calibrated from real upright pitch.
pub const THETA_ZERO_RAD: f32 = 0.0;

// These were used for sign testing.
// Direction has been tested and is currently correct.
pub const THETA_SIGN: f32 = 1.0;
pub const MOTOR_SIGN: f32 = 1.0;

// Replace with output from tools/lqr_gain.py.
pub const LQR_K: [f32; 4] = [
    0.0,
    0.0,
    0.0,
    0.0,
];
```

Because the user said the robot responds in the correct direction, `THETA_SIGN` and `MOTOR_SIGN` should not be changed unless later hardware wiring changes.

---

# Pitch unit

The MPU6050 DMP pitch was tested:

```text
physical tilt about 90 degrees -> printed pitch around 1.57
```

Therefore, `YawPitchRoll::from(quat).pitch` is already in **radians**.

In `main.rs`, this is correct:

```rust
latest_pitch_rad = Some(ypr.pitch as f32);
```

This is wrong and must not be used:

```rust
const DEG_TO_RAD: f32 = core::f32::consts::PI / 180.0;
latest_pitch_rad = Some((ypr.pitch as f32) * DEG_TO_RAD);
```

The LQR model expects radians, and the reference article also discusses converting measured states into correct units, including radians and meters/second, before applying the LQR gains. 

---

# Upright pitch calibration

`THETA_ZERO_RAD` must be calibrated.

Procedure:

```text
1. Hold the robot physically upright.
2. Print pitch.
3. If upright pitch is 0.03, set THETA_ZERO_RAD = 0.03.
4. If upright pitch is -0.05, set THETA_ZERO_RAD = -0.05.
5. If upright pitch is near 0.0, keep THETA_ZERO_RAD = 0.0.
```

This is important because even a small offset causes the robot to think it is leaning and drive away.

Example:

```rust
pub const THETA_ZERO_RAD: f32 = 0.03;
```

---

# Axle-to-center-of-gravity distance

The LQR model needs a parameter `l`.

`l` means:

```text
distance from the wheel axle centerline to the robot's total center of gravity
when the robot is standing upright
```

It is **not**:

```text
- the wheel radius
- the wheel diameter
- the total robot height
```

It is the effective inverted-pendulum length.

In the article, this parameter is used because gravity creates torque depending on how far the mass center is from the pivot at the wheel axle. 

Typical small self-balancing robots often have:

```text
l ≈ 0.06 m to 0.12 m
```

A reasonable first guess is:

```python
l = 0.080
```

Better measurement method:

```text
1. Assemble the robot completely, including battery and all wiring.
2. Lay the robot sideways on a thin ruler, rod, screwdriver shaft, or edge.
3. Move it until it balances.
4. The balance line passes through the center of gravity.
5. Measure the distance from the wheel axle center to that balance line.
6. Convert mm to meters.
```

Example:

```text
measured axle-to-CG distance = 85 mm
l = 0.085
```

Then in `tools/lqr_gain.py`:

```python
l = 0.085
```

---

# `tools/lqr_gain.py`

This file runs on the PC, not on STM32.

The article’s approach is to compute the LQR gains offline and then use only the generated gain constants in the embedded Rust app. 

Current script:

```python
# tools/lqr_gain.py

import numpy as np
from scipy.signal import cont2discrete
from scipy.linalg import solve_discrete_are

g = 9.81

# TODO: measure real robot mass.
m = 1.20

# TODO: measure axle-to-center-of-gravity distance.
l = 0.080

# Firmware control period.
dt = 0.005  # 200 Hz

# State vector:
#   x = [theta, theta_dot, position, velocity]
#
# theta in radians
# theta_dot in radians/s
# position in meters
# velocity in meters/s

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

Q = np.diag([
    8.0,    # theta
    0.12,   # theta_dot
    0.0,    # position
    1.0,    # velocity
])

R = np.array([[1.0]])

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

To run it:

```bash
python tools/lqr_gain.py
```

Then copy the generated gain vector into `src/config.rs`:

```rust
pub const LQR_K: [f32; 4] = [
    // generated values here
];
```

Every time `Q`, `R`, `m`, `l`, or `dt` changes, regenerate `LQR_K`.

---

# Notes about the article’s matrix

The article prints a Python matrix that includes a row like:

```python
[-g/m, 0, 0, 0]
```

Based on the article’s own derivation of:

```text
x_ddot = F/M - g*theta
```

the corresponding state-space term should be closer to:

```python
[-g, 0, 0, 0]
```

if the state is:

```text
[theta, theta_dot, x, x_dot]
```

So the implementation currently uses:

```python
A = np.array([
    [0.0,   1.0, 0.0, 0.0],
    [g/l,   0.0, 0.0, 0.0],
    [0.0,   0.0, 0.0, 1.0],
    [-g,    0.0, 0.0, 0.0],
])
```

rather than blindly copying the printed matrix.

---

# `src/lqr.rs`

The LQR controller is implemented in Rust as a small no-alloc state machine.

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

        // Without encoders, this is estimated commanded position.
        // Keep position gain small or zero.
        self.x_m += self.v_mps * DT_S;

        Some(MOTOR_SIGN * self.v_mps)
    }
}
```

Current limitation:

```text
There are no wheel encoders.
Therefore x and x_dot are estimated from commanded velocity.
If the steppers skip steps, the controller does not know.
```

For now:

```python
Q[2,2] = 0.0
```

because the position estimate is weak without encoders.

---

# `src/motor.rs`

This converts LQR velocity command into stepper STEP frequency.

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

Currently recommended:

```rust
pub const MIN_STEP_HZ: u32 = 30;
```

Very small step frequencies do not produce useful balancing correction and can make the robot feel lazy near upright.

---

# `src/commands.rs`

This manages Bluetooth command state.

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
            V_REF_MM_S.store(60, Ordering::Relaxed);
        }
        b'B' => {
            BALANCE_ENABLE.store(true, Ordering::Relaxed);
            V_REF_MM_S.store(-60, Ordering::Relaxed);
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

Forward/backward velocity should remain low until the robot balances in place reliably.

---

# `src/telemetry.rs`

Optional helper file:

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

Important warning:

```text
Do not send long UART telemetry lines inside the 200 Hz control loop.
HC-06 at 9600 baud is slow.
Blocking UART formatting/transmission can destabilize the robot.
```

---

# `main.rs` structure

At the top of `main.rs`:

```rust
mod commands;
mod config;
mod lqr;
mod motor;
mod telemetry;
```

Important imports:

```rust
use embassy_futures::join::join;
use embassy_time::{Delay, Duration, Ticker, Timer};

use commands::{
    balance_enabled, disable_balance, handle_bluetooth_byte, velocity_reference_mps,
};
use lqr::LqrController;
use motor::{velocity_to_motor_command, MotorCommand};
```

The old global state should be removed:

```rust
static MOTOR_CMD: AtomicU8 = AtomicU8::new(0);
```

The old open-loop `motor_loop` should be removed.

The old high-rate `telemetry_loop` should be removed or made very slow.

The new control loop should look conceptually like this:

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

                        // Pitch is already radians.
                        latest_pitch_rad = Some(ypr.pitch as f32);
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

The user already tested motor direction and it is correct.

---

# Testing sequence followed so far

Recommended sequence:

```text
1. cargo check
2. set wheel radius = 0.040
3. set microsteps = 16.0
4. remove DEG_TO_RAD conversion
5. measure THETA_ZERO_RAD
6. run tools/lqr_gain.py
7. paste LQR_K into config.rs
8. flash
9. test suspended direction
10. only then floor test
```

The user tested direction and confirmed it is correct.

---

# Current tuning status

The robot initially reacted too slowly.

The first response was to increase:

```rust
pub const MAX_SPEED_MPS: f32 = 0.35;
pub const MAX_ACCEL_MPS2: f32 = 1.0;
```

and tune the Python LQR weights to something like:

```python
Q = np.diag([
    4.0,
    0.08,
    0.0,
    1.0,
])

R = np.array([[1.5]])
```

Then the user wanted more acceleration/aggressiveness.

The next recommended values were:

```rust
pub const MAX_SPEED_MPS: f32 = 0.45;
pub const MAX_ACCEL_MPS2: f32 = 1.5;
pub const MAX_FORCE_N: f32 = MASS_KG * MAX_ACCEL_MPS2;

pub const MIN_STEP_HZ: u32 = 30;
pub const MAX_STEP_HZ: u32 = 12_000;
```

And:

```python
Q = np.diag([
    8.0,
    0.12,
    0.0,
    1.0,
])

R = np.array([[1.0]])
```

If still too soft, the next step is:

```rust
pub const MAX_SPEED_MPS: f32 = 0.60;
pub const MAX_ACCEL_MPS2: f32 = 2.0;
```

and:

```python
Q = np.diag([
    12.0,
    0.18,
    0.0,
    1.2,
])

R = np.array([[0.7]])
```

But this is already aggressive for open-loop steppers and must be tested suspended first.

Avoid starting with:

```python
R = np.array([[0.1]])
```

because with steppers that can cause missed steps, buzzing, violent oscillation, or driver overheating.

---

# How to make the robot more aggressive

There are three control layers.

## 1. Firmware limits

In `src/config.rs`:

```rust
pub const MAX_ACCEL_MPS2: f32 = 1.5;
pub const MAX_SPEED_MPS: f32 = 0.45;
```

Increase gradually:

```rust
pub const MAX_ACCEL_MPS2: f32 = 2.0;
pub const MAX_SPEED_MPS: f32 = 0.55;
```

This matters because LQR output is clamped:

```rust
let force_n = clampf(force_n, -MAX_FORCE_N, MAX_FORCE_N);
```

and:

```rust
pub const MAX_FORCE_N: f32 = MASS_KG * MAX_ACCEL_MPS2;
```

So if `MAX_ACCEL_MPS2` is too low, the controller cannot command hard wheel acceleration even if LQR wants to.

## 2. LQR weights

More aggressive response:

```python
Q[0,0] bigger  -> react more strongly to pitch angle
Q[1,1] bigger  -> more pitch-rate damping
R smaller      -> less penalty on control effort, more aggressive motor output
```

Current aggressive-but-still-reasonable version:

```python
Q = np.diag([
    8.0,
    0.12,
    0.0,
    1.0,
])

R = np.array([[1.0]])
```

More aggressive:

```python
Q = np.diag([
    12.0,
    0.18,
    0.0,
    1.2,
])

R = np.array([[0.7]])
```

Every change requires:

```bash
python tools/lqr_gain.py
```

then paste the new `LQR_K` into Rust.

## 3. Physical driver/motor limits

If the controller asks for more acceleration but the motors do not deliver it, the bottleneck may be:

```text
DRV8825 current limit too low
motor supply voltage too low
battery voltage sag
motor coil wiring issue
driver thermal limiting
microstepping torque reduction
step rate too high for torque available
```

Symptoms:

```text
buzzing
twitching
missed steps
robot tries to correct but cannot accelerate wheels
wheels lag behind pitch correction
motors or drivers get very hot
```

For DRV8825 Pololu-style carriers with 0.100 ohm sense resistors, the approximate relation is:

```text
current_limit = 2 * VREF
```

Examples:

```text
VREF = 0.60 V -> about 1.2 A
VREF = 0.75 V -> about 1.5 A
VREF = 0.90 V -> about 1.8 A
```

But the exact carrier sense resistor must be checked before relying on this.

---

# Current known facts / assumptions

Known:

```text
Pitch is in radians.
Wheel diameter is 80 mm.
Wheel radius should be 0.040 m.
DRV8825 microstepping is 1/16.
Direction test is correct.
STM32 should run LQR locally.
Phone should only send commands.
```

Assumptions to verify:

```text
Motor step angle is 1.8 degrees -> 200 full steps/rev.
Robot mass is around 1.20 kg.
Axle-to-CG distance is around 0.080 m.
DRV8825 current limit is set safely and high enough.
Battery/motor supply can provide enough current.
```

If the motor is 0.9 degrees instead of 1.8 degrees, change:

```rust
pub const FULL_STEPS_PER_REV: f32 = 400.0;
```

instead of:

```rust
pub const FULL_STEPS_PER_REV: f32 = 200.0;
```

Most NEMA17 motors are 1.8 degrees, so `200.0` is the current starting assumption.

---

# No wheel encoder limitation

The robot currently has no encoders.

Therefore:

```text
x and x_dot are estimated from commanded step frequency, not measured.
```

This is acceptable for first prototype testing only if the steppers do not skip steps.

The line:

```rust
self.x_m += self.v_mps * DT_S;
```

is not true measured position. It is estimated commanded position.

Therefore, keep the position weight zero:

```python
Q = np.diag([
    ...,
    0.0,    # position
    ...
])
```

Later, if encoders are added, replace estimated velocity/position with measured wheel velocity/position.

---

# TIM2 shared-frequency limitation

Both STEP pins are currently using different channels of the same STM32 timer:

```text
PB3 -> TIM2 CH2
PA2 -> TIM2 CH3
```

This means both channels share the same timer period/frequency.

That is fine for straight balancing because both wheels need the same speed.

But for steering, left and right wheels need different frequencies. Options later:

```text
1. Use two independent timers.
2. Implement a software step scheduler.
3. Use a different motion architecture.
```

Current LQR version only handles straight balancing and forward/backward velocity.

---

# Safety behavior

The firmware should disable balance if pitch exceeds:

```rust
pub const FALL_LIMIT_RAD: f32 = 0.50;
```

This is about 28.6 degrees.

For first tests, it can be reduced:

```rust
pub const FALL_LIMIT_RAD: f32 = 0.35;
```

That is safer but may disable too easily.

Emergency command:

```text
X
```

should disable balance and set velocity reference to zero.

---

# Recommended next exact experiment

Since the robot is now responding faster but the user wants more wheel acceleration/aggressiveness, the next controlled change should be:

## `src/config.rs`

```rust
pub const MAX_SPEED_MPS: f32 = 0.55;
pub const MAX_ACCEL_MPS2: f32 = 2.0;
pub const MAX_FORCE_N: f32 = MASS_KG * MAX_ACCEL_MPS2;

pub const MIN_STEP_HZ: u32 = 30;
pub const MAX_STEP_HZ: u32 = 12_000;
```

## `tools/lqr_gain.py`

```python
Q = np.diag([
    12.0,
    0.18,
    0.0,
    1.2,
])

R = np.array([[0.7]])
```

Then:

```bash
python tools/lqr_gain.py
```

Paste new gains into:

```rust
pub const LQR_K: [f32; 4] = [
    ...
];
```

Then:

```bash
cargo build
cargo run
```

Test suspended first.

If it becomes violent or oscillatory:

```python
R = np.array([[1.0]])
```

or:

```rust
pub const MAX_ACCEL_MPS2: f32 = 1.5;
```

If it buzzes or misses steps, the problem is likely not only LQR tuning; it is motor torque/current/voltage/stepper dynamics.

---

# Tuning symptom table

```text
Behavior:
  Reacts in correct direction but too slowly

Changes:
  Increase MAX_ACCEL_MPS2
  Increase Q[0,0]
  Decrease R moderately

Behavior:
  Starts correcting but cannot catch itself

Changes:
  Increase MAX_ACCEL_MPS2
  Increase MAX_SPEED_MPS
  Check driver current limit

Behavior:
  Jumps violently

Changes:
  Increase R
  Reduce MAX_ACCEL_MPS2
  Reduce Q[0,0]

Behavior:
  Fast oscillation around upright

Changes:
  Increase Q[1,1] slightly
  Increase R
  Reduce MAX_ACCEL_MPS2

Behavior:
  Drives away while upright

Changes:
  Fix THETA_ZERO_RAD
  Verify pitch axis

Behavior:
  Instant throw/fall

Changes:
  Wrong THETA_SIGN or MOTOR_SIGN
  But current direction test was confirmed correct

Behavior:
  Buzzing or twitching with little movement

Changes:
  DRV8825 current too low
  Supply voltage too low
  Acceleration too high
  Stepper skipping
  Driver overheating

Behavior:
  One wheel fights the other

Changes:
  DIR polarity for one motor is wrong
```

---

# What not to do

Do not run the LQR on the phone.

Do not use Bluetooth telemetry inside the 200 Hz control path.

Do not convert pitch from degrees to radians, because pitch is already radians.

Do not set `MICROSTEPS` to `1.0` if the DRV8825 is actually wired for 1/16.

Do not test on the floor if suspended direction test is wrong.

Do not set `R` extremely low at the beginning.

Do not use the article’s gain values directly.

Do not rely on `x` position control without encoders.

---

# Current high-level conclusion

The STM32 Nucleo-U545RE-Q can absolutely run the LQR algorithm directly in Rust. The algorithm at runtime is very light: a few `f32` operations, a derivative estimate/filter, a dot product, clamping, and step frequency generation. The difficult part is not CPU load; the difficult part is tuning, actuator torque, missed steps, and accurate physical parameters.

The current project is in the correct phase:

```text
1. Hardware communication works.
2. MPU pitch is readable and in radians.
3. Stepper control works.
4. LQR structure is defined.
5. Motor direction is correct.
6. Robot response has improved.
7. Next step is increasing wheel acceleration and checking whether the physical motor/driver system can deliver it.
```

The next engineering priority is to determine whether sluggishness is from:

```text
software clamps / conservative LQR gains
```

or from:

```text
stepper torque / DRV8825 current / battery voltage / missed steps
```

A good next diagnostic is to log or temporarily print the commanded `v_cmd_mps` and `step_hz` at low rate. If `step_hz` jumps high but the wheels do not accelerate hard, the bottleneck is physical. If `step_hz` remains low, the bottleneck is controller tuning or firmware clamps.
