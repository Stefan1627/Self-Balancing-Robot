Now you should proceed in this order. Do **not** put the robot on the floor yet.

## 1. First compile the refactor

Run:

```bash
cargo check
```

Fix compile errors before doing anything physical.

Common errors you may see:

```text
unused import
wrong type for Hertz(...)
missing trait import for set_duty/get_max_duty
module not found
```

Make sure `src/main.rs` has:

```rust
mod commands;
mod config;
mod lqr;
mod motor;
mod telemetry;
```

and that these files exist:

```text
src/commands.rs
src/config.rs
src/lqr.rs
src/motor.rs
src/telemetry.rs
```

---

## 2. Put the correct physical constants in `src/config.rs`

For your current hardware, set:

```rust
pub const WHEEL_RADIUS_M: f32 = 0.040;
pub const FULL_STEPS_PER_REV: f32 = 200.0;
pub const MICROSTEPS: f32 = 16.0;
```

Use conservative limits first:

```rust
pub const MAX_SPEED_MPS: f32 = 0.25;
pub const MAX_ACCEL_MPS2: f32 = 0.6;
pub const FALL_LIMIT_RAD: f32 = 0.50;
pub const MAX_STEP_HZ: u32 = 12_000;
```

Also measure your robot and update:

```rust
pub const MASS_KG: f32 = 1.20; // replace with real mass
```

If you do not know the mass yet, keep `1.20` temporarily. It will still compile, but your LQR gains will be less accurate.

---

## 3. Measure the upright pitch offset

Flash a temporary version that only prints pitch while the robot is physically upright.

You already confirmed that pitch is in radians, so **do not use degree conversion**.

In the control loop, this must be:

```rust
latest_pitch_rad = Some(ypr.pitch as f32);
```

not:

```rust
latest_pitch_rad = Some((ypr.pitch as f32) * DEG_TO_RAD);
```

Now hold the robot perfectly upright and read the printed pitch.

If upright pitch is:

```text
0.03
```

then set:

```rust
pub const THETA_ZERO_RAD: f32 = 0.03;
```

If upright pitch is:

```text
-0.05
```

then set:

```rust
pub const THETA_ZERO_RAD: f32 = -0.05;
```

If it is close to zero, keep:

```rust
pub const THETA_ZERO_RAD: f32 = 0.0;
```

This calibration is mandatory. Even a few degrees of offset will make the robot drive away.

---

## 4. Generate the LQR gains

Edit `tools/lqr_gain.py` with your approximate robot values:

```python
m = 1.20      # real total mass in kg
l = 0.080     # axle-to-center-of-gravity distance in meters
dt = 0.005
```

Run:

```bash
python tools/lqr_gain.py
```

Copy the printed gain vector into:

```rust
pub const LQR_K: [f32; 4] = [
    ...
];
```

inside `src/config.rs`.

The article’s approach is exactly this: compute the LQR gain vector offline, then the embedded Rust side only builds the state-error vector and applies the gain vector at runtime. 

---

## 5. Build and flash

Run:

```bash
cargo check
cargo build
cargo run
```

or your normal flashing command.

At this point, keep the robot lifted so the wheels cannot touch the ground.

---

## 6. Test motor direction while suspended

Hold the robot in the air.

Send from your phone:

```text
E
```

That enables balancing.

Now tilt the robot slightly forward.

Expected behavior:

```text
robot tilts forward -> wheels spin forward
robot tilts backward -> wheels spin backward
```

If the wheels react in the wrong direction, change **one** of these:

```rust
pub const THETA_SIGN: f32 = -1.0;
```

or:

```rust
pub const MOTOR_SIGN: f32 = -1.0;
```

Do not flip both at the same time.

Then rebuild and retest.

---

## 7. Test left/right motor polarity

Still suspended, verify both wheels push the robot in the same physical direction.

For forward correction:

```text
left wheel should drive forward
right wheel should drive forward
```

Because the motors are mirrored, your direction pins may need opposite logic. Your current logic is:

```rust
if forward {
    dir_left.set_high();
    dir_right.set_low();
} else {
    dir_left.set_low();
    dir_right.set_high();
}
```

If one wheel spins the wrong way, invert only that wheel’s `DIR` logic.

Example:

```rust
if forward {
    dir_left.set_high();
    dir_right.set_high();
} else {
    dir_left.set_low();
    dir_right.set_low();
}
```

Use whatever combination makes both wheels move the robot in the same physical direction.

---

## 8. Only after sign tests: first floor test

Use these conservative values:

```rust
pub const MAX_SPEED_MPS: f32 = 0.20;
pub const MAX_ACCEL_MPS2: f32 = 0.4;
pub const FALL_LIMIT_RAD: f32 = 0.35;
```

Put the robot on the floor while holding it lightly.

Send:

```text
E
```

Do not send `F` or `B` yet. First test balancing in place.

Expected behavior:

```text
S = zero velocity, keep balancing
X = disable motors / emergency stop
F = forward
B = backward
```

For first tests, use only:

```text
E
S
X
```

---

## 9. Tune in this order

If it falls slowly and does not correct enough:

```text
increase Q theta in tools/lqr_gain.py
```

If it reacts violently:

```text
increase R in tools/lqr_gain.py
```

If it oscillates fast:

```text
increase Q theta_dot slightly
or increase R
or reduce MAX_ACCEL_MPS2
```

If it drives away while upright:

```text
fix THETA_ZERO_RAD
```

If it instantly throws itself:

```text
wrong THETA_SIGN or MOTOR_SIGN
```

If it buzzes but does not move enough:

```text
DRV8825 current limit too low
stepper torque insufficient
acceleration limit too high for available torque
```

---

## 10. After it can stand in place

Only then test velocity commands:

```text
F = small forward command
B = small backward command
S = stop translation, keep balancing
X = emergency stop
```

Start with:

```rust
V_REF_MM_S.store(60, Ordering::Relaxed);
```

instead of:

```rust
V_REF_MM_S.store(120, Ordering::Relaxed);
```

So in `commands.rs`, temporarily use:

```rust
b'F' => {
    BALANCE_ENABLE.store(true, Ordering::Relaxed);
    V_REF_MM_S.store(60, Ordering::Relaxed);
}
b'B' => {
    BALANCE_ENABLE.store(true, Ordering::Relaxed);
    V_REF_MM_S.store(-60, Ordering::Relaxed);
}
```

---

## Immediate checklist

Do these now, in order:

```text
1. cargo check
2. set WHEEL_RADIUS_M = 0.040
3. set MICROSTEPS = 16.0
4. remove DEG_TO_RAD from main.rs
5. measure THETA_ZERO_RAD
6. run tools/lqr_gain.py
7. paste LQR_K into config.rs
8. flash
9. test suspended direction
10. only then test on the floor
```

The most important safety rule: **if the suspended direction test is wrong, do not try a floor test.**
