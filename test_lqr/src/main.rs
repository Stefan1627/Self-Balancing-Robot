#![no_std]
#![no_main]

mod commands;
mod config;
mod lqr;
mod motor;
mod telemetry;

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use core::fmt::Write as _;
use core::sync::atomic::{AtomicU8, Ordering}; // <--- ADDED for Buzzer state

use embassy_executor::Spawner;
use embassy_futures::join::join4; // <--- CHANGED to join4
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::{Config as I2cConfig, I2c};
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{Config as UartConfig, Uart};
use embassy_stm32::{bind_interrupts, i2c, peripherals, usart};
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::timer::Channel;

use embedded_hal::Pwm;

use mpu6050_dmp::address::Address;
use mpu6050_dmp::quaternion::Quaternion;
use mpu6050_dmp::sensor::Mpu6050;
use mpu6050_dmp::yaw_pitch_roll::YawPitchRoll;

use embassy_time::{Delay, Duration, Ticker, Timer}; // <--- ADDED Timer for buzzer async delays

use commands::{
    balance_enabled, disable_balance, handle_bluetooth_byte, motor_cmd, velocity_reference_mps,
};
use lqr::LqrController;
use motor::{velocity_to_motor_command, MotorCommand};
use telemetry::{read_telem, update_telem, UartBuf};

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    USART3 => usart::InterruptHandler<peripherals::USART3>;
});

// --- GLOBAL BUZZER COMMAND ---
// 0 = Idle
// 1 = Single Beep (Stop event)
// 2 = Double Beep (Calibrated event)
static BUZZER_CMD: AtomicU8 = AtomicU8::new(0);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    config.rcc.hsi = true;
    config.rcc.sys = embassy_stm32::rcc::Sysclk::HSI;
    let p = embassy_stm32::init(config);

    info!("Robot Booting: Initializing Peripherals...");

    // 1. Initialize Motor GPIOs
    let mut en_motors = Output::new(p.PA4, Level::High, Speed::Low);
    let mut dir_left  = Output::new(p.PA1, Level::Low,  Speed::VeryHigh);
    let mut dir_right = Output::new(p.PA3, Level::Low,  Speed::VeryHigh);

    // STEP pins → Hardware PWM (TIM2)
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

    // 2. Initialize UART (HC-06 Bluetooth)
    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 9600;

    let hc06 = Uart::new(
        p.USART3, p.PA5, p.PB10, Irqs, p.GPDMA1_CH2, p.GPDMA1_CH3, uart_config,
    ).expect("Failed to initialize UART");

    let (mut tx, mut rx) = hc06.split();
    let _ = tx.write(b"--- Robot Online ---\r\n").await;

    // 3. Initialize I2C (MPU6050)
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = Hertz(100_000);

    let i2c = I2c::new(
        p.I2C1, p.PB6, p.PB7, Irqs, p.GPDMA1_CH0, p.GPDMA1_CH1, i2c_config,
    );

    let mut sensor = match Mpu6050::new(i2c, Address::default()) {
        Ok(s) => s,
        Err(_) => {
            error!("Failed to find MPU6050! Check wiring and power.");
            let _ = tx.write(b"ERROR: MPU6050 not found!\r\n").await;
            return;
        }
    };

    let mut delay = Delay;
    let _ = tx.write(b"Loading DMP Firmware...\r\n").await;
    match sensor.initialize_dmp(&mut delay) {
        Ok(_) => {
            info!("DMP Firmware Loaded!");
            let _ = tx.write(b"Ready. Awaiting App Commands...\r\n\r\n").await;
        }
        Err(_) => {
            error!("Failed to initialize DMP!");
            return;
        }
    }

    info!("Initialization complete. Starting concurrent loops...");

    // ==========================================
    // LOOP 1: BLUETOOTH RECEIVER
    // ==========================================
    let receiver_loop = async {
        let mut buf = [0u8; 1];

        loop {
            if rx.read(&mut buf).await.is_ok() {
                handle_bluetooth_byte(buf[0]);
            }
        }
    };

    // ==========================================
    // LOOP 2: LQR CONTROL
    // ==========================================
    let control_loop = async {
        let mut ticker = Ticker::every(Duration::from_millis(config::CONTROL_PERIOD_MS));
        let mut lqr = LqrController::new();

        let mut motor_running = false;
        let mut motor_forward = true;
        let mut current_hz: u32 = 0;

        // --- NEW: State trackers for Buzzer triggers ---
        // --- Buzzer State Trackers ---
        let mut is_calibrated = false;
        let mut has_fallen = false;

        loop {
            ticker.next().await;

            let mut latest_ypr: Option<YawPitchRoll> = None;

            if let Ok(mut len) = sensor.get_fifo_count() {
                let mut fifo_buf = [0u8; 28];

                while len >= 28 {
                    if sensor.read_fifo(&mut fifo_buf).is_ok() {
                        if let Some(quat) = Quaternion::from_bytes(&fifo_buf[..16]) {
                            let quat = quat.normalize();
                            latest_ypr = Some(YawPitchRoll::from(quat));
                        }
                    }
                    len -= 28;
                }
            }

            let v_ref_mps = velocity_reference_mps();

            let mut ax: i16 = 0;
            let mut ay: i16 = 0;
            let mut az: i16 = 0;

            if let Some(ref ypr) = latest_ypr {
                if let Ok(accel) = sensor.accel() {
                    ax = accel.x();
                    ay = accel.y();
                    az = accel.z();
                }

                // --- NEW: Trigger calibration and fall beeps ---
                let theta_rad = ypr.roll as f32;
                
                // 1. Ready to use: R is between -0.05 and 0.05
                if !is_calibrated && theta_rad >= -0.05 && theta_rad <= 0.05 {
                    is_calibrated = true;
                    has_fallen = false; // Reset fall state so it can fall again
                    BUZZER_CMD.store(2, Ordering::Relaxed);
                    info!("Robot calibrated (upright)!");
                }

                // 2. Robot Falls: R is > 0.7 or < -0.7
                if !has_fallen && theta_rad.abs() > 0.70 {
                    has_fallen = true;
                    is_calibrated = false; // Reset calibration so it can recalibrate when picked up
                    BUZZER_CMD.store(3, Ordering::Relaxed);
                    info!("Robot fell!");
                }

                update_telem(
                    ypr.yaw as f32, ypr.pitch as f32, ypr.roll as f32,
                    ax, ay, az,
                    0.0, v_ref_mps, balance_enabled(),
                );
            }

            let motor_command = if balance_enabled() {
                let Some(ypr) = latest_ypr else {
                    continue;
                };

                let theta_rad = ypr.roll as f32;

                match lqr.step(theta_rad, v_ref_mps) {
                    Some(v_cmd_mps) => {
                        update_telem(
                            ypr.yaw as f32, ypr.pitch as f32, ypr.roll as f32,
                            ax, ay, az,
                            v_cmd_mps, v_ref_mps, true,
                        );
                        velocity_to_motor_command(v_cmd_mps)
                    }
                    None => {
                        disable_balance();
                        MotorCommand::DisableDrivers
                    }
                }
            } else {
                lqr.reset();
                let cmd = motor_cmd();
                match cmd {
                    1 => MotorCommand::Run { forward: true, step_hz: 4000 },
                    2 => MotorCommand::Run { forward: false, step_hz: 4000 },
                    _ => MotorCommand::DisableDrivers,
                }
            };

            match motor_command {
                MotorCommand::DisableDrivers => {
                    en_motors.set_high(); 
                    if motor_running {
                        pwm.disable(Channel::Ch2);
                        pwm.disable(Channel::Ch3);
                        motor_running = false;
                    }
                    current_hz = 0;
                }
                MotorCommand::StopPulses => {
                    en_motors.set_low();
                    if motor_running {
                        pwm.disable(Channel::Ch2);
                        pwm.disable(Channel::Ch3);
                        motor_running = false;
                    }
                    current_hz = 0;
                }
                MotorCommand::Run { forward, step_hz: target_hz } => {
                    en_motors.set_low();

                    if motor_running && forward != motor_forward {
                        pwm.disable(Channel::Ch2);
                        pwm.disable(Channel::Ch3);
                        motor_running = false;
                        current_hz = 0;
                    }

                    if forward != motor_forward || !motor_running {
                        motor_forward = forward;

                        let left_fwd  = forward ^ config::INVERT_LEFT_MOTOR;
                        let right_fwd = forward ^ config::INVERT_RIGHT_MOTOR;

                        if left_fwd  { dir_left.set_high();  } else { dir_left.set_low();  }
                        if right_fwd { dir_right.set_low();  } else { dir_right.set_high(); }

                        embassy_time::block_for(Duration::from_micros(2));
                    }

                    let new_hz = if current_hz < target_hz {
                        (current_hz + config::ACCEL_STEP_HZ).min(target_hz)
                    } else {
                        target_hz
                    };

                    if new_hz != current_hz || !motor_running {
                        current_hz = new_hz;
                        pwm.set_frequency(Hertz(current_hz));

                        let max_duty = pwm.get_max_duty();
                        pwm.set_duty(Channel::Ch2, max_duty / 2);
                        pwm.set_duty(Channel::Ch3, max_duty / 2);

                        if !motor_running {
                            pwm.enable(Channel::Ch2);
                            pwm.enable(Channel::Ch3);
                            motor_running = true;
                        }
                    }
                }
            }

            // --- NEW: Trigger stop single-beep ---
            // If it was balancing on the last tick, but isn't anymore
            // (either via App Command 'X' or because it fell over).
            let current_balance = balance_enabled();
            if has_fallen && !current_balance {
                BUZZER_CMD.store(1, Ordering::Relaxed);
                info!("Robot balance stopped!");
            }
            has_fallen = current_balance;
        }
    };

    // ==========================================
    // LOOP 3: TELEMETRY 
    // ==========================================
    let telemetry_loop = async {
        let mut uart_buf = UartBuf::new();
        let mut ticker = Ticker::every(Duration::from_millis(config::TELEMETRY_PERIOD_MS));

        loop {
            ticker.next().await;

            let t = read_telem();

            uart_buf.clear();
            let _ = core::write!(
                &mut uart_buf,
                "Y: {:.2}, P: {:.2}, R: {:.2} | Ax: {}, Ay: {}, Az: {}\r\n",
                t.yaw_rad, t.pitch_rad, t.roll_rad, t.ax, t.ay, t.az
            );

            let _ = tx.write(uart_buf.as_slice()).await;
        }
    };

    // ==========================================
    // LOOP 4: BUZZER (Async so it never blocks LQR)
    // ==========================================
    let buzzer_loop = async {
        // NOTE: If your buzzer beeps when set to Low, swap `set_high` and `set_low`!
        let mut buzzer = Output::new(p.PC6, Level::Low, Speed::Low);

        // Requirement 1: Beep once at Init (150ms)
        buzzer.set_high();
        Timer::after(Duration::from_millis(150)).await;
        buzzer.set_low();

        loop {
            // Read and consume the command
            let cmd = BUZZER_CMD.swap(0, Ordering::Relaxed);
            
            match cmd {
                2 => {
                    // Requirement 2: Ready / Calibration Beep (Two short bips)
                    buzzer.set_high();
                    Timer::after(Duration::from_millis(100)).await;
                    buzzer.set_low();
                    Timer::after(Duration::from_millis(100)).await;
                    buzzer.set_high();
                    Timer::after(Duration::from_millis(100)).await;
                    buzzer.set_low();
                }
                3 => {
                    // Requirement 3: Fall Beep (One long bip - 1 full second)
                    buzzer.set_high();
                    Timer::after(Duration::from_millis(1000)).await;
                    buzzer.set_low();
                }
                _ => {}
            }
            
            // Poll command every 50ms (yielding back to other tasks)
            Timer::after(Duration::from_millis(50)).await;
        }
    };

    // Run all 4 loops concurrently
    join4(control_loop, receiver_loop, telemetry_loop, buzzer_loop).await;
}