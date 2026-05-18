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
use core::sync::atomic::{AtomicU8, Ordering};

use embassy_executor::Spawner;
use embassy_futures::join::join4;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::{Config as I2cConfig, I2c};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::timer::Channel;
use embassy_stm32::usart::{Config as UartConfig, Uart};
use embassy_stm32::{bind_interrupts, i2c, peripherals, usart};
use embassy_time::{Delay, Duration, Ticker, Timer};

use embedded_hal::Pwm;

use mpu6050_dmp::address::Address;
use mpu6050_dmp::quaternion::Quaternion;
use mpu6050_dmp::sensor::Mpu6050;
use mpu6050_dmp::yaw_pitch_roll::YawPitchRoll;

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

// Buzzer command.
// 0 = idle
// 1 = single beep, balance stopped
// 2 = double beep, upright/calibrated
// 3 = long beep, fall detected
static BUZZER_CMD: AtomicU8 = AtomicU8::new(0);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut stm32_config = embassy_stm32::Config::default();
    stm32_config.rcc.hsi = true;
    stm32_config.rcc.sys = embassy_stm32::rcc::Sysclk::HSI;
    let p = embassy_stm32::init(stm32_config);

    info!("Robot Booting: Initializing Peripherals...");

    // 1. Motor GPIOs.
    let mut en_motors = Output::new(p.PA4, Level::High, Speed::Low);
    let mut dir_left = Output::new(p.PA1, Level::Low, Speed::VeryHigh);
    let mut dir_right = Output::new(p.PA3, Level::Low, Speed::VeryHigh);

    // STEP pins -> hardware PWM, TIM2.
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

    // 2. UART, HC-06 Bluetooth.
    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 9600;

    let hc06 = Uart::new(
        p.USART3,
        p.PA5,
        p.PB10,
        Irqs,
        p.GPDMA1_CH2,
        p.GPDMA1_CH3,
        uart_config,
    )
    .expect("Failed to initialize UART");

    let (mut tx, mut rx) = hc06.split();
    let _ = tx.write(b"--- Robot Online ---\r\n").await;

    // 3. I2C, MPU6050.
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = Hertz(100_000);

    let i2c = I2c::new(
        p.I2C1,
        p.PB6,
        p.PB7,
        Irqs,
        p.GPDMA1_CH0,
        p.GPDMA1_CH1,
        i2c_config,
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
    // LOOP 1: Bluetooth receiver.
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
    // LOOP 2: LQR control.
    // ==========================================
    let control_loop = async {
        let mut ticker = Ticker::every(Duration::from_millis(config::CONTROL_PERIOD_MS));
        let mut lqr = LqrController::new();

        let mut motor_running = false;
        let mut motor_forward = true;
        let mut current_hz: u32 = 0;

        // Buzzer / state tracking.
        let mut is_calibrated = false;
        let mut fall_latched = false;
        let mut was_balancing = false;

        // Accelerometer is read at reduced rate to avoid adding I2C jitter
        // to the 200 Hz balance loop.
        let mut accel_divider: u8 = 0;
        let mut ax: i16 = 0;
        let mut ay: i16 = 0;
        let mut az: i16 = 0;

        loop {
            ticker.next().await;

            let mut latest_ypr: Option<YawPitchRoll> = None;

            // Read all available DMP packets and keep only the newest one.
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

            if let Some(ref ypr) = latest_ypr {
                // Current balancing axis.
                // If your robot balances using pitch, change this to ypr.pitch.
                let theta_rad = ypr.roll as f32;

                // Read raw accelerometer only every 10 control ticks.
                // At 5 ms/tick, this is every 50 ms = 20 Hz.
                accel_divider = accel_divider.wrapping_add(1);

                if accel_divider >= 10 {
                    accel_divider = 0;

                    if let Ok(accel) = sensor.accel() {
                        ax = accel.x();
                        ay = accel.y();
                        az = accel.z();
                    }
                }

                // Upright/calibrated beep.
                if !is_calibrated && theta_rad >= -0.05 && theta_rad <= 0.05 {
                    is_calibrated = true;
                    fall_latched = false;
                    BUZZER_CMD.store(2, Ordering::Relaxed);
                    info!("Robot calibrated upright.");
                }

                // Fall beep.
                if !fall_latched && theta_rad.abs() > config::FALL_LIMIT_RAD {
                    fall_latched = true;
                    is_calibrated = false;
                    BUZZER_CMD.store(3, Ordering::Relaxed);
                    info!("Robot fell.");
                }

                // Publish telemetry even if balance is disabled.
                update_telem(
                    ypr.yaw as f32,
                    ypr.pitch as f32,
                    ypr.roll as f32,
                    ax,
                    ay,
                    az,
                    0.0,
                    v_ref_mps,
                    balance_enabled(),
                );
            }

            let motor_command = if balance_enabled() {
                let Some(ypr) = latest_ypr else {
                    continue;
                };

                // Current balancing axis.
                // If your robot balances using pitch, change this to ypr.pitch.
                let theta_rad = ypr.roll as f32;

                match lqr.step(theta_rad, v_ref_mps) {
                    Some(v_cmd_mps) => {
                        update_telem(
                            ypr.yaw as f32,
                            ypr.pitch as f32,
                            ypr.roll as f32,
                            ax,
                            ay,
                            az,
                            v_cmd_mps,
                            v_ref_mps,
                            true,
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

                match motor_cmd() {
                    1 => MotorCommand::Run {
                        forward: true,
                        step_hz: 4000,
                    },
                    2 => MotorCommand::Run {
                        forward: false,
                        step_hz: 4000,
                    },
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
                    // Keep drivers enabled for holding torque, but stop step pulses.
                    en_motors.set_low();

                    if motor_running {
                        pwm.disable(Channel::Ch2);
                        pwm.disable(Channel::Ch3);
                        motor_running = false;
                    }

                    current_hz = 0;
                }
                MotorCommand::Run {
                    forward,
                    step_hz: target_hz,
                } => {
                    en_motors.set_low();

                    // If direction changes, stop pulses briefly before changing DIR.
                    if motor_running && forward != motor_forward {
                        pwm.disable(Channel::Ch2);
                        pwm.disable(Channel::Ch3);
                        motor_running = false;
                        current_hz = 0;
                    }

                    if forward != motor_forward || !motor_running {
                        motor_forward = forward;

                        let left_fwd = forward ^ config::INVERT_LEFT_MOTOR;
                        let right_fwd = forward ^ config::INVERT_RIGHT_MOTOR;

                        if left_fwd {
                            dir_left.set_high();
                        } else {
                            dir_left.set_low();
                        }

                        // Kept from your working direction logic.
                        // If the right motor is not mirrored on your chassis,
                        // this may need to be inverted.
                        if right_fwd {
                            dir_right.set_low();
                        } else {
                            dir_right.set_high();
                        }

                        embassy_time::block_for(Duration::from_micros(2));
                    }

                    let new_hz = if current_hz < target_hz {
                        current_hz
                            .saturating_add(config::ACCEL_STEP_HZ)
                            .min(target_hz)
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

            // Stop beep if balance was active and now is not active.
            let current_balance = balance_enabled();

            if was_balancing && !current_balance {
                BUZZER_CMD.store(1, Ordering::Relaxed);
                info!("Robot balance stopped.");
            }

            was_balancing = current_balance;
        }
    };

    // ==========================================
    // LOOP 3: Telemetry.
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
                "Y:{:.2},P:{:.2},R:{:.2}|Ax:{},Ay:{},Az:{}|Vc:{:.2},Vr:{:.2},B:{}\r\n",
                t.yaw_rad,
                t.pitch_rad,
                t.roll_rad,
                t.ax,
                t.ay,
                t.az,
                t.v_cmd_mps,
                t.v_ref_mps,
                t.balanced
            );

            let _ = tx.write(uart_buf.as_slice()).await;
        }
    };

    // ==========================================
    // LOOP 4: Buzzer.
    // ==========================================
    let buzzer_loop = async {
        // If your buzzer is active-low, swap set_high and set_low.
        let mut buzzer = Output::new(p.PC6, Level::Low, Speed::Low);

        // Beep once at init.
        buzzer.set_high();
        Timer::after(Duration::from_millis(150)).await;
        buzzer.set_low();

        loop {
            let cmd = BUZZER_CMD.swap(0, Ordering::Relaxed);

            match cmd {
                1 => {
                    // Stop beep: one short beep.
                    buzzer.set_high();
                    Timer::after(Duration::from_millis(150)).await;
                    buzzer.set_low();
                }
                2 => {
                    // Ready/calibrated beep: two short beeps.
                    buzzer.set_high();
                    Timer::after(Duration::from_millis(100)).await;
                    buzzer.set_low();

                    Timer::after(Duration::from_millis(100)).await;

                    buzzer.set_high();
                    Timer::after(Duration::from_millis(100)).await;
                    buzzer.set_low();
                }
                3 => {
                    // Fall beep: one long beep.
                    buzzer.set_high();
                    Timer::after(Duration::from_millis(1000)).await;
                    buzzer.set_low();
                }
                _ => {}
            }

            Timer::after(Duration::from_millis(50)).await;
        }
    };

    join4(control_loop, receiver_loop, telemetry_loop, buzzer_loop).await;
}