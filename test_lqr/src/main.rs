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

use core::fmt::Write;
use core::sync::atomic::Ordering;

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::{Config as I2cConfig, I2c};
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{Config as UartConfig, Uart};
use embassy_stm32::{bind_interrupts, i2c, peripherals, usart};
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::timer::Channel;
use embassy_time::{Delay, Duration, Ticker, Timer};

use embedded_hal::Pwm;

use mpu6050_dmp::address::Address;
use mpu6050_dmp::quaternion::Quaternion;
use mpu6050_dmp::sensor::Mpu6050;
use mpu6050_dmp::yaw_pitch_roll::YawPitchRoll;

use embassy_futures::join::join;
use embassy_time::{Delay, Duration, Ticker, Timer};

use commands::{
    balance_enabled, disable_balance, handle_bluetooth_byte, velocity_reference_mps,
};
use lqr::LqrController;
use motor::{velocity_to_motor_command, MotorCommand};

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    USART3 => usart::InterruptHandler<peripherals::USART3>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    config.rcc.hsi = true;
    config.rcc.sys = embassy_stm32::rcc::Sysclk::HSI;
    let p = embassy_stm32::init(config);

    info!("Robot Booting: Initializing Peripherals...");

    // 1. Initialize Motor GPIOs
    let mut en_motors = Output::new(p.PA4, Level::High, Speed::Low);
    let mut dir_left = Output::new(p.PA1, Level::Low, Speed::VeryHigh);
    let mut dir_right = Output::new(p.PA3, Level::Low, Speed::VeryHigh);

    // STEP pins moved to Hardware PWM (TIM2)
    // --- FIX 2: Updated to the modern Embassy PwmPin::new API ---
    let ch2 = PwmPin::new(p.PB3, embassy_stm32::gpio::OutputType::PushPull);
    let ch3 = PwmPin::new(p.PA2, embassy_stm32::gpio::OutputType::PushPull);
    
    let mut pwm = SimplePwm::new(
        p.TIM2,
        None,      
        Some(ch2), 
        Some(ch3), 
        None,      
        Hertz(2000), // 2000 Hz = 500us between steps
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
}