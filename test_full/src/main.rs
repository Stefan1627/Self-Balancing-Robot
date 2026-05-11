#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use core::fmt::Write;
use core::sync::atomic::{AtomicU8, Ordering};

use embassy_executor::Spawner;
use embassy_futures::join::join3;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::{Config as I2cConfig, I2c};
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{Config as UartConfig, Uart};
use embassy_stm32::{bind_interrupts, i2c, peripherals, usart};
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::timer::Channel;
use embassy_time::{Delay, Duration, Timer};

// --- FIX 3: Bring the PWM trait into scope ---
use embedded_hal::Pwm;

use mpu6050_dmp::address::Address;
use mpu6050_dmp::quaternion::Quaternion;
use mpu6050_dmp::sensor::Mpu6050;
use mpu6050_dmp::yaw_pitch_roll::YawPitchRoll;

// --- Global Command State ---
// 0 = Stop, 1 = Forward, 2 = Backward
// --- FIX 1: Fixed the AtomicU8 typo ---
static MOTOR_CMD: AtomicU8 = AtomicU8::new(0);

// --- Custom String Buffer for UART ---
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
// -------------------------------------

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

    // ==========================================
    // LOOP 1: MOTOR CONTROL (With Hardware Ramp)
    // ==========================================
    let motor_loop = async {
        // --- Tuning Parameters ---
        let max_speed: u32 = 4000;    // Target max Hz
        let start_speed: u32 = 10;   // Safe start Hz (No stall)
        let accel_step: u32 = 200;    // How many Hz to add per step
        let step_delay_ms = 10;       // Delay between speed increases

        loop {
            let cmd = MOTOR_CMD.load(Ordering::Relaxed);
            
            if cmd == 0 {
                // STOP: Turn off hardware pulses and drivers
                pwm.disable(Channel::Ch2);
                pwm.disable(Channel::Ch3);
                en_motors.set_high(); 
                
                Timer::after(Duration::from_millis(50)).await;
            } else {
                // MOVE: Enable drivers and set direction
                en_motors.set_low(); 
                
                if cmd == 1 {
                    dir_left.set_high();
                    dir_right.set_low();
                } else if cmd == 2 {
                    dir_left.set_low();
                    dir_right.set_high();
                }

                embassy_time::block_for(Duration::from_micros(1)); // Setup time

                // 1. Reset to the safe starting speed
                let mut current_speed = start_speed;
                pwm.set_frequency(Hertz(current_speed));
                
                // 2. Calculate the 50% duty cycle for this specific starting frequency
                let mut max_duty = pwm.get_max_duty();
                pwm.set_duty(Channel::Ch2, max_duty / 2);
                pwm.set_duty(Channel::Ch3, max_duty / 2);

                // 3. Turn on the hardware pulses!
                pwm.enable(Channel::Ch2);
                pwm.enable(Channel::Ch3);
                
                // 4. THE ACCELERATION RAMP
                // Keep looping until we hit max_speed OR the user lets go of the button
                while current_speed < max_speed && MOTOR_CMD.load(Ordering::Relaxed) == cmd {
                    // Wait a tiny bit before increasing speed
                    Timer::after(Duration::from_millis(step_delay_ms)).await;
                    
                    // Increase the speed, capped at max_speed
                    current_speed += accel_step;
                    if current_speed > max_speed {
                        current_speed = max_speed;
                    }
                    
                    // Apply the new frequency
                    pwm.set_frequency(Hertz(current_speed));
                    
                    // CRITICAL: Recalculate and apply the new 50% duty cycle
                    max_duty = pwm.get_max_duty();
                    pwm.set_duty(Channel::Ch2, max_duty / 2);
                    pwm.set_duty(Channel::Ch3, max_duty / 2);
                }
                
                // 5. CRUISE CONTROL
                // Once we hit max speed, just sleep the loop and let the hardware 
                // run indefinitely until the user releases the button (cmd changes)
                while MOTOR_CMD.load(Ordering::Relaxed) == cmd {
                    Timer::after(Duration::from_millis(50)).await;
                }
            }
        }
    };

    // ==========================================
    // LOOP 2: BLUETOOTH RECEIVER (App -> Robot)
    // ==========================================
    let receiver_loop = async {
        let mut buf = [0u8; 1];
        loop {
            if rx.read(&mut buf).await.is_ok() {
                match buf[0] as char {
                    'F' => MOTOR_CMD.store(1, Ordering::Relaxed),
                    'B' => MOTOR_CMD.store(2, Ordering::Relaxed),
                    'S' => MOTOR_CMD.store(0, Ordering::Relaxed),
                    _ => {} 
                }
            }
        }
    };

    // ==========================================
    // LOOP 3: TELEMETRY (Robot -> App)
    // ==========================================
    let telemetry_loop = async {
        let mut uart_buf = UartBuf::new();
        loop {
            if let Ok(mut len) = sensor.get_fifo_count() {
                let mut buf = [0u8; 28];
                let mut got_fresh_data = false;

                while len >= 28 {
                    if sensor.read_fifo(&mut buf).is_ok() {
                        got_fresh_data = true;
                    }
                    len -= 28;
                }

                if got_fresh_data {
                    if let Some(quat) = Quaternion::from_bytes(&buf[..16]) {
                        let quat = quat.normalize();
                        let ypr = YawPitchRoll::from(quat);
                        uart_buf.clear();

                        if let Ok(accel) = sensor.accel() {
                            let _ = core::write!(
                                &mut uart_buf,
                                "Y: {:.2}, P: {:.2}, R: {:.2} | Ax: {}, Ay: {}, Az: {}\r\n",
                                ypr.yaw, ypr.pitch, ypr.roll, accel.x(), accel.y(), accel.z()
                            );
                        } else {
                            let _ = core::write!(
                                &mut uart_buf,
                                "Y: {:.2}, P: {:.2}, R: {:.2}\r\n",
                                ypr.yaw, ypr.pitch, ypr.roll
                            );
                        }
                        let _ = tx.write(uart_buf.as_slice()).await;
                    }
                }
            }
            Timer::after(Duration::from_millis(50)).await; 
        }
    };

    join3(motor_loop, receiver_loop, telemetry_loop).await;
}