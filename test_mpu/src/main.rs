#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_stm32::i2c::{Config, I2c};
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, peripherals, i2c};
use embassy_time::{Delay, Duration, Timer};

// Import the specific modules from mpu6050-dmp
use mpu6050_dmp::sensor::Mpu6050;
use mpu6050_dmp::address::Address;
use mpu6050_dmp::quaternion::Quaternion;
use mpu6050_dmp::yaw_pitch_roll::YawPitchRoll;

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();

    // Enable the High-Speed Internal (HSI) oscillator (runs at 16 MHz)
    config.rcc.hsi = true;
    // Set the main system clock to use the HSI
    config.rcc.sys = embassy_stm32::rcc::Sysclk::HSI; 

    let p = embassy_stm32::init(config);
    info!("Initializing I2C for MPU6050 DMP...");
    
    let mut i2c_config = Config::default();
    i2c_config.frequency = Hertz(400_000); // 400kHz recommended for fast DMP reading
    
    let i2c = I2c::new(
        p.I2C1,
        p.PB6,
        p.PB7,
        Irqs,
        p.GPDMA1_CH0,
        p.GPDMA1_CH1,
        i2c_config,
    );

    // 1. Initialize the MPU6050 driver using the default I2C address
    let mut sensor = match Mpu6050::new(i2c, Address::default()) {
        Ok(s) => s,
        Err(_) => {
            error!("Failed to find MPU6050 on I2C bus!");
            return;
        }
    };

    // 2. Initialize the DMP (Digital Motion Processor)
    let mut delay = Delay;
    info!("Loading DMP firmware onto MPU6050... this may take a couple of seconds.");
    match sensor.initialize_dmp(&mut delay) {
        Ok(_) => info!("DMP Firmware Loaded Successfully!"),
        Err(_) => {
            error!("Failed to initialize DMP!");
            return;
        }
    }

    // 3. Main Balancing Loop
    loop {
        // Check how much data is in the MPU6050's hardware buffer
        if let Ok(len) = sensor.get_fifo_count() {
            // A full DMP packet is 28 bytes
            if len >= 28 {
                let mut buf = [0u8; 28];
                
                // Read the 28-byte packet from the FIFO
                if let Ok(data) = sensor.read_fifo(&mut buf) {
                    
                    // The first 16 bytes contain the Quaternion data
                    if let Some(quat) = Quaternion::from_bytes(&data[..16]) {
                        let quat = quat.normalize();
                        
                        // Convert the Quaternion into human-readable Euler angles
                        let ypr = YawPitchRoll::from(quat);
                        
                        // Query the sensor directly for the latest accelerometer readings
                        if let Ok(accel) = sensor.accel() {
                            info!(
                                "Yaw: {}, Pitch: {}, Roll: {} | Accel X: {}, Y: {}, Z: {}", 
                                ypr.yaw, ypr.pitch, ypr.roll,
                                accel.x(), accel.y(), accel.z()
                            );
                        } else {
                            // Fallback if accel read fails for one cycle
                            info!("Yaw: {}, Pitch: {}, Roll: {}", ypr.yaw, ypr.pitch, ypr.roll);
                        }
                    }
                }
            }
        }
        // Poll frequently so the 28-byte FIFO buffer doesn't overflow. 
        Timer::after(Duration::from_millis(5)).await;
    }
}