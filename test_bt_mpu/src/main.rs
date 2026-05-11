#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use core::fmt::Write; // Required for formatting strings
use embassy_executor::Spawner;
use embassy_stm32::i2c::{Config as I2cConfig, I2c};
use embassy_stm32::usart::{Config as UartConfig, Uart};
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, peripherals, i2c, usart};
use embassy_time::{Delay, Duration, Timer};

// Import the specific modules from mpu6050-dmp
use mpu6050_dmp::sensor::Mpu6050;
use mpu6050_dmp::address::Address;
use mpu6050_dmp::quaternion::Quaternion;
use mpu6050_dmp::yaw_pitch_roll::YawPitchRoll;

// --- Custom String Buffer for UART ---
// This allows us to use standard Rust `write!` formatting without needing the `std` or `alloc` library
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
        self.buf[self.len..self.len+bytes.len()].copy_from_slice(bytes);
        self.len += bytes.len();
        Ok(())
    }
}
// -------------------------------------

// Bind both I2C and UART interrupts
bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    USART3 => usart::InterruptHandler<peripherals::USART3>; // PB10 is typically USART3
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();

    // Enable the High-Speed Internal (HSI) oscillator (runs at 16 MHz)
    config.rcc.hsi = true;
    config.rcc.sys = embassy_stm32::rcc::Sysclk::HSI; 

    let p = embassy_stm32::init(config);
    info!("Booting: Initializing I2C and Bluetooth UART...");

    // 1. Initialize UART (HC-06 Bluetooth)
    // We are using USART3 because PB10 is standard for USART3_TX.
    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 9600; // HC-06 Default Baudrate

    let mut hc06 = Uart::new(
        p.USART3,
        p.PA5,        // RX Pin
        p.PB10,       // TX Pin
        Irqs,
        p.GPDMA1_CH2, // TX DMA (Different from I2C)
        p.GPDMA1_CH3, // RX DMA (Different from I2C)
        uart_config,
    ).expect("Failed to initialize UART");

    let _ = hc06.write(b"--- Robot Telemetry Online ---\r\n").await;

    // 2. Initialize I2C (MPU6050)
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = Hertz(400_000); 
    
    let i2c = I2c::new(
        p.I2C1,
        p.PB6,        // SCL Pin
        p.PB7,        // SDA Pin
        Irqs,
        p.GPDMA1_CH0, // TX DMA
        p.GPDMA1_CH1, // RX DMA
        i2c_config,
    );

    let mut sensor = match Mpu6050::new(i2c, Address::default()) {
        Ok(s) => s,
        Err(_) => {
            error!("Failed to find MPU6050 on I2C bus!");
            let _ = hc06.write(b"ERROR: MPU6050 not found!\r\n").await;
            return;
        }
    };

    let mut delay = Delay;
    info!("Loading DMP firmware onto MPU6050...");
    let _ = hc06.write(b"Loading DMP Firmware...\r\n").await;
    
    match sensor.initialize_dmp(&mut delay) {
        Ok(_) => {
            info!("DMP Firmware Loaded!");
            let _ = hc06.write(b"DMP Ready. Streaming data...\r\n\r\n").await;
        },
        Err(_) => {
            error!("Failed to initialize DMP!");
            return;
        }
    }

    // 3. Main Telemetry Loop
    let mut uart_buf = UartBuf::new();

    loop {
        // Check MPU hardware buffer
        if let Ok(mut len) = sensor.get_fifo_count() {
            
            let mut buf = [0u8; 28];
            let mut got_fresh_data = false;

            // --- THE FIX: DRAIN THE BUFFER ---
            // The sensor generates ~5 packets during our 50ms sleep. 
            // We must read ALL of them, but only keep the last (freshest) one!
            while len >= 28 {
                if sensor.read_fifo(&mut buf).is_ok() {
                    got_fresh_data = true;
                }
                len -= 28; // Subtract the packet we just read
            }
            
            // Only do the heavy math and Bluetooth TX if we successfully grabbed fresh data
            if got_fresh_data {
                // buf now holds the absolutely newest 28 bytes available
                if let Some(quat) = Quaternion::from_bytes(&buf[..16]) {
                    let quat = quat.normalize();
                    let ypr = YawPitchRoll::from(quat);
                    
                    uart_buf.clear();

                    if let Ok(accel) = sensor.accel() {
                        let _ = core::write!(
                            &mut uart_buf,
                            "Y: {:.2}, P: {:.2}, R: {:.2} | Ax: {}, Ay: {}, Az: {}\r\n",
                            ypr.yaw, ypr.pitch, ypr.roll,
                            accel.x(), accel.y(), accel.z()
                        );
                    } else {
                        let _ = core::write!(
                            &mut uart_buf,
                            "Y: {:.2}, P: {:.2}, R: {:.2}\r\n",
                            ypr.yaw, ypr.pitch, ypr.roll
                        );
                    }

                    // Send the string buffer over Bluetooth!
                    let _ = hc06.write(uart_buf.as_slice()).await;
                }
            }
        }
        
        // Wait 50ms (20Hz). We can safely sleep now because we will drain 
        // the accumulated packets instantly when we wake up.
        Timer::after(Duration::from_millis(50)).await;
    }
}