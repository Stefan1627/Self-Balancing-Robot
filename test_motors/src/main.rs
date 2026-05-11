#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::{Duration, Timer};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    // Enable the High-Speed Internal (HSI) oscillator (runs at 16 MHz)
    config.rcc.hsi = true;
    config.rcc.sys = embassy_stm32::rcc::Sysclk::HSI; 
    let p = embassy_stm32::init(config);
    
    info!("Stepper Motor Async Test Booted!");

    // --- SHARED ENABLE PIN ---
    // Start with Enable HIGH (Both motors Disabled)
    let mut en_motors = Output::new(p.PA4, Level::High, Speed::Low); 

    // Configure Left Motor Pins
    let mut step_left = Output::new(p.PA0, Level::Low, Speed::VeryHigh);
    let mut dir_left = Output::new(p.PA1, Level::Low, Speed::VeryHigh);

    // Configure Right Motor Pins
    let mut step_right = Output::new(p.PA2, Level::Low, Speed::VeryHigh);
    let mut dir_right = Output::new(p.PA3, Level::Low, Speed::VeryHigh);

    // 1. Set Direction
    dir_left.set_high();
    dir_right.set_low();

    // 2. Enable BOTH motors simultaneously (Active LOW)
    info!("Enabling DRV8825 drivers...");
    en_motors.set_low();
    
    // Give the drivers a millisecond to power up the coils
    Timer::after(Duration::from_millis(1)).await;

    info!("Spinning motors...");

    // 3. The Async Stepping Loop
    // 3,200 steps = 1 revolution (at 1/16 microstepping)
    // 12,800 steps = 4 full revolutions
    for _ in 0..12_800 { 
        // Pulse High
        step_left.set_high();
        step_right.set_high();
        
        Timer::after(Duration::from_micros(2)).await;
        
        // Pulse Low
        step_left.set_low();
        step_right.set_low();
        
        // Speed Control: Lower delay = faster speed!
        // 500 microseconds = 2000 steps per second.
        // At 3200 steps/rev, this is about 0.6 revolutions per second.
        Timer::after(Duration::from_micros(500)).await;
    }

    // 4. Disable BOTH motors to save power and prevent overheating
    info!("Test Complete. Disabling motors to free-spin.");
    en_motors.set_high();
}